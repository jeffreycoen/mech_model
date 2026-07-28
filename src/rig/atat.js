/* ===== rig/atat.mjs =====
   Heavy Walker. Four legs, a slab hull, a head on a neck ring at the front.

   Why a quadruped is a different machine and not a biped with spare legs: the whole
   bipedal controller is built on there being exactly two feet that alternate. The DCM
   planner walks a ZMP from one foot to the other, `plant` has an L and an R, and the
   swing/stance split has one of each. None of that means anything with four feet down.
   This rig is driven by CrawlController (control/crawl.js) -- a static gait that never
   needs to be caught, because it never lets the COM leave the triangle of the three feet
   still on the ground.

   HEIGHT BUDGET, and it is the constraint that set the proportions rather than the
   silhouette. Every rig is rendered at the same target height, so scale SC = target /
   native height, and ui/03-sim.js takes simSteps = round(1/sqrt(SC)) control+physics
   ticks per display frame. The boundary between 2 ticks and 3 sits at SC = 0.16, i.e. a
   native height of 7.8125 m at the 4 ft setting. Worked on the shipped rigs the same
   arithmetic gives MK1 4.91 m -> SC 0.2546 (which ui/03-sim.js already states) and the
   Scout 8.47 m (which presets.js already states), so the method is anchored.

   This machine lands at 7.00 m -> SC 0.1787 -> 2 ticks, 5.4% inside the boundary. A canon
   AT-AT silhouette on a 6.2 m leg comes out near 9.8 m and costs 3, which is a flat 33%
   of the frame budget on a Pi already carrying 25 joints and 12 pair constraints against
   the biped's 16 and 2. The legs are short for the hull on purpose; that is the price of
   the tick. */
const D_Q = Math.PI / 180;

/* Mount envelope. TWO laws, not one.

   Tension, shear and bend carry WEIGHT. rig/mech.js says so in as many words. MK1's table
   shows it -- tension runs 420e3 at the hip down to 350e3 at the foot, a 1.2:1 taper,
   while its tauMax runs 190e3 down to 40e3, a 4.75:1 taper. The Scout is flatter still:
   1890-2280e3 of tension across every leg joint regardless of torque. A helper that scales
   tension with tauMax reproduces the 4.75:1 and hands the FOOT mount a sixth of the
   thigh's weight capacity -- and it is foot mounts, not thigh mounts, that tear in the
   biped's driving logs at util 1.0-1.16.

   So: tension/shear/bend are fractions of THIS rig's weight W and of W*L, with the shape
   carried over from MK1's measured table; torsion alone tracks tauMax.

   ATAT_HK is the height allowance. tauMax/torsion <= 0.73 is a FLOOR on the mount, not a
   target: MK1 sits on it at 0.73, the Scout sits at 0.26 because at a naive scaling its
   hip yoke tore 0.45 s into simply standing up. This machine is 2.2x the Scout's mass, has
   never been driven, and takes the Scout's 3x rather than rediscovering that failure. */
const ATAT_W  = 175.0e3;         // 17 840 kg x 9.81 = 175 010 N, this rig's weight
const ATAT_WL = ATAT_W * 4.10;   // 717.5e3 N.m, W*L -- the bending scale
const ATAT_HK = 3.0;
const ATAT_PROX = { t: 5.12, s: 4.15, b: 0.868 };  // MK1 hipYaw/hipYoke/thigh, per W and W*L
const ATAT_MID  = { t: 4.63, s: 3.66, b: 0.744 };  // MK1 shin
const ATAT_DIST = { t: 4.27, s: 3.41, b: 0.620 };  // MK1 ankleYoke/foot
const atatMount = (tau, c) => ({
  tension: ATAT_HK * c.t * ATAT_W,
  shear:   ATAT_HK * c.s * ATAT_W,
  bend:    ATAT_HK * c.b * ATAT_WL,
  torsion: ATAT_HK * tau / 0.73,
});

/* SINGLE-LEG SIZING, x2 on the dimensionless table, same as MK1 and the Scout.

   This was 1.25 -- a "crawl margin" over the ORIGINAL shared-support table, on the argument
   that a statically crawling quadruped never has fewer than three feet down so the worst
   any one leg sees is W/2. That argument is arithmetically correct and it is beside the
   point: the standing design rule on this project is that EITHER LEG HOLDS THE WHOLE BODY
   AT ANY TIME, the way a person stands on one leg, and it is a requirement rather than a
   derivation to be optimised away by whatever the current gait happens to demand. A
   quadruped that cannot hold itself on one leg is a machine that falls the first time a
   crawl goes wrong -- which is exactly the regime it is in now.

   kpTau does NOT move with it, so the servo loop is untouched and only the saturation angle
   changes, 3 deg -> 6 deg. Mount torsion tracks tauMax through atatMount(), so the 0.73
   invariant holds without a second edit. */
const ATAT_CRAWL_MARGIN = 2.0;

const ATAT_SPEC = {
  name: 'Heavy',
  root: 'pelvis',
  gait: 'quad',
  /* 5 deg/step. At a uniform 8 this rig delivered 9.93 deg/step median -- it OVER-rotates against
     its own command -- and fell at 5.7 s. A quadruped puts three rings on the ground against a
     biped's one, so it has roughly triple the yaw authority per step for the same kSteer; footYaw
     now divides kSteer by the stance count, and this halves what is asked on top of that. */
  yawPerStepDeg: 5,
  leg: { thigh: 1.95, shin: 2.15 },     // L = 4.10 m
  ankle: [-0.15, 0.26, 0],              // ankle pivot in the foot's local frame
  links: {
    // The hull. Everything hangs off it, it is 53% of the mass, and its 1.60 m thickness
    // is a height-budget decision as much as a styling one -- see the header.
    pelvis: { mass: 9400, dim: [6.20, 1.60, 3.60] },
    /* NECK RING. The same joint the biped calls a waist: the right stick aims it, it slews
       at `waistRate`, and the legs only come round once it has used up `waistFollow` of
       its travel. On this silhouette the head is what turns, which is what the shape wants.

       PURE YAW, and that is a hard constraint rather than a style choice. groundTruthState
       reads lean.pitch/lean.roll and torsoRate off `torso`, and checkFall tests the torso's
       up vector. Those are only valid because the ring frees yaw ALONE, so torso attitude
       is identically chassis attitude. Put a pitch axis anywhere between the hull and the
       head and the state estimator and the fall detector both start reading the turret.

       Sized off its OWN inertia, not off m*g*L -- it only ever has to move the head, and a
       fraction of a 17.8 t machine would be absurd here. neck + head about the ring:
         neck 420 x (1.10^2 + 1.10^2)/12                        =  84.7
         head 580 x (1.40^2 + 1.25^2)/12                        = 170.3
         head centre sits 0.95 m off the axis: 580 x 0.95^2     = 523.5
                                                          total = 778.5 kg.m^2
       26 kN.m is 33 rad/s^2, the angular acceleration MK1's 60 kN.m ring puts on its own
       1 800 kg.m^2 assembly. Same brief, smaller flywheel. */
    torso:  { mass: 420, dim: [1.10, 0.90, 1.10], parent: 'pelvis', type: 'weld',
              axis: [0, 1, 0], angle0: 0, tauMax: 26e3, range: [-60 * D_Q, 60 * D_Q],
              jp: [2.70, 0.70, 0], jc: [0, -0.35, 0],
              lim: { tension: 900e3, shear: 760e3, bend: 620e3, torsion: 260e3 } },
    /* Head. Mount scaled off MK1's head at equal margin-to-carried-weight: MK1 carries
       320 kg on 240e3 of tension, 76x its own weight; 580 kg at 76x is 432e3. Bend and
       torsion sit above the straight MK1 ratio because this head is 0.95 m out on the
       neck's lever where MK1's is essentially on-axis. */
    head:   { mass: 580, dim: [1.40, 0.85, 1.25], parent: 'torso', type: 'weld',
              jp: [0.40, 0.20, 0], jc: [-0.55, 0, 0],
              lim: { tension: 430e3, shear: 360e3, bend: 240e3, torsion: 160e3 } },
  },
  /* Four limbs named for where they sit. `fx` is the fore/aft sign, `s` the lateral one.
     The chain below is the SAME six joint names the biped uses, in the same order, so
     legIK, Posture and BalanceController drive it with no quadruped special case. */
  limbs: [
    { side: 'FL', s: +1, fx: +1 }, { side: 'FR', s: -1, fx: +1 },
    { side: 'RL', s: +1, fx: -1 }, { side: 'RR', s: -1, fx: -1 },
  ],
  /* STANCE. Hips at +-2.30 fore/aft (4.60 m base, 1.12 x leg length) and +-1.35 lateral
     (2.70 m track).

     The track is 0.329*L against the design table's "hip offset <= 0.21*L". That rule is
     BIPEDAL by construction and its mechanism is not invoked here. It exists because two
     feet must converge toward the midline to walk, so a wide hip forces a large hipRoll
     splay through legIK and jams hipYoke on its stop. A quadruped puts each foot directly
     under its own hip and never crosses the midline: legIK's hipRoll = -atan2(d.z, -d.y)
     is exactly 0 at the home stance. The one case that does load the roll ring is the
     crawl's lateral body shift, and 0.41 m of sway on a 4.06 m leg is 5.8 deg against a
     +-32 deg range.

     ACTUATOR SIZING, per leg, as fractions of m*g*L = 717.5e3 N.m:
       hipYaw 0.25   hipYoke 0.29   thigh 0.39   shin 0.39
     That is the ORIGINAL shared-support table, deliberately NOT the doubled one the biped
     carries. The doubling exists so either leg can hold the WHOLE body, which is the design
     intent for something standing on two. The quadruped's worst case is exactly half, and
     it is provable rather than estimated: for a rectangular stance, lifting any one foot
     leaves a triangle whose critical edge is the diagonal between the two remaining feet
     opposite the lifted one -- and a rectangle's diagonal passes through its own centre. So
     the machine standing at its geometric centre has ZERO static margin, and at that
     instant the two diagonal feet carry W/2 each while the third carries nothing. Every
     shift away only REDUCES the peak. At the design margin (0.10*L = 0.410 m) the
     barycentric split for a lifted RL is FL 0.455, FR 0.369, RR 0.176 -- peak 0.455 W.

     `tauMax` is that table x ATAT_CRAWL_MARGIN; `kpTau` is the table itself. Splitting them
     is the construction rig/mech.js uses: identical small-error behaviour, saturation angle
     3 deg -> 3.75 deg. Raising a ceiling must never stiffen the loop -- kp x2 and x4 were
     both measured strictly worse at every gait timing. */
  chain: (s, side, limb) => {
    const fx = limb.fx, CM = ATAT_CRAWL_MARGIN, MGL = 717.5e3;
    const tHipYaw  = 0.25 * MGL * CM,  kHipYaw  = 0.25 * MGL;   // 224e3 / 179e3
    const tHipYoke = 0.29 * MGL * CM,  kHipYoke = 0.29 * MGL;   // 260e3 / 208e3
    const tThigh   = 0.39 * MGL * CM,  kThigh   = 0.39 * MGL;   // 350e3 / 280e3
    /* ANKLES are not weight levers, so they take no crawl margin either -- they are sized
       off the CoP authority they must produce, 1.40*W*copLimitX and 1.45*W*copLimitZ, with
       W the PEAK LOAD ON ONE FOOT (W_body/2, from the tripod result above), not the whole
       machine. The biped uses whole-machine W because in single support one foot genuinely
       carries everything; this one never does.
         copLimitX = 0.80 x (0.80 + 0.15) = 0.760 m  ->  1.40 x 87.5e3 x 0.760 = 93.1e3
         copLimitZ = 0.65 x 0.70          = 0.455 m  ->  1.45 x 87.5e3 x 0.455 = 57.7e3
       kpTau is the same rule at the PRE-widening zone (0.60/0.45), exactly as MK1's
       40e3/28e3 are, so the loop keeps the gains it was tuned with while the ceiling covers
       the wider zone.

       Worth knowing before anyone tunes `foot`: BalanceController zeroes foot-roll tauFF
       whenever nSupport > 1, and a crawl is never below three feet down. So `foot`
       contributes NO CoP on this machine, ever -- it is a sole-levelling servo and its
       ceiling is spare. The consequence is not cosmetic: the only lateral CoP authority a
       crawler has is where it puts its body, which is why control/crawl.js spends a whole
       phase per step doing exactly that. */
    /* Ankles take the same rule: sized off the CoP authority they must produce with the
       WHOLE machine on one foot, not half of it -- 1.40*W*copLimitX and 1.45*W*copLimitZ at
       W = 175.0e3 N rather than W/2.
         copLimitX = 0.80 x (0.80 + 0.15) = 0.760  ->  1.40 x 175.0e3 x 0.760 = 186.2e3
         copLimitZ = 0.65 x 0.70          = 0.455  ->  1.45 x 175.0e3 x 0.455 = 115.4e3
       kpTau stays where it was, at the half-weight pre-widening figures, for the same
       reason the leg joints keep theirs. */
    const tAnkle = 186e3, kAnkle = 70e3;
    const tFoot  = 115e3, kFoot  = 40e3;
    return [
      { name: `hipYaw${side}`, parent: 'pelvis', mass: 170, dim: [0.72, 0.50, 0.72],
        type: 'hinge', axis: [0, 1, 0], angle0: 0,
        jp: [fx * 2.30, -0.80, s * 1.35], jc: [0, 0, 0],
        tauMax: tHipYaw, kpTau: kHipYaw, range: [-40 * D_Q, 40 * D_Q],
        /* Soft and lightly damped, as on MK1 and the Scout: a yaw ring reacts a whole leg's
           torsional inertia through its own small one, so full torque at 8 deg rather than
           3, kd = 0.004 kp. The default heuristic saturates this joint from frame zero on
           hull yaw rate alone. */
        kpDeg: 8,   // see rig/mech.js hipYaw: soft by hand, kd now from SERVO_GAMMA
        lim: atatMount(tHipYaw, ATAT_PROX) },
      { name: `hipYoke${side}`, parent: `hipYaw${side}`, mass: 180, dim: [0.66, 0.54, 0.66],
        type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [0, 0, 0],
        tauMax: tHipYoke, kpTau: kHipYoke, range: [-32 * D_Q, 32 * D_Q],
        lim: atatMount(tHipYoke, ATAT_PROX) },
      { name: `thigh${side}`, parent: `hipYoke${side}`, mass: 520, dim: [0.78, 1.95, 0.78],
        type: 'hinge', axis: [0, 0, 1], angle0: -8 * D_Q, jp: [0, 0, 0], jc: [0, 0.975, 0],
        // kpDeg 7.5 -- see rig/mech.js thigh: per-iteration gain above the iteration bound.
        tauMax: tThigh, kpTau: kThigh, kpDeg: 7.5, range: [-40 * D_Q, 85 * D_Q],
        lim: atatMount(tThigh, ATAT_PROX) },
      { name: `shin${side}`, parent: `thigh${side}`, mass: 400, dim: [0.70, 2.15, 0.70],
        type: 'hinge', axis: [0, 0, 1], angle0: 16 * D_Q, jp: [0, -0.975, 0], jc: [0, 1.075, 0],
        tauMax: tThigh, kpTau: kThigh, range: [0, 100 * D_Q],
        lim: atatMount(tThigh, ATAT_MID) },
      { name: `ankleYoke${side}`, parent: `shin${side}`, mass: 110, dim: [0.54, 0.44, 0.54],
        type: 'hinge', axis: [0, 0, 1], angle0: -8 * D_Q, jp: [0, -1.075, 0], jc: [0, 0, 0],
        tauMax: tAnkle, kpTau: kAnkle, range: [-40 * D_Q, 30 * D_Q],
        lim: atatMount(tAnkle, ATAT_DIST) },
      /* -8 + 16 - 8 = 0: the three pitch angle0 values sum to zero, so the sole assembles
         level, which is what legIK's level-sole solution assumes. */
      { name: `foot${side}`, parent: `ankleYoke${side}`, mass: 480, dim: [1.60, 0.40, 1.40],
        type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [-0.15, 0.26, 0],
        tauMax: tFoot, kpTau: kFoot, range: [-25 * D_Q, 25 * D_Q],
        lim: atatMount(tFoot, ATAT_DIST) },
    ];
  },
};
