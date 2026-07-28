/* ===== rig/mech.mjs ===== */
// mech.mjs — rig definition and assembly.
//
// Frame convention: +Y up, +X forward (sagittal), +Z left (lateral). Mech faces +X.
// Hinge axes: Z = pitch (fore/aft swing), X = roll (lateral).
//
// Every link is a box: blocky by design, physics first. The visual mesh will later
// hang off these proxies without touching the solver.


const D = Math.PI / 180;

/* Link table. Dimensions are [x, y, z] extents in metres, mass in kg.
   joint: position of the joint in PARENT local frame and in CHILD local frame.
   tauMax in N.m, lim in N / N.m. */
const MECH_SPEC = {
  name: 'MK1',
  root: 'pelvis',
  // Geometry the CONTROLLER needs to know about, kept with the rig rather than hardcoded
  // in the control code, so a differently-proportioned machine is a new table and not a
  // fork of the solver or the gait.
  leg: { thigh: 1.50, shin: 1.45 },   // hip->knee, knee->ankle, metres
  ankle: [-0.10, 0.15, 0],            // ankle pivot in the foot's local frame
  links: {
    pelvis:    { mass: 2800, dim: [0.90, 0.62, 1.95] },
    /* WAIST RING. The torso yaws on the pelvis instead of being welded to it, so the right
       stick aims the upper body immediately rather than waiting for the feet.
       This is the only fast axis on the machine and it is fast for a structural reason:
       it turns ONE body against one actuator, where turning the whole rig means scrubbing
       two planted feet around against friction (measured at ~4 deg per step, whatever it
       was commanded). The legs still come round -- see waistFollow in control/gait.mjs --
       but they do it in their own time, underneath a torso that is already pointing.
       Sized to snap the torso ~90 deg in half a second: the assembly it swings (torso,
       head and both arms) is about 1 800 kg.m^2 about +Y, so 60 kN.m is ~33 rad/s^2.
       That is 0.25 m*g*L, the same fraction the hip yaw ring used to carry. */
    torso:     { mass: 1200, dim: [1.35, 0.58, 2.05], parent: 'pelvis', type: 'weld',
                 axis: [0, 1, 0], angle0: 0, tauMax: 60e3, range: [-50 * D, 50 * D],
                 jp: [0, 0.31, 0], jc: [0, -0.29, 0],
                 lim: { tension: 900e3, shear: 800e3, bend: 600e3, torsion: 500e3 } },
    /* Head mount doubled for the same reason as the arms: it now rides a body that yaws.
       It tore as collateral in two of the arm-off logs (util 1.02 and 1.65), always after
       the arms had already gone. */
    head:      { mass: 320,  dim: [0.85, 0.50, 1.10], parent: 'torso', type: 'weld',
                 jp: [0, 0.29, 0], jc: [0, -0.25, 0],
                 lim: { tension: 240e3, shear: 200e3, bend: 90e3, torsion: 60e3 } },
  },
  // limbs are generated for both sides; s = +1 left (+Z), -1 right
  limbs: [
    { side: 'L', s: +1 }, { side: 'R', s: -1 },
  ],
};

/* Per-side chain: parent -> child with joint spec. Lateral offsets are multiplied by s.

   SINGLE-LEG SIZING. The four proximal leg joints carry 2x what the original table gave
   them (hipYaw 0.25 -> 0.50, hipYoke 0.29 -> 0.58, thigh 0.39 -> 0.78, shin 0.39 -> 0.78,
   as fractions of m*g*L). The old numbers sized the leg for SHARED support; the design
   intent is that either leg holds the whole body at any time, the way a person stands on
   one leg, and one leg carrying the whole body is exactly twice the load. No new lever
   estimate is involved -- it is the same table times two.

   The ankles are not weight levers, so they do not double. They are sized off the CoP
   authority they must produce (1.40*W*copLimitX and 1.45*W*copLimitZ), and the balanced
   zone in rig/derive.js widened from 0.60/0.45 of the foot to 0.80/0.65. Their ceilings
   move by exactly those ratios -- 40e3*(0.80/0.60) and 28e3*(0.65/0.45) -- because a
   wider zone the ankle cannot actually reach is a command the machine will not follow.

   `kpTau` is the torque the SERVO GAIN is tuned to, and it stays at the old ceiling.
   Raising tauMax must NOT stiffen the loop: kp x2 and x4 were both measured strictly
   worse at every gait timing, and stiffness is a different knob from authority. Holding
   kp fixed while tauMax doubles means the saturation angle goes 3 deg -> 6 deg: identical
   behaviour under small error, twice the authority once the leg is genuinely loaded.

   Mount torsion rises with tauMax so the design invariant tauMax/torsion = 0.73 survives
   -- an actuator that tears its own mount at full authority is not authority. Tension,
   shear and bend carry weight rather than actuator torque and are unchanged, and the
   Overdriven preset still detonates (x3 against 0.73 is 2.19). */
function sideChain(s, side) {
  const S = (v) => [v[0], v[1], v[2] * s];
  return [
    /* Arm mounts carry 2x what they used to. They were sized when the torso was WELDED to
       the pelvis and the only thing that ever shook them was the gait. The waist ring
       changed that load case completely: the shoulders now sit 1.025 m off the fastest
       axis on the machine, so they are the outermost mass on a body that slews, and they
       were already the weakest mounts on the rig by a wide margin (bend 95e3 against a
       leg joint's 210e3).
       They are what tore in every arm-off log -- upperArmL at util 1.27, upperArmR 1.04.
       The root cause was the missing waist rate limit and that is fixed in control/gait.mjs;
       this is the load case being sized honestly rather than a patch over it. */
    { name: `upperArm${side}`, parent: 'torso', mass: 350, dim: [0.42, 0.95, 0.42],
      type: 'hinge', axis: [0, 0, 1], angle0: 4 * D, jp: S([0, 0.12, 1.025]), jc: [0, 0.475, 0],
      tauMax: 12e3, range: [-150 * D, 60 * D],
      lim: { tension: 440e3, shear: 360e3, bend: 190e3, torsion: 50e3 } },
    { name: `foreArm${side}`, parent: `upperArm${side}`, mass: 200, dim: [0.36, 0.80, 0.36],
      type: 'hinge', axis: [0, 0, 1], angle0: -8 * D, jp: [0, -0.475, 0], jc: [0, 0.40, 0],
      tauMax: 6e3, range: [-140 * D, 0],
      lim: { tension: 360e3, shear: 300e3, bend: 140e3, torsion: 30e3 } },

    // Hip yaw. The outer ring of a 3-DOF hip gimbal, concentric with the roll ring so the
    // three axes intersect at a single pivot and the leg lengths in legIK are unchanged.
    // Without this axis the feet can only point where the pelvis already points, which is
    // why facing could never be commanded separately from travel, why every turn rate fell
    // over, and why strafing died in three steps.
    // Explicit servo gains. The default heuristic (full torque at 3 deg of error, kd =
    // 0.06 kp) gives this joint kp = 1.15e6 N.m/rad and kd = 69e3 N.m.s/rad, and pelvis
    // yaw rate alone then saturates it at 60 kN.m from frame zero. A yaw ring reacts the
    // torsional inertia of a whole leg through its own small inertia, so it wants a soft,
    // lightly damped servo: full torque at 8 deg, kd = 0.004 kp. Measured against the drive matrix at
    // 10 substeps x 8 iterations, which is the configuration that ships.
    { name: `hipYaw${side}`, parent: 'pelvis', mass: 110, dim: [0.46, 0.30, 0.46],
      type: 'hinge', axis: [0, 1, 0], angle0: 0, jp: S([0, -0.31, 0.60]), jc: [0, 0, 0],
      tauMax: 120e3, kpTau: 60e3, range: [-45 * D, 45 * D],
      /* kpDeg 8, not 3: hand-tuned soft, and the reason is recorded above. The explicit
         kd that used to sit here was gamma = 2.4 applied by hand -- the same damping law
         at a fourth site. It now comes from SERVO_GAMMA like every other joint, which
         raises this ring's kd x2.5 and still leaves damping-alone-rails-the-actuator at
         79x the commanded slew rate. */
      kpDeg: 8,
      lim: { tension: 420e3, shear: 340e3, bend: 210e3, torsion: 220e3 } },
    { name: `hipYoke${side}`, parent: `hipYaw${side}`, mass: 120, dim: [0.38, 0.34, 0.38],
      type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [0, 0, 0],
      tauMax: 140e3, kpTau: 70e3, range: [-35 * D, 35 * D],
      lim: { tension: 420e3, shear: 340e3, bend: 210e3, torsion: 190e3 } },
    { name: `thigh${side}`, parent: `hipYoke${side}`, mass: 450, dim: [0.50, 1.50, 0.50],
      type: 'hinge', axis: [0, 0, 1], angle0: -9 * D, jp: [0, 0, 0], jc: [0, 0.75, 0],
      /* kpDeg 7.5. Per-iteration gain kp*h^2/I_red measured 1.99 here -- above the
         Gauss-Seidel fixed-point bound of 1, so the error DIVERGES with more iterations
         (31x at it=8, 4001x at it=32). Softening the saturation angle is what takes it
         under 1; the torque ceiling does not move. */
      tauMax: 190e3, kpTau: 95e3, kpDeg: 7.5, range: [-45 * D, 110 * D],
      lim: { tension: 420e3, shear: 340e3, bend: 210e3, torsion: 260e3 } },
    { name: `shin${side}`, parent: `thigh${side}`, mass: 300, dim: [0.42, 1.45, 0.42],
      type: 'hinge', axis: [0, 0, 1], angle0: 18 * D, jp: [0, -0.75, 0], jc: [0, 0.725, 0],
      tauMax: 190e3, kpTau: 95e3, range: [0, 130 * D],
      lim: { tension: 380e3, shear: 300e3, bend: 180e3, torsion: 260e3 } },
    { name: `ankleYoke${side}`, parent: `shin${side}`, mass: 90, dim: [0.34, 0.30, 0.34],
      type: 'hinge', axis: [0, 0, 1], angle0: -9 * D, jp: [0, -0.725, 0], jc: [0, 0, 0],
      tauMax: 53e3, kpTau: 40e3, range: [-40 * D, 30 * D],
      lim: { tension: 350e3, shear: 280e3, bend: 150e3, torsion: 73e3 } },
    { name: `foot${side}`, parent: `ankleYoke${side}`, mass: 400, dim: [0.95, 0.30, 1.10],
      type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [-0.10, 0.15, 0],
      tauMax: 40e3, kpTau: 28e3, range: [-25 * D, 25 * D],
      lim: { tension: 350e3, shear: 280e3, bend: 150e3, torsion: 58e3 } },
  ];
}
