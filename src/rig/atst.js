/* ===== rig/atst.mjs =====
   Scout-walker proportions, built to the canonical silhouette rather than to what is
   known to balance: a tall boxy cockpit carried high on two digitigrade legs set close
   together, on small feet. MK1's knees are already reverse-jointed, so the leg chain is
   unchanged -- only the proportions and the mass distribution differ.

   This is the shape the notes already warned about. The Narrow Track preset (hips 0.42 on
   0.55 m feet) fell in 11 s, and a scout walker is that stance made taller and more
   top-heavy. It is built here to canon and measured; where it fails is the answer. */
const ATST_SPEC = {
  name: 'Scout',
  root: 'pelvis',
  /* 5 deg/step, against the Light Frame's 8. At a uniform 8 this rig delivered 7.99 deg/step --
     it meets the command where the Light Frame only reaches 6.06 -- and it fell at 8.6 s while the
     Light Frame walked 21 s. It is the lightest rig as driven (316 g against 972 g) on the longest
     legs, so the same ring torque turns far more of it per step. deriveGait builds turnRate from
     this same number, so the two cannot disagree. */
  yawPerStepDeg: 5,
  leg: { thigh: 2.40, shin: 2.60 },
  ankle: [-0.15, 0.175, 0],
  links: {
    // hip carriage: the box the legs actually hang from
    pelvis: { mass: 1100, dim: [2.00, 0.85, 2.30] },
    // the cockpit is the machine. 4 500 kg of it, centred 1.75 m above the hip line.
    /* Waist ring, same idiom as MK1 -- and on this silhouette it is the cockpit that
       turns, which is what the shape wants to do anyway. Sized to the same 0.25 m*g*L
       fraction; the cockpit is a much bigger flywheel, so it swings correspondingly
       slower for the same authority. */
    /* COCKPIT AT HALF SCALE. Every linear dimension halved and the mass taken with the
       volume, 4 200 -> 525 kg, so the density is unchanged and this is a smaller cockpit
       rather than a hollow one. jc follows the box: it is the torso's own half-height, so
       -1.15 -> -0.575, or the ring would float inside the hull.
       This is a big change and it is meant to be. The cockpit WAS 51% of an 8 200 kg
       machine, carried 1.75 m above the hip line, and the rig's own note records that the
       earlier over-large cockpit "could not walk at all without the gyro -- that was a
       proportion error on my part". The machine is now 4 333 kg with the pelvis as its
       heaviest single body, which is why the gyro moves to the pelvis in presets.js.
       Mount limits are deliberately NOT reduced with the mass: they are now heavily
       over-strength for what they carry, and overpowering is allowed. */
    torso:  { mass: 525, dim: [1.65, 1.15, 1.50], parent: 'pelvis', type: 'weld',
              axis: [0, 1, 0], angle0: 0, tauMax: 100e3, range: [-50 * D, 50 * D],
              jp: [0, 0.42, 0], jc: [-0.075, -0.575, 0],
              lim: { tension: 4800e3, shear: 4200e3, bend: 3300e3, torsion: 2700e3 } },
    /* Sensor pods / chin cluster, forward and low on the cockpit face. Halved with it --
       jp is a point on the TORSO's face and jc a point on the HEAD's, so both scale with
       their own box or the head floats off the front of a cockpit half its old size. */
    head:   { mass: 27.5, dim: [0.65, 0.375, 0.85], parent: 'torso', type: 'weld',
              jp: [0.775, -0.35, 0], jc: [-0.325, 0, 0],
              lim: { tension: 780e3, shear: 660e3, bend: 330e3, torsion: 210e3 } },
  },
  limbs: [{ side: 'L', s: +1 }, { side: 'R', s: -1 }],
  chain: (s, side) => {
    const S2 = (v) => [v[0], v[1], v[2] * s];
    const D2 = Math.PI / 180;
    return [
      { name: `hipYaw${side}`, parent: 'pelvis', mass: 100, dim: [0.95, 0.55, 0.95],
        type: 'hinge', axis: [0, 1, 0], angle0: 0, jp: S2([0, -0.45, 1.00]), jc: [0, 0, 0],
        tauMax: 220e3, kpTau: 110e3, range: [-45 * D2, 45 * D2],
        kpDeg: 8,   // see rig/mech.js hipYaw: soft by hand, kd now from SERVO_GAMMA
        lim: { tension: 2280e3, shear: 1830e3, bend: 1140e3, torsion: 1200e3 } },
      /* STANCE WIDTH. Hip offset 0.70 -> 1.00 m, which is 0.20*L against the <=0.21*L rule
         in the design table, so it stays inside the one constraint that governs it.
         The narrow track is what killed this rig. Driving log s20260727000055 run 1: fell
         at 15.5 s with peak joint utilisation never above 0.16 -- nothing was overloaded,
         it simply ran out of lateral authority. `dcmZ` sat at -0.25 to -0.39 for the whole
         trace and never once converged. It had 0.062 m of lateral CoP travel against the
         Light Frame's 0.091 m while carrying half its mass in a cockpit at 65% of height.
         Widening the track is the direct treatment; the feet below follow. */
      { name: `hipYoke${side}`, parent: `hipYaw${side}`, mass: 110, dim: [0.88, 0.60, 0.88],
        type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [0, 0, 0],
        tauMax: 260e3, kpTau: 130e3, range: [-35 * D2, 35 * D2],
        lim: { tension: 2280e3, shear: 1830e3, bend: 1140e3, torsion: 1020e3 } },
      /* LEG BEAM SLIMMED 1.10 -> 0.55. Mass is UNCHANGED: this is a section change, not a
         mass change, so no torque table, mount limit, COM or gait timing moves. The old
         section implied a 3.9 mm steel wall on a 1.10 m beam -- slenderness a/t = 280,
         where a flat panel buckles well below 150 and the Light Frame sits at 61. It read
         as fat legs because it WAS fat for its mass. 0.55 lands a/t at 63. */
      { name: `thigh${side}`, parent: `hipYoke${side}`, mass: 400, dim: [0.55, 2.40, 0.55],
        type: 'hinge', axis: [0, 0, 1], angle0: -9 * D2, jp: [0, 0, 0], jc: [0, 1.20, 0],
        tauMax: 360e3, kpTau: 180e3, kpDeg: 7.5, range: [-45 * D2, 110 * D2],
        lim: { tension: 2280e3, shear: 1830e3, bend: 1140e3, torsion: 1410e3 } },
      // Same section change, same reason: a/t 290 -> 63. Mass unchanged.
      { name: `shin${side}`, parent: `thigh${side}`, mass: 280, dim: [0.45, 2.60, 0.45],
        type: 'hinge', axis: [0, 0, 1], angle0: 18 * D2, jp: [0, -1.20, 0], jc: [0, 1.30, 0],
        tauMax: 360e3, kpTau: 180e3, range: [0, 130 * D2],
        lim: { tension: 2070e3, shear: 1620e3, bend: 975e3, torsion: 1410e3 } },
      { name: `ankleYoke${side}`, parent: `shin${side}`, mass: 90, dim: [0.70, 0.50, 0.70],
        type: 'hinge', axis: [0, 0, 1], angle0: -9 * D2, jp: [0, -1.30, 0], jc: [0, 0, 0],
        tauMax: 107e3, kpTau: 80e3, range: [-40 * D2, 30 * D2],
        lim: { tension: 1890e3, shear: 1515e3, bend: 810e3, torsion: 400e3 } },
      /* FOOT WIDTH. 1.30 -> 1.70 m. A canon AT-ST foot is 0.153 of the machine's own
         height where MK1's is 0.224, and that ratio -- not the silhouette, not the mass --
         is what the lateral CoP limit is computed from. 1.70 puts the Scout at 0.201 of
         height and its lateral authority at 0.081 m against Light Frame's 0.091 m.
         Ankle roll authority moves with it, because the two are one rule: the foot joint
         is sized 1.45*W*copLimitZ, so widening the zone without the torque to reach it
         just commands a CoP the machine cannot produce. 55e3 * (0.65*0.85)/(0.45*0.65). */
      { name: `foot${side}`, parent: `ankleYoke${side}`, mass: 360, dim: [2.00, 0.38, 1.70],
        type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [-0.15, 0.175, 0],
        tauMax: 104e3, kpTau: 55e3, range: [-25 * D2, 25 * D2],
        lim: { tension: 1890e3, shear: 1515e3, bend: 810e3, torsion: 408e3 } },
    ];
  },
};
