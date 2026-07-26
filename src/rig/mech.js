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
    torso:     { mass: 1200, dim: [1.35, 0.58, 2.05], parent: 'pelvis', type: 'weld',
                 jp: [0, 0.31, 0], jc: [0, -0.29, 0],
                 lim: { tension: 900e3, shear: 800e3, bend: 600e3, torsion: 500e3 } },
    head:      { mass: 320,  dim: [0.85, 0.50, 1.10], parent: 'torso', type: 'weld',
                 jp: [0, 0.29, 0], jc: [0, -0.25, 0],
                 lim: { tension: 120e3, shear: 100e3, bend: 45e3, torsion: 30e3 } },
  },
  // limbs are generated for both sides; s = +1 left (+Z), -1 right
  limbs: [
    { side: 'L', s: +1 }, { side: 'R', s: -1 },
  ],
};

/* Per-side chain: parent -> child with joint spec. Lateral offsets are multiplied by s. */