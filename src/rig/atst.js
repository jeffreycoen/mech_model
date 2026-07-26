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
  leg: { thigh: 2.40, shin: 2.60 },
  ankle: [-0.15, 0.175, 0],
  links: {
    // hip carriage: the box the legs actually hang from
    pelvis: { mass: 1100, dim: [2.00, 0.85, 2.30] },
    // the cockpit is the machine. 4 500 kg of it, centred 1.75 m above the hip line.
    torso:  { mass: 4200, dim: [3.30, 2.30, 3.00], parent: 'pelvis', type: 'weld',
              jp: [0, 0.42, 0], jc: [-0.15, -1.15, 0],
              lim: { tension: 4800e3, shear: 4200e3, bend: 3300e3, torsion: 2700e3 } },
    // sensor pods / chin cluster, forward and low on the cockpit face
    head:   { mass: 220, dim: [1.30, 0.75, 1.70], parent: 'torso', type: 'weld',
              jp: [1.55, -0.70, 0], jc: [-0.65, 0, 0],
              lim: { tension: 780e3, shear: 660e3, bend: 330e3, torsion: 210e3 } },
  },
  limbs: [{ side: 'L', s: +1 }, { side: 'R', s: -1 }],
  chain: (s, side) => {
    const S2 = (v) => [v[0], v[1], v[2] * s];
    const D2 = Math.PI / 180;
    return [
      { name: `hipYaw${side}`, parent: 'pelvis', mass: 100, dim: [0.95, 0.55, 0.95],
        type: 'hinge', axis: [0, 1, 0], angle0: 0, jp: S2([0, -0.45, 0.70]), jc: [0, 0, 0],
        tauMax: 110e3, range: [-45 * D2, 45 * D2],
        kp: 110e3 / (8 * D2), kd: (110e3 / (8 * D2)) * 0.004,
        lim: { tension: 2280e3, shear: 1830e3, bend: 1140e3, torsion: 600e3 } },
      { name: `hipYoke${side}`, parent: `hipYaw${side}`, mass: 110, dim: [0.88, 0.60, 0.88],
        type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [0, 0, 0],
        tauMax: 130e3, range: [-35 * D2, 35 * D2],
        lim: { tension: 2280e3, shear: 1830e3, bend: 1140e3, torsion: 510e3 } },
      { name: `thigh${side}`, parent: `hipYoke${side}`, mass: 400, dim: [1.10, 2.40, 1.10],
        type: 'hinge', axis: [0, 0, 1], angle0: -9 * D2, jp: [0, 0, 0], jc: [0, 1.20, 0],
        tauMax: 180e3, range: [-45 * D2, 110 * D2],
        lim: { tension: 2280e3, shear: 1830e3, bend: 1140e3, torsion: 705e3 } },
      { name: `shin${side}`, parent: `thigh${side}`, mass: 280, dim: [0.92, 2.60, 0.92],
        type: 'hinge', axis: [0, 0, 1], angle0: 18 * D2, jp: [0, -1.20, 0], jc: [0, 1.30, 0],
        tauMax: 180e3, range: [0, 130 * D2],
        lim: { tension: 2070e3, shear: 1620e3, bend: 975e3, torsion: 705e3 } },
      { name: `ankleYoke${side}`, parent: `shin${side}`, mass: 90, dim: [0.70, 0.50, 0.70],
        type: 'hinge', axis: [0, 0, 1], angle0: -9 * D2, jp: [0, -1.30, 0], jc: [0, 0, 0],
        tauMax: 80e3, range: [-40 * D2, 30 * D2],
        lim: { tension: 1890e3, shear: 1515e3, bend: 810e3, torsion: 300e3 } },
      { name: `foot${side}`, parent: `ankleYoke${side}`, mass: 360, dim: [2.00, 0.38, 1.30],
        type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [-0.15, 0.175, 0],
        tauMax: 55e3, range: [-25 * D2, 25 * D2],
        lim: { tension: 1890e3, shear: 1515e3, bend: 810e3, torsion: 216e3 } },
    ];
  },
};

function sideChain(s, side) {
  const S = (v) => [v[0], v[1], v[2] * s];
  return [
    { name: `upperArm${side}`, parent: 'torso', mass: 350, dim: [0.42, 0.95, 0.42],
      type: 'hinge', axis: [0, 0, 1], angle0: 4 * D, jp: S([0, 0.12, 1.025]), jc: [0, 0.475, 0],
      tauMax: 12e3, range: [-150 * D, 60 * D],
      lim: { tension: 220e3, shear: 180e3, bend: 95e3, torsion: 25e3 } },
    { name: `foreArm${side}`, parent: `upperArm${side}`, mass: 200, dim: [0.36, 0.80, 0.36],
      type: 'hinge', axis: [0, 0, 1], angle0: -8 * D, jp: [0, -0.475, 0], jc: [0, 0.40, 0],
      tauMax: 6e3, range: [-140 * D, 0],
      lim: { tension: 180e3, shear: 150e3, bend: 70e3, torsion: 15e3 } },

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
      tauMax: 60e3, range: [-45 * D, 45 * D],
      kp: 60e3 / (8 * D), kd: (60e3 / (8 * D)) * 0.004,
      lim: { tension: 420e3, shear: 340e3, bend: 210e3, torsion: 110e3 } },
    { name: `hipYoke${side}`, parent: `hipYaw${side}`, mass: 120, dim: [0.38, 0.34, 0.38],
      type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [0, 0, 0],
      tauMax: 70e3, range: [-35 * D, 35 * D],
      lim: { tension: 420e3, shear: 340e3, bend: 210e3, torsion: 95e3 } },
    { name: `thigh${side}`, parent: `hipYoke${side}`, mass: 450, dim: [0.50, 1.50, 0.50],
      type: 'hinge', axis: [0, 0, 1], angle0: -9 * D, jp: [0, 0, 0], jc: [0, 0.75, 0],
      tauMax: 95e3, range: [-45 * D, 110 * D],
      lim: { tension: 420e3, shear: 340e3, bend: 210e3, torsion: 130e3 } },
    { name: `shin${side}`, parent: `thigh${side}`, mass: 300, dim: [0.42, 1.45, 0.42],
      type: 'hinge', axis: [0, 0, 1], angle0: 18 * D, jp: [0, -0.75, 0], jc: [0, 0.725, 0],
      tauMax: 95e3, range: [0, 130 * D],
      lim: { tension: 380e3, shear: 300e3, bend: 180e3, torsion: 130e3 } },
    { name: `ankleYoke${side}`, parent: `shin${side}`, mass: 90, dim: [0.34, 0.30, 0.34],
      type: 'hinge', axis: [0, 0, 1], angle0: -9 * D, jp: [0, -0.725, 0], jc: [0, 0, 0],
      tauMax: 40e3, range: [-40 * D, 30 * D],
      lim: { tension: 350e3, shear: 280e3, bend: 150e3, torsion: 55e3 } },
    { name: `foot${side}`, parent: `ankleYoke${side}`, mass: 400, dim: [0.95, 0.30, 1.10],
      type: 'hinge', axis: [1, 0, 0], angle0: 0, jp: [0, 0, 0], jc: [-0.10, 0.15, 0],
      tauMax: 28e3, range: [-25 * D, 25 * D],
      lim: { tension: 350e3, shear: 280e3, bend: 150e3, torsion: 40e3 } },
  ];
}

function buildLinkTable(spec = MECH_SPEC) {
  const table = {};
  // dim is copied, not shared: footWidth overrides used to mutate the spec itself, so one
  // preset could quietly resize the rig for every preset built afterwards.
  for (const [name, L] of Object.entries(spec.links))
    table[name] = { name, ...L, dim: [...L.dim], jp: L.jp ? [...L.jp] : L.jp, jc: L.jc ? [...L.jc] : L.jc };
  const chain = spec.chain || sideChain;
  for (const limb of spec.limbs)
    for (const L of chain(limb.s, limb.side, limb)) table[L.name] = L;
  return table;
}

/* pick a unit vector perpendicular to `axis`, used as the hinge angle reference */
function perpTo(axis) {
  const a = vnorm(axis);
  const t = Math.abs(a.x) < 0.9 ? V(1, 0, 0) : V(0, 1, 0);
  return vnorm(vcross(a, t));
}

/* Assemble the rig into `world`. rootPos places the pelvis COM.
   Returns { bodies, joints, welds, byName } with joints keyed by child link name. */
function assembleMech(world, opts = {}) {
  const spec = opts.spec || MECH_SPEC;
  const table = buildLinkTable(spec);
  // geometry overrides for design sweeps
  if (opts.footWidth) for (const s of ['L', 'R']) table[`foot${s}`].dim[2] = opts.footWidth;
  // hipYaw is the link that mounts to the pelvis now, so it carries the lateral offset
  if (opts.hipOffset) for (const s of ['L', 'R']) table[`hipYaw${s}`].jp[2] = Math.sign(table[`hipYaw${s}`].jp[2]) * opts.hipOffset;
  // Mounting height of the hip in the pelvis. -0.31 is the underside of the pelvis box;
  // raising it toward 0 hangs the legs off the LATERAL faces instead of underneath.
  if (opts.hipY !== undefined) for (const s of ['L', 'R']) table[`hipYaw${s}`].jp[1] = opts.hipY;
  const bodies = {}, joints = {}, welds = {};

  const mk = (L, pos, quat) => {
    const [dx, dy, dz] = L.dim;
    const b = new Body({
      name: L.name, mass: L.mass, inertia: boxInertia(L.mass, dx, dy, dz),
      pos, quat,
    });
    b.half = V(dx / 2, dy / 2, dz / 2);
    b.dim = V(dx, dy, dz);
    return world.add(b);
  };

  const root = table[spec.root];
  bodies[root.name] = mk(root, opts.rootPos || V(0, 3.55, 0), Q());

  // place children breadth-first so every parent exists before its child
  const pending = Object.values(table).filter((L) => L.parent);
  let guard = 0;
  while (pending.length && guard++ < 1000) {
    for (let i = 0; i < pending.length; i++) {
      const L = pending[i];
      const P = bodies[L.parent];
      if (!P) continue;
      const jp = V(...L.jp), jc = V(...L.jc);
      const jointWorld = P.toWorld(jp);
      let q;
      if (L.type === 'hinge') q = qmul(P.q, qAxisAngle(V(...L.axis), L.angle0 || 0));
      else q = P.q;
      const pos = vsub(jointWorld, qrot(q, jc));
      const b = mk(L, pos, q);
      bodies[L.name] = b;

      if (L.type === 'hinge') {
        const ax = vnorm(V(...L.axis));
        const ref = perpTo(ax);
        const j = new Hinge({
          name: L.name, a: P, b, ra: jp, rb: jc,
          axisA: ax, axisB: ax, refA: ref, refB: ref,
          tauMax: L.tauMax, target: L.angle0 || 0, limits: L.range,
          kp: L.kp ?? (L.tauMax / (3 * Math.PI / 180)) * (opts.kpScale ?? 1),
          kd: L.kd ?? (L.tauMax / (3 * Math.PI / 180)) * (opts.kdScale ?? 0.06),
          lim: L.lim,
        });
        world.addJoint(j);
        joints[L.name] = j;
      } else {
        const w = new Weld({
          name: L.name, a: P, b, ra: jp, rb: jc,
          axis: V(0, 1, 0), lim: L.lim,
        });
        world.addWeld(w);
        welds[L.name] = w;
      }
      pending.splice(i--, 1);
    }
  }
  if (pending.length) throw new Error('unresolved links: ' + pending.map((l) => l.name).join(','));

  // Feet (and shins) must not pass through each other. Nothing else in the solver
  // prevents it, and the walk drifts inward far enough to need it.
  world.addPair(new PairCollision({ a: bodies.footL, b: bodies.footR, margin: opts.footClearance ?? 0.04 }));
  world.addPair(new PairCollision({ a: bodies.shinL, b: bodies.shinR, margin: 0.02 }));

  const legLen = spec.leg || { thigh: 1.50, shin: 1.45 };
  const ank = spec.ankle || [-0.10, 0.15, 0];
  return { bodies, joints, welds, table, spec,
           leg: legLen, ankle: V(ank[0], ank[1], ank[2]) };
}

/* Drop the rig so the lowest point of either foot sits exactly on y = 0. */
function groundRig(rig) {
  let lowest = Infinity;
  for (const b of Object.values(rig.bodies)) {
    if (!b.half) continue;
    for (let i = 0; i < 8; i++) {
      const p = b.toWorld(V(i & 1 ? b.half.x : -b.half.x, i & 2 ? b.half.y : -b.half.y, i & 4 ? b.half.z : -b.half.z));
      if (p.y < lowest) lowest = p.y;
    }
  }
  for (const b of Object.values(rig.bodies)) b.x = vadd(b.x, V(0, -lowest, 0));
  return -lowest;
}

/* Shift the rig horizontally so its COM starts directly over the mean ankle position.
   Assembling with a crouch leaves the COM ahead of the ankles, which is a large initial
   disturbance for the balance controller to absorb before it has done anything wrong. */
/* Horizontal offset from the COM to the mean ankle pivot in the assembled stance.
   The crouch is chosen so this is already small; it is reported, not corrected, because
   correcting it by translation is a no-op (the ankles move with the rig) and correcting
   it by joint trim is the balance controller's job. */
function comAnkleOffset(rig) {
  let M = 0, c = V();
  for (const b of Object.values(rig.bodies)) { M += b.mass; c = vadd(c, vmul(b.x, b.mass)); }
  c = vmul(c, 1 / M);
  let ax = 0, az = 0;
  for (const s of ['L', 'R']) {
    const p = rig.bodies[`foot${s}`].toWorld(rig.ankle);
    ax += p.x / 2; az += p.z / 2;
  }
  return V(c.x - ax, 0, c.z - az);
}

/* Total mass, COM, and standing height of the assembled rig. */
function rigStats(rig) {
  let M = 0, c = V(), top = -Infinity, bottom = Infinity;
  for (const b of Object.values(rig.bodies)) {
    M += b.mass; c = vadd(c, vmul(b.x, b.mass));
    for (let i = 0; i < 8; i++) {
      const p = b.toWorld(V(i & 1 ? b.half.x : -b.half.x, i & 2 ? b.half.y : -b.half.y, i & 4 ? b.half.z : -b.half.z));
      if (p.y > top) top = p.y;
      if (p.y < bottom) bottom = p.y;
    }
  }
  return { mass: M, com: vmul(c, 1 / M), height: top - bottom, top, bottom };
}
