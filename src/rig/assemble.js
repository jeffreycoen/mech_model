/* ===== rig/assemble.mjs =====
   Spec -> world. Shared by every rig table: MECH_SPEC (rig/mech.js) and ATST_SPEC
   (rig/atst.js) are data, this is the machinery that turns either one into bodies,
   hinges and welds. It lived in atst.js only because the original single-file split was
   cut at byte offsets rather than at logical boundaries. */
/* Resolved servo stiffness for a link. ONE site: both kp and kd below are built from this,
   so a per-link saturation angle cannot desynchronise them. */
function kpOf(L, opts = {}) {
  if (L.kp !== undefined) return L.kp;
  const deg = L.kpDeg ?? 3;
  return ((L.kpTau ?? L.tauMax) / (deg * Math.PI / 180)) * (opts.kpScale ?? 1);
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

/* Inertia of a body about a unit axis through its own COM, in the body's local frame: a^T I a.
   The child's inertia about the hinge axis is what the servo actually has to accelerate, so it is
   the denominator in every stability number for this loop. */
function inertiaAbout(b, a) {
  const M = b.I;
  const Ia = [M[0] * a.x + M[1] * a.y + M[2] * a.z,
              M[3] * a.x + M[4] * a.y + M[5] * a.z,
              M[6] * a.x + M[7] * a.y + M[8] * a.z];
  return Math.abs(a.x * Ia[0] + a.y * Ia[1] + a.z * Ia[2]) || 1e-12;
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
  /* Side keys come from the SPEC, not from a hardcoded ['L','R']. A quadruped is four
     limbs named FL/FR/RL/RR running the identical chain, and every loop that assumed two
     of them named L and R was a reason the AT-AT could not be built. */
  const sides = spec.limbs.map((l) => l.side);
  if (opts.footWidth) for (const s of sides) table[`foot${s}`].dim[2] = opts.footWidth;
  // hipYaw is the link that mounts to the pelvis now, so it carries the lateral offset
  if (opts.hipOffset) for (const s of sides) table[`hipYaw${s}`].jp[2] = Math.sign(table[`hipYaw${s}`].jp[2]) * opts.hipOffset;
  // Mounting height of the hip in the pelvis. -0.31 is the underside of the pelvis box;
  // raising it toward 0 hangs the legs off the LATERAL faces instead of underneath.
  if (opts.hipY !== undefined) for (const s of sides) table[`hipYaw${s}`].jp[1] = opts.hipY;
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
          // Servo gain is tuned to `kpTau`, which is NOT the same as the torque ceiling.
          // Defaulting it to tauMax ties stiffness to authority, so raising a leg's
          // ceiling silently stiffens its loop -- and kp x2 and x4 were both measured
          // strictly worse. Splitting them keeps the small-error behaviour identical and
          // moves only the saturation angle (3 deg at kpTau, 6 deg at a doubled ceiling).
          //
          // `kpDeg` is that saturation angle, per link, default 3. It is a knob because the
          // thigh needs a softer one: its per-iteration gain kp*h^2/I_red measured 1.99 on
          // the Light Frame, above the fixed-point bound of 1, which diverges WITH more
          // iterations rather than converging (one-substep error gain 31x at it=8, 4001x at
          // it=32). hipYaw already used 8 deg by hand on all three rigs, so a per-link angle
          // is the existing practice rather than a new idea.
          kp: kpOf(L, opts),
          // kd from the FINAL kp, never from kpTau. Deriving it from kpTau/3deg while kp
          // came from kpDeg would silently give any softened joint gamma = 36*kpDeg/3 --
          // 15 on the thigh, on the one joint the whole fix depends on.
          kd: L.kd ?? kpOf(L, opts) * (opts.gamma ?? SERVO_GAMMA) * H_NATIVE,
          lim: L.lim,
        });
        /* DISCRETE DAMPER STABILITY. gamma is applied to every joint alike, and on two of them
           that is unstable. The damping term is EXPLICIT -- physics.js freezes `wRel` for all 8
           iterations of a substep -- so it is an explicit-Euler damper on the child's inertia and
           needs kd*h/I < 2. Measured on the shipped Light Frame at 0.30 m, substep 412 us:

             hipYoke   I 1.10e-6  wn 462 Hz  kd*h/I 8.57   <-- 4x over
             ankleYoke I 6.54e-7  wn 453 Hz  kd*h/I 8.26   <-- 4x over
             hipYaw    I 1.65e-6  wn 214 Hz  kd*h/I 1.85   <-- at the edge
             everything else                 kd*h/I 0.03-0.53

           Torque clamps at tauMax, so it does not diverge numerically -- it bang-bangs the
           ceiling. Driving log s20260727231426 (MK1.16.0): satFrac 1.00 on the arms, feet and
           torso, upperArmR demanding 4.64x its ceiling, and every one of the 17 joints reversing
           direction at 9-21 Hz against a 4.09 Hz gait. The arms have NO controller writing their
           targets and still swing 12.6-14.8 deg, which is what rules out the command path.

           This is NOT a small-rig artefact. gamma is Froude-invariant by construction and so is
           wn*h: kp goes as s^4 and I as s^5, so wn goes as s^-0.5 while h goes as s^0.5. The
           product is scale-free, so the same two joints are over the limit at every size -- which
           is why the cap can be computed once, here, at native scale.

           Cap at 1.0 rather than the 2.0 stability bound: 2.0 is the divergence threshold, not a
           safe operating point, and the joints being clipped are 4x past it either way. Joints
           already under the cap keep gamma exactly, so I2 still reads 6.000 for them. */
        const Ia = inertiaAbout(b, ax);
        const kdMax = Ia / H_NATIVE;                   // kd*h/I <= 1.0 at h = H_NATIVE
        if (L.kd === undefined && j.kd > kdMax) { j.kdUncapped = j.kd; j.kd = kdMax; }
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

  /* Feet (and shins) must not pass through each other. Nothing else in the solver
     prevents it, and the walk drifts inward far enough to need it. EVERY pair, not just
     the one L/R pair -- a quadruped has six of each and the diagonal ones are exactly the
     pairs a turning crawl drives together. */
  for (let i = 0; i < sides.length; i++) for (let j = i + 1; j < sides.length; j++) {
    world.addPair(new PairCollision({ a: bodies[`foot${sides[i]}`], b: bodies[`foot${sides[j]}`],
                                      margin: opts.footClearance ?? 0.04 }));
    world.addPair(new PairCollision({ a: bodies[`shin${sides[i]}`], b: bodies[`shin${sides[j]}`],
                                      margin: 0.02 }));
  }

  const legLen = spec.leg || { thigh: 1.50, shin: 1.45 };
  const ank = spec.ankle || [-0.10, 0.15, 0];
  /* `sides` travels WITH the rig. Controllers, telemetry and the state estimator all read
     it instead of assuming two legs. */
  return { bodies, joints, welds, table, spec, sides,
           legs: sides.length, gait: spec.gait || 'biped',
           /* Per-rig turn commitment, degrees of chassis yaw per step. Travels on the rig so
              deriveGait can build BOTH yawPerStep and turnRate from one number -- overriding it
              in a preset's `gait` block instead would set yawPerStep while turnRate stayed
              derived from the default, which is the two-site failure this project keeps hitting
              and which I3 would not have caught. */
           yawPerStepDeg: spec.yawPerStepDeg,
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
