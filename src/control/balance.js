/* ===== control/balance.mjs ===== */
// balance.mjs — standing balance.
//
// The controller consumes a StateEstimate and nothing else. It never touches `world`
// or a Body. Today groundTruthState() fills that struct from the simulator; later the
// IMU + encoder estimator fills the identical struct and the controller is unchanged.


let G = 9.81;
function setGravity(g) { G = g; }
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

/* ---- StateEstimate: the only thing the controller is allowed to see ---- */
function groundTruthState(rig, world) {
  let M = 0, c = V(), p = V();
  for (const b of Object.values(rig.bodies)) {
    M += b.mass;
    c = vadd(c, vmul(b.x, b.mass));
    p = vadd(p, vmul(b.v, b.mass));
  }
  c = vmul(c, 1 / M);
  const vcom = vmul(p, 1 / M);

  // foot contact: which feet are loaded, and where the pressure acts
  const feet = {};
  for (const side of ['L', 'R']) {
    const body = rig.bodies[`foot${side}`];
    const F = body.contactForce || 0;
    feet[side] = {
      force: F,
      cop: body.contactCop,
      contact: F > 0.02 * M * G,
      ankle: body.toWorld(rig.ankle),
    };
  }
  const loaded = ['L', 'R'].filter((s) => feet[s].contact);
  let support = null;
  if (loaded.length) {
    let sx = 0, sz = 0;
    for (const s of loaded) { sx += feet[s].ankle.x; sz += feet[s].ankle.z; }
    support = V(sx / loaded.length, 0, sz / loaded.length);
  }

  const torso = rig.bodies.torso;
  const up = qrot(torso.q, V(0, 1, 0));
  return {
    mass: M,
    com: c,
    comVel: vcom,
    comHeight: c.y - (support ? 0 : 0),
    torsoUp: up,
    torsoRate: torso.w,
    // roll about +X (lean toward +Z), pitch about +Z (lean toward +X)
    lean: { pitch: Math.atan2(up.x, up.y), roll: Math.atan2(-up.z, up.y) },
    feet,
    support,
    totalContactForce: feet.L.force + feet.R.force,
  };
}
