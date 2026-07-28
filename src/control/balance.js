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
  /* Over the rig's OWN side list. Two-leg assumptions here were one of the things that
     made a quadruped impossible: `loaded`, the support centre and the total contact force
     are all already n-ary, they were just being fed exactly two entries. */
  const feet = {};
  for (const side of rig.sides) {
    const body = rig.bodies[`foot${side}`];
    const F = body.contactForce || 0;
    feet[side] = {
      force: F,
      cop: body.contactCop,
      contact: F > 0.02 * M * G,
      ankle: body.toWorld(rig.ankle),
      /* Substep truth alongside the frame mean. `force` says the foot is carrying its
         share; forceMin says whether it was carrying it the WHOLE frame or dropping to
         zero and back inside it. cone is how much of the available friction is already
         spent (1 = sliding), slip is the ground speed of the sole in m/s. Nothing reads
         these yet -- they exist so the next drive can be read, and so a controller CAN. */
      forceMin: body.contactForceMin || 0,
      forceMax: body.contactForceMax || 0,
      cone: body.contactCone || 0,
      slip: body.contactSlip || 0,
    };
  }
  const loaded = rig.sides.filter((s) => feet[s].contact);
  let support = null;
  if (loaded.length) {
    let sx = 0, sz = 0, sy = 0;
    for (const s of loaded) { sx += feet[s].ankle.x; sy += feet[s].ankle.y; sz += feet[s].ankle.z; }
    /* The y was hardcoded to 0 here, which made comHeight below a no-op subtraction and
       any "height above the ground" question unanswerable off flat terrain. No consumer
       reads support.y today, so filling it in changes nothing and fixes the sensor. */
    support = V(sx / loaded.length, sy / loaded.length, sz / loaded.length);
  }
  /* Actuator state. `saturated` has been computed every substep since the solver was
     written, shown in the UI bars, and read by no controller -- so the machine has never
     been able to tell that it is railed, which is the exact condition behind the standing
     slide. satFrac is the fraction of substeps against the ceiling: working hard reads
     low, bang-banging reads near 1. */
  let nSat = 0, nJoint = 0, peakUtil = 0, peakSat = 0, peakSatName = null;
  for (const j of Object.values(rig.joints)) {
    if (j.broken) continue;
    nJoint++;
    const sf = j.satFrac || 0;
    if (sf > 0.5) nSat++;
    if (sf > peakSat) { peakSat = sf; peakSatName = j.name; }
    if (j.util > peakUtil) peakUtil = j.util;
  }

  const torso = rig.bodies.torso;
  const up = qrot(torso.q, V(0, 1, 0));
  /* CHASSIS yaw, reported separately from the torso's. With a waist ring the torso is a
     turret: it yaws fast, on purpose, and it is NOT where the machine is heading. Anything
     asking "which way is this thing pointed" or "how fast is it rotating" means the
     pelvis. Feeding the stabiliser torso yaw instead would have it braking the turret --
     the waist actuator is 60 kN.m and the gyro's yaw damping alone is 21 kN.m per rad/s,
     so a fast slew would have been fighting a third of its own authority.
     Roll and pitch still come from the torso: the waist frees YAW only, so the two bodies
     share those axes rigidly and the torso is where the attitude sensor would sit. */
  const pf = qrot(rig.bodies.pelvis.q, V(1, 0, 0));
  return {
    mass: M,
    com: c,
    comVel: vcom,
    // COM height above the SUPPORT PLANE, which is what an inverted-pendulum omega wants.
    // Identical to com.y on flat ground at y=0; buildPlan still uses com.y, deliberately --
    // switching the pendulum height changes omega, and that is a control change, not a
    // sensor fix. Logged next to it so the next drive shows whether they diverge.
    /* NULL when there is no support polygon, NOT a silent fallback to raw com.y. The
       fallback made this quantity change definition mid-log: the moment contact was lost it
       jumped by support.y (~20-35 mm) and back, which reads as the machine hopping. It is
       the difference between "COM is 24 mm higher" and "I stopped subtracting something".
       A sensor that returns a different quantity under a different condition is worse than
       one that returns nothing. */
    comHeight: support ? c.y - support.y : null,
    comHeightRaw: c.y,
    torsoUp: up,
    torsoRate: torso.w,
    pelvisYaw: Math.atan2(-pf.z, pf.x),
    pelvisRate: rig.bodies.pelvis.w,
    // roll about +X (lean toward +Z), pitch about +Z (lean toward +X)
    lean: { pitch: Math.atan2(up.x, up.y), roll: Math.atan2(-up.z, up.y) },
    feet,
    support,
    sides: rig.sides,
    nSupport: loaded.length,
    totalContactForce: rig.sides.reduce((a, s) => a + feet[s].force, 0),
    joints: { nSat, nJoint, peakUtil, peakSat, peakSatName },
  };
}
class BalanceController {
  constructor(rig, cfg = {}) {
    this.rig = rig;
    this.k = Object.assign({
      ankleKp: 0.03, ankleKd: 0.011,     // capture-point error (m) -> ankle angle (rad)
      ankleTrim: 0.16,                   // max ankle trim, rad
      signPitch: 1, signRoll: 1, signHip: 1,
      kCop: 0.6,             // CoP error (m) -> ankle torque, as a fraction of F
      /* Ceiling on the F in that product, in body weights. See the note at the
         feedforward: it exists to stop a contact spike multiplying into the ankle. */
      forceCap: 1.5,
      copLimitX: 0.36, copLimitZ: 0.24,  // CoP travel from the ankle pivot, m (foot geometry)
      hipStrategy: 0.0,                  // rad of hip trim per m of capture-point excess
      lateralShift: 0.0,                 // rad of hip roll per m of lateral capture error
      capture: 1.6,
      torsoKp: 26e3, torsoKd: 7.0e3,     // torso attitude, N.m / rad
      hipKp: 0.08, hipKd: 0.024,         // hip angle servo trim (rad per rad of lean)
      hipTrimLimit: 0.35,
      kneeTarget: 18 * Math.PI / 180,
      comHeightTarget: null,
      kneeKp: 1.4, kneeKd: 0.10,
    }, cfg);
    this.stance = { hip: -9 * Math.PI / 180, knee: 18 * Math.PI / 180, ankle: -9 * Math.PI / 180 };
    // All joints stay in position mode. Balance is applied as small angle trims on top
    // of a stance that is known to hold statically; commanding ankle TORQUE directly
    // makes the ankle a free pivot on any frame where contact force reads low, and the
    // legs fold before the loop can engage.
    this.debug = {};
  }

  update(st, dt) {
    const J = this.rig.joints;
    if (!st.support) {                       // airborne: hold the stance pose
      return;
    }

    // --- capture point (LIPM): where the COM will come to rest if we do nothing ---
    // SCALE FIX: absolute 0.5 m floor; bound at 2 ft and below.
    const hCom = Math.max(1e-4, st.com.y);
    const w0 = Math.sqrt(G / hCom);
    const xiX = st.com.x + st.comVel.x / w0;
    const xiZ = st.com.z + st.comVel.z / w0;

    const eX = (xiX - st.support.x), eZ = (xiZ - st.support.z);

    // --- desired centre of pressure ---------------------------------------------------
    // Default: drive the capture point back to the support centre. When a gait planner
    // supplies copOverride, track that instead -- it already encodes a dynamically
    // feasible ZMP plus DCM error feedback.
    const copDesX = this.copOverride ? this.copOverride.x : xiX + this.k.capture * eX;
    const copDesZ = this.copOverride ? this.copOverride.z : xiZ + this.k.capture * eZ;

    const sides = st.sides || ['L', 'R'];
    const nSupport = st.nSupport !== undefined ? st.nSupport
                   : sides.filter((s) => st.feet[s].contact).length;
    // --- ankles: hold the stance pose on PD, bias the torque to move the MEASURED CoP.
    // Closing on measured CoP rather than commanding open-loop torque means the loop is
    // robust to whatever the servo happens to be doing.
    for (const s of sides) {
      const f = st.feet[s];
      const aJ = J[`ankleYoke${s}`], rJ = J[`foot${s}`];
      aJ.target = this.stance.ankle;
      rJ.target = 0;
      if (!f.contact || !f.cop) { aJ.tauFF = 0; rJ.tauFF = 0; continue; }
      /* FRAME FIX: the ankle's pitch and roll axes turn with the body, so the CoP error
         must be projected into the FOOT frame before it becomes joint torque. Applied in
         world axes this loop is exact at facing 0 and reversed at facing 135 -- which is
         why every walk after a turn fell over while cold-start walking was fine. */
      const ch = Math.cos(this.facing || 0), sh = Math.sin(this.facing || 0);
      const rdX = copDesX - f.ankle.x, rdZ = copDesZ - f.ankle.z;
      const dDesF = clamp(rdX * ch - rdZ * sh, -this.k.copLimitX, this.k.copLimitX);
      const dDesL = clamp(rdX * sh + rdZ * ch, -this.k.copLimitZ, this.k.copLimitZ);
      const rcX = f.cop.x - f.ankle.x, rcZ = f.cop.z - f.ankle.z;
      const errX = dDesF - (rcX * ch - rcZ * sh);
      const errZ = dDesL - (rcX * sh + rcZ * ch);
      /* CONTACT-FORCE CAP. Moving the CoP by `err` costs torque F*err, so scaling the
         feedforward by the measured load is correct -- right up until the load is a solver
         spike rather than a load. Measured while driving: cf_max reached 13.33x body weight
         in a single substep, alternating with frames of zero contact, so a hammer-blow
         landing became a 13x ankle torque impulse, which shoves the machine, which produces
         a harder landing. The Light Frame was moving 199 mm/s against a 4 mm/s command --
         48.5x -- while the Scout and Heavy tracked theirs at 0.8x and 0.7x.
         One foot cannot steadily carry more than the whole machine; 1.5 W leaves room for a
         genuine landing transient and cuts the worst observed spike by 8.9x. This bounds
         the feedforward without touching what it does at normal load, where F < 1.5 W and
         the cap never binds. */
      const Fcap = this.k.forceCap * st.mass * G;
      const Fff = Math.min(f.force, Fcap);
      aJ.tauFF = this.k.signPitch * -this.k.kCop * Fff * errX;
      // In DOUBLE support the net lateral CoP is set by how load is shared between the
      // feet, not by rolling either one. Commanding foot roll here just tips a foot onto
      // its edge and sheds contact area, which is strictly destabilising. Ankle roll is
      // only the right lever in single support.
      rJ.tauFF = nSupport > 1 ? 0 : this.k.signRoll * this.k.kCop * Fff * errZ;
    }

    // --- hip strategy ---------------------------------------------------------------
    // Ankle torque caps how far the CoP can travel: dxMax = tauMax / F. Past that the
    // ankle is saturated and the only remaining authority is counter-rotating the trunk
    // to generate centroidal angular momentum. Engage strictly on the excess.
    // SCALE FIX: absolute 1 N floor is 10% of a 1 kg rig's weight.
    const Fsum = Math.max(1e-3 * st.mass * G, st.totalContactForce);
    const dxMax = (J[`ankleYoke${sides[0]}`].tauMax * 2) / Fsum;
    const dzMax = (J[`foot${sides[0]}`].tauMax * 2) / Fsum;
    const exX = Math.sign(eX) * Math.max(0, Math.abs(eX) - dxMax);
    const exZ = Math.sign(eZ) * Math.max(0, Math.abs(eZ) - dzMax);

    // --- torso attitude held by the hips (position servo, trimmed by lean error) ---
    // d(lean)/dt, not raw body rate: tipping forward (+X) is a NEGATIVE rotation about
    // +Z, so feeding omega_z straight in makes the damping term anti-damping.
    const pitchErr = st.lean.pitch, rollErr = st.lean.roll;
    const pitchRate = -st.torsoRate.z, rollRate = -st.torsoRate.x;
    const T = this.k.hipTrimLimit;
    const hipPitchTrim = clamp(this.k.signHip * (this.k.hipKp * pitchErr + this.k.hipKd * pitchRate)
                             + this.k.hipStrategy * exX, -T, T);
    const hipRollTrim = clamp(this.k.signHip * (this.k.hipKp * rollErr + this.k.hipKd * rollRate)
                            + this.k.hipStrategy * exZ + this.k.lateralShift * eZ, -T, T);

    for (const s of sides) {
      J[`thigh${s}`].target = this.stance.hip + hipPitchTrim;
      J[`hipYoke${s}`].target = hipRollTrim;
      J[`shin${s}`].target = this.stance.knee;
    }
    this.debug = { copDesX, copDesZ, xiX, xiZ, eX, eZ, hipPitchTrim, hipRollTrim, loaded: nSupport };
  }
}
