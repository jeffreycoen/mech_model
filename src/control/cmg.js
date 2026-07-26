/* ===== control/cmg.mjs =====
   Control moment gyro / reaction wheel assembly.

   This is the ONE piece of assistance in the build, and it is modelled hardware rather
   than a fudge factor: a flywheel assembly carried in the torso that exchanges angular
   momentum with the chassis. It can right the torso without pushing on the ground, which
   is exactly what a walking machine wants when a foot is in the air.

   It is not free, and the three costs are all simulated:
     - MASS. The assembly is added to the torso, so it is carried and it loads the mounts.
     - TORQUE LIMIT. tauMax caps what it can apply, same as any actuator here.
     - MOMENTUM SATURATION. A wheel can only absorb so much angular momentum before it is
       spun up and has nothing left to give. H tracks the stored momentum; once |H| hits
       hMax, torque that would spin it further is simply unavailable. That is the honest
       failure mode and it is why this cannot make the mech unfallable.
   Momentum bleeds off slowly against the ground through the stance legs (desaturation),
   which is what lets it recover authority between disturbances.

   Total angular momentum is still conserved -- the wheel holds whatever the chassis gave
   up, which is what H measures. Nothing here reaches into the solver. */
class CMG {
  constructor(o = {}) {
    this.body = o.body;
    this.tauMax = o.tauMax ?? 45e3;      // N.m
    this.hMax = o.hMax ?? 2.2e4;         // N.m.s of stored momentum
    this.kp = o.kp ?? 150e3;             // N.m per rad of lean
    this.kd = o.kd ?? 42e3;              // N.m per rad/s
    this.desat = o.desat ?? 7.0;         // s, momentum bleed time constant
    this.yawDamp = o.yawDamp ?? 0.5;     // yaw rate damping, fraction of kd
    /* 0.30, swept: 135 deg turn in 4.2 s vs 13.3 s at 0.03, and forward travel IMPROVES
       (4.68 m vs 4.51 m per 25 s) rather than regressing. Before this session a turn
       command never rotated the machine at all. */
    this.yawGain = o.yawGain ?? 0.30;   // yaw steering authority, fraction of kp
    /* IDEAL MODE: full-authority attitude assist. No momentum store, no saturation,
       and it also servos yaw onto the commanded facing. Not physical hardware any
       more -- the stated goal is a tabletop bot that only falls when the maneuver is
       extreme, so the stabiliser is allowed to be as good as the fiction needs. */
    this.ideal = o.ideal !== false;
    this.targetYaw = undefined;
    this.H = V(); this.tau = V();
    this.enabled = o.enabled !== false;
    this.satFrac = 0; this.tauFrac = 0;
  }
  update(st, dt) {
    if (!this.enabled || !this.body) { if (this.body) this.body.tExt = V(); this.tau = V(); return; }
    // Sign convention matches balance.mjs: tipping toward +X is a NEGATIVE rotation about
    // +Z, so the corrective torque about +Z is positive for a positive pitch lean, and the
    // rate term uses -omega rather than raw body rate or the damping becomes anti-damping.
    const pitchRate = -st.torsoRate.z, rollRate = -st.torsoRate.x;
    let yawT = this.kd * this.yawDamp * -st.torsoRate.y;
    if (this.ideal && this.targetYaw !== undefined) {
      const f = qrot(this.body.q, V(1, 0, 0));
      let e = this.targetYaw - Math.atan2(-f.z, f.x);
      while (e > Math.PI) e -= 2 * Math.PI; while (e < -Math.PI) e += 2 * Math.PI;
      yawT = this.kp * this.yawGain * e + this.kd * 0.5 * -st.torsoRate.y;
    }
    let t = V(this.kp * st.lean.roll + this.kd * rollRate, yawT,
              this.kp * st.lean.pitch + this.kd * pitchRate);
    const cap = this.tauMax;
    const m = vlen(t);
    if (m > cap) t = vmul(t, cap / m);
    if (!this.ideal) {
      // Saturation: strip any component that would spin the wheel past its momentum store.
      const hn = vlen(this.H);
      if (hn > this.hMax) {
        const n = vmul(this.H, 1 / hn);
        const along = vdot(t, n);
        if (along > 0) t = vsub(t, vmul(n, along));
      }
      this.H = vadd(this.H, vmul(t, dt));
      this.H = vmul(this.H, Math.max(0, 1 - dt / this.desat));   // bleed into the ground
      const hn2 = vlen(this.H);
      if (hn2 > this.hMax) this.H = vmul(this.H, this.hMax / hn2);
    }
    this.tau = t;
    this.tauFrac = vlen(t) / cap;
    this.satFrac = this.ideal ? 0 : vlen(this.H) / this.hMax;
    this.body.tExt = t;
  }
}

/* Fit a CMG to an assembled rig. The flywheel assembly is real mass bolted into the
   torso, so it is added there and the inertia scaled with it. */
function fitCMG(rig, o = {}) {
  const b = rig.bodies.torso;
  const mass = o.mass ?? 180;
  const k = (b.mass + mass) / b.mass;
  b.mass += mass; b.invMass = 1 / b.mass;
  b.I = b.I.map((v) => v * k); b.invI = m3inv(b.I);
  return new CMG(Object.assign({ body: b }, o));
}

class BalanceController {
  constructor(rig, cfg = {}) {
    this.rig = rig;
    this.k = Object.assign({
      ankleKp: 0.03, ankleKd: 0.011,     // capture-point error (m) -> ankle angle (rad)
      ankleTrim: 0.16,                   // max ankle trim, rad
      signPitch: 1, signRoll: 1, signHip: 1,
      kCop: 0.6,             // CoP error (m) -> ankle torque, as a fraction of F
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

    const nSupport = (st.feet.L.contact ? 1 : 0) + (st.feet.R.contact ? 1 : 0);
    // --- ankles: hold the stance pose on PD, bias the torque to move the MEASURED CoP.
    // Closing on measured CoP rather than commanding open-loop torque means the loop is
    // robust to whatever the servo happens to be doing.
    for (const s of ['L', 'R']) {
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
      aJ.tauFF = this.k.signPitch * -this.k.kCop * f.force * errX;
      // In DOUBLE support the net lateral CoP is set by how load is shared between the
      // feet, not by rolling either one. Commanding foot roll here just tips a foot onto
      // its edge and sheds contact area, which is strictly destabilising. Ankle roll is
      // only the right lever in single support.
      rJ.tauFF = nSupport > 1 ? 0 : this.k.signRoll * this.k.kCop * f.force * errZ;
    }

    // --- hip strategy ---------------------------------------------------------------
    // Ankle torque caps how far the CoP can travel: dxMax = tauMax / F. Past that the
    // ankle is saturated and the only remaining authority is counter-rotating the trunk
    // to generate centroidal angular momentum. Engage strictly on the excess.
    // SCALE FIX: absolute 1 N floor is 10% of a 1 kg rig's weight.
    const Fsum = Math.max(1e-3 * st.mass * G, st.totalContactForce);
    const dxMax = (J.ankleYokeL.tauMax * 2) / Fsum;
    const dzMax = (J.footL.tauMax * 2) / Fsum;
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

    for (const s of ['L', 'R']) {
      J[`thigh${s}`].target = this.stance.hip + hipPitchTrim;
      J[`hipYoke${s}`].target = hipRollTrim;
      J[`shin${s}`].target = this.stance.knee;
    }
    this.debug = { copDesX, copDesZ, xiX, xiZ, eX, eZ, hipPitchTrim, hipRollTrim, loaded: (st.feet.L.contact?1:0) + (st.feet.R.contact?1:0) };
  }
}
