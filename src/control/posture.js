/* ===== control/posture.mjs ===== */
// posture.mjs — drive the legs from a desired pelvis pose plus desired foot placements.
// Every joint target comes out of legIK, so the commanded configuration is always
// consistent with the closed chain through the ground.




class Posture {
  constructor(rig, opts = {}) {
    this.rig = rig;
    /* HOLD AN UNREACHABLE TARGET INSTEAD OF STRAIGHTENING -- OFF BY DEFAULT, and it is off
       because it was DRIVEN and turning got worse.

       Shipped 2026-07-27 to stop the launch documented at ik.js's clamp: an out-of-range target
       commanded the stance leg 11.7 mm longer than it was holding, which is 3.4x what rails a
       knee. The arithmetic was verified and reachable targets were proven bit-identical. Turning
       still degraded, which is exactly the cost that was flagged with it: a turn plants one foot
       and rotates the body about it, so the trailing leg's hip-to-ankle distance grows, and
       freezing its length instead of extending leaves the foot short and drops that hip -- a roll
       disturbance introduced in the middle of the manoeuvre that was already the hardest.

       Kept behind a flag rather than deleted: the launch it prevents is real and measured, and
       the fix may be right with a load gate on it (freeze a LOADED leg, let a swing leg extend)
       -- but posture.apply() is not told which feet are loaded, so that is a bigger change than
       this one. Do not turn this on again without instrumentation that can see the clamp fire;
       there is no log field for it today. */
    this.holdUnreachable = opts.holdUnreachable === true;
    this.hip = {};
    /* Over the rig's own side list -- two legs named L and R, or four named FL/FR/RL/RR.
       legIK does not care how many limbs there are; only these loops did. */
    for (const s of rig.sides) {
      // hipYaw is the ring bolted to the pelvis, so it carries the pivot location
      const jp = rig.table[`hipYaw${s}`].jp;
      this.hip[s] = V(jp[0], jp[1], jp[2]);      // hip pivot in pelvis local
    }
    /* Never COMMAND the ring past its own end stop. Beyond it the request is unreachable,
       the actuator saturates against the stop, and legIK still swings the whole leg plane
       out to a target the joint cannot deliver. Stay inside it.
       Both the limit and the slew rate come from deriveGait (rig/derive.js) when the caller has
       them; the fallback keeps Posture constructible standalone. */
    const rng = rig.table[`hipYaw${rig.sides[0]}`].range;
    this.yawLimit = opts.yawLimit ?? (rng ? 0.9 * Math.min(Math.abs(rng[0]), Math.abs(rng[1])) : 0);
    /* Ring slew, rad/s. Infinity = the old unlimited behaviour, which measured 209 rad/s of
       commanded step at a gait transition and tore mounts off. See rig/derive.js hipYawRate. */
    this.yawRate = opts.yawRate ?? Infinity;
    this.ring = {};                      // slewed ring command per side, rad
    /* NEVER COMMAND A JOINT PAST ITS OWN END STOP.
       legIK solves for a LEVEL SOLE -- anklePitch = -(hipPitch + knee), ankleRoll = -hipRoll --
       and it has no knowledge of joint ranges. On the Scout's leg geometry that solution falls
       outside the mechanical limits, and Posture wrote it anyway, so the actuator drove into the
       bumper and held it there.
       Measured, log s20260728002350 (MK1.23.0), the eight samples before the Scout tore its ankles
       off: ankleYokeL held 30.5-30.9 deg against a 30 deg limit, footL held -25.0 against -25,
       footR 25.6 against 25, with end-stop reaction torque of 285-657 mN.m. The Scout's ankleYoke
       ceiling as driven is 303 mN.m and its foot 294 -- so the STOPS were carrying up to 2.2x the
       actuator's entire authority, continuously. measure() folds that into the torsion channel
       (Mt = |tau + tauLimit|), which parked utilisation at 0.13-0.17 before anything hit the
       ground; one impact then took it to 1.5 and the mount let go. In the softring log ankleYokeL
       sat at exactly -40.0 deg, its range floor, for six samples running.
       0.95 of the declared range, so the servo stops short of the bumper rather than on it. The
       sole tilts instead of the mount tearing, which is what a real leg does when it runs out of
       ankle. Beyond the range the command was never achievable anyway -- this gives up nothing
       the machine could actually deliver. */
    this.jlim = {};
    for (const n of Object.keys(rig.table)) {
      const r = rig.table[n] && rig.table[n].range;
      if (r) this.jlim[n] = [0.95 * r[0], 0.95 * r[1]];
    }
    this.last = {};
  }
  /* pelvis: world position reference for the pelvis (orientation assumed upright)
     feet:   { L: worldAnklePos, R: worldAnklePos }
     actual: measured pelvis position, optional

     Reference frame split. Solving purely against the REFERENCE pelvis means a commanded
     foot lands at actual + (target - reference), so whenever the body deviates the
     planted foot is dragged along with it -- that is what made tracking error accumulate
     until the walk collapsed. Solving purely against the MEASURED pelvis keeps the feet
     planted but loses vertical support: if the pelvis sags the IK just shortens the leg.
     So take horizontal from the measurement and vertical from the reference. */
  /* footYaw: optional { L, R } desired foot heading in WORLD radians. The pelvis frame
     already carries `quat`, so what legIK needs is the difference. This is the whole
     point of the yaw ring: the feet can be pointed somewhere the pelvis is not. */
  /* dt is the last argument and is only used to slew the yaw ring. Callers that pass nothing get
     the ring applied unslewed, which is the behaviour that tore mounts off -- every shipped call
     site passes it. */
  apply(pelvis, feet, actual = null, quat = null, footYaw = null, pelvisYawMeas = null, dt = 0) {
    const J = this.rig.joints;
    const base = actual ? V(actual.x, pelvis.y, actual.z) : pelvis;
    /* Two different yaws, and conflating them is what made the rings dead.
       `yaw` (the COMMANDED frame) is GEOMETRY: it rotates the hip offsets and the
       hip->foot vector, so the footstep pattern is laid out on the commanded heading.
       `pelvisYaw` is the frame the RING ANGLE is measured against, and it has to be the
       measured pelvis or the ring cannot steer anything. It used to be read back out of
       the same commanded quat, so `rel = footYaw - pelvisYaw` subtracted a number from
       itself and every ring was commanded to exactly 0.000000 at every heading from 0 to
       180 deg. The one joint on the machine that can point the feet somewhere the pelvis
       is not was held flat, and the only yaw authority left was the gyro dragging the
       whole rig round against foot friction -- about 4 deg per step, whatever it asked
       for.
       Passing measured yaw here was tried once before and collapsed everything inside
       6 s, because both legs then absorbed the error and neither corrected it. The caller
       now supplies stance and swing headings with OPPOSITE signs (control/gait.mjs,
       footYaw()), which is the piece that was missing. Callers that want the old flat
       behaviour -- standing, warm-up, double support -- pass null and get it. */
    const yaw = quat || Q();
    const pelvisYaw = pelvisYawMeas !== null ? pelvisYawMeas
                    : 2 * Math.atan2(yaw.y, yaw.w);   // yaw-only quat -> angle about +Y
    for (const s of this.rig.sides) {
      // hip offset and the hip->foot vector both live in the PELVIS frame, so a yawed
      // pelvis just rotates them; legIK is unchanged.
      const hipWorld = vadd(base, qrot(yaw, this.hip[s]));
      const d = qrotInv(yaw, vsub(feet[s], hipWorld));
      /* RING COMMAND, CLAMPED THEN SLEWED. `rel` is a POSITION command and it was applied raw:
         it is identically 0 whenever no leg is free (double support, STAND, warm-up) and jumps to
         the full clamp the instant single support begins, so the ring saw a 40.5 deg step at every
         gait transition -- 209 rad/s, 5.7x the waist-ring slam that tore the arms off. Slew it. */
      const want = footYaw ? clamp(wrapPi(footYaw[s] - pelvisYaw), -this.yawLimit, this.yawLimit) : 0;
      const held = this.ring[s] !== undefined ? this.ring[s] : want;
      const dr = this.yawRate * dt;
      const rel = dr > 0 && Number.isFinite(dr) ? held + clamp(want - held, -dr, dr) : want;
      this.ring[s] = rel;
      /* holdExt is the extension this leg was LAST COMMANDED, fed back so an unreachable target
         cannot straighten the leg past what it is already holding. Passing undefined gives
         ik.js's plain 0.995 clamp, which is the shipped behaviour -- see holdUnreachable above
         for why this is gated off. */
      const q = legIK(d, { yaw: rel, thigh: this.rig.leg.thigh, shin: this.rig.leg.shin,
                           holdExt: this.holdUnreachable ? this.last[s]?.ext : undefined });
      /* Every leg target goes through the range clamp -- see jlim above. A joint with no declared
         range passes through untouched. */
      const set = (name, v) => {
         const L = this.jlim[name];
         J[name].target = L ? Math.max(L[0], Math.min(L[1], v)) : v;
      };
      set(`hipYaw${s}`, q.hipYaw);
      set(`hipYoke${s}`, q.hipRoll);
      set(`thigh${s}`, q.hipPitch);
      set(`shin${s}`, q.knee);
      set(`ankleYoke${s}`, q.anklePitch);
      set(`foot${s}`, q.ankleRoll);
      this.last[s] = q;
    }
  }
}
