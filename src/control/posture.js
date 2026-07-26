/* ===== control/posture.mjs ===== */
// posture.mjs — drive the legs from a desired pelvis pose plus desired foot placements.
// Every joint target comes out of legIK, so the commanded configuration is always
// consistent with the closed chain through the ground.




class Posture {
  constructor(rig) {
    this.rig = rig;
    this.hip = {};
    for (const s of ['L', 'R']) {
      // hipYaw is the ring bolted to the pelvis, so it carries the pivot location
      const jp = rig.table[`hipYaw${s}`].jp;
      this.hip[s] = V(jp[0], jp[1], jp[2]);      // hip pivot in pelvis local
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
  apply(pelvis, feet, actual = null, quat = null, footYaw = null) {
    const J = this.rig.joints;
    const base = actual ? V(actual.x, pelvis.y, actual.z) : pelvis;
    /* Yaw reference. This uses the COMMANDED pelvis yaw. Driving it from the pelvis's
       measured yaw instead was tried and is much worse: both legs then command the yaw
       ring to absorb whatever yaw error the body has, so neither leg ever corrects it and
       the body windmills -- every case in the drive matrix, standing included, collapsed
       inside 6 s. Absorbing yaw error and correcting it need different signs on the stance
       and swing legs, which is a controller that does not exist yet. Until it does, the
       ring is commanded flat and earns its keep as a compliant element. */
    const yaw = quat || Q();
    const pelvisYaw = 2 * Math.atan2(yaw.y, yaw.w);   // yaw-only quat -> angle about +Y
    for (const s of ['L', 'R']) {
      // hip offset and the hip->foot vector both live in the PELVIS frame, so a yawed
      // pelvis just rotates them; legIK is unchanged.
      const hipWorld = vadd(base, qrot(yaw, this.hip[s]));
      const d = qrotInv(yaw, vsub(feet[s], hipWorld));
      const rel = footYaw ? wrapPi(footYaw[s] - pelvisYaw) : 0;
      const q = legIK(d, { yaw: rel, thigh: this.rig.leg.thigh, shin: this.rig.leg.shin });
      J[`hipYaw${s}`].target = q.hipYaw;
      J[`hipYoke${s}`].target = q.hipRoll;
      J[`thigh${s}`].target = q.hipPitch;
      J[`shin${s}`].target = q.knee;
      J[`ankleYoke${s}`].target = q.anklePitch;
      J[`foot${s}`].target = q.ankleRoll;
      this.last[s] = q;
    }
  }
}
