/* ===== control/ik.mjs ===== */
// ik.mjs — closed-form inverse kinematics for one leg.
//
// Why this exists: the two legs plus the ground form a CLOSED kinematic chain. Ten leg
// joints, six loop-closure constraints, so only four are independently commandable.
// Commanding all ten separately over-constrains the loop and the actuators fight each
// other hard enough to diverge the solver. Driving joint targets from a desired pelvis
// pose plus desired foot poses makes every command consistent by construction.
//
// Leg chain from the pelvis:
//   hipYoke (roll, +X) -> thigh (pitch, +Z) -> shin (knee, +Z)
//     -> ankleYoke (ankle pitch, +Z) -> foot (roll, +X)
// The three pitch joints sum to the foot's sagittal orientation, so a level sole means
// thigh + shin + ankleYoke = 0.


const LEG = { thigh: 1.50, shin: 1.45 };
const wrapPi = (a) => Math.atan2(Math.sin(a), Math.cos(a));

/* Solve for the five joint angles that put the ankle pivot at `d`, expressed in the
   pelvis frame relative to the hip pivot, with the sole held flat.
   Returns { hipRoll, hipPitch, knee, anklePitch, ankleRoll, reach } where `reach` is the
   fraction of full leg extension demanded (>=1 means the target is unreachable). */
function legIK(d0, opts = {}) {
  const Lt = opts.thigh ?? LEG.thigh, Ls = opts.shin ?? LEG.shin;
  const maxR = (Lt + Ls) * (opts.maxExtend ?? 0.995);

  // Hip yaw is the outermost axis of the gimbal, so it can be taken off the front in
  // closed form: rotate the target into the yawed leg frame by -yaw and the remaining
  // five joints solve exactly as before. `yaw` is the commanded foot heading relative to
  // the pelvis. Everything downstream is unchanged, which is why the leg-length solution
  // stays exact rather than becoming an iterative 6-DOF solve.
  const hipYaw = opts.yaw ?? 0;
  const cy = Math.cos(hipYaw), sy = Math.sin(hipYaw);
  const d = { x: cy * d0.x - sy * d0.z, y: d0.y, z: sy * d0.x + cy * d0.z };

  // Hip roll puts the sagittal plane of the leg through the target. Positive rotation
  // about +X carries the downward leg axis toward -Z, so reaching toward +Z needs a
  // NEGATIVE roll.
  const hipRoll = -Math.atan2(d.z, -d.y);

  // in that plane: a = forward offset, b = drop
  const a = d.x;
  const b = Math.hypot(d.y, d.z);
  let r = Math.hypot(a, b);
  const reach = r / (Lt + Ls);
  if (r > maxR) r = maxR;
  const rEps = 3.4e-5 * (Lt + Ls);   // SCALE FIX: was an absolute 1e-4 m
  if (r < Math.abs(Lt - Ls) + rEps) r = Math.abs(Lt - Ls) + rEps;

  // knee from the law of cosines; joint angle is the supplement of the interior angle
  const cosInterior = (Lt * Lt + Ls * Ls - r * r) / (2 * Lt * Ls);
  const interior = Math.acos(Math.max(-1, Math.min(1, cosInterior)));
  const knee = Math.PI - interior;

  // thigh sits `alpha` off the hip->ankle line, which itself sits `beta` off vertical
  const beta = Math.atan2(a, b);
  const cosAlpha = (Lt * Lt + r * r - Ls * Ls) / (2 * Lt * r);
  const alpha = Math.acos(Math.max(-1, Math.min(1, cosAlpha)));
  const hipPitch = beta - alpha;

  // level sole: the three pitch joints must sum to zero; roll cancels the hip roll
  const anklePitch = -(hipPitch + knee);
  const ankleRoll = -hipRoll;

  return { hipYaw, hipRoll, hipPitch, knee, anklePitch, ankleRoll, reach };
}

/* Forward kinematics of the same chain, used to verify the inverse. */
function legFK(q, opts = {}) {
  const Lt = opts.thigh ?? LEG.thigh, Ls = opts.shin ?? LEG.shin;
  const thighTilt = q.hipPitch;
  const shinTilt = q.hipPitch + q.knee;
  const a = Lt * Math.sin(thighTilt) + Ls * Math.sin(shinTilt);
  const b = Lt * Math.cos(thighTilt) + Ls * Math.cos(shinTilt);
  // the planar solution lives in the rolled plane: down axis is R_x(roll) * (0,-1,0)
  const p = V(a, -b * Math.cos(q.hipRoll), -b * Math.sin(q.hipRoll));
  // undo the yaw taken off the front in legIK: rotate back by +yaw about +Y
  const y = q.hipYaw || 0, cy = Math.cos(y), sy = Math.sin(y);
  return V(cy * p.x + sy * p.z, p.y, -sy * p.x + cy * p.z);
}
