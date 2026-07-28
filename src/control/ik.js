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
   Returns { hipRoll, hipPitch, knee, anklePitch, ankleRoll, reach, ext } where `reach` is the
   fraction of full leg extension DEMANDED (>=1 means the target is unreachable) and `ext` is
   the fraction actually COMMANDED after clamping. Feed `ext` back as `opts.holdExt` next
   frame -- see the unreachable-target clamp below for why. */
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
  /* UNREACHABLE TARGET: CLAMP, BUT NEVER LENGTHEN THE LEG TO MEET IT.
     This line used to be a bare `if (r > maxR) r = maxR`, which is the machine's launch
     mechanism. maxExtend is 0.995 and the stance leg rests at restExt 0.93, so an out-of-range
     target did not merely fail to be met -- it commanded the leg 11.7 mm LONGER than the one
     it was holding (Light Frame, leg 180.1 mm), on both legs at once, into a stance spring of
     11.5 kN/m per leg where 3.46 mm of length error already rails a knee. The servos drove to
     it and the machine threw itself off the floor: driving log s20260727004023 at t=10.6,
     pelvis 0.830 -> 0.962 m, vertical velocity -0.01 -> +1.50 m/s in one 10 Hz sample, both
     feet unloading for 0.3 s, torso mount torn at util 1.04. chassis.js bodyRef() rate-limits
     the REFERENCE, which changes how fast the target leaves the reachable set; it cannot stop
     the clamp lengthening the leg once it has.

     So when the demand is unreachable, hold the length already commanded (`opts.holdExt`, fed
     back from the previous frame's `ext`) instead of straightening. REACHABLE targets are
     untouched -- the clamp only engages above maxR -- so ordinary walking, where the stride cap
     is sized to keep reach inside 0.995 at both ends of stance, sees no change at all.

     THE COST, stated because it is real: a leg that genuinely needs to extend past its held
     length while the target is out of range will now refuse to, and a swing foot can end up
     short of its placement rather than snapping straight. That is deliberate -- an
     under-extended leg in the air costs a footfall, an over-extended one on the ground costs
     the whole machine. With no holdExt supplied (gate G11, any standalone caller) the old
     behaviour is unchanged. */
  if (r > maxR) {
    r = opts.holdExt !== undefined ? Math.min(maxR, (Lt + Ls) * opts.holdExt) : maxR;
  }
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

  return { hipYaw, hipRoll, hipPitch, knee, anklePitch, ankleRoll, reach, ext: r / (Lt + Ls) };
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
