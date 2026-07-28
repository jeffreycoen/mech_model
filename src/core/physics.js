/* ===== core/physics.mjs ===== */
// physics.mjs — XPBD substepped rigid body core (SI units: m, kg, s, N, N.m)

/* ---------------- vec3 ---------------- */
const V = (x = 0, y = 0, z = 0) => ({ x, y, z });
const vadd = (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
const vsub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const vmul = (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s });
const vdot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
const vcross = (a, b) => ({
  x: a.y * b.z - a.z * b.y,
  y: a.z * b.x - a.x * b.z,
  z: a.x * b.y - a.y * b.x,
});
const vlen = (a) => Math.hypot(a.x, a.y, a.z);
const vlen2 = (a) => a.x * a.x + a.y * a.y + a.z * a.z;
const vnorm = (a) => { const l = vlen(a); return l > 1e-12 ? vmul(a, 1 / l) : V(); };

/* ---------------- quat (x,y,z,w) ---------------- */
const Q = (x = 0, y = 0, z = 0, w = 1) => ({ x, y, z, w });
const qmul = (a, b) => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
});
const qconj = (q) => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });
const qnorm = (q) => {
  const l = Math.hypot(q.x, q.y, q.z, q.w) || 1;
  return { x: q.x / l, y: q.y / l, z: q.z / l, w: q.w / l };
};
function qrot(q, v) {
  const t = vmul(vcross(V(q.x, q.y, q.z), v), 2);
  return vadd(vadd(v, vmul(t, q.w)), vcross(V(q.x, q.y, q.z), t));
}
const qrotInv = (q, v) => qrot(qconj(q), v);
function qAxisAngle(axis, ang) {
  const a = vnorm(axis), s = Math.sin(ang / 2);
  return { x: a.x * s, y: a.y * s, z: a.z * s, w: Math.cos(ang / 2) };
}

/* ---------------- mat3 (row-major array of 9) ---------------- */
const m3mulv = (m, v) => ({
  x: m[0] * v.x + m[1] * v.y + m[2] * v.z,
  y: m[3] * v.x + m[4] * v.y + m[5] * v.z,
  z: m[6] * v.x + m[7] * v.y + m[8] * v.z,
});
function m3inv(m) {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  // Relative, not absolute. det has units of (kg m^2)^3, so a fixed 1e-18 floor is a
  // statement about rig SIZE rather than about singularity: a 1/16-scale rig has a
  // perfectly well-conditioned ankle whose determinant is legitimately below it.
  const mag = Math.max(Math.abs(a), Math.abs(e), Math.abs(i), 1e-300);
  if (Math.abs(det) < 1e-12 * mag * mag * mag) throw new Error('singular inertia tensor');
  const id = 1 / det;
  return [
    A * id, (c * h - b * i) * id, (b * f - c * e) * id,
    B * id, (a * i - c * g) * id, (c * d - a * f) * id,
    C * id, (b * g - a * h) * id, (a * e - b * d) * id,
  ];
}
const m3diag = (x, y, z) => [x, 0, 0, 0, y, 0, 0, 0, z];
const m3add = (a, b) => a.map((v, i) => v + b[i]);
const m3sub = (a, b) => a.map((v, i) => v - b[i]);
const m3scale = (a, s) => a.map((v) => v * s);
const skew = (v) => [0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0];
function qExp(v) {  // exponential map: rotation vector -> quat
  const t = vlen(v);
  if (t < 1e-9) return qnorm(Q(v.x * 0.5, v.y * 0.5, v.z * 0.5, 1));
  const s = Math.sin(t / 2) / t;
  return { x: v.x * s, y: v.y * s, z: v.z * s, w: Math.cos(t / 2) };
}

/* ---------------- inertia primitives (about own COM, local axes) ---------------- */
const boxInertia = (m, w, h, d) =>
  m3diag(m * (h * h + d * d) / 12, m * (w * w + d * d) / 12, m * (w * w + h * h) / 12);

// Compose sub-parts into one rigid body's mass properties.
// parts: [{mass, inertia:mat3 (about sub-COM, in sub frame), pos:vec3, quat?:quat}]
function compound(parts) {
  let M = 0, com = V();
  for (const p of parts) { M += p.mass; com = vadd(com, vmul(p.pos, p.mass)); }
  com = vmul(com, 1 / M);
  const I = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (const p of parts) {
    const q = p.quat || Q();
    // rotate sub inertia into parent frame: R I R^T
    const R = quatToM3(q), Rt = m3transpose(R);
    const Ir = m3mul(m3mul(R, p.inertia), Rt);
    const d = vsub(p.pos, com), d2 = vlen2(d);
    const pa = [
      p.mass * (d2 - d.x * d.x), p.mass * (-d.x * d.y), p.mass * (-d.x * d.z),
      p.mass * (-d.y * d.x), p.mass * (d2 - d.y * d.y), p.mass * (-d.y * d.z),
      p.mass * (-d.z * d.x), p.mass * (-d.z * d.y), p.mass * (d2 - d.z * d.z),
    ];
    for (let k = 0; k < 9; k++) I[k] += Ir[k] + pa[k];
  }
  return { mass: M, com, inertia: I };
}
function quatToM3(q) {
  const { x, y, z, w } = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}
const m3transpose = (m) => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
function m3mul(a, b) {
  const o = new Array(9);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++)
    o[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
  return o;
}

/* ---------------- Body ---------------- */
class Body {
  constructor(o = {}) {
    this.name = o.name || 'body';
    this.mass = o.mass ?? 1;
    this.I = o.inertia || m3diag(1, 1, 1);
    this.kinematic = !!o.kinematic;
    this.invMass = this.kinematic ? 0 : 1 / this.mass;
    this.invI = this.kinematic ? [0, 0, 0, 0, 0, 0, 0, 0, 0] : m3inv(this.I);
    this.x = o.pos || V();
    this.q = qnorm(o.quat || Q());
    this.v = o.vel || V();
    this.w = o.angVel || V();
    this.xp = this.x; this.qp = this.q;
    this.fExt = V(); this.tExt = V();
    this.hRotor = 0;   // stored flywheel momentum about local +Y, N.m.s
  }
  toWorld(p) { return vadd(this.x, qrot(this.q, p)); }
  dirWorld(d) { return qrot(this.q, d); }
  kinetic() {
    const wl = qrotInv(this.q, this.w);
    return 0.5 * this.mass * vlen2(this.v) + 0.5 * vdot(wl, m3mulv(this.I, wl));
  }
}

/* ---------------- generalized inverse masses ---------------- */
function genInvMass(b, r, n) {
  if (b.invMass === 0) return 0;
  const rn = vcross(r, n);
  const rnL = qrotInv(b.q, rn);
  return b.invMass + vdot(rnL, m3mulv(b.invI, rnL));
}
function angInvMass(b, n) {
  if (b.invMass === 0) return 0;
  const nL = qrotInv(b.q, n);
  return vdot(nL, m3mulv(b.invI, nL));
}
function applyInvI(b, pWorld) {
  const pL = qrotInv(b.q, pWorld);
  return qrot(b.q, m3mulv(b.invI, pL));
}
function rotateBy(b, dwWorld, sign) {
  const dq = qmul(Q(dwWorld.x * sign, dwWorld.y * sign, dwWorld.z * sign, 0), b.q);
  b.q = qnorm({ x: b.q.x + 0.5 * dq.x, y: b.q.y + 0.5 * dq.y, z: b.q.z + 0.5 * dq.z, w: b.q.w + 0.5 * dq.w });
}

/* Positional correction. err = anchor2 - anchor1 (world). Bodies pulled together.
   Returns dlambda (>=0 when err>0). p = dlambda * n is the impulse applied to b1 (+) / b2 (-). */
function solvePositional(b1, b2, r1, r2, err, compliance, h, lambda) {
  const c = vlen(err);
  if (c < 1e-12) return { dl: 0, n: V() };
  const n = vmul(err, 1 / c);
  const w1 = genInvMass(b1, r1, n), w2 = genInvMass(b2, r2, n);
  const wsum = w1 + w2;
  if (wsum < 1e-18) return { dl: 0, n };
  const a = compliance / (h * h);
  const dl = (c - a * lambda) / (wsum + a);
  if (dl === 0) return { dl: 0, n };
  const p = vmul(n, dl);
  if (b1.invMass) { b1.x = vadd(b1.x, vmul(p, b1.invMass)); rotateBy(b1, applyInvI(b1, vcross(r1, p)), +1); }
  if (b2.invMass) { b2.x = vsub(b2.x, vmul(p, b2.invMass)); rotateBy(b2, applyInvI(b2, vcross(r2, p)), -1); }
  return { dl, n };
}

/* Angular correction. theta = rotation vector (world) to apply to b1 (+) / b2 (-). */
function solveAngular(b1, b2, theta, compliance, h, lambda) {
  const c = vlen(theta);
  if (c < 1e-12) return { dl: 0, n: V() };
  const n = vmul(theta, 1 / c);
  const w1 = angInvMass(b1, n), w2 = angInvMass(b2, n);
  const wsum = w1 + w2;
  if (wsum < 1e-18) return { dl: 0, n };
  const a = compliance / (h * h);
  const dl = (c - a * lambda) / (wsum + a);
  if (dl === 0) return { dl: 0, n };
  const p = vmul(n, dl);
  if (b1.invMass) rotateBy(b1, applyInvI(b1, p), +1);
  if (b2.invMass) rotateBy(b2, applyInvI(b2, p), -1);
  return { dl, n };
}

/* rotation vector taking b1's rest-relative frame onto b2's actual frame */
function orientationError(b1, b2, qRest) {
  const target = qmul(b1.q, qRest);       // where b2 should be
  let qe = qmul(b2.q, qconj(target));     // error rotation applied to b2
  if (qe.w < 0) qe = { x: -qe.x, y: -qe.y, z: -qe.z, w: -qe.w };
  return vmul(V(qe.x, qe.y, qe.z), 2);    // small-angle rotation vector, rotate b2 by -this
}

/* ---------------- structural failure envelope, shared by Weld and Hinge ----------------
   A mount carries four independent load types. Compression is not counted toward the
   tension term. Failure when the combined utilization reaches 1 (less accumulated damage). */
function structuralUtil(lim, Fax, Fsh, Mb, Mt) {
  const t = Math.max(0, Fax) / lim.tension;
  return Math.sqrt(t * t + (Fsh / lim.shear) ** 2 + (Mb / lim.bend) ** 2 + (Mt / lim.torsion) ** 2);
}
const NO_LIMIT = { tension: Infinity, shear: Infinity, bend: Infinity, torsion: Infinity };

/* ---------------- Weld: 6-DOF rigid attachment with breakage ---------------- */
class Weld {
  constructor(o) {
    this.name = o.name;
    this.a = o.a; this.b = o.b;
    this.ra = o.ra; this.rb = o.rb;              // anchors in each body's local frame
    this.qRest = o.qRest || qmul(qconj(o.a.q), o.b.q);
    this.axis = o.axis || V(0, 1, 0);            // weld normal in body A local frame
    this.lim = Object.assign({ tension: 1e9, shear: 1e9, bend: 1e9, torsion: 1e9 }, o.lim);
    this.compliance = o.compliance ?? 0;
    this.angCompliance = o.angCompliance ?? 0;
    this.broken = false;
    // telemetry (per substep, N and N.m)
    this.F = V(); this.T = V();
    this.util = 0; this.peakUtil = 0;
    this.Fax = 0; this.Fsh = 0; this.Mb = 0; this.Mt = 0;
  }
  reset() { this.lp = V(); this.la = V(); }
  solve(h) {
    if (this.broken) return;
    const { a, b } = this;
    // angular first. Same call Hinge.solve makes for the same job -- this was hand-inlined,
    // and the copy still carried `al * 0`, the dead accumulated-multiplier term.
    const theta = orientationError(a, b, this.qRest);
    const ra = solveAngular(a, b, theta, this.angCompliance, h, 0);
    if (ra.dl !== 0) this.la = vadd(this.la, vmul(ra.n, ra.dl));
    // positional (anchors recomputed AFTER the angular pass)
    const r1 = a.dirWorld(this.ra), r2 = b.dirWorld(this.rb);
    const err = vsub(vadd(b.x, r2), vadd(a.x, r1));
    const res = solvePositional(a, b, r1, r2, err, this.compliance, h, 0);
    if (res.dl !== 0) this.lp = vadd(this.lp, vmul(res.n, res.dl));
  }
  measure(h) {
    if (this.broken) return;
    const inv = 1 / (h * h);
    this.F = vmul(this.lp, inv);
    this.T = vmul(this.la, inv);
    const nAx = this.a.dirWorld(this.axis);
    const fa = vdot(this.F, nAx);
    this.Fax = fa;
    this.Fsh = vlen(vsub(this.F, vmul(nAx, fa)));
    const ta = vdot(this.T, nAx);
    this.Mt = ta;
    this.Mb = vlen(vsub(this.T, vmul(nAx, ta)));
    this.util = structuralUtil(this.lim, this.Fax, this.Fsh, this.Mb, this.Mt);
    if (this.util > this.peakUtil) this.peakUtil = this.util;
    if (this.util >= 1) { this.broken = true; this.brokeAtUtil = this.util; }
  }
}

/* ---------------- Revolute joint with torque-limited actuator ---------------- */
class Hinge {
  constructor(o) {
    this.name = o.name;
    this.a = o.a; this.b = o.b;
    this.ra = o.ra; this.rb = o.rb;
    this.axisA = vnorm(o.axisA); this.axisB = vnorm(o.axisB || o.axisA);
    this.refA = vnorm(o.refA); this.refB = vnorm(o.refB || o.refA);
    this.compliance = o.compliance ?? 0;
    this.alignCompliance = o.alignCompliance ?? 0;
    this.tauMax = o.tauMax ?? 0;                 // N.m
    // actuator stiffness / damping in physical units. Defaults give full torque at
    // ~3 deg of error and roughly critical damping for a limb-scale inertia.
    this.kp = o.kp ?? this.tauMax / (3 * Math.PI / 180);
    this.kd = o.kd ?? this.kp * 0.06;
    this.target = o.target ?? 0;                 // rad
    this.limits = o.limits || null;              // [lo,hi] rad
    // End-stop compliance, rad per N.m. 1e-7 gives ~0.23 deg of deflection at 40 kN.m,
    // i.e. a stiff elastomeric bumper rather than an infinitely rigid wall.
    this.limitCompliance = o.limitCompliance ?? 1e-7;
    this.lLim = 0; this.tauLimit = 0; this.onStop = false;
    this.angle = 0; this.tau = 0; this.saturated = false;
    /* SERVO INTERNALS, exposed for the log. Both of these were function-locals inside solve()
       and were computed 8 iterations x 10 substeps x every joint -- ~330 000 times a second on
       the shipped rig -- and discarded every time, which is why no driving log could answer
       "what was the actuator being asked for".
         wRel      relative angular rate about the hinge axis, rad/s. The damping half of the
                   servo is kd*wRel, and kd alone reaches the ceiling at 21-113 rad/s
                   depending on the joint, so this term can BE the entire actuator output.
         tauDemand the raw PD+feedforward demand BEFORE the tauMax clamp, N.m. satFrac says a
                   joint was railed; only this says whether it wanted 1.05x its ceiling or 40x,
                   and those have opposite remedies (raise the ceiling vs the command is wrong). */
    this.wRel = 0; this.tauDemand = 0;
    /* ACTUATOR GOVERNOR. Ceiling on how fast the OUTPUT TORQUE may change, N.m/s -- distinct from
       tauMax, which caps how hard it can push. Infinity = ungoverned, the old behaviour.
       Real hardware has this: current cannot step through an inductance and a gearbox has windup.
       The gyro has modelled it since MK1.8 (`slewSteps`, full authority in stepPeriod/slewSteps)
       and the joints did not, so every actuator on the machine could go from nothing to its
       ceiling and back inside one substep -- which is what the measured chatter is.
       SIGNED, deliberately. Limiting |tau| would let the torque flip +tauMax -> -tauMax freely,
       since the magnitude never changes; a reversal is exactly the thing being suppressed. */
    this.tauRate = o.tauRate ?? Infinity;
    this.tauHeld = 0;                  // torque actually delivered last substep, N.m
    this.governed = false;             // the rate cap bound this substep, not the torque ceiling
    /* Per-FRAME substep extremes, filled by World.step. A log samples once per frame at best,
       and every per-joint field used to be the value from ONE substep -- 1 in 232 of those
       actually executed between samples. A snapshot of a bang-banging actuator is noise; the
       envelope over the substeps is not, and it is honest at any sampling rate. */
    this.tauPeak = 0; this.demandPeak = 0; this.ratePeak = 0; this.limitPeak = 0;
    this.enabled = true;
    this.tauFF = 0;                     // feedforward torque added to the servo, N.m
    this.lim = Object.assign({}, NO_LIMIT, o.lim);
    this.broken = false; this.util = 0; this.peakUtil = 0;
    this.F = V(); this.T = V(); this.Fax = 0; this.Fsh = 0; this.Mb = 0; this.Mt = 0;
  }
  reset() { this.lm = 0; this.lp = V(); this.la = V(); this.lLim = 0; }
  currentAngle() {
    const ax = this.a.dirWorld(this.axisA);
    const r1 = this.a.dirWorld(this.refA), r2 = this.b.dirWorld(this.refB);
    const p1 = vnorm(vsub(r1, vmul(ax, vdot(r1, ax))));
    const p2 = vnorm(vsub(r2, vmul(ax, vdot(r2, ax))));
    return Math.atan2(vdot(vcross(p1, p2), ax), vdot(p1, p2));
  }
  solve(h) {
    const { a, b } = this;
    // 1. axis alignment (removes 2 rotational DOF)
    const ax = a.dirWorld(this.axisA), bx = b.dirWorld(this.axisB);
    const dtheta = vcross(ax, bx);
    const ra = solveAngular(a, b, dtheta, this.alignCompliance, h, 0);
    if (ra.dl !== 0) this.la = vadd(this.la, vmul(ra.n, ra.dl));
    // 2. actuator about the hinge axis (torque-limited = clamped multiplier)
    this.angle = this.currentAngle();
    if (this.enabled && this.tauMax > 0) {
      const axis = a.dirWorld(this.axisA);
      const maxL = this.tauMax * h * h;
      // A real PD actuator with FINITE stiffness and damping, not a rigid constraint. A
      // near-zero-compliance servo has effectively infinite gain, so millirad-level noise
      // in a control signal becomes kN.m of oscillating torque.
      // (There was a second `mode: 'torque'` branch here commanding tauCmd directly. No
      // construction site ever passed `mode` and nothing ever wrote a Hinge's tauCmd, so it
      // was unreachable, and its body was this one line for line after the command term.)
      let e = this.target - this.angle;
      e = Math.atan2(Math.sin(e), Math.cos(e));
      const wRel = vdot(vsub(b.w, a.w), axis);
      // Feedforward rides ON TOP of the servo. A balance loop that replaces the PD term
      // throws away the passive stiffness holding the posture up.
      const tauPD = this.kp * e - this.kd * wRel + this.tauFF;
      this.wRel = wRel; this.tauDemand = tauPD;      // telemetry only; see the ctor
      /* Governor, before the ceiling. tauHeld is what was actually delivered last substep and is
         updated once per substep in measure(), so the cap is the same for all 8 iterations --
         slewing per iteration would let it move 8x faster than the rate says. */
      let tauCmd = tauPD;
      if (this.tauRate !== Infinity) {
        const dmax = this.tauRate * h;
        const want = tauPD - this.tauHeld;
        tauCmd = this.tauHeld + Math.max(-dmax, Math.min(dmax, want));
        this.governed = Math.abs(want) > dmax;
      }
      const tot = Math.max(-maxL, Math.min(maxL, tauCmd * h * h));
      const dl = tot - this.lm;
      this.lm = tot;
      const p = vmul(axis, dl);
      if (b.invMass) rotateBy(b, applyInvI(b, p), +1);
      if (a.invMass) rotateBy(a, applyInvI(a, p), -1);
      this.saturated = Math.abs(tauPD) > this.tauMax;
    }
    // 3. joint angle limits. A mechanical end stop is a real structure: it has finite
    // stiffness and whatever torque it carries is reacted by the same mount the actuator
    // hangs off. Previously this was solved rigidly with an unbounded multiplier whose
    // return value was DISCARDED, so a stop could pass full actuator authority (measured
    // at 40.6 kN.m on the right ankle, held for 549 substeps) while reading zero in
    // telemetry and contributing nothing to the failure envelope.
    if (this.limits) {
      const ang = this.currentAngle();
      let viol = 0;
      if (ang < this.limits[0]) viol = ang - this.limits[0];
      else if (ang > this.limits[1]) viol = ang - this.limits[1];
      if (viol !== 0) {
        const rl = solveAngular(a, b, vmul(a.dirWorld(this.axisA), viol),
                                this.limitCompliance, h, this.lLim);
        this.lLim += rl.dl * (viol < 0 ? -1 : 1);
      }
    }
    // 4. anchor
    const r1 = a.dirWorld(this.ra), r2 = b.dirWorld(this.rb);
    const err = vsub(vadd(b.x, r2), vadd(a.x, r1));
    const rp = solvePositional(a, b, r1, r2, err, this.compliance, h, 0);
    if (rp.dl !== 0) this.lp = vadd(this.lp, vmul(rp.n, rp.dl));
  }
  measure(h) {
    const inv = 1 / (h * h);
    this.tau = this.lm * inv;
    // Governor state, updated ONCE per substep (measure runs after the iteration loop). Reading
    // the DELIVERED torque rather than the commanded one keeps the cap from winding up when the
    // joint is also against tauMax or an end stop.
    this.tauHeld = this.tau;
    this.F = vmul(this.lp, inv);
    this.T = vmul(this.la, inv);
    const n = this.a.dirWorld(this.axisA);
    this.Fax = vdot(this.F, n);
    this.Fsh = vlen(vsub(this.F, vmul(n, this.Fax)));
    this.tauLimit = this.lLim * inv;
    this.onStop = Math.abs(this.tauLimit) > 1e-4 * Math.max(1e-9, this.tauMax);
    this.Mb = vlen(this.T);                 // alignment constraint is perpendicular to the axis
    // Both the actuator and the end stop act about the hinge axis, and the mount reacts
    // their sum. Omitting the stop term hid a full-authority load path from the envelope.
    this.Mt = Math.abs(this.tau + this.tauLimit);
    this.util = structuralUtil(this.lim, this.Fax, this.Fsh, this.Mb, this.Mt);
    if (this.util > this.peakUtil) this.peakUtil = this.util;
    if (this.util >= 1) { this.broken = true; this.brokeAtUtil = this.util; }
  }
}

/* ---------------- Contacts: box corners vs ground plane y = 0 ---------------- */
class GroundContacts {
  constructor(o = {}) {
    this.lscale = o.lscale ?? 1;
    this.mu = o.mu ?? 0.9;
    this.compliance = o.compliance ?? 0;
    this.active = [];
  }
  /* Collected once per SUBSTEP with a speculative margin that scales with how far the
     corner can travel in one substep. Collecting per frame lets a fast corner tunnel
     through the plane between collections and then take a huge push-out next frame. */
  collect(bodies, h) {
    this.active.length = 0;
    for (const b of bodies) {
      if (!b.half || b.invMass === 0) continue;
      const hx = b.half.x, hy = b.half.y, hz = b.half.z;
      const margin = 0.01 * (this.lscale || 1) + 2 * (Math.abs(b.v.y) + vlen(b.w) * vlen(b.half)) * h;
      for (let i = 0; i < 8; i++) {
        const lp = V((i & 1 ? hx : -hx), (i & 2 ? hy : -hy), (i & 4 ? hz : -hz));
        const wp = b.toWorld(lp);
        if (wp.y < margin) this.active.push({ b, lp, lambda: 0, lambdaT: 0 });
      }
    }
  }
  solve(h) {
    for (const c of this.active) {
      const b = c.b;
      const wp = b.toWorld(c.lp);
      const depth = -wp.y;
      const r = qrot(b.q, c.lp);
      const n = V(0, 1, 0);
      const w = genInvMass(b, r, n);
      if (w < 1e-18) continue;
      const a = this.compliance / (h * h);
      // In XPBD lambda IS the accumulated contact impulse, and it has to persist across
      // solver iterations -- it is the quantity the force telemetry divides by h^2.
      // Zeroing it the moment the constraint is satisfied throws the support impulse
      // away and leaves only the last iteration's residual push-out, which measured
      // 53 kN against an 80 kN rig while standing still. Relax toward zero instead, and
      // clamp the TOTAL at zero so the contact stays unilateral (it can push, not pull).
      let dl = (depth - a * c.lambda) / (w + a);
      if (c.lambda + dl < 0) dl = -c.lambda;
      if (dl !== 0) {
        c.lambda += dl;
        const p = vmul(n, dl);
        b.x = vadd(b.x, vmul(p, b.invMass));
        rotateBy(b, applyInvI(b, vcross(r, p)), +1);
      }
      if (c.lambda <= 0) { c.lambdaT = 0; continue; }   // separated: no friction to apply
      // Static friction resists motion of the contact point relative to where it sat at
      // the START of the substep (pre-integration pose), NOT relative to a mid-solve
      // snapshot -- measuring from mid-solve makes friction fight the normal push-out
      // and inject energy.
      const wpNew = b.toWorld(c.lp);
      const wpPrev = vadd(b.xp, qrot(b.qp, c.lp));
      const d = vsub(wpNew, wpPrev);
      const dt = V(d.x, 0, d.z);
      const dtl = vlen(dt);
      if (dtl > 1e-9 * this.lscale) {
        const tn = vmul(dt, -1 / dtl);
        const r2 = qrot(b.q, c.lp);
        const wt = genInvMass(b, r2, tn);
        if (wt > 1e-18) {
          let dlt = dtl / wt;
          const maxT = this.mu * c.lambda;
          if (c.lambdaT + dlt > maxT) dlt = Math.max(0, maxT - c.lambdaT);
          if (dlt > 0) {
            c.lambdaT += dlt;
            const pt = vmul(tn, dlt);
            b.x = vadd(b.x, vmul(pt, b.invMass));
            rotateBy(b, applyInvI(b, vcross(r2, pt)), +1);
          }
        }
      }
    }
  }
  normalForce(h) { let s = 0; for (const c of this.active) s += c.lambda / (h * h); return s; }
}

/* ---------------- Pair non-penetration ----------------
   The solver has ground contact only, so two bodies could pass straight through each
   other. Feet did exactly that. This is a separating-axis test on the six face normals
   of two boxes; for bodies that stay near-level (which feet do) the nine edge-edge axes
   contribute nothing, so they are skipped.

   The correction is applied through both centres with no torque. That is not full
   contact dynamics -- a real contact acts at the overlap region and would spin the
   bodies -- but it makes interpenetration impossible without introducing a new source
   of angular energy into a gait that is already sensitive. */
class PairCollision {
  constructor(o) {
    this.a = o.a; this.b = o.b;
    this.compliance = o.compliance ?? 0;
    this.margin = o.margin ?? 0;         // keep this much clear air between them
    this.depth = 0;
  }
  reset() { this.lam = 0; }
  axes() {
    const A = quatToM3(this.a.q), B = quatToM3(this.b.q);
    return [
      V(A[0], A[3], A[6]), V(A[1], A[4], A[7]), V(A[2], A[5], A[8]),
      V(B[0], B[3], B[6]), V(B[1], B[4], B[7]), V(B[2], B[5], B[8]),
    ];
  }
  extent(body, n) {
    const M = quatToM3(body.q), h = body.half;
    return Math.abs(n.x * M[0] + n.y * M[3] + n.z * M[6]) * h.x
         + Math.abs(n.x * M[1] + n.y * M[4] + n.z * M[7]) * h.y
         + Math.abs(n.x * M[2] + n.y * M[5] + n.z * M[8]) * h.z;
  }
  solve(h) {
    const { a, b } = this;
    if (!a.half || !b.half) return;
    const d = vsub(b.x, a.x);
    let best = Infinity, axis = null;
    for (const n of this.axes()) {
      const ov = this.extent(a, n) + this.extent(b, n) + this.margin - Math.abs(vdot(d, n));
      if (ov <= 0) { this.depth = 0; return; }        // separated on this axis: done
      if (ov < best) { best = ov; axis = vdot(d, n) < 0 ? vmul(n, -1) : n; }
    }
    this.depth = best;
    const w1 = a.invMass, w2 = b.invMass, ws = w1 + w2;
    if (ws < 1e-18) return;
    const al = this.compliance / (h * h);
    const dl = (best - al * this.lam) / (ws + al);
    this.lam += dl;
    const p = vmul(axis, dl);
    if (w1) a.x = vsub(a.x, vmul(p, w1));
    if (w2) b.x = vadd(b.x, vmul(p, w2));
  }
}

/* ---------------- World ---------------- */
class World {
  constructor(o = {}) {
    this.g = o.gravity || V(0, -9.81, 0);
    this.bodies = [];
    this.joints = [];
    this.welds = [];
    this.pairs = [];
    this.contacts = new GroundContacts(o.contact || {});
    Object.defineProperty(this, 'lscale', { set(v){ this.contacts.lscale = v; }, get(){ return this.contacts.lscale; } });
    this.substeps = o.substeps ?? 20;
    this.iterations = o.iterations ?? 1;
    this.time = 0;
    this.breakEvents = [];
    this.enableGround = o.enableGround ?? true;
  }
  add(b) { this.bodies.push(b); return b; }
  addJoint(j) { this.joints.push(j); return j; }
  addWeld(w) { this.welds.push(w); return w; }
  addPair(p) { this.pairs.push(p); return p; }

  step(dt) {
    const n = this.substeps, h = dt / n;
    /* SENSOR ACCUMULATORS. contactForce below is a frame MEAN, and a frame mean cannot see
       a servo limit-cycling inside the substep: standing still, the instantaneous load on a
       foot swings 0.00-3.28 W while the mean reads 1.00. Every "the foot is loaded" gate in
       this project has been blind to that. _cMin/_cMax carry the substep extremes out with
       the mean, _cTan gives the friction-cone utilisation that predicts a slide before it
       happens, and _cSlip measures the slide itself. Instrumentation only -- nothing here
       feeds a control law yet. */
    for (const b of this.bodies) {
      b._cImp = 0; b._cCop = V(); b._cTan = 0; b._cSlip = 0;
      b._cMin = Infinity; b._cMax = 0; b._cN = 0;
    }
    /* JOINT ACCUMULATORS, the same argument as the contact ones above. Every per-joint number
       a log could carry was the last substep's, so 231 of every 232 executed substeps were
       observed by nothing and a railed actuator was indistinguishable from a quiet one that
       happened to be sampled mid-swing. Carry the substep extremes out with the frame. */
    for (const j of this.joints) {
      j._satN = 0; j._govN = 0; j._tauPk = 0; j._demPk = 0; j._wRelPk = 0; j._limPk = 0;
    }
    for (let s = 0; s < n; s++) {
      if (this.enableGround) this.contacts.collect(this.bodies, h);
      for (const b of this.bodies) integrate(b, h, this.g);
      for (const j of this.joints) j.reset();
      for (const w of this.welds) w.reset();
      for (const p of this.pairs) p.reset();
      for (let it = 0; it < this.iterations; it++) {
        for (const j of this.joints) if (!j.broken) j.solve(h);
        for (const w of this.welds) w.solve(h);
        for (const p of this.pairs) p.solve(h);
        if (this.enableGround) this.contacts.solve(h);
      }
      for (const b of this.bodies) b._cSub = 0;
      for (const c of this.contacts.active) {
        if (c.lambda > 0) {
          c.b._cImp += c.lambda;
          c.b._cSub += c.lambda;
          c.b._cCop = vadd(c.b._cCop, vmul(c.b.toWorld(c.lp), c.lambda));
          c.b._cTan += c.lambdaT;
        }
      }
      for (const b of this.bodies) {
        if (b._cSub <= 0) continue;
        if (b._cSub < b._cMin) b._cMin = b._cSub;
        if (b._cSub > b._cMax) b._cMax = b._cSub;
        b._cN++;
      }
      for (const b of this.bodies) updateVel(b, h);
      /* Slip is read AFTER updateVel, from the contact point's post-solve velocity: it is
         what the foot is actually doing on the ground, not what the solver asked for.
         Impulse-weighted so a barely-loaded contact cannot dominate the average. */
      for (const c of this.contacts.active) {
        if (c.lambda <= 0) continue;
        const rc = qrot(c.b.q, c.lp);
        const vp = vadd(c.b.v, vcross(c.b.w, rc));
        c.b._cSlip += Math.sqrt(vp.x * vp.x + vp.z * vp.z) * c.lambda;
      }
      for (const j of this.joints) {
        if (j.broken) continue;
        j.measure(h);
        if (j.saturated) j._satN++;
        if (j.governed) j._govN++;
        const aTau = Math.abs(j.tau);            if (aTau > j._tauPk) j._tauPk = aTau;
        const aDem = Math.abs(j.tauDemand);      if (aDem > j._demPk) j._demPk = aDem;
        const aRate = Math.abs(j.wRel);          if (aRate > j._wRelPk) j._wRelPk = aRate;
        const aLim = Math.abs(j.tauLimit);       if (aLim > j._limPk) j._limPk = aLim;
        if (j.broken) {
          this.breakEvents.push({ t: this.time + (s + 1) * h, joint: j.name, util: j.brokeAtUtil, F: j.F, T: j.T });
        }
      }
      for (const w of this.welds) {
        w.measure(h);
        if (w.broken && !w.reported) {
          w.reported = true;
          this.breakEvents.push({ t: this.time + (s + 1) * h, weld: w.name, util: w.brokeAtUtil, F: w.F, T: w.T });
        }
      }
      this.time += h;
    }
    // frame-averaged contact load: a single substep snapshot is far too noisy to
    // feed a controller
    for (const b of this.bodies) {
      b.contactForce = b._cImp / (n * h * h);
      b.contactCop = b._cImp > 0 ? vmul(b._cCop, 1 / b._cImp) : null;
      /* ...and the substep extremes alongside it, because the mean is what hid the
         limit cycle. contactForceMin is over substeps that HAD contact, so a swing foot
         reads 0 rather than dragging the minimum of a stance foot to zero. */
      b.contactForceMin = b._cN ? b._cMin / (h * h) : 0;
      b.contactForceMax = b._cN ? b._cMax / (h * h) : 0;
      // 0 = no tangential demand, 1 = riding the friction cone and about to slide.
      b.contactCone = b._cImp > 0 ? b._cTan / (this.contacts.mu * b._cImp) : 0;
      b.contactSlip = b._cImp > 0 ? b._cSlip / b._cImp : 0;   // m/s
    }
    // Fraction of substeps this joint spent against its torque ceiling. A joint that is
    // merely working hard reads low; one that is bang-banging reads near 1.
    for (const j of this.joints) {
      j.satFrac = j._satN / n;
      j.govFrac = j._govN / n;
      j.tauPeak = j._tauPk; j.demandPeak = j._demPk;
      j.ratePeak = j._wRelPk; j.limitPeak = j._limPk;
    }
  }
}

function integrate(b, h, g) {
  if (b.invMass === 0) { b.xp = b.x; b.qp = b.q; return; }
  b.v = vadd(b.v, vmul(vadd(g, vmul(b.fExt, b.invMass)), h));
  b.xp = b.x;
  b.x = vadd(b.x, vmul(b.v, h));
  // implicit gyroscopic term (one Newton step), then external torque
  let wl = qrotInv(b.q, b.w);
  const Iw = m3mulv(b.I, wl);
  /* SPINNING ROTOR. A flywheel bolted into this body carries angular momentum that is not
     I*omega, so Euler's equation is dL/dt + omega x L = tau with L = I*omega + h_rotor,
     not L = I*omega. Adding h_rotor to the momentum HERE rather than as an external torque
     is the whole point: this branch is solved implicitly in omega, and the same term
     applied explicitly through tExt is unconditionally unstable.
     It was shipped that way once. tau = -omega x h does zero work, which was verified
     exactly, and it still detonated: explicit Euler on a precession term does not precess,
     it spirals, growing by sqrt(1 + (h*dt/I)^2) every substep. Measured at MK1.7.0 the
     Light Frame ran h*dt/I = 0.89, which is 3.8e305 per second, and the passive torque hit
     173 840% of the actuator ceiling in 0.7 s while the active loop sat at 4-9%.
     Same defect as the servo damping term, which is also explicit. Checking that a term
     conserves energy says nothing about whether its DISCRETISATION does.
     Spin axis is the body's own +Y. */
  const Lw = b.hRotor ? vadd(Iw, V(0, b.hRotor, 0)) : Iw;
  const resid = vmul(vcross(wl, Lw), h);
  const J = m3add(b.I, m3scale(m3sub(m3mul(skew(wl), b.I), skew(Lw)), h));
  wl = vsub(wl, m3mulv(m3inv(J), resid));
  const tl = qrotInv(b.q, b.tExt);
  wl = vadd(wl, vmul(m3mulv(b.invI, tl), h));
  b.w = qrot(b.q, wl);
  b.qp = b.q;
  b.q = qnorm(qmul(qExp(vmul(b.w, h)), b.q));
}
function updateVel(b, h) {
  if (b.invMass === 0) return;
  b.v = vmul(vsub(b.x, b.xp), 1 / h);
  let dq = qmul(b.q, qconj(b.qp));
  if (dq.w < 0) dq = { x: -dq.x, y: -dq.y, z: -dq.z, w: -dq.w };
  const sv = Math.hypot(dq.x, dq.y, dq.z);          // exact log map
  if (sv < 1e-12) { b.w = V(); return; }
  const ang = 2 * Math.atan2(sv, dq.w);
  b.w = vmul(V(dq.x, dq.y, dq.z), ang / (sv * h));
}
