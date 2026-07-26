/* ===== control/dcm.mjs ===== */
// dcm.mjs — divergent component of motion planning.
//
// The linear inverted pendulum splits into a stable part (the COM chasing the DCM) and
// an unstable part (the DCM), where
//     xi = x + xdot / omega,      omega = sqrt(g / z_com)
//     xidot = omega * (xi - p)    p = ZMP
// With the ZMP held constant over a phase this integrates exactly:
//     xi(t) = p + (xi_0 - p) * e^(omega t)
// so planning runs BACKWARD from a desired final rest state:
//     xi_i = p_i + (xi_{i+1} - p_i) * e^(-omega T_i)
// Every trajectory produced this way keeps the ZMP inside the phase's support by
// construction, which is exactly the property the hand-built pelvis pattern lacked.


class DCMPlan {
  /* phases: [{ zmpA: V, zmpB: V, duration, kind }] with the ZMP moving LINEARLY from
     zmpA to zmpB across the phase. Holding it constant per phase makes it jump 0.28 m
     at every transition, which no real centre of pressure can do.

     For p(t) = pA + c t, with c = (pB - pA)/T, the DCM integrates exactly to
        xi(t) = e^(w t) (xi_0 - pA - c/w) + pA + c t + c/w
     and the backward recursion is
        xi_0 = pA + c/w + e^(-w T) (xi_T - pB - c/w) */
  constructor(phases, zCom, g = 9.81) {
    this.phases = phases;
    this.omega = Math.sqrt(g / zCom);
    this.zCom = zCom;
    let t = 0;
    this.t0 = phases.map((p) => { const s = t; t += p.duration; return s; });
    this.T = t;
    const w = this.omega;
    this.xi = new Array(phases.length + 1);
    this.xi[phases.length] = phases[phases.length - 1].zmpB;
    for (let i = phases.length - 1; i >= 0; i--) {
      const p = phases[i];
      const c = vmul(vsub(p.zmpB, p.zmpA), 1 / p.duration);
      const e = Math.exp(-w * p.duration);
      const inner = vsub(vsub(this.xi[i + 1], p.zmpB), vmul(c, 1 / w));
      this.xi[i] = vadd(vadd(p.zmpA, vmul(c, 1 / w)), vmul(inner, e));
    }
  }
  indexAt(t) {
    if (t <= 0) return 0;
    for (let i = this.phases.length - 1; i >= 0; i--) if (t >= this.t0[i]) return i;
    return 0;
  }
  localAt(t) {
    const i = this.indexAt(t);
    const p = this.phases[i];
    const tau = Math.min(Math.max(t - this.t0[i], 0), p.duration);
    return { i, p, tau, c: vmul(vsub(p.zmpB, p.zmpA), 1 / p.duration) };
  }
  zmpAt(t) {
    const { p, tau, c } = this.localAt(t);
    return vadd(p.zmpA, vmul(c, tau));
  }
  kindAt(t) { return this.phases[this.indexAt(t)].kind; }
  phaseProgress(t) {
    const { i, p, tau } = this.localAt(t);
    return { i, s: tau / p.duration, kind: p.kind };
  }
  xiAt(t) {
    const { i, p, tau, c } = this.localAt(t);
    const w = this.omega;
    const inner = vsub(vsub(this.xi[i], p.zmpA), vmul(c, 1 / w));
    const e = Math.exp(w * tau);
    return vadd(vadd(vadd(vmul(inner, e), p.zmpA), vmul(c, tau)), vmul(c, 1 / w));
  }
}

/* Forward-integrates the stable COM dynamics xdot = -omega (x - xi) against a plan.
   Produces the pelvis reference that the IK layer tracks. */
class COMTracker {
  constructor(plan, x0) {
    this.plan = plan;
    this.x = V(x0.x, 0, x0.z);
    this.v = V();
  }
  step(t, dt) {
    const xi = this.plan.xiAt(t);
    const w = this.plan.omega;
    // semi-implicit: stable direction, so this is unconditionally well behaved
    const ax = w * (xi.x - this.x.x), az = w * (xi.z - this.x.z);
    this.v = V(ax, 0, az);
    this.x = V(this.x.x + ax * dt, 0, this.x.z + az * dt);
    return { com: this.x, vel: this.v, xi };
  }
}

/* Build an alternating footstep plan. stride = 0 marches in place. */
function buildPhases(opts) {
  const {
    left, right,
    stride = V(), nSteps = 6,
    tDS = 0.45, tSS = 0.85, tStart = 1.2, tEnd = 2.5,
    first = 'L',
  } = opts;
  const sv = typeof stride === 'number' ? V(stride, 0, 0) : stride;
  const flat = (p) => V(p.x, 0, p.z);
  const mid = (a, b) => V((a.x + b.x) / 2, 0, (a.z + b.z) / 2);
  const foot = { L: flat(left), R: flat(right) };
  const centre = mid(foot.L, foot.R);
  const phases = [{ zmpA: centre, zmpB: centre, duration: tStart, kind: 'DS' }];
  let swing = first;
  let prev = centre;
  for (let i = 0; i < nSteps; i++) {
    const stance = swing === 'L' ? 'R' : 'L';
    // shift the ZMP off the previous point onto the stance foot, then hold it there
    phases.push({ zmpA: prev, zmpB: foot[stance], duration: tDS, kind: `DS->${stance}` });
    phases.push({ zmpA: foot[stance], zmpB: foot[stance], duration: tSS, kind: `SS-${stance}` });
    foot[swing] = V(foot[swing].x + sv.x, 0, foot[swing].z + sv.z);
    prev = foot[stance];
    swing = swing === 'L' ? 'R' : 'L';
  }
  const end = mid(foot.L, foot.R);
  phases.push({ zmpA: prev, zmpB: end, duration: tDS, kind: 'DS' });
  phases.push({ zmpA: end, zmpB: end, duration: tEnd, kind: 'DS' });
  return phases;
}
