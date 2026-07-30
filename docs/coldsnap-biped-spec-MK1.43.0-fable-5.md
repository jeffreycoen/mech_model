# Biped Mech Walking — plug-in spec + reference code for Coldsnap
### Distilled from mech_model Light Frame, MK1.43.0 (Claude Fable 5) — v4: hinge/island (§6), ground-vs-Baumgarte settled (§4), headless gate (§7)

Engine-agnostic JavaScript. Three drop-in modules: **scale**, **gait**, **servo** — plus
the contact recipe and the two design constraints code can't fix. Everything derives from
leg length `L`, gravity `g`, and mass. No absolute constants anywhere: that is what makes
it scale.

Your engine supplies: rigid bodies, hinge joints with applied torque, ground contacts,
and leg IK (pelvis pose + foot pose → joint angles). Everything else is below.

---

## 0. Design constraints code cannot fix

1. **Hull ≥ ~60% of total mass, legs light.** The planner is a point-mass pendulum; on a
   leg-heavy rig (ours: 38% hull) no tuning saved it — every swing shoves the CoM in
   ways the plan can't see.
2. **No stiff position servos on passive appendages** (arms, pods). They resonate and
   tear off. Tune them as dampers (§5) or weld them.

---

## 1. Scale module — build-time, once per mech size

```js
// Froude scaling. s = legLength / referenceLegLength of the tuned reference rig.
// A walk scaled this way is dynamically identical at any size.
function froudeScale(rig, s) {
  const s3 = s*s*s, s4 = s3*s, s5 = s4*s, rs = Math.sqrt(s);
  for (const b of rig.bodies) {
    b.pos    = mulVec(b.pos, s);
    b.dims   = mulVec(b.dims, s);
    b.mass  *= s3;
    b.inertia = b.inertia.map(v => v * s5);
  }
  for (const j of rig.joints) {
    j.anchorA = mulVec(j.anchorA, s); j.anchorB = mulVec(j.anchorB, s);
    j.tauMax *= s4;            // torque ceiling
    j.kp     *= s4;            // servo stiffness (N·m/rad)
    j.kd     *= s4 * rs;       // linear damping (N·m per rad/s)
    j.kv     *= s5;            // quadratic damping (N·m per (rad/s)^2)
  }
  rig.groundCompliance /= s*s; // m/N  (see §4)
  // THE CLASSIC PORTING FAILURE lives here: the physics/controller timestep MUST
  // scale too. Small machine = faster clock. Pin dt to the display rate and small
  // mechs collapse while big ones work.
  rig.dt = rig.dtReference * rs;
}
```

```js
// Everything the controller needs, derived from geometry. Call after scaling.
// L = thigh+shin length, comH = standing CoM height, halfStance = half the
// lateral distance between foot centers, foot = {halfLen, halfWid, ankleOffX}.
function deriveGait(L, comH, halfStance, foot, g = 9.81) {
  const omega = Math.sqrt(g / comH);          // pendulum frequency, rad/s
  const AG = 0.7;                             // agility, 0.5..1.0; 0.7 measured best
  const tSS = 1.66 * AG / omega;              // single-support (swing) time
  const tDS = 0.92 * AG / omega;              // double-support time
  return {
    omega, tSS, tDS,
    stepPeriod : tSS + tDS,
    strideCap  : 0.28 * L,                    // max commanded step length
    stepHeight : 0.08 * L,                    // swing apex (see §3 profile)
    pelvisDrop : 0.085 * L,                   // walking crouch below stand height
    // Ankle authority box: how far the CoP may be pushed under one foot.
    copLimitX  : 0.80 * (foot.halfLen - foot.ankleOffX),
    copLimitZ  : 0.65 * foot.halfWid,
    copClamp   : 0.45 * (L / 2.95),           // max CoP correction vs plan, m (ref leg 2.95)
    halfStance,
    minFootSep : Math.min(2*foot.halfWid + 0.06*(L/2.95), 1.9*halfStance), // never cross
    splayMax   : 1.40,                        // max separation / nominal stance
    yawPerStep : 8 * Math.PI/180,             // 5° for tall/narrow rigs
    turnRate   : (8 * Math.PI/180) / (tSS + tDS), // DERIVED — command can't outrun legs
    travelRate : 0.6 * Math.sqrt(L / 2.95),   // stick-command slew, m/s per s
    kDCM       : 2.0,                         // ankle CoP feedback gain (>1 = stable)
    kCapture   : 1.0,                         // foothold feedback; 0.65 if legs heavy
    capCommit  : 0.5,                         // freeze catch target at 50% of swing
    capDeadband: 0.15,                        // ignore ξ error < 15% of strideCap
    restExt    : 0.93,                        // standing leg extension (IK clamp 0.995)
  };
}
```

---

## 2. Gait module — the balance core, one call per tick

The full state machine (double/single support alternation, close-step on stop, squaring)
is ordinary game code; below are the parts that make it *balance*. `k` is `deriveGait()`.

```js
// Capture point: where the machine stops if it plants a foot there. THE quantity.
// com/comVel are FULL-BODY (all links), not hull-only. omega from live CoM height.
function capturePoint(com, comVel, g) {
  const om = Math.sqrt(g / Math.max(1e-4, com.y));
  return { x: com.x + comVel.x / om, z: com.z + comVel.z / om };
}
```

```js
// Swing-foot target for the current step. Called every tick during swing.
//   s        : swing phase 0..1
//   from     : this foot's lift-off print   nom : planned foothold (plan §2b)
//   xiErr    : capturePoint(now) - capturePoint(planned), world x/z
//   hold     : per-step state object ({} at each lift-off)
function swingTarget(s, from, nom, xiErr, hold, k) {
  // Capture feedback: step INTO the fall — but COMMIT at mid-swing. Chasing the
  // live error to touchdown lands the foot moving sideways and recovery becomes
  // a growing limit cycle (measured; the single worst failure mode we hit).
  if (s < k.capCommit || !hold.cap) {
    let cx = k.kCapture * xiErr.x, cz = k.kCapture * xiErr.z;
    const m = Math.hypot(cx, cz), db = k.capDeadband * k.strideCap;
    const sc = m > 1e-9 ? Math.max(0, m - db) / m : 0;   // soft deadband
    cx *= sc; cz *= sc;
    const cm = Math.hypot(cx, cz);
    if (cm > k.strideCap) { cx *= k.strideCap/cm; cz *= k.strideCap/cm; }
    hold.cap = { x: cx, z: cz };
  }
  let tx = nom.x + hold.cap.x, tz = nom.z + hold.cap.z;
  // ...then clamp (tx,tz) against the stance foot: separation >= k.minFootSep,
  // separation <= k.splayMax * 2*k.halfStance. Clamps run AFTER the correction.
  const e = smoothstep(s);
  return { x: from.x + (tx - from.x) * e,
           y: swingLift(s, k.stepHeight),      // §3
           z: from.z + (tz - from.z) * e };
}
```

```js
// Ankle CoP trim (stance foot torque), event-triggered to kill chatter.
// zmpRef comes from the plan; xiErr as above; hold persists across ticks.
function copCommand(zmpRef, xiErr, hold, k) {
  // Re-decide only when the error MOVED since last decision. Never gates a big
  // error (it re-decides immediately); only suppresses per-tick dither.
  if (!hold.cop || Math.hypot(xiErr.x - hold.cop.x, xiErr.z - hold.cop.z) > 0.25 * k.copClamp)
    hold.cop = { x: xiErr.x, z: xiErr.z };
  return {
    x: zmpRef.x + clamp(k.kDCM * hold.cop.x, -k.copClamp, k.copClamp),
    z: zmpRef.z + clamp(k.kDCM * hold.cop.z, -k.copClamp, k.copClamp),
  };
  // Ankle pitch torque ≈ -kCop * stanceLoad * (copX - footX), roll likewise,
  // kCop = 0.40, only while that foot is loaded. Clamp inside the physical sole.
}
```

```js
// Catch step FROM STANDING. Stepping must be available in every state.
// Fire when ξ leaves what the ankles can reach; foot on the side of the fall.
// NO time-based cooldown between catches — tried, it gated real recoveries and
// the machine toppled while "settling". The deadband is the only gate.
function standCatch(xi, feetMid, bodyAxes, k) {
  const eF = dot2(sub2(xi, feetMid), bodyAxes.fwd);
  const eL = dot2(sub2(xi, feetMid), bodyAxes.left);
  if (Math.abs(eF) > k.copLimitX || Math.abs(eL) > k.copLimitZ + k.halfStance)
    return { side: eL > 0 ? 'L' : 'R' };     // -> launch one step, swingTarget places it
  return null;
}
```

```js
// Feedforward stop: on stick release, decide ONCE where the CoM is going and
// land the closing step there. No live chasing during the stopping swing.
function planStop(com, comVel, feetMid, k, g) {
  const xi = capturePoint(com, comVel, g);
  let ax = k.kCapture * (xi.x - feetMid.x), az = k.kCapture * (xi.z - feetMid.z);
  const m = Math.hypot(ax, az);
  if (m > k.strideCap) { ax *= k.strideCap/m; az *= k.strideCap/m; }
  return { x: ax, z: az };   // offset for the closing step's nominal print
}
```

```js
// Cadence ramp — crane rule. First plan out of a stand runs 1.35x-long phases,
// decaying 60% of the excess per step. Stopping ramps out via the plan's tail.
function rampedTimes(k, rampK) { return { tSS: k.tSS * rampK, tDS: k.tDS * rampK }; }
// rampK: 1.35 on leaving STAND; after each replan rampK = 1 + (rampK-1)*0.6.
```

**Plan (§2b), in one paragraph.** Keep footprints for N=4 steps ahead: alternate feet,
each print = previous print + commanded stride (clamped to `strideCap`), lateral at
nominal stance width. Replan at **every touchdown from the measured landing** — feet land
short and the error compounds if you trust commands. Between prints, run the analytic
LIPM/DCM reference for CoM and ZMP (any textbook DCM implementation; ~80 lines) — or, at
minimum viable: move the pelvis reference at the commanded velocity and let ξ feedback do
the work. Every command channel — travel, heading, aim — is slewed (`travelRate`,
`turnRate`); a raw position step anywhere becomes a structural impact somewhere.

---

## 3. Swing profile + fall handling

```js
// sin^2: zero slope at BOTH ends — the foot lifts smoothly and lands at zero
// commanded vertical speed. Plain sin lands at MAX downward speed: shuffle-slam,
// 2-3x bodyweight contact spikes. One line; do not improvise here.
function swingLift(s, h) { const p = Math.sin(Math.PI * s); return p * p * h; }
```

```js
// Fall = limp. The instant attitude is lost (up·worldUp < 0.6 or pelvis dropped
// 30% of standing height): STOP the walk controller and pin every servo target
// to its current angle — damping keeps acting, stiffness holds nothing.
// A walk controller driving a fallen machine saturates every joint and tears mounts.
function onFall(rig) {
  for (const j of rig.joints) {
    j.target = j.angle;
    if (j.kp === 0) j.kd = Math.min(j.kd * 8, 0.9 * j.childInertia / rig.hSub); // arms
  }
}
```

---

## 4. Contact + servo laws

```js
// Joint servo, per substep. wRel = relative angular velocity about the hinge.
// The kv term is a hydraulic orifice: invisible at walking speeds, crushes
// fast oscillation. tauFF carries the ankle CoP command.
tau = clamp(j.kp * (j.target - j.angle) - j.kd * wRel - j.kv * wRel * Math.abs(wRel)
            + j.tauFF, -j.tauMax, j.tauMax);
```

```js
// BUILD-TIME STABILITY CAPS — assert these, they are why the servo doesn't buzz.
// h = physics substep. I = child link inertia about the hinge axis.
assert(j.kd * h / I < 1.0);                          // explicit damper stability (2 = divergence)
assert((j.kd + 2 * Math.sqrt(j.tauMax * j.kv)) * h / I < 2.0);  // quadratic damper bound
// gamma = kd/(kp*h) must come out IDENTICAL at every scale — cheapest scaling test.
```

```js
// Ground: compliant, damped, ZERO restitution (crushed soil, not a trampoline).
// One bodyweight sinks ~1% of leg length; dissipates on approach; never springs back.
groundCompliance = (0.01 * L) / (totalMass * g);     // m/N   (scales s^-2 — see §1)
groundDampingWeight = 1.0;                            // XPBD damping weight, or your
                                                      // engine's equivalent of
                                                      // "critically damped, e=0"
//
// STIFF BAUMGARTE TERRAIN (bias ~0.2/dt, restitution gated above a closing-speed
// threshold): probably fine for WALKING as-is — sin^2 zeroes the commanded
// touchdown speed, so nominal landings stay far under a 1.6 m/s gate and warm
// starts absorb the rest. (Caveat: we never isolated sin^2 on rigid ground;
// profile and crush shipped together.) The case that bites is the BLAST/FALL:
// a thrown mech closes above the gate, restitution opens, and the bounce pumps
// energy back into the rig — the exact flail-and-tear mode the zero-e rule
// exists to kill. Insurance, in order:
//   1. restitution = 0 on ALL mech-body contacts (one flag, no global parity hit)
//   2. per-foot compliance INSIDE the mech island (§6) if the headless gate
//      shows standing chatter or landing spikes — global terrain untouched
// Torque sizing: EVERY leg joint holds the whole machine on one leg:
// tauMax >= totalMass * g * lever_in_single_support. Size the ceiling for that,
// but tune kp to a SEPARATE reference torque — raising the ceiling must not
// stiffen the loop (stiffer measured strictly worse).
```

---

## 5. Passive appendages as dampers (the free stability)

```js
// Den Hartog tuning for anything that hangs off the hull (arms, pods, or a
// dedicated internal bob at ~3% machine mass on a 2-axis gimbal).
// mu = appendage mass / rest of machine;  wSway = omega from deriveGait.
function tuneDamper(mu, wSway, I_aboutHinge, m, leverToCoM, g, hangs) {
  const wT   = wSway / (1 + mu);
  const zeta = Math.sqrt(3 * mu / (8 * Math.pow(1 + mu, 3)));
  // hanging: gravity ADDS stiffness (kp may clamp to 0 — gravity is the spring).
  // inverted bob (CoM above pivot): gravity SUBTRACTS, so kp can tune EXACTLY.
  const kg = (hangs ? +1 : -1) * m * g * leverToCoM;
  const kp = Math.max(0, wT * wT * I_aboutHinge - kg);
  const wA = Math.sqrt((kp + kg) / I_aboutHinge);    // achieved frequency
  return { kp, kd: 2 * zeta * wA * I_aboutHinge };
}
```

---

## 6. Hinge joint for a contacts+welds engine (the Coldsnap gap)

The engine has contacts and welds only (3 linear axes + diagonal-approx full angular
lock). A mech needs a hinge: **weld's linear part + two axis-projected angular locks +
motor torque on the free axis.** ~150 lines. Reference below is written for a
single-velocity-pass solver at fixed `dt = 1/120`; position-level (XPBD) notes inline.

```js
// ---- hinge constraint, velocity level -------------------------------------
// Locks: 3 linear (reuse weld point-to-point at the anchor), 2 angular.
// Free axis: world hinge axis from body A.  a1 = A.rot(axisLocalA).
// The two locked axes p1, p2 complete an orthonormal frame with a1.
//
// Angular locks (each axis n of p1,p2):
//   Cdot = dot(wB - wA, n)                      // relative spin off-axis
//   Cpos = dot(orientationError(A,B), n)        // drift, for Baumgarte
//   lambda = -(Cdot + beta/dt * Cpos) / (nT*(IinvA + IinvB)*n)
//   apply +lambda*n to B, -lambda*n to A        // angular impulse
// beta = 0.2 at 1/120. USE THE FULL PROJECTED INVERSE INERTIA nT*Iinv*n here,
// not the diagonal approximation: the locked axes of a limb are exactly where
// non-principal terms show up, and a diagonal shortcut leaks energy into the
// spin the servo then has to fight. (Assert at build time that each link's
// hinge frame is within ~10 deg of principal axes if you keep the shortcut.)
```

```js
// ---- motor about the free axis, IMPLICIT damping --------------------------
// This kills stability-cap landmine #2 outright. Compute once per tick:
//   e    = wrapPi(target - angle)               // angle via atan2 of projected refs
//   wRel = dot(wB - wA, a1)
//   Ieff = 1 / (a1T*IinvA*a1 + a1T*IinvB*a1)    // effective inertia about axis
// Implicit velocity-level damper (unconditionally stable at ANY kd):
//   wNew   = (wRel + (kp*e + tauFF) * dt / Ieff) / (1 + kd * dt / Ieff)
//   tauReq = (wNew - wRel) * Ieff / dt          // torque implied by the update
//   tau    = clamp(tauReq, -tauMax, +tauMax)    // ceiling AFTER, so clamp wins
//   apply angular impulse tau*dt about a1 (+B, -A)
// Accumulate tau*dt as a clamped multiplier if your solver iterates; do NOT
// re-derive from scratch each iteration or the clamp is per-iteration, not total.
// kv (quadratic damper): fold into kd per tick as kdEff = kd + kv*|wRel| before
// the implicit step — same unconditional stability.
//
// If you keep an EXPLICIT damper instead (tau = kp*e - kd*wRel applied openly),
// the caps are mandatory and assertable at build time:
//   kd*dt/I  < 1        (2 = divergence; past it the joint bang-bangs the
//                        ceiling at 9-21 Hz and shakes the whole machine)
//   kp*dt*dt/I < 1      (this one diverges WITH more iterations, not fewer —
//                        soften kp, don't iterate harder)
//   (kd + 2*sqrt(tauMax*kv))*dt/I < 2
// At dt=1/120 on a large-L mech all three pass with margin — big mechs are the
// favorable regime; the caps exist so a future small mech fails LOUDLY at build.
```

```js
// ---- end stops -------------------------------------------------------------
// A stop is a one-sided angular constraint with FINITE compliance (elastomeric
// bumper, ~1e-7 rad per N·m at our reference torques) and its impulse counts
// toward the mount's failure/damage budget. A rigid stop with a discarded
// multiplier can silently carry 40 kN·m and read zero in telemetry — measured.
```

```js
// ---- solver LOD exemption (landmine #1) ------------------------------------
// The global solver drops 12 -> 4 iterations past 900 active constraints, i.e.
// exactly when the mech is knee-deep in rubble. The mech gets its OWN island
// with fixed iterations, and the island includes its FOOT CONTACTS — balance
// authority is contact-solve quality as much as joint stiffness. Our chain
// needed 8 interleaved sweeps: 6 walks in place but falls when travelling.
function stepMechIsland(mech, dt, IT = 10) {   // IT fixed; NEVER tied to LOD
  for (let it = 0; it < IT; it++) {
    for (const j of mech.joints)  j.solve(dt);      // hinges (above)
    for (const c of mech.contacts) c.solve(dt);     // this mech's feet vs ground
  }                                                  // INTERLEAVED, same loop
}
// In stepWorld: solve the world normally but EXCLUDE mech-owned constraints
// from the LOD-tiered pass; run stepMechIsland at fixed cost instead. A 10-joint
// mech + 8 foot-corner contacts at 10 iterations is ~180 constraint solves —
// constant, tiny next to a 900-constraint demolition, and the mech never sags.
// Rubble resting ON the mech still couples through the shared bodies; if rubble
// jitter feeds back, promote contacts that touch mech bodies into the island too.
```

---

## 7. Headless acceptance gate

"Stands, walks 20 m, eats a mortar" — plus the cheap asserts that caught our worst bugs:

```js
// build-time (no stepping):
assert(touchdownSlope(swingLift) ≈ 0);         // finite-diff at s=0.999 vs mid-swing peak
assertStabilityCaps(everyJoint);               // §6: kd·dt/I, kp·dt²/I, kv bound
assertNoNaNorUndefined(builtRig, at 2 scales); // every derived constant, both sizes
assert(gammaIdentical(scaleA, scaleB));        // kd/(kp·dt) per joint, exact
// stepped (the gate itself):
stands 10 s, |accel| quiet;  walks 20 m without a fall;
mortar impulse -> falls -> ASSERT walk controller stopped and servos limp
(a walk controller driving a downed mech is the tear-everything mode);
respawn -> stands again.
```

---

## 8. Port order

1. §6 hinge + mech island — the engine gap; nothing works without it.
2. §1 scale module — including the √s timestep. Verify: gamma identical at two sizes.
3. §4 servo law + caps, §3 swing profile, ground recipe (+ e=0 on mech contacts).
4. §2 stepping: plan → swingTarget → copCommand → standCatch → planStop → ramp.
5. §0 mass layout on the art rig (this is a *design* gate, not code).
6. §5 dampers on whatever hangs off the hull.
7. §7 gate wired into CI before tuning starts — every bug we lost days to was a
   number that was wrong when typed and had no check.

1–4 walks. 5 decides whether it *keeps* walking. 6 makes it feel planted. 7 keeps it true.

Tuning order if it misbehaves: mass layout → timestep scaling → kd caps → kCapture
(lower it if catches overshoot and ring) → agility AG. Everything else stays derived.
