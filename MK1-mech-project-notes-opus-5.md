# MK1 — Physics-Driven Walking Mech

**Status document · Claude Opus 5**

A browser artifact of a walking mech where the physics is real: every component has
mass, every joint is a torque-limited actuator, every mount has a failure envelope, and
overdriving an actuator can tear a limb off. Blocky first, detail later.

---

## 1. Vision

The original brief, unchanged:

- A highly detailed 3D mech the viewer can walk around, in 2.5D isometric.
- The mech has mass and obeys physics. **Every component has its own mass.**
- Every joint is an actuator moving on a planned path with a known output in N·m.
- Limbs attach with a known mount strength. **Mounts shear under enough load**, so
  overdriving an actuator can rip a limb off.
- Mobile-first. Physics before geometry.

Three decisions taken early and still holding:

| Decision | Choice | Why |
|---|---|---|
| Behaviour | Walking gait + balance | Not a static rig, not scripted poses |
| Camera | Snapped isometric, 24 azimuth detents | True iso is a fixed camera; walking around is an orbit |
| Build order | Physics first, blocky | Geometry is cheap once the sim is gated |

**Solver choice.** Maximal coordinates + XPBD rather than Featherstone. In XPBD the
constraint multiplier *is* the reaction force (λ/h²), so mount load readout and shear
tests fall directly out of the solver. Featherstone joints are rigid by construction, so
tearing would have to be faked. A torque-limited actuator is just a clamped Lagrange
multiplier; saturation and droop are emergent, not special-cased.

---

## 2. Current state

**Rig:** 8.14 t · 4.91 m tall · 2.30 m footprint · 17 rigid bodies · 14 actuated joints ·
2 welds.

**Sim:** XPBD, 12 substeps × 4 solver iterations, running live in the browser at ~2.4 ms
per frame.

**Gait:** 32 steps, 11.6 m, 0.236 m/s at a 0.78 m stride cap.

**Gates:** core 11/11 · rig 13/13 · IMU 13/13 · balance 6/6 · gait 5/6.

```
core/physics.mjs      XPBD solver, bodies, joints, welds, contacts, pair collision
control/ik.mjs        closed-form leg inverse kinematics
control/posture.mjs   pelvis + foot poses -> joint angles
control/balance.mjs   StateEstimate boundary, ankle centre-of-pressure control
control/dcm.mjs       divergent-component-of-motion planner
control/gait.mjs      state machine, footstep planning, live command
rig/mech.mjs          parameterised link table and assembly
sensors/imu.mjs       ADIS16505-2 model  <-- BUILT BUT NOT WIRED IN
gates/                five suites, run headless
```

---

## 3. Design constraints discovered

These are the load-bearing findings — each was measured, and each cost several wrong
guesses to reach.

**Single-support feasibility is governed by `hipOffset − footHalfWidth`.** Not by hip
width alone. Widening the hips while scaling the feet to match keeps walking intact;
widening them alone kills it. Hip 0.50 with 0.80 m feet manages 7 steps; hip 0.55 with
1.06 m feet manages 22.

**Foot clearance trades directly against walking robustness.** Gap 0.00 → 16 steps,
0.04 → 13, 0.10 → 9, 0.20 → 7 at the geometry of the time. Wider stances need
proportionally bigger feet to hold the ratio.

**Stance width and speed trade off directly.** Lateral transfer distance scales with hip
width; the time available doesn't. At the current leg length no hip offset above 0.60 m
has a stable configuration at any timing swept.

**Leg mass fraction is the dominant stride limiter.** Legs were 45% of total mass and the
densest parts of the machine (~3050 kg/m³ against a 1184 kg/m³ torso). Lightening thigh
1150 → 450 kg and shin 780 → 300 kg took the stable stride from 0.62 m to 0.80 m and
travel from 8.7 m to 11.6 m. Nothing else attempted came close.

**A dynamically feasible plan is not optional.** The first hand-built pelvis pattern moved
0.40 m laterally in 0.65 s, which demands a centre of pressure 1.8 m outside the foot
against an admissible 0.40 m. It was never executable. The DCM planner produces
trajectories that are feasible by construction — ZMP consistency measures 0.000 µm.

**`copClamp` is the one true cheat.** Everything else on the preset list is a plausible
engineering choice. Letting the centre of pressure leave the support polygon is magnetic
feet, and it makes the mech nearly unfallable.

---

## 4. Bugs found

Nearly all were found by gates or instrumentation, not by reading code. Ordered by how
much they mattered.

### Solver

1. **Stale anchor lever arms in `Weld.solve`.** `r1`/`r2` computed before the angular
   pass, reused by the positional pass after the body had rotated. Loop gain exceeded 1;
   welds diverged within a single frame. Fixing it took weld load error from 3.6e7% to
   **0.000%**.
2. **Friction measured from a mid-solve snapshot** instead of the pre-integration pose, so
   friction fought the normal push-out and pumped energy. The rig launched itself to 200 m.
3. **Contacts collected per frame, not per substep.** A fast corner tunnelled through the
   ground between collections and took a huge push-out next frame.
4. **One Gauss-Seidel iteration can't propagate along a 7-link leg.** The rig yawed 30° in
   10 s. Four iterations brings it to 1.9°. Iterations matter more than substeps: 12×3
   falls, 10×4 walks.
5. **Explicit gyroscopic integration** → implicit, one Newton step.
6. **Small-angle approximation in angular velocity extraction** → exact log map. With (5),
   angular momentum drift went 0.7% → 0.22% per 10 s.
7. **No body-to-body collision at all.** Only ground contact existed, so the feet passed
   through each other by up to **300 mm** while walking. Added a separating-axis pair
   constraint for the foot and shin pairs.

### Actuators

8. **Infinite-gain servo.** `motorCompliance: 1e-9` meant ~17 kN·m of torque per
   milliradian of command, so control-signal noise became kN·m of oscillation. Replaced
   with a real PD actuator in physical units. This is what unlocked balance working at all.
9. **Scaling servo gains with `tauMax`.** Raising actuator authority shouldn't stiffen the
   loop; doing both produced 280 kN·m transients and tore the rig at frame 0. Made the
   same mistake twice — once in the presets, once while testing ankle torque.

### Control

10. **The balance loop replaced the PD servo instead of adding to it**, discarding exactly
    the passive stiffness holding the mech up. Feedforward now rides on top.
11. **Ankle roll in double support is destabilising.** Net lateral CoP moves by load
    sharing between feet, not by rolling either one; commanding roll just tips a foot onto
    its edge and sheds contact area.
12. **Commands read live mid-swing**, so moving a slider teleported the swing foot target.
    Latched at touchdown, and rate-limited — a step change in speed or heading is a
    disturbance the walk doesn't survive.
13. **Replan inserted a redundant leading double-support phase.** Each step cost
    `2×tDS + tSS` instead of `tDS + tSS` — a third of the cycle doing nothing. Removing it
    took speed from 0.046 to 0.213 m/s.
14. **No standing state.** The state machine cycled forever, so zero stride meant
    "zero-length steps," not "stand." Hands off the controls, the mech marched 32 steps in
    place. Shipped that way for several revisions.

### Planning

15. **Under-travel.** The swing foot lands short of the commanded stride, but `plant`
    advanced by the *commanded* amount, so the plan desynchronised a little more each step.
    Replanning from measured landings took travel from 0.749 m to 1.550 m.
16. **Footprint separation collapse.** Nothing commanded lateral foot placement, so each
    landing inherited the previous one's inward drift: 0.489 → −0.217 m over ten steps, and
    the mech walked into a cross-legged stance. Deterministic failure at exactly step 10.
17. **Constant-ZMP phases jump 0.28 m at every transition.** A centre of pressure can't
    teleport; switched to linearly interpolated ZMP, which has its own closed form.
18. **Gravity hardcoded at 9.81 in the DCM planner**, so the Lunar preset planned against
    the wrong pendulum frequency and tore itself apart.

### Measurement

19. **Sensor noise injected after the bandwidth filter**, making ARW come out 31% low. It
    belongs at the sensing element so the filter shapes signal and noise together.
20. **Gauss-Markov Allan peak coefficient.** The commonly quoted 0.437 doesn't match this
    definition of Allan deviation. Measured it at **0.6084** over a 60-hour series against
    an estimator validated to four decimals on white noise.
21. **Contact force telemetry sampled one substep** instead of averaging the frame — read
    4.7 kN against a 110 kN weight, which starved the balance loop of its main input.
22. **Ankle pivot sign error** in both the rig table and the controller, putting the foot
    on heel-forward.
23. **`centerOverFeet` was a no-op** — it translated the whole rig, ankles included, so the
    relative geometry never changed.

---

## 5. Things tried that did not work

Kept deliberately, because the negative results are what stop us re-treading.

| Attempt | Result |
|---|---|
| Hip counter-rotation strategy | No benefit at either sign. Left in at gain 0 |
| Lateral load-shift trim | No benefit at either sign. Left in at gain 0 |
| Stance/swing IK reference split | **Worse** — fell after 1 step. Removing the horizontal reference removes the only thing driving COM motion |
| Longer legs for a wide stance | No help. 2.95 → 3.65 m changed nothing above hip 0.60 |
| Ankle torque for longer stride | Marginal. Best case 1 → 11 steps at stride 1.00, never stable. Above 80 kN·m, no effect |
| Longer feet | **Worse**, twice. Done properly (pivot moved, mass rebalanced) it dropped stride 0.62 from 29 steps to 7 |
| Doubling hip width | No stable configuration at any timing, with or without scaled feet, lighter torso, or more hip torque |
| Torso lightening alone for wide stance | No effect on the wide-stance blocker |

**Three consecutive wrong diagnoses** are worth recording as a pattern: leg reach, ankle
saturation, and foot length all looked binding and weren't. In each case the rig was
changed on reasoning rather than measurement. The stride ceiling is sharp — 0.62 m walks
32 steps, 0.70 m falls after 3 — and a sharp cliff looks like a planner infeasibility
threshold, not a hardware limit. That remains uninvestigated.

---

## 6. Verification approach

Five headless suites, run before anything ships. The rules that earned their place:

- **Gates assert against hand calculations or analytic results**, not against previous
  output. Weld bending moment is checked against `m·g·L/2`; pendulum period against
  `2π√(I/mgd)`; sensor noise against the manufacturer datasheet.
- **A gate that guards a failure must be able to detect it.** R9 measures foot
  interpenetration with an independent SAT implementation, then runs the same walk with
  the constraint stripped out and requires *that* version to overlap.
- **Failing gates stay failing and visible** rather than being tuned around. W6 (20 steps
  in place) has been red for several revisions.
- **Measured constants beat remembered ones.** GM_PEAK is annotated as measured, with the
  method, because the textbook value was wrong for this definition.

The sensor model is built from the **Analog Devices ADIS16505-2** datasheet (Rev. C, Table
1) — no invented figures. Worth noting: the gyro's ARW, output noise and 3 dB bandwidth are
mutually consistent to ~1%, which validates the whole modelling chain. The accelerometer's
are not — its VRW and rms imply a 461 Hz equivalent noise bandwidth against a published
750 Hz. The published figure ships; the discrepancy is reported rather than tuned away.

---

## 7. Open items

Ordered by what I'd do next.

**1. Wire the IMU into the control loop.** The model exists, is gated 13 ways, and the
`StateEstimate` boundary was built specifically so the controller couldn't reach past it —
then the controller ran on `groundTruthState` for the entire project. **Every balance and
gait number in this document assumes perfect state knowledge.** Some of them may not
survive real sensor noise, and we don't know which. This is the item that could invalidate
earlier results, so it goes first.

**2. True twin-stick.** What ships is tank controls: left stick is throttle and turn rate,
right stick drives the camera. Twin-stick means decoupled travel and facing. Measured
capability with facing locked: forward, backward and rear diagonals work (backward manages
6.12 m, further than forward); **pure lateral and forward diagonals fall in 3 steps.** The
planner treats lateral foot motion as error and corrects it toward nominal stance width, so
in a strafe the correction cancels the step. Cheap hypothesis first: make the correction
relative to commanded travel. If that fails, it needs a lead-and-follow side-step pattern.

**3. Hip yaw.** The hip is a 2-DOF gimbal — roll and pitch, no yaw. That single missing
axis is the common cause behind the 4°/s turn cap, the strafe failure, and twin-stick being
inexpressible. Without it the feet can't point anywhere except where the pelvis points.
Probably the highest-value structural change available.

**4. The knees are reversed.** The knee sits 231 mm *behind* the hip-ankle line with a
one-sided `[0°, 130°]` range — a digitigrade leg. This was never a decision; it fell out of
the angle signs and went unexamined. Flipping it is mostly a look choice, but it changes
push-off geometry enough to need re-gating.

**5. Investigate the stride cliff** at 0.62 → 0.70 m before changing more hardware.

**6. Steering asymmetry.** Sustained left turns hold 16 steps, right turns 13. A clean
left/right asymmetry is a sign error somewhere, not a tuning issue.

**7. W6** — 20 steps in place still fails.

**8. Detail pass.** The visual mesh never touches the solver, so greebling is purely
additive and cheap. Deliberately last.

---

## 8. Performance budget

12 substeps × 4 iterations costs ~160 ms per simulated second in Node — about 16% of
realtime, ~2.4 ms per 60 Hz frame. The fidelity cliff is sharp and it is *iterations*, not
substeps: 12×3 falls, 10×4 walks. Below 4 iterations the leg chain can't propagate
corrections root-to-tip and the gait dies.

The artifact ships as a single self-contained HTML file, ~85 KB, with the solver bundled
from the same modules the gates run against — verified to reproduce the module build
step-for-step.
