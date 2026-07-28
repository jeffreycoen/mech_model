# MK1.4 — status and handoff

**Claude Opus 5 (1M context).**

Live artifact: `mech-mk1-live-opus-5-1m.html`
Served by: `node logserver.mjs 8080` → `http://<host>:8080/mech-mk1-live-opus-5-1m.html`
(the server dies when a session's background processes are torn down; just restart it).
Session logs land in `logs/*.jsonl`.

---

## 1. What this build is

A blocky biped whose physics is real: XPBD maximal-coordinate solver, every link a mass,
every joint a torque-limited actuator, every mount a four-term failure envelope that can
tear. 19 bodies, 16 joints, 2 welds.

**Controls are twin-stick.** Left stick is a travel VECTOR (any direction). Right stick
AIMS THE TORSO, which yaws on a waist ring and gets there immediately; the legs only come
round once you have used up the ring's travel. Camera stays on 24 isometric detents, driven
by the on-screen pad or arrow keys.

**Size is a free parameter.** The Size button rebuilds any preset at 4 ft / 2 ft / 1 ft.
Everything the controller needs is derived from the rig, so no retuning is involved.

Presets: **Light Frame** (default), **Scout Walker** (AT-ST proportions), **Reference
(heavy)** (the original 8 360 kg rig, what the self test asserts against), **Overdriven**
(tears itself apart on purpose — the live proof the failure envelope is real).

Self test: 15 analytic gates, on demand, run against the same solver the mech uses.

---

## 2. Design rules — dimensionless constants

Validated across MK1 (leg 2.95 m) and the Scout (leg 5.00 m). `L` = leg length,
`z` = COM height, `ω = √(g/z)`, `tRef = √(L/g)`.

| quantity | rule |
|---|---|
| single-support time | `1.66 / ω` |
| double-support time | `0.92 / ω` |
| step height | `0.047 · L` |
| crouch depth | `0.085 · L` |
| working stride | `0.28 · L` |
| CoP limit fore/aft | `0.80 ×` foot forward extent from the ankle |
| CoP limit lateral | `0.65 ×` foot half width |
| **hip offset** | **`≤ 0.21 · L`** — 0.20 walks; 0.27 and 0.33 both fail outright |
| actuator vs mount | `tauMax / torsion limit = 0.73` |
| every controller TIME | scales as `√L` |

Actuator sizing, as a fraction of `m·g·L`: **hipYaw 0.50, hipYoke 0.58, thigh 0.78,
shin 0.78** — 2× the original table. Ankles size off the CoP authority they must produce:
`1.40 · W · copLimitX` and `1.45 · W · copLimitZ`, which is why they moved by the
balance-zone ratios (0.80/0.60 and 0.65/0.45) rather than doubling.

Design intent: **either leg holds the whole body at any time, like a human standing on
one leg.** The old table sized a leg for SHARED support; one leg carrying the whole body
is exactly twice that load, so the rule is the same table × 2 — no new lever estimate.

**Servo gains:** `kp = kpTau / 3°`, `kd = 0.06 kp`, where **`kpTau` is not `tauMax`**.
The invariant is the SATURATION ANGLE at `kpTau`, and `kpTau` did not move when the
ceilings did — so small-error behaviour is identical and only the headroom changed
(3° → 6° before an actuator saturates). An inertia-based law (`kp = ωₙ²I`) was tried and
measured strictly worse — 0/3 on two rigs where this form is 3/3 — because it lets the
saturation angle vary per joint.

> ⚠ **`kd = 0.06 kp` is a DESCRIPTION of what ships, not a rule to preserve.** It is
> `γ = kd/(kp·h) = 36`, and the damping term is unstable there — see FINDINGS.md §1, where
> a +1.0 step command settles at −5.31 on the ankle. A fix is proposed and unapplied.
> `hipYaw` already ships at `kpTau/8°` on all three rigs, so the saturation-angle invariant
> above is already a two-thirds rule rather than a law. The value is asserted by
> `test/invariants.mjs` I2 across every joint and size; change it there, not here.

Not to be conflated: servo GAIN (`kp`) ×2 and ×4 both measured worse at every gait timing.
That is position-tracking stiffness. `tauMax` is authority. Different knobs — and they are
now different fields, so raising one cannot silently raise the other.

Mount torsion rose with `tauMax`, holding `tauMax / torsion = 0.73`. An actuator that
tears its own mount at full authority is not authority. Overdriven (×3) still detonates.

Solver: 10 substeps × 8 iterations. Iterations must rise with chain length — the yaw ring
made each leg an 8-link chain and 6 iterations walks in place but falls when travelling.

---

## 3. Bug patterns worth knowing

Three classes account for nearly everything found:

**Absolute constants in a model that must scale.** Eight sites, all marked `SCALE FIX`.
Worst: the DCM planner's pendulum height floored at an absolute 1.0 m, so every rig under
~1.7 m — including the shipping default — planned against a pendulum it did not have. At
1 ft the frequency was out by 2.4×. Also: capture-point height floor, a 30 mm stick
deadband (52% of a 1 ft rig's stride range), a 1 N contact-force floor, and every gait
time constant in fixed seconds.

**Free integrators.** Quantities with no restoring term that random-walk. The footprint
pair's lateral centre was one — `centreZ` was computed in `init()` and never used, so only
separation was regulated and the machine wandered off its line. **Body yaw is still one**
(see open items).

**A leftover absolute constant overriding a derived one.** This class bit twice in one
session and is the one to watch for:

- The gyro was sized from a leftover per-preset `scale` while the rig was scaled by the
  Size selector's value. They coincide at 4 ft and diverge below it; at 1 ft the gyro came
  out 266× too strong and tore the rig in 0.01 s.
- The stick's stride cap fell back to `MAX_STRIDE` — 0.62 m, from when the rig was full
  size — instead of the stride derived for the current rig. Every size got the same 0.62 m
  cap: 3× the working stride at 4 ft, 12× at 1 ft. Foot mounts tore at util 1.0–1.16 a few
  seconds into any drive, at every size.

Both were found from driving logs, not from the Node harnesses, which had the correct
values hardcoded and so could never see them.

---

## 4. Measured state

All figures are ensembles with nanometre initial-position jitter, **not single runs** —
the walk is deterministic but chaotic. A 1.9×10⁻¹⁶ relative change in one gait parameter
was measured moving a fall from 178 s to 23 s. Single-run numbers are meaningless here.

**Scale invariance** (gate G15): 4 ft (69 kg) and 1 ft (1 kg) walk the same dimensionless
walk to **2.5%**, both upright, nothing torn.

**Standing:** clean at 4 ft, 2 ft and 1 ft, with and without the gyro, zero breaks.

**Walking forward:** ~2.2 leg-lengths per 60 tRef at every size, under 2° of yaw.

**The gyro (CMG)** is modelled hardware — real mass in the torso, torque limit, finite
momentum store that saturates. On Light Frame at a 1.00 m stride it takes 3/5 → 5/5
upright and 29 breaks → 0, using half its torque and 45% of its store.

---

## 4b. Ideal-mode round (2026-07-26 evening)

The goal changed: tabletop fun over realism. Applied: full-authority attitude assist
(roll/pitch hold + gentle yaw steering toward commanded facing, no momentum store), 4x
mount margins, auto-face when the right stick is idle, camera bounds rig-relative, feet
drawn at 70% but physically unchanged (shrinking the physical feet was measured fatal:
falls inside 5.5 s in every configuration).

**Two world-frame bugs found and fixed — the root-cause class for every turning fall:**
1. Balance fed world-X/Z CoP errors straight to body-fixed ankle pitch/roll axes.
   Exact at facing 0, REVERSED at facing 135.
2. `comToPelvis` captured at spawn and re-applied unrotated, biasing the pelvis target
   off-axis after every turn.
The diagnostic that found both: cold-start walking clean at facing 0, dead at facing 135.
Suspect a third leak if turning still degrades; candidates: the DCM copClamp box,
stride-vector replan discontinuity while heading rotates.

Measured after: fwd/back/strafe/turn-in-place clean; extreme box-drive (90-degree heading
snaps at full speed every 5 s) survived 40 s with zero breaks; single-turn-then-walk still
falls sometimes (10-22 s, single runs, chaotic). Footstep-commit-at-liftoff was tried and
reverted: it desyncs feet from the DCM plan during command ramps.

## 4c. Agility round — "nimble, not bumbling"

**Turning was a NO-OP, not a stability problem.** A turn command never rotated the
machine at all: 2 steps in 25 s and the heading never reached, at 1x/3x/5x/8x turn rate.
It had been recorded as "turn-in-place is clean" because standing still cannot fall over.
Three defects, all the same class — a rule implemented at two sites and updated at one:

1. `wantMove` tested TRAVEL only, so a pure turn never left STAND.
2. A SECOND travel-only gate at touchdown closed the walk out after a few steps.
   Both now call one predicate, `wantsMove()`.
3. "Am I aligned?" was measured against the controller's own command, which slews faster
   than the body turns — so it declared victory while still pointing the old way. The
   gate now reads MEASURED body yaw (`bodyYaw()`), and the commanded frame may not lead
   measured yaw by more than `yawPerStep`. That cap replaces turn-rate tuning outright:
   `turnRate = yawPerStep / (tSS + tDS)`, one parameter instead of two that can disagree.

**Measured (1.25 m, ensembles not run — single 25-40 s runs):**

| lever | finding |
|---|---|
| gait time x0.7 (`AG`) | 4.56 m vs 3.28 m per 25 s. 0.5 is worse than 0.7. No falls in any cell. |
| servo gain x2, x4 | strictly WORSE at every timing — leave `kp` alone (this is stiffness, NOT the torque ceiling) |
| CMG `yawGain` 0.03 -> 0.30 | 135 deg turn 13.3 s -> 4.2 s AND forward travel improves 4.51 -> 4.68 m |
| `yawGain` 0.6 / 1.0 | turn 3.4 s / 3.3 s, forward flat. NOT battery-verified; 0.30 is shipped. |

Body delivers only ~4 deg of yaw per step against foot friction regardless of command —
that physical limit is why the assist, not the planner, is the lever that worked.

**Battery after (40 s each, single runs):** diagonal turn-and-walk — the case that failed
all session — now survives with ZERO breaks (was falling at 4.7-16 s). Forward 7.58 m, up
from 5.35 m. Strafe clean. Still falling: reverse-180 at 12.6 s, extreme box-drive at
21.1 s — both large heading reversals.

**Process note:** `sim.mjs` silently diverged from the artifact mid-session and a whole
sweep ran against stale code. Verify parity by grepping BOTH files for each change before
trusting a sweep, not just that the run exited 0.

## 4d. Structural round (2026-07-26, later) — NOT YET DRIVEN

Found by reading, not by measuring. Everything below ships unverified by design: under the
standing DO NOT SIMULATE rule the build is served and Jeff drives it.

**Modules now sit on logical boundaries.** The original single-file split was cut at byte
offsets, so `assembleMech`/`groundRig`/`rigStats` lived in `rig/atst.js`, `BalanceController`
lived in `control/cmg.js`, and `rig/mech.js` ended mid-sentence. Now: `rig/mech.js` holds
MK1's spec and chain, `rig/atst.js` holds only the Scout spec, the new `rig/assemble.js`
holds the spec→world machinery both share, and `control/balance.js` holds the balance
controller. Behaviour is unchanged — the concatenation order is the same.

**Two-site rules closed.**
- `yawPerStep` was passed by the test battery and NOT by the artifact, which fell back to
  the controller default. Both read 20°, so they agreed by coincidence; moving `YPS_DEG`
  would have moved `turnRate` and not the slew cap. Now passed.
- `MAX_TURN` (8°/s) and `turnCap` deleted. `turnCap` was assigned and never read;
  `MAX_TURN` was a stale absolute still being fed to the self test and logged as if it
  governed anything. Turning is one derived parameter, `yawPerStep`.
- The self test built its gait from hardcoded `turnRate`/CoP limits belonging to no rig.
  It now derives them exactly as `buildWorld` does. `copClamp`, `tSS` and `kCop` stay
  explicit — they are the analytic fixtures, and that is now stated in the code.

**Driving logs now record the derived configuration.** A `build` event at the end of
`buildWorld()` carries scale, mass, stride cap, gait timings, turn rate, `yawPerStep`, CoP
limits and every actuator ceiling. Every stale-constant bug so far was a derived value
disagreeing with a hardcoded one, and none of them were visible in a log.

**Release-to-rest.** Three defects stopped the machine coming to rest when the stick was
released, all in the same handful of lines:
1. A stop in progress was aborted by *any* `wantsMove()`, including the yaw residual left
   over from a turn — which is always non-zero, because the body delivers ~4°/step. So
   every close step was cancelled and the mech marched in place indefinitely. Only a real
   TRAVEL command aborts a close now.
2. The wide hysteresis band applied only once already standing, not while stopping — so
   the closing step's own body swing re-tripped the narrow 4° band. It now applies from
   the moment the stick is released.
3. `standing` was never initialised, so the first update after warm-up fell into the
   walking branch and the machine took two zero-length steps on spawn with nobody
   touching the controls. It now starts at rest.

Also: the gyro was fed `cmd.facing` while standing, so a leftover heading error had it
grinding yaw torque through the soles of a machine that was supposed to be still. It now
reads `gait.stabiliserYaw()`, which holds measured body yaw at rest.

**Risk to flag:** gate G10's second branch asserts that the feet overlap >5 mm when the
pair constraint is stripped out. Wider CoP limits change that walk, so G10 is the one gate
that could flip. Press *Run self test* in the page; it runs against the shipped code.

## 4e. Turning actually turns now — waist ring + yaw steering (NOT YET DRIVEN)

**The right stick was steering nothing at all.** Not badly — nothing. `posture.apply`
computed the hip yaw ring angle as `footYaw[s] − pelvisYaw`, and both sides of that came
from the same `active.facing`, so it cancelled identically. Evaluated directly at 0, 15,
45, 90, 135 and 180° of commanded facing: `hipYawL = hipYawR = 0.000000` every time. The
one joint on the machine that can point the feet somewhere the pelvis is not was held flat
at every heading. The only yaw authority left was the gyro dragging the whole rig around
against foot friction, which is the ~4°/step in §4c — that number was never a physical
limit, it was the leftover.

Driving log `s20260726223836` shows it plainly: 135° commanded, 38° reached after a second,
then the driver gave up. Section 5's "body yaw is unregulated" was understating it.

**Two changes:**

1. **Waist ring.** The torso yaws on the pelvis instead of being welded to it — a real
   hinge, 60 kN·m, ±50°, sized to swing the torso+head+arms assembly (~1 800 kg·m² about
   +Y) through 90° in about half a second. That is 0.25·m·g·L, the fraction the hip yaw
   ring used to carry. The Scout gets the same ring at 100 kN·m. Right stick sets
   `gait.aim`; the ring is there immediately, because it turns one body against one
   actuator instead of scrubbing two loaded feet across the ground.
2. **Yaw steering, stance and swing with opposite signs** — the controller §5 said did not
   exist. Swing foot is aimed straight at the commanded heading (it is unloaded, so this
   is free). The stance ring is driven the other way, so pushing against the planted foot
   rotates the pelvis onto the target. Only in single support: with both feet planted the
   rings fight through the ground, so standing, warm-up and double support pass null and
   keep the old flat, compliant behaviour. Verified statically — target 30° gives swing
   +30.0°, stance −15.0°, and 0.0/0.0 with both feet down.

`kSteer` is 0.5, not 1.0, on purpose. Driving both rings from measured yaw is what
collapsed the walk in 6 s last time; half authority on the stance leg only, in the one
phase where it is mechanically valid, is the conservative version of the same idea.

**Consequences that had to follow:**
- `bodyYaw()` now reads the **pelvis**. It read the torso, which was the same body when the
  torso was welded and is a turret now. Every rule that uses it — the `yawPerStep` cap,
  "am I aligned", what the stabiliser holds — is about the chassis.
- The gyro reads `st.pelvisYaw` / `st.pelvisRate` instead of the torso's. Left alone it
  would have spent its authority braking the turret: yaw damping is 21 kN·m per rad/s
  against a 60 kN·m waist actuator, so a fast slew would have been fighting a third of its
  own strength.
- Logs now record `aim`, `yaw` and `waist` per frame. Without those three a log cannot
  distinguish "the stick did nothing" from "the stick worked and the legs did not follow",
  which is the distinction that took a whole session to find by reading code.

**Feel to check when driving:** right stick should snap the upper body over immediately
with the feet planted. Past ~30° of waist the legs should start stepping the chassis
around underneath. Release both sticks and it should take one closing step and stand.

### 4e-i. Turning fast tore the arms off — the waist had no rate limit

Driven and broken the same evening. Six sessions, arms off in five of them: `upperArmL`
1.27, `upperArmR` 1.04–1.31, then the torso mount at 1.45–1.94 and the head as collateral.

**Cause, from `s20260726234708` at t=20.3.** The waist was parked on its +50° stop with the
chassis at yaw 76°. The aim reversed to −45°, giving `wrapPi(−45 − 76) = −121°`, clamped to
−50° — so `J.target` went **+50 → −50 in a single frame**. The ring crossed in ~0.1 s and
slammed the far stop: `torso util 1.45, SAT, STOP`, `upperArmL 1.27` in the same frame,
`upperArmR` and the head two frames later.

The waist was a POSITION command with no slew. Every other channel here is rate-limited —
travel by `travelRate`, facing by `turnRate`/`yawPerStep` — and the fastest, heaviest one
was not. The arms sit 1.025 m off the yaw axis, so they are the outermost mass on that
body and they pay first.

**Fix:** `waistRate`, derived — full ring travel in one pendulum-scaled second, ~200°/s at
4 ft. Computed on the shipped rig:

| | |
|---|---|
| unlimited step across the ring reaches | **37 rad/s** |
| rate-limited peak | **3.5 rad/s** |
| momentum into the end stop | **10.7× less** |
| servo lag while slewing / distance needed to stop | 11.9° / 0.9° |
| → ring decelerates | **before** the stop, so it never slams |
| largest single-frame target move, same reversal | 1.67° (was 100°) |

A 50° aim change still lands in a quarter second, so it should not read as slower.

**Driven after, `s20260727000055` run 0 — Light Frame, 4 ft, 58 s, 89 steps: zero falls,
zero breaks, waist worked to the full ±50° throughout.** The arms stayed on. Same run also
confirms the yaw steering: chassis yaw covered **−115° to +170°**, 285° of real turning,
with the hip yaw rings ranging ±45° and averaging 5.4° of deflection — against exactly
0.000000 at every heading before. Turning turns the machine now.

**Also:** arm, forearm and head mounts doubled. They were sized when the torso was WELDED
and the only thing that shook them was the gait; they were already the weakest mounts on
the machine (`upperArm` bend 95e3 against a leg joint's 210e3) and the waist ring put them
on the end of a lever on the fastest actuator. That is the load case being sized honestly,
not a patch — the rate limit is the actual fix. Torso mount limits left alone deliberately:
at 10.7× less momentum its 1.45 should fall well under 1, and inflating it would hide the
next real problem.

### 4e-ii. The turret was still dragging the legs after release

`s20260727000055` run 0 has four stick releases. Three stood promptly — 0 steps, 0 steps
and 2 steps / 0.60 s. The fourth took **16 steps and 9.2 s**.

The difference was where the aim happened to be pointing. `updateWaist` fed `want.facing`
from `this.aim` whenever the waist was past its follow threshold, and `aim` retains the last
stick value forever. Release with the turret still 125° off and the legs go on being handed
a heading nobody is asking for, stepping until the chassis gets within `turnEpsHold`.

Fixed with `aimHold`, set by the UI while the right stick is deflected. Travel or aim held
means the legs follow; **both released means the legs let go of the heading entirely** and
`want.facing` tracks measured chassis yaw, so `yawCmd` collapses to zero and the machine
closes and stands. Verified on the replayed case: `aimHold=true → yawCmd 125°, wantsMove
true`; `aimHold=false → yawCmd 0°, wantsMove false`. The torso keeps looking wherever you
left it — only the legs stop chasing.

Turning in place with the right stick alone still works: that is `aimHold`, not travel.

### 4e-iii. Two telemetry defects that nearly produced a wrong diagnosis

Both fixed; both were making the logs lie about the axis that matters.

- **Contact force was logged as kN to one decimal.** Written for the 8 360 kg reference rig,
  where 0.1 kN is nothing. On the 27 kg Scout at 4 ft it quantises to **37% of body weight
  per count**, so the log read `0.00 / 0.37 / 0.75 / 1.12` and looked exactly like a machine
  repeatedly leaving the ground. It is not. Now stored as a fraction of body weight, which
  is the unit every load question here is actually asked in.
- **Only the X component of each foot's CoP was recorded.** Lateral balance is the axis that
  fails, and there was no way to see a CoP sitting on the edge of a foot. Both axes now.

## 4f. Heavy Walker — the quadruped (NOT YET DRIVEN)

Proposed by a subagent from the Scout, reviewed and merged. Three of its critiques of my own
first draft were correct and material, and all three are the kind that only show up by
reading:

1. **No `kpTau` in the leg chain**, so `kp = tauMax/3°` and servo stiffness was tied to
   actuator authority — the one rule `rig/mech.js` says must never be broken. My draft's
   thigh would have run **14.6e6 N·m/rad against MK1's 1.81e6**.
2. **Mount envelope scaled all four terms with `tauMax`.** Tension, shear and bend carry
   weight, not torque; MK1's own table tapers 1.2:1 on tension against 4.75:1 on torque.
   My helper gave the *foot* mount a sixth of the thigh's tension capacity, and foot mounts
   are exactly what tears in the driving logs at util 1.0–1.16.
3. **Height budget.** `simSteps = round(1/√SC)` flips from 2 to 3 at `SC = 0.16`, i.e. a
   native height of 7.8125 m. My 9.79 m draft cost 3 ticks; 7.00 m costs 2 — a flat 33% of
   the frame budget on the Pi. It derived 9.79 m for my draft independently and matched my
   measurement exactly.

It also caught a crash I would have shipped: `11-loop.js` dereferenced `b.footL`/`b.footR`,
which are `undefined` on a quadruped — `TypeError` on the first 10 Hz log frame.

**Verified statically on the built artifact** (no simulation):

| | |
|---|---|
| native height | 6.995 m → SC 0.1787 → **2 ticks/frame** |
| mass | 17 840 kg native, 101.8 kg at 4 ft |
| structure | 4 legs, 25 joints, 1 weld, 12 pair constraints |
| actuator fractions of m·g·L | 0.3125 / 0.3625 / 0.4875 / 0.4875 (table × 1.25 crawl margin) |
| saturation angle | 3.75° (i.e. `kpTau` holds the loop at the un-margined table) |
| `tauMax / torsion` | 0.243 — the 0.73 floor with the Scout's 3× height allowance |
| body sway per lift | 73 mm, clearing every triangle edge by exactly `crawlMargin` |
| worst legIK reach at full stride | **0.918** against the 0.995 clamp |
| cycle | 1.976 s, ≈0.148 m/s |

Leg actuators use the **original** shared-support table, not the biped's doubled one, and
the argument is exact rather than estimated: for a rectangular stance the critical edge of
every single-foot triangle is the diagonal between the two remaining feet opposite the
lifted one, and a rectangle's diagonal passes through its own centre. So the worst
single-leg load is bounded at exactly W/2, and 0.455 W at the design margin.

**Expected to fail first:** the four-beat sway. The COM command steps between four discrete
targets and holds still during each swing; a statically stable *pose sequence* is not a
statically stable *trajectory*. Knob is `tShift`. Second candidate is three stance yaw rings
scrubbing against each other — knob is `kSteer`, and 0 restores flat compliant rings and
leaves turning to footstep placement, which on four legs is the strong mechanism anyway.

## 5. Open items, ranked

1. ~~**Body yaw is unregulated.**~~ Addressed in §4e — the stance/swing yaw controller this
   item asked for now exists, at half authority on the stance ring, in single support only.
   Whether it holds the heading well enough to kill the residual lateral drift is open
   until it is driven. If it collapses the walk, `kSteer` is the knob: 0 restores the old
   flat-ring behaviour exactly.
2. **Turning while travelling falls** (~8 s). That was measured when turning was a no-op,
   so the number describes a machine that no longer exists. Re-measure by driving.
3. **Strafe-left fails**; strafe-right works. A clean left/right asymmetry — a sign error
   somewhere, not tuning. Note the waist ring also caps usable strafe: past `waistFollow`
   (30°) off-axis the legs come round to the aim, so aiming and strafing now trade off.
   That is inherent to "legs follow", not a bug, but it is the knob if strafing matters.
3b. **Scout Walker falls at ~15 s** (`s20260727000055` run 1, 4 ft). Not structural — peak
   joint utilisation never exceeded 0.16, and the head break at util 2.2 came 0.13 s AFTER
   the fall started, i.e. it is the head hitting the ground. It is lateral: `dcmZ` sat at
   −0.25 to −0.39 for the entire trace and never converged, then `dcmX` ran away over the
   last 1.5 s. Its lateral CoP authority is **0.062 m against Light Frame's 0.091 m** while
   being considerably more top-heavy, because a canon AT-ST foot is 0.153 of its own height
   where MK1's is 0.224. **There is no baseline** — all 40 prior driving logs are Light
   Frame, so this cannot be attributed to any recent change. The preset was built to canon
   on the stated principle that "where it fails is the answer", and this is where.
4. ~~**AT-AT quadruped not built.**~~ Built 2026-07-27 as **Heavy Walker** (`rig/atat.js` +
   `control/crawl.js`), a static crawl. NOT YET DRIVEN. Design and review notes in 4f.
   Superseded text follows for reference:
   ~~ Needs a new controller: the DCM planner, `plant.L/R`,
   `nextSwing` and the foot-pair collision are all bipedal by construction. A static crawl
   gait (three feet down, COM inside the support triangle) is the right first target.~~
5. **Two-pass mount sizing does not converge.** Sizing each of the four load terms to its
   own measured peak ignores that utilisation is their quadrature sum. A scaling pass was
   added and still came out at 2.75× over. Needs an ensemble upper bound with a fixed
   margin rather than iterate-to-fit.
6. Three.js still loads from a CDN, so "single self-contained file" remains untrue.

---

## 6. Verification notes

Headless Chromium on this Pi can no longer screenshot the page — under software GL it
never goes idle long enough. Verification is currently: both script blocks parsed, the
full page script executed in Node with DOM stubs, and the physics exercised through the
extracted sim module.

**That is weaker than it sounds, and it has already burned us once.** The Node harnesses
replicate the artifact's build path by hand, so they can pass while the artifact fails —
which is exactly what happened with the gyro scaling bug: the harness used the correct
scale all along and never saw it. Test the code path the page actually runs, and take
live driving reports seriously; the 1 ft failure was found in one try by driving it.
