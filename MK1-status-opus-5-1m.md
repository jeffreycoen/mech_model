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

**Controls are twin-stick.** Left stick is a travel VECTOR (any direction), right stick is
FACING, and the two are independent. Camera stays on 24 isometric detents, driven by the
on-screen pad or arrow keys.

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
| CoP limit fore/aft | `0.60 ×` foot forward extent from the ankle |
| CoP limit lateral | `0.45 ×` foot half width |
| **hip offset** | **`≤ 0.21 · L`** — 0.20 walks; 0.27 and 0.33 both fail outright |
| actuator vs mount | `tauMax / torsion limit = 0.73` |
| every controller TIME | scales as `√L` |

Actuator sizing, as a fraction of `m·g·L`: hipYaw 0.25, hipYoke 0.29, thigh 0.39,
shin 0.39. Ankles size off the CoP authority they must produce: `1.40 · W · copLimitX`
and `1.45 · W · copLimitZ`.

**Servo gains:** `kp = tauMax / 3°`, `kd = 0.06 kp`. The invariant is the SATURATION
ANGLE. An inertia-based law (`kp = ωₙ²I`) was tried and measured strictly worse — 0/3 on
two rigs where this form is 3/3 — because it lets the saturation angle vary per joint.

**Torque ceilings: raise them freely when a measurement calls for it.**

Measured 2026-07-26: five of six leg joints saturate at **100%** of available torque
during single support, and stance-foot load falls to **2% of body weight**. The legs are
under-powered — the mech cannot stand on one leg.

Design intent: **either leg holds the whole body at any time, like a human standing on
one leg.** Size the legs for that.

Separately and not to be conflated: servo GAIN (`kp`) ×2 and ×4 both measured worse at
every gait timing. That is about position-tracking stiffness, not the torque ceiling.

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

## 5. Open items, ranked

1. **Body yaw is unregulated.** The hip yaw rings are commanded flat, so the feet track
   the COMMANDED frame while the pelvis's actual yaw wanders with nothing pulling it back.
   This is the residual ~2.5% lateral drift and almost certainly the cause of the two
   failures below. Needs a stance/swing yaw controller: correcting yaw and absorbing it
   need OPPOSITE signs on the planted and swinging legs. Setting both legs to track
   measured pelvis yaw was tried and collapses everything inside 6 s.
2. **Turning while travelling falls** (~8 s). Turning in place is fine.
3. **Strafe-left fails**; strafe-right works. A clean left/right asymmetry — a sign error
   somewhere, not tuning.
4. **AT-AT quadruped not built.** Needs a new controller: the DCM planner, `plant.L/R`,
   `nextSwing` and the foot-pair collision are all bipedal by construction. A static crawl
   gait (three feet down, COM inside the support triangle) is the right first target.
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
