# Biped Mech Walking Spec — for Coldsnap
### Distilled from mech_model Light Frame, MK1.43.0 (Claude Fable 5)

Everything below was found the expensive way across ~40 driven builds. The five sections
are ordered by how much breaks if you skip them. **The single most important rule is in
§5: no absolute constants anywhere — every number derives from leg length `L`, gravity
`g`, and mass at build time.** That is what makes it scalable.

---

## 1. Control stack — one pass per physics tick

```
state estimate ─► capture point ─► footstep plan ─► swing target ─► ankle CoP ─► IK ─► joint servos
```

1. **State estimate.** Full-body CoM position and velocity (all links, not just the hull).
2. **Capture point.** `ξ = com + comVel/ω`, with `ω = √(g / comHeight)` from the *live*
   CoM height. ξ is where the machine will stop if it plants a foot there. Every balance
   decision reads ξ, nothing else.
3. **Footstep plan.** Plan 4 footprints ahead on a linear-inverted-pendulum (DCM) model.
   **Replan at every touchdown from the measured landing position** — never trust the
   commanded one; feet land short.
4. **Swing-foot target = plan + capture feedback.**
   `foothold += kCapture · (ξ_measured − ξ_planned)`, where `kCapture = 1.0` when the
   hull dominates the mass (§2), lower (0.65) as leg mass fraction grows.
   - **Freeze the correction at 50% of swing** (`capCommit = 0.5`). Chasing the live
     error to touchdown lands the foot with sideways velocity and turns recovery into a
     growing limit cycle — measured, not theoretical.
   - **Deadband**: ignore error below `0.15 × strideCap` so quiet standing never walks.
   - Clamp the correction to one stride cap; clamp separation so feet can never cross
     or over-splay (max separation ≈ 1.40 × nominal stance).
5. **Ankle CoP feedback** (trim only): shift commanded centre-of-pressure by
   `kDCM · (ξ error)`, `kDCM = 2.0`, clamped well inside the physical sole
   (~80% long / 65% lateral of the half-foot). Re-decide the correction only when the
   error moves > 25% of the clamp since last decision (event-triggered — kills chatter,
   never delays a large error).
6. **IK** pelvis+feet → joint angles. Clamp leg extension at 99.5%; command rest pose at
   ≤ 93% extension so there is always knee room.
7. **Joint servos**: `τ = kp·err − kd·ω_rel − kv·ω_rel·|ω_rel| + τ_ff`, hard ceiling
   `τmax`. The quadratic `kv` term is a hydraulic-style damper: invisible at walking
   speeds, crushes fast oscillation.

**Stepping is the primary balance actuator.** A catch step must be fireable from every
state — including standing still: if ξ leaves the region the ankles can reach
(CoP box ⊕ half stance width laterally), launch one step, foot on the side of the fall,
placed by the same capture feedback. Do **not** add a time-based cooldown between catch
steps — tried, it gates the only working recovery during real falls; the magnitude
deadband is the only gate that works.

**Stopping is a manoeuvre**: on stick release, measure ξ once, take ONE closing step
aimed at it (feedforward, no live chasing), then stand. Never just ramp velocity to zero —
that parks the machine mid-lunge on one loaded ankle.

---

## 2. Mass layout — the make-or-break constraint

The planner models the machine as a point mass on a massless leg. It only works if that
is nearly true:

- **Hull (everything that is not leg) ≥ ~60% of total mass.** Our working biped is 65%;
  our failing one is 38% and no controller tuning fixed it — every leg swing shoves the
  CoM in ways the plan cannot see.
- If the design demands heavy legs, either add hull ballast or feed the swing-leg
  reaction force forward into the plan. Do not ship heavy legs on a vanilla LIPM planner.
- Passive appendages (arms, antennas, pods) must NOT be held by stiff position servos —
  they become resonant amplifiers that ring at the torque ceiling and tear their mounts
  (measured: 460–1166 deg/s ring, mounts torn on every fall). Tune them as **passive
  mass dampers** instead: spring so their pendulum frequency ≈ body sway frequency `ω`
  (Den Hartog: target `ω/(1+μ)`, damping ratio `ζ = √(3μ / 8(1+μ)³)`, μ = appendage
  mass / rest of machine). Free stability, and it looks right.
- A dedicated internal absorber works too: bob on a 2-axis gimbal, CoM *above* the pivot
  (inverted — gravity is negative stiffness, so a spring can tune it exactly), ~3% of
  machine mass.

---

## 3. Energy handling

- **Swing profile**: vertical lift = `h · sin²(π·s)`, `s` = swing phase 0→1,
  `h ≈ 0.08 · L`. sin² has zero slope at both ends: the foot lands at zero commanded
  vertical speed. (Plain `sin` lands at MAX downward speed — it reads as a shuffle-slam
  and spikes contact forces 2–3× bodyweight.)
- **Ground is compliant and dissipative**: one bodyweight sinks ~1% of L; damping on
  approach; **zero restitution** (crushed soil / gas accumulator, not a trampoline).
  This is where landing energy goes. Rigid ground rings every impact up the legs into
  the hull.
- **Falls**: the moment attitude is lost, stop the walk controller entirely and go limp —
  pin every servo target to its current angle (damping stays). A walk controller driving
  a machine on its back saturates every joint and tears mounts. Recovery = respawn (or a
  scripted stand-up); never let the gait "walk" a fallen machine.

---

## 4. Command hygiene

**Every command channel is rate-limited, without exception.** Travel vector, heading,
turret/waist aim, pelvis reference, foothold corrections. A position command with no slew
is a step function, and a step is a disturbance the structure must absorb somewhere — our
unslewed waist ring hit its end stop at 37 rad/s and tore both arms and the head off.

- Travel slew ≈ `0.6·√(L/L₀)` (per §5 scaling).
- Turning is ONE parameter: yaw-per-step (5–8°). Turn *rate* = yawPerStep / stepPeriod —
  derive one from the other so command can never outrun delivery.
- Separate "where the turret aims" from "where the chassis walks"; the turret ring slews,
  and the legs follow only after the ring nears its stop.

---

## 5. Scaling — derive everything, hardcode nothing

Froude scaling. Pick scale factor `s` (ratio of leg lengths). A walk scaled this way is
dynamically identical:

| Quantity | Scales as |
|---|---|
| length, stride, step height | s |
| mass | s³ |
| moment of inertia | s⁵ |
| force, weight | s³ |
| torque, kp, spring rates | s⁴ |
| kd (torque per rad/s) | s⁴·√s |
| kv (torque per (rad/s)²) | s⁵ |
| time, gait phases, **physics timestep** | √s |
| angular rates (turn, slew) | 1/√s |
| ground compliance (m/N) | s⁻² |

Practical rules:

1. **The physics/controller timestep scales ×√s.** Small machine = faster clock. Pinning
   the timestep to the display rate is the classic porting failure: the plant speeds up,
   the controller doesn't, and small mechs collapse while big ones work.
2. **Gait timing from the pendulum**: `ω = √(g/comHeight)`; swing ≈ `1.16/ω`;
   double-support ≈ `0.64/ω` (these are the passive coefficients 1.66/0.92 × agility
   0.7 — sweep agility 0.5–1.0 to taste; 0.7 measured best).
3. **All geometry as fractions of L**: stride cap 0.28·L (biped), step height 0.08·L,
   ankle-authority box from foot dimensions, squaring tolerance 0.15 × stance, stance
   width ≤ 0.21·L.
4. **Gains as dimensionless groups**, checked at build time:
   - `γ = kd/(kp·h)` (h = solver substep) must be identical at every scale.
   - **Explicit-damper stability**: `kd·h/I < 1` per joint (2 is divergence; joints past
     it bang-bang the torque ceiling at 9–21 Hz and shake the whole machine). Cap kd per
     joint against its own child-link inertia.
   - Quadratic damper bound: `(kd + 2·√(τmax·kv))·h/I < 2`.
5. **Torque sizing**: each leg joint holds the whole machine on one leg
   (`τmax ≥ m·g·lever`, single-support). Servo stiffness is tuned to a *separate*
   reference torque — raising the ceiling must not stiffen the loop.
6. **Assert, don't document**: every derived constant above should be a build-time check
   that fails loudly. Every bug that cost us days was a number that was wrong when typed
   and had no check.

---

## Minimum viable port, in order

1. Froude-correct build pipeline (§5) — everything derives from L, g, mass.
2. LIPM/DCM plan + capture-point stepping with mid-swing commit (§1.3–4).
3. Hull-dominant mass layout (§2) — this is a *design* constraint on the art/rig.
4. sin² swing + compliant ground + limp-on-fall (§3).
5. Rate limits on every channel (§4).
6. Ankle CoP trim, catch-from-stand, feedforward stop, passive-damper appendages.

Steps 1–5 give a mech that walks and survives driving. Step 6 is the difference between
"walks" and "feels planted".
