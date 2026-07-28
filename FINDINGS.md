# Findings — what is true, 2026-07-27

Only what survived independent reproduction or a driving log. **These are Claude's notes and
per CLAUDE.md they are not authority** — re-verify anything used to rule out an approach.

**Units: every figure is AS DRIVEN at 0.30 m unless marked native.** Machines as driven:
Light **972 g**, Scout **316 g**, Heavy **1 452 g**.
`node test/invariants.mjs` prints these, as-driven first — take them from there, not from here.

## Before touching anything

    node build.mjs && node test/load.mjs && node test/invariants.mjs

`load.mjs` evaluates every `<script>` block in ONE shared context — a per-block parse check
cannot see cross-block `const` collisions and shipped a blank page once. `invariants.mjs` is
pure arithmetic, no stepping, always allowed. Bump `BUILD` in `src/core/preamble.js` on every
serve; it reaches the tab title, the HUD and the log header from that one site.

## Shipped and confirmed by driving

**The servo damping fix (MK1.5.0).** γ = kd/(kp·h) was 36 with the damping term integrated
explicitly. Now 6 (Scout 15), plus thigh `kpDeg` 7.5. Confirmed: standing `peakSat` collapsed
0.83→0.07 (Light), 0.97→0.08 (Scout); walking travel 70%→106% and 62%→123%; slip 21→9.5 mm/s.

**Light Frame `kCop` 0.60→0.40 and a contact-force cap (MK1.9.0) — the big one.** `tauFF =
kCop·F·err` multiplied ankle torque by a contact force that was spiking to **13.3×** body
weight, so every hard landing became a shove. Light was moving 199 mm/s against a 4 mm/s
command — **48.5×** — while Scout and Heavy tracked theirs at 0.8× and 0.7×. `forceCap` 1.5 W
bounds it. Result: Light now walks with 0 falls, 0 breaks, `up` 1.000, `cf_max` 0.56–1.21,
gyro 1–16%, peak joint util 0.10.

**Gyro slew limit (MK1.7.1).** Commanded gyro torque was the only command channel without a
rate limit — nothing to ceiling in one frame. Now full authority in `stepPeriod/1.5` ≈ 0.16 s,
derived from `deriveGait`, Froude-correct to 2%.

**Asymmetric launch/release (MK1.10.0).** `travelRate` was one symmetric number. Now
`launchRate` 0.5 (within 2 steps of rest) and `releaseRate` 0.5 (whenever reducing): launch
0.68 s, cruise 0.34 s, release 0.68 s on Light. 1.0 restores the old behaviour.

**One construction site (MK1.5.x).** `src/rig/build.js` `buildRig()` is called by the
artifact, the invariants and the manoeuvre suite. `test/battery.mjs` used to re-implement it,
omit `spec`, and return an MK1 for every preset.

## Tried and REVERTED — do not retry without reading why

**Predictive (capture-point) gyro — MK1.6.0.** Fell all three rigs within ~1 s of stick input
while standing was the calmest ever logged. Attitude error → attitude torque is the correct
pairing; capture point is a POSITION error, and feeding it to a torque actuator adds an
integrator. Code kept behind `predictive:false` in `cmg.js`.

**Passive flywheel as an external torque — MK1.7.0.** τ = −ω × h does zero work, which was
verified exactly, and it still detonated: applied through `tExt` it is integrated EXPLICITLY
and spirals, growing √(1+(h·dt/I)²) per substep. Light ran h·dt/I = 0.89 → 3.8e305/s and hit
**173 840%** of the actuator ceiling in 0.7 s.
*The integration was then fixed properly* (MK1.8.0): the rotor is now a term in the existing
implicit Newton step in `physics.js integrate()`, stable to h·dt/I = 500 with |ω|
non-increasing. It still bought nothing when driven, so `rotorSpin` ships at **0**.
**Lesson: verifying that a term conserves energy says nothing about its discretisation.**

## Open

- **Heavy Walker trot vs crawl — the live decision.** MK1.11.0 put it on diagonal pairs
  (`TROT_ORDER`, FL+RR / FR+RL). Phasing verified correct. It is WORSE than the crawl:
  14.1 falls/min vs 3.5, 7 breaks vs 0, `cf_max` 3.88 vs 1.30 — though on 9 s vs 17 s of
  driving, so the counts are not separable. The mechanism is: two feet on a line through the
  COM has no static margin, and the stance diagonal loses contact entirely (traced: `SWING-FLRR`
  with only the *swinging* foot loaded). Either revert to `CRAWL_ORDER` (one word) or add a
  real double-support overlap at each diagonal handover.
- **Scout falls when turning.** Untouched. FINDINGS below: the DCM reference stalls and
  reverses during turns (`COMTracker` is re-anchored every touchdown, so only step 1 of each
  plan executes).
- **Telemetry is 10 Hz against a ~230 ms step period** — 2.3 samples/step. Contact events
  shorter than 100 ms are aliased. Two of my measurements were artifacts of this and of a
  `comHeight` fallback that changed the quantity's definition mid-log. Raise the rate before
  concluding anything about contact.
- I4 reads 0.97× on every rig: the single-leg torque rule is 3% short once the gyro flywheel
  mass is counted. Left alone deliberately.

## Standing facts

- **Foot placement has zero state feedback.** `GaitController` reads only `st.com`,
  `st.comVel`, `st.support`. `j.saturated` is computed every substep and read by no controller.
- **The one non-physical assist is the gyro in ideal mode** (`cmg.js`, `ideal !== false`): no
  momentum store, no saturation, reactionless torque. Everything else is solved — nothing in
  `src/control/` writes a body position.
- **Single runs are inadmissible** (positive Lyapunov). `test/manoeuvres.mjs` runs ensembles
  with a `kd ×20` positive control that must separate or the run is declared unproven.
- **Scout leg beams** were a/t 280 (3.9 mm wall on a 45 mm section); now 64. Mean density over
  a bounding box is NOT the diagnostic — a hollow beam is supposed to read under water.

## Ruled out — do not re-derive

| claim | verdict |
|---|---|
| "Travel is 50% of command" | averaging artefact; filter for straight segments |
| "The legs are under-powered" | static one-leg hold needs 0.3% of thigh ceiling |
| "The chain is under-converged" | converges by 4 iterations |
| "Substeps 10×8 → 40×2" | collapses the rig; 40×8 unaffordable at 3.91× |
| "Gravity compensation frees headroom" | 1.8 mm; the trunk joint is a yaw ring |
| "`I_eff` probe / the α+β table" | validates on a rod only; negative and non-converging on the rig |
| "The build stands only at 8 iterations" | refuted; monotone in iteration count |
| "Nothing is as dense as water" | wrong lens — hollow shells legitimately are |
