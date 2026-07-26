# mech_model — working rules

## Overpowering is ALLOWED

**We can overpower this mech. Raise actuator torque freely when a measurement calls for
it.** There is no "don't over-power" rule in this project.

An earlier version of `MK1-status-opus-5-1m.md` asserted one. Claude wrote that line, then
later quoted it back as an established project constraint and used it to argue against
raising torque *before testing*. It is retracted — see the retraction in section 2 of that
doc for the evidence.

What is actually measured (2026-07-26):

- Five of six leg joints saturate at **100%** of available torque during single support;
  stance-foot load drops to **2% of body weight**. The legs are under-powered.
- Widening the balance zone from 60%/45% to 80%/65% of the foot took forward travel from
  0.55 m to 4.47 m per 25 s — about 8×. Extra ankle torque on top changed nothing
  (4.46 m), so the ZONE was binding, not the ankles.
- The one anti-stiffness result that survives is about servo GAINS, not torque ceilings:
  `kp` ×2 and ×4 both measured worse at every gait timing. `kp` and `tauMax` are
  different knobs — do not conflate them.

**Design intent (Jeff): either leg should be able to hold the whole body at any time, the
way a human stands on one leg.** Size the legs for that.

## Method rules

- **Never present unverified artifacts.** Parse-check both `<script>` blocks and run the
  battery before handing a build over.
- **The Node harness is a hand-maintained copy of the artifact's script block 0 and it
  silently drifts.** It has already both (a) passed while the artifact failed, and (b)
  tested code the artifact did not have. After every patch, grep BOTH files for each
  change and compare counts. "The run exited 0" proves nothing.
- **Two harnesses disagreeing about the same baseline means one is misconfigured.**
  Reconcile before shipping a number. Cross-script comparisons are invalid.
- **Single runs are meaningless.** The walk is deterministic but chaotic — a 1.9e-16
  relative parameter change moved a fall from 178 s to 23 s. Use ensembles with nanometre
  jitter and report the spread.
- **Prefer driving logs over harness results** when they disagree. Both leftover-constant
  bugs and the 1 ft immediate-break were found from live driving, never from the harness.
- **Claude's own prior notes are not authority.** Doc claims written by Claude must be
  labelled as such and re-verified before being used to rule out an approach.

## Build

- Artifact: `mech-mk1-live-opus-5-1m.html` (model name + version belong in every artifact
  filename and title).
- Serve: `node logserver.mjs 8080` → `http://<host>:8080/mech-mk1-live-opus-5-1m.html`.
  The server dies with session teardown; restart it.
- Driving logs land in `logs/*.jsonl`.
- The design goal is a fun tabletop machine that only falls on extreme maneuvers.
  **Ideal beats realistic.** Non-physical assists are acceptable when they work.
