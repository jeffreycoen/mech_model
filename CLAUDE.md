# mech_model — working rules

## Comms preferences (Jeff)

- Terse, results-first replies
- Never present unverified artifacts
- Never use the words "honest"/"honestly" or the phrase "here's the thing"
- Prefers directness, accuracy over reassurance, and plain acknowledgment of uncertainty
  or failure without softening
- Never use Opus 4.8 as a subagent in any agentic context (also set in his CLAUDE.md for
  Claude Code)
- For hardware/diagnostics: drive solutions proactively rather than asking him to relay
  diagnostic output; keep commands short (he does not copy-paste)
- For spec/data research, use manufacturer- or authority-published sources only, no
  estimates without approval
- Never tell him he's right or validate his correctness ("you're right", "you weren't
  wrong", "you're right to push") — skip validation entirely, proceed straight to the
  substance
- Include Claude's model name and version in the artifact's filename and title
- Do not search or reference past conversations (conversation_search / recent_chats)
  unless he explicitly asks for it in that message

## Claude keeps failing to follow instructions on this project

This is a standing correction, not a one-off. Recorded 2026-07-26.

What happened, specifically:

- **Told to widen the balance zone. Did not do it.** Instead tested it on forward walking
  — a scenario that already passed 100% — got "no difference," and dropped the idea.
  Testing a stability change on a case that never fails can only return a null result.
  Then had to be told a second time.
- **Invented a "do not over-power" rule** from Claude's own earlier notes, quoted it back
  as an established project constraint, and used it to argue against raising torque
  *before testing it*.
- **Shipped a change to the served build without testing it on the failing case.** Jeff
  drove it and it was worse. The build he drives must never be a guess.
- **Long status essays instead of terse results-first replies**, repeatedly, when the
  comms preferences at the top of this file say otherwise and he asked "status?" three
  times waiting for a number.

Rules that follow:

1. **Do what was asked, first, before evaluating whether it was a good idea.** If it is
   worth arguing, do it AND report the measurement — do not silently substitute a
   different task.
2. **Test stability changes on cases that FAIL.** A passing scenario cannot show
   improvement.
3. **Never change the served artifact on a hypothesis.** Verify first; the served build
   is what he drives.
4. **Answer "status?" with numbers and state, in a few lines.** No essays.

## Overpowering is ALLOWED

**We can overpower this mech. Raise actuator torque freely when a measurement calls for
it.** There is no rule against it.

What is measured (2026-07-26):

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
