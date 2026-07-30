/* MECH SIM -- bundled from the gated modules. Source of truth: core/, control/, rig/.

   BUILD VERSION, ONE SITE. This string was written at four: preamble, the HUD chip in
   ui/03-sim.js, the session-log header in ui/08-session-logging.js, and the <title> in
   shell-head.html. Four copies of one fact is the bug class this project loses days to,
   and here it had a second cost: a driving log could not be tied to the build that
   produced it, so no two sessions could be compared with any confidence about what
   differed between them.

   Everything downstream reads BUILD / BUILD_TAG. Bump BUILD on every serve. The log header
   carries BUILD_TAG, so `logs/*.jsonl` are now self-identifying and a regression can be
   bisected against the version that introduced it. */
/* BUMP THIS ON EVERY SERVE. It was not bumped across the 2026-07-27 instrumentation change --
   new joint record (7 fields -> 13), new state rate (10 -> 20 Hz), new burst channel, physics
   telemetry members added -- so two logs with incompatible schemas both claimed MK1.11.0 and the
   only way to tell them apart was counting fields. A driving session was then diagnosed against
   the wrong build. The rule above is not decoration. */
const BUILD = 'MK1.42.0';
const BUILD_MODEL = 'fable-5';
const BUILD_TAG = BUILD + '-' + BUILD_MODEL;
const BUILD_TITLE = 'MK1 Live Rig ' + BUILD + ' — Claude Fable 5';

/* Servo damping law. gamma = kd/(kp*h) is the dimensionless group the explicit damping
   term's stability depends on -- `wRel` is frozen across all iterations of a substep while
   kp*e is re-evaluated, so the damping is fully explicit and gamma is what bounds it.

   H_NATIVE is the substep at NATIVE scale, (1/60)/10. It must be applied BEFORE scaleRig:
   scaleRig takes kp by s^4 and kd by s^4*sqrt(s) while h goes as sqrt(s), so the sqrt(s)
   cancels and gamma is preserved exactly at every size. Applied AFTER scaleRig it is not
   preserved -- measured 11.9 / 17.2 / 24.3 at 4/2/1 ft instead of a flat 6.000000. */
const H_NATIVE = (1 / 60) / 10;
const SERVO_GAMMA = 6;
