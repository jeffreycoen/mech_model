# mech_model

Physics-driven walking mech you drive with twin-stick controls. XPBD solver, every joint a
torque-limited actuator, every mount a failure envelope that can tear.

## Layout

    src/core/      solver + rigid bodies
    src/control/   ik, posture, balance, cmg, dcm, chassis, gait, crawl
    src/rig/       mech + atst + atat specs, assemble, presets, scale/derive, build
    src/ui/        three.js scene, twin sticks, telemetry, logging, self test
    src/manifest.json   build order — and the sim/ui boundary the harness loads across
    build.mjs      assembles the single self-contained artifact
    test/          harness (builds the sim from the SAME manifest) + the suites below

Files hold what their names say. Specs are data (`rig/mech.js`, `rig/atst.js`); turning
either one into bodies and joints is `rig/assemble.js`. Anything the controller needs is
derived from the rig in `rig/derive.js` and passed in from there — one site, never a
constant re-stated at the call site.

## Build and run

    node build.mjs                 # -> mech-mk1-live-opus-5-1m.html
    node logserver.mjs 8080        # serve it; driving logs land in logs/
    node test/invariants.mjs       # arithmetic gates on the assembled rigs; no stepping
    node test/load.mjs             # every <script> block of the artifact in one context
    node test/manoeuvres.mjs       # ensemble sim suite — STEPS THE PHYSICS, see CLAUDE.md

`test/harness.mjs` builds the sim from `src/manifest.json` and is the ONE loader: every suite
imports `buildSim()` from it, so adding a symbol to the sim is a one-line edit in one place.
The first two commands above are always safe to run; the third is simulation and this project
has a standing rule against reaching for it.

The artifact stays ONE self-contained file because it has to be served and driven. The
source does not, and when it was, the same rule kept being implemented twice.
