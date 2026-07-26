# mech_model

Physics-driven walking mech you drive with twin-stick controls. XPBD solver, every joint a
torque-limited actuator, every mount a failure envelope that can tear.

## Layout

    src/core/      solver + rigid bodies
    src/control/   ik, posture, balance, cmg, dcm, gait
    src/rig/       mech + atst specs, presets, scale/derive rules
    src/ui/        three.js scene, twin sticks, telemetry, logging, self test
    src/manifest.json   build order
    build.mjs      assembles the single self-contained artifact
    test/          harness (builds the sim from the SAME manifest) + battery

## Build and run

    node build.mjs                 # -> mech-mk1-live-opus-5-1m.html
    node logserver.mjs 8080        # serve it; driving logs land in logs/
    node test/battery.mjs          # fwd / strafe / diagonal / reverse-180 / box-drive

The artifact stays ONE self-contained file because it has to be served and driven. The
source does not, and when it was, the same rule kept being implemented twice.
