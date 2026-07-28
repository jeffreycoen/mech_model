/* ===== rig/build.js =====
   WORLD CONSTRUCTION, ONE SITE.

   This was inlined in ui/03-sim.js buildWorld() and re-implemented, badly, in
   test/battery.mjs. The harness copy omitted `spec`, so build('atst') silently returned an
   MK1 wearing the Scout's kCop; it also hardcoded gravity, defaulted to a display height
   that no longer ships, and had no controller map. Every number the battery ever printed
   for a non-MK1 rig described a different machine.

   `test/harness.mjs` was supposed to have fixed the drift class. It fixed MODULE LOADING.
   World construction is where every historic drift actually happened, and it stayed
   duplicated. This file is that fix: the artifact and every test build a rig through
   buildRig(), or they do not build one.

   Nothing here touches the DOM, three.js or any UI state, so it lives in the sim bundle
   and the harness gets it for free off the manifest. */

const SPECS = { atst: ATST_SPEC, atat: ATAT_SPEC };
const CONTROLLERS = { biped: GaitController, quad: CrawlController };
/* Mount margin. Ideal mode: ordinary driving must not tear a leg, only genuine abuse. */
const FORGIVE = 4;

/* THE ONE DISPLAY HEIGHT THAT SHIPS, metres. It lives here, in the sim bundle, because it
   is a SIM fact and not a UI one: it sets SC, hence SIM_DT and every derived gait constant.
   It was a literal at four sites -- this file, ui/03-sim.js's SIZES, test/manoeuvres.mjs,
   and test/invariants.mjs, which regex-scraped the UI source at runtime with 0.30 hardcoded
   again in its own match fallback, so reformatting one line of UI would have reverted that
   gate silently. The UI still owns SIZES because it owns the label and the size button;
   only the number comes from here, and the harness gets it off the manifest for free. */
const DISPLAY_H = 0.30;

/* Returns everything a caller needs to step the thing:
     { world, rig, gait, cmg, dg, SC, SIM_DT, simSteps, strideCap, envCap, preset }
   opts: { height, cmgOn, subs, iterations } */
function buildRig(key, opts = {}) {
  const P = PRESETS[key] || PRESETS.light;
  const H = opts.height ?? DISPLAY_H;
  const gv = P.gravity !== undefined ? P.gravity : 9.81;
  setGravity(gv);

  /* Assemble once unscaled to learn the native height, then pick the scale that lands this
     preset on the requested display height. */
  const probeW = new World({ substeps: 2, iterations: 1, gravity: V(0, -gv, 0) });
  const probeR = assembleMech(probeW,
    { spec: SPECS[P.spec], footWidth: P.footWidth, hipOffset: P.hipOffset });
  groundRig(probeR);
  const SC = H / rigStats(probeR).height;

  /* Froude: time goes as sqrt(scale), so the CONTROL loop and the physics both run that
     much faster. Pinning them to the 60 Hz display frame is what made small rigs collapse
     -- the plant sped up and the controller did not. */
  const SIM_DT = (1 / 60) * Math.sqrt(SC);
  const simSteps = Math.max(1, Math.round((1 / 60) / SIM_DT));

  /* 10 substeps x 8 iterations. The hip yaw ring makes each leg an 8-link chain and a
     Gauss-Seidel sweep propagates one link per iteration, so the count rises with the
     chain: 6 iterations walks in place but falls travelling, 8 walks. It is iterations
     rather than substeps that matters -- 8 substeps x 8 fails. */
  const world = new World({
    substeps: opts.subs ?? 10, iterations: opts.iterations ?? 8,
    contact: { mu: P.friction !== undefined ? P.friction : 1.0 },
    gravity: V(0, -gv, 0),
  });
  world.lscale = SC;
  const rig = assembleMech(world,
    { spec: SPECS[P.spec], footWidth: P.footWidth, hipOffset: P.hipOffset, gamma: P.gamma });
  groundRig(rig);
  applyPreset(rig, P);
  scaleRig(world, rig, SC);
  groundRig(rig);
  const dg = deriveGait(rig);

  /* ACTUATOR GOVERNOR -- OFF, and it must stay off in this form. Shipped MK1.18.0 at
     tauRateFrac = 3/(tSS+tDS), i.e. full torque in 81 ms, and all three rigs blew apart on spawn.

     Two independent reasons, both of which were checkable before serving it:
     1. BANDWIDTH. The joint servos measure wn = 100-460 Hz, so their periods are 2-9 ms. An 81 ms
        rate limit is 10-40x slower than the loop it sits inside, and a rate limiter below a loop's
        own bandwidth is a standard route to a limit cycle and then divergence. The gyro tolerates
        this same form because it is a slow outer attitude loop; a 460 Hz joint servo cannot.
        A governor fast enough to be safe here -- full range in under 2 ms -- has no effect on a
        9-21 Hz oscillation, so there is no setting of this knob that does the intended job.
     2. COLD START. tauHeld begins at 0, so the machine has no support for the first 81 ms. Free
        fall over that window is 32 mm on a 300 mm rig, which is past legIK's clamp before any
        actuator has authority.
     The mechanism is left in physics.js (Hinge.tauRate, default Infinity) because it is correct
     for a slow loop; nothing in the leg chain is one. `P.tauSlew === true` re-enables it. */
  if (P.tauSlew === true)
    for (const j of Object.values(rig.joints)) j.tauRate = j.tauMax * dg.tauRateFrac;

  for (const j of Object.values(rig.joints))
    j.lim = { tension: j.lim.tension * FORGIVE, shear: j.lim.shear * FORGIVE,
              bend: j.lim.bend * FORGIVE, torsion: j.lim.torsion * FORGIVE };
  for (const wl of Object.values(rig.welds))
    wl.lim = { tension: wl.lim.tension * FORGIVE, shear: wl.lim.shear * FORGIVE,
               bend: wl.lim.bend * FORGIVE, torsion: wl.lim.torsion * FORGIVE };

  /* The gyro is real hardware: mass into the mount body before anything is measured. It
     scales off SC, not off any per-preset scale -- getting that wrong once built it with
     266x its correct torque and a flywheel heavier than the body it was bolted into. */
  const s3 = SC * SC * SC, s4 = s3 * SC, rs = Math.sqrt(SC);
  const cmg = P.cmg ? fitCMG(rig, Object.assign({}, P.cmg, {
    mass: P.cmg.mass * s3, tauMax: P.cmg.tauMax * s4, hMax: P.cmg.hMax * s4 * rs,
    kp: (P.cmg.kp || 150e3) * s4, kd: (P.cmg.kd || 42e3) * s4 * rs,
    // desat is a controller TIME and goes as sqrt(scale) like every other one.
    desat: (P.cmg.desat ?? 7.0) * rs, enabled: opts.cmgOn !== false,
    // Gyro slew is scaled off the gait, so it comes from deriveGait and is not re-derived.
    stepPeriod: dg.tSS + dg.tDS,
  })) : null;

  const Ctl = CONTROLLERS[rig.gait] || GaitController;
  /* Every derived quantity is passed. yawPerStep was once NOT, so the artifact fell back to
     the controller's own 20 deg default while turnRate came from deriveGait -- the same
     rule at two sites, agreeing only because both happened to read 20. */
  const gait = new Ctl(rig, Object.assign({
    gravity: gv, tSS: dg.tSS, tDS: dg.tDS, stepHeight: dg.stepHeight,
    // Crawl timings are the SAME derived quantities under crawl names; the biped ignores
    // the crawl keys and vice versa, which keeps the scale law in one place.
    tSwing: dg.tSS, tShift: dg.tDS, crawlMargin: dg.crawlMargin,
    settleTime: dg.settleTime, crouchTime: dg.crouchTime, tStart: dg.tStart, tEnd: dg.tEnd,
    pelvisDrop: dg.pelvisDrop, minFootSep: dg.minFootSep, copClamp: dg.copClamp,
    travelRate: dg.travelRate, turnRate: dg.turnRate, yawPerStep: dg.yawPerStep,
    waistLimit: dg.waistLimit, waistRate: dg.waistRate, pelvisRate: dg.pelvisRate,
    balance: Object.assign({ kCop: P.kCop !== undefined ? P.kCop : 0.40,
      copLimitX: dg.copLimitX, copLimitZ: dg.copLimitZ }, P.balance || {}),
  }, P.gait || {}));

  /* Stride cap must be the one derived for THIS rig at THIS size. It once fell back to an
     absolute 0.62 m from when the rig was full size -- 12x the working stride at 1 ft, and
     the driving logs show foot mounts tearing a few seconds in at every size. */
  const strideCap = P.strideCap !== undefined ? P.strideCap : dg.strideCap;
  return { world, rig, gait, cmg, dg, SC, SIM_DT, simSteps, preset: P, gravity: gv,
           strideCap, envCap: P.envCap !== undefined ? P.envCap : strideCap * 1.5 };
}
