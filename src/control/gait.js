/* ===== control/gait.mjs ===== */
// gait.mjs — walking driven by a DCM plan and executed through inverse kinematics.
//
//   DCMPlan      dynamically feasible ZMP + DCM reference (control/dcm.mjs)
//   COMTracker   stable COM trajectory that realises that ZMP
//   Posture      COM/pelvis + foot poses -> joint angles via legIK
//   Balance      ankle centre-of-pressure feedforward, now tracking the planned ZMP
//
// The previous hand-built pelvis pattern failed because it was never dynamically
// feasible: 0.40 m of lateral shift in 0.65 s demands a ZMP 1.8 m outside the foot.





const smooth = (t) => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t));

/* The biped. Command model, waist ring and yaw steering are inherited from Chassis --
   what is here is the part that is genuinely bipedal: a DCM plan, two footprints, and a
   swing/stance alternation. */
class GaitController extends Chassis {
  constructor(rig, cfg = {}) {
    super(rig, cfg, {
      pelvisDrop: 0.25,
      settleTime: 0.4,
      crouchTime: 1.4,
      stride: 0.0,
      nSteps: 6,
      horizon: 4,          // steps planned ahead; replanned every touchdown
      tStart: 1.2, tEnd: 3.0,
      stepHeight: 0.14,
      kDCM: 2.0,           // DCM error feedback, >1 for stable error dynamics
      copClamp: 0.45,      // how far the CoP may deviate from the planned ZMP, m
      trackMeasured: false, // reference-frame IK is correct: leg deflection IS the force
      replan: true,        // rebuild the plan from MEASURED feet at each touchdown
      lateralCorrect: 0.22, // lateral error corrected in the commanded swing target
      centrePull: 0.18,     // how hard the footprint pair is pulled back onto its line
      plantPin: 0.22,      // lateral error corrected in the recorded print at touchdown
      minFootSep: 1.16,    // commanded lateral clearance between prints, m
      /* Hard ceiling on foot separation, as a multiple of nominal stance. The recorded
         failure is at 1.6x -- "a strafe splays the pair to 1.6x nominal stance and tears
         the outboard leg" -- so this sits below it rather than at it. It only binds when
         the commanded lateral stride would open the pair further than this in one step;
         below that the step-out/step-together bound in the swing target governs. */
      /* 1.45 -> 1.30. At 1.45 the Scout tore footL at util 1.141 with the machine UPRIGHT
         (up 0.993, both feet loaded at 1.61 W) -- a structural failure, not post-fall
         impact, and the Heavy tore footRR the same way at 4.68 W. This was ranked the
         highest-risk item on the list when it shipped and it is the one that failed.
         1.30 keeps roughly two thirds of the strafe the widening bought while pulling the
         ceiling well clear of the 1.6x the code records as tearing the outboard leg. */
      splayMax: 1.30,
      /* Fore-aft offset, as a fraction of nominal stance, above which a release takes a
         squaring step instead of resting. 0.15 of a 1.20 m stance is 0.18 m native; the
         two bad releases measured 141 and 147 mm at a 73 mm stance, which is 2x. */
      squareTol: 0.15,
      /* Pelvis rise/crouch rate, in units of pelvisDrop per second. 1.5 puts a full
         stand-up at 0.67 s -- slower than a step, so it cannot outrun the gait, and fast
         enough to read as standing up rather than drifting. */
      riseRate: 1.5,
      tSS: 0.90, tDS: 0.50,
      closeOnStop: true,   // take a squaring-up step when the stick is released
    });
  }


  init(st) {
    const r = this.rig;
    this.pelvisStart = V(r.bodies.pelvis.x.x, r.bodies.pelvis.x.y, r.bodies.pelvis.x.z);
    this.pelvisY = this.pelvisStart.y - this.k.pelvisDrop;
    this.plant = {
      L: r.bodies.footL.toWorld(r.ankle),
      R: r.bodies.footR.toWorld(r.ankle),
    };
    this.comToPelvis = vsub(this.pelvisStart, st.com);   // constant offset, held through the walk
    // Nominal lateral footprint. Without commanding this, each landing inherits the
    // previous one's inward drift, the stance narrows every step and the legs cross over.
    /* FIX: the lateral centre of the footprint pair was computed here and then never
       used. Every placement rule regulated only the SEPARATION between the two prints,
       always relative to the other foot, so where the pair sat sideways was a free
       integrator -- any small asymmetry accumulated and the machine wandered off its line.
       It reads as veering one way going forward and the other going backward, because the
       drift is fixed in the world while the direction of travel is not.
       `centre` is now tracked: advanced by the commanded travel each step, with the
       SAGITTAL component slaved to the measured prints so travel stays honest, and the
       LATERAL component left purely commanded so it cannot drift. */
    this.centre = V((this.plant.L.x + this.plant.R.x) / 2, 0,
                    (this.plant.L.z + this.plant.R.z) / 2);
    this.halfStance = (this.plant.L.z - this.plant.R.z) / 2;
    this.planned = false;
    /* Start AT REST. `standing` was left undefined here, which is falsy, so the first
       update after warm-up fell straight into the walking branch and the machine took two
       zero-length steps on spawn with nobody touching the controls -- then discovered
       there was no command and closed itself out. Standing is the resting state; it is
       left the instant a command arrives. */
    this.standing = true;
    this.stopping = false;
  }

  buildPlan(st) {
    /* SCALE FIX: this floor used to be an absolute 1.0 m. Any rig shorter than about
       1.7 m has its COM below that, so the planner sized its inverted pendulum to a
       height the machine does not have -- at 1 ft the COM is 0.18 m and the planner was
       planning against a pendulum 5.6x too tall, putting omega out by a factor of 2.4.
       It bound even on the 4 ft default (0.75 m clamped to 1.0). */
    const zc = Math.max(1e-4, st.com.y);
    this.plan = new DCMPlan(buildPhases({
      left: this.plant.L, right: this.plant.R,
      stride: this.strideVec(), nSteps: this.k.horizon,
      tDS: this.k.tDS, tSS: this.k.tSS, tStart: this.k.tStart, tEnd: this.k.tEnd,
    }), zc, this.k.gravity);
    this.tracker = new COMTracker(this.plan, st.com);
    this.tPlan = 0;
    this.planned = true;
    this.latch();
    this.stepsTaken = 0;
    this.stepsRemaining = this.k.nSteps;
    this.swingPrev = null;
    this.nextSwing = 'L';
  }

  /* One closing step, then rest. Travel is forced to zero so the DCM plan runs out to its
     terminal double-support phase and brings the COM to a stop over `tEnd` rather than
     leaving it moving when the feet stop. `_closing` tells the swing-target code to aim
     alongside the stance foot instead of advancing from this foot's own last print. */
  closeStep(st) {
    this.cmd.tx = 0; this.cmd.tz = 0;
    this.want.tx = 0; this.want.tz = 0;
    this.latch();
    this.nextSwing = this.nextSwing === 'L' ? 'R' : 'L';
    /* SCALE FIX: this floor used to be an absolute 1.0 m. Any rig shorter than about
       1.7 m has its COM below that, so the planner sized its inverted pendulum to a
       height the machine does not have -- at 1 ft the COM is 0.18 m and the planner was
       planning against a pendulum 5.6x too tall, putting omega out by a factor of 2.4.
       It bound even on the 4 ft default (0.75 m clamped to 1.0). */
    const zc = Math.max(1e-4, st.com.y);
    this.plan = new DCMPlan(buildPhases({
      left: this.plant.L, right: this.plant.R,
      stride: V(), nSteps: 1,
      tDS: this.k.tDS, tSS: this.k.tSS,
      tStart: 0.02, tEnd: this.k.tEnd,
      first: this.nextSwing,
    }), zc, this.k.gravity);
    this.tracker = new COMTracker(this.plan, st.com);
    this.tPlan = 0;
  }

  /* Rebuild the plan from the measured feet and the measured COM, for whatever steps
     are left. Standard practice: replan at every footstep rather than trusting an
     open-loop pattern to stay valid for the whole walk. */
  rebuild(st) {
    this.stepsRemaining = this.k.horizon;      // receding horizon: always plan ahead
    this.latch();
    this.nextSwing = this.nextSwing === 'L' ? 'R' : 'L';
    /* SCALE FIX: this floor used to be an absolute 1.0 m. Any rig shorter than about
       1.7 m has its COM below that, so the planner sized its inverted pendulum to a
       height the machine does not have -- at 1 ft the COM is 0.18 m and the planner was
       planning against a pendulum 5.6x too tall, putting omega out by a factor of 2.4.
       It bound even on the 4 ft default (0.75 m clamped to 1.0). */
    const zc = Math.max(1e-4, st.com.y);
    this.plan = new DCMPlan(buildPhases({
      left: this.plant.L, right: this.plant.R,
      stride: this.strideVec(), nSteps: this.stepsRemaining,
      tDS: this.k.tDS, tSS: this.k.tSS,
      // Replanning happens AT touchdown, which is already the start of double support,
      // so a full leading DS phase there is dead time -- it cost 2*tDS + tSS per step
      // instead of tDS + tSS. Starting from a STANDSTILL is the exception: the COM is
      // parked in the middle and needs a real phase to get moving.
      tStart: this._fromStand ? this.k.tDS * 1.6 : 0.02, tEnd: this.k.tEnd,
      first: this.nextSwing,
    }), zc, this.k.gravity);
    this.tracker = new COMTracker(this.plan, st.com);
    this.tPlan = 0;
  }

  update(st, dt) {
    if (!this.plant) this.init(st);
    /* Before anything else, and unconditionally -- the turret answers the stick during
       warm-up, while standing, mid-swing and after a fall. It is the one control on the
       machine that never has to wait for the gait. */
    this.waistErr = this.updateWaist(dt);
    this.slew(dt);
    /* MEASURED yaw, not the commanded frame. `active.facing` is latched at step
       boundaries and is allowed to lead measured yaw by a full yawPerStep (20 deg); the
       ankle axes follow the BODY. At a 20 deg mismatch about a third of the fore/aft CoP
       error is injected into the lateral ankle and back -- a persistent mis-projection
       during exactly the manoeuvre that still falls. */
    this.balance.facing = this.bodyYaw();
    if (!this.k.enabled || !st.support) { this.balance.update(st, dt); return; }
    this.t += dt;

    // --- warm-up: settle, then ease into the crouch --------------------------------
    const warm = this.k.settleTime + this.k.crouchTime;
    if (this.t < warm) {
      const u = smooth(clamp((this.t - this.k.settleTime) / this.k.crouchTime, 0, 1));
      /* Against the explicit crouch, not against this.pelvisY -- pelvisY is now a live
         value that rises while standing, and reading it here would make the warm-up ease
         toward whatever height the machine last happened to be at. */
      const crouchY = this.pelvisStart.y - this.k.pelvisDrop;
      const y = this.pelvisStart.y + (crouchY - this.pelvisStart.y) * u;
      this.pelvisY = y;
      this.balance.copOverride = null;
      this.balance.update(st, dt);
      // Warm-up: both feet planted, so no phase-correct steering. Rings stay flat.
      this.posture.apply(this.bodyRef(V(this.pelvisStart.x, y, this.pelvisStart.z), dt),
                         this.plant, null, this.yawQuat(), this.footYaw(null), null, dt);
      this.state = 'WARMUP';
      this.debug = { state: this.state, u };
      return;
    }
    if (!this.planned) this.buildPlan(st);

    // ---- STAND -----------------------------------------------------------------------
    // The state machine used to cycle forever, so the mech marched in place with no input
    // and "stop" only meant "take zero-length steps". Standing is now a real state: both
    // feet stay planted and only the balance loop runs. It is entered at a touchdown so
    // we never freeze mid-swing, and left as soon as a command arrives.
    /* FIX: this tested TRAVEL only, so a pure turn command never left STAND -- the mech
       stood still and the heading never changed. Measured: 2 steps in 25 s and the target
       heading never reached, at 1x, 3x, 5x and 8x turn rate. Turning in place had been
       recorded as "clean" purely because standing still cannot fall over. */
    const wantMove = this.wantsMove();
    /* Only a real TRAVEL command aborts a stop in progress. Aborting on any wantsMove()
       meant the yaw residual left over from the turn cancelled every close, so letting go
       of the stick after turning marched in place for ever instead of coming to rest. A
       driver who wants to keep going pushes the stick; a residual does not. A driver still
       asking for facing gets it one step later, out of STAND, which is survivable -- an
       abandoned close is not, because it parks the rig mid-lunge on one loaded ankle. */
    if (this.stopping && this.speedCmd() > this.moveEps()) this.stopping = false;
    if (this.standing) {
      if (wantMove) { this.standing = false; this._fromStand = true; this.rebuild(st); this._fromStand = false; }
      else {
        /* NEUTRAL STANCE, part 1: do not rest in a lunge.
           The closing step only ever fired at a touchdown, so releasing the stick outside
           that window left the pair wherever the walk had them. Measured over four releases
           in s20260727124635: two squared up to a 7 mm fore-aft offset and two rested at
           141 and 147 mm -- on a 300 mm machine that is half its own height, parked on one
           loaded ankle, which is the load case the closing step exists to avoid.
           Squaring by sliding the planted feet would be commanded scrub, so instead take
           the closing STEP that already exists and is already tested. squareTries caps it
           so a close that cannot converge cannot march for ever. */
        const b0 = this.basis();
        const fa = (this.plant.L.x - this.plant.R.x) * b0.fwd.x
                 + (this.plant.L.z - this.plant.R.z) * b0.fwd.z;
        if (Math.abs(fa) > this.k.squareTol * 2 * this.halfStance
            && (this.squareTries || 0) < 2) {
          this.squareTries = (this.squareTries || 0) + 1;
          this.standing = false; this.stopping = true;
          this.closeStep(st);
        } else {
          this.squareTries = 0;
          this.balance.copOverride = null;
          this.balance.update(st, dt);
          const mid = V((this.plant.L.x + this.plant.R.x) / 2, 0, (this.plant.L.z + this.plant.R.z) / 2);
          // Resting. Both feet planted, rings flat and compliant -- steering here would have
          // the two legs scrubbing against each other through the ground while the machine
          // is meant to be standing still.
          this.posture.apply(this.bodyRef(V(mid.x + this.comOff().x, this.standHeight(dt),
                                            mid.z + this.comOff().z), dt),
                             this.plant, null, this.yawQuat(), this.footYaw(null), null, dt);
          this.state = 'STAND';
          this.debug = { state: 'STAND', steps: this.stepsTaken, rise: +(this.pelvisY).toFixed(4) };
          return;
        }
      }
    }

    // --- reference trajectories ------------------------------------------------------
    this.tPlan += dt;
    const t = Math.min(this.tPlan, this.plan.T - 1e-6);
    const ref = this.tracker.step(t, dt);
    const { s, kind } = this.plan.phaseProgress(t);
    const zmpRef = this.plan.zmpAt(t);
    const xiRef = this.plan.xiAt(t);

    // --- swing foot ------------------------------------------------------------------
    const stance = kind.startsWith('SS-') ? kind.slice(3) : null;
    const swing = stance ? (stance === 'L' ? 'R' : 'L') : null;
    if (this.swingPrev && this.swingPrev !== swing) {
      // Touchdown. The foot lands short of the commanded stride, so advancing `plant` by
      // the commanded amount desynchronises the plan from reality and the error compounds
      // every step. Take the MEASURED landing position and replan the remainder.
      const w2 = this.swingPrev;
      const land = this.rig.bodies[`foot${w2}`].toWorld(this.rig.ankle);
      // sagittal from measurement so travel stays honest, lateral pinned to nominal
      // The foot lands roughly 0.1 m inboard of command every step. Correcting only the
      // commanded target cannot overcome that, so also pull the RECORDED print back
      // toward nominal stance width; without this the stance collapses and the legs
      // cross over after about seven steps.
      const bb = this.basis();
      const oth = this.plant[w2 === 'L' ? 'R' : 'L'];
      const sgn = w2 === 'L' ? 1 : -1;
      const latLand = land.x * bb.left.x + land.z * bb.left.z;
      const latOther = oth.x * bb.left.x + oth.z * bb.left.z;
      let corr = ((latOther + sgn * 2 * this.halfStance) - latLand) * this.k.plantPin;
      const after = latLand + corr;
      if (sgn * (after - latOther) < this.k.minFootSep) corr = latOther + sgn * this.k.minFootSep - latLand;
      this.plant[w2] = V(land.x + bb.left.x * corr, land.y, land.z + bb.left.z * corr);
      /* Advance the tracked centre by half the commanded travel -- one foot moves per
         step, so the pair's centre advances by half a stride -- then slave only the
         SAGITTAL axis to where the feet actually are. Leaving the lateral axis commanded
         is what removes the drift; slaving the sagittal is what keeps travel honest when
         the foot lands short. */
      {
        const svc = this.strideVec();
        const c = V(this.centre.x + svc.x * 0.5, 0, this.centre.z + svc.z * 0.5);
        const meas = V((this.plant.L.x + this.plant.R.x) / 2, 0,
                       (this.plant.L.z + this.plant.R.z) / 2);
        const dF = (meas.x - c.x) * bb.fwd.x + (meas.z - c.z) * bb.fwd.z;
        this.centre = V(c.x + bb.fwd.x * dF, 0, c.z + bb.fwd.z * dF);
      }
      this.stepsTaken++;
      this.latch();
      /* Stopping is a manoeuvre, not the absence of one.
         Releasing the stick used to just ramp travel to zero, which leaves the swing foot
         targeting its OWN last print -- a full stride behind the stance foot -- so the rig
         froze mid-lunge and then had to absorb the whole deceleration on one loaded ankle.
         Driving logs caught exactly that: ankleYokeR broke at util 1.035 fifty
         milliseconds after the stick was released, while the rig was still upright.
         Instead, take one CLOSING step that brings the trailing foot alongside the stance
         foot, then stand. */
      if (this.stopping) { this.stopping = false; this.standing = true; this.state = 'STAND'; }
      else if (!this.wantsMove()) {
        if (this.k.closeOnStop) { this.stopping = true; this.closeStep(st); }
        else { this.standing = true; this.state = 'STAND'; }
      }
      else if (this.k.replan) this.rebuild(st);
    }
    this.swingPrev = swing;

    const feet = { L: this.plant.L, R: this.plant.R };
    if (swing) {
      const from = this.plant[swing];
      const lift = Math.sin(Math.PI * s) * this.k.stepHeight;
      const b = this.basis(), sv = this.strideVec();
      const other = this.plant[swing === 'L' ? 'R' : 'L'];
      const sign = swing === 'L' ? 1 : -1;
      // Full stride along the heading from this foot's own last print...
      const baseX = from.x + sv.x, baseZ = from.z + sv.z;
      // ...then correct ONLY the lateral component toward the nominal stance width.
      // Blending both axes together (an earlier version) also corrupts stride length.
      /* Placement relative to the OTHER foot holds the pair together and keeps the body
         straight -- placing each print absolutely about the tracked centre was tried and
         removed that coupling, which let the body yaw 27 deg over a short walk.
         Separation is regulated here; the pair's lateral CENTRE is regulated separately
         below, gently, because it is otherwise a free integrator. */
      const nomX = other.x + sv.x + b.left.x * sign * 2 * this.halfStance;
      const nomZ = other.z + sv.z + b.left.z * sign * 2 * this.halfStance;
      const eLat = (nomX - baseX) * b.left.x + (nomZ - baseZ) * b.left.z;
      let tx = baseX + b.left.x * eLat * this.k.lateralCorrect;
      let tz = baseZ + b.left.z * eLat * this.k.lateralCorrect;
      /* Centre regulation. Small gain on purpose: this competes with the minFootSep clamp
         (nominal separation 1.20 against a clamp at 1.16), so a hard pull gets clamped back
         out on one side only and veers the machine. */
      {
        const cx = (from.x + other.x) / 2, cz = (from.z + other.z) / 2;
        const eC = (this.centre.x - cx) * b.left.x + (this.centre.z - cz) * b.left.z;
        tx += b.left.x * eC * this.k.centrePull;
        tz += b.left.z * eC * this.k.centrePull;
      }
      // Closing step: square up on the stance foot instead of advancing from this foot's
      // own print, which is a full stride behind and would leave the rig parked in a lunge.
      if (this.stopping) { tx = nomX; tz = nomZ; }
      // Never COMMAND a placement the feet cannot physically occupy. The solver now
      // refuses to let them interpenetrate, so a planner that keeps asking just fights
      // the constraint and falls over -- which is what turning did.
      const sepLat = (tx - other.x) * b.left.x + (tz - other.z) * b.left.z;
      const need = sign * this.k.minFootSep;
      if (sign * sepLat < this.k.minFootSep) {
        tx += b.left.x * (need - sepLat);
        tz += b.left.z * (need - sepLat);
      }
      /* Splay bound -- mirror of the pinch clamp above. Without any bound a strafe splays
         the pair to 1.6x nominal stance and tears the outboard leg.
         The old bound was a CONSTANT, max(4*halfStance - minFootSep, minFootSep*1.05),
         which on the Scout is 1.24 m against a 1.20 m nominal separation and a 1.40 m
         stride cap: the swing foot got 0.04 m of lateral travel out of a possible 1.40, so
         97% of the coronal channel was clamped away and a sideways command produced
         essentially nothing. That was survivable only because auto-face turned every
         sideways command into a forwards one before the gait ever saw it. With rotation
         moved to the right stick, this bound IS the strafe.
         So bound it by what was actually asked for instead of by a constant: one step may
         open the pair by the commanded LATERAL stride and the pinch clamp above closes it
         on the next -- step out, step together, which is how a person side-steps. The hard
         ceiling stays below the 1.6x that tore the leg, with margin. */
      const latStride = Math.abs(sv.x * b.left.x + sv.z * b.left.z);
      const maxSep = Math.min(
        Math.max(2 * this.halfStance + latStride,
                 4 * this.halfStance - this.k.minFootSep,
                 this.k.minFootSep * 1.05),
        this.k.splayMax * 2 * this.halfStance);
      if (sign * sepLat > maxSep) {
        tx += b.left.x * (sign * maxSep - sepLat);
        tz += b.left.z * (sign * maxSep - sepLat);
      }
      feet[swing] = V(from.x + (tx - from.x) * smooth(s), from.y + lift,
                      from.z + (tz - from.z) * smooth(s));
    }

    // --- DCM tracking: p_cmd = p_ref + k (xi_meas - xi_ref), stable for k > 1 ---------
    const w = this.plan.omega;
    const xiMeas = V(st.com.x + st.comVel.x / w, 0, st.com.z + st.comVel.z / w);
    const cop = V(
      clamp(zmpRef.x + this.k.kDCM * (xiMeas.x - xiRef.x), zmpRef.x - this.k.copClamp, zmpRef.x + this.k.copClamp),
      0,
      clamp(zmpRef.z + this.k.kDCM * (xiMeas.z - xiRef.z), zmpRef.z - this.k.copClamp, zmpRef.z + this.k.copClamp));
    this.balance.copOverride = cop;
    this.balance.update(st, dt);

    // --- pelvis follows the planned COM ---------------------------------------------
    // walkHeight() slews back down to the crouch: leaving STAND at full height and stepping
    // from there would hand the swing leg the whole drop in one frame.
    const pelvisRef = V(ref.com.x + this.comOff().x, this.walkHeight(dt), ref.com.z + this.comOff().z);
    const pelvisNow = this.rig.bodies.pelvis.x;
    /* `swing` is null in double support, which passes null through and leaves the rings
       flat. Steering only happens in single support, where one foot is pinned and the
       other is free -- the only phase where the two legs can be given opposite jobs. */
    this.posture.apply(this.bodyRef(pelvisRef, dt), feet, this.k.trackMeasured ? pelvisNow : null,
                       this.yawQuat(), this.footYaw(swing), swing ? this.bodyYaw() : null, dt);

    this.state = kind;
    this.debug = {
      state: kind, t, swing, cop, zmpRef, xiRef, xiMeas, comRef: ref.com,
      dcmErrZ: xiMeas.z - xiRef.z, dcmErrX: xiMeas.x - xiRef.x,
      steps: this.stepsTaken,
      reachL: this.posture.last.L?.reach, reachR: this.posture.last.R?.reach,
    };
  }
}