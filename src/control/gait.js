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

class GaitController {
  constructor(rig, cfg = {}) {
    this.rig = rig;
    this.posture = new Posture(rig);
    this.balance = new BalanceController(rig, Object.assign({ hipKp: 0, hipKd: 0 }, cfg.balance || {}));
    this.k = Object.assign({
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
      gravity: 9.81,       // must match the world; the LIPM frequency depends on it
      trackMeasured: false, // reference-frame IK is correct: leg deflection IS the force
      replan: true,        // rebuild the plan from MEASURED feet at each touchdown
      lateralCorrect: 0.22, // lateral error corrected in the commanded swing target
      centrePull: 0.18,     // how hard the footprint pair is pulled back onto its line
      plantPin: 0.22,      // lateral error corrected in the recorded print at touchdown
      turnEps: 4 * Math.PI / 180,   // heading error that justifies a step, rad
      turnEpsHold: 14 * Math.PI / 180,  // wider band to LEAVE stand, so it settles
      yawPerStep: 20 * Math.PI / 180,  // most the body can turn in one step, rad
      minFootSep: 1.16,    // commanded lateral clearance between prints, m (foot width + air)
      travelRate: 0.06,    // m of per-step travel change per second of command
      turnRate: 4 * Math.PI / 180,  // rad/s of facing change
      tSS: 0.90, tDS: 0.50,
      closeOnStop: true,   // take a squaring-up step when the stick is released
      enabled: true,
    }, cfg);
    this.t = 0;
    this.state = 'INIT';
    this.debug = {};
    /* Command model. `travel` is a per-step displacement VECTOR in world metres and
       `facing` is the yaw the body points at -- the two are independent, which is what
       twin-stick means and what the old {stride, heading} pair could not express. With
       only a scalar stride along a single heading there is no way to ask for a strafe.

       Every command starts at ZERO. The old constructor seeded cmd and active from
       k.stride (0.30 in the shipped artifact), and callers that wanted a standstill only
       ever zeroed `want`. `cmd` then slewed down from 0.30 while warm-up ran, and the plan
       latched `active` at roughly 0.19 m before the slew reached zero -- so the rig walked
       off on its own with nobody touching the controls. */
    this.stopping = false;
    this.cmd    = { tx: 0, tz: 0, facing: 0 };
    this.active = { tx: 0, tz: 0, facing: 0 };
    // What the UI asks for. `cmd` slews toward it at a rate the gait can absorb; a step
    // change is a disturbance the walk may not survive, so the limiter is a correctness
    // device. Its rate is measured against an adversarial input script, not guessed.
    this.want   = { tx: 0, tz: 0, facing: 0 };
  }

  /* magnitude of the commanded per-step travel, m */
  speedCmd() { return Math.hypot(this.want.tx, this.want.tz); }
  /* Yaw error between where the FEET are pointed and where the stick wants to face.
     Turning is a stepping manoeuvre -- the feet are planted, so the only way to rotate
     the machine is to pick one up and set it down rotated. */
  bodyYaw() { const f = qrot(this.rig.bodies.torso.q, V(1, 0, 0)); return Math.atan2(-f.z, f.x); }
  yawCmd() { return Math.abs(wrapPi(this.want.facing - this.bodyYaw())); }
  /* ONE predicate for "the driver wants something". It was written out separately at the
     two sites that need it -- entering the walk and deciding to end it -- and only the
     first got the turn term, so a pure turn started and then closed itself out after a
     few steps with the heading still unreached. */
  wantsMove() {
    /* Hysteresis. The machine delivers only ~4 deg of yaw per step and body yaw is
       unregulated, so a bare 4 deg threshold is re-tripped by the residual the moment it
       stands: close step -> STAND -> still off by more than turnEps -> walk again, for
       ever. It never settles, and every extra shuffle re-rolls the dice on a fall.
       Standing takes a wider band than starting does. */
    const eps = this.standing ? this.k.turnEpsHold : this.k.turnEps;
    return this.speedCmd() > this.moveEps() || this.yawCmd() > eps;
  }
  /* SCALE FIX: the "is the stick pushed" threshold was an absolute 30 mm. A 1 ft rig has
     a total stride range of 58 mm, so more than half of the stick read as stopped. */
  moveEps() { return 0.01 * (this.rig.leg.thigh + this.rig.leg.shin); }

  slew(dt) {
    // travel slews as a VECTOR, so a direction change costs the same as a speed change
    let ex = this.want.tx - this.cmd.tx, ez = this.want.tz - this.cmd.tz;
    const e = Math.hypot(ex, ez), dr = this.k.travelRate * dt;
    if (e > dr && e > 0) { ex *= dr / e; ez *= dr / e; }
    this.cmd.tx += ex; this.cmd.tz += ez;
    const dh = this.k.turnRate * dt;
    const eh = wrapPi(this.want.facing - this.cmd.facing);
    this.cmd.facing += Math.max(-dh, Math.min(dh, eh));
    /* The feet only rotate when one is picked up and set down turned, so the commanded
       frame must never lead the PLANTED feet by more than a single step's worth of yaw.
       Without this the command races to the target, the controller reads itself as
       aligned and stops after two steps while the machine still points the old way --
       measured at 3x, 5x and 8x turn rate. This one constraint also replaces turn-rate
       tuning: the per-step cap is the real limiter. */
    /* Cap against MEASURED body yaw, not against another command. The machine delivers
       only about a quarter of the yaw it is told to per step, so capping command-against-
       command let the commanded frame run four steps ahead of where the mech physically
       pointed -- and every placement rule works in that frame. Capping against reality
       makes the command wait for the body. */
    const by = this.bodyYaw(), cap = this.k.yawPerStep;
    const lead = wrapPi(this.cmd.facing - by);
    if (lead > cap) this.cmd.facing = by + cap;
    else if (lead < -cap) this.cmd.facing = by - cap;
  }

  latch() { this.active = { tx: this.cmd.tx, tz: this.cmd.tz, facing: this.cmd.facing }; }

  /* forward and left unit vectors for the current FACING (yaw about +Y). Stance width and
     the lateral placement correction are body-relative, so they key off facing, never off
     the direction of travel. */
  /* comToPelvis is captured at spawn, facing 0. Anywhere it is added back it must be
     rotated to the CURRENT facing, or the pelvis is commanded off-axis relative to the
     legs after every turn -- a standing bias the balance loop then fights forever. */
  comOff() {
    const h = this.active.facing, ch = Math.cos(h), sh = Math.sin(h);
    const c = this.comToPelvis;
    return V(c.x * ch + c.z * sh, 0, -c.x * sh + c.z * ch);
  }
  basis() {
    const h = this.active.facing;
    return { fwd: V(Math.cos(h), 0, -Math.sin(h)), left: V(Math.sin(h), 0, Math.cos(h)) };
  }
  strideVec() { return V(this.active.tx, 0, this.active.tz); }
  yawQuat() { return qAxisAngle(V(0, 1, 0), this.active.facing); }
  /* both feet point where the body faces; the yaw ring is what makes this expressible */
  footYaw() { return { L: this.active.facing, R: this.active.facing }; }

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
      const y = this.pelvisStart.y + (this.pelvisY - this.pelvisStart.y) * u;
      this.balance.copOverride = null;
      this.balance.update(st, dt);
      this.posture.apply(V(this.pelvisStart.x, y, this.pelvisStart.z), this.plant, null, this.yawQuat(), this.footYaw());
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
    if (wantMove && this.stopping) this.stopping = false;   // stick came back mid-close
    if (this.standing) {
      if (wantMove) { this.standing = false; this._fromStand = true; this.rebuild(st); this._fromStand = false; }
      else {
        this.balance.copOverride = null;
        this.balance.update(st, dt);
        const mid = V((this.plant.L.x + this.plant.R.x) / 2, 0, (this.plant.L.z + this.plant.R.z) / 2);
        this.posture.apply(V(mid.x + this.comOff().x, this.pelvisY, mid.z + this.comOff().z),
                           this.plant, null, this.yawQuat(), this.footYaw());
        this.state = 'STAND';
        this.debug = { state: 'STAND', steps: this.stepsTaken };
        return;
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
      /* Splay bound -- mirror of the pinch clamp above. Without it a strafe splays the
         pair to 1.6x nominal stance and tears the outboard leg; the direction that
         happens to pinch instead gets clamped into a survivable step-together gait.
         Bound the splay symmetrically so both directions get the survivable gait. */
      const maxSep = Math.max(4 * this.halfStance - this.k.minFootSep,
                              this.k.minFootSep * 1.05);
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
    const pelvisRef = V(ref.com.x + this.comOff().x, this.pelvisY, ref.com.z + this.comOff().z);
    const pelvisNow = this.rig.bodies.pelvis.x;
    this.posture.apply(pelvisRef, feet, this.k.trackMeasured ? pelvisNow : null, this.yawQuat(), this.footYaw());

    this.state = kind;
    this.debug = {
      state: kind, t, swing, cop, zmpRef, xiRef, xiMeas, comRef: ref.com,
      dcmErrZ: xiMeas.z - xiRef.z, dcmErrX: xiMeas.x - xiRef.x,
      steps: this.stepsTaken,
      reachL: this.posture.last.L?.reach, reachR: this.posture.last.R?.reach,
    };
  }
}