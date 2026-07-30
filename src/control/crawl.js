/* ===== control/crawl.mjs =====
   The quadruped. A STATIC crawl: three feet on the ground at all times, and the commanded
   COM kept inside the triangle they make, by a margin, at every instant a foot is up.

   Why static rather than a scaled-down DCM plan. The biped's stability is DYNAMIC -- it is
   falling forward and catching itself, and the DCM planner exists to make that fall
   recoverable. A machine that never leaves its support polygon does not need catching,
   which removes the entire failure mode the biped spends its planner on. What it costs is
   speed: duty factor 3/4 instead of 1/2.

   Command model, waist/neck ring, slew, the yawPerStep cap, the body-position rate limit
   and the stance/swing yaw law are all inherited from Chassis and are not restated. What is
   here is the part that is genuinely quadrupedal: a lift order, a body-shift phase, and
   footstep placement around a tracked chassis centre.

   ONE PIECE OF PHYSICS THAT SHAPES EVERYTHING BELOW. BalanceController zeroes foot-roll
   tauFF whenever nSupport > 1, and this machine is never below three feet down. So the
   ankles contribute NO lateral centre of pressure, ever. The only lateral authority a
   crawler has is where it puts its body -- which is why a whole phase per step is spent
   moving it, and why the shift is not garnish. */

/* LIFT ORDER, derived from this rig's own stance rather than quoted from a textbook.
   Lifting a foot forces the COM inside the remaining triangle, along the inward normal of
   the critical edge -- which for a rectangular stance is always the diagonal between the
   two remaining feet opposite the lifted one, and a rectangle's diagonal passes through its
   own centre. All four single-foot triangles are congruent, so the ORDER cannot buy margin;
   it can only buy body travel, and body travel is the expensive quantity here for the
   reason in the header. Summing |delta n_hat| around a cycle at a = 2.30, b = 1.35:

     RL -> FL -> RR -> FR  (same-side pairs)   6.02 * margin      <-- this
     RL -> FR -> RR -> FL  (diagonal)          7.01 * margin
     FL -> FR -> RL -> RR  (front pair first)  7.45 * margin

   14% cheaper than the diagonal order, and its two cheap transitions are pure fore/aft
   moves at constant lateral offset: sway right, two steps, sway left, two steps. That is
   the classical LH/LF/RH/RF wave crawl, reached from the geometry. */
const CRAWL_ORDER = [['RL'], ['FL'], ['RR'], ['FR']];
/* TROT. Diagonal pairs move together: FL with RR, FR with RL. Two feet down instead of
   three, so this is NOT the static crawl the rest of this file was written around -- the
   support "polygon" is a LINE, and a rectangle's diagonal passes through its own centre,
   so the commanded COM sits exactly ON that line with zero margin either side of it. It is
   neutrally stable in roll about the diagonal and relies on the stabiliser to hold that
   axis, which is the trade: half the steps, twice the speed, no static guarantee.
   Groups are the general form -- a crawl is just groups of one -- so both gaits run through
   the same phase machine and the crawl keeps its guarantee untouched. */
const TROT_ORDER = [['FL', 'RR'], ['FR', 'RL']];

class CrawlController extends Chassis {
  constructor(rig, cfg = {}) {
    super(rig, cfg, {
      /* Every one of these is overwritten from deriveGait in buildWorld. They are here so
         the class is constructible standalone, and the values are the 4 ft figures so a
         bare construction is not wildly wrong. */
      pelvisDrop: 0.35,
      settleTime: 0.4, crouchTime: 1.4,
      tSwing: 0.32, tShift: 0.18,
      stepHeight: 0.19,
      crawlMargin: 0.41,
      /* How hard the tracked chassis centre is pulled onto the measured feet, SAGITTAL AXIS
         ONLY. The lateral axis stays purely commanded. That split is the biped's hard-won
         lesson: regulating only relative quantities leaves the pattern's absolute lateral
         position a free integrator, and it random-walks the machine off its line. */
      centrePull: 0.25,
      /* How much of the commanded landing is written back into the recorded print instead
         of the measured one. The foot lands short and inboard of command every step;
         recording the raw measurement integrates that error into the stance. */
      kLand: 0.30,
      order: CRAWL_ORDER,
    });
    /* Fall back to the spec's own side order if the rig does not use these names. A three-
       or six-legged spec is not supported, but it should degrade to "step them in the order
       the spec lists" rather than throw. */
    /* Accept either a flat list (legacy, one foot per step) or a list of groups. Validate
       that every named foot exists and that each foot appears exactly once per cycle --
       a gait that lifts the same leg twice, or never, is a typo, not a gait. */
    const groups = (this.k.order || []).map((g) => (Array.isArray(g) ? g.slice() : [g]));
    const flat = groups.flat();
    const ok = flat.length === rig.sides.length
            && flat.every((s) => rig.sides.indexOf(s) >= 0)
            && new Set(flat).size === flat.length;
    this.order = ok ? groups : rig.sides.map((s) => [s]);
  }

  init(st) {
    const r = this.rig;
    this.pelvisStart = V(r.bodies.pelvis.x.x, r.bodies.pelvis.x.y, r.bodies.pelvis.x.z);
    this.pelvisY = this.pelvisStart.y - this.k.pelvisDrop;
    this.comToPelvis = vsub(this.pelvisStart, st.com);
    /* Where each foot lives, world metres, ankle pivot. Over rig.sides, not over a
       hardcoded pair -- the whole reason assembleMech now carries `sides`. */
    this.plant = {};
    for (const s of r.sides) this.plant[s] = r.bodies[`foot${s}`].toWorld(r.ankle);
    this.groundY = this.plant[r.sides[0]].y;
    /* NOMINAL footprint, per leg, in the CHASSIS frame: each foot's home is directly under
       its own hip. Read off the rig table rather than written here, so a re-proportioned
       hull moves the stance and nothing in the controller has to know. This is also why the
       biped's "hip offset <= 0.21*L" rule does not bind: no foot ever crosses the midline,
       so legIK's hipRoll is 0 at home and the yoke is never splayed. */
    this.nom = {};
    for (const s of r.sides) {
      const jp = r.table[`hipYaw${s}`].jp;
      this.nom[s] = V(jp[0], 0, jp[2]);
    }
    this.centre = this.footMean();
    this.comCmd = V(this.centre.x, 0, this.centre.z);
    this.stepIdx = 0;
    this.stepsTaken = 0;
    this.lift = null;
    this.phase = 'SHIFT';
    this.tPhase = 0;
    this.squareLeft = 0;
    /* Start AT REST. On the biped this was left undefined -- falsy -- and the machine took
       two zero-length steps on spawn with nobody touching the controls. */
    this.standing = true;
    this.stopping = false;
  }

  /* Mean of the planted ankle positions. Over rig.sides, so it is the pair centre on a
     biped and the polygon centroid here, with no branch. */
  footMean() {
    let mx = 0, mz = 0;
    for (const s of this.rig.sides) { mx += this.plant[s].x; mz += this.plant[s].z; }
    return V(mx / this.rig.legs, 0, mz / this.rig.legs);
  }

  /* THE BODY TARGET. Push `from` inside the triangle left when `lift` comes up, until it
     clears every edge by `crawlMargin`.

     Minimal travel by construction: an edge already clear is not touched, so the machine
     only sways as far as the margin actually demands, and on a rectangular stance that is
     one edge and one push. Three passes over three edges, because fixing one edge can in
     principle push you back across another on a skinny triangle.

     Working off the MEASURED planted points rather than the nominal rectangle is what makes
     this survive turning, where the footprint pattern skews relative to the body and the
     critical edge is no longer the geometric diagonal. */
  /* Signed margin from a world point to the boundary of the CURRENT support polygon --
     the planted feet, excluding whatever `lift` says is (about to be) airborne. Positive
     inside, negative outside. Instrumentation for the capturability light (MK1.40.0):
     a statically-walking machine has no catch step, so once the capture point leaves this
     polygon nothing recovers it -- that IS red on a quad. Points are convex-ordered by
     angle about their centroid because rig.sides order (FL,FR,RL,RR) draws a bowtie. */
  supportMargin(px, pz) {
    const up = this.lift ? (Array.isArray(this.lift) ? this.lift : [this.lift]) : [];
    const pts = [];
    for (const s of this.rig.sides) if (up.indexOf(s) < 0) pts.push(this.plant[s]);
    if (pts.length < 3) return 0;
    const cx = pts.reduce((a, p) => a + p.x, 0) / pts.length,
          cz = pts.reduce((a, p) => a + p.z, 0) / pts.length;
    pts.sort((a, b) => Math.atan2(a.z - cz, a.x - cx) - Math.atan2(b.z - cz, b.x - cx));
    let m = Infinity;
    for (let i = 0; i < pts.length; i++) {
      const A = pts[i], B = pts[(i + 1) % pts.length];
      let nx = -(B.z - A.z), nz = (B.x - A.x);
      const len = Math.hypot(nx, nz); if (len < 1e-9) continue;
      nx /= len; nz /= len;
      if ((cx - A.x) * nx + (cz - A.z) * nz < 0) { nx = -nx; nz = -nz; }
      m = Math.min(m, (px - A.x) * nx + (pz - A.z) * nz);
    }
    return m === Infinity ? 0 : m;
  }

  shiftTarget(lift, from) {
    const pts = [];
    const up = Array.isArray(lift) ? lift : [lift];
    for (const s of this.rig.sides) if (up.indexOf(s) < 0) pts.push(this.plant[s]);
    /* TWO feet down: the support set is a segment, not a polygon, and the edge-margin loop
       below has no interior to push into. The best available command is the point ON the
       segment nearest the target -- for a diagonal trot that is essentially the body centre,
       which is the whole reason the diagonal pairing is the one that works. */
    if (pts.length === 2) {
      const A = pts[0], B = pts[1];
      const dx = B.x - A.x, dz = B.z - A.z, L2 = dx * dx + dz * dz;
      if (L2 < 1e-12) return V(A.x, 0, A.z);
      let t = ((from.x - A.x) * dx + (from.z - A.z) * dz) / L2;
      t = t < 0 ? 0 : t > 1 ? 1 : t;
      return V(A.x + dx * t, 0, A.z + dz * t);
    }
    if (pts.length < 2) return V(from.x, 0, from.z);
    let px = from.x, pz = from.z;
    const m = this.k.crawlMargin;
    for (let pass = 0; pass < 3; pass++) {
      for (let i = 0; i < pts.length; i++) {
        const A = pts[i], B = pts[(i + 1) % pts.length], C = pts[(i + 2) % pts.length];
        let nx = -(B.z - A.z), nz = (B.x - A.x);        // perpendicular to edge AB
        const len = Math.hypot(nx, nz);
        if (len < 1e-9) continue;                        // degenerate stance: skip, do not divide
        nx /= len; nz /= len;
        // orient it toward the third vertex, i.e. into the triangle
        if ((C.x - A.x) * nx + (C.z - A.z) * nz < 0) { nx = -nx; nz = -nz; }
        const d = (px - A.x) * nx + (pz - A.z) * nz;
        if (d < m) { px += nx * (m - d); pz += nz * (m - d); }
      }
    }
    return V(px, 0, pz);
  }

  /* WHERE A FOOT LANDS. The chassis frame is `centre` plus `active.facing`; `nom` is the
     foot's own hip offset rotated into that frame. TURNING IS THIS LINE: as cmd.facing
     slews, basis() rotates and each print goes down on an arc, which walks the whole
     support polygon round. On a quadruped that is far stronger authority than the biped
     has, because all four prints rotate rather than two.

     The lead term puts the print 3/8 of a stride ahead of home. The foot swings a full
     stride in world, but the body advances stride/nLegs during that swing and again on each
     of the nLegs-1 steps this foot sits through, so its HIP-RELATIVE excursion is only
     (nLegs-1)/nLegs of a stride. Leading by half of that centres the excursion: the foot
     lands at +3/8 and lifts at -3/8 instead of running +3/4 to 0. Symmetric is what keeps
     legIK's `reach` inside 0.995 at BOTH ends of stance rather than only one. */
  footTarget(side) {
    const b = this.basis(), sv = this.strideVec(), n = this.rig.legs;
    const nm = this.nom[side], lead = (n - 1) / (2 * n);
    return V(this.centre.x + b.fwd.x * nm.x + b.left.x * nm.z + sv.x * lead,
             this.groundY,
             this.centre.z + b.fwd.z * nm.x + b.left.z * nm.z + sv.z * lead);
  }

  /* Advance the chassis centre one step's worth of travel, then plan the shift that makes
     the next lift safe. TRAVEL ENTERS HERE and nowhere else: the body advances
     stride/nLegs per step because one foot in nLegs moves one stride per step -- the same
     rule as the biped's `centre + svc*0.5`, with nLegs in place of the 2. */
  beginShift() {
    this.latch();
    const sv = this.strideVec(), n = this.rig.legs;
    /* Body advance per step is stride * (feet moved / total feet). One foot in four gives
       stride/4, a diagonal pair gives stride/2 -- the same rule, not a second one, which is
       why a trot covers ground twice as fast for the same stride. */
    const moved = this.order[this.stepIdx % this.order.length].length;
    this.centre = V(this.centre.x + sv.x * moved / n, 0, this.centre.z + sv.z * moved / n);
    this.lift = this.order[this.stepIdx % this.order.length];
    this.shiftFrom = V(this.comCmd.x, 0, this.comCmd.z);
    this.shiftTo = this.shiftTarget(this.lift, this.centre);
    this.phase = 'SHIFT';
    this.tPhase = 0;
  }

  beginSwing() {
    this.liftFrom = {}; this.swingTo = {};
    for (const w of this.lift) {
      this.liftFrom[w] = this.plant[w];
      this.swingTo[w] = this.footTarget(w);
    }
    this.phase = 'SWING';
    this.tPhase = 0;
  }

  /* Touchdown. The commanded COM does NOT move during the swing, so the margin computed at
     the start of the shift holds for the whole time a foot is off the ground -- that
     guarantee is the entire point of splitting the two phases, and anything that creeps the
     body while a foot is up gives it away. */
  endSwing() {
    for (const w of this.lift) this.landFoot(w);
    this.stepsTaken++;
    this.stepIdx++;
    this.endSwingTail();
  }
  landFoot(w) {
    const land = this.rig.bodies[`foot${w}`].toWorld(this.rig.ankle);
    const want = this.footTarget(w);
    /* Blend the measurement toward command. Raw measurement integrates the landing error
       into the stance until the legs converge; raw command ignores where the foot actually
       is and desynchronises the plan from reality. The biped needed both terms too. */
    this.plant[w] = V(land.x + (want.x - land.x) * this.k.kLand, land.y,
                      land.z + (want.z - land.z) * this.k.kLand);
  }
  endSwingTail() {
    /* Slave the SAGITTAL centre to the measured feet; leave the lateral commanded. Travel
       stays honest when feet land short, and the lateral axis cannot random-walk. */
    {
      const b = this.basis(), m = this.footMean();
      const dF = (m.x - this.centre.x) * b.fwd.x + (m.z - this.centre.z) * b.fwd.z;
      this.centre = V(this.centre.x + b.fwd.x * dF * this.k.centrePull, 0,
                      this.centre.z + b.fwd.z * dF * this.k.centrePull);
    }
    this.lift = null;
  }

  update(st, dt) {
    if (!this.plant) this.init(st);
    /* Unconditionally and before anything else: the neck answers the stick during warm-up,
       standing, mid-swing and after a fall. It is the one control that never waits. */
    this.waistErr = this.updateWaist(dt);
    this.slew(dt);
    /* MEASURED yaw. active.facing is latched at step boundaries and may lead the body by a
       full yawPerStep; the ankle axes turn with the BODY, and projecting CoP error through
       the commanded frame injects fore/aft error into the lateral ankle. */
    this.balance.facing = this.bodyYaw();
    if (!this.k.enabled || !st.support) { this.balance.update(st, dt); return; }
    this.t += dt;

    // --- warm-up: settle, then ease into the crouch ---------------------------------
    const warm = this.k.settleTime + this.k.crouchTime;
    if (this.t < warm) {
      const u = smooth(clamp((this.t - this.k.settleTime) / this.k.crouchTime, 0, 1));
      // Explicit crouch, not this.pelvisY -- see the same note in control/gait.js.
      const crouchY = this.pelvisStart.y - this.k.pelvisDrop;
      const y = this.pelvisStart.y + (crouchY - this.pelvisStart.y) * u;
      this.pelvisY = y;
      this.balance.copOverride = null;
      this.balance.update(st, dt);
      // Four feet planted: no phase-correct steering exists, so the rings stay flat.
      this.posture.apply(this.bodyRef(V(this.pelvisStart.x, y, this.pelvisStart.z), dt),
                         this.plant, null, this.yawQuat(), this.footYaw(null), null, dt);
      this.state = 'WARMUP';
      this.debug = { state: this.state, u };
      return;
    }

    /* Only a real TRAVEL command aborts a stop in progress. Aborting on any wantsMove() let
       the yaw residual left over from a turn cancel every close, and the biped marched in
       place for ever instead of coming to rest. */
    if (this.stopping && this.speedCmd() > this.moveEps()) {
      this.stopping = false; this.squareLeft = 0;
    }

    // --- STAND ----------------------------------------------------------------------
    if (this.standing) {
      if (this.wantsMove()) { this.standing = false; this.beginShift(); }
      else {
        const mid = this.footMean();
        this.comCmd = mid;
        this.centre = mid;
        this.balance.copOverride = V(mid.x, 0, mid.z);
        this.balance.update(st, dt);
        const off = this.comOff();
        // Stand UP at rest, same rule and same rate as the biped -- Chassis owns both.
        this.posture.apply(this.bodyRef(V(mid.x + off.x, this.standHeight(dt), mid.z + off.z), dt),
                           this.plant, null, this.yawQuat(), this.footYaw(null), null, dt);
        this.state = 'STAND';
        this.debug = { state: 'STAND', steps: this.stepsTaken, rise: +(this.pelvisY).toFixed(4) };
        return;
      }
    }

    // --- phase machine ---------------------------------------------------------------
    this.tPhase += dt;
    let swing = null;
    const feet = {};
    for (const s of this.rig.sides) feet[s] = this.plant[s];

    if (this.phase === 'SHIFT') {
      const u = smooth(clamp(this.tPhase / this.k.tShift, 0, 1));
      this.comCmd = V(this.shiftFrom.x + (this.shiftTo.x - this.shiftFrom.x) * u, 0,
                      this.shiftFrom.z + (this.shiftTo.z - this.shiftFrom.z) * u);
      this.state = 'SHIFT-' + this.lift.join('');
      if (this.tPhase >= this.k.tShift) this.beginSwing();
    }

    if (this.phase === 'SWING') {
      swing = this.lift;
      const u = clamp(this.tPhase / this.k.tSwing, 0, 1), e = smooth(u);
      // Profile lives in chassis.js swingLift() -- one site, shared with gait.js.
      for (const w of this.lift) {
        const a = this.liftFrom[w], b = this.swingTo[w];
        feet[w] = V(a.x + (b.x - a.x) * e,
                    a.y + swingLift(u, this.k.stepHeight),
                    a.z + (b.z - a.z) * e);
      }
      this.state = 'SWING-' + this.lift.join('');
      if (this.tPhase >= this.k.tSwing) {
        this.endSwing();
        swing = null;
        /* COMING TO REST, guaranteed by construction rather than by a convergence test that
           might never fire. On release the machine keeps crawling at ZERO travel for exactly
           rig.legs more steps; with strideVec zero, footTarget puts every foot back at its
           own home under its own hip, so those steps square the stance up, and then it
           stands unconditionally whatever the geometry looks like. A quadruped can do this
           where the biped cannot, because it is statically stable at every instant of the
           squaring -- there is no equivalent of the biped's "parked mid-lunge on one loaded
           ankle" state to escape from.
           `stopping` also widens the wantsMove() hysteresis band from the moment the stick
           is released, so the closing steps' own body swing cannot re-open the walk. */
        if (this.stopping) {
          this.squareLeft--;
          if (this.squareLeft <= 0) {
            this.stopping = false;
            this.standing = true;
            this.state = 'STAND';
            this.balance.copOverride = null;
            this.balance.update(st, dt);
            this.debug = { state: 'STAND', steps: this.stepsTaken };
            return;
          }
        } else if (!this.wantsMove()) {
          this.stopping = true;
          this.squareLeft = this.rig.legs;
          this.want.tx = 0; this.want.tz = 0;
          this.cmd.tx = 0; this.cmd.tz = 0;
        }
        this.beginShift();
      }
    }

    // --- balance, then posture. ORDER MATTERS -----------------------------------------
    /* STATIC equilibrium is the definition: zero horizontal COM acceleration means the
       centre of pressure sits directly under the centre of mass. So the CoP command is
       simply the COM command, and the ankle loop and the leg IK are consistent by
       construction -- there is no dynamically-feasible-ZMP problem to solve, which is the
       whole of what the biped's DCMPlan and COMTracker are for.

       BalanceController must run BEFORE posture. It writes thigh/hipYoke/shin TARGETS
       (which posture then overwrites, correctly) and ankle tauFF (which it does not). Run
       the other way round and the balance loop would flatten the IK solution every frame. */
    this.balance.copOverride = V(this.comCmd.x, 0, this.comCmd.z);
    this.balance.update(st, dt);

    const off = this.comOff();
    // walkHeight(): slew back to the crouch on leaving STAND, same rule as the biped.
    const pelvisRef = V(this.comCmd.x + off.x, this.walkHeight(dt), this.comCmd.z + off.z);
    /* footYaw(swing): the swing ring points its unloaded foot straight at the commanded
       heading; the three stance rings are driven the other way, so pushing against planted
       feet rotates the hull onto the target. Passing null in SHIFT is deliberate -- with
       four feet planted there is no phase-correct answer and the rings would only scrub
       against each other through the ground. */
    this.posture.apply(this.bodyRef(pelvisRef, dt), feet, null, this.yawQuat(),
                       this.footYaw(swing), swing ? this.bodyYaw() : null, dt);

    this.debug = {
      state: this.state, phase: this.phase, lift: this.lift, swing,
      comCmd: this.comCmd, centre: this.centre, steps: this.stepsTaken,
      stopping: this.stopping, squareLeft: this.squareLeft,
      reach: this.posture.last[this.rig.sides[0]] && this.posture.last[this.rig.sides[0]].reach,
    };
  }
}
