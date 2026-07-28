/* ===== control/chassis.mjs =====
   The command model, shared by every walking machine regardless of how many legs it has.

   This exists because the recurring bug on this project is ONE RULE WRITTEN AT TWO SITES,
   updated at one. It has been found in `wantMove`, the touchdown gate, the gyro scale, the
   stride cap, `yawPerStep` and `waistRate`. Adding a quadruped controller alongside the
   biped is exactly the situation that breeds another one, so everything both of them share
   -- what the sticks mean, how commands slew, what "the driver wants something" means, what
   the stabiliser holds, and the whole waist/turret ring -- lives here once and is inherited.

   What is NOT here is anything about how feet get placed. That is what actually differs
   between a biped running a DCM plan and a quadruped running a static crawl. */

const CHASSIS_DEFAULTS = {
  gravity: 9.81,       // must match the world; the LIPM frequency depends on it
  enabled: true,
  turnEps: 4 * Math.PI / 180,       // heading error that justifies a step, rad
  turnEpsHold: 14 * Math.PI / 180,  // wider band to LEAVE stand, so it settles
  yawPerStep: 20 * Math.PI / 180,   // most the body can turn in one step, rad
  travelRate: 0.06,                 // m of per-step travel change per second of command
  /* Multipliers on travelRate. 1.0 = the old symmetric behaviour, so these are the two
     numbers to move if launching feels sluggish or stopping feels abrupt. 0.5 doubles the
     time in each case: ~0.7 s from a standstill to full stride, and the same to release. */
  launchRate: 0.5,     // while within launchSteps of a standstill
  launchSteps: 2,      // steps after leaving rest that still count as launching
  releaseRate: 0.5,    // whenever the commanded travel is being reduced
  turnRate: 4 * Math.PI / 180,      // rad/s of facing change
  waistLimit: 50 * Math.PI / 180,   // waist/neck ring travel, rad; comes from the rig
  /* Fraction of the waist travel you may use up before the LEGS come round. Below it the
     torso does the looking and the chassis holds its heading, which is what makes the
     right stick feel like a turret; above it you have run out of ring and the only way to
     keep looking further is to move the feet. */
  waistFollow: 0.6,
  /* Pelvis rise/crouch rate, in units of pelvisDrop per second, shared by both gaits.
     1.5 puts a full stand-up at 0.67 s: slower than a step, so it cannot outrun the gait,
     and fast enough to read as standing up rather than drifting. */
  riseRate: 1.5,
  /* Leg extension to rest at, as a fraction of full leg length. legIK clamps at 0.995;
     the assembled pose measures 0.988. 0.93 buys ~9x the clamp headroom and still stands
     visibly taller than the walking crouch. Asserted by test/invariants.mjs I5. */
  restExt: 0.93,
  /* Waist slew, rad/s, from the rig. A POSITION command with no rate limit is a step, and
     a step across this ring is 100 deg of the machine's heaviest fast-moving assembly
     arriving at an end stop with nowhere to put its momentum.
     Measured, driving log s20260726234708 at t=20.3: aim reversed from +163 to -45, the
     target jumped +50 -> -50 in one frame, the ring crossed in ~0.1 s (~17 rad/s) and
     slammed the opposite stop. torso mount util 1.45, upperArmL 1.27 in the same frame,
     upperArmR 1.04 and the head 1.02 right after -- the arms are the outermost mass on the
     yaw axis, so they pay first. Every other command channel here is rate-limited; this
     one was not. */
  waistRate: 3.5,
  /* How fast the torso squares back up once nobody is driving, as a fraction of the
     commanded slew rate. Deliberately well under 1: relaxing to centre should read as the
     machine settling, not as a second command nobody gave. */
  waistReturn: 0.35,
  /* Stance yaw-ring authority, 0..1. 1.0 asks the planted feet to drive the chassis all
     the way onto the commanded heading in one step. Deliberately below that: the ring is
     pushing against ground friction with the machine's whole weight on it, and the last
     time every ring was driven from measured yaw the walk collapsed in 6 s. */
  kSteer: 0.5,
  /* Ceiling on how fast the commanded body position may move, m/s. Derived (see
     rig/derive.js) as a multiple of the fastest the machine can actually walk, so it never
     binds during normal travel and only ever catches a discontinuity. */
  pelvisRate: 1.5,
  /* Freeze leg length on an unreachable IK target rather than straightening to 0.995. OFF:
     driven 2026-07-27 and turning got worse. See Posture's holdUnreachable for the mechanism
     and for what would have to change to make it safe. */
  holdUnreachable: false,
};

class Chassis {
  constructor(rig, cfg = {}, defaults = {}) {
    this.rig = rig;
    // k first: Posture needs a flag out of it. BalanceController takes cfg.balance directly.
    this.k = Object.assign({}, CHASSIS_DEFAULTS, defaults, cfg);
    this.posture = new Posture(rig, { holdUnreachable: this.k.holdUnreachable,
                                     yawLimit: this.k.hipYawLimit, yawRate: this.k.hipYawRate });
    this.balance = new BalanceController(rig, Object.assign({ hipKp: 0, hipKd: 0 }, cfg.balance || {}));
    this.t = 0;
    this.state = 'INIT';
    this.debug = {};
    /* Command model. `travel` is a per-step displacement VECTOR in world metres and
       `facing` is the yaw the CHASSIS points at -- the two are independent, which is what
       twin-stick means and what the old {stride, heading} pair could not express. With
       only a scalar stride along a single heading there is no way to ask for a strafe.

       Every command starts at ZERO. An older constructor seeded cmd and active from
       k.stride, and callers that wanted a standstill only ever zeroed `want`; `cmd` then
       slewed down from the seed while warm-up ran and the plan latched a real stride
       before the slew reached zero, so the rig walked off with nobody touching it. */
    this.stopping = false;
    this.standing = false;
    /* Where the TORSO is pointed, in world radians. Separate from `want.facing`, which is
       where the LEGS are going. The waist ring is what makes the two independent, and
       keeping them as two fields is the point: aiming is instant and costs nothing,
       walking somewhere is slow and can fall over. */
    this.aim = 0;
    /* True while the driver is actively holding the aim stick. Turning in place with the
       right stick alone is a legitimate command, so "no travel" is not enough to decide
       the machine should stop -- but "no travel AND no aim input" is. */
    this.aimHold = false;
    this.cmd    = { tx: 0, tz: 0, facing: 0 };
    this.active = { tx: 0, tz: 0, facing: 0 };
    // What the UI asks for. `cmd` slews toward it at a rate the gait can absorb; a step
    // change is a disturbance the walk may not survive, so the limiter is a correctness
    // device. Its rate is measured against an adversarial input script, not guessed.
    this.want   = { tx: 0, tz: 0, facing: 0 };
  }

  /* magnitude of the commanded per-step travel, m */
  speedCmd() { return Math.hypot(this.want.tx, this.want.tz); }
  /* CHASSIS heading — the PELVIS, not the torso. They were the same body when the torso
     was welded on; with a waist ring the torso is a turret that can sit 50 deg off where
     the machine is actually pointed, and every rule here (the yawPerStep cap, "am I
     aligned", what the stabiliser holds) is about the chassis. */
  bodyYaw() { const f = qrot(this.rig.bodies.pelvis.q, V(1, 0, 0)); return Math.atan2(-f.z, f.x); }
  footYawMeas(s) { const f = qrot(this.rig.bodies[`foot${s}`].q, V(1, 0, 0)); return Math.atan2(-f.z, f.x); }
  yawCmd() { return Math.abs(wrapPi(this.want.facing - this.bodyYaw())); }
  /* SCALE FIX: the "is the stick pushed" threshold was an absolute 30 mm. A 1 ft rig has
     a total stride range of 58 mm, so more than half of the stick read as stopped. */
  moveEps() { return 0.01 * (this.rig.leg.thigh + this.rig.leg.shin); }
  /* ONE predicate for "the driver wants something". It was written out separately at the
     two sites that need it -- entering the walk and deciding to end it -- and only the
     first got the turn term, so a pure turn started and then closed itself out after a
     few steps with the heading still unreached. */
  wantsMove() {
    /* Hysteresis. The machine delivers only ~4 deg of yaw per step and body yaw is
       unregulated, so a bare 4 deg threshold is re-tripped by the residual the moment it
       stands: close step -> STAND -> still off by more than turnEps -> walk again, for
       ever. It never settles, and every extra shuffle re-rolls the dice on a fall.
       Standing takes a wider band than starting does -- and so does a stop already in
       progress. The closing step itself swings the body several degrees, so the narrow
       band re-opened the walk in the middle of every close and the machine marched in
       place instead of coming to rest. Coming to rest IS the manoeuvre; it gets the wide
       band from the moment the stick is released, not only once it has already stopped. */
    const eps = (this.standing || this.stopping) ? this.k.turnEpsHold : this.k.turnEps;
    return this.speedCmd() > this.moveEps() || this.yawCmd() > eps;
  }
  /* The yaw the attitude stabiliser should hold, as opposed to the yaw the gait is
     planning around. At rest the feet are planted and the body physically cannot turn, so
     feeding it `cmd.facing` leaves it pushing against a heading error it is not allowed to
     work off -- up to the full turnEpsHold band -- grinding yaw torque into the ground
     through the soles of a machine that is supposed to be standing still. Standing, it
     holds the yaw the body actually has. */
  stabiliserYaw() { return this.standing ? this.bodyYaw() : this.cmd.facing; }

  slew(dt) {
    // travel slews as a VECTOR, so a direction change costs the same as a speed change
    let ex = this.want.tx - this.cmd.tx, ez = this.want.tz - this.cmd.tz;
    /* LAUNCH AND RELEASE. travelRate was one symmetric number: 0 -> full stride in 1.4-2.0
       step periods, and exactly the same coming back down, whether the machine was standing
       still or already walking.
       Neither is right. From a standstill both feet are planted, there is no momentum and no
       plan running, so the first stride is the largest disturbance the machine ever absorbs
       -- and it was reached at the same rate as any other. The planner already knows this
       (`_fromStand` buys the first step a 1.6x longer leading double-support phase); the
       command channel did not. Releasing is the mirror: dropping the command quickly leaves
       the COM still travelling with nothing commanded to catch it, which is what the closing
       step then has to absorb on one loaded ankle.
       Both are multipliers on travelRate, so 1.0 is exactly the old behaviour and either can
       be reverted with one number.

       `sinceRest` counts steps since the command was last at zero, NOT stepsTaken: that is
       reset only in buildPlan, and leaving STAND goes through rebuild(), which does not
       reset it -- so the second departure from a standstill would not have counted as one. */
    const cmdMag0 = Math.hypot(this.cmd.tx, this.cmd.tz);
    if (cmdMag0 < 1e-9) this._launchBase = this.stepsTaken || 0;
    const sinceRest = (this.stepsTaken || 0) - (this._launchBase ?? 0);
    const wantMag = Math.hypot(this.want.tx, this.want.tz);
    let rate = this.k.travelRate;
    // Release takes priority: letting go during the first step is still letting go.
    if (wantMag < cmdMag0) rate *= this.k.releaseRate;
    else if (sinceRest < this.k.launchSteps) rate *= this.k.launchRate;
    const e = Math.hypot(ex, ez), dr = rate * dt;
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
       tuning: the per-step cap is the real limiter.
       Cap against MEASURED body yaw, not against another command: the machine delivers
       only about a quarter of the yaw it is told to per step, so capping command-against-
       command let the commanded frame run four steps ahead of where the mech physically
       pointed -- and every placement rule works in that frame. */
    const by = this.bodyYaw(), cap = this.k.yawPerStep;
    const lead = wrapPi(this.cmd.facing - by);
    if (lead > cap) this.cmd.facing = by + cap;
    else if (lead < -cap) this.cmd.facing = by - cap;
  }

  latch() { this.active = { tx: this.cmd.tx, tz: this.cmd.tz, facing: this.cmd.facing }; }

  /* comToPelvis is captured at spawn, facing 0. Anywhere it is added back it must be
     rotated to the CURRENT facing, or the body is commanded off-axis relative to the legs
     after every turn -- a standing bias the balance loop then fights forever. This is one
     of the two world-frame bugs in section 4b, and it applies to any controller that plans
     a COM and commands a body position, which is both of them. */
  comOff() {
    const h = this.active.facing, ch = Math.cos(h), sh = Math.sin(h);
    const c = this.comToPelvis || V();
    return V(c.x * ch + c.z * sh, 0, -c.x * sh + c.z * ch);
  }

  /* Rate-limit the commanded BODY POSITION. Every other command channel on this machine is
     rate-limited -- travel by travelRate, facing by turnRate/yawPerStep, the waist by
     waistRate -- and this one was not, which is the same omission that tore the arms off.

     What it cost, driving log s20260727004023 at t=10.6: the mech was standing still, the
     driver pushed the aim stick, and in ONE 10 Hz sample the pelvis went 0.830 -> 0.962 m
     with vertical velocity -0.01 -> +1.50 m/s, both feet unloading to zero for 0.3 s. It
     peaked at 1.060 -- a 230 mm hop on a 1.25 m machine -- came down and spun out, tearing
     the torso mount at util 1.04.

     The mechanism is a step in the TARGET, not a physical push. STAND commands the pelvis
     over the footprint midpoint; the walk commands it over the tracked COM; and `latch()`
     rotates `comOff` onto the freshly latched facing at the same instant. Those disagree by
     enough that legIK is handed a target beyond `maxExtend`, clamps, and returns a
     STRAIGHT leg -- which is longer than the crouched one it was holding. The servos drive
     to it and the machine launches itself.

     Routing STAND, warm-up and the walk through one limiter also makes them agree by
     construction, so there is no transition to get wrong.

     Y IS LIMITED HERE TOO, at the same pelvisRate, so "every command channel is rate-limited
     without exception" is true of all three axes at one site rather than two of them here and
     the third by convention upstream. It is a BACKSTOP and it does not bind today -- measured
     on the Light Frame, the three things that feed target.y are all ramps already and all
     share one `pelvisY` accumulator, so the transitions between them are continuous:
       warm-up   pelvisDrop over crouchTime          = 15.3 mm / 0.242 s = 63 mm/s
       standHeight/walkHeight  pelvisDrop*riseRate   = 23 mm/s
       full STAND-to-WALK travel                     = 4.9 mm (top is pelvisStart.y-10.4 mm,
                                                       crouch is pelvisStart.y-15.3 mm)
     against a pelvisRate of 619 mm/s. Ten times the fastest legitimate ramp, which is what
     pelvisRate is for -- its own note says it never binds during normal travel and only ever
     catches a discontinuity. Do not tighten it to riseRate: that WOULD bind, and would slow
     the initial crouch by 2.7x.

     WHAT THIS DOES NOT FIX, so nobody reads it as the cure for the hop: the launch above is a
     LEG-LENGTH step, not a reference step. ik.js:47 clamps an unreachable target with
     `if (r > maxR) r = maxR`, i.e. to 0.995 of full leg length -- LONGER than the 0.93 the
     stance leg was holding. That is 11.7 mm of commanded extension on the Light Frame against
     the 3.46 mm that rails a knee, on both legs at once. Rate-limiting the reference changes
     how fast the target gets out of reach; it cannot stop the clamp lengthening the leg once
     it is. */
  bodyRef(target, dt) {
    if (!this.pelvisCmd) { this.pelvisCmd = target; return target; }
    const dx = target.x - this.pelvisCmd.x, dz = target.z - this.pelvisCmd.z;
    const d = Math.hypot(dx, dz), dr = this.k.pelvisRate * dt;
    const s = (d > dr && d > 0) ? dr / d : 1;
    const dy = target.y - this.pelvisCmd.y;
    this.pelvisCmd = V(this.pelvisCmd.x + dx * s,
                       this.pelvisCmd.y + Math.max(-dr, Math.min(dr, dy)),
                       this.pelvisCmd.z + dz * s);
    return this.pelvisCmd;
  }

  /* forward and left unit vectors for the current FACING (yaw about +Y). Stance width and
     the lateral placement correction are body-relative, so they key off facing, never off
     the direction of travel. */
  basis() {
    const h = this.active.facing;
    return { fwd: V(Math.cos(h), 0, -Math.sin(h)), left: V(Math.sin(h), 0, Math.cos(h)) };
  }
  /* NEUTRAL STANCE: stand UP at rest, crouch back down to walk.
     `pelvisY` was set once in each controller's init() to pelvisStart.y - pelvisDrop and
     never moved again, so the machine walked crouched and then rested at exactly the same
     crouch. Measured at rest across all three rigs in s20260727124635: the shin sits 34-41
     deg off its rest angle, so every "standing" pose in every log so far is a squat. The
     drop exists to buy swing clearance and there is nothing for it to buy while every foot
     is planted.
     Rate-limited, like every command channel here without exception. A step to full height
     is a disturbance the legs have to absorb somewhere, and the waist ring is the standing
     lesson on what an unslewed position command costs. riseRate is in units of the drop per
     second, so it scales with the rig and with Froude time.
     Both live on Chassis because the biped and the crawler need the identical rule, and a
     rule written at two sites is the bug this class exists to prevent. */
  /* REST CEILING. Standing all the way up to pelvisStart.y put the legs at 0.972-0.983 of
     full extension, against legIK's 0.995 clamp -- and the assembled pose is already 0.988.
     A leg there has almost no vertical compliance: holding a height at the singularity costs
     enormous joint travel per millimetre and the servo fights itself. Measured when STAND
     first rose to full height: the Scout's standing drift went 11.5 -> 30.8 mm/s and its
     saturated-joint count 10% -> 29%, and the Light Frame's standing |v| went 24 -> 93 mm/s
     while its NET drift fell -- motion with no displacement, i.e. jitter.
     So rest at a target EXTENSION rather than at a height. Measured once, from the assembled
     pose, before the walk has moved anything. */
  restCeiling() {
    if (this._restY !== undefined) return this._restY;
    const r = this.rig, s = r.sides[0], L = r.leg.thigh + r.leg.shin;
    const hip = r.bodies[`hipYoke${s}`].x;
    const ank = r.bodies[`foot${s}`].toWorld(r.ankle);
    const e = Math.hypot(hip.x - ank.x, hip.y - ank.y, hip.z - ank.z) / L;
    // Drop the pelvis by the excess extension. Near-vertical leg, so reach falls ~1:1 with
    // pelvis height; exactness does not matter here, having compliance at all does.
    this._restY = this.pelvisStart.y - Math.max(0, e - this.k.restExt) * L;
    return this._restY;
  }
  standHeight(dt) {
    const dr = this.k.pelvisDrop * this.k.riseRate * dt;
    const top = Math.max(this.restCeiling(), this.pelvisStart.y - this.k.pelvisDrop);
    this.pelvisY = Math.min(top, this.pelvisY + dr);
    return this.pelvisY;
  }
  walkHeight(dt) {
    const dr = this.k.pelvisDrop * this.k.riseRate * dt;
    this.pelvisY = Math.max(this.pelvisStart.y - this.k.pelvisDrop, this.pelvisY - dr);
    return this.pelvisY;
  }

  strideVec() { return V(this.active.tx, 0, this.active.tz); }
  yawQuat() { return qAxisAngle(V(0, 1, 0), this.active.facing); }

  /* WAIST / NECK RING. The right stick aims the upper body and it gets there now -- one
     body, one actuator, no ground contact in the loop. The legs are only dragged into it
     once the ring runs out of travel, which is what separates "looking over there" from
     "going over there". On the quadruped this same ring is the neck. Returns the aim
     error so the caller can report it. */
  updateWaist(dt) {
    const J = this.rig.joints.torso;
    const lim = this.k.waistLimit;
    const py = this.bodyYaw();
    /* THE AIM MAY NEVER LEAD THE CHASSIS BY MORE THAN THE RING CAN EXPRESS.
       Without this the aim is an unbounded absolute heading: a stick flick or a held turn button
       walks it arbitrarily far from the body, `want.facing` is set straight from it past
       waistFollow, and the machine is commanded to a heading it needs eight or more steps to
       reach. It then grinds the stance rings against planted feet for the whole journey.
       Measured, log s20260727234609 (MK1.20.0), all three rigs: every failure carries a
       want.facing lead of 132-174 deg. The Scout tore ankleYokeL, footL and footR off at t=2.21
       IN STAND, feet planted and the whole weight on them, with the command 153 deg away.
       Clamping to waistLimit does not prevent a full turn -- the aim goes to the end of the ring,
       the legs bring the chassis round, and the aim advances again. That is what makes it a
       PLANNED manoeuvre with lag rather than a reactive one: the command can only ever be one
       ring-width ahead of the machine, so holding the button walks the heading round in stages
       the legs can actually deliver. Applied here, once, so the stick, the turn pad and the
       relax-to-centre path below all inherit it. */
    this.aim = py + clamp(wrapPi(this.aim - py), -lim, lim);
    /* Is anyone actually asking for anything? `aimHold` is the right stick being held;
       travel is the left. Neither means the machine should be coming to rest. */
    const driving = this.aimHold || this.speedCmd() > this.moveEps();
    /* Nobody driving: the torso SQUARES UP. Coming to rest means coming to rest -- the aim
       used to be a latch that held whatever angle the stick last saw, so releasing
       mid-look left the machine parked indefinitely with its torso cranked up to 50 deg
       off its own hips, leaning on the waist actuator to hold it there. It relaxes back
       instead, at a fraction of the commanded slew rate so it reads as settling rather
       than snapping. Grab the stick again and `driving` goes true on the same frame. */
    /* Relax on !aimHold, not on !driving. With auto-face gone the left stick no longer
       touches the aim, so "the travel stick is being held" is no longer a reason for the
       turret to stay cranked over -- and if it did stay, the chassis would be commanded
       back to a heading nobody asked for the moment the ring passed waistFollow. Let go of
       the aim stick and the torso squares up to the chassis; that is also what makes
       "release both sticks and come to rest, balanced" true for the turret as well. */
    if (!this.aimHold) {
      const de = wrapPi(this.aim - py), dr = this.k.waistRate * this.k.waistReturn * dt;
      this.aim = Math.abs(de) <= dr ? py : this.aim - Math.sign(de) * dr;
    }
    const e = wrapPi(this.aim - py);
    if (driving) {
      /* The legs get told from the RAW aim, not the slewed ring: if you are looking 120
         deg away, they should be turning now rather than after the waist has travelled.
         Gated on aimHold as well, because this is now the ONLY thing that yaws the
         chassis: without the gate a relaxing turret would drag the legs round behind it. */
      if (this.aimHold && Math.abs(e) > lim * this.k.waistFollow) this.want.facing = this.aim;
    } else {
      /* Both sticks released. The legs LET GO of the heading -- the torso keeps looking
         wherever you left it until it has relaxed back, but the chassis stops chasing.
         Without this the turret went on feeding the legs a target nobody was asking for:
         driving log s20260727000055 run 0, released at t=15.0 with the aim still 125 deg
         off, and the machine walked itself round for another 16 steps and 9.2 s before it
         reached STAND. The other three releases in that run stood in 0, 0 and 2 steps --
         they were the ones where the aim happened to be nearly reached already. */
      this.want.facing = py;
    }
    if (!J) return e;                       // welded torso: no turret to command
    // Slew, do not step. See waistRate above for what stepping this cost.
    const want = clamp(e, -lim, lim), dr = this.k.waistRate * dt;
    J.target += clamp(want - J.target, -dr, dr);
    return e;
  }

  /* Desired WORLD heading for each foot, from which posture derives the ring angle as
     (this - measured chassis yaw). This is the yaw-steering law, and the whole trick is
     that planted and lifted legs get OPPOSITE signs:

       SWING  foot is in the air and carries nothing, so aim it straight at the commanded
              heading. It costs nothing and it pre-positions the foot for the next stance.
       STANCE foot is pinned by friction, so its ring angle IS (foot heading - chassis
              heading). Command it to (foot heading - TARGET) and the servo has nowhere to
              put the error except into the chassis, which rotates onto the target.

     Driving BOTH from measured yaw is what collapsed in 6 s: each leg then absorbs the
     error, neither one corrects it, and the body windmills between them.
     `swing` may be null -- no leg is free, every ring goes flat and compliant, because
     there is no phase-correct answer when everything is planted and they would only fight
     each other through the ground. */
  footYaw(swing) {
    const f = this.active.facing;
    const out = {};
    if (!swing) { for (const s of this.rig.sides) out[s] = f; return out; }
    /* `swing` may be one side or a GROUP of them -- a trot lifts a diagonal pair. Anything
       in the air points straight at the commanded heading; everything planted is driven the
       other way, because a stance ring turns the chassis by pushing against the ground. */
    const up = Array.isArray(swing) ? swing : [swing];
    const e = wrapPi(f - this.bodyYaw());
    /* kSteer PER STANCE RING, not per ring. Every planted foot pushes the chassis the same way, so
       the authority scales with how many are down -- a quadruped with three on the ground gets
       three times a biped's for the same constant. Measured on log s20260727234900: the Heavy
       delivered 9.93 deg of yaw per step against an 8 deg command while the Light Frame reached
       6.06, and the Heavy fell at 5.7 s. Dividing by the stance count makes one constant mean the
       same thing on two, three or four legs instead of meaning "times however many legs are down". */
    const nStance = Math.max(1, this.rig.sides.length - up.length);
    const kS = this.k.kSteer / nStance;
    for (const s of this.rig.sides)
      out[s] = up.indexOf(s) >= 0 ? f : this.footYawMeas(s) - kS * e;
    return out;
  }
}
