/* ===== control/cmg.mjs =====
   Control moment gyro / reaction wheel assembly.

   This is the ONE piece of assistance in the build, and it is modelled hardware rather
   than a fudge factor: a flywheel assembly carried in the torso that exchanges angular
   momentum with the chassis. It can right the torso without pushing on the ground, which
   is exactly what a walking machine wants when a foot is in the air.

   It is not free, and the three costs are all simulated:
     - MASS. The assembly is added to the torso, so it is carried and it loads the mounts.
     - TORQUE LIMIT. tauMax caps what it can apply, same as any actuator here.
     - MOMENTUM SATURATION. A wheel can only absorb so much angular momentum before it is
       spun up and has nothing left to give. H tracks the stored momentum; once |H| hits
       hMax, torque that would spin it further is simply unavailable. That is the honest
       failure mode and it is why this cannot make the mech unfallable.
   Momentum bleeds off slowly against the ground through the stance legs (desaturation),
   which is what lets it recover authority between disturbances.

   Total angular momentum is still conserved -- the wheel holds whatever the chassis gave
   up, which is what H measures. Nothing here reaches into the solver. */
class CMG {
  constructor(o = {}) {
    this.body = o.body;
    this.tauMax = o.tauMax ?? 45e3;      // N.m
    this.hMax = o.hMax ?? 2.2e4;         // N.m.s of stored momentum
    /* SPINNING ROTOR -- the passive half of a flywheel, which this class never had.
       Everything else here is an ACTIVE torque source: a control law reads attitude and
       commands a torque. A real flywheel also does something with no control law at all --
       it stores angular momentum, and any attempt to tilt the body it is bolted into
       produces a gyroscopic reaction torque -tau = -omega x h automatically, at zero
       latency. That cannot over-react, overshoot or oscillate, because it is not a
       controller; it is the same physics that keeps a spinning top up.

       It was worth adding because the machine's complaint is speed, not weakness: it peaks
       at 1.18 m/s and 1.6 g while the ACTIVE loop already rails at 100% for 11-24% of
       driving. More active authority makes a proportional-on-lean law slam harder. Stored
       momentum instead resists RATE of attitude change, which is the actual problem.

       hMax was already the right order for this and was doing nothing: `ideal` mode skips
       the whole momentum store, so the flywheel's momentum has never had any effect on the
       machine. At the shipped hMax the passive torque at 1 rad/s of tilt is 0.25-0.57x the
       gravity torque tipping the rig at a 5 deg lean; ROTOR_SPIN carries it past 1x, which
       is spinning the same wheel faster rather than making it heavier. */
    /* rotorSpin 0 -- OFF. The implicit integration works: the runaway is gone and the
       passive term stays bounded at 34-268% of the actuator ceiling instead of reaching
       173 840%. It just does not help. Driven at MK1.8.0 the fall rate did not improve,
       and a rotor that contributes torque comparable to the whole actuator during normal
       walking while buying nothing is mass and noise. Safe at any value now if it is ever
       worth revisiting; the integration fix in physics.js stands on its own and is what
       actually needed doing. */
    this.hRotor = o.hRotor ?? (o.hMax ?? 2.2e4) * (o.rotorSpin ?? 0);
    this.kp = o.kp ?? 150e3;             // N.m per rad of lean
    this.kd = o.kd ?? 42e3;              // N.m per rad/s
    this.desat = o.desat ?? 7.0;         // s, momentum bleed time constant
    /* SLEW LIMIT on commanded torque. Every command channel on this machine is rate-limited
       without exception -- travel, facing, the waist ring, the pelvis -- and the gyro was
       the one that was not. It could go from nothing to its full ceiling in a single frame,
       which is what "moves very fast, almost jumping around" measures: 1.18 m/s on a 300 mm
       machine, 1.6 g of lateral acceleration, and the actuator pinned at 100% for 11-24% of
       driving.

       The rate is DERIVED, not chosen: full authority in one step period divided by
       `slewSteps`. Tying it to the gait means it is Froude-correct for free (tauMax goes as
       s^4, the step period as sqrt(s), so the rate goes as s^3.5) and it means the
       stabiliser answers a disturbance on the same timescale the machine moves on --
       slightly faster than it walks, not instantly.
       stepPeriod is handed over from deriveGait by rig/build.js rather than re-derived
       here; the fallback only exists so the class is constructible standalone. */
    this.stepPeriod = o.stepPeriod ?? 0.5;
    this.slewSteps = o.slewSteps ?? 1.5;    // full ceiling in stepPeriod/slewSteps seconds
    this.tauCmd = V();
    this.yawDamp = o.yawDamp ?? 0.5;     // yaw rate damping, fraction of kd
    /* 0.30, swept: 135 deg turn in 4.2 s vs 13.3 s at 0.03, and forward travel IMPROVES
       (4.68 m vs 4.51 m per 25 s) rather than regressing. Before this session a turn
       command never rotated the machine at all. */
    this.yawGain = o.yawGain ?? 0.30;   // yaw steering authority, fraction of kp
    /* IDEAL MODE: full-authority attitude assist. No momentum store, no saturation,
       and it also servos yaw onto the commanded facing. Not physical hardware any
       more -- the stated goal is a tabletop bot that only falls when the maneuver is
       extreme, so the stabiliser is allowed to be as good as the fiction needs. */
    this.ideal = o.ideal !== false;
    /* CAPTURE-POINT ATTITUDE -- OFF BY DEFAULT, and it should stay off. Tried at MK1.6.0
       and it fell all three rigs within ~1 s of the stick being touched, every time, while
       standing was the calmest ever logged (capErr 0.2 mm, gyro 1%, up 1.000).

       Why it fails, and it is not a tuning problem: the lean term is an ATTITUDE error
       driving an ATTITUDE actuator. Capture point is a POSITION error. Feeding it to a
       torque actuator puts an extra integrator in the loop -- the gyro tilts the body, the
       tilt moves the COM, the moving COM grows the capture error -- so it is quiet at rest
       where the error is zero and diverges as soon as anything moves. Measured escalation
       on the Light Frame: capErr 13.5 -> 29 -> 116 -> 179 mm over 1.0 s, gyro pinned at
       100%, up 1.000 -> 0.703, torso and both arm mounts torn.
       The gain conversion was wrong too. kp/h assumed the error would be about lean*h; a
       real walk carries tens of mm of DCM tracking error, which at h = 0.19 m is 6x the
       old torque at 40 mm and 27x at 179 mm.

       The ankles are where prediction belongs and they already have it: CoP is a POSITION,
       so a position error commanding a position is the correct pairing (balance.js:140).
       Kept behind a flag rather than deleted, because the measurement is worth more than
       the code and this is the cheapest way to keep both. */
    this.predictive = o.predictive === true;
    this.capErr = 0;
    this.copRef = null;
    this.targetYaw = undefined;
    this.H = V(); this.tau = V();
    this.enabled = o.enabled !== false;
    this.satFrac = 0; this.tauFrac = 0; this.gyroTau = 0;
  }
  update(st, dt) {
    if (!this.enabled || !this.body) { if (this.body) this.body.tExt = V(); this.tau = V(); return; }
    // Sign convention matches balance.mjs: tipping toward +X is a NEGATIVE rotation about
    // +Z, so the corrective torque about +Z is positive for a positive pitch lean, and the
    // rate term uses -omega rather than raw body rate or the damping becomes anti-damping.
    const pitchRate = -st.torsoRate.z, rollRate = -st.torsoRate.x;
    /* Yaw is measured on the CHASSIS, not on the body this thing is bolted into. Once the
       torso yaws on a waist ring it is a turret, and a stabiliser that reads turret yaw
       spends its authority braking the aim instead of holding the machine's heading.
       Roll and pitch still come from the torso -- the waist frees yaw only. */
    const yawRate = st.pelvisRate ? st.pelvisRate.y : st.torsoRate.y;
    let yawT = this.kd * this.yawDamp * -yawRate;
    if (this.ideal && this.targetYaw !== undefined) {
      let cur;
      if (st.pelvisYaw !== undefined) cur = st.pelvisYaw;
      else { const f = qrot(this.body.q, V(1, 0, 0)); cur = Math.atan2(-f.z, f.x); }
      let e = this.targetYaw - cur;
      while (e > Math.PI) e -= 2 * Math.PI; while (e < -Math.PI) e += 2 * Math.PI;
      yawT = this.kp * this.yawGain * e + this.kd * 0.5 * -yawRate;
    }
    /* PREDICTIVE ATTITUDE. The proportional term used to read st.lean -- where the body IS
       tilted, right now. That is the fastest and strongest actuator on the machine
       responding to a position error with no notion of where the mass is heading, so a
       lean that was already recovering got hit just as hard as one that was diverging: it
       slams, overshoots, sees the opposite lean, slams back. Measured over 38 s of driving
       at MK1.5.1: pelvis peaks of 1 179 mm/s on a 300 mm machine -- four body-heights per
       second -- 1.6 g of lateral acceleration, and the gyro pinned at its ceiling 11-24% of
       the time.

       The ankles have never worked this way. balance.js computes the LIPM capture point,
       xi = com + comVel/omega -- "where the COM comes to rest if we do nothing" -- and
       drives the CoP to it. This is the same quantity, on the actuator that needed it most.
       A body leaning forward but decelerating fast enough has xi behind the support and now
       gets no torque at all, which is most of the thrashing.

       GAIN CONVERSION, not a new number: kp is N.m per RAD of lean and the error is now in
       METRES, and a lean of theta puts the COM theta*h off its support, so kp/h is the same
       authority for the same geometric displacement. Nothing here changes how hard the gyro
       can push -- only what it pushes about. Authority and ideal mode are deliberately
       untouched so this can be judged on its own.

       Airborne (no support polygon) there is no capture point to speak of, so it falls back
       to lean -- attitude control matters most with a foot in the air. */
    let eRoll = st.lean.roll, ePitch = st.lean.pitch, kpEff = this.kp;
    if (this.predictive && st.support && st.comVel) {
      const h = Math.max(1e-4, st.comHeight ?? st.comHeightRaw);
      const w0 = Math.sqrt(G / h);
      /* REFERENCE POINT. Not the support centre. During a normal walk the capture point is
         SUPPOSED to sit ahead of the feet -- that is what walking is -- so measuring against
         the support centre would have the gyro applying a large braking torque throughout
         every step, fighting the gait instead of the imbalance. At a walking speed of
         100 mm/s and omega ~7 rad/s the capture point leads by ~14 mm, which through kp/h
         is roughly 3x the torque the old lean term produced. That is not a stabiliser, it
         is a handbrake.
         balance.js does not have this problem because it tracks the planner's ZMP
         (copOverride) rather than the support centre, and falls back to the support centre
         only when no plan exists. `copRef` is that same reference, handed over by the loop,
         so the gyro's error is "how far is the capture point from where the plan says it
         should be" -- zero through a nominal walk, non-zero only on real imbalance. */
      const refX = this.copRef ? this.copRef.x : st.support.x;
      const refZ = this.copRef ? this.copRef.z : st.support.z;
      // eX is toward +X and pairs with lean.pitch; eZ is toward +Z and pairs with
      // lean.roll -- same sign convention, so this is a substitution, not a re-derivation.
      ePitch = (st.com.x + st.comVel.x / w0) - refX;
      eRoll  = (st.com.z + st.comVel.z / w0) - refZ;
      kpEff = this.kp / h;
      this.capErr = Math.hypot(ePitch, eRoll);
    } else this.capErr = 0;
    let t = V(kpEff * eRoll + this.kd * rollRate, yawT,
              kpEff * ePitch + this.kd * pitchRate);
    const cap = this.tauMax;
    const m = vlen(t);
    if (m > cap) t = vmul(t, cap / m);
    if (!this.ideal) {
      // Saturation: strip any component that would spin the wheel past its momentum store.
      const hn = vlen(this.H);
      if (hn > this.hMax) {
        const n = vmul(this.H, 1 / hn);
        const along = vdot(t, n);
        if (along > 0) t = vsub(t, vmul(n, along));
      }
      this.H = vadd(this.H, vmul(t, dt));
      this.H = vmul(this.H, Math.max(0, 1 - dt / this.desat));   // bleed into the ground
      const hn2 = vlen(this.H);
      if (hn2 > this.hMax) this.H = vmul(this.H, this.hMax / hn2);
    }
    /* Slew the COMMANDED torque. Applied after the ceiling clamp and after saturation, so
       the limit is on how fast the drive can change its output, which is what a real
       gimbal or wheel drive is limited by. */
    {
      const dr = this.tauMax * (this.slewSteps / Math.max(1e-6, this.stepPeriod)) * dt;
      const d = vsub(t, this.tauCmd), dn = vlen(d);
      t = (dn > dr && dn > 0) ? vadd(this.tauCmd, vmul(d, dr / dn)) : t;
      this.tauCmd = t;
    }
    /* Gyroscopic reaction, applied OUTSIDE the actuator ceiling. tauMax is what the drive
       motor can command; this is not commanded by anything -- it is the reaction the mount
       feels because the rotor's momentum vector is being carried around by a rotating body.
       Capping it at tauMax would be capping physics. */
    /* PRECESSION STABILITY -- why this ships at zero.
       tau = -omega x h does zero work and cannot add energy in continuous time; that was
       verified exactly (0.0e+0) before it shipped. It is integrated EXPLICITLY, though --
       physics.js reads omega at the start of the substep -- and explicit Euler on a pure
       precession term does not precess, it spirals outward. Growth per substep is
       sqrt(1 + (h*dt/I)^2), so it needs h*dt/I << 1. Measured as shipped at MK1.7.0:

         rig     mount    I_min      h*dt/I   growth/substep   growth/second
         light   torso    1.05e-4     0.89        1.336           3.8e+305
         atst    pelvis   6.17e-5     0.41        1.081           3.1e+100
         atat    pelvis   1.87e-3     0.03        1.000           3.9

       which is exactly the order the rigs failed in when driven: the Light Frame blew
       apart in 0.7 s with the passive term reaching 173 840% of the actuator ceiling while
       the ACTIVE loop sat at 4-9% and its slew limit worked perfectly. Scout fell at 10-15 s,
       Heavy at 9 s.

       This is the same defect as the servo damping term, which is also explicit (`wRel`
       frozen across the substep) and cost this project a night. Same shape, different term.

       The stable ceiling, h <= 0.045*I/dt, is 20x below the shipped value on the Light
       Frame and 9x below on the Scout -- so a rotor small enough to integrate stably here
       is too small to be worth carrying. Fixing it properly means solving
       (I - dt*[h]x) * omega_new = I * omega_old in the integrator, which is unconditionally
       stable and lets h be as large as the fiction wants. That is a solver change and has
       not been made. */
    /* The rotor is handed to the INTEGRATOR, not applied as a torque here. See the
       h_rotor note in physics.js integrate(): as an external torque this term is explicit
       and detonates; inside the implicit Newton step it is unconditionally stable and
       |omega| is non-increasing by construction. */
    this.body.hRotor = this.hRotor;
    // Telemetry only -- the magnitude the rotor is contributing, |omega x h|.
    this.gyroTau = this.hRotor > 0 ? vlen(vcross(this.body.w, qrot(this.body.q, V(0, this.hRotor, 0)))) : 0;
    this.tau = t;
    this.tauFrac = vlen(t) / cap;
    this.satFrac = this.ideal ? 0 : vlen(this.H) / this.hMax;
    this.body.tExt = t;
  }
}

/* Fit a CMG to an assembled rig. The flywheel assembly is real mass bolted into the
   chosen body, so it is added there and the inertia scaled with it.

   MOUNT. This defaulted to `torso` and was hardcoded, which is right for the bipeds --
   their torso IS the hull, 4 200 kg of it. It is wrong for the Heavy Walker, whose
   `torso` is a 420 kg NECK RING carrying the head: the hull is `pelvis` at 9 400 kg. A
   gyro bolted to the neck would have been reacting against a body 22x lighter than the
   machine, through a hinge that is free in yaw -- so the yaw channel would have spun the
   head and delivered nothing to the chassis. Roll and pitch would have transmitted (the
   ring frees yaw alone) but through a 26 kN.m hinge that the flywheel outweighs.
   Pass `mount` to put it on the body that actually is the machine. */
function fitCMG(rig, o = {}) {
  const b = rig.bodies[o.mount || 'torso'];
  if (!b) throw new Error(`fitCMG: no body '${o.mount || 'torso'}' to mount to`);
  const mass = o.mass ?? 180;
  const k = (b.mass + mass) / b.mass;
  b.mass += mass; b.invMass = 1 / b.mass;
  b.I = b.I.map((v) => v * k); b.invI = m3inv(b.I);
  return new CMG(Object.assign({ body: b }, o));
}
