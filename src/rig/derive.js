/* ---------- scale + derived gait ----------
   Froude scaling. Lengths x s, masses x s^3, inertias x s^5, forces x s^3, moments x s^4.
   Time goes as sqrt(s), so damping (torque per rad/s) picks up an extra sqrt(s) and turn
   rates go as 1/sqrt(s). A walk scaled this way is dynamically identical -- framed to fill
   the screen it looks the same -- but everything happens sqrt(1/s) times faster in real
   seconds, which is the whole point of shrinking it. */
function scaleRig(w,rig,s){
  if(!s||s===1) return;
  const s3=s*s*s, s4=s3*s, s5=s4*s, rs=Math.sqrt(s);
  for(const b of Object.values(rig.bodies)){
    b.x=vmul(b.x,s); b.dim=vmul(b.dim,s); b.half=vmul(b.half,s);
    b.mass*=s3; b.I=b.I.map(v=>v*s5);
    if(!b.kinematic){ b.invMass=1/b.mass; b.invI=m3inv(b.I); }
  }
  for(const j of Object.values(rig.joints)){
    j.ra=vmul(j.ra,s); j.rb=vmul(j.rb,s);
    j.tauMax*=s4; j.kp*=s4; j.kd*=s4*rs; if(j.kv) j.kv*=s5;
    j.limitCompliance/=s4;
    j.lim={tension:j.lim.tension*s3, shear:j.lim.shear*s3,
           bend:j.lim.bend*s4, torsion:j.lim.torsion*s4};
  }
  for(const wl of Object.values(rig.welds)){
    wl.ra=vmul(wl.ra,s); wl.rb=vmul(wl.rb,s);
    wl.lim={tension:wl.lim.tension*s3, shear:wl.lim.shear*s3,
            bend:wl.lim.bend*s4, torsion:wl.lim.torsion*s4};
  }
  for(const pr of w.pairs) pr.margin*=s;
  for(const L of Object.values(rig.table)){
    if(L.jp) L.jp=[L.jp[0]*s,L.jp[1]*s,L.jp[2]*s];
    if(L.jc) L.jc=[L.jc[0]*s,L.jc[1]*s,L.jc[2]*s];
  }
  rig.ankle=vmul(rig.ankle,s);
  rig.leg={thigh:rig.leg.thigh*s, shin:rig.leg.shin*s};
}

/* ---------- arm tuned-mass-damper plan (MK1.37.0) ----------
   The Light Frame's arms are passive pendulum masses -- no controller ever writes their
   targets -- held by stiff position servos. Driving logs show them as the machine's
   oscillation AMPLIFIER: upperArm ringing at 460-1166 deg/s with the servo saturation
   flag pinned, upperArm mounts torn in every recent fall, the reaction pumping the torso
   the driver sees shaking. A stiff servo on a passive pendulum is a tuned mass damper
   installed backwards.

   So tune it forwards, Den Hartog: mass ratio mu = arm mass / rest-of-machine mass,
   target frequency wT = wSway/(1+mu), damping zeta = sqrt(3*mu/(8*(1+mu)^3)). Per joint,
   the spring that puts a hanging pendulum at wT is kp = wT^2*I - m*g*lc (gravity already
   provides m*g*lc of stiffness about the hang pose; angle0 is within 8 deg of it), and
   kd = 2*zeta*wA*I at each joint's ACHIEVED frequency. The shoulder is planned with the
   WHOLE arm (upper+fore, parallel-axis about the shoulder) as its pendulum; the elbow
   with the forearm alone. Both are AIMED at wT, but a spring can only add stiffness and
   gravity alone already puts both above it, so kp clamps to 0 on this rig and the two
   joints land at DIFFERENT achieved frequencies (shoulder ~2x wT, elbow ~3x -- I14
   prints both). The arm therefore does NOT swing as one tuned unit; it is two damped
   pendulums, each dissipating at its own frequency. The drive decides whether that
   absorbs enough; this file only makes the arithmetic one-site (buildRig APPLIES this
   plan; invariant I14 checks it against an independent re-implementation).

   Computed AFTER scaleRig on the driven rig, all quantities as driven. kd here lands far
   BELOW the kd*h/I explicit-damper cap (zeta ~0.2 at wT ~ a few rad/s against a cap set
   by substep rates), and I8 sweeps the result anyway. */
function deriveArmTMD(rig, g2){
  if(!rig.joints.upperArmL) return null;
  g2=g2??9.81;
  const plan={};
  let mach=0; for(const b of Object.values(rig.bodies)) mach+=b.mass;
  /* Sway frequency from the rig AS IT IS NOW -- not dg.omega. deriveGait runs before
     fitCMG bolts the flywheel in, so dg.omega is the sway of a machine missing 2-5% of
     its mass; mu here was already summed post-flywheel, and mixing the two put wT 1.24%
     off on the Scout (review finding, MK1.38.0). One rule: every input to this plan is
     read at call time from the same rig. */
  const om=Math.sqrt(g2/Math.max(1e-4,rigStats(rig).com.y));
  const armM=['upperArmL','foreArmL','upperArmR','foreArmR']
    .reduce((a,n)=>a+rig.bodies[n].mass,0);
  /* mu here is a raw mass ratio; Den Hartog's is strictly a translational modal-mass
     ratio, so for a rotational pendulum absorber this OVERSTATES coupling somewhat and
     zeta/wT inherit the approximation. Accepted: zeta enters as a broad optimum and the
     drive is the arbiter. Flagged by review, recorded rather than hidden. */
  const mu=armM/(mach-armM);
  const wT=om/(1+mu), zeta=Math.sqrt(3*mu/(8*Math.pow(1+mu,3)));
  /* Inertia about the hinge axis via the REAL quadratic form (assemble.js inertiaAbout).
     The first cut indexed the flattened 3x3 as if it were a 3-vector -- b.I[2] is Ixz, an
     off-diagonal that is identically 0 for every box body -- so every arm segment's own
     rotational inertia silently vanished from the sums and only the parallel-axis
     point-mass terms survived: kd 4.7% low at the shoulder, 15.5% at the elbow. Caught by
     the MK1.37.0 review; I14 could not see it because apply and check shared this same
     function, which is why I14 now re-derives everything through a second, deliberately
     independent implementation. */
  const axisI=(b)=>inertiaAbout(b, V(0,0,1));
  for(const side of ['L','R']){
    const up=rig.bodies['upperArm'+side], fo=rig.bodies['foreArm'+side];
    const shoulder=rig.joints['upperArm'+side], elbow=rig.joints['foreArm'+side];
    // World anchors at the assembled rest pose (joint anchor local to the CHILD body,
    // pushed to world); lever = CoM distance perpendicular to the hinge axis (z), i.e.
    // in the x-y plane, which is where a hanging arm swings.
    const anchS=up.toWorld(shoulder.rb);
    const anchE=fo.toWorld(elbow.rb);
    const dUp=Math.hypot(up.x.x-anchS.x, up.x.y-anchS.y);
    const dFoS=Math.hypot(fo.x.x-anchS.x, fo.x.y-anchS.y);
    const dFoE=Math.hypot(fo.x.x-anchE.x, fo.x.y-anchE.y);
    const Ish=axisI(up)+up.mass*dUp*dUp + axisI(fo)+fo.mass*dFoS*dFoS;
    const Iel=axisI(fo)+fo.mass*dFoE*dFoE;
    const lcS=(up.mass*dUp+fo.mass*dFoS)/(up.mass+fo.mass);
    /* A spring can only ADD stiffness; on this rig gravity alone already puts the hanging
       arm ABOVE the Den Hartog target, so kp clamps to 0 (gravity IS the spring) and
       exact frequency tuning is unreachable. The damper is then set at the frequency the
       arm actually HAS -- wA = sqrt((kp + m*g*lc)/I) -- because damping computed for a
       frequency the arm does not swing at damps nothing. A mistuned-but-damped absorber
       still absorbs; the mistuning ratio is in the plan (wA/wT) for I14 to print. */
    const kpS=Math.max(0, wT*wT*Ish-(up.mass+fo.mass)*g2*lcS);
    const wAS=Math.sqrt((kpS+(up.mass+fo.mass)*g2*lcS)/Ish);
    plan['upperArm'+side]={ kp:kpS, kd:2*zeta*wAS*Ish, wA:wAS };
    const kpE=Math.max(0, wT*wT*Iel-fo.mass*g2*dFoE);
    const wAE=Math.sqrt((kpE+fo.mass*g2*dFoE)/Iel);
    plan['foreArm'+side]={ kp:kpE, kd:2*zeta*wAE*Iel, wA:wAE };
  }
  plan.mu=mu; plan.wT=wT; plan.zeta=zeta;
  return plan;
}

/* ---------- gimbal tuned-mass-damper plan (MK1.38.0) ----------
   The Scout has no arms to retune, so its absorber is purpose-built: tmdRing (roll hinge
   on the pelvis) carrying tmdBob (pitch hinge), bob CoM ABOVE the pivot -- see the spec
   note in rig/atst.js for why inverted. Gravity on an inverted bob is NEGATIVE stiffness,
   kg = -m*g*lc, so the spring places the natural frequency anywhere:
   w^2 = (kp + kg)/I  ->  kp = wT^2*I + m*g*lc, and both axes land EXACTLY on the Den
   Hartog target wT = omega/(1+mu) -- no gravity floor, unlike the arms. kd = 2*zeta*wT*I.
   Same mu approximation caveat as deriveArmTMD. Roll is planned with ring+bob composite
   about the ring hinge; pitch with the bob alone about its own hinge; at the assembled
   rest pose the CoM offsets are vertical, which is perpendicular to both horizontal axes,
   so the parallel-axis lever and the gravity lever are the same measured offset.
   buildRig APPLIES this; invariant I15 checks it against an independent implementation. */
function deriveGimbalTMD(rig, g2){
  if(!rig.joints.tmdRing) return null;
  g2=g2??9.81;
  const ring=rig.bodies.tmdRing, bob=rig.bodies.tmdBob;
  const rj=rig.joints.tmdRing, bj=rig.joints.tmdBob;
  let mach=0; for(const b of Object.values(rig.bodies)) mach+=b.mass;
  // Same rule as deriveArmTMD: sway frequency read from the rig at call time, never dg.
  const om=Math.sqrt(g2/Math.max(1e-4,rigStats(rig).com.y));
  const tm=ring.mass+bob.mass;
  /* mu treats the full 142 kg (ring+bob) as absorber mass on BOTH axes, though the roll
     axis swings ring+bob and pitch only the bob -- a shared approximation on top of the
     modal-mass one already noted in deriveArmTMD. Review-flagged, accepted: it moves wT
     by well under the tuning breadth of a zeta ~0.1 absorber. */
  const mu=tm/(mach-tm);
  const wT=om/(1+mu), zeta=Math.sqrt(3*mu/(8*Math.pow(1+mu,3)));
  const plan={mu,wT,zeta};
  {
    const anch=ring.toWorld(rj.rb);
    const dR=ring.x.y-anch.y, dB=bob.x.y-anch.y;      // signed: + is above the pivot
    const I=inertiaAbout(ring,V(1,0,0))+ring.mass*dR*dR
           +inertiaAbout(bob,V(1,0,0))+bob.mass*dB*dB;
    const lc=(ring.mass*dR+bob.mass*dB)/tm;
    plan.tmdRing={kp:wT*wT*I+tm*g2*lc, kd:2*zeta*wT*I};
  }
  {
    const anch=bob.toWorld(bj.rb);
    const dB=bob.x.y-anch.y;
    const I=inertiaAbout(bob,V(0,0,1))+bob.mass*dB*dB;
    plan.tmdBob={kp:wT*wT*I+bob.mass*g2*dB, kd:2*zeta*wT*I};
  }
  return plan;
}

/* Gait parameters that follow from size alone -- the dimensionless constants measured
   across MK1 (leg 2.95 m) and the Scout (leg 5.00 m). Scaling a rig therefore needs no
   retuning: these fall out of the geometry. */
/* HOW FAR THE BODY MAY TURN IN ONE STEP. 8 deg, was 20.
   20 deg/step is 81.8 deg/s at the shipped step period -- a 180 deg turn in 2.2 s on a machine
   that cannot deliver it. The driving logs show the command running 132-174 deg ahead of the body
   at every single failure, and the measured yaw actually delivered is 1-9 deg per step, so the
   commanded rate was never achievable in the first place.
   8 deg/step is 32.8 deg/s and a 180 deg turn takes 22 steps, about 5.4 s. That is a mech.
   turnRate is DERIVED from this and the step cycle (see the return below), so the two cannot
   disagree -- asserted by test/invariants.mjs I3. */
let YPS_DEG=8;
function deriveGait(rig){
  const st=rigStats(rig), L=rig.leg.thigh+rig.leg.shin, z=Math.max(1e-4,st.com.y);
  const rt=Math.sqrt(L/2.95);
  const omega=Math.sqrt(9.81/z), foot=rig.bodies[`foot${rig.sides[0]}`];
  /* AGILITY. The 1.66/0.92 coefficients are the PASSIVE pendulum values -- what a machine
     must obey if gravity is its only stabiliser. With attitude assist it does not have to.
     Measured sweep at 1.25 m, 25 s forward, no falls in any cell: 1.0 -> 3.28 m, 0.7 ->
     4.56 m, 0.5 -> 3.71 m. 0.7 is the peak; below it the steps stop paying for themselves.
     Servo gains were swept in the same run and are strictly WORSE stiffer (x2 and x4 both
     lose ground at every timing), so that lever stays alone -- matching the over-power
     rule already in the design table. */
  const AG=0.7;
  const tSS=1.66*AG/omega, tDS=0.92*AG/omega;
  /* CRAWL TEMPO (MK1.40.0). The crawl inherited the biped's AG-scaled phase times, and a
     support-triangle argument run at a dynamic walk's tempo is not static: log
     s20260730185950 shows the Heavy's loaded-foot count at 0-3 during SWING phases (plan
     says exactly 3), one fully airborne sample, 350-527 mm/s body-speed spikes during
     SHIFT -- the phase whose entire correctness argument is quasi-static -- and vertical
     velocity swinging +-270 mm/s. The quad gets its own tempo: 2.75x slower swings and
     shifts. Travel speed drops with it; that is the crawl's identity -- slow, deliberate,
     never airborne. stepPeriod below is the ONE site for "how long is a step" and turnRate
     derives from it, so the command cannot outrun the slowed gait (the class of bug that
     yawPerStep already existed to kill). */
  const crawlTempo=rig.gait==='quad'?2.75:1;
  const tSwing=tSS*crawlTempo, tShift=tDS*crawlTempo;
  const stepPeriod=rig.gait==='quad'?tSwing+tShift:tSS+tDS;
  /* Turning is now ONE parameter, not two: how far the body may turn per step. The slew
     rate follows from it and the step cycle, so they cannot disagree. */
  /* Per-rig, because 8 deg/step is not the same commitment on three machines with different
     yaw authority and different yaw inertia. Measured on log s20260727234900 (MK1.21.0) with a
     uniform 8 deg: Light delivered 6.06 deg/step median and survived 21 s; the Scout delivered
     7.99 at a third the mass and fell at 8.6 s; the Heavy delivered 9.93 -- MORE than commanded --
     and fell at 5.7 s. Both of the rigs that fell were meeting or beating the command, so the
     command is what to lower. Scout and Heavy carry 5 deg (see their specs); Light keeps 8. */
  const yawPerStep=((rig.yawPerStepDeg??YPS_DEG)*Math.PI/180);
  /* Waist travel comes from the RIG, not from a control constant -- it is the ring's own
     end stop, and a controller that commands past it is leaning on a mechanical stop. A
     rig whose torso is welded reports 0 and the turret code disables itself. */
  const wLim=rig.table.torso.range
    ? Math.min(Math.abs(rig.table.torso.range[0]),Math.abs(rig.table.torso.range[1])) : 0;
  /* Hip yaw ring travel, from the rig's own end stops, at the 0.9 margin the posture solver has
     always used. Computed HERE rather than in control/posture.js so the limit and the slew rate
     below come from one expression -- posture restating it was one edit away from a ring commanded
     past a stop it thought it was inside. */
  const hyR=rig.table[`hipYaw${rig.sides[0]}`].range;
  const hyLim=hyR?0.9*Math.min(Math.abs(hyR[0]),Math.abs(hyR[1])):0;
  return { tSS, tDS, tSwing, tShift, stepPeriod, yawPerStep, 
    /* 0.047 -> 0.12 (MK1.35.0) -> 0.08 (MK1.36.0). 4.7% read as a shuffle; 12% was driven
       and REGRESSED -- Light fell at 10 and 6 steps, substep contact p95 rose 1.50 -> 1.97 W
       (log s20260730101320). The lift itself is the disturbance: the plan is a point-mass
       LIPM that never sees the swing leg's momentum, and 2.55x the lift is 2.55x the
       unmodeled shove. 8% until swing reaction is compensated. The sin^2 profile in
       gait.js is what makes any of these land at zero commanded vertical speed. */
    stepHeight:0.08*L, pelvisDrop:0.085*L,
    /* 0.28, not the 0.32 measured earlier: that figure was calibrated against a planner
       whose pendulum height was being clamped by the old 1.0 m floor. With the floor gone
       and the planner correct, 0.32 fails 1 run in 3 while 0.28 is 3/3 with the travel
       spread collapsing to under 5% -- deterministic rather than chaotic. */
    /* 0.28*L on two legs. A crawler gets 0.40*L, because its foot's HIP-RELATIVE excursion
       is only (nLegs-1)/nLegs of a stride -- the body advances stride/nLegs during the swing
       and again on each stance step -- so 0.40 costs the leg the same reach 0.28 does on a
       biped. Checked against legIK's 0.995 clamp at the crouch: r = sqrt(0.615^2 + 0.41^2 +
       3.712^2)/4.10 = 0.923. 0.45 overruns. */
    strideCap:(rig.gait==='quad'?0.40:0.28)*L,
    /* CRAWL STABILITY MARGIN. How far inside the support triangle the commanded COM is held
       while a foot is up. 0.10*L, so it scales like every other length here. Geometric
       ceiling on the Heavy is 2ab/sqrt(a^2+b^2) = 2.33 m, so this uses 18% of what is
       available and leaves the peak single-leg load at 0.455 W against the 0.500 W bound at
       zero margin. Larger is more stable and costs more body sway per step -- and the sway
       is the machine's ONLY lateral CoP authority, so it is not free in either direction. */
    crawlMargin:0.10*L,
    /* The separation clamp must sit BELOW the assembled stance width (2x hip offset), or
       it fires on one side every step and shoves the machine sideways -- the Scout's came
       out 1.7 mm ABOVE its stance and it veered constantly. Cap at 95% of stance. */
    minFootSep:Math.min(foot.dim.z+0.06*(L/2.95),
                        1.9*Math.abs(rig.table[`hipYaw${rig.sides[0]}`].jp[2])),
    /* BALANCED ZONE. How far the centre of pressure may travel under the foot before the
       controller is out of authority -- i.e. how far off the CoP may be and still be
       treated as recoverable. The shipped values were 0.60/0.45, so the machine balanced
       on a patch well inside the foot it was standing on and declared itself out of
       authority while there was still a third of the sole left to push on.
       Now 0.80/0.65. The remaining 20%/35% is edge margin: driving the CoP to the literal
       rim tips the foot onto its edge and sheds contact area, which is what the clamp
       exists to prevent.
       This clamp and the ankle ceilings are ONE rule at two sites -- the ankles in
       rig/mech.js are sized as 1.40*W*copLimitX and 1.45*W*copLimitZ, and were raised by
       these same ratios in the same change. A zone the ankle cannot reach is a command
       the machine will not follow. Change one, change the other.
       UNVERIFIED as of this build: the earlier sweep that put a number on this widening
       ran with the wrong kCop, so its magnitude does not stand. The direction does -- the
       clamp was measurably binding. Driven, not simulated, from here. */
    copLimitX:0.80*(foot.half.x-rig.ankle.x), copLimitZ:0.65*foot.half.z,
    copClamp:0.45*(L/2.95), travelRate:0.6*Math.sqrt(L/2.95),
    /* SCALE FIX: every TIME in the controller goes as sqrt(leg length). These were fixed
       seconds, so a 1 ft rig spent 10.4 pendulum periods crouching where a full-size one
       spent 5, and its plan phases were similarly out of proportion. */
    settleTime:0.4*rt*AG, crouchTime:1.4*rt*AG, tStart:1.2*rt*AG, tEnd:3.0*rt*AG,
    waistLimit:wLim,
    /* HIP YAW RING: LIMIT AND SLEW, derived here so the controller never restates either.
       Measured on driving log s20260727225156 (MK1.13.0), all three rigs: the ring's COMMANDED
       angle stepped up to 40.50 deg in ONE sim step -- 12 004 deg/s, 209 rad/s -- because the
       target is legIK's `rel` with no rate limit, and `rel` is identically 0 in double support
       and jumps to the full clamp the moment single support begins. It sat at exactly 0 for
       39-81% of sim steps and then slammed. Ring angle reached 44.5-45.1 deg against a +-45 deg
       hard stop, servo demand hit 2.8x the torque ceiling, servo error 38 deg on a joint that
       saturates at 16, and the session tore ankleYokeR and torso on the Light Frame and torso
       and head on the Heavy.
       This is the waist-ring bug on a different joint. That one stepped 100 deg and slammed at
       37 rad/s; this is 5.7x the rate. `waistRate` fixed it there; the same rule applies here,
       and "every command channel is rate-limited, without exception" is the standing rule.
       RATE: the full ring travel in one step period. The legitimate demand is at most yawPerStep
       on a swing ring and half of it on a stance ring, inside tSS -- about 127 deg/s -- so full
       travel per step gives 2.6x headroom and never binds on a real command while cutting the
       slam by ~36x. It is a controller time, so tying it to the step period makes it
       Froude-correct for free. Asserted by test/invariants.mjs I7. */
    hipYawLimit:hyLim, hipYawRate:(2*hyLim)/(tSS+tDS),
    /* ACTUATOR GOVERNOR, as a fraction of each joint's OWN ceiling per second. Multiply by
       j.tauMax to get N.m/s -- done in rig/build.js, so the number lives here once.
       `tauSlewSteps` 3 = full authority in one third of a step period, 81 ms at 0.30 m. Chosen
       against the measured chatter rather than picked: the jitter in log s20260727231426 reverses
       at 9-21 Hz, so a half cycle is 24-56 ms, and a torque that needs 81 ms to cross its range
       cannot complete a reversal inside one. The gyro has used exactly this form since MK1.8
       (slewSteps 1.5); the joints are the channel that never got it.
       Froude-correct for free: tauMax goes as s^4 and the step period as sqrt(s), so the rate
       goes as s^3.5, same as the gyro's. */
    tauSlewSteps:3, tauRateFrac:3/(tSS+tDS),
    /* Waist slew rate: the full ring travel in one pendulum-scaled second. At 4 ft that is
       about 200 deg/s -- a 50 deg aim change lands in a quarter second, which still reads
       as instant on a stick, against a slam that peaked near 21 rad/s and tore the arms
       off. Six times less momentum into the end stop for a response nobody will notice is
       slower. It is a controller TIME, so it goes as sqrt(L) like every other one: an
       absolute rad/s would leave a 1 ft rig swinging its turret at half the pace of its
       own gait. */
    waistRate:(2*wLim)/rt,
    /* Body-position rate ceiling: 3x the fastest this machine can walk (one full stride
       per step cycle). It must never bind during ordinary travel -- the reference genuinely
       moves at COM speed -- so it is set well above that and exists only to turn a STEP in
       the commanded pelvis into a ramp. */
    pelvisRate:3*(0.28*L)/(tSS+tDS),
    turnRate:yawPerStep/stepPeriod, omega, L, height:st.height };
}
