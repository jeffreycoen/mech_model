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
    j.tauMax*=s4; j.kp*=s4; j.kd*=s4*rs;
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
  return { tSS, tDS, yawPerStep, 
    stepHeight:0.047*L, pelvisDrop:0.085*L,
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
    turnRate:yawPerStep/(tSS+tDS), omega, L, height:st.height };
}
