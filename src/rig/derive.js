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
let YPS_DEG=20;
function deriveGait(rig){
  const st=rigStats(rig), L=rig.leg.thigh+rig.leg.shin, z=Math.max(1e-4,st.com.y);
  const rt=Math.sqrt(L/2.95);
  const omega=Math.sqrt(9.81/z), foot=rig.bodies.footL;
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
  const yawPerStep=(YPS_DEG*Math.PI/180);
  return { tSS, tDS, yawPerStep, 
    stepHeight:0.047*L, pelvisDrop:0.085*L,
    /* 0.28, not the 0.32 measured earlier: that figure was calibrated against a planner
       whose pendulum height was being clamped by the old 1.0 m floor. With the floor gone
       and the planner correct, 0.32 fails 1 run in 3 while 0.28 is 3/3 with the travel
       spread collapsing to under 5% -- deterministic rather than chaotic. */
    strideCap:0.28*L,
    /* The separation clamp must sit BELOW the assembled stance width (2x hip offset), or
       it fires on one side every step and shoves the machine sideways -- the Scout's came
       out 1.7 mm ABOVE its stance and it veered constantly. Cap at 95% of stance. */
    minFootSep:Math.min(foot.dim.z+0.06*(L/2.95),
                        1.9*Math.abs(rig.table.hipYawL.jp[2])),
    /* BALANCED ZONE. How far the centre of pressure may travel under the foot before the
       controller is out of authority -- i.e. how far off it may be and still be treated as
       balanced. Was 0.60/0.45, so the machine balanced on a patch well inside the foot it
       was standing on. Widened toward the real edge. */
    copLimitX:0.60*(foot.half.x-rig.ankle.x), copLimitZ:0.45*foot.half.z,
    copClamp:0.45*(L/2.95), travelRate:0.6*Math.sqrt(L/2.95),
    /* SCALE FIX: every TIME in the controller goes as sqrt(leg length). These were fixed
       seconds, so a 1 ft rig spent 10.4 pendulum periods crouching where a full-size one
       spent 5, and its plan phases were similarly out of proportion. */
    settleTime:0.4*rt*AG, crouchTime:1.4*rt*AG, tStart:1.2*rt*AG, tEnd:3.0*rt*AG,
    turnRate:yawPerStep/(tSS+tDS), omega, L, height:st.height };
}
