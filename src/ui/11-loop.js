/* ---------- loop ---------- */
const target=new THREE.Vector3(0,3.1,0);
let last=performance.now(), accum=0, fpsT=0, fpsN=0, fps=0;
// Sim IMU + capturability light (MK1.36.0); written each display frame, read by the HUD
// extras in 05-telemetry-sheet.js and logged in every `st` record.
let accPrevV=null, accPrevT=0, accVec=[0,0,0], capState=0;
const DT=1/60;
/* Tab title from BUILD_TITLE, so the version in the tab, the HUD chip and the session-log
   header are one string from core/preamble.js and cannot drift apart. */
document.title=BUILD_TITLE;

function tick(now){
  requestAnimationFrame(tick);
  const real=Math.min(0.1,(now-last)/1000); last=now;
  fpsN++; fpsT+=real; if(fpsT>0.5){ fps=fpsN/fpsT; fpsN=0; fpsT=0; }

  /* ---- TWIN STICK ----
     Left stick is a TRAVEL VECTOR, right stick is FACING, and the two are independent.
     Both are camera-relative, because the camera sits on fixed isometric detents and
     "push away from me" has to mean the same thing at every detent.
     Screen-forward on the ground plane is -(sin az, 0, cos az); screen-right follows from
     crossing that with world up. */
  const svx=-Math.sin(azShown), svz=-Math.cos(azShown);      // screen forward
  const srx= Math.cos(azShown), srz=-Math.sin(azShown);      // screen right

  const lx=Math.abs(stickL.x)>0.12?stickL.x:kbAxis('a','d');
  const ly=Math.abs(stickL.y)>0.12?stickL.y:kbAxis('s','w');
  let dx=ly*svx+lx*srx, dz=ly*svz+lx*srz;
  const mag=Math.min(1,Math.hypot(dx,dz));
  const cap=pushEnvelope?envCap:strideCap;
  if(mag>1e-3){ const n=Math.hypot(dx,dz); dx/=n; dz/=n; } else { dx=0; dz=0; }
  gait.want.tx=dx*mag*cap;
  gait.want.tz=dz*mag*cap;

  /* Right stick AIMS THE TORSO. Its angle is a heading, not a rate: push it where you want
     the mech to look and the waist ring is already there, because it turns one body
     against one actuator. The legs stay out of it until you have used up the waist, at
     which point gait.updateWaist() hands them the heading and they walk the chassis round
     underneath you.
     Before the waist ring existed this line wrote gait.want.facing and the machine did not
     turn at all: the hip yaw rings were commanded to exactly 0 deg at every heading, so
     the only yaw authority on the rig was the gyro dragging the whole thing against foot
     friction -- about 4 deg per step regardless of what was asked. */
  const rx=Math.abs(stickR.x)>STICK_DEAD?stickR.x:kbAxis('q','e')*-1;
  const ry=Math.abs(stickR.y)>STICK_DEAD?stickR.y:0;
  /* AIM_MIN was 0.35 while the stick reads from 0.12, so the knob visibly tracked your
     thumb through the first third of its travel while the machine ignored it completely --
     which reads as broken input, not as a deadband. It only needs to be above the noise
     floor on atan2 of a near-zero deflection, and 0.18 is. One constant, and the auto-face
     threshold below uses it too rather than a third number (it was 0.15). */
  gait.aimHold=Math.hypot(rx,ry)>AIM_MIN;
  if(gait.aimHold){
    const fx=ry*svx+rx*srx, fz=ry*svz+rx*srz;
    gait.aim=Math.atan2(-fz,fx);
  }
  /* TURN PAD, and it takes priority over the aim stick because it is the explicit command.
     The stick is a POSITION command -- gait.aim is set to the absolute heading the thumb points
     at -- so a flick across it steps the heading by whatever angle the thumb crossed, and the
     ring, the legs and the mounts absorb that step. A held button is a RATE command: the heading
     integrates, so there is no step anywhere in the chain by construction.
     Rate is `waistRate` from the rig (rig/derive.js: full ring travel per pendulum-scaled
     second), so the aim can never outrun the ring that has to follow it, and it scales with the
     machine like every other controller time. aimHold is forced true while held, which is what
     lets the legs come round once the ring passes waistFollow. */
  if(turnBtn){
    /* turnRate, not waistRate. waistRate is the RING's slew -- full travel in a pendulum-scaled
       second, ~400 deg/s -- and integrating the heading at it walked the aim 174 deg past the body
       in the log that tore three rigs apart. turnRate is what the LEGS can deliver: yawPerStep
       over one step cycle, 32.8 deg/s. Matching the command rate to the delivery rate is what
       makes holding the button a planned turn instead of a standing request. */
    gait.aim=wrapPi(gait.aim+turnBtn*gait.k.turnRate*real);
    gait.aimHold=true;
  }
  /* AUTO-FACE REMOVED. The branch that used to live here wrote BOTH gait.aim and
     gait.want.facing from the LEFT stick whenever the right one was idle, so pushing the
     travel stick rotated the turret and swung the chassis onto the travel heading. That is
     a one-stick game's control scheme and it is not this one: it meant the left stick
     turned the head, and it meant a sideways push was answered by turning to face sideways
     rather than by stepping sideways -- there was no strafe at any point, only a transient
     while the body came round. Rotation belongs to the right stick alone now; the left
     stick translates and nothing else. The chassis heading is commanded in exactly one
     place, Chassis.updateWaist, once the aim has used up the waist ring.
     This is what makes the coronal gait load-bearing rather than decorative -- see the
     splay bound in control/gait.js, which had 97% of it clamped away. */
  const turning=Math.abs(wrapPi(gait.aim-gait.bodyYaw()))>0.02;
  steering=turning&&mag>0.05;

  logFrame++;
  logRec({t:'in',f:logFrame,st:+simT.toFixed(3),
    L:[+lx.toFixed(3),+ly.toFixed(3)],R:[+rx.toFixed(3),+ry.toFixed(3)],
    want:[+gait.want.tx.toFixed(3),+gait.want.tz.toFixed(3),+gait.want.facing.toFixed(4)],
    /* aim = where the TORSO is told to point, yaw = where the chassis actually points,
       waist = the ring angle between them. Without these three a log cannot tell "the
       stick did nothing" from "the stick worked and the legs did not follow" -- which is
       exactly the distinction that took a session to find by reading code. */
    aim:+gait.aim.toFixed(4), yaw:+gait.bodyYaw().toFixed(4),
    waist:rig.joints.torso?+(rig.joints.torso.angle*57.2958).toFixed(1):null,
    cmd:[+gait.cmd.tx.toFixed(3),+gait.cmd.tz.toFixed(3),+gait.cmd.facing.toFixed(4)],
    act:[+gait.active.tx.toFixed(3),+gait.active.tz.toFixed(3),+gait.active.facing.toFixed(4)],
    s:gait.state,n:gait.stepsTaken||0,env:pushEnvelope?1:0,az:+azShown.toFixed(3)});

  const camX=kbAxis('arrowleft','arrowright')+orbitBtn;
  const camY=kbAxis('arrowdown','arrowup')+zoomBtn+(keys['-']?-1:0)+(keys['=']||keys['+']?1:0);
  azCont-=camX*1.9*real;
  detent=((Math.round(azCont/STEP)%DET)+DET)%DET;
  azimuth=detent*STEP;
  /* Bounds and zoom rate are rig-relative -- the old absolute 6.8 m floor overrode the
     derived framing every frame and drew a 1 ft rig at 4% of the viewport. */
  spanWant=Math.max(spanMin,Math.min(spanMax,spanWant-camY*spanWant*1.5*real));
  span+=(spanWant-span)*Math.min(1,real*6);

  const slow=(now<slowUntil)?0.25:1;
  accum+=real*slow;
  if(paused) accum=0;              /* self test owns the CPU; do not fast-forward after */
  const t0=performance.now(); let n=0, stEst=null;
  while(accum>=SIM_DT && n<3*simSteps){
    const before=world.breakEvents.length;
    stEst=groundTruthState(rig,world);
    /* DOWN STATE (MK1.36.0). The walk controller used to keep running after a fall,
       commanding step targets in a pose its plan never modeled -- every servo saturated
       chasing them, which was both the on-screen flailing and the post-fall break
       cascades (5 mounts at once in log s20260730011112). Down means LIMP: each joint's
       target is pinned to its own current angle, so kp holds nothing and kd/kv damp
       whatever motion is left. The gyro is not updated -- no attitude authority on a
       machine that has no attitude to save. Recovery is the respawn button. */
    if(!fallen){
      gait.update(stEst,SIM_DT);
      /* stabiliserYaw(), not cmd.facing: at rest the feet are planted and the body cannot
         turn, so a leftover heading error would have the gyro grinding yaw torque through
         the soles of a machine that is meant to be standing still. */
      /* Hand the gyro the SAME reference the ankles balance against -- the planner's ZMP
         while a plan is running, null when there is none. Without it the gyro measures its
         capture error from the support centre and brakes every step of a normal walk. */
      if(cmg){ cmg.targetYaw=gait.stabiliserYaw();
               cmg.copRef=gait.balance?gait.balance.copOverride:null;
               cmg.update(stEst,SIM_DT); }
    } else {
      for(const j of Object.values(rig.joints)) j.target=j.angle;
    }
    world.step(SIM_DT);
    if(world.breakEvents.length>before) slowUntil=now+1500;
    accum-=SIM_DT; n++; simT+=SIM_DT;
    checkFall();

    /* ---- BURST SAMPLE, every sim step ----
       Written into a ring in memory, transmitted only if something triggers. This is the only
       place in the build that observes every sim step; the continuous `st` channel sees one in
       twelve. Keep it narrow -- it costs a push per sim step, ~243 times a second. */
    const bb=rig.bodies;
    burstPush([r4(simT),
      r5(bb.pelvis.x.y), r4(bb.pelvis.v.y),
      r4(Math.hypot(bb.pelvis.v.x,bb.pelvis.v.z)),
      r4(qrot(bb.torso.q,V(0,1,0)).y),
      rig.sides.map(s2=>r3((bb['foot'+s2].contactForce||0)/rigWeight)),
      rig.sides.map(s2=>r3((bb['foot'+s2].contactForceMax||0)/rigWeight)),
      rig.sides.map(s2=>{const q=gait.posture.last[s2];return q?r4(q.reach):0;}),
      jointNames.map(n2=>r1(rig.joints[n2].tauPeak*1e3)),
      jointNames.map(n2=>r2(rig.joints[n2].target*57.2958)),
      jointNames.map(n2=>r2(rig.joints[n2].angle*57.2958)),
      cmg?r3(cmg.tauFrac):0]);

    /* TRIGGERS. Deliberately generous -- a burst is ~120 kB and a false positive costs nothing
       next to missing the one event that explains a session. Each names itself in the record so
       a log can be filtered by cause. */
    if(world.breakEvents.length>before) burstTrigger('break');
    else if(fallen&&!fallLogged) burstTrigger('fall');
    else if(rig.sides.every(s2=>(bb['foot'+s2].contactForce||0)<0.02*rigWeight))
      burstTrigger('airborne');
    else if(Math.abs(bb.pelvis.v.y)>0.5) burstTrigger('vertical');
    else if(rig.sides.some(s2=>(bb['foot'+s2].contactForceMax||0)>4*rigWeight))
      burstTrigger('impact');
    else if(rig.sides.some(s2=>{const q=gait.posture.last[s2];return q&&q.reach>1;}))
      burstTrigger('unreachable');
    else if(jointNames.some(n2=>(rig.joints[n2].satFrac||0)>=0.999)) burstTrigger('railed');
    burstTick();
  }
  if(n) simMs=simMs*0.9+((performance.now()-t0)/n)*0.1;
  stepCount=gait.stepsTaken||0;

  let lo=Infinity, hi=-Infinity, peak=0, peakName='';
  for(const name of Object.keys(rig.bodies)){
    const b=rig.bodies[name], m=meshes[name];
    m.position.set(b.x.x,b.x.y,b.x.z);
    m.quaternion.set(b.q.x,b.q.y,b.q.z,b.q.w);
    const half=Math.max(b.half.x,b.half.y,b.half.z);
    if(b.x.y-half<lo) lo=b.x.y-half;
    if(b.x.y+half>hi) hi=b.x.y+half;
  }
  for(const n2 of jointNames){
    const j=rig.joints[n2], r=rowEls[n2];
    const u=j.broken?1:Math.min(1,j.util);
    meshes[n2].material.color.setHex(j.broken?0x6E2222:ramp(u));
    const t=j.tauMax?Math.max(-1,Math.min(1,j.tau/j.tauMax)):0;
    const w2=Math.abs(t)*50;
    r.fill.style.width=w2+'%'; r.fill.style.left=(t>=0?50:50-w2)+'%';
    r.fill.style.background='#'+ramp(u).toString(16).padStart(6,'0');
    r.util.textContent=j.broken?'TORN':(u*100).toFixed(0)+'%';
    r.util.style.color=(j.broken||u>0.85)?'#A83232':'#4A463C';
    r.root.classList.toggle('torn',!!j.broken);
    if(u>peak){ peak=u; peakName=n2; }
  }

  /* ---- PELVIS ACCELEROMETER + CAPTURABILITY LIGHT (MK1.36.0) ----
     accVec is the exact sim IMU: pelvis dv/dt across the display frame, m/s^2, world
     axes. capState is 0 green / 1 yellow / 2 red: green while the capture point
     xi = com + comVel/omega sits inside the box the ankles can reach (no step needed);
     yellow while a single catch step could still land at it (sagittal strideCap,
     lateral splay headroom); red beyond that or fallen -- no single step arrests it.
     Both are HUD + log instrumentation; NO control law reads either. */
  if(stEst){
    const dtA=simT-accPrevT;
    if(accPrevV&&dtA>1e-6){
      accVec=[(rig.bodies.pelvis.v.x-accPrevV[0])/dtA,
              (rig.bodies.pelvis.v.y-accPrevV[1])/dtA,
              (rig.bodies.pelvis.v.z-accPrevV[2])/dtA];
    }
    accPrevV=[rig.bodies.pelvis.v.x,rig.bodies.pelvis.v.y,rig.bodies.pelvis.v.z];
    accPrevT=simT;
    capState=2;
    if(!fallen){
      if(!gait.plant||!gait.plant.L){
        /* Quad (MK1.40.0): the capture point against the crawl's own support polygon.
           The Heavy had been green-at-fall since the light existed because the biped
           model didn't apply. Red = xi outside the polygon: a static walker has no catch
           step, so that is genuinely unrecoverable, which is exactly what the light is
           for. Yellow inside but under half the crawl margin. */
        if(typeof gait.supportMargin==='function'&&stEst.com){
          const zc=Math.max(1e-4,stEst.com.y), om=Math.sqrt((gait.k.gravity||9.81)/zc);
          const mg=gait.supportMargin(stEst.com.x+stEst.comVel.x/om,
                                      stEst.com.z+stEst.comVel.z/om);
          const cm=gait.k.crawlMargin||0.01;
          capState=mg>=0.5*cm?0:(mg>=0?1:2);
        } else capState=0;
      }
      else{
        const zc=Math.max(1e-4,stEst.com.y), om=Math.sqrt((gait.k.gravity||9.81)/zc);
        const xiX=stEst.com.x+stEst.comVel.x/om, xiZ=stEst.com.z+stEst.comVel.z/om;
        const midX=(gait.plant.L.x+gait.plant.R.x)/2, midZ=(gait.plant.L.z+gait.plant.R.z)/2;
        const bs=gait.basis();
        const eF=Math.abs((xiX-midX)*bs.fwd.x+(xiZ-midZ)*bs.fwd.z);
        const eL=Math.abs((xiX-midX)*bs.left.x+(xiZ-midZ)*bs.left.z);
        const bk=gait.balance.k, hs=gait.halfStance;
        const latReach=((gait.k.splayMax||1.3)-1)*2*hs;
        if(eF<=bk.copLimitX && eL<=bk.copLimitZ+hs) capState=0;
        else if(eF<=bk.copLimitX+(gait.k.strideCap||0) && eL<=bk.copLimitZ+hs+latReach) capState=1;
        else capState=2;
      }
    }
  }
  renderHudExtras();
  driveCardTick();

  document.getElementById('peak').textContent='peak '+(peak*100).toFixed(0)+'% '+peakName;
  document.getElementById('c-rate').textContent=fps.toFixed(0)+' fps · '+simMs.toFixed(1)+' ms';
  /* Report speed as travel over elapsed SIM time, not the instantaneous pelvis velocity.
     Pelvis velocity oscillates with every step and reads high mid-swing; it also keeps
     reading a plausible number while the rig is sliding along the ground on its back. */
  const travelled=Math.hypot(rig.bodies.pelvis.x.x-startX,rig.bodies.pelvis.x.z-startZ);
  document.getElementById('c-speed').textContent=
    (simT>0.5?(travelled/simT):0).toFixed(2)+' m/s avg';
  document.getElementById('c-steps').textContent=
    (fallen?fellAtSteps:stepCount)+' steps · '+(fallen?fellAtTravel:travelled).toFixed(1)+' m';
  const torn=world.breakEvents.length, tc=document.getElementById('c-torn');
  if(torn){ tc.style.display=''; tc.className='chip warn';
    tc.textContent=torn+' joint'+(torn>1?'s':'')+' torn'; }
  const sc=document.getElementById('c-state');
  sc.textContent=fallen?'DOWN':(gait.state||'—').toLowerCase();
  sc.className=fallen?'chip warn':'chip';
  const fc=document.getElementById('c-fall');
  if(fallen){ fc.style.display='';
    fc.textContent='FELL at '+fellAtSteps+' steps / '+fellAtTravel.toFixed(1)+' m &mdash; TAP TO RESPAWN';
    fc.innerHTML='FELL at '+fellAtSteps+' steps / '+fellAtTravel.toFixed(1)+' m &mdash; TAP TO RESPAWN'; }
  else fc.style.display='none';
  document.getElementById('c-steer').style.display=steering?'':'none';
  const cc=document.getElementById('c-cmg');
  if(cmg&&cmgOn){ cc.style.display='';
    /* Momentum saturation is the gyro's real failure mode: at 100% the wheel is spun up
       and has nothing left to give, whatever its torque rating says. Shown, not hidden. */
    cc.textContent='gyro '+(cmg.tauFrac*100).toFixed(0)+'% · store '+(cmg.satFrac*100).toFixed(0)+'%';
    cc.className=cmg.satFrac>0.95?'chip warn':'chip';
  } else cc.style.display='none';

  /* ---- body state, plus events ----
     20 Hz, was 10. The gait fundamental is 1/(tSS+tDS) = 4.09 Hz, so 10 Hz gave 2.44 samples
     per step against a Nyquist of 5 Hz -- 22% of margin, and every harmonic of the touchdown
     impulse folded back. Worse, the stance-leg spring measured 24.5 Hz aliases to 4.50 Hz at
     10 Hz, which is 0.40 Hz from the step rate: a spring ring-down and the walk itself were
     THE SAME SIGNAL in every log this project has collected. Not attenuated -- relabelled.
     20 Hz doubles the margin on the gait; the aliasing of the spring is fixed by logging the
     substep ENVELOPE (see the joint record) rather than by chasing 245 Hz, and by the burst
     buffer for the cases where the waveform actually matters. */
  if(logFrame%3===0){
    const b=rig.bodies, u=qrot(b.torso.q,V(0,1,0));
    /* The sim may not have stepped this frame (paused, or the accumulator was short), in
       which case stEst is null and the sensors would log as undefined rather than as the
       machine's actual state. Recomputing costs one pass over the bodies. */
    const stLog=stEst||groundTruthState(rig,world);
    /* PER-JOINT RECORD. The old one was 7 fields and the FIRST of them was structurally
       incapable of a nonzero value: torque was written as (tau/1e3).toFixed(2), i.e. kN.m at
       two decimals, one count = 10 000 mN.m, against a strongest-joint-on-any-preset ceiling
       of 1 893 mN.m as driven. 91 295 joint-torque samples across 11 sessions, every one
       exactly 0.00. Every actuator reading in every driving log said "no torque anywhere",
       which is worse than logging nothing because it reads as a measurement.
       Torque is now mN.m at 1 dp: 0.1 mN.m quantum, ~13 200 counts across the thigh's range.

       And the COMMAND half is here for the first time. Only the achieved angle was ever
       logged, so no log could show a servo error -- and the servo error IS the torque
       (tau = kp*e - kd*wRel + tauFF). A stepped target was invisible by construction, which
       is exactly the failure that took a whole session to find by reading source. */
    const R=57.2958, M=1e3;                      // rad->deg, N.m->mN.m
    const jt={};
    for(const n2 of jointNames){ const j=rig.joints[n2];
      const err=Math.atan2(Math.sin(j.target-j.angle),Math.cos(j.target-j.angle));
      jt[n2]=[
        +(j.tau*M).toFixed(1),                   // [0]  torque now, mN.m
        +((j.tauPeak||0)*M).toFixed(1),          // [1]  worst torque over the frame's substeps
        +((j.demandPeak||0)/Math.max(1e-12,j.tauMax)).toFixed(3), // [2] peak DEMAND / ceiling
        +((j.tauFF||0)*M).toFixed(1),            // [3]  balance feedforward, mN.m
        +((j.limitPeak||0)*M).toFixed(1),        // [4]  end-stop reaction torque, mN.m
        +(j.angle*R).toFixed(2),                 // [5]  achieved angle, deg
        +(j.target*R).toFixed(2),                // [6]  COMMANDED angle, deg
        +(err*R).toFixed(2),                     // [7]  servo error, deg
        +((j.ratePeak||0)*R).toFixed(1),         // [8]  peak |joint rate| over substeps, deg/s
        +j.util.toFixed(3),                      // [9]  structural utilisation
        +(j.satFrac||0).toFixed(3),              // [10] fraction of substeps at the ceiling
        j.onStop?1:0, j.broken?1:0,              // [11][12]
        +(j.govFrac||0).toFixed(3),              // [13] fraction of substeps the GOVERNOR bound
      ]; }
    logRec({t:'st',f:logFrame,st:+simT.toFixed(3),
      pel:[+b.pelvis.x.x.toFixed(3),+b.pelvis.x.y.toFixed(3),+b.pelvis.x.z.toFixed(3)],
      pv:[+b.pelvis.v.x.toFixed(3),+b.pelvis.v.y.toFixed(3),+b.pelvis.v.z.toFixed(3)],
      up:+u.y.toFixed(4),
      /* Contact force as a FRACTION OF BODY WEIGHT, not kN to one decimal. The old form
         was written for an 8 360 kg rig where 0.1 kN was a rounding error; on the 27 kg
         Scout at 4 ft it quantised to 37% of body weight per count, so the log read
         0.00 / 0.37 / 0.75 / 1.12 and looked exactly like a machine leaving the ground.
         It is the number every load question is actually asked in ("stance foot fell to
         2% of body weight"), so store it that way and it cannot be mis-scaled again. */
      /* Over rig.sides. This was two hardcoded feet named L and R, and on a four-legged rig
         b.footL is undefined -- the first 10 Hz log frame throws a TypeError. */
      cf:rig.sides.map(s2=>+((b['foot'+s2].contactForce||0)/rigWeight).toFixed(3)),
      /* NEW SENSORS, all instrumentation -- no control law reads any of them yet.
         cfmin/cfmax are the SUBSTEP extremes behind that frame mean. Standing still, cf
         read 1.00 while the true load swung 0.00-3.28 W, and every load gate in this
         project was blind to it because the mean is all there was.
         cone = tangential impulse / (mu x normal impulse): 1 means the foot is riding the
         friction cone and about to slide. slip = the sole's actual ground speed in mm/s.
         Between them they answer "is it slipping, and was it about to" directly, instead
         of by differencing CoP against pelvis after the fact. */
      cfmin:rig.sides.map(s2=>+((b['foot'+s2].contactForceMin||0)/rigWeight).toFixed(3)),
      cfmax:rig.sides.map(s2=>+((b['foot'+s2].contactForceMax||0)/rigWeight).toFixed(3)),
      cone:rig.sides.map(s2=>+(b['foot'+s2].contactCone||0).toFixed(3)),
      slip:rig.sides.map(s2=>+((b['foot'+s2].contactSlip||0)*1000).toFixed(1)),
      /* COM height above the support plane, next to the raw com.y the DCM planner still
         uses for omega. Equal on flat ground; if they ever diverge, the planner is wrong. */
      // [0] is null when no foot is loaded -- that is information, not a gap to fill.
      ch:[stLog.comHeight===null?null:+stLog.comHeight.toFixed(4),+stLog.com.y.toFixed(4)],
      /* Actuator saturation as a fraction of substeps, and the worst offender by name.
         `saturated` per joint below is the LAST substep only, which reads as noise; this
         is the one that shows a joint bang-banging. */
      sat:[stLog.joints.nSat,stLog.joints.nJoint,+stLog.joints.peakSat.toFixed(3),
           stLog.joints.peakSatName],
      /* BOTH CoP axes. Only X was recorded, and lateral balance is the axis that actually
         fails -- there was no way to see a CoP sitting on the edge of a foot. */
      cop:rig.sides.map(s2=>{const c=b['foot'+s2].contactCop;
        return c?[+c.x.toFixed(4),+c.z.toFixed(4)]:null;}),
      dcm:gait.debug&&gait.debug.dcmErrX!==undefined?
        [+gait.debug.dcmErrX.toFixed(4),+gait.debug.dcmErrZ.toFixed(4)]:null,
      /* [2] capErr: the capture-point error the gyro is now acting on, mm. Zero means it
         fell back to lean (airborne, or predictive off), which is how to tell the two
         apart in a log rather than by guessing. */
      /* [2] capture error mm (0 unless predictive is on), [3] PASSIVE gyroscopic torque
         as a fraction of the actuator ceiling -- it is not limited by it, so values
         above 1 are real and are the flywheel doing more than the motor could. */
      cmg:cmg&&cmgOn?[+cmg.tauFrac.toFixed(3),+cmg.satFrac.toFixed(3),
                      +((cmg.capErr||0)*1000).toFixed(1),
                      +((cmg.gyroTau||0)/cmg.tauMax).toFixed(3)]:null,
      /* THE COMMANDED BODY REFERENCE, and the leg reach it produces. None of this was logged,
         which is why a target step -- the mechanism behind the documented 230 mm hop -- could
         not be seen in a log at all and had to be found by reading source.
           ref   pelvisCmd, the rate-limited body position the legs are actually solved against
           refY  pelvisY, the crouch/stand ramp feeding ref.y
           reach fraction of full leg extension DEMANDED per leg; >1 means legIK could not
                 reach and clamped, which is the launch condition
           ext   fraction actually COMMANDED after the clamp. reach>1 with ext at 0.995 is the
                 leg being straightened to meet an impossible target. */
      ref:gait.pelvisCmd?[+gait.pelvisCmd.x.toFixed(4),+gait.pelvisCmd.y.toFixed(4),
                          +gait.pelvisCmd.z.toFixed(4)]:null,
      refY:gait.pelvisY!==undefined?+gait.pelvisY.toFixed(4):null,
      reach:rig.sides.map(s2=>{const q=gait.posture.last[s2];
        return q?+q.reach.toFixed(4):null;}),
      ext:rig.sides.map(s2=>{const q=gait.posture.last[s2];
        return q&&q.ext!==undefined?+q.ext.toFixed(4):null;}),
      /* Waist COMMAND next to the achieved angle already in the `in` record. An unslewed
         position command on this ring tore both arms and the head off in five sessions out of
         six, and the only record of it was the achieved angle -- which cannot distinguish "the
         command stepped" from "the joint was disturbed". */
      wt:rig.joints.torso?+(rig.joints.torso.target*R).toFixed(2):null,
      /* Records the transport dropped because the client buffer was full. Counted since the
         first session and never once reported, so a truncated log was indistinguishable from
         a quiet one. Nonzero here means this record's neighbours are missing. */
      drop:logDropped,
      /* Transport health. `q` is the backlog waiting to be sent and `rt` the consecutive POST
         failures -- a rising q with rt>0 is the transport losing ground, which is what silently
         truncated a whole session before flushLog retried. */
      q:logBuf.length, rt:logFails,
      // Sim IMU (pelvis dv/dt, m/s^2, world axes) and the capturability light
      // (0 green / 1 yellow / 2 red) -- see the HUD block for both definitions.
      acc:accVec.map(v=>+v.toFixed(3)),cap:capState,
      fall:fallen?1:0,brk:world.breakEvents.length,fps:+fps.toFixed(1),ms:+simMs.toFixed(2),j:jt});
  }
  if(fallen&&!fallLogged){ fallLogged=true;
    logEvent('fall',{steps:fellAtSteps,travel:+fellAtTravel.toFixed(2)}); flushLog(true); }
  if(world.breakEvents.length>breakLogged){
    for(const e of world.breakEvents.slice(breakLogged))
      logEvent('break',{part:e.joint||e.weld,util:+e.util.toFixed(3),at:+e.t.toFixed(3)});
    breakLogged=world.breakEvents.length;
  }
  flushLog(false);

  /* frame the whole rig on its actual vertical extent, not on the pelvis */
  const p=rig.bodies.pelvis.x, mid=(lo+hi)/2;
  target.x+=(p.x-target.x)*Math.min(1,real*2.2);
  target.z+=(p.z-target.z)*Math.min(1,real*2.2);
  /* bias the framing upward so the rig sits above the stick zone rather than behind it */
  const aim=mid-span*0.12;
  target.y+=(aim-target.y)*Math.min(1,real*1.6);

  azShown+=(azimuth-azShown)*Math.min(1,real*10);
  const rad=60;
  camera.position.set(target.x+rad*Math.cos(ISO)*Math.sin(azShown),
                      target.y+rad*Math.sin(ISO),
                      target.z+rad*Math.cos(ISO)*Math.cos(azShown));
  camera.lookAt(target);

  if(Math.abs(span-lastSpan)>0.01||innerWidth!==lastW||innerHeight!==lastH) resize();
  renderer.render(scene,camera);
}

showNote(); buildWorld('light');
optList.querySelector('button').setAttribute('aria-pressed','true');
resize(); requestAnimationFrame(tick);