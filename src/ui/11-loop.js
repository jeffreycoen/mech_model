/* ---------- loop ---------- */
const target=new THREE.Vector3(0,3.1,0);
let last=performance.now(), accum=0, fpsT=0, fpsN=0, fps=0;
const DT=1/60;

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

  /* Right stick points the mech. Its ANGLE is the facing, not a rate -- push it where you
     want the mech to look and the body turns to match at turnRate. */
  const rx=Math.abs(stickR.x)>0.12?stickR.x:kbAxis('q','e')*-1;
  const ry=Math.abs(stickR.y)>0.12?stickR.y:0;
  if(Math.hypot(rx,ry)>0.35){
    const fx=ry*svx+rx*srx, fz=ry*svz+rx*srz;
    gait.want.facing=Math.atan2(-fz,fx);
  } else if(mag>0.15){
    /* Right stick idle: face the way we are walking, like every twin-stick game. Without
       this the body keeps its spawn yaw and any other travel direction reads as a
       crab-walk -- "forward" felt like holding the stick back-right. Aiming still owns
       facing the moment the right stick moves. */
    gait.want.facing=Math.atan2(-dz,dx);
  }
  const turning=Math.abs(wrapPi(gait.want.facing-gait.cmd.facing))>0.02;
  steering=turning&&mag>0.05;

  logFrame++;
  logRec({t:'in',f:logFrame,st:+simT.toFixed(3),
    L:[+lx.toFixed(3),+ly.toFixed(3)],R:[+rx.toFixed(3),+ry.toFixed(3)],
    want:[+gait.want.tx.toFixed(3),+gait.want.tz.toFixed(3),+gait.want.facing.toFixed(4)],
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
  const t0=performance.now(); let n=0;
  while(accum>=SIM_DT && n<3*simSteps){
    const before=world.breakEvents.length;
    const stEst=groundTruthState(rig,world);
    gait.update(stEst,SIM_DT);
    if(cmg){ cmg.targetYaw=gait.cmd.facing; cmg.update(stEst,SIM_DT); }
    world.step(SIM_DT);
    if(world.breakEvents.length>before) slowUntil=now+1500;
    accum-=SIM_DT; n++; simT+=SIM_DT;
    checkFall();
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
    fc.textContent='FELL at '+fellAtSteps+' steps / '+fellAtTravel.toFixed(1)+' m'; }
  document.getElementById('c-steer').style.display=steering?'':'none';
  const cc=document.getElementById('c-cmg');
  if(cmg&&cmgOn){ cc.style.display='';
    /* Momentum saturation is the gyro's real failure mode: at 100% the wheel is spun up
       and has nothing left to give, whatever its torque rating says. Shown, not hidden. */
    cc.textContent='gyro '+(cmg.tauFrac*100).toFixed(0)+'% · store '+(cmg.satFrac*100).toFixed(0)+'%';
    cc.className=cmg.satFrac>0.95?'chip warn':'chip';
  } else cc.style.display='none';

  /* ---- 10 Hz body state, plus events ---- */
  if(logFrame%6===0){
    const b=rig.bodies, u=qrot(b.torso.q,V(0,1,0));
    const jt={};
    for(const n2 of jointNames){ const j=rig.joints[n2];
      jt[n2]=[+(j.tau/1e3).toFixed(2),+j.util.toFixed(3),+(j.angle*57.2958).toFixed(1),
              j.saturated?1:0,j.onStop?1:0,j.broken?1:0]; }
    logRec({t:'st',f:logFrame,st:+simT.toFixed(3),
      pel:[+b.pelvis.x.x.toFixed(3),+b.pelvis.x.y.toFixed(3),+b.pelvis.x.z.toFixed(3)],
      pv:[+b.pelvis.v.x.toFixed(3),+b.pelvis.v.y.toFixed(3),+b.pelvis.v.z.toFixed(3)],
      up:+u.y.toFixed(4),
      cf:[+((b.footL.contactForce||0)/1e3).toFixed(1),+((b.footR.contactForce||0)/1e3).toFixed(1)],
      cop:[b.footL.contactCop?+b.footL.contactCop.x.toFixed(3):null,
           b.footR.contactCop?+b.footR.contactCop.x.toFixed(3):null],
      dcm:gait.debug&&gait.debug.dcmErrX!==undefined?
        [+gait.debug.dcmErrX.toFixed(4),+gait.debug.dcmErrZ.toFixed(4)]:null,
      cmg:cmg&&cmgOn?[+cmg.tauFrac.toFixed(3),+cmg.satFrac.toFixed(3)]:null,
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