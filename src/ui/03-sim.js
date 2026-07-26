/* ---------- sim ---------- */
let world,rig,gait,cmg=null,meshes={},rowEls={},jointNames=[],preset='light';
let stepCount=0,simMs=0,slowUntil=0;
/* Honest failure state. The original artifact had none of this: once the rig went down it
   kept reporting its step count and speed from the floor, and a torn limb only ever showed
   as a small counter. A viewer had no way to tell a walk from a collapse. */
let rigHeight=1, startPelvisY=0, fallen=false, fellAtSteps=0, fellAtTravel=0, simT=0, startX=0, startZ=0;
let steering=false;
/* Travel cap, m per step. Measured on the MK1.1 build (before the yaw ring) at 1.16 m for
   300 s; the yaw ring is new geometry and has only been swept to 0.62 so far, so the cap
   ships at 0.62 with the envelope control below able to push past it. */
const MAX_STRIDE=0.62, ENVELOPE_STRIDE=1.00;
/* Facing rate. 8 deg/s turns in place indefinitely on this build. Turning WHILE
   travelling still falls at 8.4 s and is flagged live, not hidden. */
const MAX_TURN=8*Math.PI/180;
/* Travel slew, m of per-step travel per second of command. The shipped value was 0.06,
   which needs 16.7 s to go from a standstill to a 1.00 m stride -- your input arrived
   roughly eighteen seconds late and the machine felt like a recording. Swept at 0.06,
   0.3, 0.6 and 1.2 against both an ordinary driving profile and an adversarial one; no
   rate survived hard direction reversals, and no rate was WORSE than 0.06 either, so the
   limiter was never buying the stability it cost. 0.6 is ten times more responsive and
   measured no worse. */
const TRAVEL_RATE=0.6;
let pushEnvelope=false, orbitBtn=0, zoomBtn=0, fallLogged=false, breakLogged=0;
let strideCap=MAX_STRIDE, envCap=ENVELOPE_STRIDE, cmgOn=true, turnCap=MAX_TURN;
/* Target standing height, metres. Scale is derived per preset so every machine comes out
   the same height whatever its native proportions.
   1 ft is what was asked for and it does NOT hold up -- it falls around 11 s. That is not
   a resolution problem: doubling the control rate and doubling the substeps both made it
   slightly worse, so the solve has converged and something in the model genuinely does not
   scale. 0.30 m ships anyway, labelled, because you asked for it. 1.25 m walks clean. */
const SIZES=[{h:1.25,label:'4 ft'},{h:0.60,label:'2 ft'},{h:0.30,label:'1 ft'}];
let sizeIdx=0, SIM_DT=1/60, simSteps=1;

function buildWorld(k){
  const P=PRESETS[k]||PRESETS.verified;
  for(const m of Object.values(meshes)) scene.remove(m);
  meshes={};
  const gv=P.gravity!==undefined?P.gravity:9.81;
  setGravity(gv);
  /* 10 substeps x 8 iterations. The hip yaw ring makes each leg an 8-link chain, and a
     Gauss-Seidel sweep propagates one link per iteration, so the count had to rise with
     the chain: 6 iterations walks in place but falls when travelling, 8 walks. Measured,
     and it is iterations rather than substeps that matters -- 8 substeps x 8 fails. */
  /* Froude scaling raises every natural frequency by 1/sqrt(scale), and the timestep is
     fixed by the display at 1/60 s, so substep count has to rise with it or the solver
     simply cannot resolve the motion -- at 1/10 scale with 10 substeps the rig detonated
     on the first contact. */
  /* Assemble once unscaled to learn the native height, then pick the scale that lands this
     preset on the selected target height. */
  const probeW=new World({substeps:2,iterations:1,gravity:V(0,-gv,0)});
  const probeR=assembleMech(probeW,{spec:P.spec==='atst'?ATST_SPEC:undefined,
                                    footWidth:P.footWidth,hipOffset:P.hipOffset});
  groundRig(probeR);
  const SC=SIZES[sizeIdx].h/rigStats(probeR).height;
  const subs=10;
  /* Froude: time goes as sqrt(scale), so the CONTROL loop and the physics both have to run
     that much faster. Pinning them to the 60 Hz display frame is what made small rigs
     collapse -- the plant sped up and the controller did not. */
  SIM_DT=(1/60)*Math.sqrt(SC);
  simSteps=Math.max(1,Math.round((1/60)/SIM_DT));
  world=new World({substeps:subs,iterations:8,contact:{mu:P.friction!==undefined?P.friction:1.0},
                   gravity:V(0,-gv,0)});
  world.lscale=SC;
  rig=assembleMech(world,{spec:P.spec==='atst'?ATST_SPEC:undefined,
                          footWidth:P.footWidth,hipOffset:P.hipOffset});
  groundRig(rig);
  applyPreset(rig,P);
  /* The gyro is real hardware: it adds mass to the torso before anything is measured. */
  scaleRig(world,rig,SC);
  groundRig(rig);
  const dg=deriveGait(rig);
  /* FIX: this used P.scale -- the per-preset scale left over from before the Size
     selector existed -- while the RIG is scaled by SC, which the selector derives from the
     chosen height. They coincide at 4 ft (0.2546 vs 0.25) and diverge badly below it: at
     1 ft SC is 0.0611, so the gyro was being built with 266x its correct torque and a
     flywheel heavier than the torso it was bolted into. It tore the rig on contact. */
  /* IDEAL MODE: mounts get margin so ordinary driving cannot tear a leg -- only genuinely
     extreme abuse (Overdriven's x3 torque, big falls) still breaks things. */
  const FORGIVE=4;
  for(const j of Object.values(rig.joints)) j.lim={tension:j.lim.tension*FORGIVE,
    shear:j.lim.shear*FORGIVE, bend:j.lim.bend*FORGIVE, torsion:j.lim.torsion*FORGIVE};
  for(const wl of Object.values(rig.welds)) wl.lim={tension:wl.lim.tension*FORGIVE,
    shear:wl.lim.shear*FORGIVE, bend:wl.lim.bend*FORGIVE, torsion:wl.lim.torsion*FORGIVE};
  const sc=SC, s3=sc*sc*sc, s4=s3*sc;
  cmg=P.cmg?fitCMG(rig,Object.assign({},P.cmg,{mass:P.cmg.mass*s3,
      tauMax:P.cmg.tauMax*s4, hMax:P.cmg.hMax*s4*Math.sqrt(sc),
      kp:(P.cmg.kp||150e3)*s4, kd:(P.cmg.kd||42e3)*s4*Math.sqrt(sc),
      /* desat is a controller TIME and goes as sqrt(scale) like every other one; left
         absolute it is 4x too slow at 1 ft, the store pins at 100% and the gyro is dead
         weight. */
      desat:(P.cmg.desat??7.0)*Math.sqrt(sc), enabled:cmgOn})):null;
  gait=new GaitController(rig,Object.assign({
    gravity:gv, tSS:dg.tSS, tDS:dg.tDS, stepHeight:dg.stepHeight,
    settleTime:dg.settleTime, crouchTime:dg.crouchTime, tStart:dg.tStart, tEnd:dg.tEnd,
    pelvisDrop:dg.pelvisDrop, minFootSep:dg.minFootSep, copClamp:dg.copClamp,
    travelRate:dg.travelRate, turnRate:dg.turnRate,
    balance:Object.assign({kCop:P.kCop!==undefined?P.kCop:0.40,
      copLimitX:dg.copLimitX, copLimitZ:dg.copLimitZ},P.balance||{})},P.gait||{}));
  turnCap=dg.turnRate;
  /* FIX: this fell back to MAX_STRIDE -- 0.62 m, an absolute figure from when the rig was
     full size -- instead of the stride derived for THIS rig at THIS size. The presets stopped
     defining their own, so every size got a 0.62 m cap: 3x the working stride at 4 ft and 12x
     it at 1 ft. Driving logs show the consequence plainly, foot mounts tearing at util 1.0-1.16
     a few seconds in, at every size. The stick must not be able to command a stride the
     machine cannot take. */
  strideCap=P.strideCap!==undefined?P.strideCap:dg.strideCap;
  spanWant=dg.height*2.1; span=spanWant;   // frame every rig to its own height
  spanMin=dg.height*1.2; spanMax=dg.height*9;   // zoom bounds follow the rig too
  envCap=P.envCap!==undefined?P.envCap:strideCap*1.5;   // deliberate over-drive band
  document.getElementById('c-mass').innerHTML=(P.spec==='atst'?'SCOUT':'MK1.4')+' · <b>'+
    (rigStats(rig).mass<50?rigStats(rig).mass.toFixed(1)+' kg':
      Math.round(rigStats(rig).mass).toLocaleString('en-US').replace(/,/g,' ')+' kg')+
    ' · '+rigStats(rig).height.toFixed(2)+' m</b>';
  jointNames=Object.keys(rig.joints);
  for(const name of Object.keys(rig.bodies)){
    const b=rig.bodies[name];
    /* Feet DRAW at 70% of their physical plan so they look right; the collision box and
       CoP authority keep the full footprint. Shrinking the physics feet was measured
       fatal (falls inside 5.5 s in every configuration), so the fix is cosmetic only. */
    const cos=name.startsWith('foot')?0.70:1.0;
    const g=new THREE.BoxGeometry(b.dim.x*cos,b.dim.y,b.dim.z*cos);
    const m=new THREE.Mesh(g,new THREE.MeshLambertMaterial({color:0xB9B3A5}));
    m.add(new THREE.LineSegments(new THREE.EdgesGeometry(g),
      new THREE.LineBasicMaterial({color:0x1C1F1A,transparent:true,opacity:.5})));
    scene.add(m); meshes[name]=m;
  }
  const host=document.getElementById('rows'); host.innerHTML=''; rowEls={};
  for(const n of jointNames){
    const el=document.createElement('div'); el.className='jr';
    el.innerHTML='<div class="n">'+n+'</div><div class="bar"><u></u><i></i></div><div class="u">0%</div>';
    host.appendChild(el);
    rowEls[n]={root:el,fill:el.querySelector('i'),util:el.querySelector('.u')};
  }
  stepCount=0;
  startPelvisY=rig.bodies.pelvis.x.y; startX=rig.bodies.pelvis.x.x; startZ=rig.bodies.pelvis.x.z;
  rigHeight=rigStats(rig).height;
  fallen=false; fellAtSteps=0; fellAtTravel=0; simT=0; fallLogged=false; breakLogged=0;
  document.getElementById('c-torn').style.display='none';
  document.getElementById('c-fall').style.display='none';
}

/* A fall is the torso losing upright attitude, or the pelvis dropping far enough that the
   legs have clearly folded. Both thresholds are the ones the headless harness uses, so the
   number you see here is the number that was measured. */
function checkFall(){
  if(fallen) return;
  const upY=qrot(rig.bodies.torso.q,V(0,1,0)).y;
  const drop=startPelvisY-rig.bodies.pelvis.x.y;
  if(upY<0.6||drop>0.30*rigHeight){
    fallen=true; fellAtSteps=gait.stepsTaken||0;
    fellAtTravel=Math.hypot(rig.bodies.pelvis.x.x-startX,rig.bodies.pelvis.x.z-startZ);
  }
}
