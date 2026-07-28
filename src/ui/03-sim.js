/* ---------- sim ---------- */
let world,rig,gait,cmg=null,meshes={},rowEls={},jointNames=[],preset='light';
let stepCount=0,simMs=0,slowUntil=0;
/* Honest failure state. The original artifact had none of this: once the rig went down it
   kept reporting its step count and speed from the floor, and a torn limb only ever showed
   as a small counter. A viewer had no way to tell a walk from a collapse. */
let rigHeight=1, startPelvisY=0, fallen=false, fellAtSteps=0, fellAtTravel=0, simT=0, startX=0, startZ=0;
/* Body weight in newtons, so contact force can be logged as a fraction of it. Every load
   question on this project is asked that way and the raw kN figure has already been
   mis-scaled once. */
let rigWeight=1;
let steering=false;
/* Travel cap, m per step. Measured on the MK1.1 build (before the yaw ring) at 1.16 m for
   300 s; the yaw ring is new geometry and has only been swept to 0.62 so far, so the cap
   ships at 0.62 with the envelope control below able to push past it. */
const MAX_STRIDE=0.62, ENVELOPE_STRIDE=1.00;
/* There is deliberately no MAX_TURN constant. It was 8 deg/s, an absolute figure from
   before the rig could be resized, and it was the same bug class as the MAX_STRIDE
   fallback below it: a stale absolute overriding a derived one. Turning is now ONE
   derived parameter -- yawPerStep, how far the body may turn in a single step -- and the
   slew rate falls out of it and the step cycle (rig/derive.js), so the two cannot
   disagree. Both are handed to the controller from deriveGait, never from a constant. */
/* Travel slew, m of per-step travel per second of command. The shipped value was 0.06,
   which needs 16.7 s to go from a standstill to a 1.00 m stride -- your input arrived
   roughly eighteen seconds late and the machine felt like a recording. Swept at 0.06,
   0.3, 0.6 and 1.2 against both an ordinary driving profile and an adversarial one; no
   rate survived hard direction reversals, and no rate was WORSE than 0.06 either, so the
   limiter was never buying the stability it cost. 0.6 is ten times more responsive and
   measured no worse. */
const TRAVEL_RATE=0.6;
/* Stick thresholds, named once. There used to be three separate magic numbers -- 0.12 to
   read an axis, 0.15 to auto-face, 0.35 before the aim stick did anything at all -- and
   the gap between the first and last is what made the right stick feel dead. */
const STICK_DEAD=0.12, AIM_MIN=0.18;
let pushEnvelope=false, orbitBtn=0, zoomBtn=0, fallLogged=false, breakLogged=0;
/* Turn pad, -1 / 0 / +1. Set by the hold handlers in ui/07-camera-pad.js, consumed in
   ui/11-loop.js. Declared here with the other input state because top-level const/let in the
   concatenated blocks share ONE global lexical scope -- declaring it in two modules is a
   load-time SyntaxError, which is how a blank page happened before. */
let turnBtn=0;
let strideCap=MAX_STRIDE, envCap=ENVELOPE_STRIDE, cmgOn=true;
/* ONE SIZE. Three were carried because the Froude scaling needed somewhere to be checked,
   and the check is done -- the servo loop measures Froude-invariant to five digits at
   4/2/1 ft, so the other two sizes were re-testing arithmetic rather than the machine.
   1 ft is the one that drives best and the one the numbers favour: standing creep goes
   43.7 -> 14.2 mm/s from 4 ft to 1 ft on the Light Frame, against 21.4 predicted by
   Froude alone, so it is 34% better in dimensionless terms rather than merely slower.
   The size button is hidden rather than deleted, and SIZES stays an array, so restoring a
   size is one line and none of the scale plumbing rots in the meantime. */
/* The NUMBER comes from rig/build.js (DISPLAY_H), which is where the sim can reach it and
   the test suites can import it. This file owns the label and the size button, not the
   height -- it was a literal here and at three other sites, one of them a regex scrape of
   this very line. */
const SIZES=[{h:DISPLAY_H,label:'1 ft'}];
let sizeIdx=0, SIM_DT=1/60, simSteps=1;

/* SPECS and CONTROLLERS moved to rig/build.js with the rest of world construction. They
   were left here as well, and top-level `const` in classic <script> blocks shares ONE
   global lexical scope, so the duplicate declaration killed the page at load -- while a
   per-block parse check still reported "parses", because each block is legal alone. */

function buildWorld(k){
  const P=PRESETS[k]||PRESETS.light;   // Reference/Overdriven are gone; Light Frame is the fallback
  for(const m of Object.values(meshes)) scene.remove(m);
  meshes={};
  /* ONE construction site. Everything from "assemble a probe to learn the native height"
     through "hand the controller its derived gait constants" now lives in rig/build.js and
     is shared with the test harness. It was inlined here and re-implemented in
     test/battery.mjs, where the copy omitted `spec` and silently built an MK1 for every
     preset -- so every non-MK1 number the battery ever printed described a machine nobody
     was driving. Module loading was already de-duplicated; this is world construction,
     which is where the drift actually kept happening. */
  const B=buildRig(k,{height:SIZES[sizeIdx].h,cmgOn:cmgOn});
  world=B.world; rig=B.rig; gait=B.gait; cmg=B.cmg;
  SIM_DT=B.SIM_DT; simSteps=B.simSteps;
  const SC=B.SC, dg=B.dg, gv=B.gravity;
  strideCap=B.strideCap;
  spanWant=dg.height*2.1; span=spanWant;   // frame every rig to its own height
  spanMin=dg.height*1.2; spanMax=dg.height*9;   // zoom bounds follow the rig too
  envCap=P.envCap!==undefined?P.envCap:strideCap*1.5;   // deliberate over-drive band
  document.getElementById('c-mass').innerHTML=(rig.spec.name||'MK1').toUpperCase()+' '+BUILD+' · <b>'+
    (rigStats(rig).mass<50?rigStats(rig).mass.toFixed(1)+' kg':
      Math.round(rigStats(rig).mass).toLocaleString('en-US').replace(/,/g,' ')+' kg')+
    ' · '+rigStats(rig).height.toFixed(2)+' m</b>';
  /* A rig with no gyro must not offer a gyro switch. The quadruped has none by design --
     a machine that never leaves static stability has nothing for one to do -- and the
     button silently did nothing on it. */
  document.getElementById('cmgBtn').style.display=cmg?'':'none';
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
  rigWeight=Math.max(1e-9,rigStats(rig).mass*gv);
  fallen=false; fellAtSteps=0; fellAtTravel=0; simT=0; fallLogged=false; breakLogged=0;
  document.getElementById('c-torn').style.display='none';
  document.getElementById('c-fall').style.display='none';
  /* Record the DERIVED configuration, per build, into the driving log. The session header
     can only carry the module-level constants -- it is written before any rig exists -- so
     everything that actually governs the walk used to be absent from the one evidence
     source this project trusts. Every stale-constant bug found so far was a derived value
     disagreeing with a hardcoded one, and none of them were visible in a log. */
  logEvent('build',{preset:k,h:SIZES[sizeIdx].h,scale:+SC.toFixed(4),
    legs:rig.legs,gaitKind:rig.gait,
    /* Native height from the scale, not from a probe rig: the probe now lives inside
       buildRig() and this line was left referencing it -- a ReferenceError thrown on the
       first build event, i.e. every page load. */
    nativeH:+(SIZES[sizeIdx].h/SC).toFixed(2),ticks:simSteps,
    mass:+rigStats(rig).mass.toFixed(1),stride:+strideCap.toFixed(3),env:+envCap.toFixed(3),
    tSS:+dg.tSS.toFixed(3),tDS:+dg.tDS.toFixed(3),turnRate:+dg.turnRate.toFixed(4),
    yawPerStep:+dg.yawPerStep.toFixed(4),
    cop:[+dg.copLimitX.toFixed(3),+dg.copLimitZ.toFixed(3)],
    tau:Object.fromEntries(jointNames.map(n=>[n,+(rig.joints[n].tauMax/1e3).toFixed(1)]))});
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
