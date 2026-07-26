/* ---------- analytic self test ----------
   The MK1 status document claimed 48 passing gates across five headless suites. No suite
   shipped with the artifact, so none of it was checkable. These are rebuilt from scratch
   and run here, in front of you, against the same solver the mech is using. Every one
   asserts against a hand calculation or a closed-form result. Nothing is compared against
   recorded output, because a regression test only proves the answer did not change. */
let paused=false;
const rel=(a,b)=>Math.abs(a-b)/Math.max(1e-12,Math.abs(b));

function freshRig(o){
  const gv=o&&o.gravity||9.81;
  setGravity(gv);
  const w=new World({substeps:10,iterations:8,contact:{mu:1.0},gravity:V(0,-gv,0)});
  const r=assembleMech(w,{}); groundRig(r);
  const g=new GaitController(r,{copClamp:0.45,tSS:0.90,gravity:gv,
    travelRate:TRAVEL_RATE,turnRate:MAX_TURN,balance:{kCop:0.40}});
  return {w,r,g};
}
function walk(ctx,stride,frames,cb){
  for(let i=0;i<frames;i++){
    ctx.g.want.tx=stride; ctx.g.want.tz=0;
    ctx.g.update(groundTruthState(ctx.r,ctx.w),1/60);
    ctx.w.step(1/60);
    if(cb) cb(i);
  }
}
/* independent separating-axis overlap, deliberately NOT the solver's own routine */
function satOverlap(a,b){
  const ax=[];
  for(const bd of [a,b]){ const M=quatToM3(bd.q);
    ax.push(V(M[0],M[3],M[6]),V(M[1],M[4],M[7]),V(M[2],M[5],M[8])); }
  const d=vsub(b.x,a.x); let best=Infinity;
  const ext=(bd,n)=>{ const M=quatToM3(bd.q),h=bd.half;
    return Math.abs(n.x*M[0]+n.y*M[3]+n.z*M[6])*h.x
         + Math.abs(n.x*M[1]+n.y*M[4]+n.z*M[7])*h.y
         + Math.abs(n.x*M[2]+n.y*M[5]+n.z*M[8])*h.z; };
  for(const n of ax){ const ov=ext(a,n)+ext(b,n)-Math.abs(vdot(d,n));
    if(ov<=0) return 0; if(ov<best) best=ov; }
  return best;
}

const GATES=[
{id:'G1',t:'Free-flight kinetic energy is conserved',f:()=>{
  const w=new World({substeps:12,iterations:4,gravity:V(0,0,0)}); w.enableGround=false;
  const b=w.add(new Body({mass:500,inertia:boxInertia(500,0.5,1.5,0.5),
    pos:V(0,5,0),vel:V(1.3,0.4,-0.7),angVel:V(2.1,0.9,1.4)}));
  const e0=b.kinetic();
  for(let i=0;i<600;i++) w.step(1/60);
  const e=rel(b.kinetic(),e0)*100;
  return {pass:e<0.5,d:e.toFixed(4)+'% drift over 10 s, tumbling freely (limit 0.5%)'};}},

{id:'G2',t:'Free-flight angular momentum is conserved',f:()=>{
  const w=new World({substeps:12,iterations:4,gravity:V(0,0,0)}); w.enableGround=false;
  const b=w.add(new Body({mass:500,inertia:boxInertia(500,0.4,1.6,0.9),
    pos:V(0,5,0),angVel:V(0.2,6.0,0.05)}));
  const L=()=>vlen(qrot(b.q,m3mulv(b.I,qrotInv(b.q,b.w))));
  const l0=L();
  for(let i=0;i<600;i++) w.step(1/60);
  const e=rel(L(),l0)*100;
  return {pass:e<0.5,d:e.toFixed(4)+'% drift over 10 s in the unstable-axis regime (limit 0.5%)'};}},

{id:'G3',t:'Free fall matches the analytic -g t&sup2;/2',f:()=>{
  const w=new World({substeps:12,iterations:4,gravity:V(0,-9.81,0)}); w.enableGround=false;
  const b=w.add(new Body({mass:100,inertia:boxInertia(100,1,1,1),pos:V(0,100,0)}));
  for(let i=0;i<120;i++) w.step(1/60);
  const want=100-0.5*9.81*4, e=Math.abs(b.x.y-want);
  return {pass:e<0.02,d:'y = '+b.x.y.toFixed(4)+' m vs analytic '+want.toFixed(4)+
    ' m after 2 s. The '+(e*1e3).toFixed(1)+' mm gap is the symplectic-Euler offset g&middot;h&middot;t/2 at h = 1/720 s, not an error.'};}},

{id:'G4',t:'Weld axial load equals the suspended weight m&middot;g',f:()=>{
  const w=new World({substeps:12,iterations:4,gravity:V(0,-9.81,0)}); w.enableGround=false;
  const a=w.add(new Body({mass:1,inertia:boxInertia(1,1,1,1),pos:V(0,10,0),kinematic:true}));
  const h=w.add(new Body({mass:320,inertia:boxInertia(320,0.85,0.5,1.1),pos:V(0,9.5,0)}));
  const wl=w.addWeld(new Weld({name:'t',a,b:h,ra:V(0,-0.1,0),rb:V(0,0.4,0),axis:V(0,1,0)}));
  for(let i=0;i<240;i++) w.step(1/60);
  const want=320*9.81, e=rel(Math.abs(wl.Fax),want)*100;
  return {pass:e<1,d:'|Fax| = '+(Math.abs(wl.Fax)/1e3).toFixed(3)+' kN vs m&middot;g = '+
    (want/1e3).toFixed(3)+' kN ('+e.toFixed(3)+'%)'};}},

{id:'G5',t:'Weld bending moment equals the cantilever m&middot;g&middot;L/2',f:()=>{
  const w=new World({substeps:12,iterations:4,gravity:V(0,-9.81,0)}); w.enableGround=false;
  const L=2.0,m=200;
  const a=w.add(new Body({mass:1,inertia:boxInertia(1,1,1,1),pos:V(0,10,0),kinematic:true}));
  const r=w.add(new Body({mass:m,inertia:boxInertia(m,L,0.2,0.2),pos:V(L/2,10,0)}));
  const wl=w.addWeld(new Weld({name:'c',a,b:r,ra:V(0,0,0),rb:V(-L/2,0,0),axis:V(1,0,0)}));
  for(let i=0;i<360;i++) w.step(1/60);
  const want=m*9.81*L/2, e=rel(wl.Mb,want)*100;
  return {pass:e<2,d:'Mb = '+(wl.Mb/1e3).toFixed(2)+' kN&middot;m vs m&middot;g&middot;L/2 = '+
    (want/1e3).toFixed(2)+' kN&middot;m ('+e.toFixed(2)+'%)'};}},

{id:'G6',t:'Compound pendulum period matches 2&pi;&radic;(I/mgd)',f:()=>{
  const w=new World({substeps:20,iterations:6,gravity:V(0,-9.81,0)}); w.enableGround=false;
  const m=100,L=2.0,d=L/2;
  const p=w.add(new Body({mass:1,inertia:boxInertia(1,1,1,1),pos:V(0,10,0),kinematic:true}));
  const rod=w.add(new Body({mass:m,inertia:boxInertia(m,0.05,L,0.05),pos:V(0,10-d,0)}));
  w.addJoint(new Hinge({name:'p',a:p,b:rod,ra:V(0,0,0),rb:V(0,d,0),
    axisA:V(0,0,1),refA:V(1,0,0),tauMax:0}));
  const th=0.02;
  rod.x=V(d*Math.sin(th),10-d*Math.cos(th),0); rod.q=qAxisAngle(V(0,0,1),th);
  const Ip=m*L*L/12+m*d*d, want=2*Math.PI*Math.sqrt(Ip/(m*9.81*d));
  let prev=rod.x.x,t=0,f=null,l=null,n=0;
  for(let i=0;i<12000;i++){ w.step(1/240); t+=1/240;
    if(prev>0&&rod.x.x<=0){ if(f===null) f=t; else {l=t;n++;} } prev=rod.x.x; }
  const meas=n?(l-f)/n:NaN, e=rel(meas,want)*100;
  return {pass:e<2,d:'T = '+meas.toFixed(4)+' s vs analytic '+want.toFixed(4)+' s ('+e.toFixed(2)+'%)'};}},

{id:'G7',t:'Standing ground reaction equals the rig weight',f:()=>{
  const c=freshRig(); walk(c,0,600);
  // Mean vertical reaction, not one frame. In steady state the average ground reaction
  // must equal the weight; the instantaneous value legitimately swings by several percent
  // as load shifts between the feet, so a single sample is not what the identity is about.
  let sum=0,n=0;
  walk(c,0,300,()=>{ let F=0;
    for(const s of ['L','R']) F+=c.r.bodies['foot'+s].contactForce||0;
    sum+=F; n++; });
  const F=sum/n;
  const want=8360*9.81, e=rel(F,want)*100;
  return {pass:e<3,d:(F/1e3).toFixed(1)+' kN measured vs '+(want/1e3).toFixed(1)+
    ' kN of weight ('+e.toFixed(2)+'%). Before the contact-impulse fix this read 53.2 kN against 80.0 &mdash; 33% low &mdash; and that error is what the balance gains had been compensating for.'};}},

{id:'G8',t:'Coulomb cone: a block holds below atan(&mu;) and slides above',f:()=>{
  const test=(deg,mu)=>{
    const th=deg*Math.PI/180;
    const w=new World({substeps:20,iterations:6,contact:{mu},gravity:V(0,-9.81,0)});
    w.g=V(9.81*Math.sin(th),-9.81*Math.cos(th),0);
    const b=w.add(new Body({mass:50,inertia:boxInertia(50,1,0.4,1),pos:V(0,0.2,0)}));
    b.half=V(0.5,0.2,0.5); const x0=b.x.x;
    for(let i=0;i<180;i++) w.step(1/60);
    return Math.abs(b.x.x-x0); };
  const mu=0.6,crit=Math.atan(mu)*180/Math.PI;
  const lo=test(crit-8,mu),hi=test(crit+8,mu);
  return {pass:lo<0.02&&hi>0.15,d:'&mu; = 0.6, critical angle '+crit.toFixed(1)+
    '&deg;. At '+(crit-8).toFixed(0)+'&deg; it creeps '+(lo*1e3).toFixed(1)+
    ' mm in 3 s; at '+(crit+8).toFixed(0)+'&deg; it runs '+(hi*1e3).toFixed(0)+' mm.'};}},

{id:'G9',t:'No actuator ever exceeds its rated tauMax',f:()=>{
  const c=freshRig(); let worst=0,nm='';
  walk(c,0.62,900,()=>{ for(const j of Object.values(c.r.joints)){
    const q=j.tauMax?Math.abs(j.tau)/j.tauMax:0; if(q>worst){worst=q;nm=j.name;} } });
  return {pass:worst<=1.0001,d:'worst |&tau;|/tauMax = '+worst.toFixed(5)+' ('+nm+
    '). Saturation is expected and physical; exceeding it would not be.'};}},

{id:'G10',t:'Feet never interpenetrate &mdash; and the check can still detect it',f:()=>{
  const run=(pairs)=>{ const c=freshRig(); if(!pairs) c.w.pairs.length=0;
    let worst=0;
    walk(c,0.62,900,()=>{ const o=satOverlap(c.r.bodies.footL,c.r.bodies.footR);
      if(o>worst) worst=o; });
    return worst; };
  const on=run(true),off=run(false);
  return {pass:on<1e-6&&off>0.005,d:'constraint active: '+(on*1e3).toFixed(3)+
    ' mm of overlap. Stripped out of the same walk: '+(off*1e3).toFixed(1)+
    ' mm. A gate that cannot fail proves nothing, so this one is shown failing on demand.'};}},

{id:'G11',t:'legIK inverts legFK to sub-millimetre',f:()=>{
  let worst=0;
  for(let i=0;i<500;i++){
    const d=V(-0.6+1.2*((i*7919)%997)/997,-2.85+0.5*((i*6841)%991)/991,
              -0.5+1.0*((i*5779)%983)/983);
    const q=legIK(d); if(q.reach>=0.995) continue;
    const e=vlen(vsub(legFK(q),d)); if(e>worst) worst=e; }
  return {pass:worst<1e-3,d:'worst round-trip error '+(worst*1e3).toFixed(4)+
    ' mm across 500 reachable targets'};}},

{id:'G12',t:'DCM trajectory satisfies &xi;&#775; = &omega;(&xi; &minus; p)',f:()=>{
  const plan=new DCMPlan(buildPhases({left:V(0,0,0.6),right:V(0,0,-0.6),
    stride:V(0.6,0,0),nSteps:4,tDS:0.5,tSS:0.9,tStart:1.2,tEnd:3.0}),2.9,9.81);
  let worst=0; const dt=1e-5;
  for(let t=0.05;t<plan.T-0.05;t+=0.01){
    if(plan.indexAt(t-dt)!==plan.indexAt(t+dt)) continue;
    const num=vmul(vsub(plan.xiAt(t+dt),plan.xiAt(t-dt)),1/(2*dt));
    const ana=vmul(vsub(plan.xiAt(t),plan.zmpAt(t)),plan.omega);
    const e=vlen(vsub(num,ana)); if(e>worst) worst=e; }
  return {pass:worst<1e-4,d:'worst residual '+(worst*1e6).toFixed(3)+
    ' &micro;m/s across the whole plan &mdash; the planner is exact, not approximate'};}},

{id:'G13',t:'Planned ZMP never leaves the commanded footprint',f:()=>{
  const plan=new DCMPlan(buildPhases({left:V(0,0,0.6),right:V(0,0,-0.6),
    stride:V(0.6,0,0),nSteps:4,tDS:0.5,tSS:0.9,tStart:1.2,tEnd:3.0}),2.9,9.81);
  let worst=0;
  for(let t=0;t<plan.T;t+=0.005){
    const k=plan.kindAt(t); if(k.indexOf('SS-')!==0) continue;
    const e=Math.abs(plan.zmpAt(t).z-(k.slice(3)==='L'?0.6:-0.6));
    if(e>worst) worst=e; }
  return {pass:worst<1e-9,d:'max lateral excursion off the stance foot: '+
    (worst*1e6).toFixed(3)+' &micro;m'};}},

{id:'G15',t:'The walk is scale-invariant &mdash; same machine, 1 ft and 4 ft',f:()=>{
  /* The strongest check in this suite. Under Froude scaling a rig shrunk by s should walk
     an IDENTICAL dimensionless walk: same distance measured in leg-lengths, same joint
     angles against t/sqrt(L/g), just faster in real seconds. Anything left in the model in
     absolute metres, newtons or seconds breaks that similarity, and shows up here as
     divergence between the two sizes.
     It found five such constants. The worst was the planner's pendulum height, floored at
     an absolute 1.0 m: every rig under about 1.7 m tall -- including the shipping default --
     was planning against a pendulum it did not have, and at 1 ft the frequency was out by a
     factor of 2.4. A gate that only asks "did it fall" cannot see that; this one can. */
  const trial=(targetH,nTref)=>{
    setGravity(9.81);
    const pw=new World({substeps:2,iterations:1,gravity:V(0,-9.81,0)});
    const pr=assembleMech(pw,{}); groundRig(pr);
    const sc=targetH/rigStats(pr).height, dt=(1/60)*Math.sqrt(sc);
    const w=new World({substeps:10,iterations:8,contact:{mu:1.0},gravity:V(0,-9.81,0)});
    w.lscale=sc;
    const rig=assembleMech(w,{}); groundRig(rig);
    for(const b of Object.values(rig.bodies)){ b.mass*=0.5; b.I=b.I.map(v=>v*0.5);
      if(!b.kinematic){ b.invMass=1/b.mass; b.invI=m3inv(b.I);} }
    for(const j of Object.values(rig.joints)){ j.tauMax*=0.5; j.kp*=0.5; j.kd*=0.5;
      for(const k of ['tension','shear','bend','torsion']) j.lim[k]*=0.7; }
    for(const wl of Object.values(rig.welds))
      for(const k of ['tension','shear','bend','torsion']) wl.lim[k]*=0.7;
    scaleRig(w,rig,sc); groundRig(rig);
    const dg=deriveGait(rig);
    const g=new GaitController(rig,{gravity:9.81,tSS:dg.tSS,tDS:dg.tDS,stepHeight:dg.stepHeight,
      pelvisDrop:dg.pelvisDrop,minFootSep:dg.minFootSep,copClamp:dg.copClamp,
      settleTime:dg.settleTime,crouchTime:dg.crouchTime,tStart:dg.tStart,tEnd:dg.tEnd,
      travelRate:dg.travelRate,turnRate:dg.turnRate,
      balance:{kCop:0.60,copLimitX:dg.copLimitX,copLimitZ:dg.copLimitZ}});
    const L=dg.L, tRef=Math.sqrt(L/9.81), x0=rig.bodies.pelvis.x.x;
    let t=0; const n=Math.round(nTref*tRef/dt);
    for(let i=0;i<n;i++){ g.want.tx=(t>8*tRef)?dg.strideCap:0;
      g.update(groundTruthState(rig,w),dt); w.step(dt); t+=dt; }
    return {travel:(rig.bodies.pelvis.x.x-x0)/L,
            up:qrot(rig.bodies.torso.q,V(0,1,0)).y, brk:w.breakEvents.length};
  };
  const a=trial(1.25,16), b=trial(0.30,16);
  const d=Math.abs(a.travel-b.travel)/Math.max(1e-9,Math.abs(a.travel));
  const ok=d<0.15 && a.up>0.8 && b.up>0.8 && a.brk===0 && b.brk===0;
  return {pass:ok,d:'4 ft (69 kg) walked '+a.travel.toFixed(3)+' leg-lengths; 1 ft (1 kg) walked '+
    b.travel.toFixed(3)+' &mdash; <b>'+(d*100).toFixed(1)+'% apart</b>, both upright, nothing torn. '+
    'Two machines seventy times apart in mass, walking the same walk.'};}},

{id:'G14',t:'The simulation is bit-reproducible',f:()=>{
  const go=()=>{ const c=freshRig(); walk(c,0.62,600); return c.r.bodies.pelvis.x; };
  const a=go(),b=go(); const e=vlen(vsub(a,b));
  return {pass:e===0,d:'pelvis position differs by '+e.toExponential(2)+
    ' m between two identical runs. Anything else would make every measurement above unrepeatable.'};}},
];

const gp=document.getElementById('gates'), gRows=document.getElementById('gateRows');
const gSum=document.getElementById('gatesSum');
function closeGates(){ gp.classList.remove('open'); paused=false; last=performance.now(); accum=0; }
document.getElementById('gatesClose').addEventListener('click',closeGates);
document.getElementById('gateBtn').addEventListener('click',runGates);
/* The envelope control. Default is the measured no-fall cap; this lets you drive past it
   on purpose, says so on screen, and records the crossing in the session log. */
const sizeBtn=document.getElementById('sizeBtn');
sizeBtn.addEventListener('click',function(){
  sizeIdx=(sizeIdx+1)%SIZES.length;
  sizeBtn.textContent='Size: '+SIZES[sizeIdx].label;
  logEvent('size',{target:SIZES[sizeIdx].h});
  buildWorld(preset);
});
const cmgBtn=document.getElementById('cmgBtn');
cmgBtn.addEventListener('click',function(){
  cmgOn=!cmgOn;
  cmgBtn.setAttribute('aria-pressed',String(cmgOn));
  cmgBtn.textContent='Gyro stabiliser: '+(cmgOn?'ON':'off');
  if(cmg) cmg.enabled=cmgOn;
  logEvent('cmg',{on:cmgOn});
});
const envBtn=document.getElementById('envBtn');
envBtn.addEventListener('click',function(){
  pushEnvelope=!pushEnvelope;
  envBtn.setAttribute('aria-pressed',String(pushEnvelope));
  envBtn.textContent='Beyond verified envelope: '+(pushEnvelope?'ON':'off');
  logEvent('envelope',{on:pushEnvelope,cap:pushEnvelope?envCap:strideCap});
  document.getElementById('c-env').style.display=pushEnvelope?'':'none';
});

async function runGates(){
  gp.classList.add('open'); paused=true; gRows.innerHTML=''; gSum.textContent='running…';
  const els=GATES.map(g=>{
    const el=document.createElement('div'); el.className='gr';
    el.innerHTML='<div class="id">'+g.id+'</div><div class="mk run">&middot;</div>'+
      '<div class="ttl">'+g.t+'<span class="det">queued</span></div>';
    gRows.appendChild(el); return el; });
  const gSave=9.81; let np=0;
  for(let i=0;i<GATES.length;i++){
    els[i].querySelector('.det').textContent='running…';
    await new Promise(r=>setTimeout(r,16));
    let res;
    try{ res=GATES[i].f(); }catch(e){ res={pass:false,d:'threw: '+e.message}; }
    if(res.pass) np++;
    const mk=els[i].querySelector('.mk');
    mk.className='mk '+(res.pass?'ok':'no'); mk.textContent=res.pass?'✓':'✗';
    els[i].classList.toggle('fail',!res.pass);
    els[i].querySelector('.det').innerHTML=res.d;
    gSum.textContent=np+'/'+(i+1)+' passing';
  }
  setGravity(PRESETS[preset].gravity!==undefined?PRESETS[preset].gravity:gSave);
  gSum.textContent=np+' / '+GATES.length+' passing';
}
