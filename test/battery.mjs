/* Driving battery. Imports the sim built from src/ -- same files as the artifact. */
import {buildSim} from './harness.mjs';
const S=await import(buildSim());
const {World,assembleMech,groundRig,rigStats,GaitController,groundTruthState,setGravity,V,qrot,
       PRESETS,applyPreset,scaleRig,deriveGait,fitCMG}=S;
const wrapPi=a=>{while(a>Math.PI)a-=2*Math.PI;while(a<-Math.PI)a+=2*Math.PI;return a;};
export function build(key='light',targetH=1.25,o={}){
  const P=PRESETS[key], gv=9.81; setGravity(gv);
  const pw=new World({substeps:2,iterations:1,gravity:V(0,-gv,0)});
  const pr=assembleMech(pw,{footWidth:P.footWidth,hipOffset:P.hipOffset}); groundRig(pr);
  const SC=targetH/rigStats(pr).height, DT=(1/60)*Math.sqrt(SC);
  const w=new World({substeps:10,iterations:8,contact:{mu:P.friction!==undefined?P.friction:1.0},gravity:V(0,-gv,0)});
  w.lscale=SC;
  const r=assembleMech(w,{footWidth:P.footWidth,hipOffset:P.hipOffset});
  groundRig(r); applyPreset(r,P); scaleRig(w,r,SC); groundRig(r);
  const dg=deriveGait(r), s3=SC**3, s4=s3*SC, F=4;
  for(const j of Object.values(r.joints)) j.lim={tension:j.lim.tension*F,shear:j.lim.shear*F,bend:j.lim.bend*F,torsion:j.lim.torsion*F};
  for(const wl of Object.values(r.welds)) wl.lim={tension:wl.lim.tension*F,shear:wl.lim.shear*F,bend:wl.lim.bend*F,torsion:wl.lim.torsion*F};
  const cmg=P.cmg?fitCMG(r,Object.assign({},P.cmg,{mass:P.cmg.mass*s3,tauMax:P.cmg.tauMax*s4,
    hMax:P.cmg.hMax*s4*Math.sqrt(SC),kp:(P.cmg.kp||150e3)*s4,kd:(P.cmg.kd||42e3)*s4*Math.sqrt(SC),
    desat:7.0*Math.sqrt(SC),enabled:true})):null;
  const g=new GaitController(r,Object.assign({gravity:gv,tSS:dg.tSS,tDS:dg.tDS,stepHeight:dg.stepHeight,
    settleTime:dg.settleTime,crouchTime:dg.crouchTime,tStart:dg.tStart,tEnd:dg.tEnd,pelvisDrop:dg.pelvisDrop,
    minFootSep:dg.minFootSep,copClamp:dg.copClamp,travelRate:dg.travelRate,turnRate:dg.turnRate,
    yawPerStep:dg.yawPerStep,
    balance:Object.assign({kCop:P.kCop!==undefined?P.kCop:0.40,copLimitX:dg.copLimitX,copLimitZ:dg.copLimitZ},P.balance||{})},P.gait||{}));
  return {w,r,g,cmg,dg,DT,SC};
}
export function run(scen,T=40,o={}){
  const {w,r,g,cmg,dg,DT}=build('light',1.25,o);
  const h0=rigStats(r).height, py0=r.bodies.pelvis.x.y, x0=r.bodies.pelvis.x.x, z0=r.bodies.pelvis.x.z;
  let fell=null,brk=[];
  const N=Math.round(T/DT);
  for(let i=0;i<N;i++){
    const t=i*DT; let d;
    if(scen==='fwd') d=[1,0];
    else if(scen==='strafeL') d=[0,1];
    else if(scen==='rev180') d=[-1,0];
    else if(scen==='diag') d=[-0.707,-0.707];
    else d=[[1,0],[0,1],[-1,0],[0,-1]][Math.floor(t/5)%4];
    g.want.tx=d[0]*dg.strideCap; g.want.tz=d[1]*dg.strideCap;
    if(scen!=='strafeL') g.want.facing=Math.atan2(-d[1],d[0]);
    const st=groundTruthState(r,w); g.update(st,DT); if(cmg){cmg.targetYaw=g.cmd.facing; cmg.update(st,DT);} w.step(DT);
    const up=qrot(r.bodies.torso.q,V(0,1,0)).y, drop=py0-r.bodies.pelvis.x.y;
    if(fell===null&&(up<0.6||drop>0.30*h0)) fell=+t.toFixed(2);
    if(w.breakEvents.length>brk.length) brk=w.breakEvents.map(e=>e.joint||e.weld);
  }
  return {fell,steps:g.stepsTaken,d:[+(r.bodies.pelvis.x.x-x0).toFixed(2),+(r.bodies.pelvis.x.z-z0).toFixed(2)],brk:[...new Set(brk)].slice(0,2)};
}
if(import.meta.url===`file://${process.argv[1]}`){
  for(const s of ['fwd','strafeL','diag','rev180','box'])
    console.log(`  ${s.padEnd(9)}`,JSON.stringify(run(s,40)));
}
