/* Standard manoeuvre suite.  node test/manoeuvres.mjs [--n 8] [--rigs light,atst,atat]

   THIS IS SIMULATION, and this project's standing rule is that Jeff drives the machine.
   It is here because he asked for it, and it is built to answer the two objections that
   made every previous harness number worthless:

     1. The world is built by rig/build.js -- the SAME function ui/03-sim.js calls. The old
        test/battery.mjs re-implemented construction, omitted `spec`, and returned an MK1
        for every preset. There is no second construction site left to drift.
     2. The plant is chaotic (measured positive Lyapunov exponent; a 1.9e-16 parameter
        change moved a fall from 178 s to 23 s). Single runs are inadmissible, so every
        figure here is a MEDIAN over an ensemble with nanometre jitter, reported with its
        spread. A single trace is not evidence and is never printed as one.

   A POSITIVE CONTROL (kd x3, which must degrade) runs alongside. Without it a null result
   cannot be told from a broken measurement, and "no difference" has been wrong here before.

   Results are written to logs/suite-<BUILD>.json so versions can be diffed. */
import {writeFileSync,mkdirSync} from 'node:fs';
import {buildSim} from './harness.mjs';
const S=await import(buildSim());
const {groundTruthState,buildRig,BUILD,BUILD_TAG,V,qrot,DISPLAY_H}=S;

const arg=(k,d)=>{const i=process.argv.indexOf('--'+k);return i>0?process.argv[i+1]:d;};
const N=+arg('n',8), RIGS=arg('rigs','light,atst,atat').split(',');
/* The shipped display height comes off the manifest (rig/build.js), not a literal here.
   AIM_MIN is still duplicated from ui/03-sim.js because the whole stick->command mapping
   below is duplicated from ui/11-loop.js; that moves into control/chassis.js separately. */
const H=DISPLAY_H, AIM_MIN=0.18;

/* Counter-based stateless PRNG: f(seed,i). A stateful generator would desynchronise the
   moment one variant makes an extra draw, which is exactly when common random numbers are
   worth having. */
const rnd=(seed,i)=>{let h=Math.imul(seed^i,2654435761);h^=h>>>15;h=Math.imul(h,2246822507);
  h^=h>>>13;return ((h>>>0)/4294967296)-0.5;};

/* ---- the manoeuvres -------------------------------------------------------------------
   Each returns {L:[x,y], R:[x,y]} for a time t, in CAMERA frame, exactly as ui/11-loop.js
   reads the sticks. Cardinal directions are camera-relative because that is what the
   driver's thumb means. */
const CARD=[['N',0,-1],['E',1,0],['S',0,1],['W',-1,0]];
const MAN=[];
for(const [name,x,y] of CARD)
  MAN.push({id:'move-'+name, T:5.0, kind:'travel', dir:[x,y],
            f:t=>({L:t<3?[x,y]:[0,0], R:[0,0]})});
/* Turn in place. The aim is a HEADING, so a 360 is a swept heading; the legs follow once
   the waist ring passes waistFollow. rate chosen so the sweep takes 6 s -- slower than the
   ring can slew, so it is the legs being tested and not the turret. */
for(const [id,sgn] of [['turn-L',+1],['turn-R',-1]])
  MAN.push({id, T:8.0, kind:'turn', sgn,
            f:t=>{const a=sgn*2*Math.PI*Math.min(t,6)/6;
                  return {L:[0,0], R:[Math.sin(a), -Math.cos(a)]};}});
/* Turn while travelling: the case FINDINGS 2.1 says stalls the DCM reference. */
for(const [id,sgn] of [['move-turn-L',+1],['move-turn-R',-1]])
  MAN.push({id, T:8.0, kind:'both', sgn, dir:[0,-1],
            f:t=>{const a=sgn*2*Math.PI*Math.min(t,6)/6;
                  return {L:[0,-1], R:[Math.sin(a), -Math.cos(a)]};}});

const wrap=a=>{while(a>Math.PI)a-=2*Math.PI;while(a<-Math.PI)a+=2*Math.PI;return a;};

function run(key, man, seed, mod){
  const B=buildRig(key,{height:H});
  const {world,rig,gait,cmg,SIM_DT,strideCap}=B;
  if(mod) mod(rig);
  // Nanometre jitter, scaled by the rig, applied to every free body's position.
  for(const b of Object.values(rig.bodies)){
    if(b.kinematic) continue;
    const s=1e-9*H;
    b.x=V(b.x.x+rnd(seed,1)*s, b.x.y+rnd(seed,2)*s, b.x.z+rnd(seed,3)*s);
  }
  const az=0.785;                                  // fixed camera, as the artifact spawns
  const svx=-Math.sin(az), svz=-Math.cos(az), srx=Math.cos(az), srz=-Math.sin(az);
  const st0=groundTruthState(rig,world);
  const p0={x:rig.bodies.pelvis.x.x, z:rig.bodies.pelvis.x.z};
  const yaw0=gait.bodyYaw();
  let t=0, fell=null, yawUnwrapped=0, prevYaw=yaw0, slip=[], sat=[], cfmin=[];
  let cmdDist=0;
  while(t<man.T){
    const s=man.f(t);
    let dx=s.L[1]*svx+s.L[0]*srx, dz=s.L[1]*svz+s.L[0]*srz;
    const mag=Math.min(1,Math.hypot(dx,dz));
    if(mag>1e-3){const n=Math.hypot(dx,dz);dx/=n;dz/=n;} else {dx=0;dz=0;}
    gait.want.tx=dx*mag*strideCap; gait.want.tz=dz*mag*strideCap;
    const rx=s.R[0], ry=s.R[1];
    gait.aimHold=Math.hypot(rx,ry)>AIM_MIN;
    if(gait.aimHold){
      const fx=ry*svx+rx*srx, fz=ry*svz+rx*srz;
      gait.aim=Math.atan2(-fz,fx);
    }
    const est=groundTruthState(rig,world);
    gait.update(est,SIM_DT);
    if(cmg){ cmg.targetYaw=gait.stabiliserYaw(); cmg.update(est,SIM_DT); }
    world.step(SIM_DT);
    t+=SIM_DT;
    const y=gait.bodyYaw(); yawUnwrapped+=wrap(y-prevYaw); prevYaw=y;
    cmdDist+=mag*strideCap*SIM_DT/(B.dg.tSS+B.dg.tDS);
    for(const sd of rig.sides){
      const f=rig.bodies['foot'+sd];
      if((f.contactForce||0)>0.02*est.mass*9.81){ slip.push(f.contactSlip||0);
        cfmin.push((f.contactForceMin||0)/(est.mass*9.81)); }
    }
    if(est.joints) sat.push(est.joints.peakSat);
    const up=qrot(rig.bodies.torso.q,V(0,1,0)).y;
    if(up<0.5 && !fell) { fell=t; break; }
  }
  const p1=rig.bodies.pelvis.x;
  const med=a=>{if(!a.length)return NaN;const b=a.slice().sort((x,y)=>x-y);return b[b.length>>1];};
  return {
    dist:Math.hypot(p1.x-p0.x,p1.z-p0.z), cmdDist,
    yaw:yawUnwrapped, fell, steps:gait.stepsTaken||0,
    slip:med(slip)*1000, cfmin:med(cfmin), sat:med(sat),
    broke:world.breakEvents.length,
  };
}

const med=a=>{const b=a.filter(Number.isFinite).sort((x,y)=>x-y);
  return b.length?b[b.length>>1]:NaN;};
/* Spread. At n<4 the quartiles collapse onto one sample and printed a misleading +-0, so
   fall back to the full range -- small, but honest about being small. */
const iqr=a=>{const b=a.filter(Number.isFinite).sort((x,y)=>x-y);
  if(b.length<2) return 0;
  return b.length>3 ? b[Math.floor(b.length*0.75)]-b[Math.floor(b.length*0.25)]
                    : b[b.length-1]-b[0];};

const OUT={build:BUILD_TAG, height:H, n:N, rigs:{}};
console.log(`\nMANOEUVRE SUITE  build ${BUILD_TAG}  ${H} m  n=${N} seeds per cell\n`);
for(const key of RIGS){
  console.log(`--- ${key} ---`);
  console.log('  manoeuvre        travel_mm  of_cmd   yaw_deg   falls   slip_mm/s  cf_min  peakSat');
  OUT.rigs[key]={};
  for(const man of MAN){
    const R=[]; for(let i=0;i<N;i++) R.push(run(key,man,1000+i,null));
    const d=R.map(r=>r.dist*1000), c=R.map(r=>r.cmdDist*1000);
    const yw=R.map(r=>r.yaw*57.2958), falls=R.filter(r=>r.fell).length;
    const row={travel:med(d), travelIQR:iqr(d), cmd:med(c), yaw:med(yw), yawIQR:iqr(yw),
               falls, slip:med(R.map(r=>r.slip)), cfmin:med(R.map(r=>r.cfmin)),
               sat:med(R.map(r=>r.sat)), broke:R.reduce((a,r)=>a+r.broke,0)};
    OUT.rigs[key][man.id]=row;
    const eff=row.cmd>1?(row.travel/row.cmd*100).toFixed(0)+'%':'   -';
    console.log('  '+man.id.padEnd(15),
      (row.travel.toFixed(0)+'±'+row.travelIQR.toFixed(0)).padStart(10),
      eff.padStart(7),
      (row.yaw.toFixed(0)+'±'+row.yawIQR.toFixed(0)).padStart(10),
      String(falls+'/'+N).padStart(7),
      row.slip.toFixed(1).padStart(10), row.cfmin.toFixed(2).padStart(7),
      row.sat.toFixed(2).padStart(8));
  }
}

/* POSITIVE CONTROL. It must measurably degrade or nothing above means anything.
   kd x3 was the obvious choice and it is the WRONG one now: the shipped gamma is 6, so x3
   only reaches 18 -- still below the 36 that shipped before the gain fix, i.e. a control
   that lands inside the known-survivable band and cannot be expected to separate. It duly
   did not. x20 puts gamma at 120, far outside anything this servo has ever run at. */
const CTL_KD=20;
console.log(`\n--- positive control: kd x${CTL_KD} on the Light Frame (must degrade) ---`);
const ctlMan=MAN.find(m=>m.id==='move-N');
const base=[],ctl=[];
for(let i=0;i<N;i++){
  base.push(run('light',ctlMan,1000+i,null));
  ctl.push(run('light',ctlMan,1000+i,r=>{for(const j of Object.values(r.joints)) j.kd*=CTL_KD;}));
}
const bs=med(base.map(r=>r.slip)), cs=med(ctl.map(r=>r.slip));
const bd=med(base.map(r=>r.dist))*1000, cd=med(ctl.map(r=>r.dist))*1000;
const bf=base.filter(r=>r.fell).length, cf=ctl.filter(r=>r.fell).length;
console.log(`  slip  ${bs.toFixed(1)} -> ${cs.toFixed(1)} mm/s   travel ${bd.toFixed(0)} -> ${cd.toFixed(0)} mm   falls ${bf}/${N} -> ${cf}/${N}`);
const separated = cs>bs*1.2 || cf>bf || cd<bd*0.8;
console.log('  ' + (separated
  ? 'control SEPARATED -- the suite can detect a change of this size'
  : 'control DID NOT SEPARATE -- treat every number above as unproven'));
OUT.control={kd:CTL_KD, slipBase:bs, slipCtl:cs, fallsBase:bf, fallsCtl:cf, separated};

mkdirSync('logs',{recursive:true});
writeFileSync(`logs/suite-${BUILD}.json`,JSON.stringify(OUT,null,1));
console.log(`\nwritten to logs/suite-${BUILD}.json\n`);
