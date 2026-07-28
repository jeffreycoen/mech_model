/* Build-time invariants. Pure arithmetic on the assembled rigs -- no stepping, no gait,
   nothing that could be called a simulation. Run it after any edit to src/rig or src/control:

     node test/invariants.mjs

   WHY THIS FILE EXISTS. Every bug this project has lost days to was a number that was
   wrong the moment it was typed and stayed wrong because nothing ever checked it. Prose
   in a status document is not a check. The list, all found the expensive way:

     - `yawPerStep` derived at two sites; battery passed, artifact did not.
     - `kd = 0.06*kp` written at five sites and stated as a design rule in a sixth.
     - The quadruped's torque table doubled in code while the UI note said otherwise.
     - The Scout's leg beams implying 3.9 mm walls on a 45 mm section -- never noticed,
       because nobody ever divided a mass by a volume.
     - Masses quoted at NATIVE scale in every report while the machine being driven is
       scaled to 0.30 m. Same number, 4000x apart.

   RULE: everything here prints AS DRIVEN unless a column says `native`. The rigs are
   normalised to a display height, so a native mass is not a fact about the machine anyone
   drives, and reporting one is how the last item on that list happened repeatedly. */
import {buildSim} from './harness.mjs';
const S=await import(buildSim());
const {World,assembleMech,groundRig,rigStats,setGravity,V,PRESETS,applyPreset,
       buildRig,DISPLAY_H}=S;
const SPECS={atst:S.ATST_SPEC,atat:S.ATAT_SPEC};

/* The one display height that ships. It is a SIM fact -- it sets the scale, hence SIM_DT
   and every derived gait constant -- so it lives in rig/build.js and comes off the manifest
   with everything else. This used to regex-scrape `const SIZES=[{h:...` out of
   src/ui/03-sim.js at runtime with 0.30 hardcoded AGAIN in the match fallback, so
   reformatting that one line in the UI would have silently reverted this gate to the old
   number with no error. A file exists to kill two-site constants; it must not create one. */
const H=DISPLAY_H;

const G=9.81, STEEL=7850;
let pass=0, fail=0, warn=0;
const P=(n,ok,got,want)=>{ if(ok){pass++;} else {fail++;}
  console.log(`  ${ok?'ok  ':'FAIL'}  ${n.padEnd(46)} ${got}${want?'   want '+want:''}`); };
const W=(n,ok,got,note)=>{ if(!ok){warn++;}
  console.log(`  ${ok?'ok  ':'warn'}  ${n.padEnd(46)} ${got}${note?'   '+note:''}`); };

/* Rigs come from buildRig() -- the SAME function the artifact calls. This file used to
   assemble its own, which meant it silently missed anything buildRig passes: the per-rig
   `gamma` override reached the artifact and not the gate, so I2 reported a Scout at 6.000
   while the shipped machine ran 15. Duplicate construction is the bug this whole gate
   exists to catch; having it here too was not defensible. */
function build(key){
  const B=buildRig(key,{height:H});
  // Native figures come from an unscaled probe, for the as-driven/native table only.
  const p=PRESETS[key]; setGravity(G);
  const pw=new World({substeps:2,iterations:1,gravity:V(0,-G,0)});
  const pr=assembleMech(pw,{spec:SPECS[p.spec],footWidth:p.footWidth,hipOffset:p.hipOffset});
  groundRig(pr); applyPreset(pr,p);
  return {p, key, r:B.rig, sc:B.SC, dg:B.dg, gait:B.gait,
          nat:{height:rigStats(pr).height,
               mass:Object.values(pr.bodies).reduce((a,b)=>a+b.mass,0)},
          mass:Object.values(B.rig.bodies).reduce((a,b)=>a+b.mass,0)};
}
const RIGS=['light','atst','atat'].map(build);

console.log(`\nINVARIANTS -- all figures AS DRIVEN at ${H} m unless marked native\n`);
console.log('as driven:  '+RIGS.map(b=>`${b.p.label} ${(b.mass*1000).toFixed(0)} g`).join('   '));
console.log('native:     '+RIGS.map(b=>`${b.p.label} ${b.nat.mass.toFixed(0)} kg @ ${b.nat.height.toFixed(2)} m`).join('   '));

/* ---- I1. Structural slenderness ------------------------------------------------------
   Mean density over a bounding box is NOT the diagnostic -- a hollow beam is supposed to
   read under water. The diagnostic is section width over implied wall thickness, taking
   the link as a steel shell: phi = rho_mean/rho_steel, t = phi / (2*sum(1/side)).
   a/t above ~150 is a flat panel that would buckle; the Light Frame sits at 60. Scale
   cancels in a ratio, so this is scale-free. */
console.log('\nI1  leg-beam slenderness a/t (steel shell; >150 = too fat for its mass)');
for(const b of RIGS){
  for(const n of Object.keys(b.r.bodies)){
    if(!/^(thigh|shin)/.test(n)) continue;
    if(!/(L|FL)$/.test(n)) continue;                       // one limb is enough
    const bd=b.r.bodies[n], d=bd.dim;
    const vol=d.x*d.y*d.z, phi=(bd.mass/vol)/STEEL;
    const t=phi/(2*(1/d.x+1/d.y+1/d.z)), at=Math.min(d.x,d.z)/t;
    W(`${b.p.label} ${n}`, at<150,
      `a/t ${at.toFixed(0).padStart(4)}  wall ${(t*1000).toFixed(1)} mm on ${(Math.min(d.x,d.z)*1000).toFixed(0)} mm`);
  }
}

/* ---- I2. Servo damping ratio is Froude-invariant --------------------------------------
   gamma = kd/(kp*h) is the group the explicit damping term's stability depends on. h is the
   SUBSTEP, (1/60)*sqrt(scale)/substeps.

   THIS USED TO ASSERT "identical across joints and sizes" AND THAT WAS TOO STRONG. The
   requirement is per SIZE: a joint whose gamma changes with scale is not Froude-invariant and the
   servo behaves differently on the same machine at two heights. Equality across JOINTS was only
   ever a convenience -- one number for the whole table -- and it is now deliberately false,
   because a single gamma applied to every joint alike put hipYoke at kd*h/I = 8.57 and ankleYoke
   at 8.26, four times past the explicit damper's divergence bound (see I8). assemble.js caps kd
   per joint against the child's own inertia, which clips gamma on exactly those joints.
   The cap is itself scale-free -- kd goes as s^4*sqrt(s), I as s^5, h as sqrt(s), so kd*h/I is
   invariant -- so per-joint gamma still matches across sizes, which is what this now checks by
   building each rig at two heights and comparing joint by joint. Strictly stronger than the old
   test: the old one could not have caught a scale-dependent gamma on a single joint at all. */
console.log('\nI2  gamma = kd/(kp*h) per joint, identical at two sizes');
const gammaAt=(key,hh)=>{
  const B=buildRig(key,{height:hh}), h=(1/60)*Math.sqrt(B.SC)/10, out={};
  for(const [n,j] of Object.entries(B.rig.joints)) if(j.kp>0) out[n]=j.kd/(j.kp*h);
  return out;
};
for(const b of RIGS){
  const a=gammaAt(b.key,H), c=gammaAt(b.key,H*4);
  let worst=0, wn='';
  for(const n of Object.keys(a)){
    const d=Math.abs(a[n]-c[n])/Math.max(1e-12,a[n]);
    if(d>worst){ worst=d; wn=n; }
  }
  const groups=[...new Set(Object.values(a).map(v=>v.toFixed(3)))].sort((x,y)=>y-x);
  P(`${b.p.label} gamma scale-invariant (worst ${wn})`, worst<1e-6,
    `${(worst*100).toExponential(1)}% drift`, '< 1e-4 %');
  console.log(`        gamma groups at ${H} m: ${groups.join(' / ')}   (declared ${b.p.gamma??6}, capped where I8 required)`);
}

/* ---- I3. One rule, one site ----------------------------------------------------------
   turnRate IS yawPerStep/(tSS+tDS). Both were derived independently and agreed only
   because both happened to read 20 deg. */
console.log('\nI3  turnRate * step period == yawPerStep');
for(const b of RIGS){
  const d=b.dg, T=d.tSS+d.tDS, got=d.turnRate*T;
  P(`${b.p.label}`, Math.abs(got-d.yawPerStep)/d.yawPerStep<0.02,
    `${(got*57.2958).toFixed(2)} deg`, `${(d.yawPerStep*57.2958).toFixed(2)} deg`);
}

/* ---- I4. The single-leg rule ---------------------------------------------------------
   STANDING DESIGN RULE: either leg must hold the whole body at any time, the way a person
   stands on one leg. This is a requirement, not a derivation to be optimised away by
   whatever the current gait demands. The proximal leg joints therefore carry 2x the
   shared-support table. Checked as tauMax against the fraction of m*g*L it is meant to be. */
console.log('\nI4  proximal leg tauMax >= its single-leg m*g*L fraction');
const FRAC={hipYaw:0.50,hipYoke:0.58,thigh:0.78,shin:0.78};
for(const b of RIGS){
  const L=b.r.leg.thigh+b.r.leg.shin, MGL=b.mass*G*L;
  for(const [k,f] of Object.entries(FRAC)){
    const j=Object.entries(b.r.joints).find(([n])=>n.startsWith(k)&&/(L|FL)$/.test(n));
    if(!j) continue;
    const need=f*MGL, have=j[1].tauMax;
    W(`${b.p.label} ${j[0]}`, have>=need*0.98, `${(have/need).toFixed(2)}x`,
      `(${(have*1e3).toFixed(1)} vs ${(need*1e3).toFixed(1)} mN.m)`);
  }
}

/* ---- I5. Rest-pose leg extension -----------------------------------------------------
   legIK clamps hip-to-ankle reach at 0.995 of full leg length. A rest pose near that has
   no vertical compliance: holding a height at the singularity demands enormous joint
   travel per millimetre, and the servo fights itself. Measured at 0.983 the Scout's
   standing slide tripled and its saturated-joint count went 10% -> 29%. */
console.log('\nI5  rest-pose leg extension vs the legIK 0.995 clamp');
for(const b of RIGS){
  const L=b.r.leg.thigh+b.r.leg.shin;
  const ank=b.r.bodies[`foot${b.r.sides[0]}`].toWorld(b.r.ankle);
  const hip=b.r.bodies[`hipYoke${b.r.sides[0]}`].x;
  const ext=Math.hypot(hip.x-ank.x,hip.y-ank.y,hip.z-ank.z)/L;
  /* The ASSEMBLED pose is a spawn fact and is always near the clamp -- that is the geometry
     the rig is built at, not what it rests at. What matters is where the controller will
     hold it, which is what restCeiling() computes. Assert that; report the other. */
  const g=b.gait;
  g.pelvisStart=b.r.bodies.pelvis.x;
  const restExt=ext-(b.r.bodies.pelvis.x.y-g.restCeiling())/L;
  P(`${b.p.label} controller rest target`, restExt<0.96,
    `${restExt.toFixed(3)}`, `< 0.960 (k.restExt ${g.k.restExt})`);
  console.log(`        assembled pose ${ext.toFixed(3)} (spawn geometry, informational)`);
}

/* ---- I6. Controllers construct, and share the rules they are supposed to share -------- */
console.log('\nI6  both controllers inherit the shared command model');
for(const b of RIGS){
  const g=b.gait, C=g.constructor;
  P(`${b.p.label} ${C.name}`,
    ['standHeight','walkHeight','updateWaist','bodyRef','stabiliserYaw']
      .every(m=>typeof g[m]==='function'),
    'stand/walk height, waist, bodyRef, stabiliser');
}

/* ---- I7. Every position command is rate-limited ---------------------------------------
   The recurring failure is a POSITION command with no slew. The waist ring stepped 100 deg at
   37 rad/s and tore both arms and the head off; the hip yaw ring stepped 40.5 deg at 209 rad/s
   and tore ankleYokeR, the torso and the head (driving log s20260727225156). Both are now
   slewed. What is asserted is the property both rings must have: the rate is FINITE (an
   unlimited position command is the bug), and it can still cross the ring's full travel in about
   one step period, so it cannot bind on a legitimate command.
   The bound is 1.2x the step period, not 1.0x: the two rings are deliberately scaled off
   different times -- hipYawRate off the step period, waistRate off a pendulum-scaled second (see
   its note in rig/derive.js) -- and the waist lands 1-15% slower. Asserting 1.0x here would be
   encoding one ring's design choice as a law and calling the other one broken. */
console.log('\nI7  ring slew is finite and crosses full travel within ~1 step period');
for(const b of RIGS){
  const d=b.dg, T=d.tSS+d.tDS;
  for(const [nm,lim,rate] of [['waist',d.waistLimit,d.waistRate],
                              ['hipYaw',d.hipYawLimit,d.hipYawRate]]){
    if(!lim){ W(`${b.p.label} ${nm}`, true, 'no ring on this rig'); continue; }
    const cross=(2*lim)/rate;                 // seconds to traverse the whole range
    P(`${b.p.label} ${nm} full travel / step period`,
      Number.isFinite(rate) && cross>0 && cross<=T*1.2,
      `${cross.toFixed(3)} s`, `<= ${(T*1.2).toFixed(3)} s`);
  }
}

/* ---- I8. The servo damper is stable in the discrete solver -----------------------------
   physics.js freezes `wRel` across all 8 iterations of a substep, so the damping term is an
   EXPLICIT Euler damper on the child link's inertia about the hinge axis, and it needs
   kd*h/I < 2. gamma = 6 was applied to every joint alike and put hipYoke at 8.57 and ankleYoke
   at 8.26 on the shipped Light Frame -- 4x past divergence, which bang-banged the torque ceiling
   and rang every joint on the machine at 9-21 Hz against a 4.09 Hz gait (log s20260727231426).
   Both gamma and wn*h are scale-free, so this holds at every size or at none. */
console.log('\nI8  kd*h/I < 2 on every joint (explicit damper stability)');
for(const b of RIGS){
  const h=(1/60)*Math.sqrt(b.sc)/10;
  let worst=0, wname='';
  for(const [n,j] of Object.entries(b.r.joints)){
    if(!j.kp||!j.b||!j.b.I) continue;
    const a=j.axisA, M=j.b.I;
    const Ia=[M[0]*a.x+M[1]*a.y+M[2]*a.z, M[3]*a.x+M[4]*a.y+M[5]*a.z, M[6]*a.x+M[7]*a.y+M[8]*a.z];
    const I=Math.abs(a.x*Ia[0]+a.y*Ia[1]+a.z*Ia[2])||1e-12;
    const s=j.kd*h/I;
    if(s>worst){ worst=s; wname=n; }
  }
  P(`${b.p.label} worst joint (${wname})`, worst<2, `kd*h/I ${worst.toFixed(2)}`, '< 2');
}

console.log(`\n${pass} passed, ${fail} failed, ${warn} warnings\n`);
process.exit(fail?1:0);
