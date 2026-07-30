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
       buildRig,DISPLAY_H,groundTruthState,swingLift}=S;
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
  // stepPeriod, not tSS+tDS: the quad's crawl runs its own 2.75x tempo (MK1.40.0) and
  // turnRate derives from the period a step ACTUALLY takes on that rig.
  const d=b.dg, T=d.stepPeriod, got=d.turnRate*T;
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

/* ---- I9. The orifice damper obeys the same explicit-stability rule ----------------------
   The kv term (-kv*wRel*|wRel|, added MK1.27.0) is explicit exactly like kd: wRel is frozen
   across the substep's iterations, so the effective damper slope is kd + 2*kv*|w| and the I8
   rule is rate-dependent. assemble.js caps kd at ONE site; kv is applied by applyPreset AFTER
   assembly and nothing capped it -- it was stable only because legInertia*3 landed in the same
   preset (review finding, MK1.33.0). The worst PRE-CLAMP rate is where the kv torque alone
   reaches tauMax, wTau = sqrt(tauMax/kv); past it the tauMax clamp bounds the impulse and the
   failure mode is bang-bang chatter, not divergence. So assert the slope at wTau:
   (kd + 2*sqrt(tauMax*kv)) * h / I < 2. Scale-free: kd, sqrt(tauMax*kv) both go as s^4*sqrt(s). */
console.log('\nI9  (kd + 2*sqrt(tauMax*kv))*h/I < 2 where kv > 0 (orifice damper stability)');
for(const b of RIGS){
  const h=(1/60)*Math.sqrt(b.sc)/10;
  let worst=0, wname='', any=false;
  for(const [n,j] of Object.entries(b.r.joints)){
    if(!j.kv||!j.b||!j.b.I) continue;
    any=true;
    const a=j.axisA, M=j.b.I;
    const Ia=[M[0]*a.x+M[1]*a.y+M[2]*a.z, M[3]*a.x+M[4]*a.y+M[5]*a.z, M[6]*a.x+M[7]*a.y+M[8]*a.z];
    const I=Math.abs(a.x*Ia[0]+a.y*Ia[1]+a.z*Ia[2])||1e-12;
    const s=(j.kd+2*Math.sqrt(j.tauMax*j.kv))*h/I;
    if(s>worst){ worst=s; wname=n; }
  }
  if(any) P(`${b.p.label} worst joint (${wname})`, worst<2, `slope*h/I ${worst.toFixed(2)}`, '< 2');
  else console.log(`  --    ${b.p.label} has no kv joints`);
}

/* ---- I10. Catch-step geometry: can one step reach where STAND decides it must go? ------
   CATCH STEP FROM STAND (gait.js, the block by that name) fires when the lateral capture
   error exceeds copLimitZ + halfStance -- past that, no CoP command can arrest the fall and
   stepping is the only recovery. But the step it takes is bounded by the SAME splay clamp
   that caps every lateral placement (the "Splay bound" block in gait.js): outboard of
   nominal stance by at most splayMax*2*halfStance, i.e. (splayMax-1)*2*halfStance beyond
   where the feet already are. If the trigger distance exceeds what one step can deliver,
   the machine can decide it must move further than the recovery mechanism can move it --
   a real, measured shortfall (see the ratio below), not a regression this gate is meant to
   hold the line on, so it WARNs rather than FAILs.
   halfStance is set by GaitController.init(), which is pure geometry off the assembled
   feet -- no stepping -- so it is built fresh here rather than reused off RIGS to avoid
   mutating the shared controller instance's standing/plant state for whatever runs after. */
console.log('\nI10 catch-step: trigger distance vs one step\'s splay-clamped reach');
for(const key of ['light','atst']){
  const B=buildRig(key,{height:H});
  const st=groundTruthState(B.rig,B.world);
  B.gait.init(st);
  const g=B.gait;
  const trigger=g.balance.k.copLimitZ+g.halfStance;
  const deliverable=(g.k.splayMax-1)*2*g.halfStance;
  W(`${PRESETS[key].label} one-step reach >= catch trigger`, deliverable>=trigger,
    `${(deliverable/trigger).toFixed(3)}x`,
    `(trigger ${(trigger*1e3).toFixed(1)} mm, one step ${(deliverable*1e3).toFixed(1)} mm)`);
}

/* ---- I11. Swing-foot touchdown velocity ------------------------------------------------
   swingLift(s,h) = sin(pi*s)^2 * h, ONE site shared by gait.js and crawl.js (see the note at
   its definition in chassis.js) -- asserted on the function itself so a copy anywhere else
   could never silently diverge from what this checks. sin^2 replaced a bare sin because
   sin's slope is steepest exactly at s=1, so the foot arrived at its MAXIMUM downward speed
   at touchdown; substep contact telemetry read 1.5 W p95 / 3.2 W peak on landings before the
   fix. Central finite difference at s=0.999 against the mid-swing slope at s=0.25 as the
   reference peak. */
console.log('\nI11 swingLift touchdown slope (central difference)');
{
  const h=0.14, eps=1e-4;
  const slope=(s)=>(swingLift(s+eps,h)-swingLift(s-eps,h))/(2*eps);
  const mid=slope(0.25), end=slope(0.999);
  P('swingLift touchdown slope < 2% of mid-swing peak', Math.abs(end)<0.02*Math.abs(mid),
    `${(Math.abs(end/mid)*100).toFixed(2)}%`, '< 2%');
  P('swingLift(0,h) == 0', swingLift(0,h)===0, `${swingLift(0,h)}`, '0');
  P('swingLift(1,h) ~= 0', Math.abs(swingLift(1,h))<1e-9, `${swingLift(1,h).toExponential(2)}`, '~0');
}

/* ---- I12. Ground compliance sink is in range and Froude-invariant ---------------------
   world.contacts.compliance is derived in rig/build.js as 0.01*(thigh+shin)/(mass*g), sized
   so one bodyweight of load sinks 1% of leg length. Checked directly against the sink it
   produces -- mass*g*compliance -- which must land between 0.5% and 2% of leg length, and
   must be the SAME fraction at any display height: compliance goes as s/s^3 = s^-2, exactly
   cancelling mass*g's s^3, so sink/L is scale-free by construction. This asserts that
   construction against arithmetic instead of leaving it as a comment. */
console.log('\nI12 ground compliance sink: 0.5-2% of leg length, Froude-invariant across height');
for(const key of ['light','atst','atat']){
  const label=PRESETS[key].label, at={};
  for(const hh of [H,1.20]){
    const B=buildRig(key,{height:hh});
    let mass=0; for(const bd of Object.values(B.rig.bodies)) mass+=bd.mass;
    const L=B.rig.leg.thigh+B.rig.leg.shin;
    at[hh]={frac:(mass*G*B.world.contacts.compliance)/L, damp:B.world.contacts.damp};
  }
  const f30=at[H].frac, f120=at[1.20].frac;
  P(`${label} sink/L in range at ${H} m`, f30>0.005&&f30<0.02, `${(f30*100).toFixed(3)}%`, '0.5-2%');
  P(`${label} sink/L Froude-invariant (${H} vs 1.20 m)`,
    Math.abs(f30-f120)/f30<1e-6, `${(Math.abs(f30-f120)/Math.max(f30,1e-300)).toExponential(1)}`, '< 1e-6 rel');
  P(`${label} contacts.damp > 0`, at[H].damp>0 && at[1.20].damp>0, `${at[H].damp}`, '> 0');
}

/* ---- I13. Mount torsion vs tauMax on the single-leg-sizing joints (Light Frame) --------
   mech.js's design note claims tauMax/lim.torsion = 0.73 survives the single-leg-sizing
   change, stated against the RAW MECH_SPEC table (hipYoke 140e3/190e3, thigh & shin
   190e3/260e3 -- hipYaw itself is 120e3/220e3 = 0.545 even there, already off-note).
   That note predates two multipliers that land on the LIGHT preset specifically and touch
   only one side of the ratio: presets.js applies torque:0.5 to tauMax and envelope:0.7 to
   lim.torsion (factor 0.5/0.7 on the ratio), then rig/build.js's FORGIVE=4 multiplies every
   lim by 4 AFTER presets (factor 1/4) -- neither is mentioned by the 0.73 note. Measured
   here on the built rig exactly as driven, not forced to agree with the note. */
console.log('\nI13 tauMax/lim.torsion on the four proximal leg joints (Light Frame; design note: 0.73)');
{
  const b=RIGS.find(x=>x.key==='light');
  for(const base of ['hipYaw','hipYoke','thigh','shin']){
    for(const side of ['L','R']){
      const j=b.r.joints[base+side];
      if(!j) continue;
      const ratio=j.tauMax/j.lim.torsion;
      W(`${b.p.label} ${base}${side}`, Math.abs(ratio-0.73)/0.73<0.05,
        `${ratio.toFixed(3)}`, 'want ~0.73 (+/-5%)');
    }
  }
}

/* ---- I14. Arm tuned-mass-damper tuning (Light Frame) ----------------------------------
   NOT a call into deriveArmTMD. The first version was: it compared buildRig's applied
   gains against the same function's output -- structurally circular, and it printed
   0.0e+0 while an indexing bug inside that function was zeroing every arm segment's own
   inertia (b.I[2] read as if the flat 3x3 were a 3-vector; it is Ixz, identically 0 for
   a box). The review caught the bug; this gate had certified it.
   So this is a SECOND implementation, written independently on purpose: box Izz straight
   from m/12*(dx^2+dy^2), parallel axis from the assembled anchor geometry, Den Hartog
   mu/wT/zeta from masses and dg.omega, kd at the achieved frequency, kv from the I9
   half-cap form build.js uses. A bug in either implementation now fails the match. */
console.log('\nI14 arm TMD tuning vs independent re-derivation (Light Frame)');
{
  const b=RIGS.find(x=>x.key==='light');
  if(!b.r.joints.upperArmL) console.log('  --    no arms on this rig');
  else{
    let mach=0; for(const bd of Object.values(b.r.bodies)) mach+=bd.mass;
    const armM=['upperArmL','foreArmL','upperArmR','foreArmR']
      .reduce((a,n)=>a+b.r.bodies[n].mass,0);
    /* omega recomputed HERE from a manual COM sum -- NOT dg.omega, which deriveGait
       froze before fitCMG added the flywheel; the derive functions read the live rig and
       so must this check, independently (review finding, MK1.38.0). */
    let my=0; for(const bd of Object.values(b.r.bodies)) my+=bd.mass*bd.x.y;
    const om=Math.sqrt(G/(my/mach));
    const mu=armM/(mach-armM), wT=om/(1+mu),
          zeta=Math.sqrt(3*mu/(8*Math.pow(1+mu,3)));
    P('mu in (0,1)', mu>0&&mu<1, mu.toFixed(3));
    P('wT below body sway omega', wT<om,
      `${wT.toFixed(2)} vs ${om.toFixed(2)} rad/s`);
    P('zeta in (0,1)', zeta>0&&zeta<1, zeta.toFixed(3));
    const izz=bd=>bd.mass/12*(bd.dim.x*bd.dim.x+bd.dim.y*bd.dim.y);
    const h=(1/60)*Math.sqrt(b.sc)/10;
    let worst=0, wn='', wAsh=0, wAel=0;
    for(const side of ['L','R']){
      const up=b.r.bodies['upperArm'+side], fo=b.r.bodies['foreArm'+side];
      const sj=b.r.joints['upperArm'+side], ej=b.r.joints['foreArm'+side];
      const aS=up.toWorld(sj.rb), aE=fo.toWorld(ej.rb);
      const dUp=Math.hypot(up.x.x-aS.x,up.x.y-aS.y);
      const dFS=Math.hypot(fo.x.x-aS.x,fo.x.y-aS.y);
      const dFE=Math.hypot(fo.x.x-aE.x,fo.x.y-aE.y);
      const Ish=izz(up)+up.mass*dUp*dUp+izz(fo)+fo.mass*dFS*dFS;
      const Iel=izz(fo)+fo.mass*dFE*dFE;
      const lcS=(up.mass*dUp+fo.mass*dFS)/(up.mass+fo.mass);
      const want={};
      {
        const kp=Math.max(0,wT*wT*Ish-(up.mass+fo.mass)*G*lcS);
        const wA=Math.sqrt((kp+(up.mass+fo.mass)*G*lcS)/Ish);
        want['upperArm'+side]={kp,kd:2*zeta*wA*Ish,I:izz(up)}; wAsh=wA;
      }
      {
        const kp=Math.max(0,wT*wT*Iel-fo.mass*G*dFE);
        const wA=Math.sqrt((kp+fo.mass*G*dFE)/Iel);
        want['foreArm'+side]={kp,kd:2*zeta*wA*Iel,I:izz(fo)}; wAel=wA;
      }
      for(const [n,w2] of Object.entries(want)){
        const j=b.r.joints[n];
        for(const k of ['kp','kd']){
          const d=Math.abs(j[k]-w2[k])/Math.max(1e-12,Math.max(Math.abs(w2[k]),1e-12));
          if(d>worst&&(w2[k]>1e-12||j[k]>1e-12)){ worst=d; wn=`${n}.${k}`; }
        }
        // kv: the I9 half-cap form build.js applies, recomputed here from box Izz
        const room=Math.max(0,0.9*(2*w2.I/h-j.kd)/2);
        const kvWant=0.5*room*room/j.tauMax;
        const dv=Math.abs(j.kv-kvWant)/Math.max(1e-12,kvWant);
        if(dv>worst){ worst=dv; wn=`${n}.kv`; }
      }
    }
    console.log(`        achieved wA/wT: shoulder ${(wAsh/wT).toFixed(2)}x  elbow ${(wAel/wT).toFixed(2)}x   (1.00 = exact Den Hartog; gravity floor prevents lower)`);
    P(`applied matches independent derivation (worst ${wn})`, worst<1e-6,
      worst.toExponential(1), '< 1e-6 rel');
  }
}

/* ---- I15. Gimbal TMD tuning vs independent re-derivation (Scout) -----------------------
   Same discipline as I14 and for the same reason: a check that shares its arithmetic with
   the apply path certifies its own bugs. Second implementation on purpose -- box inertia
   straight from dims (Ixx = m/12*(dy^2+dz^2) for the roll axis, Izz = m/12*(dx^2+dy^2)
   for pitch), parallel axis and gravity lever from the assembled anchors, inverted-bob
   sign (kp = wT^2*I + m*g*lc, gravity DESTABILISES so it adds spring), kv from the I9
   half-cap form. Also asserts the achieved frequency IS the Den Hartog target on both
   axes -- the whole point of the inverted layout. */
console.log('\nI15 gimbal TMD tuning vs independent re-derivation (Scout)');
{
  const b=RIGS.find(x=>x.key==='atst');
  if(!b.r.joints.tmdRing) console.log('  --    no gimbal TMD on this rig');
  else{
    const ring=b.r.bodies.tmdRing, bob=b.r.bodies.tmdBob;
    const rj=b.r.joints.tmdRing, bj=b.r.joints.tmdBob;
    let mach=0; for(const bd of Object.values(b.r.bodies)) mach+=bd.mass;
    const tm=ring.mass+bob.mass, mu=tm/(mach-tm);
    // Independent omega, same reasoning as I14: manual COM sum on the live rig.
    let my=0; for(const bd of Object.values(b.r.bodies)) my+=bd.mass*bd.x.y;
    const om=Math.sqrt(G/(my/mach));
    const wT=om/(1+mu), zeta=Math.sqrt(3*mu/(8*Math.pow(1+mu,3)));
    P('mu in (0,1)', mu>0&&mu<1, mu.toFixed(3));
    P('zeta in (0,1)', zeta>0&&zeta<1, zeta.toFixed(3));
    const h=(1/60)*Math.sqrt(b.sc)/10;
    const ixx=bd=>bd.mass/12*(bd.dim.y*bd.dim.y+bd.dim.z*bd.dim.z);
    const izz=bd=>bd.mass/12*(bd.dim.x*bd.dim.x+bd.dim.y*bd.dim.y);
    let worst=0, wn='';
    const chk=(n,j,I,lc,m,Iown)=>{
      const kp=wT*wT*I+m*G*lc, kd=2*zeta*wT*I;
      for(const [k,w2] of [['kp',kp],['kd',kd]]){
        const d=Math.abs(j[k]-w2)/Math.max(1e-12,w2);
        if(d>worst){ worst=d; wn=`${n}.${k}`; }
      }
      const room=Math.max(0,0.9*(2*Iown/h-j.kd)/2);
      const kvW=0.5*room*room/j.tauMax;
      const dv=Math.abs(j.kv-kvW)/Math.max(1e-12,kvW);
      if(dv>worst){ worst=dv; wn=`${n}.kv`; }
      /* The first version also asserted wA == wT here, computed from the SAME I/lc/m the
         kp check three lines up already used -- algebraically identical to that check,
         i.e. an assertion that could never fail independently (review finding). Removed.
         What IS independently checkable: the fixed spec tauMax must cover the derived
         spring at the end stop with margin, or the absorber clips exactly when it works
         hardest. */
      const range=Math.abs(b.r.table[n].range[1]);
      P(`${n} tauMax covers spring at end stop x2`, j.tauMax>2*j.kp*range,
        `${(j.tauMax/(j.kp*range)).toFixed(1)}x`, '> 2x');
      return Math.sqrt((j.kp-m*G*lc)/I);
    };
    const aR=ring.toWorld(rj.rb);
    const dR=ring.x.y-aR.y, dB=bob.x.y-aR.y;
    const Iroll=ixx(ring)+ring.mass*dR*dR+ixx(bob)+bob.mass*dB*dB;
    const lcR=(ring.mass*dR+bob.mass*dB)/tm;
    const wA1=chk('tmdRing',rj,Iroll,lcR,tm,ixx(ring));
    const aB=bob.toWorld(bj.rb);
    const dBE=bob.x.y-aB.y;
    const Ipitch=izz(bob)+bob.mass*dBE*dBE;
    const wA2=chk('tmdBob',bj,Ipitch,dBE,bob.mass,izz(bob));
    console.log(`        achieved w/wT: roll ${(wA1/wT).toFixed(4)}  pitch ${(wA2/wT).toFixed(4)}   (1.0000 = exact Den Hartog, both axes)`);
    P(`applied matches independent derivation (worst ${wn})`, worst<1e-6,
      worst.toExponential(1), '< 1e-6 rel');
  }
}

console.log(`\n${pass} passed, ${fail} failed, ${warn} warnings\n`);
process.exit(fail?1:0);
