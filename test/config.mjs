/* CONFIG SANITY -- pure arithmetic on the assembled rigs, no stepping. Run it after any
   edit to src/rig or src/control:

     node test/config.mjs

   WHY THIS FILE EXISTS. gait.k and gait.balance.k are assembled from several layers
   (CHASSIS_DEFAULTS, per-controller defaults, deriveGait, the preset's own cfg.gait/
   cfg.balance) and every joint's kp/kd/tauMax is derived from a link table entry divided
   by an angle -- any of those can silently produce `undefined` (a key never merged in) or
   `NaN` (a derived quantity built from one) and nothing downstream throws, because the
   controllers and the solver are both happy to propagate either one until the machine goes
   limp on frame one. test/invariants.mjs checks that the numbers are RIGHT; this checks
   that they are numbers at all, at both ends of the size range this project runs at. */
import {buildSim} from './harness.mjs';
const S=await import(buildSim());
const {buildRig,PRESETS,DISPLAY_H}=S;
const H=DISPLAY_H;
const HEIGHTS=[H,1.20];

let fails=0;

/* Recursively walk an object's own enumerable properties. Arrays walk the same way
   (Object.entries on an array yields index -> element), which is what lets this same
   walker cross into gait.k.balance and atat's gait.k.order without a special case for
   either. Leaves that are `undefined` or a NaN number are reported by their full dotted
   path; everything else (strings, booleans, null, functions) is not a numeric config
   value and is silently skipped -- this file checks numbers, not shape. */
function walk(obj, path, out){
  for(const [k,v] of Object.entries(obj)){
    const p=`${path}.${k}`;
    if(v===undefined){ out.push(`${p} = undefined`); continue; }
    if(typeof v==='number'){ if(Number.isNaN(v)) out.push(`${p} = NaN`); continue; }
    if(v!==null && typeof v==='object') walk(v,p,out);
  }
}

/* Joints are NOT walked generically: `j.a`/`j.b` are the parent/child Body objects, which
   point back at the whole rig and would turn a "walk the config" pass into "walk the
   entire assembled machine". The config surface a joint actually carries is this fixed
   list -- kp, kd, kv, tauMax, kpTau (only when present: it is not copied onto the
   assembled joint at all in the common case, see rig/assemble.js kpOf(), so its absence is
   normal and only a NaN if present is a fault) and the four lim.* fields. */
const JOINT_SCALARS=['kp','kd','kv','tauMax'];
function checkJoint(name, j, rigLabel, out){
  for(const f of JOINT_SCALARS){
    const v=j[f];
    if(v===undefined) out.push(`${rigLabel} joints.${name}.${f} = undefined`);
    else if(typeof v==='number' && Number.isNaN(v)) out.push(`${rigLabel} joints.${name}.${f} = NaN`);
  }
  if('kpTau' in j && j.kpTau!==undefined){
    if(typeof j.kpTau==='number' && Number.isNaN(j.kpTau)) out.push(`${rigLabel} joints.${name}.kpTau = NaN`);
  }
  if(!j.lim){ out.push(`${rigLabel} joints.${name}.lim = undefined`); return; }
  for(const f of ['tension','shear','bend','torsion']){
    const v=j.lim[f];
    if(v===undefined) out.push(`${rigLabel} joints.${name}.lim.${f} = undefined`);
    else if(typeof v==='number' && Number.isNaN(v)) out.push(`${rigLabel} joints.${name}.lim.${f} = NaN`);
  }
}

console.log(`\nCONFIG -- undefined/NaN sweep of gait.k, gait.balance.k and every joint, at ${HEIGHTS.join(' and ')} m\n`);

for(const key of ['light','atst','atat']){
  for(const hh of HEIGHTS){
    const rigLabel=`${key}@${hh.toFixed(2)}`;
    const B=buildRig(key,{height:hh});
    const bad=[];
    walk(B.gait.k, `${rigLabel} gait.k`, bad);
    walk(B.gait.balance.k, `${rigLabel} gait.balance.k`, bad);
    for(const [name,j] of Object.entries(B.rig.joints)) checkJoint(name, j, rigLabel, bad);
    if(bad.length){
      for(const b of bad) console.log(`  FAIL  ${b}`);
      fails+=bad.length;
    } else {
      console.log(`  ok    ${rigLabel}  gait.k, gait.balance.k, ${Object.keys(B.rig.joints).length} joints clean`);
    }
  }
}

console.log(fails ? `\n${fails} bad value(s) found\n` : '\nno undefined or NaN config values found\n');
process.exit(fails?1:0);
