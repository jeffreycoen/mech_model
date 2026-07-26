/* Builds the sim from src/manifest.json -- THE SAME FILES build.mjs puts in the artifact.
   There is no hand-maintained copy any more. Three separate drifts were found in one day
   when there was: the harness passed while the artifact failed, tested code the artifact
   did not have, and kept an old turnRate formula. */
import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
const EXPORTS='World, assembleMech, groundRig, rigStats, GaitController, groundTruthState, '
  +'setGravity, V, qrot, PRESETS, applyPreset, scaleRig, deriveGait, fitCMG, ATST_SPEC';
export function buildSim(){
  const man=JSON.parse(readFileSync('src/manifest.json','utf8'));
  const src=man.sim.map(p=>readFileSync(p,'utf8')).join('\n')+`\nexport { ${EXPORTS} };\n`;
  mkdirSync('.build',{recursive:true});
  writeFileSync('.build/sim.gen.mjs',src);
  return new URL('../.build/sim.gen.mjs',import.meta.url).href;
}
