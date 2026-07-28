/* Builds the sim from src/manifest.json -- THE SAME FILES build.mjs puts in the artifact.
   There is no hand-maintained copy any more. Three separate drifts were found in one day
   when there was: the harness passed while the artifact failed, tested code the artifact
   did not have, and kept an old turnRate formula.

   ONE LOADER, ONE EXPORT LIST. This block was written out again in test/invariants.mjs and
   test/manoeuvres.mjs, each with its own list and its own .build/*.gen.mjs output, so adding
   a symbol to the sim meant editing three files -- and the three lists had already drifted:
   this one was missing `buildRig` and `CrawlController` while invariants.mjs had both, and
   manoeuvres.mjs alone had BUILD/BUILD_TAG. That is the two-site rule inside the very thing
   that exists to prevent it. Every entry point calls buildSim(); add a symbol here and every
   suite gets it. */
import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
const EXPORTS='World, assembleMech, groundRig, rigStats, GaitController, CrawlController, '
  +'groundTruthState, setGravity, V, qrot, PRESETS, applyPreset, scaleRig, deriveGait, '
  +'fitCMG, ATST_SPEC, ATAT_SPEC, buildRig, DISPLAY_H, BUILD, BUILD_TAG';
/* One output path, where there used to be three (sim/inv/suite.gen.mjs). Every suite now
   generates byte-identical content from the same manifest and the same list, so two running
   at once write the same bytes -- but writeFileSync is not atomic, so do not run two suites
   concurrently and expect a clean import. Nothing does today. */
export function buildSim(){
  const man=JSON.parse(readFileSync('src/manifest.json','utf8'));
  const src=man.sim.map(p=>readFileSync(p,'utf8')).join('\n')+`\nexport { ${EXPORTS} };\n`;
  mkdirSync('.build',{recursive:true});
  writeFileSync('.build/sim.gen.mjs',src);
  return new URL('../.build/sim.gen.mjs',import.meta.url).href;
}
