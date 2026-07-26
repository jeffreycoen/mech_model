#!/usr/bin/env node
/* Build the single-file artifact from src/. The artifact must stay one self-contained
   HTML file to be served and driven; the SOURCE does not have to be, and when it was,
   the same rule kept getting implemented twice 800 lines apart.
   The test harness builds the sim from this same manifest -- see test/harness.mjs --
   so a hand-maintained copy can never drift from the artifact again. */
import {readFileSync,writeFileSync} from 'node:fs';
const man=JSON.parse(readFileSync('src/manifest.json','utf8'));
const cat=list=>list.map(p=>readFileSync(p,'utf8')).join('\n');
const out=[readFileSync('src/shell-head.html','utf8'),
           cat(man.sim),
           readFileSync('src/shell-mid.html','utf8'),
           cat(man.ui),
           readFileSync('src/shell-tail.html','utf8')].join('\n');
const target=process.argv[2]||'mech-mk1-live-opus-5-1m.html';
writeFileSync(target,out);
console.log(`built ${target}  ${out.length} bytes  (${man.sim.length} sim + ${man.ui.length} ui modules)`);
