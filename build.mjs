#!/usr/bin/env node
/* Build the single-file artifact from src/. The artifact must stay one self-contained
   HTML file to be served and driven; the SOURCE does not have to be, and when it was,
   the same rule kept getting implemented twice 800 lines apart.
   The test harness builds the sim from this same manifest -- see test/harness.mjs --
   so a hand-maintained copy can never drift from the artifact again. */
import {readFileSync,writeFileSync} from 'node:fs';
const man=JSON.parse(readFileSync('src/manifest.json','utf8'));
const cat=list=>list.map(p=>readFileSync(p,'utf8')).join('\n');
/* Stamp the real BUILD_TITLE into the static <title> too. 11-loop.js sets document.title from
   BUILD_TITLE at runtime, but the placeholder in shell-head.html carried no version -- so the
   FILE on disk did not name its own build unless you executed it. */
const pre=readFileSync('src/core/preamble.js','utf8');
const titleOf=()=>{const m=pre.match(/const BUILD_TITLE\s*=\s*(.+);/);
  if(!m) return null;
  const b=(pre.match(/const BUILD\s*=\s*'([^']+)'/)||[,''])[1];
  return m[1].replace(/'\s*\+\s*BUILD\s*\+\s*'/g,b).replace(/^'|'$/g,'');};
const head=(t=>t?readFileSync('src/shell-head.html','utf8')
  .replace(/<title>[^<]*<\/title>/,`<title>${t}</title>`)
  :readFileSync('src/shell-head.html','utf8'))(titleOf());
const out=[head,
           cat(man.sim),
           readFileSync('src/shell-mid.html','utf8'),
           cat(man.ui),
           readFileSync('src/shell-tail.html','utf8')].join('\n');
/* THE BUILD VERSION IS IN THE FILENAME, derived from preamble.js -- ONE site, never typed.
   The name was a bare literal here, in logserver.mjs and in test/load.mjs, and it carried the
   model but not the version. So every serve overwrote the previous artifact at the same URL, a
   phone could hold a cached copy indistinguishable from the current one, and a whole driving
   session got diagnosed against the wrong build. A new version is now a new file and a new URL,
   which makes stale bytes impossible rather than merely unlikely. */
export function artifactName(src=readFileSync('src/core/preamble.js','utf8')){
  const g=re=>(src.match(re)||[,''])[1];
  const b=g(/const BUILD\s*=\s*'([^']+)'/), m=g(/const BUILD_MODEL\s*=\s*'([^']+)'/);
  if(!b||!m) throw new Error('build.mjs: cannot read BUILD/BUILD_MODEL from src/core/preamble.js');
  return `mech-${b}-${m}.html`;
}
/* Only WRITE when run as a command. test/load.mjs imports artifactName from here, and without
   this guard that import would silently rebuild the artifact as a side effect of load-testing it. */
if(import.meta.url===`file://${process.argv[1]}`){
  const target=process.argv[2]||artifactName();
  writeFileSync(target,out);
  console.log(`built ${target}  ${out.length} bytes  (${man.sim.length} sim + ${man.ui.length} ui modules)`);
}
