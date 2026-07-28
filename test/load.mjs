/* LOAD TEST.  node test/load.mjs

   Evaluates every <script> block of the built artifact in ONE shared context, in order,
   against a stub DOM -- which is what a browser does.

   WHY: the check this replaces called `new vm.Script(block)` on each block separately and
   reported "all script blocks parse". Each block IS legal alone. Top-level `const` in a
   classic <script> goes into the shared GLOBAL LEXICAL SCOPE, so moving SPECS and
   CONTROLLERS into rig/build.js while leaving them in ui/03-sim.js produced

       SyntaxError: Identifier 'CONTROLLERS' has already been declared

   at load -- a blank page, handed over as verified, because the old check was structurally
   incapable of seeing a cross-block collision. Per-block parsing is not a load test.

   This stops at the first real error and prints it, so it is also a fast triage tool. */
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

/* Default to the artifact build.mjs would produce for the CURRENT source, not a literal name.
   A hardcoded name here load-tested whatever file happened to be on disk under that name, which
   after a version bump is the PREVIOUS build. */
import { artifactName } from '../build.mjs';
const FILE = process.argv[2] || artifactName(readFileSync('src/core/preamble.js', 'utf8'));
const html = readFileSync(FILE, 'utf8');

const blocks = [];
const re = /<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g;
let m;
while ((m = re.exec(html))) if (m[1].trim()) blocks.push(m[1]);

/* Stub DOM. Deliberately permissive -- the point is to reach the end of every top-level
   statement, not to emulate a browser. Anything the artifact calls at load returns a chainable
   nothing; anything it reads is an object. */
const el = () => new Proxy(function () {}, {
  get: (t, k) => (k === 'style' || k === 'classList' || k === 'dataset') ? el()
    : (k === 'textContent' || k === 'innerHTML' || k === 'value') ? ''
    : (k === Symbol.toPrimitive) ? () => '' : el(),
  set: () => true,
  apply: () => el(),
  construct: () => el(),
});
const ctx = vm.createContext({
  console, Math, JSON, Date, isNaN, parseFloat, parseInt, Number, String, Array, Object,
  Map, Set, Promise, Error, Float32Array, Uint8Array, Symbol,
  performance: { now: () => 0 },
  document: new Proxy({}, { get: () => el(), set: () => true }),
  window: new Proxy({}, { get: () => el(), set: () => true }),
  navigator: { userAgent: 'load-test', sendBeacon: () => true },
  location: { href: '', protocol: 'http:' },
  THREE: new Proxy({}, { get: () => el() }),
  requestAnimationFrame: () => 0, setTimeout: () => 0, setInterval: () => 0,
  addEventListener: () => {}, fetch: () => Promise.resolve({ ok: true }),
  localStorage: { getItem: () => null, setItem: () => {} },
  devicePixelRatio: 1, innerWidth: 440, innerHeight: 751, screen: { width: 440, height: 751 },
  Blob: function () {}, URL: { createObjectURL: () => '' }, atob: () => '', btoa: () => '',
});
ctx.globalThis = ctx; ctx.self = ctx;

let failed = 0;
for (let i = 0; i < blocks.length; i++) {
  try {
    new vm.Script(blocks[i], { filename: `block${i + 1}` }).runInContext(ctx);
    console.log(`  ok    block ${i + 1}  (${blocks[i].length} chars)`);
  } catch (e) {
    /* A SyntaxError or ReferenceError is a real defect: the page would not load. Anything
       thrown from inside a function the artifact CALLS at load may just be the stub DOM
       being too thin, so it is reported but not failed. Declaration collisions and
       undefined identifiers are exactly the class this exists to catch, and they surface
       as SyntaxError / ReferenceError before any DOM call. */
    /* NOT `e instanceof SyntaxError`: an error thrown inside a vm context is built from
       THAT realm's constructors, so cross-realm instanceof is always false and every real
       defect was being downgraded to a warning. Match on the name. */
    const fatal = e.name === 'SyntaxError' || e.name === 'ReferenceError'
               || e.name === 'TypeError' && /is not a function|of undefined/.test(e.message);
    console.log(`  ${fatal ? 'FAIL' : 'warn'}  block ${i + 1}  ${e.constructor.name}: ${e.message}`);
    if (fatal) failed++;
  }
}
/* Declarations the artifact depends on having reached the global scope. If a block threw
   partway through, these are how you find out what was lost. */
const NEED = ['BUILD', 'BUILD_TAG', 'BUILD_TITLE', 'World', 'assembleMech', 'buildRig',
              'GaitController', 'CrawlController', 'PRESETS', 'deriveGait', 'groundTruthState'];
/* `const`/`let`/`class` at the top level of a classic script land in the script's LEXICAL
   scope, not on globalThis -- so ctx[name] cannot see them. Probe by evaluating a
   reference in the same context, which is exactly how the next block would reach them. */
const missing = NEED.filter(n => {
  try { return new vm.Script(`typeof ${n}`).runInContext(ctx) === 'undefined'; }
  catch { return true; }
});
if (missing.length) { console.log(`  FAIL  missing globals: ${missing.join(', ')}`); failed++; }
else console.log(`  ok    all ${NEED.length} expected globals present`);

console.log(failed ? `\nLOAD FAILED (${failed})\n` : `\nartifact loads clean\n`);
process.exit(failed ? 1 : 0);
