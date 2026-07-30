/* SERVED-BUILD CHECK -- confirms the bytes a browser would fetch right now are the bytes
   this source tree would build. Not a simulation and not a physics check: one HTTP GET
   against whatever `node logserver.mjs` is serving, compared to BUILD/BUILD_MODEL read out
   of src/core/preamble.js.

   Rule 6 in this project's CLAUDE.md: before telling Jeff to drive, confirm the served
   bytes are the changed ones. This is that check, runnable instead of eyeballed -- the
   failure it exists to catch is "he drove a full session on the pre-instrumentation build"
   (recorded 2026-07-27), which a version string nobody checked would have shown.

     node test/served.mjs [host:port]

   Three outcomes, three exit codes:
     0  fetched, and the body contains this source tree's BUILD string           -- match
     1  fetched, but the body does NOT contain it                               -- stale serve
     2  could not connect at all (server down, wrong port, timeout)             -- unreachable */
import {readFileSync} from 'node:fs';
import http from 'node:http';

const HOST = process.argv[2] || 'localhost:8080';
const TIMEOUT_MS = 3000;

const pre = readFileSync('src/core/preamble.js', 'utf8');
const g = (re) => (pre.match(re) || [, ''])[1];
const BUILD = g(/const BUILD\s*=\s*'([^']+)'/);
const BUILD_MODEL = g(/const BUILD_MODEL\s*=\s*'([^']+)'/);
if (!BUILD || !BUILD_MODEL) {
  console.log(`  FAIL  cannot read BUILD/BUILD_MODEL out of src/core/preamble.js`);
  process.exit(1);
}
const FILE = `mech-${BUILD}-${BUILD_MODEL}.html`;
const url = `http://${HOST}/${FILE}`;

console.log(`\nSERVED BUILD CHECK -- ${url}\n`);
console.log(`  source tree BUILD    ${BUILD}`);
console.log(`  source tree MODEL    ${BUILD_MODEL}`);

const req = http.get(url, { timeout: TIMEOUT_MS }, (res) => {
  let body = '';
  res.on('data', (c) => { body += c; });
  res.on('end', () => {
    if (res.statusCode !== 200) {
      console.log(`  warn  HTTP ${res.statusCode} for ${FILE} -- treating as stale serve`);
      console.log(`\nSTALE SERVE -- server reachable, but not serving ${FILE} as built\n`);
      process.exit(1);
    }
    if (body.includes(BUILD)) {
      console.log(`  ok    served body contains BUILD ${BUILD}`);
      console.log(`\nMATCH -- served bytes are ${FILE}\n`);
      process.exit(0);
    } else {
      console.log(`  FAIL  served body does not contain BUILD ${BUILD}`);
      console.log(`\nSTALE SERVE -- ${HOST} is answering, but not with this build. Restart the server.\n`);
      process.exit(1);
    }
  });
});
req.on('timeout', () => {
  req.destroy();
  console.log(`  FAIL  no response from ${HOST} within ${TIMEOUT_MS} ms`);
  console.log(`\nSERVER UNREACHABLE -- ${HOST}\n`);
  process.exit(2);
});
req.on('error', (e) => {
  console.log(`  FAIL  ${e.code || e.message}`);
  console.log(`\nSERVER UNREACHABLE -- ${HOST}\n`);
  process.exit(2);
});
