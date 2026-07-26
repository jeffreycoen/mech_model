/* Static file server + session log sink for the MK1 rig.
   POST /log  -> appends a batch of JSONL records to logs/<sessionId>.jsonl
   GET  /logs -> index of recorded sessions
   Run: node logserver.mjs [port]                                   */
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const LOGS = path.join(ROOT, 'logs');
const PORT = +process.argv[2] || 8080;
fs.mkdirSync(LOGS, { recursive: true });

const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.mjs': 'text/javascript',
  '.css': 'text/css', '.json': 'application/json', '.png': 'image/png', '.md': 'text/plain; charset=utf-8' };

const safeId = (s) => String(s || 'unknown').replace(/[^A-Za-z0-9_.-]/g, '_').slice(0, 80);

const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://x');

  if (req.method === 'POST' && url.pathname === '/log') {
    let body = '';
    req.on('data', (c) => {
      body += c;
      if (body.length > 32e6) { req.destroy(); }        // refuse absurd payloads
    });
    req.on('end', () => {
      try {
        const pkt = JSON.parse(body);
        const file = path.join(LOGS, safeId(pkt.session) + '.jsonl');
        const lines = (pkt.records || []).map((r) => JSON.stringify(r)).join('\n');
        if (lines) fs.appendFileSync(file, lines + '\n');
        res.writeHead(204).end();
      } catch (e) {
        res.writeHead(400, { 'content-type': 'text/plain' }).end('bad batch: ' + e.message);
      }
    });
    return;
  }

  if (url.pathname === '/logs') {
    const rows = fs.readdirSync(LOGS).filter((f) => f.endsWith('.jsonl')).map((f) => {
      const st = fs.statSync(path.join(LOGS, f));
      return { file: f, bytes: st.size, modified: st.mtime.toISOString() };
    }).sort((a, b) => b.modified.localeCompare(a.modified));
    res.writeHead(200, { 'content-type': 'application/json' }).end(JSON.stringify(rows, null, 1));
    return;
  }

  let p = decodeURIComponent(url.pathname);
  if (p === '/') p = '/mech-mk1-live-opus-5-1m.html';
  const file = path.join(ROOT, path.normalize(p).replace(/^(\.\.[/\\])+/, ''));
  if (!file.startsWith(ROOT) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) {
    res.writeHead(404, { 'content-type': 'text/plain' }).end('not found');
    return;
  }
  res.writeHead(200, {
    'content-type': MIME[path.extname(file)] || 'application/octet-stream',
    'cache-control': 'no-store',
  }).end(fs.readFileSync(file));
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`serving ${ROOT} on :${PORT}, logs -> ${LOGS}`);
});
