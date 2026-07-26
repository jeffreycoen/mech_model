/* ---------- session logging ----------
   Everything you do and everything the rig does, streamed to the server as JSONL so a
   session can be replayed and diagnosed afterwards instead of reconstructed from memory.
   Input is captured every frame; body state at 10 Hz; events as they happen. */
const SESSION='s'+new Date().toISOString().replace(/[^0-9]/g,'').slice(0,14)+'-'+
  Math.floor(Math.random()*1e6).toString(36);
let logBuf=[], logOn=true, logFrame=0, lastFlush=0, logDropped=0;
function logRec(r){ if(!logOn) return; if(logBuf.length>20000){ logDropped++; return; } logBuf.push(r); }
function logEvent(kind,data){ logRec(Object.assign({t:'ev',k:kind,st:+simT.toFixed(3)},data||{})); }
async function flushLog(force){
  const now=performance.now();
  if(!force && (now-lastFlush<2000 || !logBuf.length)) return;
  lastFlush=now;
  const batch=logBuf; logBuf=[];
  if(!batch.length) return;
  try{
    await fetch('/log',{method:'POST',headers:{'content-type':'application/json'},
      body:JSON.stringify({session:SESSION,records:batch}),keepalive:force===true});
  }catch(e){ logOn=false; }        // served from file://, or no sink: stop trying
}
logRec({t:'hdr',build:'MK1.4-opus-5-1m',session:SESSION,
  wall:new Date().toISOString(),ua:navigator.userAgent,
  solver:{substeps:10,iterations:8},caps:{MAX_STRIDE,ENVELOPE_STRIDE,
  MAX_TURN,TRAVEL_RATE},screen:[innerWidth,innerHeight]});
addEventListener('pagehide',()=>flushLog(true));
addEventListener('beforeunload',()=>flushLog(true));
