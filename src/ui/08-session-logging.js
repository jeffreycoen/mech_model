/* ---------- session logging ----------
   Everything you do and everything the rig does, streamed to the server as JSONL so a
   session can be replayed and diagnosed afterwards instead of reconstructed from memory.
   Input is captured every frame; body state at 20 Hz; events as they happen.

   RATES, because the sampling scheme was the single biggest defect in this instrumentation and
   nothing recorded what it was. As driven at 0.30 m: SIM_DT = 4.118 ms, so the sim runs at
   242.8 Hz with 10 substeps each (2 428 Hz) and 8 solver iterations per substep (19 426 Hz).
   The display runs 60 Hz and ~3.9 sim steps happen per display frame. The old `st` gate of
   `logFrame%6` therefore kept ONE substep in 232 -- 0.43% duty, no anti-aliasing -- and every
   per-joint field except satFrac was that single substep. Two consequences that made whole
   sessions unreadable: the 4.09 Hz gait had 2.44 samples per cycle against a Nyquist of 5 Hz,
   and the 24.5 Hz stance-leg spring folded to 4.50 Hz, 0.40 Hz away from the gait, so a spring
   ring-down and the walk were indistinguishable. Fixed three ways: 20 Hz instead of 10, substep
   EXTREMES rather than snapshots in the joint record, and the burst buffer below for the cases
   where the waveform itself is the question. */
const SESSION='s'+new Date().toISOString().replace(/[^0-9]/g,'').slice(0,14)+'-'+
  Math.floor(Math.random()*1e6).toString(36);
let logBuf=[], logOn=true, logFrame=0, lastFlush=0, logDropped=0;
/* 60 000, was 20 000. The joint record roughly tripled in width and the state rate doubled, and
   the cap was already only ~2.7 s of headroom at the old rate -- a single slow POST over the
   phone's wifi would silently eat the window around exactly the event worth looking at. Drops
   are now reported in every `st` record (`drop`), so a truncated log says so instead of just
   looking calm. */
function logRec(r){ if(!logOn) return; if(logBuf.length>60000){ logDropped++; return; } logBuf.push(r); }

/* ---- BURST BUFFER ----
   A ring of every SIM STEP, dumped only when something interesting happens.

   Why this exists: the continuous channel samples at 20 Hz and the machine's failures live at
   242.8 Hz. Raising the continuous rate to match would be ~490 kB/s and would bind the
   transport, and 99% of it would record a machine walking normally. So carry 1.05 s of full
   sim-step resolution in memory at all times, and write it out only around a fall, a break, a
   foot unloading, an actuator railing for a whole frame, or legIK failing to reach. That gives
   the waveform where the waveform is the question -- a 24.5 Hz ring-down gets ~10 samples per
   cycle here, enough to fit a damping ratio, against 0.8 on the continuous channel. */
const BURST_N=256;                       // 256 sim steps = 1.054 s at 242.8 Hz
let burst=new Array(BURST_N), burstAt=0, burstArmed=0, burstSeq=0, burstWhy='', burstCool=0;
function burstPush(rec){ burst[burstAt]=rec; burstAt=(burstAt+1)%BURST_N; }
/* ROUNDERS, not toFixed. This path runs every sim step -- 242.8 times a second -- and toFixed
   builds a string that the leading `+` then parses back to a number. At 3 joint arrays x 17
   joints that is ~12 000 string allocations a second on a phone, for numbers that are about to
   be serialised anyway. Math.round is arithmetic. */
const r1=x=>Math.round(x*10)/10, r2=x=>Math.round(x*100)/100,
      r3=x=>Math.round(x*1e3)/1e3, r4=x=>Math.round(x*1e4)/1e4, r5=x=>Math.round(x*1e5)/1e5;
/* Arm on a trigger; the dump happens once the ring has filled PAST it, so the record carries
   the lead-in AND the aftermath rather than stopping at the interesting frame. 35% after / 65%
   before = 0.69 s of run-up and 0.37 s of consequence. */
/* HARD triggers bypass the cooldown. The first version gated everything behind it, and that ate
   the burst at EVERY fall in the 2026-07-27 session: coverage was 17-19% and the three falls
   landed 0.56 / 0.77 / 1.76 s past the last window. A fall or a torn mount is the event the
   channel exists for -- it can never be skipped because something noisier fired first. */
const BURST_HARD={fall:1,break:1};
function burstTrigger(why){
  const hard=BURST_HARD[why]===1;
  if(burstArmed){ if(hard) burstWhy=why; return; }   // already capturing: relabel, do not restart
  if(burstCool>0&&!hard) return;
  burstArmed=Math.floor(BURST_N*0.35); burstWhy=why;
}
function burstTick(){
  if(burstCool>0) burstCool--;
  if(!burstArmed) return;
  if(--burstArmed>0) return;
  const out=[];
  for(let i=0;i<BURST_N;i++){ const r=burst[(burstAt+i)%BURST_N]; if(r) out.push(r); }
  logRec({t:'burst',n:++burstSeq,why:burstWhy,hz:r1(1/SIM_DT),rows:out});
  /* COOLDOWN, for the SOFT triggers only. Without one a persistent condition re-arms the instant
     the previous dump lands: `airborne` holds for many sim steps during a hop, so the ring would
     ship ~110 kB every 0.37 s. One ring-length, not four -- four gave 17-19% coverage and missed
     every fall. One gives ~1 s of quiet, so a hop sequence is sampled repeatedly instead of once,
     at a worst case of ~55 kB/s on top of the continuous channel. */
  burstCool=BURST_N;
  flushLog(true);
}
function logEvent(kind,data){ logRec(Object.assign({t:'ev',k:kind,st:+simT.toFixed(3)},data||{})); }
/* RETRY, AND NEVER DIE SILENTLY.
   This used to be `catch(e){ logOn=false; }` with the batch already swapped out of logBuf, so a
   SINGLE failed POST both threw away that batch and disabled logging for the rest of the session,
   with nothing recorded and nothing on screen. It also never checked response.ok, so a 400 from
   the sink counted as success.
   What that cost, 2026-07-27: a session where all three rigs were driven and each respawned
   logged 5.4 s and stopped. The file is a clean prefix -- frames 1..327 contiguous, no gap, no
   pagehide flush -- because the transport had already given up. The driving was unrecoverable
   and the log looked like a short quiet session rather than a broken one.
   Now: failed batches go BACK on the front of the buffer in order and are retried on the next
   flush; it takes 5 consecutive failures to stop; and the reason is reported in the log and in
   the tab title so a dead transport is visible instead of inferred. */
let logFails=0, logOffWhy='';
/* ---- CRASH BACKSTOP ----
   The retry above only helps if there is a LATER flush to retry on. When the page goes away there
   is not: pagehide fires once, and if that single POST does not land, everything since the last
   successful flush is gone with no trace but a clean-prefix file.
   That is what happened twice. MK1.19.0 session s20260727233822 ends mid-session on an `in` record
   at 6.0 s with zero frame gaps, zero drops and zero POST failures -- so the buffer was healthy
   and the last batch simply never went. The falls and breaks that were driven after 6.0 s do not
   exist anywhere, and the truncated file reads as a clean quiet session.
   So: mirror the un-sent tail into localStorage as it accumulates, and replay it on the next page
   load. A hard kill now costs at most one stash interval instead of the whole tail. */
const STASH='mechlog:'+SESSION;
function stash(){
  try{ if(logBuf.length) localStorage.setItem(STASH,JSON.stringify({session:SESSION,records:logBuf}));
       else localStorage.removeItem(STASH); }catch(e){}
}
/* Replay anything a PREVIOUS session left stashed. Keys are collected before removing any, because
   removing shifts localStorage indices under an in-flight loop. */
(function unstash(){
  try{
    const keys=[];
    for(let i=0;i<localStorage.length;i++){ const k=localStorage.key(i);
      if(k&&k.indexOf('mechlog:')===0&&k!==STASH) keys.push(k); }
    for(const k of keys){
      const body=localStorage.getItem(k);
      if(!body){ localStorage.removeItem(k); continue; }
      fetch('/log',{method:'POST',headers:{'content-type':'application/json'},body:body})
        .then(r=>{ if(r.ok) localStorage.removeItem(k); }).catch(()=>{});
    }
  }catch(e){}
})();

/* TEARDOWN FLUSH via sendBeacon, not fetch. A beacon is handed to the browser and survives the
   page dying; a keepalive fetch does not reliably on iOS Safari. Beacons cap near 64 kB, and a
   burst record alone is ~110 kB, so the tail is chunked. Anything that will not go is left in the
   buffer and stashed. */
function beaconAll(){
  if(!logBuf.length) return;
  const recs=logBuf; logBuf=[];
  let i=0;
  while(i<recs.length){
    let j=i,size=0;
    while(j<recs.length&&size<48000){ size+=JSON.stringify(recs[j]).length+1; j++; }
    if(j===i) j=i+1;                                  // one oversized record: try it alone
    const body=JSON.stringify({session:SESSION,records:recs.slice(i,j)});
    let ok=false;
    try{ ok=navigator.sendBeacon('/log',new Blob([body],{type:'application/json'})); }catch(e){}
    if(!ok){ logBuf=recs.slice(i).concat(logBuf); stash(); return; }
    i=j;
  }
  stash();
}

async function flushLog(force){
  const now=performance.now();
  /* 700 ms, was 2000. The batch at risk when a flush fails is one interval's worth, and at ~55 kB/s
     two seconds was 110 kB of driving. */
  if(!force && (now-lastFlush<700 || !logBuf.length)) return;
  lastFlush=now;
  if(!logOn || !logBuf.length) return;
  const batch=logBuf; logBuf=[];
  try{
    const res=await fetch('/log',{method:'POST',headers:{'content-type':'application/json'},
      body:JSON.stringify({session:SESSION,records:batch}),keepalive:force===true});
    if(!res.ok) throw new Error('HTTP '+res.status);
    logFails=0;
    stash();                                // sent: clear the backstop for what just landed
  }catch(e){
    logBuf=batch.concat(logBuf);            // oldest first: ordering survives the retry
    stash();                                // and keep it across a kill, not just across a retry
    if(++logFails>=5){
      logOn=false; logOffWhy=(e&&e.message)||'post failed';
      try{ document.title='[LOG OFF] '+BUILD_TITLE; }catch(_){}
      try{ console.warn('logging stopped after 5 failed POSTs:',logOffWhy); }catch(_){}
    }
  }
}
/* BUILD_TAG, not a literal. A driving log that cannot name the build that produced it
   cannot be compared against another one -- which is exactly what these logs are for. */
logRec({t:'hdr',build:BUILD_TAG,session:SESSION,
  wall:new Date().toISOString(),ua:navigator.userAgent,
  /* Module-level constants only -- this record is written before any rig is assembled, so
     it cannot carry the derived configuration. That arrives in the `build` event emitted
     at the end of buildWorld(), once per preset/size change: stride cap, gait timings,
     turn rate, yawPerStep, CoP limits and every actuator ceiling. */
  solver:{substeps:10,iterations:8},caps:{MAX_STRIDE,ENVELOPE_STRIDE,TRAVEL_RATE},
  /* SELF-DESCRIBING SCHEMA. A bumped BUILD tells you the build changed; this tells you HOW the
     records are shaped, so an analysis script can refuse a log it does not understand instead of
     reading 13 fields out of a 7-field array and reporting the difference as physics. Added after
     exactly that happened. `jf` is the per-joint field order, by name. */
  schema:{v:3, stHz:20, inHz:60, burst:BURST_N, tauUnit:'mN.m', angUnit:'deg',
    jf:['tau','tauPeak','demandFrac','tauFF','limitPeak','angle','target','err',
        'ratePeak','util','satFrac','onStop','broken','govFrac']},
  screen:[innerWidth,innerHeight]});
/* THREE teardown hooks, because on iOS none of them is reliable alone. visibilitychange->hidden is
   the one that actually fires when you switch apps or lock the phone; pagehide fires on navigation;
   beforeunload is the desktop path. All three go through the beacon, and all three are idempotent
   because beaconAll empties the buffer as it goes. */
addEventListener('visibilitychange',()=>{ if(document.visibilityState==='hidden') beaconAll(); });
addEventListener('pagehide',beaconAll);
addEventListener('beforeunload',beaconAll);
