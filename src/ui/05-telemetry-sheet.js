/* ---------- telemetry sheet ---------- */
const tel=document.getElementById('tel'), telHd=document.getElementById('telHd');
function toggleTel(){ if(innerWidth>=900) return;
  const o=tel.classList.toggle('open'); telHd.setAttribute('aria-expanded',String(o)); }
telHd.addEventListener('click',toggleTel);
telHd.addEventListener('keydown',function(e){ if(e.key==='Enter'||e.key===' '){e.preventDefault();toggleTel();} });

/* ---------- strip placement on narrow screens (MK1.41.0) ----------
   The preset bar is fixed top-right and 300 px wide; on a phone it covers x~130-430, which
   is most of the strip's first row -- the phone screenshot showed every chip after the
   first either ghosted under the bar or clipped. Below 700 px the strip drops BELOW the
   bar, measured, not guessed, so a taller bar (open picker) pushes it down too. */
(function placeStrip(){
  const strip=document.getElementById('strip'), bar=document.getElementById('presetBar');
  if(!strip||!bar) return;
  const apply=()=>{ strip.style.top=innerWidth<700?(bar.getBoundingClientRect().bottom+6)+'px':'10px'; };
  addEventListener('resize',apply);
  bar.addEventListener('click',()=>setTimeout(apply,50));
  apply();
})();

/* ---------- capturability light ----------
   capState (0 green / 1 yellow / 2 red) and accVec are written once per display frame by
   the loop in 11-loop.js, before it calls renderHudExtras(). fallen forces red defensively
   even though the loop already leaves capState at 2 while fallen -- belt and suspenders,
   cheap either way. */
const capEl=document.getElementById('c-cap'),
      capDot=document.getElementById('c-cap-dot'),
      capLabel=document.getElementById('c-cap-label');
const CAP_LABEL=['balanced','stepping','beyond'];   // fixed 9ch chip width, see shell CSS
const CAP_CLASS=['chip','chip notice','chip warn'];
const CAP_COLOR=['var(--low)','var(--mid)','var(--high)'];

/* ---------- pelvis accelerometer strip chart ----------
   Rolling display-frame history, ~6 s at an assumed 60 Hz display cadence (a frame count,
   not simT-based -- the chart is a qualitative strip, not a measurement instrument).
   One shift+push per axis per frame, no per-frame allocation in the draw path: colors are
   literal hex (canvas does not resolve CSS custom properties) matching --high/--low/--slate,
   and accPlotAxis below is a top-level function, not a closure rebuilt every call. */
const accCanvas=document.getElementById('accCanvas'), accCtx=accCanvas.getContext('2d');
const accRangeEl=document.getElementById('accRange');
const ACC_W=180, ACC_H=60, ACC_N=360;                    // 60fps * 6s
(function sizeAccCanvas(){
  const dpr=Math.max(1,window.devicePixelRatio||1);
  accCanvas.width=Math.round(ACC_W*dpr); accCanvas.height=Math.round(ACC_H*dpr);
  accCtx.setTransform(dpr,0,0,dpr,0,0);
})();
const accHistX=new Array(ACC_N).fill(0),
      accHistY=new Array(ACC_N).fill(0),
      accHistZ=new Array(ACC_N).fill(0);
function accPlotAxis(hist,color,mx){
  accCtx.strokeStyle=color; accCtx.lineWidth=1; accCtx.beginPath();
  for(let i=0;i<ACC_N;i++){
    const x=i/(ACC_N-1)*ACC_W, y=ACC_H/2-(hist[i]/mx)*(ACC_H/2-2);
    if(i===0) accCtx.moveTo(x,y); else accCtx.lineTo(x,y);
  }
  accCtx.stroke();
}
function drawAccChart(){
  let mx=0.5;                                            // floor so a still mech doesn't zoom into noise
  for(let i=0;i<ACC_N;i++){
    const ax=Math.abs(accHistX[i]),ay=Math.abs(accHistY[i]),az=Math.abs(accHistZ[i]);
    if(ax>mx) mx=ax; if(ay>mx) mx=ay; if(az>mx) mx=az;
  }
  accCtx.clearRect(0,0,ACC_W,ACC_H);
  accCtx.strokeStyle='rgba(168,162,150,.4)'; accCtx.lineWidth=1;
  accCtx.beginPath(); accCtx.moveTo(0,ACC_H/2); accCtx.lineTo(ACC_W,ACC_H/2); accCtx.stroke();
  accPlotAxis(accHistX,'#A83232',mx);                     // --high
  accPlotAxis(accHistY,'#4E7A5E',mx);                     // --low
  accPlotAxis(accHistZ,'#2E5D6B',mx);                     // --slate
  accRangeEl.textContent='±'+mx.toFixed(1)+' m/s²';
}

function renderHudExtras(){
  const cs=fallen?2:capState;
  capEl.className=CAP_CLASS[cs];
  capDot.style.background=CAP_COLOR[cs];
  capLabel.textContent=CAP_LABEL[cs];

  accHistX.shift(); accHistX.push(accVec[0]);
  accHistY.shift(); accHistY.push(accVec[1]);
  accHistZ.shift(); accHistZ.push(accVec[2]);
  drawAccChart();
}

/* ---------- drive card ----------
   Scripted prompt sequence for a driving session. This only prompts and stamps the log --
   it never touches gait.want/stick state; the human follows the prompts. Timed off simT
   deltas so pausing the sim pauses the card exactly like everything else in this file. */
const CARD_SEGS=[
  {name:'stand',    prompt:'hands off — stand still',       dur:10},
  {name:'fwd',      prompt:'left stick full forward',       dur:5},
  {name:'release1', prompt:'release stick, hands off',      dur:4},
  {name:'back',     prompt:'left stick full back',          dur:3},
  {name:'release2', prompt:'release stick, hands off',      dur:4},
  {name:'turn',     prompt:'right stick — full turn, hold', dur:12},
  {name:'walkturn', prompt:'walk forward while turning',    dur:8},
  {name:'strafe',   prompt:'left stick full sideways',      dur:5},
  {name:'done',     prompt:'card complete',                 dur:0},
];
const CARD_DONE_GRACE=2;          // seconds of simT the "card complete" chip lingers
const cardBtn=document.getElementById('c-card'), cardSegEl=document.getElementById('c-card-seg');
let cardActive=false, cardIdx=-1, cardElapsed=0, cardLastSimT=0, cardDoneAt=-Infinity;

function cardBegin(seg){
  logEvent('card',{seg:seg.name,i:cardIdx});
  if(seg.dur<=0){                 // zero-duration segment completes the card immediately
    logEvent('card',{seg:'end'});
    cardActive=false; cardDoneAt=simT;
  }
}
function cardStart(){
  cardActive=true; cardIdx=0; cardElapsed=0; cardLastSimT=simT;
  cardBegin(CARD_SEGS[0]);
}
function cardCancel(){
  logEvent('card',{seg:'end'});
  cardActive=false; cardIdx=-1;
}
function cardAdvance(){
  cardIdx++; cardElapsed=0;
  cardBegin(CARD_SEGS[cardIdx]);
}
cardBtn.addEventListener('click',function(){ if(cardActive) cardCancel(); else cardStart(); });

function driveCardTick(){
  if(cardActive){
    const dt=simT-cardLastSimT; cardLastSimT=simT;
    cardElapsed+=dt;
    const seg=CARD_SEGS[cardIdx];
    if(seg.dur>0 && cardElapsed>=seg.dur) cardAdvance();
  }
  cardBtn.setAttribute('aria-pressed',String(cardActive));
  cardBtn.textContent=cardActive?'CANCEL':'CARD';
  if(cardActive){
    const seg=CARD_SEGS[cardIdx], remain=Math.max(0,seg.dur-cardElapsed);
    cardSegEl.style.display='';
    cardSegEl.textContent=seg.dur>0?(seg.prompt+' · '+remain.toFixed(1)+'s'):seg.prompt;
  } else if(simT-cardDoneAt<CARD_DONE_GRACE){
    cardSegEl.style.display=''; cardSegEl.textContent='card complete';
  } else cardSegEl.style.display='none';
}
