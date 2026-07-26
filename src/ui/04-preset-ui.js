/* ---------- preset UI ---------- */
const bar=document.getElementById('presetBar'), hd=document.getElementById('presetHd');
const optList=document.getElementById('optList'), noteEl=document.getElementById('note');
function showNote(){ const P=PRESETS[preset];
  noteEl.innerHTML='<i>Measured: '+P.steps+'.</i> '+P.note;
  document.getElementById('curName').textContent=P.label; }
for(const k of Object.keys(PRESETS)){
  const b=document.createElement('button'); b.className='opt'; b.textContent=PRESETS[k].label;
  b.addEventListener('click',function(){ preset=k; showNote(); buildWorld(k);
    logEvent('preset',{name:k});
    const all=optList.querySelectorAll('button');
    for(const x of all) x.setAttribute('aria-pressed',String(x===b)); });
  optList.appendChild(b);
}
function toggleBar(){ const o=bar.classList.toggle('open'); hd.setAttribute('aria-expanded',String(o)); }
hd.addEventListener('click',toggleBar);
hd.addEventListener('keydown',function(e){ if(e.key==='Enter'||e.key===' '){e.preventDefault();toggleBar();} });
