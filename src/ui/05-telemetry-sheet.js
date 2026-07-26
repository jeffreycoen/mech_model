/* ---------- telemetry sheet ---------- */
const tel=document.getElementById('tel'), telHd=document.getElementById('telHd');
function toggleTel(){ if(innerWidth>=900) return;
  const o=tel.classList.toggle('open'); telHd.setAttribute('aria-expanded',String(o)); }
telHd.addEventListener('click',toggleTel);
telHd.addEventListener('keydown',function(e){ if(e.key==='Enter'||e.key===' '){e.preventDefault();toggleTel();} });
