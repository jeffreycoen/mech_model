/* ---------- camera pad ---------- */
(function(){
  const hold=(id,fn)=>{ const el=document.getElementById(id);
    const on=(e)=>{ e.preventDefault(); fn(1); };
    const off=()=>fn(0);
    el.addEventListener('pointerdown',on); el.addEventListener('pointerup',off);
    el.addEventListener('pointercancel',off); el.addEventListener('pointerleave',off); };
  hold('orbL',(v)=>{ orbitBtn=-v; });
  hold('orbR',(v)=>{ orbitBtn= v; });
  hold('zIn', (v)=>{ zoomBtn= v; });
  hold('zOut',(v)=>{ zoomBtn=-v; });
})();
