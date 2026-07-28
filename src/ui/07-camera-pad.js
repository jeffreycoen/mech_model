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
  /* TURN PAD. A held button is a RATE command, which is the difference between this and the aim
     stick: the stick points at an absolute heading (gait.aim = atan2 of the stick vector), so a
     flick is a position step across the whole ring. A button asks for "keep turning this way" and
     the heading integrates at a derived rate, so there is no step to absorb at all.
     `turnBtn` is -1 / 0 / +1 and is consumed in ui/11-loop.js. */
  hold('turnL',(v)=>{ turnBtn=-v; });
  hold('turnR',(v)=>{ turnBtn= v; });
})();
