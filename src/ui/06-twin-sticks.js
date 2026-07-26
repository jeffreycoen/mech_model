/* ---------- twin sticks ----------
   Each zone spawns its stick wherever the thumb lands, so there is no fixed target to
   hunt for on a small screen. Left: forward = speed, lateral = steer. Right: lateral =
   orbit, vertical = zoom. */
function makeStick(zoneId,svgId,knobId){
  const zone=document.getElementById(zoneId), svg=document.getElementById(svgId), knob=document.getElementById(knobId);
  const st={x:0,y:0};
  let id=null, ox=0, oy=0;
  function centre(){ const r=zone.getBoundingClientRect();
    svg.style.left=(r.width/2)+'px'; svg.style.top=(r.height*0.55)+'px'; }
  function reset(){ st.x=0; st.y=0; id=null;
    knob.setAttribute('cx',59); knob.setAttribute('cy',59); svg.classList.remove('live'); centre(); }
  zone.addEventListener('pointerdown',function(e){
    if(id!==null) return;
    id=e.pointerId; zone.setPointerCapture(id);
    ox=e.clientX; oy=e.clientY;
    const r=zone.getBoundingClientRect();
    svg.style.left=(ox-r.left)+'px'; svg.style.top=(oy-r.top)+'px';
    svg.classList.add('live');
  });
  zone.addEventListener('pointermove',function(e){
    if(e.pointerId!==id) return;
    let dx=e.clientX-ox, dy=e.clientY-oy;
    const len=Math.hypot(dx,dy), max=64;
    if(len>max){ dx*=max/len; dy*=max/len; }
    st.x=Math.max(-1,Math.min(1,dx/max)); st.y=Math.max(-1,Math.min(1,-dy/max));
    knob.setAttribute('cx',59+st.x*37); knob.setAttribute('cy',59-st.y*37);
  });
  zone.addEventListener('pointerup',function(e){ if(e.pointerId===id) reset(); });
  zone.addEventListener('pointercancel',function(e){ if(e.pointerId===id) reset(); });
  requestAnimationFrame(centre);
  addEventListener('resize',centre);
  return st;
}
const stickL=makeStick('zoneL','stickL','knobL');
const stickR=makeStick('zoneR','stickR','knobR');

const keys={};
addEventListener('keydown',function(e){ keys[e.key.toLowerCase()]=true;
  if(['arrowleft','arrowright','arrowup','arrowdown'].indexOf(e.key.toLowerCase())>=0) e.preventDefault(); });
addEventListener('keyup',function(e){ keys[e.key.toLowerCase()]=false; });
function kbAxis(neg,pos){ return (keys[pos]?1:0)-(keys[neg]?1:0); }
