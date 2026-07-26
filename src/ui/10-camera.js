/* ---------- camera ---------- */
let span=9.8, spanWant=9.8, spanMin=1, spanMax=24, lastSpan=-1, lastW=-1, lastH=-1;
function resize(){
  const w=innerWidth,h=innerHeight;
  renderer.setSize(w,h);
  const railW=(w>=900)?280:0;
  const vw=Math.max(1,w-railW), vh=Math.max(1,h), a=vw/vh;
  camera.left=-span*a/2; camera.right=span*a/2; camera.top=span/2; camera.bottom=-span/2;
  camera.updateProjectionMatrix();
  renderer.setViewport(0,0,vw,vh); renderer.setScissor(0,0,vw,vh); renderer.setScissorTest(true);
  lastSpan=span; lastW=w; lastH=h;
}
addEventListener('resize',resize);
