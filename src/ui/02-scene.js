/* ---------- scene ---------- */
const scene=new THREE.Scene(); scene.background=new THREE.Color('#E8E4DA');
const ISO=Math.atan(1/Math.SQRT2), DET=24, STEP=Math.PI*2/DET;
let detent=3, azimuth=detent*STEP, azShown=azimuth, azCont=azimuth;
const camera=new THREE.OrthographicCamera(-1,1,1,-1,0.1,400);
const renderer=new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
document.getElementById('stage').appendChild(renderer.domElement);
scene.add(new THREE.HemisphereLight(0xffffff,0x9a9382,0.85));
const keyLight=new THREE.DirectionalLight(0xfff6e6,0.6); keyLight.position.set(6,12,4); scene.add(keyLight);
scene.add(new THREE.GridHelper(200,200,0x8A8577,0xC9C3B5));

const RAMP=[[0,[0x4E,0x7A,0x5E]],[0.55,[0xC9,0x92,0x2E]],[1,[0xA8,0x32,0x32]]];
function ramp(u){u=Math.max(0,Math.min(1,u));let a=RAMP[0],b=RAMP[2];
  for(let i=0;i<2;i++){if(u>=RAMP[i][0]&&u<=RAMP[i+1][0]){a=RAMP[i];b=RAMP[i+1];break;}}
  const f=(u-a[0])/Math.max(1e-6,b[0]-a[0]);
  return (Math.round(a[1][0]+(b[1][0]-a[1][0])*f)<<16)|(Math.round(a[1][1]+(b[1][1]-a[1][1])*f)<<8)|Math.round(a[1][2]+(b[1][2]-a[1][2])*f);}
