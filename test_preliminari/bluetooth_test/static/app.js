import * as THREE from 'three';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';
import {EffectComposer} from 'three/addons/postprocessing/EffectComposer.js';
import {RenderPass} from 'three/addons/postprocessing/RenderPass.js';
import {UnrealBloomPass} from 'three/addons/postprocessing/UnrealBloomPass.js';
import {OutputPass} from 'three/addons/postprocessing/OutputPass.js';

const $ = id => document.getElementById(id);
const host = $('scene');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = .9;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
host.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color('#14202e');
const camera = new THREE.PerspectiveCamera(38,1,.1,50);
camera.position.set(6,4.4,7); camera.lookAt(0,0,0);
const pmrem = new THREE.PMREMGenerator(renderer), room = new RoomEnvironment();
const env = pmrem.fromScene(room,.04); scene.environment = env.texture;scene.environmentIntensity=.6;
room.dispose(); pmrem.dispose();
const key = new THREE.DirectionalLight(0xe3efff,1.8);key.position.set(3,7,5);
key.castShadow=true;key.shadow.mapSize.set(2048,2048);key.shadow.camera.left=-5;key.shadow.camera.right=5;
key.shadow.camera.top=5;key.shadow.camera.bottom=-5;key.shadow.bias=-.0005;key.shadow.normalBias=.025;
scene.add(key,new THREE.HemisphereLight(0xc8eaff,0x263648,1.2));
const rim = new THREE.DirectionalLight(0x78e7d5,.8);rim.position.set(-4,2,-4);scene.add(rim);
const composer = new EffectComposer(renderer);composer.addPass(new RenderPass(scene,camera));
composer.addPass(new UnrealBloomPass(new THREE.Vector2(1,1),.3,.4,.85));composer.addPass(new OutputPass());
const loader = new THREE.TextureLoader();
function texture(name,color=false){const t=loader.load(`/static/material/${name}.jpg`);t.wrapS=t.wrapT=THREE.RepeatWrapping;t.repeat.set(2,2);t.anisotropy=renderer.capabilities.getMaxAnisotropy();if(color)t.colorSpace=THREE.SRGBColorSpace;return t;}
const maps={map:texture('color',true),normalMap:texture('normal'),roughnessMap:texture('roughness'),normalScale:new THREE.Vector2(.15,.15)};
const paint = new THREE.MeshPhysicalMaterial({...maps,color:0xafd5f0,metalness:.35,roughness:.65,clearcoat:.15,clearcoatRoughness:.5});
const accent = new THREE.MeshPhysicalMaterial({...maps,color:0x37ceba,metalness:.25,roughness:.65,clearcoat:.15,clearcoatRoughness:.5});
const glass = new THREE.MeshPhysicalMaterial({...maps,color:0x133349,metalness:.15,roughness:.35,clearcoat:.5,clearcoatRoughness:.25});
const pose = new THREE.Group();scene.add(pose);pose.visible=false;
// Modello diagnostico parametrico: +X muso, +Y ala sinistra, +Z dorso.
// Cambio base B (x,y,z) -> (x,z,-y), una rotazione propria di -90° su X.
const body = new THREE.Group();body.rotation.x=-Math.PI/2;pose.add(body);
function mesh(geometry, material, position=[0,0,0]){const m=new THREE.Mesh(geometry,material);m.position.set(...position);m.castShadow=true;m.receiveShadow=true;body.add(m);return m;}
const profile=[[-2,0.06],[-1.65,.19],[-.8,.28],[.4,.36],[1.3,.27],[1.8,.12],[2,0]];
const fuselage=mesh(new THREE.LatheGeometry(profile.map(([x,r])=>new THREE.Vector2(r,x)),48),paint);fuselage.rotation.z=-Math.PI/2;
function foil(points,z,depth,material){const shape=new THREE.Shape();points.forEach(([x,y],i)=>i?shape.lineTo(x,y):shape.moveTo(x,y));shape.closePath();return mesh(new THREE.ExtrudeGeometry(shape,{depth,bevelEnabled:true,bevelSegments:3,steps:1,bevelSize:.035,bevelThickness:.025}),material,[0,0,z]);}
foil([[.6,.15],[-.1,2.65],[-.65,2.65],[-.55,.15]],-.06,.09,paint);
foil([[.6,-.15],[-.1,-2.65],[-.65,-2.65],[-.55,-.15]],-.06,.09,paint);
foil([[-1.25,.1],[-1.65,1.15],[-2,1.15],[-1.9,.1]],.06,.055,accent);
foil([[-1.25,-.1],[-1.65,-1.15],[-2,-1.15],[-1.9,-.1]],.06,.055,accent);
const fin=foil([[-1.25,0],[-1.75,.9],[-2,.9],[-1.9,0]],-.025,.06,accent);fin.rotation.x=Math.PI/2;fin.position.z=.14;
const canopy=mesh(new THREE.SphereGeometry(1,40,24),glass,[.45,0,.27]);canopy.scale.set(.65,.24,.25);
// Piccoli riferimenti ala: verde a sinistra, rosso a destra, senza luci aggiuntive.
for(const side of [-1,1]){const tip=mesh(new THREE.SphereGeometry(.075,16,12),side>0?accent:paint,[-.4,side*2.61,0]);tip.scale.set(1,1,.5);}
const ringMat=new THREE.MeshStandardMaterial({...maps,color:0x466479,metalness:.3,roughness:.6,transparent:true,opacity:.35});
for(const angle of [0,Math.PI/2]){const ring=new THREE.Mesh(new THREE.TorusGeometry(3.25,.012,8,120),ringMat);ring.rotation.x=angle;scene.add(ring);}
// Nessun suolo o traslazione fittizi: strumento sospeso con cerchi fissi di riferimento.
const basis=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2);
const invBasis=basis.clone().invert();
const raw=new THREE.Quaternion();
let lastValid=false, frames=0;
function applyQuaternion(q){raw.set(q[1],q[2],q[3],q[0]).normalize();pose.quaternion.copy(basis).multiply(raw).multiply(invBasis);}
const labels={attesa:'In attesa di dati',inizializzazione:'Inizializzazione · tieni ferma la scheda',stale:'Dati non aggiornati',disconnesso:'Scheda disconnessa',valido:'Orientamento disponibile'};
for(let i=0;i<7;i++){const s=document.createElement('span');s.textContent=`D${i+2}`;$('buttons').appendChild(s);}
function display(s){lastValid=Boolean(s.valid);pose.visible=lastValid;$('empty').style.display=lastValid?'none':'grid';$('empty').textContent=s.error||labels[s.status]||'Dati non disponibili';$('status').textContent=labels[s.status]||'Server non raggiungibile';$('mode').textContent=s.valid?s.mode:'—';$('reset').disabled=!lastValid;
$('source').textContent=s.demo?'DEMO · DATI SINTETICI':'BLE · SENSORI REALI';$('source').classList.toggle('demo',s.demo);
for(const [i,id] of ['roll','pitch','yaw'].entries())$(id).textContent=s.valid?`${s.rpy[i].toFixed(1)}°`:'—';
$('hz').textContent=`${s.hz??0} Hz`;$('age').textContent=s.age==null?'—':`${Math.round(s.age*1000)} ms`;
$('warning').textContent=s.warning||'Nessun orientamento valido';
[...$('buttons').children].forEach((el,i)=>{el.className=s.buttons?(s.buttons[i]?'on':''):'unknown';});
if(s.valid)applyQuaternion(s.quaternion);
}
let stopped=false;
async function poll(){if(stopped)return;try{const r=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(1000)});if(!r.ok)throw Error('HTTP');display(await r.json());}catch{pose.visible=false;lastValid=false;$('reset').disabled=true;$('empty').style.display='grid';$('empty').textContent='Server non raggiungibile';$('status').textContent='Visualizzatore scollegato';for(const id of ['roll','pitch','yaw'])$(id).textContent='—';[...$('buttons').children].forEach(el=>el.className='unknown');$('hz').textContent='—';$('age').textContent='—';}setTimeout(poll,50);}
$('reset').addEventListener('click',async()=>{try{const r=await fetch('/api/reset',{method:'POST',headers:{'X-Reset-Token':document.querySelector('meta[name=reset-token]').content},signal:AbortSignal.timeout(1500)});if(!r.ok)throw Error();display((await r.json()).state);$('feedback').textContent='Riferimento azzerato. Non è una calibrazione dei sensori.';}catch{$('feedback').textContent='Azzeramento non disponibile: controlla lo stato dei dati.';}});
const resize=new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h);composer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();});resize.observe(host);
renderer.setAnimationLoop(()=>{composer.render();frames++;});poll();
// Diagnostica di sola lettura per verificare cambio base e renderer nei test browser.
window.__IMU_VIEW__={getState:()=>({frames,visible:pose.visible,quaternion:pose.quaternion.toArray(),calls:renderer.info.render.calls}),transform:q=>{const p=new THREE.Quaternion(q[1],q[2],q[3],q[0]);return basis.clone().multiply(p).multiply(invBasis).toArray();}};
window.addEventListener('pagehide',()=>{stopped=true;resize.disconnect();renderer.setAnimationLoop(null);scene.traverse(o=>{o.geometry?.dispose();});for(const m of [paint,accent,glass,ringMat])m.dispose();for(const k of ['map','normalMap','roughnessMap'])maps[k].dispose();env.dispose();composer.dispose();renderer.dispose();},{once:true});
