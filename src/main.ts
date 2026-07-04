import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import * as RAPIER from '@dimforge/rapier3d-compat';
import { Rig, WORLD_GROUPS } from './rig';
import { Controller, setGainScale } from './controller';
import { clamp } from './math';

const CLEAR = new THREE.Color(0x121a28);
const FIXED_DT = 1 / 60;
// Every reference implementation sub-steps: sergioabreu runs PhysX at 500 Hz,
// DeepMimic controls at 600 Hz. Explicit PD chains alias at 60 Hz (their
// pitfall docs say exactly this), so physics+PD run at 180 Hz here.
const SUBSTEPS = 3;
const PHYS_DT = FIXED_DT / SUBSTEPS;

const statusEl = document.getElementById('status')!;
statusEl.textContent = 'compiling physics wasm…';
await RAPIER.init();

const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
world.timestep = PHYS_DT;

const scene = new THREE.Scene();
scene.background = CLEAR;
scene.fog = new THREE.Fog(CLEAR, 18, 40);

const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.1, 100);
camera.position.set(3.2, 2.4, 4.2);

statusEl.textContent = 'creating renderer…';
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
document.getElementById('app')!.appendChild(renderer.domElement);
renderer.domElement.addEventListener('webglcontextlost', (e) => {
  e.preventDefault();
  statusEl.textContent = 'graphics context lost — reloading…';
  setTimeout(() => location.reload(), 400);
});

scene.add(new THREE.HemisphereLight(0xbfd9ff, 0x35414f, 1.0));
const sun = new THREE.DirectionalLight(0xfff2dd, 2.2);
sun.position.set(6, 10, 4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = -10; sun.shadow.camera.right = 10;
sun.shadow.camera.top = 10; sun.shadow.camera.bottom = -10;
scene.add(sun);

// --- static world ---
statusEl.textContent = 'building world…';
function staticBox(hx: number, hy: number, hz: number, x: number, y: number, z: number, rotZ = 0, color = 0x2e4057) {
  const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(x, y, z);
  if (rotZ) bodyDesc.setRotation(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), rotZ));
  const body = world.createRigidBody(bodyDesc);
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(hx, hy, hz).setFriction(1.0).setCollisionGroups(WORLD_GROUPS), body);
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2),
    new THREE.MeshStandardMaterial({ color, roughness: 0.9 }));
  mesh.position.set(x, y, z);
  mesh.rotation.z = rotZ;
  mesh.receiveShadow = true;
  mesh.castShadow = y > 0;
  scene.add(mesh);
}
staticBox(15, 0.5, 15, 0, -0.5, 0, 0, 0x31543f);          // ground
staticBox(1.6, 0.08, 1.2, 3.6, 0.34, 0, -0.24, 0x4d6a55); // ramp
staticBox(0.8, 0.22, 0.8, 5.4, 0.22, 0, 0, 0x4d6a55);     // platform
scene.add(new THREE.GridHelper(30, 30, 0x4a6a8a, 0x27405a));

const crates: { body: RAPIER.RigidBody; mesh: THREE.Mesh }[] = [];
for (const [x, z] of [[-2.5, 1.5], [-3.1, 1.9], [-2.8, 1.7]]) {
  const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, 0.6, z));
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(0.22, 0.22, 0.22).setDensity(120).setFriction(0.7).setCollisionGroups(WORLD_GROUPS), body);
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.44, 0.44, 0.44),
    new THREE.MeshStandardMaterial({ color: 0xc98a4b, roughness: 0.8 }));
  mesh.castShadow = mesh.receiveShadow = true;
  scene.add(mesh);
  crates.push({ body, mesh });
}

// --- character: Soldier.glb + animation-matched physics rig ---
statusEl.textContent = 'loading character…';
const gltf = await new GLTFLoader().loadAsync('./assets/Soldier.glb');
const model = gltf.scene;
model.rotation.y = Math.PI; // Soldier faces -Z at rest; our forward is +Z
model.traverse((o) => {
  if ((o as THREE.Mesh).isMesh) {
    o.castShadow = true;
    o.frustumCulled = false; // bones are driven far from the bind-pose bounds
  }
});
scene.add(model);

const mixer = new THREE.AnimationMixer(model);
const clip = (name: string) => THREE.AnimationClip.findByName(gltf.animations, name);
const idleA = mixer.clipAction(clip('Idle'));
const walkA = mixer.clipAction(clip('Walk'));
const runA = mixer.clipAction(clip('Run'));
for (const a of [idleA, walkA, runA]) { a.enabled = true; a.setEffectiveWeight(0); a.play(); }
idleA.setEffectiveWeight(1);
mixer.update(0);
model.updateMatrixWorld(true);

const rig = new Rig(world, model);
const ctrl = new Controller(world, rig);

// In-place clips: playback speed must track actual velocity or feet skate.
// Nominal clip speeds eyeballed for Mixamo walk/run cycles.
const WALK_NOMINAL = 1.4;
const walkDur = walkA.getClip().duration;
const runDur = runA.getClip().duration;
let runPhaseOffset = 0; // half-cycle (0.5) if the clips start on opposite feet
function updateAnim(dt: number) {
  const s = ctrl.smoothSpeed;
  const wRun = clamp((s - 1.7) / 0.9, 0, 1);
  const wWalk = clamp((s - 0.15) / 0.45, 0, 1) * (1 - wRun);
  const wIdle = Math.max(0, 1 - wWalk - wRun);
  idleA.setEffectiveWeight(wIdle);
  walkA.setEffectiveWeight(wWalk);
  runA.setEffectiveWeight(wRun);
  walkA.setEffectiveTimeScale(clamp(s / WALK_NOMINAL, 0.5, 1.8));
  // Blending two gait cycles only produces a coherent target pose if their
  // phases are locked — slave Run's playhead to Walk's cycle.
  runA.setEffectiveTimeScale(0);
  const phase = (walkA.time / walkDur) % 1;
  runA.time = ((phase + runPhaseOffset) % 1) * runDur;
  mixer.update(dt);
}

// --- input ---
const keys = new Set<string>();
let virtualMove: { x: number; z: number } | null = null; // non-null overrides keyboard
addEventListener('keydown', (e) => {
  if (e.code === 'Space') { e.preventDefault(); ctrl.setRagdoll(!ctrl.ragdoll); return; }
  if (e.code === 'KeyR') { ctrl.shove(); return; }
  keys.add(e.code);
});
addEventListener('keyup', (e) => keys.delete(e.code));
document.getElementById('btn-ragdoll')!.addEventListener('click', () => ctrl.setRagdoll(!ctrl.ragdoll));
document.getElementById('btn-shove')!.addEventListener('click', () => ctrl.shove());

const controls = new OrbitControls(camera, renderer.domElement);
controls.enablePan = false;
controls.maxPolarAngle = Math.PI * 0.49;
controls.minDistance = 2;
controls.maxDistance = 14;

function moveInput(): { x: number; z: number } {
  if (virtualMove) return virtualMove;
  let fx = 0, fz = 0;
  if (keys.has('KeyW') || keys.has('ArrowUp')) fz += 1;
  if (keys.has('KeyS') || keys.has('ArrowDown')) fz -= 1;
  if (keys.has('KeyA') || keys.has('ArrowLeft')) fx += 1;
  if (keys.has('KeyD') || keys.has('ArrowRight')) fx -= 1;
  if (!fx && !fz) return { x: 0, z: 0 };
  const dir = new THREE.Vector3();
  camera.getWorldDirection(dir);
  dir.y = 0;
  dir.normalize();
  const right = new THREE.Vector3().crossVectors(dir, new THREE.Vector3(0, 1, 0));
  const out = new THREE.Vector3().addScaledVector(dir, fz).addScaledVector(right, -fx).normalize();
  return { x: out.x, z: out.z };
}

// --- loop ---
const followTarget = new THREE.Vector3(0, 0.9, 0);

function fixedStep() {
  const move = moveInput();
  // animation targets sample at 60 Hz; physics + PD run at 180 Hz (Jolt rule:
  // drive once per physics step, with the physics dt)
  updateAnim(FIXED_DT);
  model.updateMatrixWorld(true);
  rig.updateTargets(FIXED_DT);
  for (let i = 0; i < SUBSTEPS; i++) {
    ctrl.prePhysics(PHYS_DT, move);
    ctrl.applyTracking(PHYS_DT);
    world.step();
  }
  rig.writeBack();
}

function render() {
  for (const { body, mesh } of crates) {
    const t = body.translation(), q = body.rotation();
    mesh.position.set(t.x, t.y, t.z);
    mesh.quaternion.set(q.x, q.y, q.z, q.w);
  }
  const p = rig.hips.body.translation();
  followTarget.lerp(new THREE.Vector3(p.x, p.y + 0.2, p.z), 0.08);
  controls.target.copy(followTarget);
  controls.update();
  renderer.render(scene, camera);
}

let last = performance.now(), acc = 0;
function frame(now: number) {
  acc = Math.min(acc + (now - last) / 1000, 0.12);
  last = now;
  while (acc >= FIXED_DT) { fixedStep(); acc -= FIXED_DT; }
  render();
  statusEl.textContent =
    `${ctrl.ragdoll ? 'RAGDOLL' : ctrl.grounded ? 'active' : 'airborne'} · ` +
    `v=${ctrl.smoothSpeed.toFixed(2)} m/s · err=${ctrl.meanErrDeg().toFixed(0)}°`;
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

// --- headless test API (hidden preview tabs freeze rAF; tests drive the sim) ---
declare global {
  interface Window {
    __state: () => object;
    __step: (n?: number) => object;
    __setMove: (x: number | null, z?: number) => void;
    __ragdoll: (on: boolean) => void;
    __shove: () => void;
    __gain: (v: number) => void;
    __probe: () => object;
    __debug: object;
  }
}
window.__state = () => {
  const p = rig.hips.body.translation();
  return {
    hips: { x: +p.x.toFixed(3), y: +p.y.toFixed(3), z: +p.z.toFixed(3) },
    speed: +ctrl.smoothSpeed.toFixed(3),
    upY: +ctrl.upY().toFixed(3),
    errDeg: +ctrl.meanErrDeg().toFixed(1),
    alpha: +ctrl.masterAlpha.toFixed(2),
    grounded: ctrl.grounded,
    ragdoll: ctrl.ragdoll,
  };
};
window.__step = (n = 1) => {
  for (let i = 0; i < n; i++) fixedStep();
  render();
  return window.__state();
};
window.__setMove = (x, z = 0) => { virtualMove = x === null ? null : { x, z }; };
window.__ragdoll = (on) => ctrl.setRagdoll(on);
window.__shove = () => ctrl.shove();
window.__gain = (v) => setGainScale(v);
window.__probe = () => {
  render();
  const gl = renderer.getContext();
  const w = gl.drawingBufferWidth, h = gl.drawingBufferHeight;
  const px = new Uint8Array(4 * 64 * 64);
  gl.readPixels(((w - 64) / 2) | 0, ((h - 64) / 2) | 0, 64, 64, gl.RGBA, gl.UNSIGNED_BYTE, px);
  let non = 0;
  for (let i = 0; i < px.length; i += 4) {
    if (Math.abs(px[i] - 18) > 12 || Math.abs(px[i + 1] - 26) > 12 || Math.abs(px[i + 2] - 40) > 12) non++;
  }
  return { nonBackgroundFraction: +(non / (64 * 64)).toFixed(3) };
};
window.__debug = {
  world, rig, ctrl, RAPIER, mixer, model, camera, renderer,
  setRunPhase: (v: number) => { runPhaseOffset = v; },
};

statusEl.textContent = 'ready';
console.log('[poc] ready — rapier', RAPIER.version(), '· tracking', rig.ordered.length, 'bones');
