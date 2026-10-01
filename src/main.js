import { createRenderer } from './render.js';
import { buildCar, WHEELS } from './car.js';
import { light } from './light.js';
import { add, sub, scl, qrot, len } from './math.js';
import { World, step, boxCollider, setActive } from './soft.js';
import { createCar, carBones, SUB } from './vehicle.js';
import { bindSkin } from './lattice.js';
import { height, normalAt, BARRIER } from './world.js';
import { buildSet, buildLid } from './scene.js';
const $ = id => document.getElementById(id);
const cv = $('c');
const R = createRenderer(cv);
const g = buildCar();
const W = World(); W.ground = height; W.groundN = normalAt;
const car = createCar(W, 1, { payload: 78 });
setActive(W);
const skin = bindSkin(g, car.L);
const carM = R.carMesh(g, skin);
W.colliders.push(boxCollider([0, BARRIER.h / 2, BARRIER.d / 2], [BARRIER.w, BARRIER.h / 2, BARRIER.d / 2], 0, { mu: .4 }));
const setG = buildSet(), setM = R.statMesh(setG), lidM = R.statMesh(buildLid());
const I4 = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
const view = { tod: .84, ts: 1 };
const U = { uHub: WHEELS.flat(), uPaint: [.62, .07, .06], uPaint2: [.03, .03, .03], uTwo: 0, uMarks: 1, uLights: 0, uDirt: .5, uRoof: 0, uExplode: 0, uXray: 0 };
car.place([0, 0, -30], 0, 0);
function size() { const d = Math.min(devicePixelRatio || 1, 1.5); cv.width = Math.round(innerWidth * d); cv.height = Math.round(innerHeight * d); }
addEventListener('resize', size); size();
const keys = {}; addEventListener('keydown', e => keys[e.code] = 1); addEventListener('keyup', e => keys[e.code] = 0);
let simT = 0, last = performance.now();
window.__ct = { car, W, view, run(kmh) { car.reset(); car.place([0, 0, -2.05 - .3], 0, kmh / 3.6); simT = 0; }, step(n) { for (let i = 0; i < n; i++) tick(1 / 60); } };
function tick(dt) {
  car.throttle = keys.KeyW ? 1 : 0; car.brake = keys.KeyS ? 1 : 0; car.steerIn = (keys.KeyA ? 1 : 0) - (keys.KeyD ? 1 : 0);
  const ts = view.ts;
  const sdt = dt * ts;
  if (car.mode === 'soft') { const n = Math.max(1, Math.round(sdt * 60 * SUB)); step(W, n / (60 * SUB), n); }
  car.update(sdt, simT); simT += sdt;
}
function loop(now) {
  requestAnimationFrame(loop);
  const dt = Math.min((now - last) / 1000, 1 / 20); last = now;
  if (!view.paused) tick(dt);
  const cp = car.world(car.com);
  let cam = { pos: add(cp, [-4.5, 1.6, 2]), tgt: cp, fov: .7, near: .05, far: 4000 };
  if (view.cam) cam = Object.assign(cam, view.cam);
  const L = light(view.tod); L.uCam = cam.pos;
  const { B, A } = carBones(car);
  R.frame({ t: now / 1000, res: 1, cam, light: L, cars: [{ car, mesh: carM, u: Object.assign({ uB: B, uA: A }, U) }], statics: [{ mesh: setM, M: I4 }, { mesh: lidM, M: I4, glass: 1 }], focus: cp, post: { exposure: view.exp || 1, sat: 1.12 } });
}
requestAnimationFrame(loop);
