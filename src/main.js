import { createRenderer } from './render.js';
import { buildCar, WHEELS, SEAT } from './car.js';
import { light } from './light.js';
import { add, sub, scl, len, norm, cross, dot, qrot, qaxis, qconj, clamp, lerp, mul } from './math.js';
import { createSim, TESTS, REF76, RAMP, rampH } from './sim.js';
import { carBones } from './vehicle.js';
import { bindSkin } from './lattice.js';
import { buildDummyMesh, dummyBones } from './dummy.js';
import { BARRIER, AVE_Z, FIELD, sunDir } from './world.js';
import { buildSet, buildLid, buildProps } from './scene.js';
import { createFX } from './fx.js';
import { createFilm } from './story.js';
import { createAudio } from './audio.js';

const $ = id => document.getElementById(id);
const cv = $('c'), ov = $('o'), ctx = ov.getContext('2d');
let R;
try { R = createRenderer(cv); } catch (e) { $('menu').innerHTML = '<h1>DEUX CHEVAUX</h1><p>' + e.message + '</p>'; throw e; }
const g = buildCar();
const S = createSim(), W = S.W, A = S.A, B = S.B, D = S.D;
const carM = R.carMesh(g, bindSkin(g, A.L));
const dm = buildDummyMesh(), dumM = R.carMesh(dm.g, { J: dm.J, W: dm.Wt });
const I4 = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
const T4 = (x, y, z, sx = 1, sy = 1, sz = 1) => new Float32Array([sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, sz, 0, x, y, z, 1]);
const P = buildProps();
const setM = R.statMesh(buildSet()), lidM = R.statMesh(buildLid()), propM = Object.fromEntries(Object.entries(P).map(([k, v]) => [k, R.statMesh(v)]));
const FX = createFX();
const PAINTS = [['Jaune Mimosa', [.85, .62, .04]], ['Charleston (Delage / noir)', [.33, .03, .04], [.02, .02, .02]], ['Gris Cormoran', [.33, .35, .37]], ['Bleu Cyclades', [.12, .27, .5]],
  ['Vert Tuileries', [.1, .26, .14]], ['Rouge Vallelunga', [.6, .05, .04]], ['Blanc Meije', [.78, .77, .72]]];
const view = { paint: 0, paintB: 5, xray: 0, roof: 0, marks: 1, explode: 0, tod: .82, lights: 0 };
W.onBreak = j => {
  if (S.replay) return;
  const a = W.ba[j], b = W.bb[j], p = [0, 1, 2].map(k => (W.x[a * 3 + k] + W.x[b * 3 + k]) / 2), v = [0, 1, 2].map(k => W.v[a * 3 + k]);
  if (W.bm[j] === 1) {
    for (let i = 0; i < 3; i++) FX.spawn(W.t, p, v, 0, [.8, .9, .95]);
    for (const C of [A, B]) { const ia = a - C.base, ib = b - C.base; if (ia < 0 || ia >= C.n) continue;
      const r = [0, 1, 2].map(k => (C.restL[ia * 3 + k] + C.restL[ib * 3 + k]) / 2); if (r[2] > .3 && r[2] < .7 && r[1] > 1) { C.crackN = (C.crackN || 0) + 1; if (!C.crackP) C.crackP = r; C.crack = Math.min(1, C.crackN / 20); } }
  }
  else if (Math.random() < .3) FX.spawn(W.t, p, v, 1, PAINTS[view.paint][1]);
};
// ---------------- input
const keys = {}, mouse = { dx: 0, dy: 0, down: 0 };
addEventListener('keydown', e => { keys[e.code] = 1; onKey(e); if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) e.preventDefault(); });
addEventListener('keyup', e => keys[e.code] = 0);
cv.addEventListener('pointerdown', e => { mouse.down = 1; mouse.x = e.clientX; mouse.y = e.clientY; });
addEventListener('pointerup', () => mouse.down = 0);
addEventListener('pointermove', e => { if (mouse.down) { mouse.dx += e.clientX - mouse.x; mouse.dy += e.clientY - mouse.y; mouse.x = e.clientX; mouse.y = e.clientY; orbit.user = 3; } });
addEventListener('wheel', e => { orbit.d = clamp(orbit.d * Math.exp(e.deltaY * .001), 1.2, 40); orbit.user = 3; });
const orbit = { yaw: 2.3, pitch: .25, d: 7, user: 0 };
// ---------------- modes
let mode = 'menu', film = null, audio = null, labState = 'idle', seq = null, camName = 'side', camT = 0, driveCam = 0;
const show = (id, on, disp = 'block') => $(id).style.display = on ? disp : 'none';
function enter(m) {
  if (!audio) try { audio = createAudio(); } catch (e) { }
  audio?.ctx.resume();
  mode = m; show('menu', m === 'menu', 'flex'); show('lab', m === 'lab'); show('report', false); show('keys', m === 'drive');
  $('cap').textContent = ''; $('title').style.opacity = 0;
  S.stopReplay(); FX.reset(); S.paused = false; S.ts = 1;
  if (m === 'lab') { view.paint = +$('paint').value; labIdle(); }
  if (m === 'drive') { view.paint = 1; driveStart(); }
  if (m === 'film') { film = createFilm(api); }
  if (m === 'menu') { S.start(TESTS[0], 0, { runup: 0 }); A.place([3, 0, -18], 2.5, 0); S.setBarrier('wall'); S.phase = 'idle'; }
}
$('bf').onclick = () => enter('film'); $('bl').onclick = () => enter('lab'); $('bd').onclick = () => enter('drive');
// ---------------- lab UI
TESTS.forEach((t, i) => $('test').add(new Option(t.name, i)));
PAINTS.forEach((p, i) => $('paint').add(new Option(p[0], i)));
const CAMS = ['side', 'front', 'top', 'pit', 'onboard', 'wide', 'orbit'];
CAMS.forEach((c, i) => { const b = document.createElement('button'); b.textContent = (i + 1) + ' ' + c; b.onclick = () => setCam(c); $('cams').appendChild(b); });
function setCam(c) { camName = c; orbit.user = c === 'orbit' ? 3 : 0; [...$('cams').children].forEach((b, i) => b.classList.toggle('on', CAMS[i] === c)); }
const curTest = () => TESTS[+$('test').value];
$('test').onchange = () => { const t = curTest(); $('kmh').value = t.kmh; $('tnote').textContent = t.note; $('kmhv').textContent = t.kmh; labIdle(); };
$('kmh').oninput = () => $('kmhv').textContent = $('kmh').value;
$('paint').onchange = () => view.paint = +$('paint').value;
for (const id of ['xray', 'roof', 'marks']) $(id).onchange = () => view[id] = $(id).checked ? 1 : 0;
$('run').onclick = () => labRun(); $('rep').onclick = () => labReplay();
$('test').value = 0; $('test').onchange(); setCam('side');
function labIdle() {
  const t = curTest(); S.start(t, +$('kmh').value, { runup: 0, belt: $('belt').checked, dummy: $('dummy').checked });
  // park it at the start line, stopped (start() released a sled test immediately: undo that)
  S.stopReplay(); S.recOn = false; S.phase = 'idle'; A.mode = 'soft'; A.reset(); A.mode = 'rigid'; W.awake[1] = 0; W.awake[2] = 0; D.reseat();
  if (t.side) A.place([-.15, 0, -6], -Math.PI / 2, 0); else if (t.set === 'headon') { A.place([.15, 0, -52], 0, 0); B.reset(); B.place([-.15, 0, -28], Math.PI, 0); B.frozen = true; }
  else A.place([0, 0, t.set === 'ramp' ? -30 : -14], 0, 0);
  labState = 'idle'; seq = null; show('report', false); FX.reset(); setCam('orbit'); orbit.user = 0; orbit.d = 7;
}
function labRun() {
  const t = curTest(), kmh = +$('kmh').value, run = $('runup').checked && !t.side && t.set !== 'headon';
  S.start(t, kmh, { runup: run ? Math.max(25, kmh * kmh / 3.6 / 3.6 / 2 / 2.2 + 8) : 0, belt: $('belt').checked, dummy: $('dummy').checked });
  FX.reset(); labState = 'run'; seq = null; show('report', false); S.ts = 1; camT = 0;
  if (run) setCam('track'); else setCam(t.side ? 'front' : 'side');
  audio?.event('start');
}
function labReplay() { if (S.rec.length < 3) return; const w = replayWindow(); seq = { list: ['side', 'pit', 'onboard', 'top', 'front'].filter(c => c !== 'pit' || S.barrier === 'wall' || S.barrier === 'odb'), k: -1, w }; nextReplay(); }
function replayWindow() {
  const r = S.rec; if (r.length < 2) return [0, 0]; let i = r.findIndex(f => f.pulse > 2.5); if (i < 0) i = 0;
  const t0 = r[Math.max(0, i - 35)].t, i0 = r.findIndex(f => f.t >= t0), i1 = Math.min(r.length - 1, i0 + 260);
  return [i0, i1];
}
function nextReplay() {
  seq.k++;
  if (seq.k >= seq.list.length) { S.stopReplay(); seq = null; labState = 'done'; showReport(); setCam('orbit'); orbit.user = 0; return; }
  S.stopReplay(); S.startReplay(.075, seq.w[0]); S.replay.end = seq.w[1]; setCam(seq.list[seq.k]);
}
// ---------------- report
function showReport() {
  const r = S.report; if (!r) return;
  const col = (v, a, b) => v < a ? 'g' : v < b ? 'a' : 'r';
  const row = (k, v, u, c, ref) => `<tr><td>${k}</td><td class="${c || ''}">${v}${u}</td>${ref !== undefined ? `<td>${ref}${u}</td>` : ''}</tr>`;
  const f0 = v => Math.round(v).toLocaleString('en');
  const ref = r.test.ref && Math.abs(r.kmh - 40) < 1;
  let h = `<h3>${r.test.name} · ${r.kmh} km/h</h3><table>${ref ? '<tr><td></td><td><b>simulated</b></td><td><b>1976 test</b></td></tr>' : ''}`;
  h += row('Car deceleration (peak)', f0(r.g), ' g', '', ref ? REF76.g : undefined);
  if (!r.test.side) h += row('Static crush', f0(r.crush), ' mm', '', ref ? REF76.crush : undefined);
  if (r.dummy) {
    h += row('Head injury criterion HIC15', f0(r.hic), '', col(r.hic, 650, 1000), ref ? REF76.hic : undefined);
    h += row('Head acceleration (3 ms)', f0(r.head), ' g', col(r.head, 72, 88), ref ? REF76.head : undefined);
    h += row('Chest acceleration (3 ms)', f0(r.chest), ' g', col(r.chest, 45, 60), ref ? REF76.chest : undefined);
    h += row('Shoulder-belt load', f0(r.belt), ' kgf', '', ref ? REF76.belt : undefined);
  }
  h += row('Footwell intrusion', f0(Math.max(0, r.intr)), ' mm', col(r.intr, 100, 200));
  h += row('Glass bonds broken', r.glass, '', '');
  h += '</table>';
  const verdict = !r.dummy ? 'No dummy on board.' : r.hic > 1000 || r.chest > 60 ? 'Life-threatening for the driver.' : r.hic > 650 || r.chest > 45 ? 'Serious injury likely.' : 'Survivable — the belt did its job.';
  h += `<p class="note" style="margin-top:8px">${verdict}${ref ? ' The simulation re-runs the 1976 test live; the lattice was calibrated to its crush and peak g.' : ''}</p><div class="row" style="display:flex;gap:6px"><button id="r2">replay</button><button id="r3">again</button></div>`;
  $('report').innerHTML = h; show('report', true);
  $('r2').onclick = () => { show('report', false); labReplay(); }; $('r3').onclick = () => labRun();
}
S.onReport = () => { if (mode === 'lab') { labState = 'replay'; setTimeout(() => { if (labState === 'replay' && !seq) labReplay(); }, 700); } if (mode === 'drive') { driveCrash = 1; S.phase = 'free'; } };
// ---------------- drive
let driveCrash = 0, msgT = 0;
function driveStart() {
  S.start({ id: 'drive', name: 'Free drive', set: 'drive' }, 0, { runup: 0, dummy: false });
  S.stopReplay(); S.recOn = false; A.mode = 'rigid'; W.awake[1] = W.awake[2] = 0; S.phase = 'free'; S.setBarrier('drive');
  A.place([2.5, 0, AVE_Z - 2], Math.PI / 2, 0); A.gear = 2; orbit.user = 0; driveCrash = 0;
  $('keys').innerHTML = 'W/↑ throttle · S/↓ brake (hold to reverse) · A D steer · Space handbrake<br>C camera · F fix the car · P slow-mo replay of the last crash · T time of day · Esc menu<br>The test centre is south; the ploughed field is east of it.';
}
// ---------------- keys
function onKey(e) {
  if (e.code === 'Escape' && mode !== 'menu') { enter('menu'); return; }
  if (mode === 'lab') {
    const i = +e.key - 1; if (i >= 0 && i < CAMS.length) setCam(CAMS[i]);
    if (e.code === 'Space') labRun(); if (e.code === 'KeyR') labReplay();
    if (e.code === 'Enter' && seq) nextReplay();
  }
  if (mode === 'drive') {
    if (e.code === 'KeyC') driveCam = (driveCam + 1) % 3;
    if (e.code === 'KeyF') { const p = A.world(A.com), f = qrot(A.rot, [0, 0, 1]); A.reset(); A.place([p[0], 0, p[2]], Math.atan2(f[0], f[2]), 0); S.stopReplay(); }
    if (e.code === 'KeyP') { if (S.replay) S.stopReplay(); else if (S.rec.length > 10) { S.recOn = false; if (S.phase === 'crash') S.phase = 'free'; const w = replayWindow(); S.startReplay(.08, w[0]); S.replay.end = w[1]; orbit.d = 6; orbit.pitch = .3; } else msgT = 2; }
    if (e.code === 'KeyT') view.tod = view.tod > .9 ? .25 : view.tod + .15;
  }
  if (mode === 'film' && film) film.key(e);
  if (e.code === 'KeyM') audio?.mute();
}
// ---------------- cameras
function camFor(name, dt) {
  const cp = A.world(A.com), side = S.test?.side;
  const imp = side ? [0, .7, -.3] : S.barrier === 'odb' ? [.3, .7, -1.6] : S.barrier === 'tree' ? [.3, .8, -1.7] : S.useB ? [0, .7, -40] : S.barrier === 'ramp' ? [cp[0], .8, cp[2]] : [0, .7, -1.7];
  const at = (S.phase === 'crash' || S.replay || labState !== 'run') ? lerp3(imp, cp, .5) : cp;
  const C = { fov: .62, near: .05, far: 5000, tgt: at };
  switch (name) {
    case 'side': C.pos = add(at, side ? [6.5, .4, -1] : [-7.5, .35, .2]); C.fov = .5; break;
    case 'front': C.pos = add(at, side ? [3.5, 1.5, -4.5] : [-3.9, 1.3, 1.1]); C.fov = .72; break;
    case 'top': C.pos = add(at, [0, 9, .01]); C.up = [0, 0, 1]; C.fov = .55; break;
    case 'pit': C.pos = [0, -1.0, -2.4]; C.tgt = [0, .6, -1.4]; C.fov = 1.35; break;
    case 'onboard': C.pos = A.world([-.42, 1.17, .45]); C.tgt = A.world([.33, .95, -.05]); C.up = qrot(A.rot, [0, 1, 0]); C.fov = 1.15; break;
    case 'wide': C.pos = add(at, [-13, 3.2, -11]); C.fov = .45; break;
    case 'track': { const v = A.vel, f = len(v) > .5 ? norm(v) : qrot(A.rot, [0, 0, 1]); C.pos = add(cp, add(scl(cross(f, [0, 1, 0]), -5.5), [0, .9, 0])); C.tgt = add(cp, scl(f, 1.5)); C.fov = .65; break; }
    default: { if (!orbit.user) orbit.yaw += dt * .08; orbit.yaw -= mouse.dx * .006; orbit.pitch = clamp(orbit.pitch + mouse.dy * .005, -.15, 1.45); mouse.dx = mouse.dy = 0;
      C.tgt = add(cp, [0, -.1, 0]); C.pos = add(C.tgt, [Math.sin(orbit.yaw) * Math.cos(orbit.pitch) * orbit.d, Math.sin(orbit.pitch) * orbit.d + .3, Math.cos(orbit.yaw) * Math.cos(orbit.pitch) * orbit.d]); }
  }
  return C;
}
const lerp3 = (a, b, t) => [lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t)];
let chase = null;
function driveCamera(dt) {
  const cp = A.world(A.com), f = qrot(A.rot, [0, 0, 1]), fl = norm([f[0], 0, f[2]]);
  if (driveCam === 1) return { pos: A.world([.33, 1.27, -.18]), tgt: A.world([.3, 1.12, 3]), up: qrot(A.rot, [0, 1, 0]), fov: 1.05, near: .03, far: 5000 };
  if (driveCam === 2 || orbit.user) return camFor('orbit', dt);
  const want = add(cp, add(scl(fl, -6.2), [0, 2.1, 0]));
  chase = chase ? lerp3(chase, want, 1 - Math.exp(-dt * 4)) : want;
  return { pos: chase, tgt: add(cp, [0, .6, 0]), fov: .8, near: .05, far: 5000 };
}
// ---------------- api for the film
const api = { S, A, B, D, FX, view, camFor, setCam, orbit, enter, R, labRun: (t, kmh) => { $('test').value = TESTS.indexOf(t); $('kmh').value = kmh; S.start(t, kmh, { runup: 0 }); FX.reset(); }, replayWindow };
window.__ct = { S, A, D, view, enter, seek: t => film && film.seek(t), get mode() { return mode; }, get labState() { return labState; }, labRun, setCam, film: () => film, res: 1 };
// ---------------- render data
function carUnits(C, paint) {
  const { B: Bn, A: An } = carBones(C), p = PAINTS[paint];
  return { car: C, mesh: carM, u: { uB: Bn, uA: An, uHub: WHEELS.flat(), uPaint: p[1], uPaint2: p[2] || [.02, .02, .02], uTwo: p[2] ? 1 : 0, uMarks: mode === 'lab' || film?.marks ? view.marks : 0,
    uLights: view.lights, uBrake: C.brake, uDirt: .35, uRoof: view.roof, uExplode: view.explode, uXray: view.xray, uDum: 0, uCrack: C.crack || 0, uCrackP: C.crackP || [0, 1.2, .5] } };
}
function size() { const d = Math.min(devicePixelRatio || 1, 1.5); cv.width = Math.round(innerWidth * d); cv.height = Math.round(innerHeight * d); ov.width = cv.width; ov.height = cv.height; }
addEventListener('resize', size); size();
let last = performance.now(), res = 1; const ft = [];
enter('menu');
function loop(now) {
  requestAnimationFrame(loop);
  const dt = Math.min((now - last) / 1000, 1 / 20); last = now;
  ft.push(dt); if (ft.length > 40) ft.shift();
  if (!window.__ct.fixed && ft.length === 40) { const avg = ft.reduce((a, b) => a + b) / 40; if (avg > 1 / 40 && res > .55) res -= .02; else if (avg < 1 / 56 && res < 1) res += .01; }
  let cam, post = { exposure: 1, sat: 1.1, grain: .03 }, rp = null, cap = '';
  // ---- update
  if (mode === 'drive') {
    A.throttle = keys.KeyW || keys.ArrowUp ? 1 : 0; const br = keys.KeyS || keys.ArrowDown ? 1 : 0;
    const vf = dot(A.vel, qrot(A.rot, [0, 0, 1]));
    if (br && vf < .5 && A.gear !== 0 && Math.abs(vf) < .5) A.gear = 0;
    if (A.gear === 0) { A.brake = A.throttle; A.throttle = br; if (A.brake && vf > -.3) A.gear = 2; } else A.brake = br;
    A.steerIn = (keys.KeyA || keys.ArrowLeft ? 1 : 0) - (keys.KeyD || keys.ArrowRight ? 1 : 0); A.hand = keys.Space ? 1 : 0;
    S.nearby(A.world(A.com));
    if (S.replay) { rp = S.replayFrame(dt); if (S.replay.i >= S.replay.end) S.replay.i = replayWindow()[0]; cap = 'REPLAY · slow motion ×12 · drag to orbit · P to return'; }
    else S.advance(dt);
    cam = S.replay ? camFor('orbit', dt) : driveCamera(dt);
    if (msgT > 0) { msgT -= dt; cap = 'No crash recorded yet — hit something first.'; }
    else if (driveCrash && S.report && !S.replay) { cap = `Crash at ${Math.round(S.report.g)} g peak — P for a slow-motion replay, F to fix the car`; }
  } else if (mode === 'lab') {
    if (S.replay) { rp = S.replayFrame(dt); if (S.replay.i >= (S.replay.end ?? 1e9) || S.replay.done) nextReplay(); }
    else { S.ts = $('slow').checked && S.phase === 'crash' ? .1 : 1; S.advance(dt); }
    if (labState === 'run' && S.phase === 'crash' && camName === 'track') setCam(S.test.side ? 'front' : 'side');
    cam = camFor(camName, dt);
  } else if (mode === 'film') {
    const o = film.frame(dt); cam = o.cam; post = Object.assign(post, o.post || {}); rp = o.rp; cap = o.cap || '';
    if (o.done) enter('menu');
  } else { // menu: slow orbit around a parked 2CV at golden hour
    orbit.yaw += dt * .05; cam = camFor('orbit', dt); cam.pos[1] = Math.max(cam.pos[1], .4); post.ap = .25; post.focus = len(sub(cam.pos, cam.tgt));
  }
  if (window.__ct.cam) cam = Object.assign({ fov: .6, near: .02, far: 5000 }, window.__ct.cam(A));
  $('cap').textContent = cap; $('cap').style.opacity = cap ? 1 : 0;
  // ---- render
  const L = light(view.tod); L.uCam = cam.pos; L.uFill = cam.pos[1] < 0 ? [2.2, 2.1, 1.9] : [0, 0, 0];
  const cars = [carUnits(A, view.paint)];
  if (S.useB) cars.push(carUnits(B, view.paintB));
  const dShow = S.dummyOn && (mode === 'lab' || mode === 'film');
  if (dShow) cars.push({ car: A, mesh: dumM, u: { uB: dummyBones(D, A.rot, rp ? rp.X : W.x), uA: new Float32Array(48), uDum: 1, uHub: WHEELS.flat(), uPaint: [1, 1, 1], uXray: 0, uExplode: 0, uMarks: 0, uRoof: 0 } });
  const statics = [{ mesh: setM, M: I4 }, { mesh: lidM, M: I4, glass: 1 }];
  const bar = S.barrier;
  statics[0].M = bar === 'odb' ? T4(.15 + BARRIER.w, 0, 0) : bar === 'wall' || bar === 'drive' || !bar ? I4 : T4(0, -30, 0);
  if (bar === 'odb') { const cd = S.colliders.odb.cd, m = cd.reduce((a, b) => a + b, 0) / cd.length; statics.push({ mesh: propM.odb, M: T4(.65, .2, -.54 + m, 1, 1, Math.max(.05, 1 - m / .54)) }); }
  if (bar === 'pole') statics.push({ mesh: propM.pole, M: I4 });
  if (bar === 'tree') statics.push({ mesh: propM.tree, M: T4(.45, 0, 0) });
  if (bar === 'ramp') statics.push({ mesh: propM.ramp, M: I4 });
  statics.push({ mesh: propM.bales, M: I4 });
  const T = rp ? rp.rec.t + 0 : W.t;
  const parts = FX.pack(rp ? S.rec[0].t + rp.t : W.t, W.ground);
  const m = R.frame({ t: now / 1000, res, cam, light: L, cars, statics, focus: A.world(A.com), parts, post });
  hud(m, rp, dt);
  audio?.update({ rpm: A.rpm, throttle: A.throttle, speed: len(A.vel), slip: Math.max(...A.slip) * (A.mode === 'rigid' ? 1 : 0), soft: A.mode === 'soft' && !rp, crash: S.pk || 0, slow: !!rp || S.ts < 1, mode, tow: S.phase === 'tow' });
}
requestAnimationFrame(loop);
// ---------------- HUD
const projP = (VP, p) => { const x = VP[0] * p[0] + VP[4] * p[1] + VP[8] * p[2] + VP[12], y = VP[1] * p[0] + VP[5] * p[1] + VP[9] * p[2] + VP[13], w = VP[3] * p[0] + VP[7] * p[1] + VP[11] * p[2] + VP[15]; return w > 0 ? [(x / w * .5 + .5) * ov.width, (1 - (y / w * .5 + .5)) * ov.height] : null; };
function hud(m, rp, dt) {
  const Wd = ov.width, H = ov.height, s = H / 900; ctx.clearRect(0, 0, Wd, H);
  ctx.font = `${13 * s}px ui-monospace,monospace`; ctx.textAlign = 'left';
  const hs = mode === 'lab' && (S.phase === 'crash' || rp || S.phase === 'done' && seq);
  let txt = '';
  if (mode === 'lab' || (mode === 'film' && film?.hs)) {
    const r0 = S.rec[0], tt = rp ? rp.t : r0 ? W.t - r0.t : 0, imp = S.rec.find(f => f.pulse > 2.5);
    const tImp = imp && r0 ? imp.t - r0.t : .25;
    if (hs || (mode === 'film' && film?.hs)) {
      ctx.fillStyle = 'rgba(255,255,255,.85)'; const x0 = Math.max(Wd * .5 - 160 * s, mode === 'lab' ? 300 * s : 30 * s);
      ctx.fillText(`${rp ? 'HIGH-SPEED REPLAY · 1000 fps' : 'LIVE'} · CAM ${camName.toUpperCase()}`, x0, 30 * s);
      ctx.font = `${26 * s}px ui-monospace,monospace`; ctx.fillText(`T ${tt - tImp >= 0 ? '+' : '−'}${Math.abs((tt - tImp) * 1000).toFixed(1).padStart(6, '0')} ms`, x0, 62 * s);
      graph(Wd - 330 * s, H - 190 * s, 300 * s, 120 * s, s, rp ? rp.i : S.rec.length - 1);
    }
  }
  if (mode === 'drive') {
    const v = Math.abs(dot(A.vel, qrot(A.rot, [0, 0, 1]))) * 3.6;
    ctx.textAlign = 'right'; ctx.fillStyle = '#fff3d6'; ctx.font = `${44 * s}px Georgia,serif`; ctx.fillText(Math.round(v), Wd - 30 * s, H - 60 * s);
    ctx.font = `${13 * s}px ui-monospace,monospace`; ctx.fillStyle = 'rgba(255,240,210,.7)';
    ctx.fillText(`km/h · ${['R', 'N', '1', '2', '3', '4'][A.gear]} · ${Math.round(A.rpm)} rpm`, Wd - 30 * s, H - 36 * s);
    umbrella(Wd - 210 * s, H - 70 * s, s, A.gear);
  }
  $('hud').textContent = txt;
  if (mode === 'film' && film) film.overlay(ctx, m, s, projP);
}
function umbrella(x, y, s, gear) { // the dash-mounted push-pull gear lever, seen from the driver's seat
  const pos = [[-1, 1], [0, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]][gear] || [0, 0];
  ctx.strokeStyle = 'rgba(255,240,210,.35)'; ctx.lineWidth = 1.5 * s; ctx.beginPath();
  ctx.moveTo(x - 18 * s, y - 14 * s); ctx.lineTo(x - 18 * s, y + 14 * s); ctx.moveTo(x + 18 * s, y - 14 * s); ctx.lineTo(x + 18 * s, y + 14 * s); ctx.moveTo(x - 18 * s, y); ctx.lineTo(x + 18 * s, y); ctx.stroke();
  ctx.fillStyle = '#ffcc33'; ctx.beginPath(); ctx.arc(x + pos[0] * 18 * s, y - pos[1] * 14 * s, 5 * s, 0, 7); ctx.fill();
}
function graph(x, y, w, h, s, upto) {
  const r = S.rec; if (r.length < 2) return;
  ctx.fillStyle = 'rgba(8,8,8,.55)'; ctx.fillRect(x - 8 * s, y - 22 * s, w + 16 * s, h + 40 * s);
  const t0 = r[0].t, span = Math.max(.35, r[Math.min(r.length - 1, 400)].t - t0), max = 80;
  ctx.strokeStyle = 'rgba(255,255,255,.15)'; ctx.lineWidth = 1; for (let gg = 0; gg <= max; gg += 20) { const yy = y + h - gg / max * h; ctx.beginPath(); ctx.moveTo(x, yy); ctx.lineTo(x + w, yy); ctx.stroke(); }
  const line = (k, col) => { ctx.strokeStyle = col; ctx.lineWidth = 2 * s; ctx.beginPath(); for (let i = 0; i <= Math.min(upto, r.length - 1); i++) { const xx = x + (r[i].t - t0) / span * w; if (xx > x + w) break; const yy = y + h - Math.min(r[i][k], max) / max * h; i ? ctx.lineTo(xx, yy) : ctx.moveTo(xx, yy); } ctx.stroke(); };
  line('pulse', '#ffcc33'); if (S.dummyOn) { line('head', '#ff6b5b'); line('chest', '#7ec8ff'); }
  ctx.font = `${11 * s}px ui-monospace,monospace`; ctx.textAlign = 'left';
  ctx.fillStyle = '#ffcc33'; ctx.fillText('car', x, y - 8 * s); ctx.fillStyle = '#ff6b5b'; ctx.fillText('head', x + 40 * s, y - 8 * s); ctx.fillStyle = '#7ec8ff'; ctx.fillText('chest', x + 90 * s, y - 8 * s);
  ctx.fillStyle = 'rgba(255,255,255,.6)'; ctx.textAlign = 'right'; ctx.fillText('g', x + w, y - 8 * s); ctx.fillText(`${Math.round(span * 1000)} ms`, x + w, y + h + 14 * s);
}
