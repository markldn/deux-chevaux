// The simulation: one soft-body world holding up to two 2CVs and the dummy, the crash tests, time scaling,
// a 500 Hz recorder for slow-motion replays, and the crash report.
import { World, step, boxCollider, cylCollider, setActive } from './soft.js';
import { createCar, readSoft, goSoft, carBones, SUB } from './vehicle.js';
import { createDummy } from './dummy.js';
import { CLS } from './lattice.js';
import { height, normalAt, BARRIER, TREES, BALES, FAC } from './world.js';
import { add, sub, scl, len, dot, qrot, qconj, clamp } from './math.js';

// the tests. pos/yaw/v place car A; `set` is the barrier setup
export const TESTS = [
  { id: 'wall76', name: '1976 replica · rigid wall', kmh: 40, set: 'wall', ref: true, note: 'German magazine test, 1976: seven small cars into a concrete wall at 40 km/h, 3-point static belts.' },
  { id: 'wall', name: 'Full-width rigid wall', kmh: 56, set: 'wall', note: 'US FMVSS 208 full-width frontal, 35 mph.' },
  { id: 'odb', name: 'Offset deformable barrier · 40 %', kmh: 64, set: 'odb', note: 'Euro NCAP frontal offset (1997–2019): 40 % of the width into aluminium honeycomb.' },
  { id: 'pole', name: 'Side pole', kmh: 32, set: 'pole', side: 1, note: 'Euro NCAP pole: a 254 mm rigid pole at the driver\'s head, car sliding sideways.' },
  { id: 'tree', name: 'Plane tree', kmh: 70, set: 'tree', note: 'The roadside platane of the Route Nationale, hit off-centre.' },
  { id: 'headon', name: '2CV vs 2CV head-on', kmh: 50, set: 'headon', note: 'Two 2CVs, 50 km/h each, 30 cm offset.' },
  { id: 'roll', name: 'Corkscrew ramp rollover', kmh: 65, set: 'ramp', note: 'One side climbs a ramp: the softest suspension in Europe does the rest.' },
];
export const REF76 = { g: 33, crush: 375, hic: 340, head: 85, chest: 23, belt: 676, steer: 41 };
const RAMP = { x0: .3, x1: 1.8, z0: -14, z1: -8.5, h: .95 };

export function createSim() {
  const W = World();
  const S = { W, t: 0, ts: 1, paused: false, rec: [], recOn: false, replay: null, test: null, phase: 'idle', events: [], ramp: false, colliders: {}, msg: '' };
  W.ground = (x, z) => height(x, z) + (S.ramp ? rampH(x, z) : 0);
  W.groundN = (x, z) => S.ramp && rampH(x, z) > 0 ? rampN(x, z) : normalAt(x, z);
  const A = createCar(W, 1, { payload: 78 }), D = createDummy(W, 2, A);
  const B = createCar(W, 3);
  const passengers = [createDummy(W, 4, A, { side: -1, passenger: true, name: 'Front passenger' }), createDummy(W, 5, A, { rear: true, passenger: true, name: 'Rear left' }), createDummy(W, 6, A, { rear: true, side: -1, passenger: true, name: 'Rear right' })];
  const occupants = [D, ...passengers], emptyMass = A.M - 78;
  setActive(W, [3]);
  S.cars = [A, B]; S.A = A; S.B = B; S.D = D; S.occupants = occupants; S.useB = false;
  B.place([0, -50, 400], 0, 0); B.frozen = true;
  // the dummy rides with car A; it goes soft with it
  A.ext.push({ onSoft: () => { S.waitingDummy = S.dummyOn; }, onRigid: () => { S.waitingDummy = false; for (const d of occupants) if (d.enabled) d.capture(); }, near: c => S.useB && B.mode === 'rigid' && (len(c.vel) > .5 || len(B.vel) > .5) && len(sub(c.world(c.com), B.world(B.com))) < 4.4 });
  B.ext.push({ near: c => S.useB && (len(c.vel) > .5 || len(A.vel) > .5) && len(sub(c.world(c.com), A.world(A.com))) < 4.4, onSoft: () => goSoft(A) });
  S.dummyOn = true;
  W.onSub = h => {
    if (S.recOn) { const g = S.pulse(); if (g > S.pk) S.pk = g; S.lastPulse = g; }
    // Releasing the tow cable is not a collision. Keep the seated pose through the approach.
    if (S.waitingDummy && (W.hitBodies[A.body] || S.ramp && len(A.ang) > 1)) {
      S.waitingDummy = false; for (const d of occupants) if (d.enabled) d.release();
    } else if (S.waitingDummy) for (const d of occupants) if (d.enabled) d.follow();
    for (const d of occupants) if (d.enabled && !d.attached) d.sample(h);
  };
  // ---------------- colliders
  const col = S.colliders;
  col.block = boxCollider([0, BARRIER.h / 2, BARRIER.d / 2], [BARRIER.w, BARRIER.h / 2, BARRIER.d / 2], 0, { mu: .45 });
  col.odb = boxCollider([.65, .525, -.27], [.5, .325, .27], 0, { mu: .5, crush: { stress: .34e6, cell: .1, depth: .5, share: 1.5 } });
  col.pole = cylCollider(0, 0, .127, 6, 0, .35);
  col.tree = cylCollider(.45, 0, .36, 8, 0, .5);
  S.setBarrier = kind => {
    W.colliders.length = 0; S.ramp = false; S.barrier = kind;
    if (kind === 'wall') { col.block.c = [0, BARRIER.h / 2, BARRIER.d / 2]; W.colliders.push(col.block); }
    if (kind === 'odb') { col.block.c = [.15 + BARRIER.w, BARRIER.h / 2, BARRIER.d / 2]; col.odb.cd.fill(0); W.colliders.push(col.block, col.odb); }
    if (kind === 'pole') W.colliders.push(col.pole);
    if (kind === 'tree') W.colliders.push(col.tree);
    if (kind === 'ramp') S.ramp = true;
    if (kind === 'drive') { col.block.c = [0, BARRIER.h / 2, BARRIER.d / 2]; W.colliders.push(col.block); }
  };
  // drive mode: register the trees/bales/towers near the car each frame
  const near = [];
  S.nearby = p => {
    if (S.barrier !== 'drive') return;
    W.colliders.length = 1;
    for (const t of TREES) if (Math.abs(t[0] - p[0]) < 25 && Math.abs(t[1] - p[2]) < 25) W.colliders.push(cylCollider(t[0], t[1], t[3] ? .25 : .36 * t[2], 12, height(t[0], t[1]) - .3, .5));
    for (const b of BALES) if (Math.abs(b[0] - p[0]) < 15 && Math.abs(b[1] - p[2]) < 15) W.colliders.push(cylCollider(b[0], b[1], .75, 1.5, 0, .7));
    for (const [x, z] of [[-9, -6], [9, -6], [-9, 4], [9, 4]]) W.colliders.push(cylCollider(x, z, .14, 7, 0, .4));
    W.colliders.push(boxCollider([-30, 4, 25], [12, 4, 18], 0, { mu: .4 }));
  };
  // ---------------- tests
  S.start = (test, kmh, o = {}) => {
    S.test = test; S.kmh = kmh; S.replay = null; S.rec = []; S.recOn = false; S.report = null; S.events = []; S.t = 0;
    A.reset(); W.hitBodies.fill(0); S.dummyOn = o.dummy !== false; S.layout = o.occupants || 'driver'; S.waitingDummy = false; S.lastPulse = 0;
    occupants.forEach((d, i) => {
      d.enabled = S.dummyOn && (i === 0 || S.layout === 'pair' && i === 1 || S.layout === 'family' || S.layout === 'adults');
      d.reseat(S.layout === 'family' && i > 1 ? .68 : 1); d.belt(o.belt !== false && d.enabled);
    });
    const mass = emptyMass + occupants.reduce((m, d) => m + (d.enabled ? 78 * d.scale ** 3 : 0), 0), ratio = mass / A.M;
    A.M = mass; A.I = A.I.map(v => v * ratio); A.F0s = A.F0s.map(v => v * ratio); A.frozen = false;
    S.useB = test.set === 'headon';
    S.setBarrier(test.set);
    const v = kmh / 3.6;
    // run-up: towed along the guide rail, released ~0.25 s before impact so the recorder catches the approach
    const run = o.runup ?? 0;
    if (test.side) { A.place([-.15, 0, -1.3 - .1 - v * .25], -Math.PI / 2, 0); A.vel = [0, 0, v]; S.phase = 'sled'; }
    else if (test.set === 'headon') {
      const approach = run ? 3 : 1.25;
      B.reset(); B.frozen = false; B.place([-.15, 0, 1.95 + v * approach - 40], Math.PI, v); A.place([.15, 0, -1.95 - v * approach - 40], 0, v); S.phase = 'approach';
    } else if (test.set === 'ramp') { A.place([.18, 0, RAMP.z0 - 3 - run], 0, run ? 0 : v); S.phase = run ? 'tow' : 'free'; }
    else { const z0 = -1.95 - .05 - v * .25 - run; A.place([test.set === 'tree' ? 0 : 0, 0, z0], 0, run ? 0 : v); S.phase = run ? 'tow' : 'sled'; }
    if (!S.useB) { B.place([0, -50, 400], 0, 0); B.frozen = true; B.mode = 'rigid'; W.awake[3] = 0; }
    A.tow = run && !test.side && test.set !== 'headon' ? { on: 1, f: 0, x: A.pos[0], v } : null;
    S.target = v;
    pv = null; pf = 0;
    for (const d of occupants) if (d.enabled) d.follow(); else park(d);
    if (S.phase === 'sled') release();
  };
  function park(d) { for (const i of d.ids) { W.x[i * 3 + 1] = W.p[i * 3 + 1] = -60; W.v[i * 3 + 1] = 0; } W.awake[d.body] = 0; }
  function release() {
    S.phase = 'crash'; A.tow = null; S.tRel = S.t;
    goSoft(A); if (S.useB) goSoft(B);
    S.recOn = true; S.rec = []; S.lastRec = -1; S.pk = 0;
  }
  // ---------------- stepping
  S.advance = dt => {
    if (S.paused || S.replay) return;
    const sdt = dt * S.ts;
    S.t += sdt;
    if (S.phase === 'approach') {
      A.vel[2] = S.target; B.vel[2] = -S.target;
      if (len(sub(A.world(A.com), B.world(B.com))) < 4.8) release();
    }
    if (S.phase === 'tow' && A.tow) { // tow winch: speed controller on the sled
      const vf = dot(A.vel, qrot(A.rot, [0, 0, 1]));
      A.tow.f = clamp((S.target - vf) * A.M * 3, -2000, A.M * 4);
      A.throttle = 0; A.brake = 0; A.steerIn = 0;
      const dist = test().set === 'ramp' ? RAMP.z0 - A.pos[2] : -1.95 - A.pos[2];
      if (dist < S.target * .25 + .05 && test().set !== 'ramp') release();
      if (test().set === 'ramp' && dist < 1) { A.tow = null; S.phase = 'free'; }
    }
    const soft = S.cars.some(c => c.mode === 'soft') || !D.attached;
    if (soft) {
      const n = Math.max(1, Math.round(sdt * 60 * SUB)), chunk = 2, h = 1 / (60 * SUB);
      const nmax = Math.min(n, 40); // if the CPU can't keep up, time dilates instead of exploding
      for (let i = 0; i < nmax; i += chunk) {
        const k = Math.min(chunk, nmax - i); step(W, k * h, k);
        if (S.recOn && W.t - S.lastRec >= .00195) record();
      }
    }
    for (const c of S.cars) if (!c.frozen) c.update(sdt, S.t);
    for (const d of occupants) if (d.enabled && d.attached) d.follow(); else if (!d.enabled) park(d);
    // a new recording only for a real impact: a wreck resting against a tree must not wipe the last crash
    if (S.phase === 'free' && A.mode === 'soft' && len(A.vel) > 2.5) { S.phase = 'crash'; S.recOn = true; S.rec = []; S.lastRec = -1; S.tRel = S.t; S.pk = 0; }
    if (S.phase === 'crash') {
      const calm = A.mode === 'rigid' || (W.t - S.rec[0]?.t > 2.5);
      if (S.recOn && (calm || S.rec.length > 1400)) { S.recOn = false; finish(); }
    }
  };
  const test = () => S.test || TESTS[0];
  function record() {
    S.lastRec = W.t;
    const ids = [], X = [];
    const pack = (b, n) => W.x.slice(b * 3, (b + n) * 3);
    S.rec.push({ t: W.t, a: pack(A.base, A.n), b: S.useB ? pack(B.base, B.n) : null, d: pack(D.ids[0], D.ids.length), da: A.dmg.slice(), db: S.useB ? B.dmg.slice() : null,
      others: passengers.map(d => d.enabled ? pack(d.ids[0], d.ids.length) : null),
      spin: A.spin.slice(), pulse: S.lastPulse || 0, impact: !!W.hitBodies[A.body], head: D.fh || 0, chest: D.fc || 0, belt: D.diag.f, crack: A.crack || 0, glass: A.glassN || 0, refA: A.ref, refB: B.ref, c0a: A.fitC0, c0b: B.fitC0 });
  }
  // cabin deceleration (g), filtered
  let pv = null, pf = 0, pt = 0;
  S.pulse = () => {
    const ids = A.L.nodes.filter(n => (n.cls === CLS.FLOOR || n.cls === CLS.RAIL) && n.p[2] < 0 && n.p[2] > -1.4).map(n => n.id);
    S.pulse = () => {
      let v = 0; const f = qrot(A.rot, [0, 0, 1]); for (const i of ids) v += W.v[i * 3] * f[0] + W.v[i * 3 + 1] * f[1] + W.v[i * 3 + 2] * f[2]; v /= ids.length;
      const dt = W.t - pt; let a = 0; if (pv !== null && dt > 0) a = Math.abs(v - pv) / dt / 9.81; pv = v; pt = W.t;
      pf += (a - pf) * Math.min(1, 2 * Math.PI * 100 * dt); return pf;
    };
    return S.pulse();
  };
  function finish() {
    S.phase = 'done';
    const m = D.compute() || {};
    const rec = S.rec, pulse = S.pk || 0;
    // static crush: loss of length along the car's own axis (front-most to rear bumper nodes)
    const L0 = carLength(A, A.restL), L1 = carLength(A, A.loc);
    const sw = A.L.sw - A.base, steer = (A.restL[sw * 3 + 2] - A.loc[sw * 3 + 2]) * 1000;
    let broken = 0, glass = 0; for (let j = 0; j < W.nb; j++) if (!W.alive[j]) { broken++; if (W.bm[j] === 1) glass++; }
    S.report = { test: test(), kmh: S.kmh, g: pulse, crush: (L0 - L1) * 1000, hic: m.hic || 0, head: m.head3 || 0, chest: m.chest3 || 0, belt: (m.belt || 0) / 9.81, lap: (m.lap || 0) / 9.81, steer, broken, glass,
      dummy: S.dummyOn, occupants: occupants.filter(d => d.enabled).map(d => ({ name: d.name, scale: d.scale, ...d.compute() })), intr: cabinIntrusion(A) };
    S.onReport && S.onReport(S.report);
  }
  // ---------------- replay
  S.startReplay = (speed = .05, from = 0) => { if (S.rec.length < 3) return false; S.replay = { i: from, speed, t0: S.rec[0].t }; S.saved = { la: A.loc.slice(), lb: B.loc.slice(), da: A.dmg.slice(), db: B.dmg.slice(), pa: [A.pos, A.rot], pb: [B.pos, B.rot], qa: A.q.slice(), qb: B.q.slice() }; return true; };
  S.stopReplay = () => {
    if (!S.replay) return; S.replay = null; const s = S.saved;
    A.loc.set(s.la); B.loc.set(s.lb); A.dmg.set(s.da); B.dmg.set(s.db); [A.pos, A.rot] = s.pa; [B.pos, B.rot] = s.pb; A.q.set(s.qa); B.q.set(s.qb); A.dirty = B.dirty = true;
  };
  const XR = new Float64Array(W.x.length);
  S.replayFrame = dt => {
    const R = S.replay, rec = S.rec; if (!R) return null;
    const tSpan = rec[rec.length - 1].t - rec[0].t;
    R.i += dt * R.speed / (tSpan / (rec.length - 1));
    if (R.i >= rec.length - 1) { R.i = rec.length - 1; R.done = true; }
    const i0 = Math.floor(R.i), i1 = Math.min(i0 + 1, rec.length - 1), f = R.i - i0, r0 = rec[i0], r1 = rec[i1];
    const lerpInto = (a, b, base) => { for (let k = 0; k < a.length; k++) XR[base * 3 + k] = a[k] + (b[k] - a[k]) * f; };
    lerpInto(r0.a, r1.a, A.base); lerpInto(r0.d, r1.d, D.ids[0]);
    passengers.forEach((d, i) => { if (r0.others?.[i] && r1.others?.[i]) lerpInto(r0.others[i], r1.others[i], d.ids[0]); });
    A.ref = r0.refA; A.fitC0 = r0.c0a; A.crack = r0.crack; readSoft(A, 1 / 60, XR, W.v, r0.da); A.spin = r0.spin;
    if (r0.b) { lerpInto(r0.b, r1.b, B.base); B.ref = r0.refB; B.fitC0 = r0.c0b; readSoft(B, 1 / 60, XR, W.v, r0.db); }
    return { X: XR, t: r0.t + (r1.t - r0.t) * f - rec[0].t, rec: r0, i: R.i };
  };
  return S;
}
function carLength(C, arr) {
  let zf = -9, zr = 9;
  for (const n of C.L.nodes) { if (!n.g || n.cls === CLS.HUB || Math.abs(n.p[0]) > .5) continue; const i = n.id - C.base, z = arr[i * 3 + 2]; if (n.cls === CLS.BUMPER && n.p[2] < 0) zr = Math.min(zr, z); if (z > zf) zf = z; }
  return zf - zr;
}
// how far the footwell/bulkhead came back into the cabin (mm)
function cabinIntrusion(C) {
  let m = 0; for (const n of C.L.nodes) if (n.cls === CLS.BULK) { const i = n.id - C.base; m = Math.max(m, C.restL[i * 3 + 2] - C.loc[i * 3 + 2]); } return m * 1000;
}
export const rampH = (x, z) => x < RAMP.x0 || x > RAMP.x1 || z < RAMP.z0 || z > RAMP.z1 ? 0 : RAMP.h * (z - RAMP.z0) / (RAMP.z1 - RAMP.z0) * Math.min(1, (x - RAMP.x0) / .3);
const rampN = (x, z) => { const s = RAMP.h / (RAMP.z1 - RAMP.z0), l = Math.hypot(s, 1); return [0, 1 / l, -s / l]; };
export { RAMP };
