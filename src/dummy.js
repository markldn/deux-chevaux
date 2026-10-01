// A Hybrid-III-like 50th-percentile male (78 kg) as 17 particles joined by beams, sitting in the driver's seat,
// held by a 3-point static belt (two pulley constraints: lap across the pelvis, diagonal across the chest) and
// gripping the wheel until the load tears his hands off. Measures what the 1976 test measured: head g + HIC15,
// chest g (3 ms), shoulder-belt load.
import { addNode, addBeam, addPulley } from './soft.js';
import { Geo, blob, tube, lathe, frame } from './geo.js';
import { add, sub, scl, norm, cross, dot, len, qrot, qconj, qmul, qm4, mul } from './math.js';
import { CLS } from './lattice.js';

const X = .33;
// [name, local pos, mass, radius]
const PTS = [
  ['pelvis', [X, .5, -.1], 17, .14], ['abdomen', [X, .73, -.17], 8, .13], ['chest', [X, .96, -.2], 17, .14],
  ['shL', [X + .19, 1.1, -.2], 3, .06], ['shR', [X - .19, 1.1, -.2], 3, .06], ['neck', [X, 1.18, -.19], 1.2, .05], ['head', [X, 1.31, -.15], 4.5, .1],
  ['knL', [X + .1, .6, .3], 5, .07], ['knR', [X - .1, .6, .3], 5, .07], ['anL', [X + .12, .37, .5], 3, .055], ['anR', [X - .12, .37, .5], 3, .055],
  ['elL', [X + .24, .94, .02], 2, .05], ['elR', [X - .24, .94, .02], 2, .05], ['haL', [X + .17, 1.05, .2], .6, .045], ['haR', [X - .17, 1.05, .2], .6, .045],
];
const IDX = Object.fromEntries(PTS.map((p, i) => [p[0], i]));
const RIGID = { k: 4e6, fyc: 1e7, fyt: 1e7, brk: 9, damp: .9 };
const SPINE = { k: 1.5e5, fyc: 1e7, fyt: 1e7, brk: 9, damp: .9 };
const NECK = { k: 2.5e4, fyc: 1e7, fyt: 1e7, brk: 9, damp: 1 };
const SEAT = { k: 4e4, fyc: 6000, fyt: 2500, brk: 9, damp: 1 };
const GRIP = { k: 2e5, fyc: 600, fyt: 450, brk: .04, damp: .8 };
const LINKS = [['pelvis', 'abdomen', SPINE], ['abdomen', 'chest', SPINE], ['pelvis', 'chest', SPINE], ['chest', 'shL', RIGID], ['chest', 'shR', RIGID], ['shL', 'shR', RIGID],
  ['chest', 'neck', RIGID], ['shL', 'neck', RIGID], ['shR', 'neck', RIGID], ['neck', 'head', RIGID], ['chest', 'head', NECK],
  ['pelvis', 'knL', RIGID], ['pelvis', 'knR', RIGID], ['knL', 'anL', RIGID], ['knR', 'anR', RIGID], ['abdomen', 'knL', SPINE], ['abdomen', 'knR', SPINE],
  ['shL', 'elL', RIGID], ['shR', 'elR', RIGID], ['elL', 'haL', RIGID], ['elR', 'haR', RIGID]];

export function createDummy(W, body, car) {
  const D = { W, body, car, ids: [], local: PTS.map(p => p[1].slice()), attached: true, rec: [], grip: [], metrics: null };
  const total = PTS.reduce((s, p) => s + p[2], 0);
  for (const [n, p, m, r] of PTS) { const i = addNode(W, p, m * 78 / total, r, body, .5); W.flag[i] |= 4; D.ids.push(i); }
  for (const [a, b, m] of LINKS) addBeam(W, D.ids[IDX[a]], D.ids[IDX[b]], m, 3);
  const L = car.L, N = L.nodes, nid = (p, f) => N.filter(f).sort((a, b) => dist(a.p, p) - dist(b.p, p))[0].id;
  const floor = o => o.cls === CLS.FLOOR || o.cls === CLS.RAIL;
  // seat: cushion springs to the platform under the pelvis and thighs
  for (const p of [[X + .2, .25, -.05], [X - .2, .25, -.05], [X + .2, .25, -.35], [X - .2, .25, -.35]]) addBeam(W, D.ids[IDX.pelvis], nid(p, floor), SEAT, 3);
  for (const p of [[X + .1, .25, .25], [X - .1, .25, .25]]) addBeam(W, D.ids[IDX[p[0] > X ? 'knL' : 'knR']], nid(p, floor), { ...SEAT, fyt: 300, fyc: 3000 }, 3);
  // hands on the rim
  for (const h of ['haL', 'haR']) D.grip.push(addBeam(W, D.ids[IDX[h]], car.L.sw, GRIP, 3));
  // 3-point static belt: anchors on the sill, the tunnel side of the floor and high on the B-pillar
  const sill = nid([.62, .32, -.45], o => o.cls === CLS.RAIL || o.cls === CLS.FLOOR || o.cls === CLS.SHEET);
  const E = (typeof process !== 'undefined' && process.env) || {};
  const buckle = nid([.1, .25, +(E.BUZ || -.3)], floor);
  const upper = nid([.68, 1.36, +(E.UPZ || -.4)], o => o.cls === CLS.SHEET && o.p[0] > .6);
  const pillar = [upper, ...N.filter(o => o.g && o.p[0] > .6 && Math.abs(o.p[2] + .35) < .1 && o.p[1] > .3 && o.p[1] < 1.42 && o.cls !== CLS.GLASS && o.id !== upper).map(o => o.id)];
  const cl = id => { const p = [W.x[id * 3], W.x[id * 3 + 1], W.x[id * 3 + 2]]; return N.filter(o => o.g && o.cls !== CLS.GLASS && o.cls !== CLS.CANVAS && dist(o.p, p) < .25).map(o => o.id).sort((a, b) => (a !== id) - (b !== id)); };
  D.lap = addPulley(W, cl(sill), D.ids[IDX.pelvis], cl(buckle), .01, +(E.LAPK || 2e5));
  D.diag = addPulley(W, pillar, [D.ids[IDX.shL], D.ids[IDX.chest]], cl(buckle), +(E.SLACK || .005), +(E.BELTK || 1.5e5));
  D.anchors = [sill, buckle, upper];
  W.r[car.L.sw] = +(E.SWR || .05);
  D.belt = on => { D.lap.on = D.diag.on = on ? 1 : 0; };
  // ride along rigidly while the car is rigid
  D.follow = () => {
    for (let i = 0; i < D.ids.length; i++) {
      const p = car.world(D.local[i]), k = D.ids[i] * 3;
      W.x[k] = W.p[k] = p[0]; W.x[k + 1] = W.p[k + 1] = p[1]; W.x[k + 2] = W.p[k + 2] = p[2];
      const v = add(car.vel, cross(car.ang, sub(p, car.world(car.com)))); W.v[k] = v[0]; W.v[k + 1] = v[1]; W.v[k + 2] = v[2];
    }
  };
  D.release = () => { D.attached = false; W.awake[body] = 1; D.rec = []; D.prevV = null; D.metrics = null; };
  D.capture = () => { // store the current pose relative to the car (after a crash, he stays slumped)
    const iq = qconj(car.rot);
    D.local = D.ids.map(i => qrot(iq, sub([W.x[i * 3], W.x[i * 3 + 1], W.x[i * 3 + 2]], car.pos)));
    D.attached = true; W.awake[body] = 0;
  };
  D.reseat = () => { D.local = PTS.map(p => p[1].slice()); D.attached = true; W.awake[body] = 0; for (const j of D.grip) { W.alive[j] = 1; W.L0[j] = W.Lr[j]; } D.lap.fmax = D.diag.fmax = 0; D.metrics = null; D.rec = []; };
  // per-substep sampling of head and chest acceleration (called by the stepper)
  D.sample = h => {
    const v = k => [W.v[D.ids[IDX[k]] * 3], W.v[D.ids[IDX[k]] * 3 + 1], W.v[D.ids[IDX[k]] * 3 + 2]];
    const hv = v('head'), cv = v('chest');
    if (D.prevV) {
      const ah = len(sub(hv, D.prevV[0])) / h / 9.81, ac = len(sub(cv, D.prevV[1])) / h / 9.81;
      D.fh = D.fh === undefined ? ah : D.fh + (ah - D.fh) * .55; D.fc = D.fc === undefined ? ac : D.fc + (ac - D.fc) * .35;
      D.rec.push([W.t, D.fh, D.fc, D.diag.f, D.lap.f]);
    }
    D.prevV = [hv, cv];
  };
  D.compute = () => {
    const r = D.rec; if (r.length < 10) return null;
    const dt = (r[r.length - 1][0] - r[0][0]) / (r.length - 1);
    // HIC15: max over windows <= 15 ms of (t2-t1) * mean(a)^2.5
    const cum = [0]; for (const s of r) cum.push(cum[cum.length - 1] + s[1] * dt);
    let hic = 0; const wmax = Math.round(.015 / dt);
    for (let i = 0; i < r.length; i++) for (let j = i + 2; j <= Math.min(r.length, i + wmax); j++) { const T = (j - i) * dt, a = (cum[j] - cum[i]) / T; const v = T * Math.pow(a, 2.5); if (v > hic) hic = v; }
    // 3 ms clip: highest level sustained for 3 ms
    const w3 = Math.max(1, Math.round(.003 / dt)); let c3 = 0, h3 = 0;
    for (let i = 0; i + w3 <= r.length; i++) { let mc = 1e9, mh = 1e9; for (let j = i; j < i + w3; j++) { mc = Math.min(mc, r[j][2]); mh = Math.min(mh, r[j][1]); } c3 = Math.max(c3, mc); h3 = Math.max(h3, mh); }
    const belt = Math.max(...r.map(s => s[3])), lap = Math.max(...r.map(s => s[4]));
    return D.metrics = { hic, head3: h3, chest3: c3, belt, lap, headPeak: Math.max(...r.map(s => s[1])) };
  };
  return D;
}
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

// ---- dummy mesh: segments in the seated rest pose, each on its own bone
// bones: 0 head, 1 neck, 2 chest, 3 abdomen, 4 pelvis, 5/6 thigh, 7/8 shin, 9/10 foot, 11/12 upper arm, 13/14 forearm, 15/16 hand
export const SEGS = [['head', 'neck', 'head'], ['neck', 'chest', 'neck'], ['chest', 'abdomen', 'chest'], ['abdomen', 'pelvis', 'abdomen'], ['pelvis', 'abdomen', 'pelvis'],
  ['pelvis', 'knL', 'knL'], ['pelvis', 'knR', 'knR'], ['knL', 'anL', 'anL'], ['knR', 'anR', 'anR'], ['anL', 'knL', 'anL'], ['anR', 'knR', 'anR'],
  ['shL', 'elL', 'elL'], ['shR', 'elR', 'elR'], ['elL', 'haL', 'haL'], ['elR', 'haR', 'haR'], ['haL', 'elL', 'haL'], ['haR', 'elR', 'haR']];
export function buildDummyMesh() {
  const g = Geo(), P = n => PTS[IDX[n]][1], Y = [1, .78, .1, .45], K = [.05, .05, .05, .55], o = (b, col = Y) => ({ mat: 11, col, bone: b, part: 5 });
  // head with the quadrant targets painted by the shader (mat 11 + K.w = 1)
  blob(g, P('head'), [.075, .1, .09], 2.2, { ...o(0), k: () => [9, 9, 9, 1] });
  blob(g, add(P('head'), [0, -.05, .07]), [.04, .03, .03], 2.5, o(0));                         // chin
  tube(g, [P('neck'), add(P('neck'), [0, .08, .02])], .04, 10, o(1, K));
  blob(g, add(P('chest'), [0, .04, 0]), [.19, .17, .12], 3, o(2, [.12, .12, .13, .6]));              // jacket
  blob(g, P('abdomen'), [.15, .1, .1], 2.6, o(3, K));
  blob(g, P('pelvis'), [.18, .1, .14], 2.6, o(4));
  const limb = (a, b, r0, r1, bone, col = Y) => tube(g, [P(a), add(P(a), scl(sub(P(b), P(a)), .5)), P(b)], u => r0 + (r1 - r0) * u, 12, o(bone, col));
  limb('pelvis', 'knL', .085, .06, 5); limb('pelvis', 'knR', .085, .06, 6);
  limb('knL', 'anL', .055, .04, 7); limb('knR', 'anR', .055, .04, 8);
  blob(g, add(P('anL'), [0, -.03, .07]), [.05, .04, .12], 3, o(9, K)); blob(g, add(P('anR'), [0, -.03, .07]), [.05, .04, .12], 3, o(10, K));
  limb('shL', 'elL', .05, .042, 11); limb('shR', 'elR', .05, .042, 12);
  limb('elL', 'haL', .04, .035, 13); limb('elR', 'haR', .04, .035, 14);
  blob(g, P('haL'), [.035, .045, .05], 2.5, o(15)); blob(g, P('haR'), [.035, .045, .05], 2.5, o(16));
  for (const n of ['knL', 'knR', 'elL', 'elR']) blob(g, P(n), [.05, .05, .05], 2, o(SEGS.findIndex(s => s[2] === n), K));
  // the belt: five straps on stretchable bones 17-21
  const BR = beltRest(P);
  BELT.forEach(([a, b], i) => tube(g, [BR[a], add(BR[a], scl(sub(BR[b], BR[a]), .5)), BR[b]], .014, 6, { mat: 5, col: [.16, .16, .17, .6], bone: 17 + i, part: 5 }));
  for (let i = 0; i < g.M.length / 4; i++) g.M[i * 4 + 2] = .8;
  return { g, J: new Float32Array(g.P.length / 3 * 4), Wt: new Float32Array(g.P.length / 3 * 4) };
}
const BELT = [['up', 'sh'], ['sh', 'ch'], ['ch', 'bu'], ['si', 'pe'], ['pe', 'bu']];
function beltRest(P) { return { up: [.68, 1.36, -.4], sh: add(P('shL'), [-.02, .06, .02]), ch: add(P('chest'), [0, 0, .135]), bu: [.1, .33, -.28], si: [.62, .34, -.45], pe: add(P('pelvis'), [0, .03, .15]) }; }
// bone matrices from the particles: each segment's frame now vs in the rest pose
export function dummyBones(D, carRot, X = D.W.x) {
  const P = n => { const i = D.ids[IDX[n]] * 3; return [X[i], X[i + 1], X[i + 2]]; }, R0 = n => PTS[IDX[n]][1];
  const B = new Float32Array(24 * 16), fwd = qrot(carRot, [0, 0, 1]);
  const fr = (a, b, side) => { const y = norm(sub(b, a)); let x = norm(cross(side, y)); if (!isFinite(x[0])) x = [1, 0, 0]; const z = cross(x, y); return [x, y, z]; };
  const m4 = (f, o) => new Float32Array([...f[0], 0, ...f[1], 0, ...f[2], 0, ...o, 1]);
  const inv = (f, o) => { const t = [-dot(f[0], o), -dot(f[1], o), -dot(f[2], o)]; return new Float32Array([f[0][0], f[1][0], f[2][0], 0, f[0][1], f[1][1], f[2][1], 0, f[0][2], f[1][2], f[2][2], 0, ...t, 1]); };
  SEGS.forEach(([a, b, o], i) => {
    const sideNow = norm(sub(P('shL'), P('shR'))), sideRest = norm(sub(R0('shL'), R0('shR')));
    const sn = i >= 5 && i <= 10 ? norm(sub(P('knL'), P('knR'))) : sideNow, sr = i >= 5 && i <= 10 ? norm(sub(R0('knL'), R0('knR'))) : sideRest;
    const fN = fr(P(a), P(b), sn), fR = fr(R0(a), R0(b), sr);
    B.set(mul(m4(fN, P(o)), inv(fR, R0(o))), i * 16);
  });
  // belt straps: stretch each rest segment onto its current end points
  const fw = n => norm(cross(sub(n('shL'), n('shR')), sub(n('neck'), n('pelvis'))));
  const an = i => [X[D.anchors[i] * 3], X[D.anchors[i] * 3 + 1], X[D.anchors[i] * 3 + 2]], fN = fw(P);
  const cur = { up: an(2), si: an(0), bu: an(1), sh: add(add(P('shL'), scl(fN, .02)), [0, .06, 0]), ch: add(P('chest'), scl(fN, .135)), pe: add(add(P('pelvis'), scl(fN, .15)), [0, .03, 0]) };
  const rest = beltRest(R0);
  BELT.forEach(([a, b], i) => {
    const seg = (p, q) => { const u = sub(q, p), l = len(u) || 1e-6, un = scl(u, 1 / l); let v = norm(cross(un, [0, 1, 0])); if (!isFinite(v[0]) || len(v) < .5) v = [1, 0, 0]; return [un, v, cross(un, v), l]; };
    const [u0, v0, w0, l0] = seg(rest[a], rest[b]), [u1, v1, w1, l1] = seg(cur[a], cur[b]);
    B.set(mul(m4([scl(u1, l1 / l0), v1, w1], cur[a]), inv([u0, v0, w0], rest[a])), (17 + i) * 16);
  });
  return B;
}
