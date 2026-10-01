// The 2CV's structure as a node/beam lattice on a 15 cm grid, classified from the same section functions that
// build the body mesh: thin shell, platform + side members, engine/gearbox block, bulkhead, dash rail, bumpers,
// glass (brittle), canvas roof (tension only), wheel hubs on hinged arms. Plus the vertex -> node skin binding.
import { section, capXform, ZF, ZR, FW, RW, archF, winSdf, canvasSdf, WHEELS, PIVOT, SWHEEL, SEAT, FZ, P_ENGINE, P_INT, P_WHEEL, P_SUSP } from './car.js';
import { addNode, addBeam } from './soft.js';
import { add, scl, norm, cross } from './math.js';

export const GH = .15;
// calibrated: 1976 rigid wall 40 km/h -> ~34 g, ~400 mm static crush (measured 33 g, 375 mm)
export let BEND = .6; export const setBend = b => BEND = b;
// one knob for the whole structure's strength, set by test/calib.mjs against the 1976 test
export function scaleYield(f) { for (const k of ['SHEET', 'FLOOR', 'RAIL', 'MOUNT', 'BUMPER', 'BULK', 'STEER', 'HUBB']) { MATS[k].fyc *= f; MATS[k].fyt *= f; } }
// materials: k (N/m), compression / tension yield (N), break strain, damping
export const MATS = {
  SHEET: { k: 3e6, fyc: 1500, fyt: 4500, brk: .6, damp: .6 },
  FLOOR: { k: 6e6, fyc: 4200, fyt: 9000, brk: .8, damp: .6 },
  RAIL: { k: 9e6, fyc: 7000, fyt: 14000, brk: .8, damp: .6 },
  ENG: { k: 3e7, fyc: 4e5, fyt: 4e5, brk: 9, damp: .8 },
  MOUNT: { k: 8e6, fyc: 9000, fyt: 9000, brk: .7, damp: .6 },
  GLASS: { k: 4e6, fyc: 1e6, fyt: 1e6, brk: .03, damp: .6 },
  CANVAS: { k: 6e4, fyc: 30, fyt: 3000, brk: .6, damp: .3 },
  BUMPER: { k: 5e6, fyc: 3000, fyt: 6000, brk: .6, damp: .6 },
  BULK: { k: 6e6, fyc: 3500, fyt: 7000, brk: .7, damp: .6 },
  ARM: { k: 2e7, fyc: 25000, fyt: 25000, brk: .5, damp: .8 },
  HUBB: { k: 8e6, fyc: 900 * (+(globalThis.process?.env?.HB) || 3), fyt: 1800 * (+(globalThis.process?.env?.HB) || 3), brk: 1.5, damp: .8 },
  SPRING: { k: 1.5e5, fyc: 1e9, fyt: 1e9, brk: 9, damp: .8 },
  STEER: { k: 6e6, fyc: 5000, fyt: 9000, brk: .8, damp: .7 },
};
scaleYield(3.33 * (+(globalThis.process?.env?.YS) || 1.8));
const C = { SHEET: 0, FLOOR: 1, RAIL: 2, ENG: 3, GLASS: 4, CANVAS: 5, BUMPER: 6, BULK: 7, DASH: 8, WING: 9, HUB: 10, PIVOT: 11, STEER: 12, SEAT: 13, RIM: 14 };
export const CLS = C;
const MASSW = [1, 1.6, 2.2, 0, 1.1, .35, 1.6, 1.4, 1.2, .8, 0, 1, 0, 0, 0];

// distance from (x,y) to an open polyline [[x,y,s]...], also returning s of the closest point
function polyDist(pl, x, y) {
  let best = 1e9, bs = 0;
  for (let i = 1; i < pl.length; i++) {
    const a = pl[i - 1], b = pl[i], ex = b[0] - a[0], ey = b[1] - a[1], l2 = ex * ex + ey * ey || 1e-12;
    const t = Math.max(0, Math.min(1, ((x - a[0]) * ex + (y - a[1]) * ey) / l2)), dx = x - a[0] - ex * t, dy = y - a[1] - ey * t, d = dx * dx + dy * dy;
    if (d < best) { best = d; bs = a[2] + (b[2] - a[2]) * t; }
  }
  return [Math.sqrt(best), bs];
}
// fine z samples of the shell (including the collapsing caps) for 3D distance queries
const SHELL = [];
for (let z = ZF; z >= ZR - 1e-6; z -= .01) {
  SHELL.push({ z, pl: section(z, 80).map(p => [...capXform(z, p[0], p[1]), p[2]]) });
}
function shellDist(x, y, z) {
  let best = 1e9, bs = 0;
  for (const S of SHELL) { const dz = S.z - z; if (dz * dz > best * best || Math.abs(dz) > .2) continue; const [d, s] = polyDist(S.pl, x, y); const dd = Math.hypot(d, dz); if (dd < best) { best = dd; bs = s; } }
  return [best, bs];
}
function wingDist(x, y, z) {
  const ax = Math.abs(x); let best = 9;
  for (const [W, z0, z1, bot] of [[FW, .55, 1.875, z => Math.max(.4, archF(z))], [RW, -1.86, -.9, () => .335]]) {
    if (z < z0 - .1 || z > z1 + .1) continue;
    const zc = Math.max(z0, Math.min(z1, z)), [yT, xi, xo] = W(zc), xc = (xi + xo) / 2, hw = (xo - xi) / 2;
    const pl = []; for (let i = 0; i <= 16; i++) { const ph = i / 16 * Math.PI, yb = i < 8 ? Math.min(yT, .45) : Math.min(yT, bot(zc)); pl.push([xc - hw * Math.sign(Math.cos(ph)) * Math.pow(Math.abs(Math.cos(ph)), .7), yb + (yT - yb) * Math.pow(Math.sin(ph), .45), 0]); }
    best = Math.min(best, Math.hypot(polyDist(pl, ax, y)[0], z - zc));
  }
  return best;
}

export let CAB = 2.6; export const setCab = c => CAB = c;
// the platform and cabin are stronger than the engine bay: that's what makes the front the crumple zone
const zone = z => z > .78 ? 1 : z < .42 ? CAB : 1 + (CAB - 1) * (.78 - z) / .36;
function zoned(W, j, m) { const z = (W.x[W.ba[j] * 3 + 2] + W.x[W.bb[j] * 3 + 2]) / 2, f = zone(z) * (m.fyt > 1e5 ? 0 : 1); if (f) { W.fyc[j] *= f; W.fyt[j] *= f; } return j; }
export function buildLattice(W, body, totalMass = 600) {
  const nodes = [], idx = new Map(), key = (i, j, k) => i + ',' + j + ',' + k;
  const X = i => -.675 + i * GH, Y = j => .25 + j * GH, Z = k => 1.95 - k * GH;
  for (let k = 0; k <= 26; k++) for (let j = 0; j <= 9; j++) for (let i = 0; i <= 9; i++) {
    const x = X(i), y = Y(j), z = Z(k), ax = Math.abs(x);
    let cls = -1;
    const eng = ax < .33 && y > .38 && y < .7 && z > 1.0 && z < 1.7;
    const floor = j === 0 && ax < .66 && z > -1.85 && z < 1.7;
    const bump = j === 1 && (k === 0 || k === 26);
    const bulk = Math.abs(z - .6) < .01 && y < 1.0 && ax < .66;
    const dash = Math.abs(z - .45) < .01 && Math.abs(y - 1.0) < .01;
    const [ds, s] = shellDist(x, y, z), dw = wingDist(x, y, z);
    if (eng) cls = C.ENG;
    else if (floor) cls = ax > .3 && ax < .6 ? C.RAIL : C.FLOOR;
    else if (bump) cls = C.BUMPER;
    else if (bulk) cls = C.BULK;
    else if (dash) cls = C.DASH;
    else if (ds < .085) cls = winSdf(x, y, z, s) < -.02 ? C.GLASS : canvasSdf(x, y, z, s) < -.03 && y > 1.2 ? C.CANVAS : C.SHEET;
    else if (dw < .08) cls = C.WING;
    if (cls < 0) continue;
    idx.set(key(i, j, k), nodes.length); nodes.push({ p: [x, y, z], cls, g: [i, j, k] });
  }
  // explicit nodes: arm pivots (two per arm = a hinge), hubs, steering wheel + column
  const extra = (p, cls) => { nodes.push({ p, cls, g: null }); return nodes.length - 1; };
  const piv = PIVOT.map(p => [extra(p, C.PIVOT), extra([p[0] * .45, p[1], p[2]], C.PIVOT)]);
  const hubs = WHEELS.map(h => extra(h.slice(), C.HUB));
  const sw = extra(SWHEEL.c.slice(), C.STEER), col = extra([.33, .78, .52], C.STEER);
  // interior collision structure: seat-back frames (the dummy rebounds into them) and the steering-wheel rim
  const seats = SEAT.map(([x, y, z]) => { const b = z - .3; return [[x + .13, .62, b - .05], [x - .13, .62, b - .05], [x + .13, .86, b - .1], [x - .13, .86, b - .1]].map(p => extra(p, C.SEAT)); });
  const sa = norm(SWHEEL.n), sb = norm(cross(sa, [1, 0, 0])), sc = cross(sa, sb);
  const rim = [0, 1, 2, 3].map(k => extra(add(SWHEEL.c, add(scl(sb, Math.cos(k * Math.PI / 2) * SWHEEL.r), scl(sc, Math.sin(k * Math.PI / 2) * SWHEEL.r))), C.RIM));
  // mass
  let wsum = 0; for (const n of nodes) wsum += MASSW[n.cls];
  const engN = nodes.filter(n => n.cls === C.ENG).length, rest = totalMass - 105 - 4 * 15 - 8;
  const base = W.n;
  for (const n of nodes) {
    const m = n.cls === C.ENG ? 105 / engN : n.cls === C.HUB ? 15 : n.cls === C.STEER ? 4 : n.cls === C.SEAT ? 1.5 : n.cls === C.RIM ? .4 : MASSW[n.cls] / wsum * rest;
    n.id = addNode(W, n.p, m, n.cls === C.HUB ? .2925 : n.cls === C.RIM ? .025 : .07, body, n.cls === C.HUB ? 1.0 : .45);
    if (n.cls === C.RIM || n.cls === C.SEAT) W.flag[n.id] |= 8;
    if (n.cls === C.HUB) W.flag[n.id] |= 2;
    if (n.cls === C.HUB || n.cls === C.ENG || n.cls === C.STEER) W.flag[n.id] |= 4;
  }
  // beams between grid neighbours (26-neighbourhood)
  const mat = (a, b) => {
    const s = new Set([a, b]);
    if (s.has(C.GLASS)) return 'GLASS'; if (s.has(C.CANVAS)) return 'CANVAS';
    if (a === C.ENG && b === C.ENG) return 'ENG'; if (s.has(C.ENG)) return 'MOUNT';
    if (a === C.RAIL && b === C.RAIL) return 'RAIL'; if (s.has(C.RAIL) || s.has(C.FLOOR)) return 'FLOOR';
    if (s.has(C.BUMPER)) return 'BUMPER'; if (s.has(C.BULK) || s.has(C.DASH)) return 'BULK';
    return 'SHEET';
  };
  const beams = [];
  for (const n of nodes) {
    if (!n.g) continue; const [i, j, k] = n.g;
    for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
      const m = idx.get(key(i + a, j + b, k + c)); if (m === undefined) continue; const o = nodes[m];
      if (o.id <= n.id) continue;
      { const M = MATS[mat(n.cls, o.cls)]; beams.push(zoned(W, addBeam(W, n.id, o.id, M, n.cls === C.GLASS || o.cls === C.GLASS ? 1 : 0), M)); }
    }
  }
  // bending beams: straight second neighbours through an existing middle node resist the folding mechanism
  for (const n of nodes) {
    if (!n.g) continue; const [i, j, k] = n.g;
    for (const [a, b, c] of [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, -1, 0], [0, 1, 1], [0, 1, -1], [1, 0, 1], [1, 0, -1]]) {
      const m = idx.get(key(i + a, j + b, k + c)), o2 = idx.get(key(i + 2 * a, j + 2 * b, k + 2 * c));
      if (m === undefined || o2 === undefined) continue; const o = nodes[o2];
      const M = mat(n.cls, o.cls); if (M === 'GLASS' || M === 'CANVAS') continue;
      { const B = { ...MATS[M], k: MATS[M].k * .5, fyc: MATS[M].fyc * BEND, fyt: MATS[M].fyt * BEND }; beams.push(zoned(W, addBeam(W, n.id, o.id, B, 2), B)); }
    }
  }
  const dd = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  const near = (p, r, f = () => true) => nodes.filter(o => o.g && f(o) && Math.hypot(o.p[0] - p[0], o.p[1] - p[1], o.p[2] - p[2]) < r);
  // pivots welded to the platform, hubs hinged on them, a preloaded spring to the body above each wheel
  piv.forEach(([a, b], i) => {
    for (const q of [a, b]) for (const o of near(nodes[q].p, .3, o => o.cls === C.FLOOR || o.cls === C.RAIL)) addBeam(W, nodes[q].id, o.id, MATS.RAIL);
    addBeam(W, nodes[a].id, nodes[b].id, MATS.ARM);
    const hb = nodes[hubs[i]].id;
    addBeam(W, hb, nodes[a].id, MATS.ARM); addBeam(W, hb, nodes[b].id, MATS.ARM);
    // brace the hub to the platform: in the soft model the arm is a stiff (but yieldable) bracket, so a wheel
    // stays where the suspension had it unless a crash bends it
    for (const o of near(WHEELS[i], .42, o => o.cls === C.RAIL || o.cls === C.FLOOR || o.cls === C.ENG).sort((p, q) => dd(p.p, WHEELS[i]) - dd(q.p, WHEELS[i])).slice(0, 5)) addBeam(W, hb, o.id, MATS.HUBB);
    for (const o of near(WHEELS[i], .6, o => o.cls !== C.GLASS && o.cls !== C.CANVAS && o.cls !== C.WING && o.p[1] > WHEELS[i][1] + .2).sort((p, q) => dd(p.p, WHEELS[i]) - dd(q.p, WHEELS[i])).slice(0, 9)) addBeam(W, hb, o.id, MATS.HUBB);
  });
  // steering wheel on its column, column on the bulkhead and dash rail
  addBeam(W, nodes[sw].id, nodes[col].id, MATS.STEER);
  for (const o of near(SWHEEL.c, .36, o => o.cls === C.DASH || o.cls === C.SHEET)) addBeam(W, nodes[sw].id, o.id, MATS.STEER);
  for (const o of near(nodes[col].p, .32, o => o.cls === C.BULK || o.cls === C.DASH || o.cls === C.FLOOR)) addBeam(W, nodes[col].id, o.id, MATS.STEER);
  // the column shaft runs down to the rack in the front axle cross-tube: front crush pushes it back
  const rack = near([.225, .4, 1.05], .2, o => o.cls === C.ENG)[0];
  if (rack) addBeam(W, nodes[col].id, rack.id, { ...MATS.STEER, k: 4e6, fyc: 2500 * (+(globalThis.process?.env?.RACK) || 1), fyt: 9000 });
  // seat frames on the platform, rim on the hub
  const SEATB = { k: 3e6, fyc: 2500, fyt: 4000, brk: .8, damp: .8 };
  for (const g of seats) { for (const a of g) { for (const b of g) if (a < b) addBeam(W, nodes[a].id, nodes[b].id, SEATB);
    for (const o of near(nodes[a].p, .7, o => o.cls === C.FLOOR || o.cls === C.RAIL).sort((p, q) => dd(p.p, nodes[a].p) - dd(q.p, nodes[a].p)).slice(0, 3)) addBeam(W, nodes[a].id, o.id, SEATB); } }
  // the rim bends when the chest or arms load it (a single-spoke 2CV wheel folds easily)
  const RIMB = { k: 1e6, fyc: 700 * (+(globalThis.process?.env?.RIMY) || 1), fyt: 1400 * (+(globalThis.process?.env?.RIMY) || 1), brk: 1.5, damp: .8 };
  rim.forEach((r, k) => { addBeam(W, nodes[r].id, nodes[sw].id, RIMB); addBeam(W, nodes[r].id, nodes[rim[(k + 1) % 4]].id, RIMB); addBeam(W, nodes[r].id, nodes[col].id, RIMB); });
  // adjacency for node frames
  const adj = Array.from({ length: W.n }, () => []);
  for (const j of beams) { adj[W.ba[j]].push(W.bb[j]); adj[W.bb[j]].push(W.ba[j]); }
  for (let j = beams[beams.length - 1] + 1; j < W.nb; j++) { adj[W.ba[j]].push(W.bb[j]); adj[W.bb[j]].push(W.ba[j]); }
  const seatMounts = [[.33, .25, -.12], [-.33, .25, -.12], [0, .25, -.85]].map(p => nodes.filter(o => o.cls === C.FLOOR || o.cls === C.RAIL).sort((a, b) => dd(a.p, p) - dd(b.p, p)).slice(0, 4).map(o => o.id - base));
  return { nodes, idx, base, count: nodes.length, hubs: hubs.map(h => nodes[h].id), sw: nodes[sw].id, col: nodes[col].id, adj, near, seatMounts, piv: piv.map(p => [nodes[p[0]].id, nodes[p[1]].id]) };
}
// skin binding: 4 nodes + weights per vertex. Wheels/arms/driveshafts are driven by bones (weight 0).
export function bindSkin(g, L) {
  const n = g.P.length / 3, J = new Float32Array(n * 4), Wt = new Float32Array(n * 4);
  const N = L.nodes, local = i => N[i].id - L.base;
  const grid = new Map(); N.forEach((o, i) => { if (!o.g) return; grid.set(o.g.join(','), i); });
  for (let v = 0; v < n; v++) {
    const bone = g.M[v * 4 + 1], part = g.M[v * 4 + 3];
    if ((bone >= 1 && bone <= 4) || bone >= 8) continue;
    const p = [g.P[v * 3], g.P[v * 3 + 1], g.P[v * 3 + 2]];
    if (bone === 5) { J[v * 4] = local(N.findIndex(o => o.cls === C.STEER)); Wt[v * 4] = 1; continue; }
    const glass = g.M[v * 4] === 0 && g.K[v * 4] < 0;
    const ok = part === P_ENGINE ? o => o.cls === C.ENG || o.cls === C.BULK : part === P_INT ? o => o.cls === C.FLOOR || o.cls === C.RAIL || o.cls === C.BULK || o.cls === C.DASH || o.cls === C.SEAT : o => (o.cls !== C.ENG || part === P_SUSP) && (glass || o.cls !== C.GLASS);
    const gi = Math.round((p[0] + .675) / GH), gj = Math.round((p[1] - .25) / GH), gk = Math.round((1.95 - p[2]) / GH);
    const cand = [];
    for (let R = 1; R <= 6 && cand.length < 4; R++) {
      cand.length = 0;
      for (let a = -R; a <= R; a++) for (let b = -R; b <= R; b++) for (let c = -R; c <= R; c++) {
        const m = grid.get((gi + a) + ',' + (gj + b) + ',' + (gk + c)); if (m === undefined || !ok(N[m])) continue;
        const q = N[m].p, d = Math.hypot(q[0] - p[0], q[1] - p[1], q[2] - p[2]); cand.push([d, m]);
      }
    }
    if (part === P_INT) N.forEach((o, m) => { if (!o.g && o.cls === C.SEAT) cand.push([Math.hypot(o.p[0] - p[0], o.p[1] - p[1], o.p[2] - p[2]), m]); });
    cand.sort((a, b) => a[0] - b[0]);
    const use = cand.slice(0, 4); let s = 0;
    const d0 = use.length ? use[0][0] : 0;
    use.forEach(([d], k) => { const w = 1 / (d * d + .0025) * Math.max(0, 1 - (d - d0) / .2); Wt[v * 4 + k] = w; s += w; });
    use.forEach(([d, m], k) => { J[v * 4 + k] = local(m); Wt[v * 4 + k] /= s || 1; });
  }
  return { J, W: Wt };
}
