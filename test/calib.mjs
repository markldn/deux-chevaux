// Calibration: the 1976 magazine test. Rigid wall, 40 km/h. Measured 2CV: 33 g peak, 375 mm static crush.
// node test/calib.mjs [kmh] [substeps-per-1/60s]
import { World, step, boxCollider, setActive } from '../src/soft.js';
import { buildLattice, CLS, scaleYield, setBend, setCab } from '../src/lattice.js';
if (process.env.CAB) setCab(+process.env.CAB);
if (process.env.YS) scaleYield(+process.env.YS); if (process.env.BEND) setBend(+process.env.BEND);
const kmh = +(process.argv[2] || 40), SUB = +(process.argv[3] || 17), v0 = kmh / 3.6;
const W = World();
const t0 = Date.now();
const L = buildLattice(W, 1); setActive(W);
console.log('nodes', W.n, 'beams', W.nb, 'build ms', Date.now() - t0);
let mass = 0; for (let i = 0; i < W.n; i++) mass += W.w[i] ? 1 / W.w[i] : 0; console.log('mass', mass.toFixed(1));
// place the car with its bumper 2 cm from the wall face (wall face at z = 0, car drives +z)
const off = -1.95 - .09;
for (let i = 0; i < W.n; i++) { W.x[i * 3 + 2] += off; W.p[i * 3 + 2] += off; W.v[i * 3 + 2] = v0; }
W.colliders.push(boxCollider([0, 1, 1.5], [2.5, 1.5, 1.5], 0, { mu: .4 }));
const rearIds = L.nodes.filter(n => n.cls === CLS.BUMPER && n.p[2] < 0).map(n => n.id);
const cabin = L.nodes.filter(n => (n.cls === CLS.FLOOR || n.cls === CLS.RAIL) && n.p[2] < 0 && n.p[2] > -1.4).map(n => n.id);
const avg = (ids, k, a = W.x) => ids.reduce((s, i) => s + a[i * 3 + k], 0) / ids.length;
const frontZ = () => { let m = -1e9; for (const n of L.nodes) if (n.g && Math.abs(n.p[0]) < .5 && n.cls !== CLS.HUB) m = Math.max(m, W.x[n.id * 3 + 2]); return m; };
const len0 = frontZ() - avg(rearIds, 2);
const rear0 = avg(rearIds, 2);
let vPrev = avg(cabin, 2, W.v), aF = 0, peak = 0, tPeak = 0, maxDyn = 0, tStop = 0;
const dt = 1 / 60 / SUB * SUB; // one call per 1/60 s
const pulse = [];
const tt = Date.now();
for (let f = 0; f < 60 * 1.2; f++) {
  // sample the pulse at the substep rate by stepping one substep at a time
  for (let s = 0; s < SUB; s++) {
    step(W, 1 / 60 / SUB, 1);
    const v = avg(cabin, 2, W.v), a = (vPrev - v) / (1 / 60 / SUB) / 9.81; vPrev = v;
    aF += (a - aF) * Math.min(1, 2 * Math.PI * 100 / 60 / SUB / 1); // ~100 Hz first-order low-pass (CFC60-ish)
    if (aF > peak) { peak = aF; tPeak = W.t; }
    pulse.push([W.t, aF, v]);
    if (!tStop && v <= 0) tStop = W.t;
  }
  maxDyn = Math.max(maxDyn, avg(rearIds, 2) - rear0);
}
const ms = Date.now() - tt;
const len1 = frontZ() - avg(rearIds, 2);
const vEnd = avg(cabin, 2, W.v);
let broken = 0, glass = 0; for (let j = 0; j < W.nb; j++) if (!W.alive[j]) { broken++; if (W.bm[j] === 1) glass++; }
console.log(`${kmh} km/h: peak ${peak.toFixed(1)} g @ ${(tPeak * 1000).toFixed(0)} ms, stop ${(tStop * 1000).toFixed(0)} ms, dynamic crush ${(maxDyn * 1000).toFixed(0)} mm, static crush ${((len0 - len1) * 1000).toFixed(0)} mm, rebound ${(-vEnd).toFixed(2)} m/s, broken ${broken} (glass ${glass})`);
console.log('sim ms per real second:', (ms / 1.2).toFixed(0));
const step10 = pulse.filter((_, i) => i % (SUB * 1) === 0).slice(0, 10).map(p => `${(p[0] * 1000).toFixed(0)}ms:${p[1].toFixed(0)}g`).join(' ');
console.log(step10);

import fs from 'fs';
if (process.env.DUMP) { const o = []; for (const n of L.nodes) o.push([n.cls, ...[0,1,2].map(k => +W.x[n.id*3+k].toFixed(3)), ...n.p]); fs.writeFileSync(process.env.DUMP, JSON.stringify(o)); }
