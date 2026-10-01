// World shared by physics and rendering. The noise is bit-identical to the GLSL copy in glsl.js (same integer
// hash), so the ground the car drives on is the ground that is drawn.
import { clamp, smooth, lerp } from './math.js';

export function hash(x, z) {
  let h = (Math.imul(x | 0, 1597334677) ^ Math.imul(z | 0, -482951495)) >>> 0;
  h ^= h >>> 16; h = Math.imul(h, -2048144777) >>> 0;
  h ^= h >>> 13; h = Math.imul(h, -1028477379) >>> 0;
  h ^= h >>> 16;
  return (h >>> 0) / 4294967296;
}
export function vnoise(x, z) {
  const ix = Math.floor(x), iz = Math.floor(z), fx = x - ix, fz = z - iz;
  const ux = fx * fx * fx * (fx * (fx * 6 - 15) + 10), uz = fz * fz * fz * (fz * (fz * 6 - 15) + 10);
  return lerp(lerp(hash(ix, iz), hash(ix + 1, iz), ux), lerp(hash(ix, iz + 1), hash(ix + 1, iz + 1), ux), uz);
}
export function fbm(x, z, o) {
  let s = 0, a = .5;
  for (let i = 0; i < o; i++) { s += a * vnoise(x, z); const nx = 1.6 * x + 1.2 * z + 17.3, nz = -1.2 * x + 1.6 * z + 9.1; x = nx; z = nz; a *= .5; }
  return s;
}
// ---- layout (metres). The barrier face is at z = 0, cars run towards +z along the runway.
export const FAC = [-70, 70, -300, 60];      // flat test centre: x0 x1 z0 z1
export const AVE_Z = 140;                     // the avenue of plane trees (east-west road)
export const FIELD = [90, 230, -170, -30];   // ploughed field: x0 x1 z0 z1 (furrows run along z)
export const BARRIER = { x: 0, z: 0, w: 2.6, d: 3.2, h: 1.9 };
export const PIT = [-1.3, 1.3, -4.6, -.6];   // under-floor camera pit with a glass lid
const box = (x, z, r) => Math.hypot(Math.max(r[0] - x, 0, x - r[1]), Math.max(r[2] - z, 0, z - r[3]));
export const roadMask = (x, z) => Math.max(1 - smooth(3.5, 7, Math.abs(z - AVE_Z)), z > 50 && z < AVE_Z ? 1 - smooth(3, 6, Math.abs(x)) : 0);
export const plough = (x, z) => { const m = box(x, z, FIELD); return m > 0 ? 0 : .075 * Math.sin(x * 8.378) * smooth(0, 4, Math.min(x - FIELD[0], FIELD[1] - x, z - FIELD[2], FIELD[3] - z)); };
export function height(x, z) {
  if (x > FAC[0] && x < FAC[1] && z > FAC[2] && z < FAC[3]) return 0;
  const d = Math.hypot(x, z + 100);
  let h = (fbm(x / 520, z / 520, 4) - .5) * 34 * smooth(80, 600, d);
  const r = roadMask(x, z);
  h += ((fbm(x / 90 + 5, z / 90, 3) - .5) * 4 + (vnoise(x / 7, z / 7) - .5) * .25) * (1 - r);
  h *= smooth(0, 45, box(x, z, FAC));
  return h + plough(x, z);
}
export function normalAt(x, z) {
  const e = .05, hx = height(x + e, z) - height(x - e, z), hz = height(x, z + e) - height(x, z - e), l = Math.hypot(hx, 2 * e, hz);
  return [-hx / l, 2 * e / l, -hz / l];
}
// plane trees along the avenue + scattered oaks/poplars; [x, z, scale, kind]
export const TREES = [];
for (let x = -900; x <= 900; x += 11) for (const s of [-1, 1]) {
  if (Math.abs(x) < 9) continue;
  TREES.push([x + (hash(x, s) - .5) * 1.5, AVE_Z + s * (6.3 + hash(s, x) * .6), .9 + hash(x, 7 * s) * .25, 0]);
}
for (let i = 0; i < 160; i++) {
  const x = (hash(i, 91) - .5) * 1800, z = (hash(i, 37) - .5) * 1800;
  if (box(x, z, FAC) < 25 || Math.abs(z - AVE_Z) < 14 || box(x, z, FIELD) < 6 || (Math.abs(x) < 10 && z > 40 && z < AVE_Z)) continue;
  TREES.push([x, z, .8 + hash(i, 5) * .6, hash(i, 3) < .35 ? 1 : 0]);
}
// a poplar row along the field
for (let z = FIELD[2]; z <= FIELD[3]; z += 7) TREES.push([FIELD[1] + 6, z, 1, 1]);
export const BALES = [];
for (let i = 0; i < 14; i++) BALES.push([-40 + (hash(i, 11) - .5) * 30 - 30, -60 + i * 9 + (hash(i, 12) - .5) * 4]);
// sun
export function sunDir(tod) { // tod 0 = dawn, 1 = dusk
  const a = lerp(.08, Math.PI - .08, tod), el = Math.sin(a) * .95;
  const az = lerp(-1.9, 1.9, tod) + .4;
  return [Math.cos(el) * Math.sin(az), Math.sin(el), Math.cos(el) * Math.cos(az)];
}
