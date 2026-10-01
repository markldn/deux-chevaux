// Mesh builders. A Geo accumulates P (rest position), N, C (rgb + roughness), M (material, bone, ao, part),
// K (window sdf, seam sdf, canvas sdf, aux) and indices. Every builder is a parametric grid f(u,v) -> xyz.
import { norm, cross, sub, add, scl, dot } from './math.js';

export function Geo() { return { P: [], N: [], C: [], M: [], K: [], I: [] }; }
export const nv = g => g.P.length / 3;

// o: { col:[r,g,b,rough] | (p,u,v)=>col, mat, bone, part, k:(p,u,v)=>[4], flip, wu, wv }
export function grid(g, nu, nvv, f, o = {}) {
  const base = nv(g), cu = o.wu ? nu : nu + 1, cv = o.wv ? nvv : nvv + 1, pts = [];
  for (let i = 0; i < cu; i++) for (let j = 0; j < cv; j++) {
    const u = i / nu, v = j / nvv, p = f(u, v); pts.push(p);
    g.P.push(p[0], p[1], p[2]);
    const c = typeof o.col === 'function' ? o.col(p, u, v) : o.col || [.5, .5, .5, .5];
    g.C.push(c[0], c[1], c[2], c[3] ?? .5);
    g.M.push(o.mat ?? 0, o.bone ?? 0, 1, o.part ?? 0);
    const k = o.k ? o.k(p, u, v) : [9, 9, 9, 0]; g.K.push(k[0], k[1], k[2], k[3]);
  }
  const id = (i, j) => base + (o.wu ? i % nu : i) * cv + (o.wv ? j % nvv : j), tris = [];
  for (let i = 0; i < nu; i++) for (let j = 0; j < nvv; j++) {
    const a = id(i, j), b = id(i + 1, j), c = id(i + 1, j + 1), d = id(i, j + 1);
    if (o.flip) tris.push(a, c, b, a, d, c); else tris.push(a, b, c, a, c, d);
  }
  // smooth normals over this grid only (hard edges between parts stay hard)
  const n = new Float32Array(pts.length * 3);
  for (let t = 0; t < tris.length; t += 3) {
    const A = tris[t] - base, B = tris[t + 1] - base, C = tris[t + 2] - base;
    const pa = pts[A], e1 = sub(pts[B], pa), e2 = sub(pts[C], pa), fn = cross(e1, e2);
    for (const q of [A, B, C]) { n[q * 3] += fn[0]; n[q * 3 + 1] += fn[1]; n[q * 3 + 2] += fn[2]; }
  }
  const cen = pts.reduce((a, p) => add(a, scl(p, 1 / pts.length)), [0, 0, 0]);
  for (let q = 0; q < pts.length; q++) {
    let m = [n[q * 3], n[q * 3 + 1], n[q * 3 + 2]];
    if (Math.hypot(...m) < 1e-12) m = sub(pts[q], cen); // degenerate pole: point away from the part's centre
    m = norm(m); g.N.push(m[0], m[1], m[2]);
  }
  for (const t of tris) g.I.push(t);
  return base;
}
// frame: origin o, axis a (lathe axis), radial basis b,c
export const frame = (o, a, b) => { a = norm(a); b = norm(b); return { o, a, b, c: cross(a, b) }; };
const at = (F, x, r, t) => add(add(F.o, scl(F.a, x)), add(scl(F.b, Math.cos(t) * r), scl(F.c, Math.sin(t) * r)));
// lathe: profile [[x, r], ...] along axis; seg around. Profile points are taken literally (duplicate for creases).
export function lathe(g, F, prof, seg, o = {}) {
  const n = prof.length - 1;
  return grid(g, n, seg, (u, v) => { const p = prof[Math.round(u * n)]; return at(F, p[0], p[1], v * Math.PI * 2 + (o.t0 || 0)); }, { ...o, wv: 1, flip: !o.flip });
}
// tube along a polyline with parallel-transport frames; r number or fn(t)
export function tube(g, pts, r, seg, o = {}) {
  const n = pts.length, T = [], B = [];
  for (let i = 0; i < n; i++) T.push(norm(sub(pts[Math.min(i + 1, n - 1)], pts[Math.max(i - 1, 0)])));
  let b = Math.abs(T[0][1]) < .9 ? norm(cross(T[0], [0, 1, 0])) : norm(cross(T[0], [1, 0, 0]));
  for (let i = 0; i < n; i++) { b = norm(sub(b, scl(T[i], dot(b, T[i])))); B.push(b); }
  return grid(g, n - 1, seg, (u, v) => {
    const i = Math.round(u * (n - 1)), t = v * Math.PI * 2, c = cross(T[i], B[i]), rr = typeof r === 'function' ? r(u) : r;
    return add(pts[i], add(scl(B[i], Math.cos(t) * rr), scl(c, Math.sin(t) * rr)));
  }, { ...o, wv: 1, flip: !o.flip });
}
// superellipsoid / rounded box centred at c with half-extents e and squareness n (2 = ellipsoid, 8+ = box)
export function blob(g, c, e, n, o = {}, xf) {
  const sp = (x, p) => Math.sign(x) * Math.pow(Math.abs(x), p);
  return grid(g, o.nu || 16, o.nv || 24, (u, v) => {
    const ph = (u - .5) * Math.PI, th = v * Math.PI * 2, e2 = 2 / n;
    let p = [e[0] * sp(Math.cos(ph), e2) * sp(Math.cos(th), e2), e[1] * sp(Math.sin(ph), e2), e[2] * sp(Math.cos(ph), e2) * sp(Math.sin(th), e2)];
    if (xf) p = xf(p);
    return add(c, p);
  }, { ...o, wv: 1 });
}
// helper: piecewise-linear table lookup (rows sorted by first column, any order of x)
export function table(rows) {
  const s = rows.slice().sort((a, b) => a[0] - b[0]);
  return x => {
    if (x <= s[0][0]) return s[0].slice(1);
    for (let i = 1; i < s.length; i++) if (x <= s[i][0]) {
      const a = s[i - 1], b = s[i], t = (x - a[0]) / (b[0] - a[0]), k = t * t * (3 - 2 * t) * .35 + t * .65;
      return a.slice(1).map((v, j) => v + (b[j + 1] - v) * k);
    }
    return s[s.length - 1].slice(1);
  };
}
// 2D signed distance to a convex polygon (pts CCW or CW)
export function polySdf(p, pts) {
  let d = Infinity, s = 1;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const e = [pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]], w = [p[0] - pts[j][0], p[1] - pts[j][1]];
    const t = Math.max(0, Math.min(1, (w[0] * e[0] + w[1] * e[1]) / (e[0] * e[0] + e[1] * e[1])));
    const b = [w[0] - e[0] * t, w[1] - e[1] * t]; d = Math.min(d, b[0] * b[0] + b[1] * b[1]);
    const c = [p[1] >= pts[j][1], p[1] < pts[i][1], e[0] * w[1] > e[1] * w[0]];
    if ((c[0] && c[1] && c[2]) || (!c[0] && !c[1] && !c[2])) s = -s;
  }
  return s * Math.sqrt(d);
}
