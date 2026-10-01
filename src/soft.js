// Node/beam soft-body world (BeamNG-style), solved with small-step XPBD.
// Beams are force-limited: past their yield force they flow plastically (the rest length moves), which is where a
// crash's energy goes. Compression yield < tension yield (thin sheet buckles), densification stops flow below
// ~30 % length, and tension past the break strain tears the beam. Nodes collide with analytic colliders (ground,
// boxes, crushable barrier faces, poles) and with nodes of other bodies (spatial hash).
export const G = -9.81;
export function World(cap = 6000, bcap = 60000) {
  const W = {
    n: 0, nb: 0, t: 0,
    x: new Float64Array(cap * 3), p: new Float64Array(cap * 3), v: new Float64Array(cap * 3), w: new Float64Array(cap),
    r: new Float32Array(cap), rs: new Float32Array(cap), body: new Int16Array(cap), mu: new Float32Array(cap), gnd: new Float32Array(cap).fill(-1e9),
    rest: new Float32Array(cap * 3), dmg: new Float32Array(cap), flag: new Uint8Array(cap), aniso: new Float32Array(cap * 3),
    ba: new Int32Array(bcap), bb: new Int32Array(bcap), L0: new Float32Array(bcap), Lr: new Float32Array(bcap), bk: new Float32Array(bcap),
    fyc: new Float32Array(bcap), fyt: new Float32Array(bcap), brk: new Float32Array(bcap), bd: new Float32Array(bcap), alive: new Uint8Array(bcap),
    pl: new Float32Array(bcap), bm: new Uint8Array(bcap), bf: new Float32Array(bcap),
    colliders: [], pulleys: [], act: new Int32Array(0), isAct: new Uint8Array(0), bodies: [], ground: () => 0, groundN: null, contactSelf: true,
    energy: 0, onBreak: null, sleeping: new Uint8Array(64), awake: new Uint8Array(64).fill(1),
  };
  return W;
}
export function addNode(W, pos, mass, r, body, mu = .5) {
  const i = W.n++;
  W.x.set(pos, i * 3); W.p.set(pos, i * 3); W.v.fill(0, i * 3, i * 3 + 3); W.rest.set(pos, i * 3);
  W.w[i] = mass > 0 ? 1 / mass : 0; W.r[i] = r; W.rs[i] = r * .45; W.body[i] = body; W.mu[i] = mu; W.dmg[i] = 0; W.flag[i] = 0;
  return i;
}
// m = material {k, fyc, fyt, brk, damp}
export function addBeam(W, a, b, m, id = 0) {
  const j = W.nb++, dx = W.x[a * 3] - W.x[b * 3], dy = W.x[a * 3 + 1] - W.x[b * 3 + 1], dz = W.x[a * 3 + 2] - W.x[b * 3 + 2];
  const L = Math.hypot(dx, dy, dz);
  W.ba[j] = a; W.bb[j] = b; W.L0[j] = L; W.Lr[j] = L; W.bk[j] = m.k; W.fyc[j] = m.fyc; W.fyt[j] = m.fyt; W.brk[j] = m.brk; W.bd[j] = m.damp ?? 1;
  W.alive[j] = 1; W.pl[j] = 0; W.bm[j] = id; W.bf[j] = 0;
  return j;
}
// pulley (seat belt through a D-ring / across the chest): |A-C| + |C-B| <= L, tension-only, webbing compliance
// belt: a path anchor(cluster) -> particles... -> anchor(cluster), total length <= L. Anchors are clusters of
// structure nodes moved as one rigid patch so the light lattice nodes don't soak the load.
export function addPulley(W, a, mids, b, slack, k) {
  const A = [].concat(a), B = [].concat(b), M = [].concat(mids), path = [A[0], ...M, B[0]];
  const im = L => 1 / L.reduce((s, i) => s + (W.w[i] ? 1 / W.w[i] : 1e9), 0);
  let L = slack; for (let i = 1; i < path.length; i++) L += dist3(W.x, path[i - 1], path[i]);
  const P = { A, B, path, wa: im(A), wb: im(B), L, k, f: 0, fmax: 0, on: 1, a: A[0], b: B[0], c: M[0] }; W.pulleys.push(P); return P;
}
const dist3 = (x, i, j) => Math.hypot(x[i * 3] - x[j * 3], x[i * 3 + 1] - x[j * 3 + 1], x[i * 3 + 2] - x[j * 3 + 2]);
const tmp = new Float64Array(3);
export function step(W, dt, sub) {
  const h = dt / sub, h2 = h * h, n = W.n, x = W.x, p = W.p, v = W.v, w = W.w; W._h = h;
  for (let s = 0; s < sub; s++) {
    for (let i = 0; i < n; i++) {
      if (!w[i] || !W.awake[W.body[i]]) continue;
      const k = i * 3; v[k + 1] += G * h;
      p[k] = x[k]; p[k + 1] = x[k + 1]; p[k + 2] = x[k + 2];
      x[k] += v[k] * h; x[k + 1] += v[k + 1] * h; x[k + 2] += v[k + 2] * h;
    }
    solveBeams(W, h2, s & 1);
    for (const P of W.pulleys) if (P.on) solvePulley(W, P, h2);
    collide(W, h, s);
    for (let i = 0; i < n; i++) {
      if (!w[i] || !W.awake[W.body[i]]) continue;
      const k = i * 3; v[k] = (x[k] - p[k]) / h; v[k + 1] = (x[k + 1] - p[k + 1]) / h; v[k + 2] = (x[k + 2] - p[k + 2]) / h;
    }
    dampBeams(W, h);
    friction(W, h);
    W.t += h;
    if (W.onSub) W.onSub(h);
  }
}
function solveBeams(W, h2, rev) {
  const { x, w, ba, bb, L0, Lr, bk, fyc, fyt, brk, alive, pl, bf } = W, nb = W.nb;
  for (let q = 0; q < nb; q++) {
    const j = rev ? nb - 1 - q : q;
    if (!alive[j]) continue;
    const a = ba[j] * 3, b = bb[j] * 3, wa = w[ba[j]], wb = w[bb[j]], ws = wa + wb;
    if (!ws) continue;
    let dx = x[a] - x[b], dy = x[a + 1] - x[b + 1], dz = x[a + 2] - x[b + 2];
    const L = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1e-9;
    let C = L - L0[j];
    const k = bk[j], comp = C < 0, Fy = comp ? fyc[j] * densify(L0[j] / Lr[j]) : fyt[j], ey = Fy / k;
    if (C > ey || C < -ey) { // plastic flow: move the rest length, keep the elastic part at the yield strain
      const flow = C > 0 ? C - ey : C + ey;
      let nl = L0[j] + flow;
      if (nl < Lr[j] * .22) nl = Lr[j] * .22;
      pl[j] += Math.abs(nl - L0[j]) / Lr[j]; L0[j] = nl; C = L - nl;
      if (nl > Lr[j] * (1 + brk[j]) || pl[j] > brk[j] * 3) { alive[j] = 0; W.onBreak && W.onBreak(j); continue; }
    }
    if (brk[j] < .05 && (C > brk[j] * Lr[j] || C < -brk[j] * Lr[j] * 2)) { alive[j] = 0; W.onBreak && W.onBreak(j); continue; } // brittle (glass)
    const at = 1 / (k * h2);
    let dl = -C / (ws + at);
    const lim = Fy * h2 * 1.0001; if (dl > lim) dl = lim; else if (dl < -lim) dl = -lim;
    bf[j] = dl / h2;
    const s = dl / L; dx *= s; dy *= s; dz *= s;
    x[a] += dx * wa; x[a + 1] += dy * wa; x[a + 2] += dz * wa;
    x[b] -= dx * wb; x[b + 1] -= dy * wb; x[b + 2] -= dz * wb;
  }
}
// folded sheet stiffens as it compacts
const densify = r => r > .45 ? 1 : 1 + (.45 - r) * (.45 - r) * 120;
function dampBeams(W, h) {
  const { x, v, w, ba, bb, bd, alive } = W;
  for (let j = 0; j < W.nb; j++) {
    if (!alive[j] || !bd[j]) continue;
    const a = ba[j] * 3, b = bb[j] * 3, wa = w[ba[j]], wb = w[bb[j]], ws = wa + wb; if (!ws) continue;
    let dx = x[a] - x[b], dy = x[a + 1] - x[b + 1], dz = x[a + 2] - x[b + 2]; const L = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1; dx /= L; dy /= L; dz /= L;
    const vr = (v[a] - v[b]) * dx + (v[a + 1] - v[b + 1]) * dy + (v[a + 2] - v[b + 2]) * dz;
    const c = vr * Math.min(1, bd[j] * h * 400) / ws;
    v[a] -= dx * c * wa; v[a + 1] -= dy * c * wa; v[a + 2] -= dz * c * wa;
    v[b] += dx * c * wb; v[b + 1] += dy * c * wb; v[b + 2] += dz * c * wb;
  }
}
function solvePulley(W, P, h2) {
  const x = W.x, w = W.w, pa = P.path, n = pa.length, u = [];
  let C = -P.L;
  for (let i = 1; i < n; i++) { const a = pa[i - 1] * 3, b = pa[i] * 3, d = [x[b] - x[a], x[b + 1] - x[a + 1], x[b + 2] - x[a + 2]], l = Math.hypot(...d) || 1e-9; C += l; u.push(d.map(q => q / l)); }
  if (C <= 0) { P.f = 0; return; }
  // gradient: end anchors pull along their segment, each middle point is pushed by both neighbours
  const g = []; for (let i = 0; i < n; i++) { const pr = i > 0 ? u[i - 1] : [0, 0, 0], nx = i < n - 1 ? u[i] : [0, 0, 0]; g.push([pr[0] - nx[0], pr[1] - nx[1], pr[2] - nx[2]]); }
  let wsum = P.wa * (g[0][0] ** 2 + g[0][1] ** 2 + g[0][2] ** 2) + P.wb * (g[n - 1][0] ** 2 + g[n - 1][1] ** 2 + g[n - 1][2] ** 2);
  for (let i = 1; i < n - 1; i++) wsum += w[pa[i]] * (g[i][0] ** 2 + g[i][1] ** 2 + g[i][2] ** 2);
  const dl = -C / (wsum + 1 / (P.k * h2)); P.f = -dl / h2; if (P.f > P.fmax) P.fmax = P.f;
  for (let k = 0; k < 3; k++) {
    for (const i of P.A) x[i * 3 + k] += P.wa * dl * g[0][k];
    for (const i of P.B) x[i * 3 + k] += P.wb * dl * g[n - 1][k];
    for (let i = 1; i < n - 1; i++) x[pa[i] * 3 + k] += w[pa[i]] * dl * g[i][k];
  }
}
// ---- collisions. Contacts are remembered for the friction pass.
const CN = 16384, cI = new Int32Array(CN), cJ = new Int32Array(CN), cNx = new Float32Array(CN), cNy = new Float32Array(CN), cNz = new Float32Array(CN), cD = new Float32Array(CN), cMu = new Float32Array(CN);
let nc = 0;
const HS = 1 << 12, hHead = new Int32Array(HS), hNext = new Int32Array(8192);
function collide(W, h, s) {
  const { x, w, r, n } = W; nc = 0;
  // analytic colliders + ground
  for (let i = 0; i < n; i++) {
    if (!w[i] || !W.awake[W.body[i]]) continue;
    const k = i * 3; let px = x[k], py = x[k + 1], pz = x[k + 2];
    let gy = W.gnd[i];
    if (gy < -1e8 || (s & 7) === 0) gy = W.gnd[i] = W.ground(px, pz);
    const ri = r[i] * (W.flag[i] & 2 ? 1 : .5);
    if (py - ri < gy) { const N = W.groundN ? W.groundN(px, pz) : [0, 1, 0]; const d = gy - (py - ri); x[k] += N[0] * d * N[1]; x[k + 1] += d * N[1] * N[1]; x[k + 2] += N[2] * d * N[1]; addC(i, -1, N[0], N[1], N[2], d, W.mu[i]); }
    for (const c of W.colliders) c.hit(W, i, ri);
  }
  // node-node, hashed. Only "active" nodes query the hash (engine block, hubs, dummy, a second vehicle):
  // a car's thin shell folding onto itself is left to the beams.
  if (W.nnEvery2 && !(s & 1)) return;
  const cs = .3, inv = 1 / cs, act = W.act;
  hHead.fill(-1);
  for (let i = 0; i < n; i++) {
    if (!r[i]) continue; const k = i * 3;
    const hk = hashc(Math.floor(x[k] * inv), Math.floor(x[k + 1] * inv), Math.floor(x[k + 2] * inv));
    hNext[i] = hHead[hk]; hHead[hk] = i;
  }
  for (let q = 0; q < act.length; q++) {
    const i = act[q]; if (!r[i]) continue; const k = i * 3, bi = W.body[i];
    const cx = Math.floor(x[k] * inv), cy = Math.floor(x[k + 1] * inv), cz = Math.floor(x[k + 2] * inv);
    for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
      for (let j = hHead[hashc(cx + a, cy + b, cz + c)]; j >= 0; j = hNext[j]) {
        if (j === i || (W.isAct[j] && j < i)) continue;
        const same = W.body[j] === bi;
        if (same && !W.contactSelf) continue;
        const jk = j * 3, dx = x[k] - x[jk], dy = x[k + 1] - x[jk + 1], dz = x[k + 2] - x[jk + 2];
        const R = same ? W.rs[i] + W.rs[j] : r[i] + r[j], d2 = dx * dx + dy * dy + dz * dz;
        if (d2 >= R * R || d2 < 1e-12) continue;
        if (same) { const rx = W.rest[k] - W.rest[jk], ry = W.rest[k + 1] - W.rest[jk + 1], rz = W.rest[k + 2] - W.rest[jk + 2]; if (rx * rx + ry * ry + rz * rz < .09) continue; }
        const wi = w[i], wj = w[j], ws = wi + wj; if (!ws) continue;
        const d = Math.sqrt(d2), pen = R - d, nx = dx / d, ny = dy / d, nz = dz / d, si = pen * wi / ws, sj = pen * wj / ws;
        x[k] += nx * si; x[k + 1] += ny * si; x[k + 2] += nz * si; x[jk] -= nx * sj; x[jk + 1] -= ny * sj; x[jk + 2] -= nz * sj;
        addC(i, j, nx, ny, nz, pen, Math.min(W.mu[i], W.mu[j]));
      }
    }
  }
}
// which nodes query for node-node contacts: flagged ones (bit 4) and every node of a body marked fully active
export function setActive(W, bodies = []) {
  const a = []; W.isAct = new Uint8Array(W.n);
  for (let i = 0; i < W.n; i++) if ((W.flag[i] & 4) || bodies.includes(W.body[i])) { a.push(i); W.isAct[i] = 1; }
  W.act = Int32Array.from(a);
}
const hashc = (a, b, c) => ((Math.imul(a, 73856093) ^ Math.imul(b, 19349663) ^ Math.imul(c, 83492791)) >>> 0) & (HS - 1);
export function addC(i, j, nx, ny, nz, d, mu) { if (nc >= CN) return; cI[nc] = i; cJ[nc] = j; cNx[nc] = nx; cNy[nc] = ny; cNz[nc] = nz; cD[nc] = d; cMu[nc] = mu; nc++; }
// Coulomb friction from the penetration each contact resolved (d/h is the normal velocity it removed)
function friction(W, h) {
  const v = W.v, w = W.w;
  for (let c = 0; c < nc; c++) {
    const i = cI[c] * 3, j = cJ[c], nx = cNx[c], ny = cNy[c], nz = cNz[c];
    let rvx = v[i], rvy = v[i + 1], rvz = v[i + 2];
    if (j >= 0) { rvx -= v[j * 3]; rvy -= v[j * 3 + 1]; rvz -= v[j * 3 + 2]; }
    const vn = rvx * nx + rvy * ny + rvz * nz, tx = rvx - vn * nx, ty = rvy - vn * ny, tz = rvz - vn * nz;
    let vt = Math.sqrt(tx * tx + ty * ty + tz * tz); if (vt < 1e-6) continue;
    const an = W.aniso[cI[c] * 3] || W.aniso[cI[c] * 3 + 1] || W.aniso[cI[c] * 3 + 2];
    let mu = cMu[c];
    let dv = Math.min(vt, mu * cD[c] / h);
    let fx = tx / vt, fy = ty / vt, fz = tz / vt;
    if (an && j < 0) { // rolling wheel: friction only across its rolling direction (the axle axis)
      const ax = W.aniso[cI[c] * 3], ay = W.aniso[cI[c] * 3 + 1], az = W.aniso[cI[c] * 3 + 2];
      const va = tx * ax + ty * ay + tz * az, rollx = tx - va * ax, rolly = ty - va * ay, rollz = tz - va * az;
      const lat = Math.abs(va); dv = Math.min(lat, mu * cD[c] / h); fx = Math.sign(va) * ax; fy = Math.sign(va) * ay; fz = Math.sign(va) * az;
      const rr = Math.min(1, .02 * h * 60); v[i] -= rollx * rr * .02; v[i + 1] -= rolly * rr * .02; v[i + 2] -= rollz * rr * .02;
    }
    if (j < 0) { v[i] -= fx * dv; v[i + 1] -= fy * dv; v[i + 2] -= fz * dv; }
    else {
      const wi = w[cI[c]], wj = w[j], ws = wi + wj; if (!ws) continue;
      v[i] -= fx * dv * wi / ws; v[i + 1] -= fy * dv * wi / ws; v[i + 2] -= fz * dv * wi / ws;
      v[j * 3] += fx * dv * wj / ws; v[j * 3 + 1] += fy * dv * wj / ws; v[j * 3 + 2] += fz * dv * wj / ws;
    }
  }
}
// ---- analytic colliders
// oriented box: c centre, ax/ay/az unit axes, e half extents. Optional crushable face (ODB honeycomb): the -z face
// of the box gives way at `stress` (Pa) over `cell` sized cells up to `depth`.
export function boxCollider(c, e, yaw = 0, o = {}) {
  const ca = Math.cos(yaw), sa = Math.sin(yaw), ax = [ca, 0, -sa], az = [sa, 0, ca];
  const C = { c, e, ax, az, mu: o.mu ?? .5, crush: o.crush, kind: 'box', vel: o.vel || null, mass: o.mass || 0, impulse: [0, 0, 0], force: 0 };
  if (o.crush) { const nx = Math.ceil(e[0] * 2 / o.crush.cell), ny = Math.ceil(e[1] * 2 / o.crush.cell); C.cd = new Float32Array(nx * ny); C.cnx = nx; C.cny = ny; }
  C.pen = (px, py, pz, ri) => {
    const dx = px - C.c[0], dy = py - C.c[1], dz = pz - C.c[2], lx = dx * ax[0] + dz * ax[2], lz = dx * az[0] + dz * az[2];
    return Math.min(e[0] + ri - Math.abs(lx), e[1] + ri - Math.abs(dy), e[2] + ri - Math.abs(lz));
  };
  C.hit = (W, i, ri) => {
    const k = i * 3, x = W.x;
    const dx = x[k] - C.c[0], dy = x[k + 1] - C.c[1], dz = x[k + 2] - C.c[2];
    let lx = dx * ax[0] + dz * ax[2], ly = dy, lz = dx * az[0] + dz * az[2];
    let ez = e[2], cell = -1;
    if (C.cd && lz < 0) { const cx = Math.floor((lx + e[0]) / o.crush.cell), cy = Math.floor((ly + e[1]) / o.crush.cell); if (cx >= 0 && cy >= 0 && cx < C.cnx && cy < C.cny) { cell = cx + cy * C.cnx; ez = e[2] - C.cd[cell]; } }
    const qx = Math.abs(lx) - e[0] - ri, qy = Math.abs(ly) - e[1] - ri, qz = (lz < 0 ? -lz - ez : lz - e[2]) - ri;
    if (qx >= 0 || qy >= 0 || qz >= 0) return;
    // push out along the axis of least penetration
    let nx = 0, ny = 0, nz = 0, d;
    if (qz >= qx && qz >= qy) { d = -qz; nz = lz < 0 ? -1 : 1; }
    else if (qx >= qy) { d = -qx; nx = Math.sign(lx); } else { d = -qy; ny = Math.sign(ly); }
    if (cell >= 0 && nz < 0) { // honeycomb crushes: it only pushes back with its crush stress
      const m = 1 / W.w[i], h = W._h || 1e-3, maxPush = o.crush.stress * o.crush.cell * o.crush.cell * h * h / m / Math.max(1, o.crush.share);
      const push = Math.min(d, maxPush); const rest = d - push;
      if (rest > 0 && C.cd[cell] < o.crush.depth) { C.cd[cell] = Math.min(o.crush.depth, C.cd[cell] + rest); }
      d = push;
    }
    const wx = nx * ax[0] + nz * az[0], wz = nx * ax[2] + nz * az[2];
    x[k] += wx * d; x[k + 1] += ny * d; x[k + 2] += wz * d;
    C.force += d / W.w[i];
    addC(i, -1, wx, ny, wz, d, C.mu);
  };
  return C;
}
export function cylCollider(cx, cz, rad, hgt, y0 = 0, mu = .4) {
  const C = { kind: 'cyl', cx, cz, rad, hgt, mu, force: 0 };
  C.pen = (px, py, pz, ri) => py > y0 + hgt || py < y0 - .5 ? -1 : C.rad + ri - Math.hypot(px - C.cx, pz - C.cz);
  C.hit = (W, i, ri) => {
    const k = i * 3, x = W.x, dx = x[k] - C.cx, dz = x[k + 2] - C.cz, R = C.rad + ri;
    if (x[k + 1] > y0 + hgt || x[k + 1] < y0 - .5) return;
    const d2 = dx * dx + dz * dz; if (d2 >= R * R) return;
    const d = Math.sqrt(d2) || 1e-6, pen = R - d, nx = dx / d, nz = dz / d;
    x[k] += nx * pen; x[k + 2] += nz * pen; C.force += pen / W.w[i];
    addC(i, -1, nx, 0, nz, pen, mu);
  };
  return C;
}
// ---- per-node rotation (polar decomposition of the neighbourhood deformation), for skinning normals + verts
export function nodeFrames(W, adj, q, iters = 2) {
  const x = W.x, r = W.rest;
  for (let i = 0; i < W.n; i++) {
    const nb = adj[i]; if (!nb || !nb.length) continue;
    const A = [0, 0, 0, 0, 0, 0, 0, 0, 0], i3 = i * 3;
    for (const j of nb) {
      const j3 = j * 3, px = x[j3] - x[i3], py = x[j3 + 1] - x[i3 + 1], pz = x[j3 + 2] - x[i3 + 2], rx = r[j3] - r[i3], ry = r[j3 + 1] - r[i3 + 1], rz = r[j3 + 2] - r[i3 + 2];
      A[0] += px * rx; A[1] += py * rx; A[2] += pz * rx; A[3] += px * ry; A[4] += py * ry; A[5] += pz * ry; A[6] += px * rz; A[7] += py * rz; A[8] += pz * rz;
    }
    extractRotation(A, q, i * 4, iters);
  }
}
// same, for a body's own local arrays (deformed positions vs rest, local adjacency); q stored (w,x,y,z)
export function nodeFramesL(x, r, adj, q, iters = 1) {
  for (let i = 0; i < adj.length; i++) {
    const nb = adj[i]; if (!nb.length) continue;
    const A = [0, 0, 0, 0, 0, 0, 0, 0, 0], i3 = i * 3;
    for (const j of nb) {
      const j3 = j * 3, px = x[j3] - x[i3], py = x[j3 + 1] - x[i3 + 1], pz = x[j3 + 2] - x[i3 + 2], rx = r[j3] - r[i3], ry = r[j3 + 1] - r[i3 + 1], rz = r[j3 + 2] - r[i3 + 2];
      A[0] += px * rx; A[1] += py * rx; A[2] += pz * rx; A[3] += px * ry; A[4] += py * ry; A[5] += pz * ry; A[6] += px * rz; A[7] += py * rz; A[8] += pz * rz;
    }
    extractRotation(A, q, i * 4, iters);
  }
}
// Müller et al. 2016 "A robust method to extract the rotational part of deformations"; A column-major 3x3
export function extractRotation(A, q, o, iters) {
  for (let it = 0; it < iters; it++) {
    const R = qmat(q[o], q[o + 1], q[o + 2], q[o + 3]);
    // omega = sum(r_i x a_i) / (|sum r_i . a_i| + eps)
    let wx = 0, wy = 0, wz = 0, dd = 0;
    for (let c = 0; c < 3; c++) {
      const rx = R[c * 3], ry = R[c * 3 + 1], rz = R[c * 3 + 2], ax = A[c * 3], ay = A[c * 3 + 1], az = A[c * 3 + 2];
      wx += ry * az - rz * ay; wy += rz * ax - rx * az; wz += rx * ay - ry * ax; dd += rx * ax + ry * ay + rz * az;
    }
    const s = 1 / (Math.abs(dd) + 1e-9); wx *= s; wy *= s; wz *= s;
    const ang = Math.hypot(wx, wy, wz); if (ang < 1e-9) break;
    const h = ang * .5, sn = Math.sin(h) / ang, dq = [Math.cos(h), wx * sn, wy * sn, wz * sn];
    const a0 = q[o], a1 = q[o + 1], a2 = q[o + 2], a3 = q[o + 3]; // q = dq * q   (w,x,y,z)
    q[o] = dq[0] * a0 - dq[1] * a1 - dq[2] * a2 - dq[3] * a3;
    q[o + 1] = dq[0] * a1 + dq[1] * a0 + dq[2] * a3 - dq[3] * a2;
    q[o + 2] = dq[0] * a2 - dq[1] * a3 + dq[2] * a0 + dq[3] * a1;
    q[o + 3] = dq[0] * a3 + dq[1] * a2 - dq[2] * a1 + dq[3] * a0;
    const l = Math.hypot(q[o], q[o + 1], q[o + 2], q[o + 3]); q[o] /= l; q[o + 1] /= l; q[o + 2] /= l; q[o + 3] /= l;
  }
}
export function qmat(w, x, y, z) { // column-major rotation matrix of quaternion (w,x,y,z)
  return [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y), 2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x), 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)];
}
export const contactCount = () => nc;
