// A 2CV in the world. While driving it is a rigid body on four wheels with the 2CV's interconnected suspension
// (front and rear spring on each side share one canister: stiff in heave, very soft in pitch). The moment it
// touches an obstacle it hands over to its node/beam lattice in the soft-body world, which runs until the car is
// calm again, then the deformed shape becomes the new rigid shape. Crashes accumulate.
import { step, nodeFramesL, extractRotation } from './soft.js';
import { buildLattice, bindSkin, CLS } from './lattice.js';
import { WHEELS, SWHEEL, WR } from './car.js';
import { height, normalAt } from './world.js';
import { add, sub, scl, dot, cross, len, norm, clamp, lerp, qmul, qaxis, qnorm, qrot, qconj, qm4, qfromto, mul } from './math.js';

export const SUB = 17;                       // soft substeps per 1/60 s: the calibrated step (h = 0.98 ms)
const KC = 2200, KA = 2500, DAMP = 700, DROOP = -.17, BUMP = .12, MU = .85;
const GEARS = [-15, 0, 18.9, 9.8, 6.0, 4.45], IDLE = 850, RED = 6000;
const torque = rpm => rpm < 600 ? 0 : 39 * Math.max(0, 1 - ((rpm - 3500) / 3600) ** 2) * (rpm > RED ? 0 : 1); // 39 N·m @ 3500, ~29 hp @ 5750

export function createCar(W, body, opt = {}) {
  const L = buildLattice(W, body), n = L.count, base = L.base;
  const restL = new Float32Array(n * 3), loc = new Float32Array(n * 3), q = new Float32Array(n * 4), dmg = new Float32Array(n), mass = new Float32Array(n);
  for (let i = 0; i < n; i++) { for (let k = 0; k < 3; k++) restL[i * 3 + k] = loc[i * 3 + k] = W.x[(base + i) * 3 + k]; q[i * 4] = 1; mass[i] = 1 / W.w[base + i]; }
  const adj = L.adj.slice(base, base + n).map(a => a.map(j => j - base));
  const inc = Array.from({ length: n }, () => []);
  for (let j = 0; j < W.nb; j++) { const a = W.ba[j] - base, b = W.bb[j] - base; if (a >= 0 && a < n) inc[a].push(j); if (b >= 0 && b < n) inc[b].push(j); }
  const hubs = L.hubs.map(h => h - base), piv = L.piv.map(p => p.map(i => i - base));
  const C = {
    W, body, L, n, base, restL, loc, q, dmg, mass, adj, mode: 'rigid', dirty: true,
    pos: [0, 0, 0], rot: [1, 0, 0, 0], vel: [0, 0, 0], ang: [0, 0, 0],
    c: [0, 0, 0, 0], cPrev: [0, 0, 0, 0], spin: [0, 0, 0, 0], wv: [0, 0, 0, 0], contact: [0, 0, 0, 0], load: [0, 0, 0, 0], slip: [0, 0, 0, 0], lost: [0, 0, 0, 0],
    steer: 0, gear: 2, rpm: IDLE, fan: 0, throttle: 0, brake: 0, steerIn: 0, auto: true, tow: null, calm: 0, impacts: [], crashT: -1, ext: [],
    hubD: WHEELS.map(h => h.slice()), M: 0, com: [0, 0, 0], I: [0, 0, 0], frozen: false,
  };
  // mass properties
  let M = 0, cx = 0, cy = 0, cz = 0;
  for (let i = 0; i < n; i++) { M += mass[i]; cx += mass[i] * loc[i * 3]; cy += mass[i] * loc[i * 3 + 1]; cz += mass[i] * loc[i * 3 + 2]; }
  C.M = M + (opt.payload || 0); C.com = [cx / M, .5, cz / M]; // the lattice's mass sits in its shell; the real 2CV's centre of mass is ~0.5 m up
  let Ix = 0, Iy = 0, Iz = 0;
  for (let i = 0; i < n; i++) { const x = loc[i * 3] - C.com[0], y = loc[i * 3 + 1] - C.com[1], z = loc[i * 3 + 2] - C.com[2]; Ix += mass[i] * (y * y + z * z); Iy += mass[i] * (x * x + z * z); Iz += mass[i] * (x * x + y * y); }
  const kI = C.M / M; C.I = [Ix * kI, Iy * kI, Iz * kI];
  { const f = (C.com[2] - WHEELS[2][2]) / (WHEELS[0][2] - WHEELS[2][2]); C.F0s = [f, f, 1 - f, 1 - f].map(k => C.M * 9.81 * k / 2); }
  C.world = p => add(qrot(C.rot, p), C.pos);
  C.place = (p, yaw = 0, v = 0) => {
    C.rot = qaxis([0, 1, 0], yaw); C.pos = p.slice(); C.pos[1] += C.W.ground(p[0], p[2]); C.vel = qrot(C.rot, [0, 0, v]); C.ang = [0, 0, 0];
    C.c = [0, 0, 0, 0]; C.cPrev = [0, 0, 0, 0]; C.mode = 'rigid'; C.settled = false; W.awake[body] = 0; C.calm = 0;
    syncNodes(C, 0);
  };
  C.reset = () => { // factory fresh: rest shape, beams restored
    loc.set(restL); dmg.fill(0); for (let i = 0; i < n; i++) { q[i * 4] = 1; q[i * 4 + 1] = q[i * 4 + 2] = q[i * 4 + 3] = 0; }
    for (const js of inc) for (const j of js) { W.L0[j] = W.Lr[j]; W.alive[j] = 1; W.pl[j] = 0; }
    C.hubD = WHEELS.map(h => h.slice()); C.lost = [0, 0, 0, 0]; C.dirty = true; C.impacts = []; C.crashT = -1; C.crack = 0; C.crackN = 0; C.glassN = 0; C.crackP = null; C.wreck = false; C.settled = false;
  };
  C.update = (dt, t) => C.mode === 'rigid' ? rigid(C, dt, t) : soft(C, dt, t);
  C.goSoft = () => goSoft(C);
  return C;
}
// ---------------- rigid driving
function rigid(C, dt, t) {
  if (C.frozen) return;
  // A resting wreck must not restart its suspension or wake on the barrier it already touches.
  if (C.settled) {
    if (!C.wreck && (C.throttle > 0 || C.tow?.on)) C.settled = false;
    else { for (const o of C.ext) if (o.near && o.near(C)) { goSoft(C); break; } return; }
  }
  if (C.wreck) { C.vel = [0, 0, 0]; C.ang = [0, 0, 0]; C.rpm = 0; if (touches(C, dt)) goSoft(C); return; } // lost a wheel: it lies where it stopped
  const N = 4, h = dt / N;
  for (let s = 0; s < N; s++) rigidStep(C, h);
  // engine/gearbox bookkeeping
  const vf = dot(C.vel, qrot(C.rot, [0, 0, 1])), wheelW = vf / WR;
  if (C.auto && !C.tow) {
    if (C.gear >= 2 && C.rpm > 5200 && C.gear < 5) C.gear++;
    else if (C.gear > 2 && C.rpm < 2200) C.gear--;
    if (C.throttle > 0 && vf < .5 && C.gear < 2) C.gear = 2;
    if (C.brake > 0 && vf < .3 && C.gear >= 2 && C.revIn) C.gear = 0;
  }
  const ratio = GEARS[C.gear] || 0;
  C.rpm = lerp(C.rpm, Math.max(IDLE + C.throttle * 2500 * (ratio ? clamp(1 - Math.abs(wheelW * ratio * 9.55) / 2500, 0, 1) : 1), Math.abs(wheelW * ratio * 9.55)), Math.min(1, dt * 8));
  C.fan += C.rpm / 60 * Math.PI * 2 * dt;
  // contact with obstacles -> soft
  if (touches(C, dt)) goSoft(C);
}
function rigidStep(C, h) {
  const R = C.rot, up = qrot(R, [0, 1, 0]), fw = qrot(R, [0, 0, 1]), side = qrot(R, [1, 0, 0]);
  const comW = C.world(C.com);
  let F = [0, -9.81 * C.M, 0], T = [0, 0, 0];
  const force = (f, p) => { F = add(F, f); T = add(T, cross(sub(p, comW), f)); };
  const sp = Math.abs(dot(C.vel, fw));
  // keyboard steering: full lock when parking, at speed only what the tyres can use (~0.9 g)
  const lock = Math.min(.6, 21 / Math.max(sp * sp, 1));
  C.steer = lerp(C.steer, C.steerIn * lock, Math.min(1, h * 4));
  const ratio = GEARS[C.gear] || 0, drive = C.tow ? 0 : torque(C.rpm) * ratio * C.throttle / WR;
  const cs = C.c.slice();
  for (let i = 0; i < 4; i++) {
    if (C.lost[i]) { C.contact[i] = 0; continue; }
    const Pd = C.world(C.hubD[i]);
    const gy = C.W.ground(Pd[0], Pd[2]);
    let cc = (gy + WR - Pd[1]) / Math.max(up[1], .2);
    if (up[1] < .3) cc = -1;
    C.contact[i] = cc > DROOP ? 1 : 0;
    cs[i] = clamp(cc, DROOP, BUMP + .05);
  }
  for (let i = 0; i < 4; i++) {
    const pair = i ^ 2; // same side, other axle (FL<->RL, FR<->RR)
    C.cPrev[i] = C.c[i]; C.c[i] = cs[i];
    if (!C.contact[i]) { C.load[i] = 0; continue; }
    const cv = (C.c[i] - C.cPrev[i]) / h;
    let Fs = C.F0s[i] + KC * (C.c[i] + C.c[pair]) + KA * C.c[i] + DAMP * cv;
    if (C.c[i] > BUMP) Fs += 9e4 * (C.c[i] - BUMP) + 3000 * Math.max(cv, 0);
    Fs = Math.max(Fs, 0); C.load[i] = Fs;
    const Pd = C.world(C.hubD[i]), cp = add(Pd, scl(up, C.c[i] - WR));
    const gn = C.W.groundN(cp[0], cp[2]);
    force(scl(gn, Fs * Math.max(dot(up, gn), .2)), cp);
    // tyre
    const st = i < 2 ? C.steer : 0, wf = norm(add(scl(fw, Math.cos(st)), scl(side, Math.sin(st)))), ws = cross(gn, wf);
    const vp = add(C.vel, cross(C.ang, sub(cp, comW)));
    const vl = dot(vp, wf), vs = dot(vp, ws);
    const Nn = Fs * Math.max(gn[1], .3), alpha = Math.atan2(vs, Math.max(Math.abs(vl), 1.2));
    let Fy = -MU * Nn * Math.tanh(alpha * 7);
    let Fx = 0;
    if (i < 2) Fx += drive / 2;
    if (C.tow && C.tow.on) Fx += i < 2 ? C.tow.f / 2 : 0;
    const bmax = C.brake * MU * Nn * (i < 2 ? 1.1 : .8) + (C.hand && i > 1 ? MU * Nn : 0);
    Fx -= clamp(vl * C.M / 4 / h * .5, -bmax, bmax) + vl * .015 * Nn / Math.max(Math.abs(vl), .5) * Math.min(Math.abs(vl), .5);
    const fm = Math.hypot(Fx, Fy), lim = MU * Nn;
    C.slip[i] = fm > lim ? (fm - lim) / (lim + 1) : Math.abs(vs) > 1.5 ? .3 : 0;
    if (fm > lim) { Fx *= lim / fm; Fy *= lim / fm; }
    force(scl(ws, Fy), cp);
    // longitudinal force goes into the body through the swinging arm: at the pivot height (~0.31 m), plus the arm's
    // couple on the spring. Leading arms in front: driving pulls the nose down, braking lifts it (anti-lift / anti-dive),
    // which is why a front-driven 2CV doesn't squat like a rear-driven car
    const arm = i < 2 ? -.1 : .1;
    force(scl(wf, Fx), add(cp, scl(up, .31)));
    force(scl(up, arm * Fx), add(cp, scl(up, .31)));
    C.wv[i] = vl / WR;
  }
  // aero drag (Cd 0.51, A ~1.75 m^2)
  const v = len(C.vel); if (v > .01) F = add(F, scl(C.vel, -.535 * v));
  // the tow sled holds the car on the guide rail
  if (C.tow && C.tow.on) { const lat = C.pos[0] - C.tow.x; F = add(F, [-lat * 8e4 - C.vel[0] * 6e3, 0, 0]); const yawErr = Math.atan2(fw[0], fw[2]); T = add(T, [0, -yawErr * 2e4 - C.ang[1] * 4e3, 0]); }
  // integrate (semi-implicit Euler)
  C.vel = add(C.vel, scl(F, h / C.M));
  const Rl = qrot(qconj(R), T), Wl = qrot(qconj(R), C.ang), Iw = [C.I[0] * Wl[0], C.I[1] * Wl[1], C.I[2] * Wl[2]], gyro = cross(Wl, Iw);
  const al = [(Rl[0] - gyro[0]) / C.I[0], (Rl[1] - gyro[1]) / C.I[1], (Rl[2] - gyro[2]) / C.I[2]];
  C.ang = add(C.ang, scl(qrot(R, al), h));
  const nc = add(comW, scl(C.vel, h));
  const wl = len(C.ang); if (wl > 1e-9) C.rot = qnorm(qmul(qaxis(C.ang, wl * h), C.rot));
  C.pos = sub(nc, qrot(C.rot, C.com));
  for (let i = 0; i < 4; i++) C.spin[i] += C.wv[i] * h;
}
// does the rigid car touch anything (analytic colliders, other bodies, its own roof on the ground)?
function touches(C, dt) {
  const W = C.W, cols = W.colliders, comW = C.world(C.com);
  const pts = [];
  for (let i = 0; i < C.n; i += 1) {
    if (C.L.nodes[i].cls === CLS.HUB) continue;
    const p = C.world([C.loc[i * 3], C.loc[i * 3 + 1], C.loc[i * 3 + 2]]), v = add(C.vel, cross(C.ang, sub(p, comW)));
    const q = add(p, scl(v, dt * 1.5));
    for (const c of cols) if (c.pen && c.pen(q[0], q[1], q[2], .05) > 0) return true;
    if (!C.wreck && q[1] < W.ground(q[0], q[2]) + .01 && C.L.nodes[i].p[1] > .5) return true;   // body on the ground: rollover
  }
  for (const o of C.ext) if (o.near && o.near(C)) return true;
  return false;
}
// rigid -> soft: lattice nodes get the rigid motion, wheels sit where the suspension had them
export function goSoft(C) {
  if (C.mode === 'soft') return;
  const W = C.W, comW = C.world(C.com);
  syncNodes(C, 1);
  C.mode = 'soft'; C.settled = false; W.awake[C.body] = 1; C.calm = 0; C.crashT = W.t;
  C.L.hubs.forEach((h, i) => { const ax = qrot(C.rot, [1, 0, 0]); W.aniso.set([ax[0], ax[1], ax[2] || 1e-9], h * 3); });
  C.fitQ = C.rot.slice(); C.fitC0 = null; initFit(C);
  for (const o of C.ext) o.onSoft && o.onSoft(C);
}
function syncNodes(C, withVel) {
  const W = C.W, comW = C.world(C.com);
  for (let i = 0; i < C.n; i++) {
    let lp = [C.loc[i * 3], C.loc[i * 3 + 1], C.loc[i * 3 + 2]];
    const hi = C.L.hubs.indexOf(C.base + i); if (hi >= 0) lp = add(C.hubD[hi], [0, C.c[hi], 0]);
    const p = C.world(lp), k = (C.base + i) * 3, v = withVel ? add(C.vel, cross(C.ang, sub(p, comW))) : [0, 0, 0];
    W.x[k] = W.p[k] = p[0]; W.x[k + 1] = W.p[k + 1] = p[1]; W.x[k + 2] = W.p[k + 2] = p[2];
    W.v[k] = v[0]; W.v[k + 1] = v[1]; W.v[k + 2] = v[2]; W.gnd[C.base + i] = -1e9;
  }
}
// ---------------- soft: the world is stepped by the caller; here we read the car back out of it
function initFit(C) { const n = C.n, m = C.mass; C.ref = C.loc.slice(); let M = 0, c = [0, 0, 0]; for (let i = 0; i < n; i++) { M += m[i]; for (let k = 0; k < 3; k++) c[k] += m[i] * C.ref[i * 3 + k]; } C.fitC0 = c.map(v => v / M); C.fitM = M; }
export function readSoft(C, dt, X = C.W.x, V = C.W.v, dmgSrc = null) {
  const W = C.W, n = C.n, b = C.base, m = C.mass;
  // best-fit rigid frame (shape matching against the shape the car had when it went soft)
  if (!C.fitC0) initFit(C);
  let cw = [0, 0, 0], vel = [0, 0, 0];
  for (let i = 0; i < n; i++) for (let k = 0; k < 3; k++) { cw[k] += m[i] * X[(b + i) * 3 + k]; vel[k] += m[i] * V[(b + i) * 3 + k]; }
  cw = cw.map(v => v / C.fitM); vel = vel.map(v => v / C.fitM);
  const A = [0, 0, 0, 0, 0, 0, 0, 0, 0], c0 = C.fitC0;
  for (let i = 0; i < n; i++) {
    const k = (b + i) * 3, px = X[k] - cw[0], py = X[k + 1] - cw[1], pz = X[k + 2] - cw[2], rx = C.ref[i * 3] - c0[0], ry = C.ref[i * 3 + 1] - c0[1], rz = C.ref[i * 3 + 2] - c0[2], mm = m[i];
    A[0] += mm * px * rx; A[1] += mm * py * rx; A[2] += mm * pz * rx; A[3] += mm * px * ry; A[4] += mm * py * ry; A[5] += mm * pz * ry; A[6] += mm * px * rz; A[7] += mm * py * rz; A[8] += mm * pz * rz;
  }
  const qq = new Float64Array(C.fitQ); extractRotation(A, qq, 0, 6); C.fitQ = Array.from(qq);
  const prevRot = C.rot; C.rot = C.fitQ.slice(); C.pos = sub(cw, qrot(C.rot, c0));
  const dq = qmul(C.rot, qconj(prevRot)); C.ang = scl(dq.slice(1), 2 * Math.sign(dq[0]) / Math.max(dt, 1e-4));
  C.vel = vel;
  const iq = qconj(C.rot);
  let maxRel = 0; const prev = C.locPrev || (C.locPrev = C.loc.slice());
  for (let i = 0; i < n; i++) {
    const k = (b + i) * 3, l = qrot(iq, [X[k] - C.pos[0], X[k + 1] - C.pos[1], X[k + 2] - C.pos[2]]);
    C.loc[i * 3] = l[0]; C.loc[i * 3 + 1] = l[1]; C.loc[i * 3 + 2] = l[2];
    maxRel += Math.hypot(l[0] - prev[i * 3], l[1] - prev[i * 3 + 1], l[2] - prev[i * 3 + 2]) / n / Math.max(dt, 1e-3);
  }
  prev.set(C.loc);
  nodeFramesL(C.loc, C.restL, C.adj, C.q, 1);
  // wheels roll: ground friction only across each hub's axle (the axle turns with the hub's node frame and the steer)
  if (!dmgSrc) C.L.hubs.forEach((h, i) => { const li = h - C.base, q = [C.q[li * 4], C.q[li * 4 + 1], C.q[li * 4 + 2], C.q[li * 4 + 3]];
    const ax = C.lost[i] ? [0, 0, 0] : qrot(C.rot, qrot(q, i < 2 ? [Math.cos(C.steer), 0, -Math.sin(C.steer)] : [1, 0, 0])); W.aniso[h * 3] = ax[0]; W.aniso[h * 3 + 1] = ax[1]; W.aniso[h * 3 + 2] = C.lost[i] ? 0 : ax[2] || 1e-9; });
  if (dmgSrc) C.dmg.set(dmgSrc); else for (let i = 0; i < n; i++) {
    let s = 0, shattered = false;
    for (const j of incOf(C, i)) {
      if (W.bm[j] === 1 && !W.alive[j]) shattered = true;
      if (W.bk[j] > 1e5 && W.bm[j] !== 1) s += W.pl[j] + (W.alive[j] ? 0 : .3);
    }
    C.dmg[i] = C.L.nodes[i].cls === CLS.GLASS ? (shattered ? 1 : 0) : Math.min(1, s * .6);
  }
  // wheels: a hub that left its arm is lost
  if (!dmgSrc) C.L.hubs.forEach((h, i) => { const d = Math.hypot(C.loc[(h - b) * 3] - WHEELS[i][0], C.loc[(h - b) * 3 + 2] - WHEELS[i][2]); if (d > .5) C.lost[i] = 1; });
  C.dirty = true;
  C.relV = maxRel; C.speed = len(vel);
  C.calm = maxRel < .05 && C.speed < .4 ? C.calm + dt : 0;
}
const incCache = new WeakMap();
function incOf(C, i) {
  let inc = incCache.get(C);
  if (!inc) { inc = Array.from({ length: C.n }, () => []); for (let j = 0; j < C.W.nb; j++) { const a = C.W.ba[j] - C.base, b = C.W.bb[j] - C.base; if (a >= 0 && a < C.n) inc[a].push(j); if (b >= 0 && b < C.n) inc[b].push(j); } incCache.set(C, inc); }
  return inc[i];
}
function soft(C, dt) {
  readSoft(C, dt);
  // roll the wheels with the ground
  const fw = qrot(C.rot, [0, 0, 1]);
  C.L.hubs.forEach((h, i) => { const k = h * 3, v = [C.W.v[k], C.W.v[k + 1], C.W.v[k + 2]]; C.spin[i] += dot(v, fw) / WR * dt; });
  C.rpm = lerp(C.rpm, 0, dt * 2); C.fan += C.rpm / 60 * 6.28 * dt;
  if (C.calm > .5) goRigid(C);
}
export function goRigid(C) {
  const W = C.W;
  C.mode = 'rigid'; W.awake[C.body] = 0;
  C.vel = scl(C.vel, C.speed < 1 ? 0 : 1); C.ang = [0, 0, 0];
  C.settled = C.speed < .4;
  if (C.settled) { C.rpm = 0; C.speed = 0; }
  C.wreck = C.lost.some(Boolean) || qrot(C.rot, [0, 1, 0])[1] < .7; // on its side or missing a wheel: no driving on
  for (const o of C.ext) o.onRigid && o.onRigid(C);
  // the deformed shape is the new rigid shape; hubs stay where the crash left them
  C.L.hubs.forEach((h, i) => { const k = (h - C.base) * 3; C.hubD[i] = [C.loc[k], C.loc[k + 1], C.loc[k + 2]]; C.c[i] = C.cPrev[i] = 0; });
  // re-centre the hubs vertically so the car keeps its ride height (springs carry the static load at c = 0)
  const comW = C.world(C.com);
}
// ---------------- render data
export function carBones(C) {
  const B = new Float32Array(17 * 16), A = new Float32Array(16 * 3), body = qm4(C.rot, C.pos);
  const I4 = qm4([1, 0, 0, 0], [0, 0, 0]);
  for (let i = 0; i < 17; i++) B.set(body, i * 16);
  // Each seat is a rigid frame carried by a patch of floor, not separately skinned pipe vertices.
  C.L.seatMounts.forEach((ids, i) => {
    const center = arr => ids.reduce((p, id) => add(p, scl(Array.from(arr.slice(id * 3, id * 3 + 3)), 1 / ids.length)), [0, 0, 0]);
    const rest = center(C.restL), cur = center(C.loc), cov = new Float64Array(9);
    for (const id of ids) {
      const a = sub(Array.from(C.loc.slice(id * 3, id * 3 + 3)), cur), b = sub(Array.from(C.restL.slice(id * 3, id * 3 + 3)), rest);
      for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) cov[j * 3 + k] += a[k] * b[j];
    }
    const q = new Float64Array([1, 0, 0, 0]); extractRotation(cov, q, 0, 10);
    B.set(mul(body, mul(qm4(q, cur), qm4([1, 0, 0, 0], scl(rest, -1)))), (14 + i) * 16);
  });
  const wloc = i => C.mode === 'soft' ? [C.loc[(C.L.hubs[i] - C.base) * 3], C.loc[(C.L.hubs[i] - C.base) * 3 + 1], C.loc[(C.L.hubs[i] - C.base) * 3 + 2]] : add(C.hubD[i], [0, C.c[i], 0]);
  for (let i = 0; i < 4; i++) {
    const wl = wloc(i), st = i < 2 ? C.steer : 0;
    const r = qmul(qaxis([0, 1, 0], st), qaxis([1, 0, 0], C.spin[i]));
    const m = mul(qm4(r, wl), qm4([1, 0, 0, 0], scl(WHEELS[i], -1)));
    B.set(mul(body, m), (1 + i) * 16);
    // arm: rotate about its pivot towards the wheel
    const pv = C.L.piv[i][0] - C.base, pl = [C.loc[pv * 3], C.loc[pv * 3 + 1], C.loc[pv * 3 + 2]], pr = [C.restL[pv * 3], C.restL[pv * 3 + 1], C.restL[pv * 3 + 2]];
    const rq = qfromto(norm(sub(WHEELS[i], pr)), norm(sub(wl, pl)));
    B.set(mul(body, mul(qm4(rq, pl), qm4([1, 0, 0, 0], scl(pr, -1)))), (8 + i) * 16);
    if (i < 2) { const inner = [i ? -.18 : .18, .45, 1.13], dq = qfromto(norm(sub(WHEELS[i], inner)), norm(sub(wl, inner))); B.set(mul(body, mul(qm4(dq, inner), qm4([1, 0, 0, 0], scl(inner, -1)))), (12 + i) * 16); }
  }
  // pre-skin animation (rest space): steering wheel, fan, gear lever
  const about = (c, q) => mul(qm4(q, c), qm4([1, 0, 0, 0], scl(c, -1)));
  A.set(about(SWHEEL.c, qaxis(SWHEEL.n, -C.steer * 11)), 0);
  A.set(about([0, .52, 1.66], qaxis([0, 0, 1], C.fan)), 16);
  A.set(about([.08, .93, .5], qaxis([0, 1, 0], [-.25, 0, -.2, -.2, .2, .2][C.gear] || 0)), 32);
  return { B, A };
}
