// Debris, glass and dust as spawn events with analytic ballistic paths: any moment can be rebuilt exactly, so a
// slow-motion replay shows the same shards in the same places.
const G = 9.81;
export function createFX() {
  const F = { ev: [], P: new Float32Array(4096 * 4), C: new Float32Array(4096 * 4), n: 0 };
  let seed = 1; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  F.reset = () => { F.ev.length = 0; seed = 1; };
  // kind 0 glass, 1 paint chip, 2 dust, 3 plastic
  F.spawn = (t, p, v, kind, col) => {
    if (F.ev.length > 3800) return;
    const s = kind === 2 ? 1 : 3;
    F.ev.push({ t, p: p.slice(), v: [v[0] + (rnd() - .5) * s, v[1] + rnd() * s * .8, v[2] + (rnd() - .5) * s], kind, col, sz: kind === 2 ? .6 + rnd() * .5 : kind === 0 ? .012 + rnd() * .018 : .02 + rnd() * .03, r: rnd() });
  };
  F.pack = (T, ground) => {
    let n = 0;
    for (const e of F.ev) {
      let dt = T - e.t; if (dt < 0) continue;
      if (e.kind === 2) { // dust: drifts, grows, fades
        if (dt > 3) continue;
        const k = 1 - Math.exp(-dt * 3);
        F.P.set([e.p[0] + e.v[0] * k * .3, e.p[1] + .3 * k + dt * .15, e.p[2] + e.v[2] * k * .3, e.sz * (1 + dt * 2.5)], n * 4);
        F.C.set([e.col[0], e.col[1], e.col[2], .22 * Math.max(0, 1 - dt / 3)], n * 4); n++; continue;
      }
      if (dt > 12) continue;
      const gy = ground(e.p[0], e.p[2]) + .005;
      // time to land: p.y + v.y t - g t^2 / 2 = gy
      const a = -G / 2, b = e.v[1], c = e.p[1] - gy, tl = (-b - Math.sqrt(Math.max(0, b * b - 4 * a * c))) / (2 * a);
      const tt = Math.min(dt, tl), slide = Math.max(0, dt - tl), sk = slide > 0 ? (1 - Math.exp(-slide * 5)) * .15 : 0;
      const x = e.p[0] + e.v[0] * (tt + sk), z = e.p[2] + e.v[2] * (tt + sk), y = slide > 0 ? gy : e.p[1] + e.v[1] * tt - G / 2 * tt * tt;
      const tw = e.kind === 0 ? .55 + .45 * Math.sin(dt * 40 + e.r * 20) : 1;
      F.P.set([x, y, z, e.sz], n * 4); F.C.set([e.col[0] * tw, e.col[1] * tw, e.col[2] * tw, 1], n * 4); n++;
      if (n >= 4096) break;
    }
    F.n = n; return F;
  };
  return F;
}
