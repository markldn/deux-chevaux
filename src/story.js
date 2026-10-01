// The film. Every shot drives the live simulation: the avenue and the ploughed field are the car on autopilot,
// the 1976 test and the offset test are real runs of the soft-body model, the replays are their recordings.
import { TESTS, REF76 } from './sim.js';
import { AVE_Z, FIELD } from './world.js';
import { add, sub, scl, len, norm, cross, dot, qrot, clamp, lerp, smooth } from './math.js';

const L3 = (a, b, t) => a.map((v, i) => lerp(v, b[i], t));
export function createFilm(api) {
  const { S, A, D, view } = api;
  let t = 0, k = -1, st = 0, paused = false, rp = null, done = false;
  const F = { marks: 0, hs: 0 };
  // a shot that replays a test needs its recording: if we arrived by seeking, compute it now
  const ensure = (test, kmh) => { if (S.report && S.report.test === test && S.rec.length > 10) return; S.start(test, kmh, { runup: 0 }); for (let f = 0; f < 600 && S.phase !== 'done'; f++) S.advance(1 / 60); };
  // autopilot: hold a line towards `to` at `kmh`
  const drive = (to, kmh, dt) => {
    const p = A.world(A.com), f = qrot(A.rot, [0, 0, 1]), d = sub(to, p), want = Math.atan2(d[0], d[2]), have = Math.atan2(f[0], f[2]);
    let e = want - have; e = Math.atan2(Math.sin(e), Math.cos(e));
    A.steerIn = clamp(e * 2.5, -1, 1);
    const v = dot(A.vel, f) * 3.6; A.throttle = v < kmh ? clamp((kmh - v) / 8, .2, 1) : 0; A.brake = v > kmh + 6 ? .3 : 0;
    S.advance(dt);
  };
  const place = (p, yaw, v, paint, tod) => { S.start({ id: 'film', set: 'drive' }, 0, { runup: 0, dummy: false }); S.stopReplay(); S.recOn = false; A.mode = 'rigid'; S.phase = 'free'; S.setBarrier('none'); A.place(p, yaw, v / 3.6); A.gear = 4; view.paint = paint; view.tod = tod; };
  const LABELS = [
    [[0, .58, 1.5], 2, '602 cm³ air-cooled flat twin', '29 hp at 5,750 rpm, 39 N·m'], [[0, .52, 1.7], 2, 'cooling fan on the crank', 'no radiator, no water, no thermostat'],
    [[-.16, .45, 1.13], 2, 'inboard front disc brakes', 'on the gearbox: lighter wheels'], [[0, .78, 1.05], 2, 'spare wheel', 'under the bonnet'],
    [[.56, .25, 0], 3, 'interconnected suspension', 'one spring canister per side couples front and rear'], [[.5, .3, .9], 3, 'leading arm', ''], [[.5, .3, -.95], 3, 'trailing arm', ''],
    [[0, .25, -.6], 1, 'platform chassis', 'the body is bolted on top'], [[.33, .6, -.3], 5, 'tubular seats', 'canvas on rubber bands'], [[.08, .98, .32], 5, 'umbrella-handle gear lever', 'push-pull, out of the dashboard'],
    [[.33, 1.0, .24], 5, 'single-spoke steering wheel', ''], [[0, 1.6, -.6], 0, 'roll-back canvas roof', 'down to the boot lid'], [[-.63, .29, -1.2], 4, '125 R 15 radial tyres', 'on 3-stud steel wheels'],
  ];
  let labelA = 0, labelT = 0, card = 0;
  const SHOTS = [
    { d: 15, tod: .11, enter() { place([-170, 0, AVE_Z - 1.8], Math.PI / 2, 60, 2, .11); view.roof = 0; view.lights = 1; },
      frame(u, dt) { drive([400, 0, AVE_Z - 1.8], 62, dt); const p = A.world(A.com);
        const cam = u < .5 ? { pos: add(p, [-14 + u * 12, 9 - u * 10, 9]), tgt: add(p, [2, 0, 0]), fov: .6 } : { pos: [p[0] + 28 - (u - .5) * 20, .6, AVE_Z + 4.5], tgt: add(p, [0, .3, 0]), fov: .45 };
        return { cam, title: smooth(.1, .2, u) * smooth(.98, .8, u), sub: '1948 – 1990 · 3.9 million built · the car that put France on wheels' }; } },
    { d: 17, tod: .3, enter() { place([FIELD[0] - 25, 0, -100], Math.PI / 2, 28, 2, .3); view.lights = 0; },
      frame(u, dt) { drive([FIELD[1] + 30, 0, -100], 28, dt); const p = A.world(A.com);
        const cam = u < .55 ? { pos: [p[0] + 6, .45, p[2] - 3.2], tgt: add(p, [0, .1, 0]), fov: .7 } : { pos: add(p, [-1.5, .35, -3.6]), tgt: add(p, [.5, .2, 0]), fov: .9 };
        F.xray = u > .55 ? 1 : 0;
        return { cam, cap: u < .55 ? '1936. The brief: carry two farmers and 50 kg of potatoes at 60 km/h, on 3 litres per 100 km — across a ploughed field, without breaking a basket of eggs.' : 'The answer was the softest suspension in Europe: long arms, front and rear springs sharing one canister per side.' }; } },
    { d: 30, tod: .74, enter() { place([0, 0, -26], .6, 0, 1, .74); A.frozen = true; labelA = 0; },
      frame(u, dt) { const p = A.world(A.com), a = .6 + u * 4.2, dd = 6.2 - smooth(.35, .55, u) * 1.2;
        view.explode = smooth(.08, .3, u) * (1 - smooth(.8, .95, u));
        F.xray = u > .55 && u < .72 ? 1 : 0; labelA = smooth(.25, .3, u) * (1 - smooth(.9, .95, u)); labelT = clamp((u - .27) / .64, 0, 1);
        const low = smooth(.6, .7, u) * (1 - smooth(.76, .8, u));
        const cam = { pos: add(p, [Math.sin(a) * dd, 1.6 + view.explode * 1.2 - low * 1.75, Math.cos(a) * dd]), tgt: add(p, [0, .4 + view.explode * .5 - low * .1, 0]), fov: .7 };
        return { cam, cap: u < .25 ? 'Anatomy of a 2CV6: 560 kg, 3.83 m long, built like an umbrella.' : '' }; } },
    { d: 15, tod: .5, enter() { view.explode = 0; view.paint = 0; F.marks = 1; S.start(TESTS[0], 40, { runup: 34 }); },
      frame(u, dt) { S.advance(dt); const cam = S.phase === 'tow' ? api.camFor('track', dt) : api.camFor('side', dt); F.hs = S.phase !== 'tow';
        return { cam, cap: u < .55 ? '1976. A German magazine tows seven small cars into a concrete wall at 40 km/h, each with a belted dummy.' : '' }; } },
    { d: 24, tod: .5, enter() { ensure(TESTS[0], 40); view.paint = 0; F.marks = 1; const w = api.replayWindow(); this.w = w; S.startReplay(.075, w[0]); },
      frame(u, dt) { const cams = ['side', 'pit', 'onboard', 'top'], ci = Math.min(3, Math.floor(u * 4)), uu = u * 4 - ci;
        if (S.replay) { S.replay.i = lerp(this.w[0], this.w[1] - 40, uu); rp = S.replayFrame(0); }
        F.hs = 1; return { cam: api.camFor(cams[ci], dt), cap: ['Slowed 13 times. The engine sits ahead of the axle: it reaches the wall first.', 'From the camera pit: the platform stays straight while the nose folds.', 'The belt holds; his head still meets the wheel.', 'Every frame is the simulation re-run of the real test, not an animation.'][ci] }; } },
    { d: 14, tod: .5, enter() { S.stopReplay(); ensure(TESTS[0], 40); },
      frame(u, dt) { card = smooth(0, .1, u) * smooth(1, .9, u); F.hs = 0; return { cam: api.camFor('orbit', dt), cap: '' }; } },
    { d: 22, tod: .58, enter() { api.labRun(TESTS[2], 64); },
      frame(u, dt) { F.hs = 1; if (!S.replay && S.phase !== 'done') { S.advance(dt); return { cam: api.camFor('front', dt), cap: 'Today\'s offset test runs at 64 km/h into crushable aluminium honeycomb.' }; }
        if (!S.replay) { const w = api.replayWindow(); this.w = w; this.u0 = u; S.startReplay(.07, w[0]); }
        const uu = clamp((u - this.u0) / (1 - this.u0), 0, 1), ci = uu < .5 ? 0 : 1; S.replay.i = lerp(this.w[0], this.w[1] - 30, (uu * 2) % 1); rp = S.replayFrame(0);
        const r = S.report; return { cam: api.camFor(ci ? 'onboard' : 'front', dt), cap: r ? `Head injury criterion ${Math.round(r.hic)} · chest ${Math.round(r.chest)} g · footwell intrusion ${Math.round(r.intr)} mm` : '' }; } },
    { d: 9, tod: .86, enter() { S.stopReplay(); },
      frame(u, dt) { F.hs = 0; return { cam: api.camFor('orbit', dt), title: smooth(0, .2, u), sub: 'Your turn: the crash lab and the open road are in the menu.', fade: 1 - smooth(.85, 1, u) }; } },
  ];
  const total = SHOTS.reduce((a, s) => a + s.d, 0);
  F.frame = dt => {
    if (!paused) t += dt;
    let a = 0, i = 0; while (i < SHOTS.length - 1 && t >= a + SHOTS[i].d) a += SHOTS[i++].d;
    if (i !== k) { k = i; st = a; F.xray = 0; labelA = 0; card = 0; view.explode = 0; rp = null; A.frozen = false; F.hs = 0; if (i < 3) F.marks = 0; SHOTS[i].enter(); view.tod = SHOTS[i].tod; }
    const sh = SHOTS[i], u = clamp((t - st) / sh.d, 0, 1);
    rp = null;
    const o = sh.frame(u, paused ? 0 : dt);
    view.xray = F.xray || 0;
    document.getElementById('title').style.opacity = o.title || 0;
    document.getElementById('tsub').textContent = o.sub || '';
    const edge = Math.min(t - st, a + sh.d - t + (i === SHOTS.length - 1 ? 9 : 0));
    o.cam.near = o.cam.near || .05; o.cam.far = 5000;
    return { cam: o.cam, cap: o.cap, rp, post: { fade: Math.min(1, edge / .6) * (o.fade ?? 1), bars: .06, sat: 1.12, grain: .045, hs: F.hs ? .6 : 0 }, done: t > total };
  };
  F.key = e => {
    const starts = []; let a = 0; for (const s of SHOTS) { starts.push(a); a += s.d; }
    if (e.code === 'ArrowRight') t = starts[Math.min(k + 1, SHOTS.length - 1)] + .01;
    if (e.code === 'ArrowLeft') t = starts[Math.max(k - (t - starts[k] < 1.5 ? 1 : 0), 0)] + .01;
    if (e.code === 'Space') paused = !paused;
  };
  F.seek = x => { t = x; }; F.total = total;
  // labels for the anatomy shot and the result card
  F.overlay = (ctx, m, s, proj) => {
    const Wd = ctx.canvas.width, H = ctx.canvas.height;
    if (labelA > .01) {
      ctx.font = `${19 * s}px Georgia,serif`; ctx.textAlign = 'left';
      const e = view.explode, off = [[0, 1.25 * e, 0], [0, -.05 * e, 0], [0, .45 * e, .55 * e], [0, -.18 * e, 0], [.45 * e, 0, 0], [0, .62 * e, 0]];
      LABELS.forEach(([p, part, a, b], li) => {
        const ph = labelT * (LABELS.length + 2.6) - li, la = clamp(Math.min(ph, 2.6 - ph) * 3, 0, 1); if (la <= 0) return; ctx.globalAlpha = labelA * la;
        const o = part === 4 ? [Math.sign(p[0]) * .45 * e, 0, 0] : off[part]; const q = proj(m.VP, A.world(add(p, o))); if (!q) return;
        const dx = q[0] < Wd / 2 ? -60 * s : 60 * s, x2 = q[0] + dx, y2 = q[1] - 40 * s;
        ctx.strokeStyle = 'rgba(255,230,170,.8)'; ctx.lineWidth = 1.2 * s; ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(x2, y2); ctx.lineTo(x2 + Math.sign(dx) * 14 * s, y2); ctx.stroke();
        ctx.fillStyle = '#ffd877'; ctx.beginPath(); ctx.arc(q[0], q[1], 3 * s, 0, 7); ctx.fill();
        ctx.textAlign = dx < 0 ? 'right' : 'left'; ctx.fillStyle = '#fff4dc'; ctx.fillText(a, x2 + Math.sign(dx) * 18 * s, y2 + 4 * s);
        if (b) { ctx.fillStyle = 'rgba(255,240,215,.7)'; ctx.font = `italic ${14 * s}px Georgia,serif`; ctx.fillText(b, x2 + Math.sign(dx) * 18 * s, y2 + 22 * s); ctx.font = `${19 * s}px Georgia,serif`; }
      });
      ctx.globalAlpha = 1;
    }
    if (card > .01) {
      const r = S.report; if (!r) return;
      const w = 560 * s, h = 330 * s, x = (Wd - w) / 2, y = (H - h) / 2;
      ctx.globalAlpha = card; ctx.fillStyle = 'rgba(12,11,10,.82)'; ctx.fillRect(x, y, w, h); ctx.strokeStyle = 'rgba(255,220,140,.35)'; ctx.strokeRect(x, y, w, h);
      ctx.fillStyle = '#fff3d6'; ctx.font = `${24 * s}px Georgia,serif`; ctx.textAlign = 'center'; ctx.fillText('Simulation vs. the 1976 test', Wd / 2, y + 44 * s);
      ctx.font = `${16 * s}px Georgia,serif`;
      const rows = [['car peak deceleration', r.g, REF76.g, 'g'], ['static crush', r.crush, REF76.crush, 'mm'], ['head injury criterion', r.hic, REF76.hic, ''], ['head', r.head, REF76.head, 'g'], ['chest', r.chest, REF76.chest, 'g'], ['shoulder belt', r.belt, REF76.belt, 'kgf']];
      ctx.fillStyle = 'rgba(255,240,215,.6)'; ctx.textAlign = 'right'; ctx.fillText('simulated', x + w * .72, y + 84 * s); ctx.fillText('measured', x + w * .93, y + 84 * s);
      rows.forEach(([n, a, b, u], i) => { const yy = y + (116 + i * 34) * s; ctx.textAlign = 'left'; ctx.fillStyle = '#fff3d6'; ctx.fillText(n, x + 28 * s, yy); ctx.textAlign = 'right'; ctx.fillStyle = '#ffcc33'; ctx.fillText(`${Math.round(a)} ${u}`, x + w * .72, yy); ctx.fillStyle = '#fff3d6'; ctx.fillText(`${b} ${u}`, x + w * .93, yy); });
      ctx.globalAlpha = 1;
    }
  };
  return F;
}
