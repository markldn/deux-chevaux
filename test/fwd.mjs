// front-wheel-drive behaviour: pitch under acceleration, and throttle in a steady bend (FWD should push wide)
import { createSim } from '../src/sim.js';
import { qrot } from '../src/math.js';
const mk = (v) => { const S = createSim(), A = S.A, W = S.W; S.start({ id: 'd', set: 'drive' }, 0, { dummy: false }); S.stopReplay(); A.mode = 'rigid'; W.awake[1] = W.awake[2] = 0; S.setBarrier('none'); A.place([0, 0, -200], 0, v / 3.6); A.gear = v > 30 ? 4 : 2; return A; };
const pitch = A => Math.asin(qrot(A.rot, [0, 0, 1])[1]) * 57.3;
let A = mk(0); let mp = 0;
for (let f = 0; f < 240; f++) { A.throttle = 1; A.steerIn = 0; A.update(1 / 60, 0); mp = Math.max(mp, pitch(A)); }
console.log('full throttle from rest: max nose-up pitch', mp.toFixed(2), '°, front loads', A.load.slice(0, 2).map(Math.round), 'rear', A.load.slice(2).map(Math.round));
for (const thr of [0, 1]) {
  A = mk(45); let yr = [], slip = [];
  for (let f = 0; f < 360; f++) { A.steerIn = .5; const v = Math.hypot(A.vel[0], A.vel[2]) * 3.6; A.throttle = f < 180 ? (v < 45 ? .5 : 0) : thr; A.brake = 0; A.update(1 / 60, 0);
    if (f > 170 && f % 30 == 0) { const fw = qrot(A.rot, [0, 0, 1]); const beta = Math.atan2(A.vel[0] * fw[2] - A.vel[2] * fw[0], A.vel[0] * fw[0] + A.vel[2] * fw[2]) * 57.3; yr.push(A.ang[1].toFixed(2)); slip.push(beta.toFixed(1)); } }
  console.log(`bend, then throttle=${thr}: yaw rate`, yr.join(' '), '| body slip angle °', slip.join(' '));
}
