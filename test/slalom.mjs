// handling: alternate full steering input every `per` s at a speed; report max roll and whether it tipped
import { createSim } from '../src/sim.js';
for (const [kmh, per] of [[50, 1], [70, 1], [90, .8], [110, 1.2]]) {
  const S = createSim(), A = S.A, W = S.W;
  S.start({ id: 'd', set: 'drive' }, 0, { dummy: false }); S.stopReplay(); A.mode = 'rigid'; W.awake[1] = W.awake[2] = 0; S.setBarrier('none');
  A.place([0, 0, -280], 0, kmh / 3.6); A.gear = 5;
  let maxRoll = 0, tip = false, maxLat = 0;
  for (let f = 0; f < 60 * 8; f++) {
    const t = f / 60; A.steerIn = Math.sign(Math.sin(t / per * Math.PI)); 
    const v = Math.hypot(A.vel[0], A.vel[2]) * 3.6; A.throttle = v < kmh ? .7 : 0;
    A.update(1 / 60, 0);
    const side = qrot(A.rot, [1, 0, 0]), roll = Math.asin(Math.max(-1, Math.min(1, side[1]))) * 57.3;
    maxRoll = Math.max(maxRoll, Math.abs(roll)); if (A.mode === 'soft' || Math.abs(roll) > 60) { tip = true; break; }
    if (A.pos[2] > 50 || Math.abs(A.pos[0]) > 60) A.place([0, 0, -280], 0, kmh / 3.6);
  }
  console.log(`${kmh} km/h slalom ${per}s: max roll ${maxRoll.toFixed(1)}°  ${tip ? 'ROLLED / WENT SOFT' : 'ok'}`);
}
import { qrot } from '../src/math.js';
