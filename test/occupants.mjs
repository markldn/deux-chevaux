import assert from 'node:assert/strict';
import { createSim, TESTS } from '../src/sim.js';
import { dummyBones } from '../src/dummy.js';
import { len, sub } from '../src/math.js';

const S = createSim();
for (const [layout, count] of [['driver', 1], ['pair', 2], ['family', 4], ['adults', 4]]) {
  S.start(TESTS[0], 40, { occupants: layout });
  assert.equal(S.occupants.filter(d => d.enabled).length, count);
  for (let f = 0; f < 14; f++) {
    S.advance(1 / 60);
    assert(S.D.attached, layout + ': dummy released before collision');
    const chest = S.D.ids[2] * 3, head = S.D.ids[6] * 3;
    const distance = len(sub(Array.from(S.W.x.slice(head, head + 3)), Array.from(S.W.x.slice(chest, chest + 3))));
    assert(Math.abs(distance - len(sub(S.D.rest[6], S.D.rest[2]))) < 1e-8, layout + ': head slumped on approach');
  }
  for (let f = 0; f < 240 && S.phase !== 'done'; f++) S.advance(1 / 60);
  assert(S.report, layout + ': missing report');
  assert.equal(S.report.occupants.length, count);
  for (const d of S.occupants.filter(d => d.enabled)) {
    assert(Array.from(dummyBones(d, S.A.rot)).every(Number.isFinite), layout + ': invalid bone matrix');
    assert(d.rec.length > 0, layout + ': occupant never became dynamic');
  }
  assert(S.startReplay(.1, Math.floor(S.rec.length / 2)));
  const replay = S.replayFrame(0);
  for (const d of S.occupants.filter(d => d.enabled)) {
    assert(Array.from(replay.X.slice(d.ids[0] * 3, (d.ids.at(-1) + 1) * 3)).every(Number.isFinite));
    assert(Array.from(dummyBones(d, S.A.rot, replay.X)).every(Number.isFinite));
  }
  S.stopReplay();
  console.log(layout + ': approach pose, occupant dynamics, injury recording and replay passed');
}
S.start(TESTS[5], 50, { runup: 25 });
assert.equal(S.phase, 'approach');
const z0 = S.A.pos[2];
for (let f = 0; f < 60; f++) S.advance(1 / 60);
assert.equal(S.phase, 'approach', 'head-on crashed during its first second');
assert(S.A.pos[2] > z0 + 10, 'head-on car did not drive forward');
assert.equal(S.rec.length, 0, 'head-on impact recorded during approach');
console.log('Head-on has a visible moving approach before collision');
