// A wreck touching its original obstacle should sleep without suspension/soft-body cycling.
import assert from 'node:assert/strict';
import { createSim, TESTS } from '../src/sim.js';

for (const id of ['wall76', 'wall', 'odb', 'headon']) {
  const S = createSim(); S.start(TESTS.find(t => t.id === id), id === 'wall76' ? 40 : id === 'wall' ? 56 : id === 'odb' ? 64 : 50);
  let pose = null, held = 0;
  for (let f = 0; f < 540; f++) {
    S.advance(1 / 60);
    if (S.A.settled && !pose) pose = [...S.A.pos, ...S.A.rot];
    if (pose) { assert.deepEqual([...S.A.pos, ...S.A.rot], pose, id + ': resting wreck moved'); held++; }
  }
  assert(held > 60, id + ': did not sleep for at least a second');
  assert(S.D.attached, id + ': dummy remained awake');
  assert.equal(S.W.awake[S.A.body], 0, id + ': body remained awake');
  if (!S.A.wreck) { S.A.throttle = 1; S.advance(1 / 60); assert(!S.A.settled, id + ': driving input did not wake the car'); }
  console.log(id + ': resting pose held, dummy slept, driving wake checked');
}
