// Run every crash test headless and print the report: node test/tests.mjs [id]
import { createSim, TESTS } from '../src/sim.js';
const S = createSim();
for (const T of TESTS.filter(t => !process.argv[2] || t.id === process.argv[2])) {
  const t0 = Date.now(); S.start(T, T.kmh, { runup: T.id === 'roll' ? 20 : 0, dummy: !process.env.NODUMMY });
  let f = 0; while (S.phase !== 'done' && f < 60 * 12) { S.advance(1 / 60); f++; }
  const r = S.report;
  const p = S.A.world(S.A.com);
  console.log(`${T.id.padEnd(7)} ${String(T.kmh).padStart(3)} km/h  ${r ? `pulse ${r.g.toFixed(0)} g  crush ${r.crush.toFixed(0)} mm  HIC ${r.hic.toFixed(0)}  head ${r.head.toFixed(0)} g  chest ${r.chest.toFixed(0)} g  belt ${r.belt.toFixed(0)} kgf  bulkhead ${r.intr.toFixed(0)} mm  glass ${r.glass}  rec ${S.rec.length}` : 'NO REPORT phase ' + S.phase}  end pos ${p.map(v => v.toFixed(1))} up ${S.A.rot && (1 - 2 * (S.A.rot[1] ** 2 + S.A.rot[3] ** 2)).toFixed(2)}  ${f} frames ${Date.now() - t0} ms`);
}
