// Full 1976 replication with the dummy: node test/calib2.mjs [kmh]
import { World, step, boxCollider, setActive } from '../src/soft.js';
import { createCar, readSoft } from '../src/vehicle.js';
import { createDummy } from '../src/dummy.js';
import { CLS } from '../src/lattice.js';
const kmh = +(process.argv[2] || 40);
const W = World(); W.ground = () => 0;
const car = createCar(W, 1, { payload: 78 });
const D = createDummy(W, 2, car); setActive(W);
W.colliders.push(boxCollider([0, 1, 1.5], [2.5, 1.5, 1.5], 0, { mu: .4 }));
car.place([0, 0, -2.05 - .02], 0, kmh / 3.6);
// settle the dummy into the seat at speed (no wall yet: run with the car rigid)
D.follow(); car.goSoft(); D.release();
W.onSub = h => D.sample(h);
const sw0 = [...car.loc.slice((car.L.sw - car.base) * 3, (car.L.sw - car.base) * 3 + 3)];
const t0 = Date.now();
const X = i => [W.x[i * 3], W.x[i * 3 + 1], W.x[i * 3 + 2]], dd = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
let minHW = 9, log = [], back = 0;
for (let f = 0; f < 60 * .6; f++) { step(W, 1 / 60, 17); readSoft(car, 1 / 60);
  const hw = dd(X(D.ids[6]), X(car.L.sw)); minHW = Math.min(minHW, hw); back = Math.max(back, sw0[2] - car.loc[(car.L.sw - car.base) * 3 + 2]);
  if (f % 2 == 0) log.push(`${(W.t*1000).toFixed(0)}ms cw ${dd(X(D.ids[2]), X(car.L.sw)).toFixed(2)} pz ${(X(D.ids[0])[2]-X(car.L.nodes[400].id)[2]).toFixed(2)} cz ${(X(D.ids[2])[2]-X(car.L.nodes[400].id)[2]).toFixed(2)} hw ${hw.toFixed(2)} hfwd ${(X(D.ids[6])[2]-X(car.L.nodes[0].id)[2]).toFixed(2)} belt ${(D.diag.f/9.81).toFixed(0)} lap ${(D.lap.f/9.81).toFixed(0)} head ${(D.fh||0).toFixed(0)}g chest ${(D.fc||0).toFixed(0)}g`); }
console.log(log.join('\n')); console.log('min head-wheel dist', minHW.toFixed(3));
const m = D.compute();
const swz = car.loc[(car.L.sw - car.base) * 3 + 2];
const head = D.ids[6] * 3;
console.log(`${kmh} km/h  HIC15 ${m.hic.toFixed(0)}  head(3ms) ${m.head3.toFixed(0)} g peak ${m.headPeak.toFixed(0)} g  chest(3ms) ${m.chest3.toFixed(0)} g  shoulder belt ${(m.belt / 9.81).toFixed(0)} kgf  lap ${(m.lap / 9.81).toFixed(0)} kgf  steering rearward ${(back * 1000).toFixed(0)} mm`);
console.log('1976 measured:  HIC 340, head 85 g, chest 23 g, belt 676 kgf, steering 41 mm   (', Date.now() - t0, 'ms)');
const LZ = id => car.loc[(id - car.base) * 3 + 2] - car.restL[(id - car.base) * 3 + 2];
const eng = car.L.nodes.filter(n => n.cls === CLS.ENG), rail = car.L.nodes.filter(n => (n.cls === CLS.RAIL) && Math.abs(n.p[2] - 1.05) < .01);
console.log('engine dz mm', (eng.reduce((s, n) => s + LZ(n.id), 0) / eng.length * 1000).toFixed(0), 'rail@1.05 dz', (rail.reduce((s, n) => s + LZ(n.id), 0) / rail.length * 1000).toFixed(0), 'col dz', (LZ(car.L.col) * 1000).toFixed(0), 'sw dz', (LZ(car.L.sw) * 1000).toFixed(0));
