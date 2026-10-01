// Static set pieces of the test centre: the barrier block, the camera pit, light towers, the hall.
import { Geo, blob, lathe, frame, grid } from './geo.js';
import { BARRIER } from './world.js';
export function buildSet() {
  const g = Geo(), B = BARRIER;
  // 90-tonne concrete block, steel-plated face with hazard stripes
  blob(g, [0, B.h / 2, B.d / 2 + .02], [B.w, B.h / 2, B.d / 2], 12, { mat: 20, col: [.42, .41, .38, .9], nu: 12, nv: 32 });
  blob(g, [0, B.h / 2, .01], [B.w - .05, B.h / 2 - .05, .012], 14, { mat: 21, col: [1, 1, 1, .5], nu: 10, nv: 32 });
  // camera pit walls + glass lid frame
  for (const [cx, cz, ex, ez] of [[0, -.55, 1.35, .05], [0, -4.65, 1.35, .05], [-1.35, -2.6, .05, 2.05], [1.35, -2.6, .05, 2.05]]) blob(g, [cx, -.6, cz], [ex, .62, ez], 12, { mat: 20, col: [.3, .3, .3, .9], nu: 6, nv: 12 });
  blob(g, [0, -1.2, -2.6], [1.35, .02, 2.05], 12, { mat: 20, col: [.15, .15, .15, .9], nu: 6, nv: 12 });
  // light towers + high-speed camera stands
  for (const [x, z] of [[-9, -6], [9, -6], [-9, 4], [9, 4]]) {
    lathe(g, frame([x, 0, z], [0, 1, 0], [1, 0, 0]), [[0, .12], [7, .08], [7, 0]], 8, { mat: 22, col: [.5, .5, .52, .4] });
    blob(g, [x, 7.2, z], [.6, .35, .25], 8, { mat: 22, col: [.2, .2, .2, .5] });
    blob(g, [x * .97, 7.2, z + (z < 0 ? .27 : -.27)], [.5, .27, .02], 8, { mat: 24, col: [1, .95, .85, .1] });
  }
  for (const [x, z, y] of [[-10.5, -1.4, 1.2], [9, -2.5, 1.2], [-2, -16, 1.4]]) {
    for (let k = 0; k < 3; k++) { const a = k * 2.1, ox = Math.cos(a) * .32, oz = Math.sin(a) * .32; lathe(g, frame([x + ox, 0, z + oz], [-ox / y, 1, -oz / y], [1, 0, 0]), [[0, .02], [y, .015], [y, 0]], 6, { mat: 22, col: [.1, .1, .1, .5] }); }
    blob(g, [x, y + .1, z], [.12, .1, .18], 6, { mat: 5, col: [.05, .05, .05, .4] });
  }
  // the hall behind the barrier
  blob(g, [-30, 4, 25], [12, 4, 18], 14, { mat: 25, col: [.55, .56, .58, .6], nu: 8, nv: 24 });
  blob(g, [-30, 8.1, 25], [12.4, .25, 18.4], 14, { mat: 22, col: [.25, .26, .28, .5], nu: 6, nv: 24 });
  for (let i = 0; i < g.M.length / 4; i++) g.M[i * 4 + 2] = Math.min(1, .55 + g.P[i * 3 + 1] * .25);
  return g;
}
export function buildLid() { const g = Geo(); blob(g, [0, -.01, -2.6], [1.3, .01, 2.0], 12, { mat: 26, col: [1, 1, 1, .05], nu: 4, nv: 8 }); return g; }
import { BALES } from './world.js';
import { RAMP } from './sim.js';
// per-test props: offset honeycomb barrier, pole, plane tree, rollover ramp; hay bales in the field
export function buildProps() {
  const odb = Geo(), pole = Geo(), tree = Geo(), ramp = Geo(), bales = Geo();
  blob(odb, [0, .325, .27], [.5, .325, .27], 14, { mat: 23, col: [.75, .76, .78, .4], nu: 8, nv: 16 });
  blob(odb, [0, .325, .55], [.52, .4, .02], 14, { mat: 22, col: [.35, .36, .38, .4], nu: 6, nv: 12 });
  lathe(pole, frame([0, 0, 0], [0, 1, 0], [1, 0, 0]), [[0, 0], [0, .127], [3.2, .127], [3.2, 0]], 24, { mat: 22, col: [.55, .56, .58, .3] });
  blob(pole, [0, -.02, 0], [.6, .04, .6], 8, { mat: 20, col: [.35, .35, .33, .8] });
  lathe(tree, frame([0, 0, 0], [0, 1, 0], [1, 0, 0]), [[0, .62], [.4, .42], [2.5, .36], [4.5, .3], [5, 0]], 24, { mat: 27, col: [.5, .48, .4, .9] });
  const R = RAMP; grid(ramp, 1, 1, (u, v) => [R.x0 + (R.x1 - R.x0) * v, R.h * u, R.z0 + (R.z1 - R.z0) * u], { mat: 22, col: [.4, .38, .3, .6] });
  for (const s of [R.x1, R.x0]) grid(ramp, 1, 1, (u, v) => [s, R.h * u * v, R.z0 + (R.z1 - R.z0) * u], { mat: 22, col: [.3, .3, .3, .6], flip: s === R.x0 });
  grid(ramp, 1, 1, (u, v) => [R.x0 + (R.x1 - R.x0) * u, R.h * v, R.z1], { mat: 22, col: [.3, .3, .3, .6] });
  for (const [x, z] of BALES) lathe(bales, frame([x, .75, z], [1, 0, 0], [0, 1, 0]), [[-.6, 0], [-.62, .6], [-.6, .74], [.6, .74], [.62, .6], [.6, 0]], 20, { mat: 28, col: [.62, .5, .26, .95] });
  for (const g of [odb, pole, tree, ramp, bales]) for (let i = 0; i < g.M.length / 4; i++) g.M[i * 4 + 2] = .8;
  return { odb, pole, tree, ramp, bales };
}
