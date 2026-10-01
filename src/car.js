// The Citroën 2CV6, built from parametric surfaces. Car space: x = left (driver side, LHD), y up, z forward,
// origin on the ground midway between the axles. Dimensions from the 2CV6 spec: 3830 x 1480 x 1600 mm,
// wheelbase 2400, track 1260, 125 R 15 tyres (r 0.2925 m).
import { Geo, grid, lathe, tube, blob, frame, table, polySdf } from './geo.js';
import { add, sub, scl, norm, cross, len, lerp, clamp } from './math.js';

export const WB = 2.4, FZ = 1.2, RZ = -1.2, TR = .63, WR = .2925, HUBY = .2925;
export const WHEELS = [[TR, FZ], [-TR, FZ], [TR, RZ], [-TR, RZ]].map(([x, z]) => [x, HUBY, z]); // FL FR RL RR
export const PIVOT = WHEELS.map(([x, y, z]) => [x * .78, .31, z > 0 ? .8 : -.8]);
export const SEAT = [[.33, .47, -.12], [-.33, .47, -.12]]; // H-points (driver left = +x)
export const SWHEEL = { c: [.33, 1.0, .24], n: norm([0, .5, -.866]), r: .2 };
// materials
export const PAINT = 0, CHROME = 1, TYRE = 2, CANVAS = 4, BLACK = 5, FABRIC = 6, ALU = 7, CHASSIS = 8, LAMP = 9,
  WPAINT = 10, DUMMY = 11, STEEL = 12, DASH = 13;
// part categories (x-ray / exploded view)
export const P_BODY = 0, P_CHASSIS = 1, P_ENGINE = 2, P_SUSP = 3, P_WHEEL = 4, P_INT = 5;

// ---- body shell: one loft of cross-sections. z -> [yTop, yBot, hwBot, hwTop, squareness]
const SEC = table([
  [1.81, .74, .40, .37, .33, 3.4], [1.70, .80, .40, .41, .36, 3.6], [1.40, .885, .42, .45, .40, 4.4], [1.00, .965, .45, .50, .46, 5.5],
  [.72, 1.02, .42, .58, .55, 6], [.62, 1.04, .32, .66, .60, 5.5], [.55, 1.16, .30, .68, .61, 5], [.45, 1.33, .30, .69, .615, 5],
  [.36, 1.455, .30, .69, .62, 5], [.20, 1.535, .30, .69, .625, 5], [0, 1.578, .30, .69, .625, 5], [-.30, 1.588, .30, .69, .625, 5],
  [-.65, 1.565, .30, .69, .62, 5], [-.95, 1.505, .30, .685, .61, 5], [-1.25, 1.39, .30, .68, .59, 4.5], [-1.50, 1.20, .31, .67, .57, 4],
  [-1.70, .98, .32, .655, .55, 3.6], [-1.84, .72, .33, .62, .52, 3.3], [-1.92, .52, .34, .58, .48, 3]]);
export const yTop = z => SEC(z)[0];
// the ends of the shell: the nose rounds over the last 24 cm (profile taken from a 3D model of a 1964 2CV, scaled), closing towards its bottom edge (so the grille face
// leans back and the top curves down into it, as on the real car); the tail rounds over 3.5 cm
export function capXform(z, x, y) {
  if (z > ZF - .24) { const c = Math.min(1, Math.max(0, (ZF - z) / .24)), k = Math.sqrt(Math.max(0, 1 - (1 - c) ** 2)), cy = SEC(z)[1] + .02; return [x * (.25 + .75 * k), cy + (y - cy) * k]; }
  if (z < -1.89) { const c = Math.min(1, Math.max(0, (z - ZR) / .035)), k = Math.sqrt(Math.max(0, 1 - (1 - c) ** 2)); return [x * k, .44 + (y - .44) * k]; }
  return [x, y];
}
export const ZF = 1.815, ZR = -1.925;
// wings (bolt-on). z -> [top y, inner x, outer x]
export const FW = table([[1.875, .47, .47, .62], [1.82, .66, .43, .69], [1.70, .79, .44, .735], [1.45, .825, .46, .74], [1.20, .83, .465, .74], [1.0, .79, .48, .74], [.82, .67, .54, .735], [.66, .5, .6, .725], [.55, .37, .62, .72]]);
export const RW = table([[-.9, .37, .6, .7], [-.99, .6, .6, .73], [-1.12, .735, .585, .74], [-1.45, .755, .575, .74], [-1.68, .71, .56, .73], [-1.8, .56, .54, .7], [-1.86, .42, .53, .66]]);
export const archF = z => .29 + Math.sqrt(Math.max(0, .365 ** 2 - (z - FZ) ** 2));
// ribbed bonnet: five raised swages, fading out at both ends
const RIBS = [-.24, -.12, 0, .12, .24];
const rib = (x, z) => z < .72 || z > 1.74 ? 0 : RIBS.reduce((a, r) => a + Math.exp(-(((x - r) / .019) ** 2)), 0) * .011 * clamp((z - .72) / .1, 0, 1) * clamp((1.74 - z) / .12, 0, 1);
export function section(z, n = 240) {
  const [yT, yB, wB, wT, q] = SEC(z), H = yT - yB, raw = [];
  for (let i = 0; i <= 600; i++) {
    const t = Math.PI * i / 600, c = Math.cos(t), s = Math.sin(t);
    const sx = Math.sign(c) * Math.pow(Math.abs(c), 2 / q), sy = Math.pow(s, 2 / q), w = lerp(wB, wT, Math.pow(sy, 1.6));
    let x = w * sx, y = yB + H * sy;
    if (Math.abs(sx) < .9) y += rib(x, z);
    raw.push([x, y]);
  }
  // arc-length resample so the flat bonnet top gets as many vertices as the sides
  const L = [0]; for (let i = 1; i < raw.length; i++) L.push(L[i - 1] + Math.hypot(raw[i][0] - raw[i - 1][0], raw[i][1] - raw[i - 1][1]));
  const T = L[L.length - 1], out = []; let k = 0;
  for (let j = 0; j <= n; j++) {
    const d = T * j / n; while (k < L.length - 2 && L[k + 1] < d) k++;
    const f = (d - L[k]) / (L[k + 1] - L[k] || 1); out.push([lerp(raw[k][0], raw[k + 1][0], f), lerp(raw[k][1], raw[k + 1][1], f), d - T / 2]);
  }
  return out;
}
// glasshouse, in side view (z, y) and in "around" coordinates (z, s = arc distance from the roof centre line)
export const SIDE_WIN = [
  [[.475, 1.06], [-.305, 1.06], [-.305, 1.43], [.29, 1.43]],   // front door (flap-up)
  [[-.37, 1.06], [-.93, 1.06], [-.93, 1.395], [-.37, 1.43]],  // rear door
  [[-1.01, 1.06], [-1.31, 1.06], [-1.19, 1.30], [-1.01, 1.375]]]; // rear quarter (six-light)
const round = (d, r) => d - r;
const rr = (p, c, h, r) => { const q = [Math.abs(p[0] - c[0]) - h[0] + r, Math.abs(p[1] - c[1]) - h[1] + r]; return Math.hypot(Math.max(q[0], 0), Math.max(q[1], 0)) + Math.min(Math.max(q[0], q[1]), 0) - r; };
export function winSdf(x, y, z, s) {
  let d = 9;
  if (Math.abs(s) > .6) for (const w of SIDE_WIN) d = Math.min(d, round(polySdf([z, y], shrink(w, .035)), .035));
  d = Math.min(d, rr([z, s], [.497, 0], [.104, .56], .06));        // windscreen
  d = Math.min(d, rr([z, s], [-1.39, 0], [.085, .36], .05));       // rear window in the canvas
  return d;
}
function shrink(pts, r) { const c = pts.reduce((a, p) => [a[0] + p[0] / pts.length, a[1] + p[1] / pts.length], [0, 0]); return pts.map(p => { const d = [p[0] - c[0], p[1] - c[1]], l = Math.hypot(...d); return [p[0] - d[0] / l * r * 1.2, p[1] - d[1] / l * r * 1.2]; }); }
const DOORS = [[[.585, .33], [-.335, .33], [-.335, 1.475], [.36, 1.475], [.585, 1.045]], [[-.355, .33], [-.985, .33], [-.985, 1.475], [-.355, 1.475]]];
function seamSdf(x, y, z, s) {
  let d = 9;
  if (Math.abs(s) > .55) for (const w of DOORS) d = Math.min(d, Math.abs(polySdf([z, y], w)));
  if (z > .6) d = Math.min(d, Math.abs(z - .705) + (Math.abs(s) > .8 ? 9 : 0)); // bonnet rear edge
  d = Math.min(d, Math.abs(rr([z, s], [.657, 0], [.028, .46], .02)));          // scuttle vent flap
  d = Math.min(d, Math.abs(rr([z, s], [-1.71, 0], [.185, .5], .08)));          // boot lid
  return d;
}
export const canvasSdf = (x, y, z, s) => rr([z, s], [-.58, 0], [.95, .56], .07);

export function buildCar() {
  const g = Geo(), C = (r, g_, b, ro = .5) => [r, g_, b, ro];
  const paint = { mat: PAINT, col: C(1, 1, 1, .3), part: P_BODY };
  // ---------------- body shell
  const NV = 240, secs = [], zs = [];
  for (let z = ZF; z > ZF - .3; z -= .007) zs.push(z);           // fine rows where the nose rounds over
  for (let z = ZF - .3; z > ZR + .06; z -= .025) zs.push(z);
  for (let z = ZR + .06; z > ZR; z -= .008) zs.push(z); zs.push(ZR);
  const NU = zs.length - 1;
  for (const z of zs) secs.push(section(z, NV));
  grid(g, NU, NV, (u, v) => {
    const i = Math.round(u * NU), j = Math.round(v * NV), z = zs[i], p = secs[i][j];
    const [x, y] = capXform(z, p[0], p[1]);
    return [x, y, z];
  }, { ...paint, k: (p, u, v) => { const s = secs[Math.round(u * NU)][Math.round(v * NV)][2]; return [winSdf(p[0], p[1], p[2], s), seamSdf(p[0], p[1], p[2], s), canvasSdf(p[0], p[1], p[2], s), s]; } });
  // ---------------- wings (bolt-on). z -> [top y, inner x, outer x]
  for (const s of [1, -1]) {
    const wing = (W, z0, z1, bot) => grid(g, 70, 36, (u, v) => {
      const z = lerp(z0, z1, u), [yT, xi, xo] = W(z), ph = v * Math.PI, inner = v < .5;
      const yb = inner ? Math.min(yT, .45) : Math.min(yT, bot(z)), h = yT - yb;
      const xc = (xi + xo) / 2, hw = (xo - xi) / 2, sp = Math.pow(Math.sin(ph), .45);
      return [s * (xc - hw * Math.sign(Math.cos(ph)) * Math.pow(Math.abs(Math.cos(ph)), .7)), yb + h * sp, z];
    }, { ...paint, flip: s > 0 ? 1 : 0, k: () => [9, 9, 9, s * 1.5] });
    wing(FW, 1.875, .55, z => Math.max(.4, archF(z)));
    wing(RW, -.9, -1.86, () => .335);
    // wing beading (the rubber piping between wing and body)
    tube(g, Array.from({ length: 30 }, (_, i) => { const z = lerp(1.8, .6, i / 29), [yT, xi] = FW(z); return [s * (xi + .005), Math.min(yT, .45) + .005, z]; }), .006, 5, { mat: BLACK, col: C(.03, .03, .03, .5), part: P_BODY });
  }
  // ---------------- lamps, bumpers, trim
  for (const s of [1, -1]) {
    const F = frame([s * .5, .86, 1.6], [0, 0, 1], [1, 0, 0]);
    lathe(g, F, [[-.11, 0], [-.105, .04], [-.085, .07], [-.05, .088], [0, .096], [.012, .097], [.016, .09]], 28, { mat: CHROME, col: C(.9, .9, .92, .12), part: P_BODY });
    lathe(g, frame([s * .5, .86, 1.616], [0, 0, 1], [1, 0, 0]), [[0, .089], [.008, .07], [.013, .04], [.015, 0]], 28, { mat: LAMP, col: C(1, .97, .9, .05), part: P_BODY });
    tube(g, [[s * .5, .795, 1.57], [s * .48, .77, 1.58], [s * .36, .77, 1.6]], .014, 8, { mat: CHASSIS, col: C(.2, .2, .2, .4), part: P_BODY }); // lamp stalk
    lathe(g, frame([s * .615, .585, 1.83], [0, .1, 1], [1, 0, 0]), [[-.02, 0], [-.02, .034], [.004, .036], [.012, .028], [.016, 0]], 16, { mat: LAMP, col: C(1, .42, .04, .1), part: P_BODY }); // front indicator on the wing
    blob(g, [s * .62, .645, -1.805], [.04, .05, .022], 3, { mat: LAMP, col: C(.85, .05, .03, .1), part: P_BODY, nu: 8, nv: 12 }); // tail lamp
    blob(g, [s * .62, .58, -1.795], [.03, .016, .016], 3, { mat: LAMP, col: C(1, .45, .05, .1), part: P_BODY, nu: 6, nv: 10 });
    // bumper overriders
    blob(g, [s * .3, .4, 1.905], [.022, .075, .022], 4, { mat: CHROME, col: C(.85, .85, .87, .15), part: P_BODY, nu: 8, nv: 12 });
    blob(g, [s * .3, .4, -1.975], [.022, .075, .022], 4, { mat: CHROME, col: C(.85, .85, .87, .15), part: P_BODY, nu: 8, nv: 12 });
    // door handles and hinges
    for (const z of [-.27, -.93]) blob(g, [s * .695, 1.0, z], [.012, .012, .045], 3, { mat: CHROME, col: C(.8, .8, .82, .15), part: P_BODY, nu: 6, nv: 10 });
    for (const [z, y] of [[.585, .62], [.585, .92], [-.355, .62], [-.355, .92]]) blob(g, [s * .693, y, z], [.01, .03, .015], 4, { mat: CHROME, col: C(.6, .6, .62, .3), part: P_BODY, nu: 6, nv: 8 });
  }
  tube(g, [[-.53, .785, 1.57], [.53, .785, 1.57]], .016, 8, { mat: CHASSIS, col: C(.25, .25, .25, .4), part: P_BODY }); // headlamp bar
  // the bonnet's front comes down to the bumper: valance below the grille and a pan underneath (no view into the engine bay)
  blob(g, [0, .385, 1.6], [.41, .014, .2], 8, { mat: CHASSIS, col: C(.06, .06, .06, .7), part: P_BODY, nu: 6, nv: 20 });
  for (const z of [1.885, -1.955]) {
    blob(g, [0, .375, z], [.7, .036, .024], 6, { mat: CHROME, col: C(.85, .85, .87, .15), part: P_BODY, nu: 10, nv: 28 });
    blob(g, [0, .375, z + Math.sign(z) * .02], [.69, .013, .01], 4, { mat: BLACK, col: C(.03, .03, .03, .6), part: P_BODY, nu: 6, nv: 24 });
  }
  // mirror (driver side), wipers
  tube(g, [[.69, 1.04, .5], [.75, 1.12, .5]], .007, 6, { mat: CHROME, col: C(.8, .8, .8, .2), part: P_BODY });
  lathe(g, frame([.76, 1.13, .5], [0, 0, -1], [1, 0, 0]), [[-.012, 0], [-.012, .045], [0, .048], [.004, .045], [.005, 0]], 16, { mat: CHROME, col: C(.85, .85, .87, .05), part: P_BODY });
  for (const x of [.45, -.05]) tube(g, [[x, 1.07, .615], [x - .2, 1.075, .612], [x - .38, 1.08, .608]], .005, 4, { mat: BLACK, col: C(.02, .02, .02, .5), part: P_BODY });
  // ---------------- platform chassis + underbody
  const ch = { mat: CHASSIS, col: C(.08, .085, .08, .7), part: P_CHASSIS };
  blob(g, [0, .255, -.45], [.64, .028, 1.2], 8, { ...ch, nu: 8, nv: 40 });               // floor pan
  for (const s of [1, -1]) blob(g, [s * .46, .24, -.1], [.045, .05, 1.72], 6, { ...ch, nu: 8, nv: 30 }); // side members
  for (const z of [.8, -.8]) tube(g, [[-.56, .305, z], [.56, .305, z]], .045, 12, { ...ch });              // suspension cross-tubes
  blob(g, [0, .63, .64], [.62, .33, .015], 8, { ...ch, col: C(.1, .1, .1, .7), nu: 8, nv: 24 });          // bulkhead
  blob(g, [0, .305, -.4], [.6, .012, .9], 8, { mat: BLACK, col: C(.025, .025, .025, .9), part: P_INT, nu: 6, nv: 30 }); // rubber floor mat
  // suspension: canister per side (front and rear interconnected), tie rods, arms
  for (const s of [1, -1]) {
    lathe(g, frame([s * .56, .25, 0], [0, 0, 1], [1, 0, 0]), [[-.5, 0], [-.5, .05], [-.48, .057], [.48, .057], [.5, .05], [.5, 0]], 16, { mat: CHASSIS, col: C(.12, .12, .13, .45), part: P_SUSP });
    for (const k of [1, -1]) tube(g, [[s * .56, .25, k * .5], [s * .56, .27, k * .76]], .009, 6, { mat: STEEL, col: C(.35, .35, .36, .4), part: P_SUSP });
  }
  WHEELS.forEach((h, i) => {
    const pv = PIVOT[i], s = Math.sign(h[0]), bone = 8 + i, inner = [s * .56, h[1], h[2]];
    tube(g, [pv, add(pv, scl(sub(inner, pv), .5)), inner], u => .04 - u * .012, 10, { mat: CHASSIS, col: C(.1, .1, .1, .5), part: P_SUSP, bone });
    lathe(g, frame(pv, [1, 0, 0], [0, 1, 0]), [[-.07, 0], [-.07, .04], [.07, .04], [.07, 0]], 10, { mat: CHASSIS, col: C(.12, .12, .12, .5), part: P_SUSP, bone });
    // the 2CV's inertia dampers ("batteurs") sat on the arms near the wheels on early cars; here telescopic dampers
    tube(g, [add(inner, [-s * .03, .02, -Math.sign(h[2]) * .05]), add(pv, [s * .03, .17, 0])], .018, 8, { mat: STEEL, col: C(.25, .25, .26, .4), part: P_SUSP, bone });
  });
  // ---------------- engine bay: 602 cc flat twin, cooling fan, gearbox, inboard brakes, spare wheel
  const alu = { mat: ALU, col: C(.62, .62, .6, .45), part: P_ENGINE };
  blob(g, [0, .5, 1.5], [.11, .13, .14], 3.5, alu);                                      // crankcase
  for (const s of [1, -1]) {
    const fins = []; for (let i = 0; i <= 14; i++) { const x = .1 + i * .012; fins.push([x, .068], [x + .002, .094], [x + .008, .094], [x + .01, .068]); }
    lathe(g, frame([0, .52, 1.52], [s, 0, 0], [0, 1, 0]), [[.1, 0], ...fins, [.28, .068], [.28, 0]], 18, alu);       // finned barrel
    blob(g, [s * .3, .52, 1.52], [.04, .095, .095], 4, alu);                                 // head
    for (let i = 0; i < 4; i++) blob(g, [s * (.27 + i * .016), .52, 1.52], [.004, .11, .1], 5, { ...alu, nu: 6, nv: 16 }); // head fins
    blob(g, [s * .345, .53, 1.52], [.016, .075, .07], 5, { mat: BLACK, col: C(.06, .06, .06, .3), part: P_ENGINE }); // rocker cover
    tube(g, [[s * .3, .44, 1.56], [s * .22, .3, 1.62], [s * .08, .2, 1.55], [0, .19, 1.4]], .02, 8, { mat: STEEL, col: C(.3, .2, .14, .6), part: P_ENGINE }); // exhaust downpipe
    tube(g, [[0, .64, 1.68], [s * .1, .66, 1.62], [s * .22, .64, 1.55], [s * .3, .62, 1.52]], .045, 12, { mat: BLACK, col: C(.04, .04, .04, .6), part: P_ENGINE }); // air duct
    lathe(g, frame([s * .16, .45, 1.13], [s, 0, 0], [0, 1, 0]), [[-.012, 0], [-.012, .1], [.012, .1], [.012, 0]], 24, { mat: STEEL, col: C(.4, .38, .36, .35), part: P_ENGINE }); // inboard disc
    blob(g, [s * .2, .5, 1.13], [.03, .04, .05], 3, { mat: BLACK, col: C(.05, .05, .05, .5), part: P_ENGINE }); // caliper
    tube(g, [[s * .18, .45, 1.13], [s * .4, .38, 1.17], [s * .58, .2925, 1.2]], .018, 8, { mat: STEEL, col: C(.2, .2, .2, .5), part: P_SUSP, bone: 12 + (s > 0 ? 0 : 1) }); // driveshaft
  }
  lathe(g, frame([0, .52, 1.63], [0, 0, 1], [1, 0, 0]), [[-.04, .12], [-.04, .16], [.02, .175], [.07, .17], [.08, .14], [.08, .11]], 28, { mat: BLACK, col: C(.05, .05, .05, .5), part: P_ENGINE }); // fan cowling
  for (let i = 0; i < 8; i++) { const a = i / 8 * Math.PI * 2; blob(g, [Math.cos(a) * .08, .52 + Math.sin(a) * .08, 1.66], [.06, .012, .005], 3, { mat: ALU, col: C(.5, .5, .5, .4), part: P_ENGINE, bone: 6, nu: 6, nv: 8 }, p => [p[0] * Math.cos(a) - p[1] * Math.sin(a), p[0] * Math.sin(a) + p[1] * Math.cos(a), p[2] + p[0] * .3]); }
  lathe(g, frame([0, .52, 1.67], [0, 0, 1], [1, 0, 0]), [[-.02, 0], [-.02, .03], [.02, .03], [.02, 0]], 12, { mat: ALU, col: C(.5, .5, .5, .4), part: P_ENGINE, bone: 6 });
  lathe(g, frame([0, .73, 1.42], [0, 1, 0], [1, 0, 0]), [[-.04, 0], [-.04, .11], [-.03, .12], [.03, .12], [.04, .11], [.04, 0]], 28, { mat: BLACK, col: C(.07, .07, .07, .4), part: P_ENGINE }); // air filter
  blob(g, [0, .66, 1.44], [.04, .05, .04], 3, alu);                                     // carburettor
  blob(g, [.1, .66, 1.6], [.07, .05, .02], 8, { ...alu, col: C(.3, .3, .3, .5) });     // oil cooler
  blob(g, [0, .47, 1.12], [.13, .12, .17], 3, alu);                                     // gearbox
  blob(g, [-.36, .76, .74], [.09, .085, .065], 6, { mat: BLACK, col: C(.03, .03, .03, .4), part: P_ENGINE });  // battery
  // spare wheel in its cradle over the gearbox
  wheelGeo(g, frame([0, .7, 1.05], norm([0, 1, .35]), [1, 0, 0]), { part: P_ENGINE, bone: 0 });
  // exhaust run and rear silencer, fuel tank
  tube(g, [[0, .19, 1.4], [-.06, .18, 1.0], [-.25, .17, .55], [-.25, .17, -1.3]], .021, 8, { mat: STEEL, col: C(.25, .17, .12, .65), part: P_CHASSIS });
  lathe(g, frame([-.25, .19, -1.55], [0, 0, 1], [1, 0, 0]), [[-.22, 0], [-.22, .065], [-.2, .075], [.2, .075], [.22, .065], [.22, 0]], 16, { mat: STEEL, col: C(.3, .22, .15, .6), part: P_CHASSIS });
  tube(g, [[-.25, .19, -1.77], [-.25, .19, -1.99]], .018, 8, { mat: STEEL, col: C(.2, .2, .2, .5), part: P_CHASSIS });
  blob(g, [.18, .32, -1.5], [.3, .07, .2], 5, { mat: CHASSIS, col: C(.07, .07, .07, .7), part: P_CHASSIS }); // fuel tank
  // ---------------- interior
  const fab = { mat: FABRIC, col: C(.32, .3, .27, .9), part: P_INT }, frm = { mat: CHASSIS, col: C(.15, .15, .15, .4), part: P_INT };
  for (const [x, y, z] of SEAT) { // tubular seat frames with hammock-style covers
    const w = .24, b = z - .3;
    tube(g, [[x - w, .32, z + .26], [x - w, .45, z + .22], [x - w, .43, b], [x - w, .95, b - .14], [x + w, .95, b - .14], [x + w, .43, b], [x + w, .45, z + .22], [x + w, .32, z + .26]], .012, 6, frm);
    blob(g, [x, .45, z - .04], [w - .01, .04, .25], 3, fab, p => [p[0], p[1] - .03 * Math.cos(p[2] * 6), p[2]]);
    blob(g, [x, .7, b - .07], [w - .01, .24, .035], 3, fab, p => [p[0], p[1], p[2] - p[1] * .28]);
  }
  tube(g, [[-.55, .32, -.65], [-.55, .43, -.7], [.55, .43, -.7], [.55, .32, -.65]], .012, 6, frm);      // rear bench
  blob(g, [0, .44, -.85], [.55, .045, .23], 3, fab);
  blob(g, [0, .7, -1.12], [.55, .25, .04], 3, fab, p => [p[0], p[1], p[2] - p[1] * .35]);
  // dashboard: parcel shelf, instrument pod, umbrella-handle gear lever, pedals
  blob(g, [0, .97, .55], [.62, .02, .1], 6, { mat: DASH, col: C(1, 1, 1, .5), part: P_INT, nu: 8, nv: 24 });
  blob(g, [.31, 1.03, .48], [.11, .055, .05], 6, { mat: BLACK, col: C(.03, .03, .03, .5), part: P_INT });
  lathe(g, frame([.31, 1.035, .43], [0, .2, -1], [1, 0, 0]), [[0, 0], [0, .042], [.006, .046], [.008, 0]], 20, { mat: LAMP, col: C(.9, .9, .85, .1), part: P_INT });  // speedometer face
  tube(g, [[.08, .93, .5], [.08, .92, .33], [.08, .95, .3], [.08, .99, .31]], .008, 6, { mat: CHROME, col: C(.6, .6, .6, .3), part: P_INT, bone: 7 });
  blob(g, [.08, 1.0, .315], [.022, .022, .022], 2, { mat: BLACK, col: C(.02, .02, .02, .3), part: P_INT, bone: 7 });
  for (const x of [.42, .33, .24]) tube(g, [[x, .62, .62], [x, .42, .52], [x, .4, .44]], .008, 6, frm), blob(g, [x, .4, .44], [.035, .045, .01], 6, { mat: BLACK, col: C(.03, .03, .03, .7), part: P_INT, nu: 6, nv: 10 }, p => [p[0], p[1], p[2] + p[1] * .5]);
  // steering wheel (single spoke) and column
  const sw = SWHEEL, ca = norm(sw.n), cb = norm(cross(ca, [1, 0, 0])), cc = cross(ca, cb);
  tube(g, Array.from({ length: 41 }, (_, i) => { const a = i / 40 * Math.PI * 2; return add(sw.c, add(scl(cb, Math.cos(a) * sw.r), scl(cc, Math.sin(a) * sw.r))); }), .013, 8, { mat: BLACK, col: C(.03, .03, .03, .35), part: P_INT, bone: 5 });
  tube(g, [add(sw.c, scl(ca, .03)), add(sw.c, add(scl(cb, -sw.r * .7), scl(cc, -sw.r * .7)))], .012, 6, { mat: BLACK, col: C(.03, .03, .03, .35), part: P_INT, bone: 5 });
  blob(g, add(sw.c, scl(ca, .04)), [.035, .035, .035], 2, { mat: BLACK, col: C(.03, .03, .03, .3), part: P_INT, bone: 5 });
  tube(g, [add(sw.c, scl(ca, -.02)), add(sw.c, scl(ca, -.62))], .018, 8, { mat: BLACK, col: C(.04, .04, .04, .5), part: P_INT });
  // ---------------- wheels (bones 1-4)
  WHEELS.forEach((h, i) => wheelGeo(g, frame(h, [Math.sign(h[0]), 0, 0], [0, 1, 0]), { part: P_WHEEL, bone: 1 + i }));
  bakeAO(g);
  return g;
}
// 125 R 15 tyre on a 3-stud steel wheel with a chrome dome hubcap, drum behind
export function wheelGeo(g, F, o) {
  const C = (r, g_, b, ro) => [r, g_, b, ro];
  lathe(g, F, [[-.046, .19], [-.058, .205], [-.0625, .235], [-.058, .268], [-.05, .284], [-.04, .2925], [.04, .2925], [.05, .284], [.058, .268], [.0625, .235], [.058, .205], [.046, .19]], 64, { ...o, mat: TYRE, col: C(.035, .035, .037, .85) });
  lathe(g, F, [[.05, .19], [.055, .197], [.043, .185], [.03, .17], [.02, .14], [.03, .09], [.035, .075], [.04, .07], [.06, .055], [.075, .03], [.078, 0]], 32, { ...o, mat: WPAINT, col: C(.3, .3, .29, .35) });
  lathe(g, F, [[.035, .075], [.04, .07], [.06, .055], [.075, .03], [.078, 0]], 32, { ...o, mat: CHROME, col: C(.9, .9, .92, .08) });
  lathe(g, F, [[-.05, .19], [-.045, .17], [.02, .17]], 32, { ...o, mat: WPAINT, col: C(.5, .5, .48, .4), flip: 1 });
  lathe(g, F, [[-.1, 0], [-.1, .12], [-.03, .13], [-.03, 0]], 16, { ...o, mat: STEEL, col: C(.18, .17, .16, .6) });
}
// ambient occlusion: cast a few rays per vertex through a coarse occupancy grid of the whole car
function bakeAO(g) {
  const h = .04, ox = -.8, oy = -.02, oz = -2.05, nx = 40, ny = 42, nz = 103, occ = new Uint8Array(nx * ny * nz);
  const n = g.P.length / 3;
  for (let i = 0; i < n; i++) {
    if (g.K[i * 4] < 0) continue; // glass lets light in
    const x = Math.floor((g.P[i * 3] - ox) / h), y = Math.floor((g.P[i * 3 + 1] - oy) / h), z = Math.floor((g.P[i * 3 + 2] - oz) / h);
    if (x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz) occ[x + nx * (y + ny * z)] = 1;
  }
  const dirs = []; for (let i = 0; i < 32; i++) { const a = i * 2.39996, zz = 1 - (i + .5) / 32 * 2; const r = Math.sqrt(1 - zz * zz); dirs.push([Math.cos(a) * r, zz, Math.sin(a) * r]); }
  for (let i = 0; i < n; i++) {
    const P = [g.P[i * 3], g.P[i * 3 + 1], g.P[i * 3 + 2]], N = [g.N[i * 3], g.N[i * 3 + 1], g.N[i * 3 + 2]];
    let vis = 0, tot = 0;
    for (const d0 of dirs) {
      const dn = d0[0] * N[0] + d0[1] * N[1] + d0[2] * N[2]; const d = dn < 0 ? [-d0[0], -d0[1], -d0[2]] : d0, w = Math.abs(dn);
      let hit = 0;
      for (let s = 1.6; s < 14 && !hit; s *= 1.35) {
        const q = [P[0] + d[0] * s * h, P[1] + d[1] * s * h, P[2] + d[2] * s * h];
        if (q[1] < 0) { hit = .55; break; }  // the ground under the car
        const x = Math.floor((q[0] - ox) / h), y = Math.floor((q[1] - oy) / h), z = Math.floor((q[2] - oz) / h);
        if (x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz && occ[x + nx * (y + ny * z)]) hit = 1 - s / 18;
      }
      vis += w * (1 - hit); tot += w;
    }
    g.M[i * 4 + 2] = Math.pow(vis / tot, 1.2);
  }
  // two smoothing passes over shared edges hide the coarse grid
  for (let it = 0; it < 3; it++) {
    const s = new Float32Array(n), c = new Float32Array(n);
    for (let t = 0; t < g.I.length; t += 3) for (let k = 0; k < 3; k++) { const a = g.I[t + k], b = g.I[t + (k + 1) % 3]; s[a] += g.M[b * 4 + 2]; c[a]++; s[b] += g.M[a * 4 + 2]; c[b]++; }
    for (let i = 0; i < n; i++) if (c[i]) g.M[i * 4 + 2] = (g.M[i * 4 + 2] + s[i] / c[i]) * .5;
  }
}
