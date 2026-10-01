// Renderer: 2 shadow cascades -> 4x MSAA HDR -> resolve -> mip chain -> final (DOF, bloom, grade, tonemap).
import { gl, initGL, program, mesh, tex, fbo, rbo } from './gl.js';
import * as S from './glsl.js';
import { perspective, ortho, lookAt, mul, invert, add, scl, sub, norm } from './math.js';
import { TREES } from './world.js';
import { Geo, blob, lathe, frame as gframe } from './geo.js';

export function createRenderer(canvas) {
  initGL(canvas);
  const V = vs => S.HEAD + S.COMMON + vs, F = (fs, d = '') => S.HEAD + '\n' + d + S.COMMON + S.LIGHT + fs;
  const P2 = (vs, fs) => ({ m: program(V(vs), F(fs)), s: program(V(vs), F(fs, '#define SH\n')) });
  const prog = {
    sky: program(V(S.SKY_VS), F(S.SKY_FS)), terr: P2(S.TERR_VS, S.TERR_FS), car: P2(S.CAR_VS, S.CAR_FS),
    glass: program(V(S.CAR_VS), F(S.CAR_FS, '#define GLASS\n')), stat: P2(S.STAT_VS, S.STAT_FS), tree: P2(S.TREE_VS, S.TREE_FS),
    part: program(V(S.PART_VS), F(S.PART_FS)), final: program(S.HEAD + S.FS_VS, S.HEAD + S.FINAL_FS),
  };
  const quad = mesh({ aP: [[-1, -1, 0, 3, -1, 0, -1, 3, 0].flat(), 3] });
  const N = 200, tg = [], ti = [];
  for (let j = 0; j <= N; j++) for (let i = 0; i <= N; i++) tg.push(i / N * 2 - 1, j / N * 2 - 1, 0);
  for (let j = 0; j < N; j++) for (let i = 0; i < N; i++) { const a = j * (N + 1) + i; ti.push(a, a + 1, a + N + 2, a, a + N + 2, a + N + 1); }
  const terrain = mesh({ aP: [tg, 3] }, ti);
  // trees: trunk + canopy of blobs
  const tgeo = Geo();
  lathe(tgeo, gframe([0, 0, 0], [0, 1, 0], [1, 0, 0]), [[0, .5], [.3, .38], [3.5, .3], [5.5, .25], [7, .16], [7.2, 0]], 10, { mat: 30, col: [.5, .5, .4, .9] });
  for (let i = 0; i < 9; i++) { const a = i * 2.4, r = i ? 1.9 + (i % 3) * .5 : 0, y = 8.5 + (i % 4) * 1.1 - (i ? 0 : -.6); blob(tgeo, [Math.cos(a) * r, y, Math.sin(a) * r], [2.6, 2.1, 2.6].map(v => v * (i ? .85 : 1.15)), 2, { mat: 31, col: [.1, .14, .04, .9], nu: 8, nv: 12 }); }
  for (let i = 0; i < tgeo.M.length / 4; i++) tgeo.M[i * 4 + 2] = tgeo.M[i * 4] > 30.5 ? .55 + .45 * Math.min(1, Math.max(0, (tgeo.P[i * 3 + 1] - 6.5) / 6)) : .8;
  const trees = mesh({ aP: [tgeo.P, 3], aN: [tgeo.N, 3], aC: [tgeo.C, 4], aM: [tgeo.M, 4] }, tgeo.I, [{ name: 'iA', size: 4 }]);
  trees.update('iA', new Float32Array(TREES.flat()));
  const parts = mesh({ aP: [new Float32Array(4 * 4096), 4, 1], aC: [new Float32Array(4 * 4096), 4, 1] });
  // ---- shadow maps
  const SMS = [4096, 2048];
  const shTex = SMS.map(s => { const t = tex(s, s, gl.DEPTH_COMPONENT32F, gl.DEPTH_COMPONENT, gl.FLOAT, gl.LINEAR); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_COMPARE_MODE, gl.COMPARE_REF_TO_TEXTURE); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_COMPARE_FUNC, gl.LEQUAL); return t; });
  const shFbo = shTex.map(t => fbo([], t));
  let W = 0, H = 0; const T = {};
  const samples = Math.min(4, gl.getParameter(gl.MAX_SAMPLES));
  function resize(w, h) {
    if (w === W && h === H) return; W = w; H = h;
    for (const k of ['c', 'd']) if (T[k]) gl.deleteTexture(T[k]);
    T.ms = fbo([rbo(w, h, gl.RGBA16F, samples)], rbo(w, h, gl.DEPTH_COMPONENT32F, samples));
    T.c = tex(w, h, gl.RGBA16F, gl.RGBA, gl.HALF_FLOAT, gl.LINEAR, true); T.cf = fbo([T.c], null);
    T.d = tex(w, h, gl.DEPTH_COMPONENT32F, gl.DEPTH_COMPONENT, gl.FLOAT, gl.NEAREST); T.df = fbo([], T.d);
  }
  const tbind = (u, t) => { gl.activeTexture(gl.TEXTURE0 + u); gl.bindTexture(gl.TEXTURE_2D, t); };
  // a car's GPU mesh (static) + per-car node textures (deformed local positions + damage, rotations, rest)
  function carMesh(g, skin) {
    return mesh({ aP: [g.P, 3], aN: [g.N, 3], aC: [g.C, 4], aM: [g.M, 4], aK: [g.K, 4], aJ: [skin.J, 4], aW: [skin.W, 4] }, g.I);
  }
  const NW = 512;
  function nodeTex(C) {
    const h = Math.ceil(C.n / NW), mk = () => tex(NW, h, gl.RGBA32F, gl.RGBA, gl.FLOAT, gl.NEAREST);
    const T = { np: mk(), nq: mk(), nr: mk(), h, bp: new Float32Array(NW * h * 4), bq: new Float32Array(NW * h * 4) };
    const br = new Float32Array(NW * h * 4); for (let i = 0; i < C.n; i++) br.set([C.restL[i * 3], C.restL[i * 3 + 1], C.restL[i * 3 + 2], 0], i * 4);
    gl.bindTexture(gl.TEXTURE_2D, T.nr); gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32F, NW, h, 0, gl.RGBA, gl.FLOAT, br);
    return T;
  }
  function uploadNodes(C) {
    const T = C.gpu || (C.gpu = nodeTex(C));
    if (!C.dirty) return; C.dirty = false;
    for (let i = 0; i < C.n; i++) { T.bp[i * 4] = C.loc[i * 3]; T.bp[i * 4 + 1] = C.loc[i * 3 + 1]; T.bp[i * 4 + 2] = C.loc[i * 3 + 2]; T.bp[i * 4 + 3] = C.dmg[i];
      T.bq[i * 4] = C.q[i * 4 + 1]; T.bq[i * 4 + 1] = C.q[i * 4 + 2]; T.bq[i * 4 + 2] = C.q[i * 4 + 3]; T.bq[i * 4 + 3] = C.q[i * 4]; }
    gl.bindTexture(gl.TEXTURE_2D, T.np); gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32F, NW, T.h, 0, gl.RGBA, gl.FLOAT, T.bp);
    gl.bindTexture(gl.TEXTURE_2D, T.nq); gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32F, NW, T.h, 0, gl.RGBA, gl.FLOAT, T.bq);
  }
  function statMesh(g) { return mesh({ aP: [g.P, 3], aN: [g.N, 3], aC: [g.C, 4], aM: [g.M, 4] }, g.I); }

  function frame(sc) {
    const cw = canvas.width, ch = canvas.height, rw = Math.round(cw * sc.res), rh = Math.round(ch * sc.res);
    resize(rw, rh);
    const cam = sc.cam, asp = rw / rh;
    const proj = perspective(cam.fov, asp, cam.near, cam.far), view = lookAt(cam.pos, cam.tgt, cam.up || [0, 1, 0]), VP = mul(proj, view);
    const U = sc.light, sun = U.uSun;
    const fc = sc.focus || cam.tgt;
    const shM = [{ c: fc, s: 6.5, d: 40 }, { c: add(cam.pos, scl(norm(sub(cam.tgt, cam.pos)), 60)), s: 90, d: 400 }].map((cs, i) => {
      const lv = lookAt(add(cs.c, scl(sun, cs.d * .5)), cs.c, [0, 1, 0]);
      const tx = cs.s * 2 / SMS[i]; lv[12] = Math.round(lv[12] / tx) * tx; lv[13] = Math.round(lv[13] / tx) * tx;
      return mul(ortho(-cs.s, cs.s, -cs.s, cs.s, 0, cs.d), lv);
    });
    const FU = Object.assign({}, U, { uSh0M: shM[0], uSh1M: shM[1], uSh0: 6, uSh1: 7, uShOn: 1, uT: sc.t });
    gl.enable(gl.DEPTH_TEST); gl.depthFunc(gl.LEQUAL); gl.disable(gl.BLEND); gl.disable(gl.CULL_FACE);
    tbind(6, null); tbind(7, null);
    for (const c of sc.cars) if (c.car) uploadNodes(c.car);
    const drawCars = p => { for (const c of sc.cars) { if (c.hidden) continue; const T = c.car.gpu; tbind(8, T.np); tbind(9, T.nq); tbind(10, T.nr); p.setAll(c.u).setAll({ uNP: 8, uNQ: 9, uNR: 10 }); c.mesh.draw(); } };
    const drawStat = (p, glass) => { for (const s of sc.statics) { if (s.hidden || !!s.glass !== !!glass) continue; p.set('uM', s.M); s.mesh.draw(); } };
    gl.colorMask(false, false, false, false);
    for (let i = 0; i < 2; i++) {
      gl.bindFramebuffer(gl.FRAMEBUFFER, shFbo[i]); gl.viewport(0, 0, SMS[i], SMS[i]); gl.clear(gl.DEPTH_BUFFER_BIT);
      const SU = Object.assign({}, FU, { uVP: shM[i] });
      drawCars(prog.car.s.use().setAll(SU));
      drawStat(prog.stat.s.use().setAll(SU));
      if (i === 1) { prog.tree.s.use().setAll(SU); trees.draw(gl.TRIANGLES, TREES.length); }
    }
    gl.colorMask(true, true, true, true);
    tbind(6, shTex[0]); tbind(7, shTex[1]);
    gl.bindFramebuffer(gl.FRAMEBUFFER, T.ms); gl.viewport(0, 0, rw, rh);
    gl.clearBufferfv(gl.COLOR, 0, [0, 0, 0, 1]); gl.clearBufferfv(gl.DEPTH, 0, [1]);
    const MU = Object.assign({}, FU, { uVP: VP, uIVP: invert(VP) });
    gl.depthMask(false); prog.sky.use().setAll(MU); quad.draw(); gl.depthMask(true);
    prog.terr.m.use().setAll(MU).set('uC', [Math.round(cam.pos[0] / 2) * 2, Math.round(cam.pos[2] / 2) * 2]); terrain.draw();
    drawStat(prog.stat.m.use().setAll(MU));
    prog.tree.m.use().setAll(MU); trees.draw(gl.TRIANGLES, TREES.length);
    drawCars(prog.car.m.use().setAll(MU));
    // transparent: glass, particles
    gl.enable(gl.BLEND); gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
    drawCars(prog.glass.use().setAll(MU));
    if (cam.pos[1] > 0) drawStat(prog.stat.m.use().setAll(MU), 1);
    if (sc.parts && sc.parts.n) { parts.update('aP', sc.parts.P); parts.update('aC', sc.parts.C); prog.part.use().setAll(MU).set('uSz', rh * .5); parts.draw(gl.POINTS, 0, 0, sc.parts.n); }
    gl.depthMask(true); gl.disable(gl.BLEND);
    // resolve colour + depth
    gl.bindFramebuffer(gl.READ_FRAMEBUFFER, T.ms);
    gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, T.cf); gl.blitFramebuffer(0, 0, rw, rh, 0, 0, rw, rh, gl.COLOR_BUFFER_BIT, gl.NEAREST);
    gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, T.df); gl.blitFramebuffer(0, 0, rw, rh, 0, 0, rw, rh, gl.DEPTH_BUFFER_BIT, gl.NEAREST);
    tbind(0, T.c); gl.generateMipmap(gl.TEXTURE_2D);
    gl.disable(gl.DEPTH_TEST);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null); gl.viewport(0, 0, cw, ch);
    tbind(0, T.c); tbind(1, T.d);
    const P = sc.post || {};
    prog.final.use().setAll({ uC: 0, uD: 1, uAsp: asp, uFoc: P.focus || 5, uAp: P.ap || 0, uNear: cam.near, uFar: cam.far, uBars: P.bars || 0, uFade: P.fade ?? 1,
      uGrain: P.grain ?? .03, uExp: P.exposure ?? 1, uT: sc.t, uHS: P.hs || 0, uSat: P.sat ?? 1.05, uVig: P.vig ?? .9, uLevels: Math.max(rw, rh) });
    quad.draw();
    gl.enable(gl.DEPTH_TEST);
    return { VP, view, proj };
  }
  return { frame, carMesh, statMesh, gl };
}
