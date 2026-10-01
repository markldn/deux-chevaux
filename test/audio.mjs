// Exercise the real Web Audio graph offline, including transport changes and impact timing.
import fs from 'node:fs';
import assert from 'node:assert/strict';
const { chromium } = await import('playwright').catch(() => import(process.env.PLAYWRIGHT || '/home/mark/scripts/tracker/node_modules/playwright/index.mjs'));
const source = fs.readFileSync(new URL('../src/audio.js', import.meta.url), 'utf8');
const browser = await chromium.launch({ headless: true });
try {
  const page = await browser.newPage();
  const results = await page.evaluate(async source => {
    const { createAudio } = await import('data:text/javascript;base64,' + btoa(source));
    const results = {};
    for (const kind of ['menu', 'approach', 'engine', 'muted', 'frozen', 'impact', 'glass', 'film']) {
      const ctx = new OfflineAudioContext(2, 44100 * 3, 44100);
      const old = window.AudioContext; window.AudioContext = function () { return ctx; };
      const audio = createAudio(); window.AudioContext = old;
      const update = t => {
        const film = kind === 'film', engine = ['engine', 'muted', 'frozen'].includes(kind);
        audio.update({ mode: kind === 'menu' || film && t >= 2 ? 'menu' : film ? 'film' : 'lab',
          rpm: engine ? 2800 : 0, throttle: .5, speed: engine ? 12 : 0, slip: kind === 'frozen' ? .6 : 0, frozen: kind === 'frozen' && t >= .5, soft: kind === 'approach' || kind === 'impact',
          pulse: kind === 'approach' ? 20 : kind === 'impact' && t >= .5 && t < .65 ? 30 : 0, impact: kind !== 'approach', glass: 0, runTime: t,
          filmTime: t < 1 ? t : t + 50, shot: t < 1 ? 0 : 3, paused: film && t >= 1.5 && t < 2 });
      };
      update(0); if (kind === 'glass') audio.event('glass');
      const waits = [];
      for (let i = 1; i < 24; i++) waits.push(ctx.suspend(i / 8));
      const render = ctx.startRendering();
      for (let i = 0; i < waits.length; i++) {
        await waits[i]; update(ctx.currentTime);
        if (kind === 'muted' && i === 3) audio.mute();
        await ctx.resume();
      }
      const buf = await render, d = buf.getChannelData(0);
      let peak = 0, energy = 0, early = 0, tail = 0;
      for (let i = 0; i < d.length; i++) {
        if (!Number.isFinite(d[i])) throw new Error(kind + ': non-finite sample');
        peak = Math.max(peak, Math.abs(d[i])); energy += d[i] ** 2;
        if (i < 44100 * .4) early += d[i] ** 2;
        if (i > 44100 * 2.8) tail += d[i] ** 2;
      }
      results[kind] = { peak, rms: Math.sqrt(energy / d.length), early: Math.sqrt(early / (44100 * .4)), tail: Math.sqrt(tail / (44100 * .2)) };
    }
    return results;
  }, source);
  for (const [kind, r] of Object.entries(results)) {
    console.log(kind, Object.fromEntries(Object.entries(r).map(([k, v]) => [k, +v.toFixed(6)])));
    assert(r.peak < 1, kind + ' clips');
    if (['menu', 'approach'].includes(kind)) assert(r.peak === 0, kind + ' should be silent');
    else assert(r.rms > .001, kind + ' should produce audible signal');
  }
  assert(results.impact.early === 0, 'impact sounded before collision');
  assert(results.film.tail < .0001, 'score did not stop after leaving film');
  assert(results.muted.tail < .0001, 'mute did not silence the running engine');
  assert(results.frozen.early > .001 && results.frozen.tail < .0001, 'driving noise carried into the frozen anatomy shot');
  console.log('Audio synthesis and film transport checks passed');
} finally { await browser.close(); }
