// Procedural sound: the 602 cc flat twin (one firing per crank revolution, a buzzy low twin), tyre scrub, wind,
// the tow winch, crash crunch and glass, and a little accordion-ish score for the film.
export function createAudio() {
  const A = new (window.AudioContext || window.webkitAudioContext)();
  const out = A.createDynamicsCompressor(); out.connect(A.destination);
  const master = A.createGain(); master.gain.value = .8; master.connect(out);
  const g = (v, to = master) => { const x = A.createGain(); x.gain.value = v; x.connect(to); return x; };
  const noise = A.createBuffer(1, A.sampleRate * 2, A.sampleRate); { const d = noise.getChannelData(0); for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1; }
  const src = () => { const s = A.createBufferSource(); s.buffer = noise; s.loop = true; s.start(); return s; };
  const bq = (type, f, q = 1) => { const b = A.createBiquadFilter(); b.type = type; b.frequency.value = f; b.Q.value = q; return b; };
  // engine: pulse train at the firing rate through two body resonances, plus intake noise
  const eng = A.createOscillator(); eng.type = 'sawtooth'; const eng2 = A.createOscillator(); eng2.type = 'square';
  const ef = bq('lowpass', 500, 2), ef2 = bq('bandpass', 180, 3), eG = g(0);
  eng.connect(ef); eng2.connect(ef); ef.connect(eG); ef.connect(ef2); ef2.connect(eG); eng.start(); eng2.start();
  const inN = src(), inF = bq('bandpass', 900, 1.5), inG = g(0); inN.connect(inF); inF.connect(inG);
  const wN = src(), wF = bq('lowpass', 500), wG = g(0); wN.connect(wF); wF.connect(wG);
  const tN = src(), tF = bq('bandpass', 1400, 4), tG = g(0); tN.connect(tF); tF.connect(tG);
  const winch = A.createOscillator(); winch.type = 'triangle'; const wiG = g(0); winch.connect(bq('lowpass', 1200)).connect(wiG); winch.start();
  let muted = false, lastCrash = 0, prevSoft = false;
  const set = (p, v, k = .05) => p.setTargetAtTime(isFinite(v) ? v : 0, A.currentTime, k);
  function crunch(e) { // metal: filtered noise bursts + low thump + ringing partials
    const t = A.currentTime;
    for (let i = 0; i < 6; i++) { const s = A.createBufferSource(); s.buffer = noise; const f = bq('bandpass', 300 + Math.random() * 2500, 2 + Math.random() * 4), gg = A.createGain();
      gg.gain.setValueAtTime(0, t + i * .018); gg.gain.linearRampToValueAtTime(.6 * e, t + i * .018 + .005); gg.gain.exponentialRampToValueAtTime(.001, t + i * .018 + .25 + Math.random() * .4);
      s.connect(f); f.connect(gg); gg.connect(master); s.start(t + i * .018, Math.random()); s.stop(t + 1.2); }
    const o = A.createOscillator(), og = A.createGain(); o.frequency.setValueAtTime(90, t); o.frequency.exponentialRampToValueAtTime(30, t + .3);
    og.gain.setValueAtTime(e, t); og.gain.exponentialRampToValueAtTime(.001, t + .5); o.connect(og); og.connect(master); o.start(t); o.stop(t + .6);
    for (let i = 0; i < 5; i++) { const r = A.createOscillator(), rg = A.createGain(); r.frequency.value = 600 + Math.random() * 3000; rg.gain.setValueAtTime(.04 * e, t); rg.gain.exponentialRampToValueAtTime(.0005, t + .8 + Math.random()); r.connect(rg); rg.connect(master); r.start(t); r.stop(t + 2); }
  }
  function glass() { const t = A.currentTime; for (let i = 0; i < 14; i++) { const r = A.createOscillator(), rg = A.createGain(), d = Math.random() * .4; r.type = 'sine'; r.frequency.value = 2500 + Math.random() * 6000; rg.gain.setValueAtTime(0, t + d); rg.gain.linearRampToValueAtTime(.03, t + d + .002); rg.gain.exponentialRampToValueAtTime(.0005, t + d + .15); r.connect(rg); rg.connect(master); r.start(t + d); r.stop(t + d + .2); } }
  return {
    ctx: A,
    event(k) { if (k === 'glass') glass(); if (k === 'crash') crunch(1); },
    update(s) {
      const rpm = s.rpm || 0, f = rpm / 60;
      set(eng.frequency, Math.max(f, 1)); set(eng2.frequency, Math.max(f * 2, 1)); set(ef.frequency, 300 + f * 9);
      set(eG.gain, s.mode === 'film' || s.mode === 'drive' || s.mode === 'lab' ? (rpm > 300 ? .12 + .14 * s.throttle : 0) * (s.slow ? .2 : 1) : 0);
      set(inG.gain, rpm > 300 ? .015 + .05 * s.throttle : 0); set(inF.frequency, 600 + f * 12);
      set(wG.gain, Math.min(.25, s.speed * s.speed * .0004)); set(wF.frequency, 300 + s.speed * 30);
      set(tG.gain, Math.min(.2, (s.slip || 0) * .2));
      set(wiG.gain, s.tow ? .05 : 0); set(winch.frequency, 200 + s.speed * 40);
      if (s.soft && !prevSoft && s.speed > 2 && A.currentTime - lastCrash > 1) { crunch(Math.min(1, s.speed / 12)); if (s.speed > 6) glass(); lastCrash = A.currentTime; }
      prevSoft = s.soft;
    },
    mute() { muted = !muted; master.gain.value = muted ? 0 : .8; }
  };
}
