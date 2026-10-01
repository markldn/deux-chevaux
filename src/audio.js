// Procedural flat-twin exhaust, road noise, impacts and a film-clock three-beat score.
export function createAudio() {
  const A = new (window.AudioContext || window.webkitAudioContext)();
  let seed = 1976;
  const rnd = () => (seed = seed * 16807 % 2147483647) / 2147483647;
  const out = A.createDynamicsCompressor();
  out.threshold.value = -12; out.knee.value = 12; out.ratio.value = 5;
  out.attack.value = .003; out.release.value = .18; out.connect(A.destination);
  const g = (v, to = out) => { const n = A.createGain(); n.gain.value = v; n.connect(to); return n; };
  const master = g(.75), effects = g(1, master), music = g(0, master);
  const bq = (type, f, q = .7) => { const n = A.createBiquadFilter(); n.type = type; n.frequency.value = f; n.Q.value = q; return n; };
  const noise = A.createBuffer(1, A.sampleRate * 2, A.sampleRate);
  const nd = noise.getChannelData(0); for (let i = 0; i < nd.length; i++) nd[i] = rnd() * 2 - 1;
  const set = (p, v, k = .05) => p.setTargetAtTime(Number.isFinite(v) ? v : 0, A.currentTime, k);
  const loop = (filter, gain) => { const n = A.createBufferSource(); n.buffer = noise; n.loop = true; n.connect(filter).connect(gain); n.start(0, rnd()); };
  // Narrow combustion pulses with unequal harmonics, softened by the exhaust body.
  const re = new Float32Array(33), im = new Float32Array(33);
  for (let i = 1; i < im.length; i++) im[i] = Math.exp(-i * .13) * (i % 2 ? 1 : .65);
  const eng = A.createOscillator(); eng.setPeriodicWave(A.createPeriodicWave(re, im));
  const ef = bq('lowpass', 600), eb = bq('peaking', 140, 1.2), eG = g(0, effects);
  eb.gain.value = 5; eng.connect(ef).connect(eb).connect(eG); eng.start();
  const fan = A.createOscillator(), fanG = g(0, effects); fan.type = 'triangle'; fan.connect(bq('bandpass', 1800, .6)).connect(fanG); fan.start();
  const inF = bq('bandpass', 900), inG = g(0, effects); loop(inF, inG);
  const wF = bq('lowpass', 500), wG = g(0, effects); loop(wF, wG);
  const roadF = bq('bandpass', 220, .5), roadG = g(0, effects); loop(roadF, roadG);
  const tF = bq('bandpass', 1600, 1.3), tG = g(0, effects); loop(tF, tG);
  const winch = A.createOscillator(), wiG = g(0, effects); winch.type = 'triangle'; winch.connect(bq('lowpass', 1200)).connect(wiG); winch.start();
  // Generated stereo room tail for the score; impact sounds remain close and dry.
  const rev = A.createConvolver(), ir = A.createBuffer(2, A.sampleRate * .8, A.sampleRate);
  for (let c = 0; c < 2; c++) { const d = ir.getChannelData(c); for (let i = 0; i < d.length; i++) d[i] = (rnd() * 2 - 1) * Math.exp(-i / A.sampleRate * 9); }
  rev.buffer = ir; music.connect(rev); rev.connect(g(.16, master));
  const voices = new Set();
  function envelope(p, t, peak, dur, attack = .003) {
    p.setValueAtTime(0, t); p.linearRampToValueAtTime(peak, t + attack);
    p.exponentialRampToValueAtTime(.0001, t + dur); p.setValueAtTime(0, t + dur + .005);
  }
  function finish(n, nodes, t, score = false) {
    if (score) voices.add(n);
    n.onended = () => { voices.delete(n); n.disconnect(); for (const x of nodes) x.disconnect(); };
    n.stop(t);
  }
  function burst(t, f, q, amp, dur, rate = 1) {
    const n = A.createBufferSource(), filter = bq('bandpass', f, q), v = g(0, effects);
    n.buffer = noise; n.playbackRate.value = rate; envelope(v.gain, t, amp, dur);
    filter.frequency.setValueAtTime(f, t); filter.frequency.exponentialRampToValueAtTime(f * .5, t + dur);
    n.connect(filter).connect(v); n.start(t, rnd() * .6); finish(n, [filter, v], t + dur + .01);
  }
  function tone(t, f, amp, dur, type = 'sine', bus = effects, end = f, score = false) {
    const n = A.createOscillator(), v = g(0, bus); n.type = type;
    n.frequency.setValueAtTime(f, t); n.frequency.exponentialRampToValueAtTime(end, t + dur);
    envelope(v.gain, t, amp, dur, score ? .02 : .003); n.connect(v); n.start(t); finish(n, [v], t + dur + .01, score);
  }
  function glass(e = 1, slow = false) {
    const t = A.currentTime, pitch = slow ? .65 : 1;
    burst(t, 4800 * pitch, .6, .24 * e, .09 / pitch, pitch);
    for (let i = 0; i < 20; i++) {
      const d = .025 + rnd() * rnd() * .9, f = (2200 + rnd() * 7000) * pitch, dur = .025 + rnd() * .09;
      tone(t + d, f, (.009 + rnd() * .014) * e, dur / pitch);
      if (i % 4 === 0) burst(t + d, f, 2, .06 * e, dur, pitch);
    }
  }
  function crunch(e, slow = false) {
    const t = A.currentTime, pitch = slow ? .65 : 1;
    tone(t, 105 * pitch, .65 * e, .22 / pitch, 'sine', effects, 32 * pitch);
    burst(t, 1700 * pitch, .7, .8 * e, .09 / pitch, pitch);
    for (let i = 0; i < 8; i++) {
      const d = i * .025 / pitch, f = (160 + rnd() * 1200) * pitch;
      burst(t + d, f, .8 + rnd() * 2, e * (.22 + rnd() * .2), (.1 + rnd() * .2) / pitch, pitch);
      tone(t + d, f, .035 * e, (.07 + rnd() * .17) / pitch, 'triangle', effects, f * .84);
    }
    for (let i = 0; i < 5; i++) burst(t + .25 + i * .1, 350 + rnd() * 1400, 2, .045 * e, .06);
    duckUntil = t + 1.2;
  }
  const chords = [[57, 60, 64], [53, 57, 60], [50, 53, 57], [52, 56, 59]];
  const melody = [0, 2, 1, 2, 4, 2, 1, 0, 2, 1, 0, -1], beat = 60 / 112;
  let nextBeat = 0, lastFilm = null, lastShot = -1, scorePlaying = false;
  function stopScore() { for (const n of voices) { try { n.stop(); } catch {} } voices.clear(); }
  function score(s) {
    const playing = s.mode === 'film' && !s.paused && Number.isFinite(s.filmTime);
    if (!playing) { if (scorePlaying) stopScore(); scorePlaying = false; lastFilm = null; set(music.gain, 0, .025); return; }
    const ft = s.filmTime, shot = s.shot || 0;
    if (lastFilm === null || ft < lastFilm || ft - lastFilm > .3 || shot !== lastShot) {
      stopScore(); nextBeat = Math.ceil(ft / beat - 1e-6);
    }
    lastFilm = ft; lastShot = shot; scorePlaying = true;
    const tension = shot === 3 || shot === 4 || shot === 6;
    set(music.gain, (A.currentTime < duckUntil ? .15 : 1) * (tension ? .6 : .85), .08);
    while (nextBeat * beat < ft + .12) {
      const n = nextBeat++, t = A.currentTime + Math.max(0, n * beat - ft), ch = chords[Math.floor(n / 6) % 4];
      const freq = m => 440 * 2 ** ((m - 69) / 12);
      if (n % 3 === 0) tone(t, freq(ch[0] - 12), .12, .4, 'triangle', music, freq(ch[0] - 12), true);
      else for (const m of ch) tone(t, freq(m), .035, .21, 'triangle', music, freq(m), true);
      if (!tension || n % 3 === 0) {
        const ix = melody[n % melody.length];
        if (ix >= 0) {
          const m = ch[ix % 3] + 12 + (ix > 2 ? 12 : 0), f = freq(m);
          tone(t, f, .055, tension ? .7 : .43, 'triangle', music, f, true);
          tone(t, f * 2.003, .012, .35, 'sine', music, f * 2, true);
        }
      }
    }
  }
  let muted = false, lastCrash = -10, lastGlass = -10, prevGlass = 0, prevPulse = 0, lastRun = null, duckUntil = 0;
  return {
    ctx: A,
    event(k) { if (k === 'glass') glass(); if (k === 'crash') crunch(1); },
    update(s) {
      const active = s.mode !== 'menu' && !s.paused, rpm = s.rpm || 0, f = rpm / 60;
      const scale = s.slow ? .65 : 1, run = s.runTime;
      if (lastRun !== null && run < lastRun) { lastCrash = lastGlass = -10; prevPulse = prevGlass = 0; }
      lastRun = run;
      set(eng.frequency, Math.max(1, f * scale)); set(ef.frequency, (350 + f * 7 + (s.throttle || 0) * 700) * scale);
      set(eG.gain, active && rpm > 300 && !s.soft ? .13 + .1 * (s.throttle || 0) : 0);
      set(fan.frequency, Math.max(1, f * 7 * scale)); set(fanG.gain, active && rpm > 300 && !s.soft ? .012 : 0);
      set(inG.gain, active && rpm > 300 && !s.soft ? .025 + .055 * (s.throttle || 0) : 0); set(inF.frequency, 600 + f * 8);
      set(wG.gain, active ? Math.min(.15, s.speed ** 2 * .0002) : 0); set(wF.frequency, 250 + s.speed * 20);
      set(roadG.gain, active ? Math.min(.12, s.speed * .004) : 0);
      set(tG.gain, active ? Math.min(.16, Math.max(0, (s.slip || 0) - .08) * .15) : 0);
      set(wiG.gain, active && s.tow ? .045 : 0); set(winch.frequency, 180 + s.speed * 22);
      const pulse = s.pulse || 0;
      if (active && s.impact !== false && pulse > 4 && pulse > prevPulse + 1 && A.currentTime - lastCrash > .7) {
        crunch(Math.min(1, .25 + pulse / 45), s.slow);
        lastCrash = A.currentTime;
      }
      if (active && s.glass > prevGlass && A.currentTime - lastGlass > .25) {
        glass(Math.min(1, .3 + (s.glass - prevGlass) / 100), s.slow); lastGlass = A.currentTime;
      }
      prevGlass = s.glass || 0;
      prevPulse = pulse; score(s);
    },
    mute() { muted = !muted; set(master.gain, muted ? 0 : .75, .015); }
  };
}
