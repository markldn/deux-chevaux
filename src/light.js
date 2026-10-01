// Sky/sun uniforms for a time of day (0 dawn .. 1 dusk).
import { sunDir } from './world.js';
import { mix3, clamp, smooth } from './math.js';
export function light(tod) {
  const s = sunDir(tod), e = clamp(s[1], 0, 1), g = smooth(0, .45, e);
  const sunC = mix3([2.3, 1.15, .45], [2.6, 2.4, 2.1], g);
  return {
    uSun: s, uSunC: sunC, uSkyZ: mix3([.1, .17, .36], [.12, .26, .62], g), uSkyH: mix3([.62, .45, .34], [.45, .55, .7], g),
    uGndC: mix3([.2, .15, .08], [.22, .22, .12], g), uFogD: .00035,
  };
}
