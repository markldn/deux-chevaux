// Headless real-GPU screenshots. node test/view.mjs out.png 'js to run before (sets views)' [waitMs] [w] [h] [page]
import fs from 'fs';
const { chromium } = await import(process.env.PLAYWRIGHT || '/home/mark/scripts/tracker/node_modules/playwright/index.mjs');
const [out = '/tmp/v.png', js = '', wait = 2500, w = 1280, h = 720, page0 = 'dev.html'] = process.argv.slice(2);
const b = await chromium.launch({ headless: true, args: ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist', '--enable-gpu', '--autoplay-policy=no-user-gesture-required'] });
const p = await b.newPage({ viewport: { width: +w, height: +h } });
const errs = [];
p.on('pageerror', e => errs.push('PAGEERROR ' + e.message));
p.on('console', m => { if (m.type() === 'error' || m.type() === 'warning' || m.text().startsWith('LOG')) errs.push(m.type() + ' ' + m.text().slice(0, 600)); });
const t0 = Date.now();
await p.goto('http://127.0.0.1:9070/' + page0);
await p.waitForTimeout(800);
// js may contain several views separated by ';;' -> out_0.png, out_1.png ...
const views = js.split(';;');
for (let i = 0; i < views.length; i++) {
  if (views[i].trim()) await p.evaluate(views[i]).catch(e => errs.push('EVAL ' + e.message));
  await p.waitForTimeout(+wait);
  await p.screenshot({ path: views.length > 1 ? out.replace('.png', `_${i}.png`) : out });
}
console.log('load+shots ms', Date.now() - t0);
console.log(errs.length ? [...new Set(errs)].slice(0, 20).join('\n') : 'NO ERRORS');
await b.close();
