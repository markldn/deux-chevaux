// fps + errors for a page and a scenario: node test/fps.mjs page "js"
const { chromium } = await import('/home/mark/scripts/tracker/node_modules/playwright/index.mjs');
const [page0 = 'index.html', js = ''] = process.argv.slice(2);
const b = await chromium.launch({ headless: true, args: ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist', '--enable-gpu'] });
const p = await b.newPage({ viewport: { width: 1600, height: 900 } }); const errs = [];
p.on('pageerror', e => errs.push('PAGEERROR ' + e.message)); p.on('console', m => { if (m.type() === 'error') errs.push(m.text().slice(0, 300)); });
await p.goto('http://127.0.0.1:9070/' + page0); await p.waitForTimeout(1500);
for (const step of js.split(';;')) { if (step.trim()) await p.evaluate(step); await p.waitForTimeout(400);
  const r = await p.evaluate(() => new Promise(res => { let n = 0; const t0 = performance.now(); const f = () => { n++; performance.now() - t0 < 2000 ? requestAnimationFrame(f) : res(n / 2); }; requestAnimationFrame(f); }));
  console.log(step.slice(0, 50).padEnd(50), 'fps', r); }
console.log(errs.length ? errs.join('\n') : 'NO ERRORS'); await b.close();
