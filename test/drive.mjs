// drive-mode reproduction: crash into a plane tree, press P, screenshots. node test/drive.mjs outprefix
const { chromium } = await import('/home/mark/scripts/tracker/node_modules/playwright/index.mjs');
const out = process.argv[2] || '/tmp/claude-1000/ct/dr';
const b = await chromium.launch({ headless: true, args: ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist', '--enable-gpu'] });
const p = await b.newPage({ viewport: { width: 1280, height: 720 } }); const errs = [];
p.on('pageerror', e => errs.push('PAGEERROR ' + e.message)); p.on('console', m => { if (m.text().startsWith('LOG')) errs.push(m.text()); });
await p.goto('http://127.0.0.1:9070/dev.html'); await p.waitForTimeout(1200);
await p.evaluate(() => { __ct.enter('drive'); __ct.A.place([33.2, 0, 140.5], .15, 55 / 3.6); });
for (let i = 0; i < 4; i++) { await p.waitForTimeout(1200); await p.screenshot({ path: `${out}_${i}.png` }); }
const st = await p.evaluate(() => ({ rec: __ct.S.rec.length, phase: __ct.S.phase, mode: __ct.A.mode, rep: !!__ct.S.replay }));
console.log('before P', JSON.stringify(st));
await p.keyboard.press('KeyP'); await p.waitForTimeout(1500);
console.log('after P', JSON.stringify(await p.evaluate(() => ({ rec: __ct.S.rec.length, phase: __ct.S.phase, rep: !!__ct.S.replay, i: __ct.S.replay?.i }))));
await p.screenshot({ path: `${out}_4.png` }); await p.waitForTimeout(1500); await p.screenshot({ path: `${out}_5.png` });
console.log(errs.join('\n') || 'NO ERRORS'); await b.close();
