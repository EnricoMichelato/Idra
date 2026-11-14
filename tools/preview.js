// Renders docs/assets/preview.png (README hero) from docs/index.html: arm in the ready pose with the reachable-space slice.
// usage: node tools/preview.js docs
const path = require('path');
const { chromium } = require('playwright');
(async () => {
  const docs = path.resolve(process.argv[2] || 'docs');
  const b = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'] });
  const p = await b.newPage({ viewport: { width: 1100, height: 900 }, deviceScaleFactor: 2, colorScheme: 'light' });
  await p.goto('file://' + path.join(docs, 'index.html'));
  await p.waitForTimeout(5000);
  await p.click('#cart');                       // glides to the ready pose
  await p.waitForTimeout(6000);
  await p.click('#ws');
  await p.waitForTimeout(4000);
  await p.selectOption('#wsm', '2');
  await p.evaluate(() => { document.getElementById('pad').hidden = true; document.getElementById('hud').hidden = true; });
  await p.waitForTimeout(1500);
  await p.locator('#stage').screenshot({ path: path.join(docs, 'assets', 'preview.png') });
  await b.close();
  console.log('docs/assets/preview.png');
})();
