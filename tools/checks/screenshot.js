// Screenshot a viewer page with the bundled Chromium (no GPU: software WebGL).
//   node tools/checks/screenshot.js mechanical/cad/arm_v0/arm_3d.html out.png [width] [height]
const { chromium } = require('playwright');
const path = require('path');
(async () => {
  const [src, out, w = 1200, h = 900] = process.argv.slice(2);
  const b = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'] });
  const p = await b.newPage({ viewport: { width: +w, height: +h }, deviceScaleFactor: 1 });
  const errs = []; p.on('pageerror', e => errs.push(String(e)));
  await p.goto('file://' + path.resolve(src)); await p.waitForTimeout(6000);
  await p.screenshot({ path: out });
  console.log(errs.length ? 'page errors: ' + errs.join(' | ') : 'ok');
  await b.close();
})();
