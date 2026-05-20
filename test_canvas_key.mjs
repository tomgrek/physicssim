import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  await page.goto('http://localhost:5173');
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await new Promise(r => setTimeout(r, 1000));
  
  // Count how many times WebGLShadowMap appears
  let shadowMapCount = 0;
  page.on('console', msg => {
    const text = msg.text();
    if (text.includes('WebGLShadowMap')) shadowMapCount++;
    if (text.includes('addComponent') || text.includes('Atomic') || text.includes('rendering')) {
      console.log('BROWSER:', text);
    }
  });
  
  await page.evaluate(() => {
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
  });
  
  await new Promise(r => setTimeout(r, 500));
  console.log("ShadowMap reinit count:", shadowMapCount);
  
  try {
    await page.waitForFunction(() => true, { timeout: 2000 });
    console.log("SUCCESS!");
  } catch {
    console.log("TIMEOUT!");
  }
  
  await browser.close();
})();
