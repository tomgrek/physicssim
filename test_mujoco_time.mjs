import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  page.on('console', msg => console.log('BROWSER:', msg.text()));
  
  await page.goto('http://localhost:5173');
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await new Promise(r => setTimeout(r, 1500));
  
  // Time the mj_forward call
  await page.evaluate(() => {
    window._origMjForward = null; // placeholder for timing
    console.log("Timing instrumentation ready");
  });
  
  await page.evaluate(() => {
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
  });
  
  await new Promise(r => setTimeout(r, 200));
  
  // Check if JS is still responsive
  const isAlive = await page.evaluate(() => {
    return new Promise(resolve => {
      let ticks = 0;
      const check = () => {
        ticks++;
        if (ticks >= 10) resolve(true);
        else setTimeout(check, 10);
      };
      setTimeout(check, 10);
    });
  }).catch(() => false);
  
  console.log("JS alive after 200ms:", isAlive);
  
  try {
    await page.waitForFunction(() => true, { timeout: 3000 });
    console.log("SUCCESS!");
  } catch {
    console.log("TIMEOUT!");
  }
  
  await browser.close();
})();
