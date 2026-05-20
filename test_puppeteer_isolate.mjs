import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  page.on('console', msg => console.log('BROWSER:', msg.text()));
  
  await page.goto('http://localhost:5173');
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await new Promise(r => setTimeout(r, 1000));
  
  // Expose a function to disable SceneVisuals
  await page.evaluate(() => {
    window.DISABLE_SCENE = true;
  });
  
  await page.evaluate(() => {
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
  });
  
  try {
    await page.waitForFunction(() => true, { timeout: 2000 });
    console.log("SUCCESS! The browser did not freeze.");
  } catch (err) {
    console.log("TIMEOUT! The browser froze.");
  }
  
  await browser.close();
})();
