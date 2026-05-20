import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  page.on('console', msg => {
    const text = msg.text();
    if (text.includes('WebGL') || text.includes('THREE') || text.includes('Canvas') || text.includes('Atomic') || text.includes('rendering') || text.includes('addComponent') || text.includes('recompile')) {
      console.log('BROWSER:', text);
    }
  });
  
  await page.goto('http://localhost:5173');
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await new Promise(r => setTimeout(r, 1000));
  
  await page.evaluate(() => {
    window.DISABLE_PHYSICS = true;
    window.DISABLE_SCENE = false;
  });
  
  // Track how many canvases get created
  await page.evaluate(() => {
    let count = 0;
    const observer = new MutationObserver((mutations) => {
      mutations.forEach(m => m.addedNodes.forEach(n => {
        if (n.tagName === 'CANVAS') {
          count++;
          console.log('New canvas created! count:', count);
        }
      }));
    });
    observer.observe(document.body, { childList: true, subtree: true });
  });
  
  await page.evaluate(() => {
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
  });
  
  try {
    await page.waitForFunction(() => true, { timeout: 3000 });
    console.log("SUCCESS!");
  } catch {
    console.log("TIMEOUT!");
  }
  
  await browser.close();
})();
