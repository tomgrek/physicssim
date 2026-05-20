import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  page.on('console', msg => console.log('BROWSER:', msg.text()));
  
  // Intercept the store to trace exactly where it hangs
  await page.goto('http://localhost:5173');
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await new Promise(r => setTimeout(r, 1500));
  
  // Instrument the React rendering by adding a MutationObserver to detect DOM changes
  const mutationCount = await page.evaluate(async () => {
    let count = 0;
    const observer = new MutationObserver(() => count++);
    observer.observe(document.body, { childList: true, subtree: true, attributes: true });
    
    // Drop a cube
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
    
    // Wait 200ms, see how many mutations happened
    await new Promise(r => setTimeout(r, 200));
    observer.disconnect();
    return count;
  }).catch(() => -1);
  
  console.log("DOM mutations in 200ms:", mutationCount);
  
  try {
    await page.waitForFunction(() => true, { timeout: 2000 });
    console.log("SUCCESS!");
  } catch {
    console.log("TIMEOUT!");
  }
  
  await browser.close();
})();
