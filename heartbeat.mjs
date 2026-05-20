import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  page.on('console', msg => console.log('BROWSER:', msg.text()));
  
  await page.goto('http://localhost:5173');
  
  await page.evaluate(() => {
    window.lastBeat = Date.now();
    setInterval(() => {
      console.log("Heartbeat! Diff:", Date.now() - window.lastBeat);
      window.lastBeat = Date.now();
    }, 100);
  });
  
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await new Promise(r => setTimeout(r, 1000));
  
  console.log("Adding cube...");
  await page.evaluate(() => {
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
  });
  
  await new Promise(r => setTimeout(r, 2000));
  
  await browser.close();
})();
