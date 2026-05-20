import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  page.on('console', msg => console.log('BROWSER:', msg.text()));
  
  await page.goto('http://localhost:5173');
  
  console.log("Waiting for app to load...");
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
  
  console.log("Wait to see if it freezes...");
  
  try {
    await page.waitForFunction(() => {
      // test if we can run code in browser
      return true;
    }, { timeout: 2000 });
    console.log("SUCCESS! The browser did not freeze.");
  } catch (err) {
    console.log("TIMEOUT! The browser froze.");
  }
  
  await browser.close();
})();
