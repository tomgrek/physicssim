const puppeteer = require('puppeteer');

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  page.on('console', msg => console.log('BROWSER:', msg.text()));
  
  await page.goto('http://localhost:5173');
  
  console.log("Waiting for app to load...");
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await page.waitForTimeout(1000); // Wait for MuJoCo
  
  console.log("Adding cube...");
  // Simulate drop event directly since puppeteer drag-and-drop is tricky
  await page.evaluate(() => {
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
  });
  
  console.log("Wait to see if it freezes...");
  
  try {
    // If we can execute JS after 2 seconds, it didn't freeze!
    await page.waitForFunction(() => true, { timeout: 2000 });
    console.log("SUCCESS! The browser did not freeze.");
  } catch (err) {
    console.log("TIMEOUT! The browser froze.");
  }
  
  await browser.close();
})();
