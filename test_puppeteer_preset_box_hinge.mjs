import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  page.on('console', msg => console.log('BROWSER:', msg.text()));
  page.on('pageerror', err => console.error('BROWSER ERROR:', err.message));
  
  console.log("Navigating to http://localhost:5173...");
  await page.goto('http://localhost:5173');
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  
  console.log("Waiting for MuJoCo to initialize...");
  await page.waitForFunction(() => {
    const text = document.body.textContent || '';
    return !text.includes('Initializing MuJoCo');
  });
  console.log("MuJoCo initialized!");
  
  console.log("Clicking 'Simulate'...");
  await page.evaluate(() => {
    const btn = Array.from(document.querySelectorAll('button')).find(b => b.textContent.includes('Simulate'));
    if (btn) btn.click();
  });
  
  await new Promise(r => setTimeout(r, 3000));
  
  console.log("Checking if the browser is responsive...");
  const responsive = await page.evaluate(() => {
    return typeof window !== 'undefined';
  });
  
  console.log("Responsive status:", responsive);
  await browser.close();
})();
