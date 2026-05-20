import puppeteer from 'puppeteer';

async function testFreeze(physics, scene) {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  await page.goto('http://localhost:5173');
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await new Promise(r => setTimeout(r, 1000));
  
  await page.evaluate((p, s) => {
    window.DISABLE_PHYSICS = !p;
    window.DISABLE_SCENE = !s; 
  }, physics, scene);
  
  await page.evaluate(() => {
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
  });
  
  try {
    await page.waitForFunction(() => true, { timeout: 2000 });
    console.log(`P=${physics} S=${scene}: SUCCESS!`);
  } catch (err) {
    console.log(`P=${physics} S=${scene}: TIMEOUT!`);
  }
  
  await browser.close();
}

await testFreeze(true, false);
await testFreeze(false, true);
