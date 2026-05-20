import puppeteer from 'puppeteer';

const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

(async () => {
  console.log("Starting gears simulation test...");
  const browser = await puppeteer.launch({
    args: ['--no-sandbox', '--disable-setuid-sandbox', '--disable-web-security']
  });
  const page = await browser.newPage();
  await page.setCacheEnabled(false);

  page.on('console', msg => {
    const txt = msg.text();
    if (!txt.includes('DynamicGeom') && !txt.includes('useFrame') && !txt.includes('ShadowMap')) {
      console.log('BROWSER CONSOLE:', txt);
    }
  });
  page.on('pageerror', err => {
    console.log('BROWSER PAGE ERROR:', err.toString());
  });

  await page.goto('http://localhost:5173');
  await delay(2000);

  console.log("Loading gears preset...");
  await page.evaluate(() => {
    window.useStore.getState().loadPreset('gears');
  });
  
  await delay(1000);
  
  console.log("Status before simulate:");
  const statusBefore = await page.evaluate(() => {
    const data = window.useStore.getState().data;
    return {
      ctrl: Array.from(data.ctrl),
      qpos: Array.from(data.qpos),
      qvel: Array.from(data.qvel)
    };
  });
  console.log(statusBefore);

  console.log("Starting simulation...");
  await page.evaluate(() => {
    window.useStore.getState().togglePlay();
  });

  await delay(2000);

  console.log("Status after 2s:");
  const statusAfter = await page.evaluate(() => {
    const data = window.useStore.getState().data;
    return {
      ctrl: Array.from(data.ctrl),
      qpos: Array.from(data.qpos),
      qvel: Array.from(data.qvel)
    };
  });
  console.log(statusAfter);

  await browser.close();
  console.log("Done");
})();
