import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  
  await page.goto('http://localhost:5173');
  
  await page.waitForFunction(() => !!document.querySelector('canvas'));
  await new Promise(r => setTimeout(r, 1000));
  
  const client = await page.target().createCDPSession();
  await client.send('Profiler.enable');
  await client.send('Profiler.start');
  
  await page.evaluate(() => {
    const e = new Event('drop');
    e.clientX = window.innerWidth / 2;
    e.clientY = window.innerHeight / 2;
    e.dataTransfer = { getData: () => 'box' };
    window.dispatchEvent(e);
  });
  
  await new Promise(r => setTimeout(r, 2000)); // wait 2s for it to freeze
  
  const profile = await client.send('Profiler.stop');
  import('fs').then(fs => fs.writeFileSync('profile.json', JSON.stringify(profile.profile)));
  
  await browser.close();
})();
