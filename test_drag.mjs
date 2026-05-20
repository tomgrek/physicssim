import puppeteer from 'puppeteer';

(async () => {
  console.log("Starting Puppeteer test...");
  const browser = await puppeteer.launch({
    args: ['--no-sandbox', '--disable-setuid-sandbox', '--disable-web-security']
  });
  const page = await browser.newPage();

  page.on('console', msg => {
    if (msg.type() === 'error') {
      console.log('BROWSER ERROR:', msg.text());
    } else {
      console.log('BROWSER:', msg.text());
    }
  });
  page.on('pageerror', err => {
    console.log('BROWSER PAGE ERROR:', err.toString());
  });

  await page.goto('http://localhost:5173');
  await new Promise(r => setTimeout(r, 2000)); //(2000);

  console.log("Dispatching Drop Event...");
  await page.evaluate(() => {
    const main = document.querySelector('main');
    const evt = new DragEvent('drop', {
      bubbles: true,
      cancelable: true,
      clientX: 500,
      clientY: 500,
      dataTransfer: new DataTransfer()
    });
    evt.dataTransfer.setData('component_type', 'box');
    window.useStore.getState().addComponent("box", [1, 2, 3]);
  });
  
  await new Promise(r => setTimeout(r, 2000)); //(2000);
  
  await browser.close();
  console.log("Done");
})();
