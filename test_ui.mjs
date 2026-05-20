import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch({ args: ['--no-sandbox', '--disable-web-security'] });
  const page = await browser.newPage();
  
  await page.goto('http://localhost:5173');
  await new Promise(r => setTimeout(r, 2000));
  
  console.log("Dropping a component...");
  await page.evaluate(() => {
    // Inject script to call addComponent
    const root = document.querySelector('#root');
    const fiber = Object.values(root).find(n => n && n.stateNode && n.stateNode.containerInfo);
    // Well, it's easier to just expose useStore in App.tsx temporarily
  });
  
  await browser.close();
})();
