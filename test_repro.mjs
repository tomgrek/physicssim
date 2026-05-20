import puppeteer from 'puppeteer';

(async () => {
  const browser = await puppeteer.launch({ args: ['--no-sandbox'] });
  const page = await browser.newPage();
  
  page.on('console', msg => console.log('BROWSER:', msg.text()));
  
  await page.goto('http://localhost:5173');
  await page.waitForFunction('window.useStore !== undefined');
  
  console.log("Checking UI state before drag...");
  let state = await page.evaluate(() => {
    return {
      selectedNodeId: window.useStore.getState().selectedNodeId,
      nodesCount: window.useStore.getState().sceneGraph.nodes.length
    }
  });
  console.log(state);
  
  console.log("Adding component...");
  await page.evaluate(() => {
    window.useStore.getState().addComponent('box', [1, 1, 2]);
  });
  
  await new Promise(r => setTimeout(r, 1000));
  
  console.log("Checking UI state after drag...");
  state = await page.evaluate(() => {
    return {
      selectedNodeId: window.useStore.getState().selectedNodeId,
      nodesCount: window.useStore.getState().sceneGraph.nodes.length
    }
  });
  console.log(state);
  
  await browser.close();
})();
