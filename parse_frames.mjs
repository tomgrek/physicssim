import fs from 'fs';

const trace = JSON.parse(fs.readFileSync('trace.json', 'utf8'));
const events = trace.traceEvents;

const frames = events.filter(e => e.name === 'FireAnimationFrame');
console.log(`Found ${frames.length} frames.`);
if (frames.length > 0) {
  const first = frames[0].ts;
  const last = frames[frames.length - 1].ts;
  console.log(`Duration: ${(last - first) / 1000}ms`);
}

