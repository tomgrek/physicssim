import fs from 'fs';

const profile = JSON.parse(fs.readFileSync('profile.json', 'utf8'));

// Accumulate samples by function name
const times = {};
profile.nodes.forEach(node => {
  const name = node.callFrame.functionName || '(anonymous)';
  const url = node.callFrame.url || '';
  const key = `${name} (${url})`;
  times[node.id] = { key, hits: 0 };
});

profile.samples.forEach(sampleId => {
  if (times[sampleId]) times[sampleId].hits++;
});

const sorted = Object.values(times).sort((a, b) => b.hits - a.hits);
console.log("Top 10 functions by CPU time:");
sorted.slice(0, 10).forEach(t => {
  if (t.hits > 0) console.log(`${t.hits} hits: ${t.key}`);
});
