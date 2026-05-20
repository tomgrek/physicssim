import fs from 'fs';

const trace = JSON.parse(fs.readFileSync('trace.json', 'utf8'));
const events = trace.traceEvents;

// Find events with longest duration
const xEvents = events.filter(e => e.ph === 'X' && e.name !== 'MessageLoop::RunTask').sort((a, b) => b.dur - a.dur);
console.log("Top 10 Longest Events:");
xEvents.slice(0, 10).forEach(e => {
  console.log(`${e.name}: ${e.dur / 1000}ms, args: ${JSON.stringify(e.args)}`);
});

// Check if there's a repeating function call causing a stack overflow or infinite loop
const counts = {};
events.forEach(e => {
  if (e.name === 'FunctionCall' || e.name === 'EvaluateScript') {
    counts[e.name] = (counts[e.name] || 0) + 1;
  }
});
console.log("Event counts:", counts);
