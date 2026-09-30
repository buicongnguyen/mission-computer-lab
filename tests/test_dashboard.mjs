// Exercise dashboard state/events against a minimal DOM; this does not test layout.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
class Element {
  constructor() {
    this.value = '0';
    this.children = [];
    this.events = {};
    this.style = {};
    this.textContent = '';
    this.width = 800;
    this.height = 592;
    this.clientWidth = 400;
  }
  append(...children) {
    this.children.push(...children);
  }
  replaceChildren() {
    this.children = [];
  }
  addEventListener(name, fn) {
    this.events[name] = fn;
  }
  getContext() {
    return new Proxy({}, { get: () => () => {}, set: () => true });
  }
}
const elements = new Map();
const document = {
  getElementById(id) {
    if (!elements.has(id)) elements.set(id, new Element());
    return elements.get(id);
  },
  createElement() {
    return new Element();
  }
};
// Load the same data file the page loads, not report.json.
const window = { addEventListener() {} };
vm.runInNewContext(fs.readFileSync(path.join(root, 'artifacts/sample/report-data.js'), 'utf8'), { window });
const report = window.MISSION_REPORT;
let frame = null;
vm.runInNewContext(fs.readFileSync(path.join(root, 'web/app.js'), 'utf8'), {
  document,
  window,
  requestAnimationFrame(fn) {
    frame = fn;
  },
  console
});
assert.equal(elements.get('scenario').children.length, 8);
assert.equal(elements.get('security').children.length, 6);
for (let i = 0; i < 8; i++) {
  elements.get('scenario').value = String(i);
  elements.get('scenario').events.change();
  assert.equal(elements.get('result').textContent, 'PASS');
  const frames = report.results[i].trace;
  elements.get('time').events.input({ target: { value: String(frames.length - 1) } });
  assert.equal(elements.get('mode').textContent, frames.at(-1).mode);
  assert.ok(elements.get('camera').src.startsWith('data:image/png;base64,'));
}
// Playback: advance through real animation frames, stop at the end, then restart from the beginning.
elements.get('scenario').value = '0';
elements.get('scenario').events.change();
const last = report.results[0].trace.length - 1;
elements.get('play').events.click();
assert.equal(elements.get('play').textContent, 'Pause');
let t = 0;
frame(t);
for (let k = 0; k <= last + 2; k++) {
  t += 80;
  frame(t);
}
assert.equal(elements.get('time').value, last, 'playback reaches the final frame');
assert.equal(elements.get('play').textContent, 'Play', 'playback stops at the end');
elements.get('play').events.click();
t += 80;
frame(t);
assert.equal(elements.get('time').value, 1, 'Play at the end restarts from the beginning');
console.log(
  'Dashboard state checks passed: all 8 scenarios, timeline, images, rAF playback and restart; layout untested.'
);
