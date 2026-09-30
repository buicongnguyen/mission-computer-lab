// State/event checks only. This test deliberately makes no layout claim.
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
    this.height = 610;
    this.clientWidth = 640;
  }
  append(...items) {
    this.children.push(...items);
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
const elements = new Map(),
  document = {
    getElementById(id) {
      if (!elements.has(id)) elements.set(id, new Element());
      return elements.get(id);
    },
    createElement() {
      return new Element();
    }
  };
// Load the same data file the page loads, not report.json.
const window = { devicePixelRatio: 2, addEventListener() {} };
vm.runInNewContext(fs.readFileSync(path.join(root, 'artifacts/sitl-sample/report-data.js'), 'utf8'), { window });
const report = window.SITL_REPORT;
let frame = null;
vm.runInNewContext(fs.readFileSync(path.join(root, 'web/sitl.js'), 'utf8'), {
  document,
  window,
  requestAnimationFrame(fn) {
    frame = fn;
  }
});
assert.equal(elements.get('scenario').children.length, 4);
assert.equal(elements.get('overall').textContent, '4 / 4 pass');
assert.equal(elements.get('map').width, 1280, 'canvas bitmap follows displayed width × devicePixelRatio');
const events = () => elements.get('events').children.map(d => d.textContent);
for (let i = 0; i < 4; i++) {
  elements.get('scenario').value = String(i);
  elements.get('scenario').events.change();
  const result = report.results[i];
  assert.equal(elements.get('checks').children.length, Object.keys(result.checks).length);
  assert.ok(events().length >= 4);
  assert.ok(
    events().every(t => !/PX4 \d/.test(t)),
    `${result.scenario}: every PX4 navigation state has a name`
  );
  assert.ok(
    events().every(t => !t.startsWith('-')),
    `${result.scenario}: no negative event times`
  );
  assert.ok(
    events().every((t, j, a) => !j || t !== a[j - 1]),
    `${result.scenario}: no repeated event rows`
  );
  if (result.statuses.some(s => s.nav_state === 12))
    assert.ok(
      events().some(t => t.includes('PX4 DESCEND')),
      'failsafe descend is named'
    );
  elements.get('time').events.input({ target: { value: String(result.trace.length - 1) } });
  assert.ok(elements.get('state').textContent.includes('DISARMED'));
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
  t += 100;
  frame(t);
}
assert.equal(elements.get('time').value, last, 'playback reaches the final frame');
assert.equal(elements.get('play').textContent, 'Play', 'playback stops at the end');
elements.get('play').events.click();
t += 100;
frame(t);
assert.equal(elements.get('time').value, 1, 'Play at the end restarts from the beginning');
console.log(
  'SITL replay checks passed: four scenarios, named PX4 states, event rows, final disarm, rAF playback and restart; layout untested.'
);
