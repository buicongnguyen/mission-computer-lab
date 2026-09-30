// State/event checks for the fleet page only. This test deliberately makes no layout or WebGL claim.
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
    this.style = {
      setProperty(k, v) {
        this[k] = v;
      }
    };
    this.textContent = '';
    this.attributes = {};
    this.hidden = false;
  }
  append(...items) {
    this.children.push(...items);
  }
  addEventListener(name, fn) {
    this.events[name] = fn;
  }
  setAttribute(k, v) {
    this.attributes[k] = v;
  }
  removeAttribute(k) {
    delete this.attributes[k];
    if (k === 'src') delete this.src;
  }
  pause() {
    this.paused = true;
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
// A stand-in for the WebGL scene records what the page asks it to draw.
let loaded = null;
const window = {
  addEventListener() {},
  Replay3D: class {
    load(data) {
      loaded = data;
    }
    setTime(t) {
      this.t = t;
    }
  }
};
vm.runInNewContext(fs.readFileSync(path.join(root, 'artifacts/sitl-sample/report-data.js'), 'utf8'), { window });
const fleet = window.SITL_REPORT.fleet;
assert.ok(fleet, 'the published report carries the fleet run');
let frame = null;
vm.runInNewContext(fs.readFileSync(path.join(root, 'web/fleet.js'), 'utf8'), {
  document,
  window,
  requestAnimationFrame(fn) {
    frame = fn;
  }
});
const text = id => document.getElementById(id).textContent,
  rows = id => document.getElementById(id).children.map(d => d.textContent);
assert.equal(text('error'), '', 'no evidence error');
const checks = Object.values(fleet.checks);
assert.equal(text('overall'), `${checks.length} / ${checks.length} pass`, 'every published fleet check passes');
assert.equal(elements.get('checks').children.length, checks.length);
assert.equal(fleet.vehicles.length, 3);
assert.equal(elements.get('legend').children.length, 4, 'one legend entry per drone plus the carrier');
assert.ok(parseFloat(text('separation')) >= 1, 'closest approach is at least 1 m');
assert.ok(parseFloat(text('travel')) > 0, 'the carrier moved');
assert.ok(parseFloat(text('padError')) < 0.5, 'every drone touched down within 0.5 m of its pad');
const events = rows('events'),
  seconds = events.map(e => parseFloat(e));
assert.ok(
  seconds.every(s => s >= 0),
  'no negative event times'
);
assert.ok(
  seconds.every((s, i) => !i || s >= seconds[i - 1]),
  'events are in time order'
);
for (const v of fleet.vehicles) {
  assert.ok(
    events.some(e => e.includes(`Station grants launch to ${v.ns}`)),
    `${v.ns}: launch clearance shown`
  );
  assert.ok(
    events.some(e => e.includes(`Station grants land to ${v.ns}`)),
    `${v.ns}: landing clearance shown`
  );
  assert.ok(
    events.some(e => e.includes(`${v.ns} touchdown`)),
    `${v.ns}: touchdown shown`
  );
}
// One landing at a time, each land grant after the previous touchdown, and lowest layer first: a descent passes
// through every lower layer, so a drone lands only after every lower one is down.
const landing = events
  .filter(e => /grants land|touchdown/.test(e))
  .map(e => (e.includes('touchdown') ? 'down' : 'grant'));
assert.deepEqual(landing, ['grant', 'down', 'grant', 'down', 'grant', 'down'], 'landings are sequenced one at a time');
const granted = events.filter(e => e.includes('grants land to')).map(e => e.match(/grants land to (\S+)/)[1]);
const byLayer = [...fleet.vehicles].sort((a, b) => a.altitude - b.altitude).map(v => v.ns);
assert.deepEqual(granted, byLayer, 'landings go lowest layer first');
// 3D: a landed drone is drawn riding its pad, not at PX4's estimate, which stops following a moving deck.
assert.ok(loaded, 'the 3D view receives the fleet');
fleet.vehicles.forEach((v, k) => {
  const after = loaded.vehicles[k].samples.filter(s => s.t >= v.touchdown.wall_time),
    last = after.at(-1);
  const c = fleet.carrier.findLast(c => c.wall_time <= last.t);
  assert.ok(after.length && after.every(s => s.u === fleet.deck_height_m), `${v.ns}: on the deck after touchdown`);
  assert.ok(
    Math.hypot(last.e - (c.e + v.pad * Math.cos(c.yaw)), last.n - (c.n + v.pad * Math.sin(c.yaw))) < 1e-9,
    `${v.ns}: rides its pad`
  );
  const before = loaded.vehicles[k].samples.filter(s => s.t < v.touchdown.wall_time);
  assert.deepEqual(
    before.map(s => s.e),
    v.trace.filter(p => p.wall_time < v.touchdown.wall_time).map(p => p.e),
    `${v.ns}: flight is the PX4 estimate`
  );
});
// View switching: the 3D panel is the default; each video view swaps in its own recording.
assert.equal(elements.get('scene3d').hidden, false);
assert.equal(elements.get('videoPanel').hidden, true);
elements.get('viewDeck').events.click();
assert.equal(elements.get('scene3d').hidden, true);
assert.equal(elements.get('videoPanel').hidden, false);
assert.equal(elements.get('viewDeck').attributes['aria-pressed'], 'true');
assert.equal(elements.get('view3d').attributes['aria-pressed'], 'false');
assert.ok(
  elements.get('fleetVideo').src.endsWith(`fleet_carrier/${fleet.videos.deck}`),
  'deck view loads the deck recording'
);
elements.get('viewOverview').events.click();
assert.ok(
  elements.get('fleetVideo').src.endsWith(`fleet_carrier/${fleet.videos.overview}`),
  'overview view loads the overview recording'
);
for (const name of Object.values(fleet.videos))
  assert.ok(fs.existsSync(path.join(root, 'artifacts/sitl-sample/fleet_carrier', name)), `${name} is published`);
elements.get('view3d').events.click();
assert.equal(elements.get('scene3d').hidden, false);
// Playback: advance through animation frames and stop at the end.
const last = Number(elements.get('time').max);
elements.get('play').events.click();
assert.equal(text('play'), 'Pause');
let t = 0;
frame(t);
for (let k = 0; k <= last + 2; k++) {
  t += 100;
  frame(t);
}
assert.equal(elements.get('time').value, last, 'playback reaches the final frame');
assert.equal(text('play'), 'Play', 'playback stops at the end');
// Deep link: a fresh page opened at ?view=deck&t=0.5 shows the deck video and the middle of the run.
elements.clear();
vm.runInNewContext(fs.readFileSync(path.join(root, 'web/fleet.js'), 'utf8'), {
  document,
  window: { ...window, location: { search: '?view=deck&t=0.5' } },
  URLSearchParams,
  requestAnimationFrame() {}
});
assert.equal(elements.get('videoPanel').hidden, false);
assert.equal(elements.get('viewDeck').attributes['aria-pressed'], 'true');
assert.equal(
  elements.get('time').value,
  Math.round(0.5 * Number(elements.get('time').max)),
  'deep link sets the replay time'
);
console.log(
  `Fleet replay checks passed: ${checks.length} checks, clearances and touchdowns per drone, landing order, landed drones on their pads, video views, rAF playback, deep link; layout and WebGL untested.`
);
