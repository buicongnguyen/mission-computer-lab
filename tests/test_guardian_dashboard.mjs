// State checks for the guardian page only. This test deliberately makes no layout, canvas-pixel or WebGL claim.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
class Element {
  constructor(tag) {
    this.tag = tag;
    this.value = '';
    this.children = [];
    this.events = {};
    this.textContent = '';
    this.attributes = {};
    this.hidden = false;
    this.dataset = {};
    this.style = {
      setProperty(k, v) {
        this[k] = v;
      }
    };
    this.width = 800;
    this.height = 610;
    this.clientWidth = 640;
  }
  append(...items) {
    this.children.push(...items);
  }
  replaceChildren(...items) {
    this.children = [...items];
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
  getContext() {
    return new Proxy({}, { get: () => () => {}, set: () => true });
  }
}
const elements = new Map(),
  document = {
    getElementById(id) {
      if (!elements.has(id)) elements.set(id, new Element('div'));
      return elements.get(id);
    },
    createElement(tag) {
      return new Element(tag);
    }
  };
const $ = id => document.getElementById(id),
  text = e => e.textContent + e.children.map(text).join('');
let loaded = null;
const window = {
  devicePixelRatio: 1,
  addEventListener() {},
  Replay3D: class {
    load(d) {
      loaded = d;
    }
    setTime() {}
  }
};
for (const file of [
  'artifacts/guardian/report-data.js',
  'artifacts/sitl-sample/report-data.js',
  'artifacts/sitl-sample/guardian-data.js'
])
  vm.runInNewContext(fs.readFileSync(path.join(root, file), 'utf8'), { window });
const R = window.GUARDIAN_REPORT,
  FLIGHTS = window.SITL_GUARDIANS;
assert.ok(!('guardians' in window.SITL_REPORT), 'the flight and fleet pages do not load the guardian flights');
assert.ok(
  R && FLIGHTS && Object.keys(FLIGHTS).length === 8,
  'the simulator report and all eight PX4 guardian flights are published'
);
let frame = null;
vm.runInNewContext(fs.readFileSync(path.join(root, 'web/guardian.js'), 'utf8'), {
  document,
  window,
  requestAnimationFrame(fn) {
    frame = fn;
  }
});
assert.equal(text($('error')), '', 'no evidence error');
// Headline tiles and the comparison tables come straight from the report.
assert.equal($('kpis').children.length, 4);
assert.ok(
  text($('kpis')).includes(`${R.summary.intruder.hybrid.warning_s.median.toFixed(0)} s`),
  'hybrid warning tile'
);
const scenarios = Object.keys(R.summary);
for (const id of ['heatSafe', 'heatWarn']) {
  const rows = $(id).children;
  assert.equal(rows.length, scenarios.length + 1, `${id}: a header and one row per scenario`);
  rows.slice(1).forEach(r => assert.equal(r.children.length, 5));
}
const safeCell = $('heatSafe').children[1 + scenarios.indexOf('jamming')].children[4];
assert.equal(safeCell.textContent, Math.round(R.summary.jamming.hybrid.guardians_safe_rate * 100) + '%');
assert.ok(safeCell.title.includes('Hybrid'), 'cells carry an accessible description');
assert.equal($('layouts').children.length, 1 + Object.keys(R.layouts.summary).length);
assert.equal($('authority').children.length, 4);
// Replay: every scenario loads, only designs with a recorded replay are selectable, and playback advances.
assert.equal($('scenario').children.length, scenarios.length);
for (const s of scenarios) {
  $('scenario').value = s;
  $('scenario').events.change();
  const buttons = $('archButtons').children,
    enabled = buttons.filter(b => !b.disabled).map(b => b.dataset.arch);
  assert.deepEqual(
    enabled.sort(),
    Object.keys(R.traces[s]).sort(),
    `${s}: selectable designs match the recorded replays`
  );
  assert.ok($('events').children.length > 0 || s === 'spoofing', `${s}: decision log`);
  assert.ok(Number($('time').max) > 100, `${s}: replay length`);
}
$('scenario').value = 'jamming';
$('scenario').events.change();
assert.ok(
  $('events').children.some(e => text(e).includes('lost link hold')),
  'the jammed picket falls back on its own'
);
$('play').events.click();
let t = 0;
frame(t);
for (let k = 0; k < 5; k++) {
  t += 100;
  frame(t);
}
assert.equal($('time').value, 5, 'replay advances one second per 100 ms');
// The PX4 flights: one per scenario, the jamming flight shown first; checks, decisions, 3D data and videos.
assert.equal($('sitlScenario').children.length, 8, 'one option per PX4 flight');
assert.equal($('sitlScenario').value, 'guardian_jamming');
const show = name => {
  loaded = null;
  $('sitlScenario').value = name;
  $('sitlScenario').events.change({ target: { value: name } });
  return FLIGHTS[name];
};
const MODEL = { drone: 'intruder', fast: 'fast object', bird: 'bird' };
let checks = 0;
for (const name of Object.keys(FLIGHTS)) {
  $('sitlPlay').events.click();
  frame((t += 100)); // Playing when the flight changes must not carry over.
  const G = show(name);
  checks += Object.keys(G.checks).length;
  assert.equal($('sitlTime').value, 0, `${name}: the replay restarts`);
  assert.equal($('sitlPlay').textContent, 'Play');
  assert.equal($('checks').children.length, Object.keys(G.checks).length, `${name}: one row per check`);
  assert.ok(!text($('checks')).includes('FAIL'), `${name}: every published check passes on the page`);
  assert.equal($('sitlKpis').children.length, 4, `${name}: four tiles`);
  const threats = Object.entries(G.threats);
  assert.ok(loaded, `${name}: the 3D view was loaded for this flight`);
  for (const [model, t] of threats) {
    assert.ok(t.trace.length > 0, `${name}: ${model} has a truth trace`);
    const v = loaded.vehicles.find(x => x.name === model);
    assert.ok(v && v.model === t.kind, `${name}: ${model} drawn as a ${t.kind}`);
    assert.equal(v.hostile, t.kind !== 'bird', `${name}: only intruders and fast objects are drawn as hostile`);
    assert.ok(v.window && v.samples.every(q => q.u >= 0), `${name}: ${model} shown only while it flies`);
    assert.ok(text($('sitlLegend')).includes(MODEL[t.kind]), `${name}: legend names the ${MODEL[t.kind]}`);
  }
  assert.equal(loaded.zones.length, G.layout.jammer ? 1 : 0, `${name}: jamming zone only when jammed`);
  for (const [id, cam] of [
    ['viewOverview', 'overview'],
    ['viewClose', 'close']
  ])
    assert.equal($(id).disabled, !(G.videos && G.videos[cam]), `${name}: the ${cam} button follows its own recording`);
  assert.equal($('view3d').attributes['aria-pressed'], 'true', `${name}: falls back to the 3D view`);
  const events = $('sitlEvents').children.map(text),
    all = events.join('\n');
  assert.ok(!all.includes('undefined') && !all.includes('NaN'), `${name}: no undefined values in the log`);
  if (!G.layout.jammer)
    assert.ok(
      !all.includes('jamming') && !text($('sitlLegend')).includes('jamming'),
      `${name}: no jamming without a jammer`
    );
  if (G.expect.recovery === 'center')
    assert.ok(
      events.some(e => e.includes('decides recover')) && events.some(e => e.includes("accepts the center's recovery")),
      `${name}: the center decides recovery and the station accepts it`
    );
  else
    assert.ok(
      events.some(e => e.includes('unreachable')) && events.some(e => e.includes('no answer from the center')),
      `${name}: the center is unreachable and the station recovers under delegation`
    );
}
let G = show('guardian_jamming');
const sitl = $('sitlEvents').children.map(text);
assert.ok(G.videos && G.videos.overview && G.videos.close, 'the jamming flight is filmed from both cameras');
assert.ok(
  sitl.some(e => e.includes('keep clear') && e.includes('onboard')),
  'the jammed guardian keeps clear on its own'
);
assert.ok(
  sitl.some(e => e.includes('not delivered')),
  'orders to the jammed guardian are marked undelivered'
);
const links = G.jamming.filter(j => j.kind === 'link');
assert.ok(links.some(l => l.jammed) && links.some(l => !l.jammed), 'the link is lost and restored');
$('viewClose').events.click();
assert.ok($('flightVideo').src.endsWith(`guardian_jamming/${G.videos.close}`));
$('viewOverview').events.click();
assert.ok($('flightVideo').src.endsWith(`guardian_jamming/${G.videos.overview}`));
for (const name of Object.values(G.videos))
  assert.ok(fs.existsSync(path.join(root, 'artifacts/sitl-sample/guardian_jamming', name)), `${name} is published`);
G = show('guardian_spoofing');
assert.ok(
  $('sitlEvents')
    .children.map(text)
    .some(e => e.includes('fails the cross-check')),
  'spoofing: the station flags the drag-off'
);
assert.ok(text($('sitlKpis')).includes('WORST DRIFT'), 'spoofing: drift tile');
assert.ok($('videoPanel').hidden && $('viewOverview').disabled, 'a flight without video falls back to the 3D view');
console.log(
  `Guardian page checks passed: ${scenarios.length} scenarios, heatmaps, replay controls, 8 PX4 flights with ${checks} checks, decisions, 3D data and videos; layout and pixels untested.`
);
