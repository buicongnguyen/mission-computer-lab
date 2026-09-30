// Theme toggle behavior against a minimal DOM; this does not test colors or layout.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const source = fs.readFileSync(path.join(root, 'web/theme.js'), 'utf8');

function page({ systemDark = false, saved = null, storageThrows = false } = {}) {
  const store = new Map(saved ? [['mission-computer-lab-theme', saved]] : []);
  const events = [];
  const listeners = {};
  const nav = {
    children: [],
    append(child) {
      this.children.push(child);
    },
    querySelector: () => null
  };
  const button = () => nav.children.find(c => c.className === 'theme-toggle');
  const document = {
    readyState: 'complete',
    documentElement: { dataset: {} },
    querySelector: s => (s === 'nav' ? nav : s === '.theme-toggle' ? button() : null),
    createElement: () => ({
      attributes: {},
      handlers: {},
      setAttribute(k, v) {
        this.attributes[k] = v;
      },
      addEventListener(n, f) {
        this.handlers[n] = f;
      }
    }),
    addEventListener: (n, f) => {
      listeners[n] = f;
    },
    dispatchEvent: e => events.push(e.detail.theme)
  };
  const localStorage = {
    getItem: k => {
      if (storageThrows) throw new Error('blocked');
      return store.get(k) ?? null;
    },
    setItem: (k, v) => {
      if (storageThrows) throw new Error('blocked');
      store.set(k, v);
    }
  };
  const window = { matchMedia: () => ({ matches: systemDark, addEventListener() {} }) };
  window.window = window;
  const context = {
    document,
    localStorage,
    window,
    matchMedia: window.matchMedia,
    CustomEvent: class {
      constructor(type, init) {
        this.type = type;
        this.detail = init.detail;
      }
    }
  };
  vm.runInNewContext(source, context);
  return { root: document.documentElement, button: button(), store, events, window };
}

let p = page();
assert.equal(p.root.dataset.theme, 'light', 'follows a light system theme');
assert.equal(p.button.textContent, '☾ Dark');
p.button.handlers.click();
assert.equal(p.root.dataset.theme, 'dark', 'toggle switches to dark');
assert.equal(p.store.get('mission-computer-lab-theme'), 'dark', 'choice is remembered');
assert.equal(p.button.attributes['aria-label'], 'Switch to light theme');
assert.deepEqual(p.events, ['light', 'dark'], 'diagram renderer is told about each theme');

p = page({ systemDark: true });
assert.equal(p.root.dataset.theme, 'dark', 'follows a dark system theme');
p = page({ systemDark: true, saved: 'light' });
assert.equal(p.root.dataset.theme, 'light', 'a saved choice overrides the system');
p = page({ saved: 'purple' });
assert.equal(p.root.dataset.theme, 'light', 'invalid stored values are ignored');
p = page({ storageThrows: true });
p.button.handlers.click();
assert.equal(p.root.dataset.theme, 'dark', 'toggle still works when storage is blocked');
assert.equal(p.window.missionTheme.current(), 'dark');
console.log('Theme checks passed: system default, toggle, persistence, invalid/blocked storage; colors untested.');
