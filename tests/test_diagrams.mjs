// Parse diagram syntax and check offline packaging without a browser/layout claim.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { JSDOM } from 'jsdom';
// Mermaid sanitizes labels during parsing. Supply an inert DOM; no URLs are loaded.
const dom = new JSDOM('');
globalThis.window = dom.window;
globalThis.document = dom.window.document;
const { default: mermaid } = await import('mermaid');
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
mermaid.initialize({ startOnLoad: false, securityLevel: 'strict', flowchart: { htmlLabels: false } });
let count = 0;
const kinds = new Set();
for (const name of fs.readdirSync(path.join(root, 'docs')).filter(n => n.endsWith('.md'))) {
  const source = fs.readFileSync(path.join(root, 'docs', name), 'utf8');
  const diagrams = [...source.matchAll(/```mermaid\r?\n([\s\S]*?)```/g)];
  if (!diagrams.length) continue;
  const html = fs.readFileSync(path.join(root, 'docs', name.replace(/\.md$/, '.html')), 'utf8');
  assert.equal((html.match(/class="mermaid"/g) || []).length, diagrams.length, name);
  assert(html.includes('src="assets/mermaid-11.17.2.min.js"'), name);
  assert(html.includes('src="assets/diagrams.js"'), name);
  for (const [, text] of diagrams) {
    await mermaid.parse(text);
    kinds.add(text.trim().split(/\s/)[0]);
    count++;
  }
}
// The README's diagram renders on GitHub rather than in the docs HTML; its syntax is checked the same way.
const readme = [...fs.readFileSync(path.join(root, 'README.md'), 'utf8').matchAll(/```mermaid\r?\n([\s\S]*?)```/g)];
assert(readme.length, 'README.md has its architecture diagram');
for (const [, text] of readme) {
  await mermaid.parse(text);
  count++;
}
for (const type of ['sequenceDiagram', 'flowchart', 'graph']) assert(kinds.has(type), `Missing requested ${type}`);
assert(
  fs
    .readFileSync(path.join(root, 'docs/assets/mermaid-11.17.2.min.js'))
    .equals(fs.readFileSync(path.join(root, 'node_modules/mermaid/dist/mermaid.min.js')))
);
assert(fs.existsSync(path.join(root, 'docs/assets/mermaid-LICENSE.txt')));
console.log(`Parsed ${count} Mermaid diagrams; offline bundle and requested diagram types verified.`);
dom.window.close();
