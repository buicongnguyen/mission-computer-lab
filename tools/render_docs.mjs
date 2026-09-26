import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { marked } = process.env.MARKED_MODULE
  ? await import(pathToFileURL(process.env.MARKED_MODULE).href)
  : await import('marked');
const css = fs.readFileSync(path.join(root, 'web/style.css'), 'utf8');
const repository = 'https://github.com/buicongnguyen/mission-computer-lab';
const vendor = path.join(root, 'docs/assets');
fs.mkdirSync(vendor, { recursive: true });
for (const [from, to] of [['dist/mermaid.min.js', 'mermaid-11.17.2.min.js'], ['LICENSE', 'mermaid-LICENSE.txt']]) {
  fs.copyFileSync(path.join(root, 'node_modules/mermaid', from), path.join(vendor, to));
}
// three.js 0.147 is the last release with a non-module build and OrbitControls, so the 3D replay
// also works when a page is opened straight from disk (module scripts are blocked on file://).
const webVendor = path.join(root, 'web/vendor');
fs.mkdirSync(webVendor, { recursive: true });
for (const [from, to] of [['build/three.min.js', 'three-0.147.0.min.js'],
  ['examples/js/controls/OrbitControls.js', 'OrbitControls-0.147.0.js'], ['LICENSE', 'three-LICENSE.txt']]) {
  fs.copyFileSync(path.join(root, 'node_modules/three', from), path.join(webVendor, to));
}
const escape = text => text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
const captions = { flowchart: 'Data-flow diagram', graph: 'Dependency graph', stateDiagram: 'State diagram',
  'stateDiagram-v2': 'State diagram', sequenceDiagram: 'Sequence diagram' };
marked.use({ renderer: { code(token) {
  if (token.lang !== 'mermaid') return false;
  const source = escape(token.text);
  const caption = captions[token.text.trim().split(/\s/)[0]] || 'Diagram';
  return `<figure class="diagram"><pre class="mermaid">${source}</pre><figcaption>${caption} — source is available below.</figcaption><details><summary>Mermaid source</summary><pre><code>${source}</code></pre></details></figure>`;
} } });
const docs = fs.readdirSync(path.join(root, 'docs')).filter(x => x.endsWith('.md'));
for (const name of docs) {
  const md = fs.readFileSync(path.join(root, 'docs', name), 'utf8');
  const title = escape(md.match(/^# (.+)$/m)?.[1] || name);
  const html = marked.parse(md).replace(/href="([^"#:]+)\.md(#[^"]*)?"/g, 'href="$1.html$2"');
  const diagrams = md.includes('```mermaid') ? '<script src="assets/mermaid-11.17.2.min.js" defer></script><script src="assets/diagrams.js" defer></script>' : '';
  fs.writeFileSync(path.join(root, 'docs', name.replace('.md', '.html')),
    `<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>${title}</title><style>${css}</style><script src="../web/theme.js"></script>${diagrams}</head><body><nav><a href="../web/index.html">MISSION COMPUTER LAB</a><a href="../web/sitl.html">Flights</a><a href="../web/fleet.html">Fleet ops</a><a href="../web/guardian.html">Guardians</a><a href="architecture.html">Architecture</a><a href="wsl-guide.html">Run in WSL</a><a href="domain-notes.html">Domain notes</a><a href="review-report.html">Review</a><a href="references.html">References</a><a href="${repository}">GitHub</a></nav><main class="document">${html}</main><footer>Mission Computer Lab · software simulation portfolio · <a href="${repository}">source on GitHub</a> · 2026</footer></body></html>`);
  console.log('Rendered', name.replace('.md', '.html'));
}
