import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { marked } = process.env.MARKED_MODULE
  ? await import(pathToFileURL(process.env.MARKED_MODULE).href)
  : await import('marked');
const css = fs.readFileSync(path.join(root, 'web/style.css'), 'utf8');
const vendor = path.join(root, 'docs/assets');
fs.mkdirSync(vendor, { recursive: true });
for (const [from, to] of [['dist/mermaid.min.js', 'mermaid-11.17.2.min.js'], ['LICENSE', 'mermaid-LICENSE.txt']]) {
  fs.copyFileSync(path.join(root, 'node_modules/mermaid', from), path.join(vendor, to));
}
const escape = text => text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
marked.use({ renderer: { code(token) {
  if (token.lang !== 'mermaid') return false;
  const source = escape(token.text);
  return `<figure class="diagram"><pre class="mermaid">${source}</pre><figcaption>Pipeline diagram — source is available below.</figcaption><details><summary>Mermaid source</summary><pre><code>${source}</code></pre></details></figure>`;
} } });
const docs = fs.readdirSync(path.join(root, 'docs')).filter(x => x.endsWith('.md'));
for (const name of docs) {
  const md = fs.readFileSync(path.join(root, 'docs', name), 'utf8');
  const title = (md.match(/^# (.+)$/m)?.[1] || name).replace(/[<>&"]/g, '');
  const html = marked.parse(md).replace(/href="([^"#:]+)\.md(#[^"]*)?"/g, 'href="$1.html$2"');
  const diagrams = md.includes('```mermaid') ? '<script src="assets/mermaid-11.17.2.min.js" defer></script><script src="assets/diagrams.js" defer></script>' : '';
  fs.writeFileSync(path.join(root, 'docs', name.replace('.md', '.html')),
    `<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>${title}</title><style>${css}</style>${diagrams}</head><body><nav><a href="../web/index.html">MISSION COMPUTER LAB</a><a href="architecture.html">Architecture</a><a href="wsl-guide.html">Run in WSL</a><a href="domain-notes.html">Domain notes</a><a href="review-report.html">Review and fixes</a></nav><main class="document">${html}</main><footer>Mission Computer Lab · software simulation portfolio · 2026</footer></body></html>`);
  console.log('Rendered', name.replace('.md', '.html'));
}
