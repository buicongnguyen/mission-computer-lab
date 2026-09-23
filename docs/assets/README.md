# Offline diagram assets

`mermaid-11.17.2.min.js` is copied unchanged from the exact `mermaid@11.17.2` npm distribution by `tools/render_docs.mjs`. `mermaid-LICENSE.txt` accompanies it. `package-lock.json` pins the installation and registry integrity hashes; `tests/test_diagrams.mjs` compares this copy byte-for-byte with the installed package.

`diagrams.js` initializes the local renderer in strict mode and retains the readable Mermaid definition if a diagram fails. The HTML pages use ordinary local script tags rather than CDN imports, which permits offline viewing after cloning. Mermaid source remains in the Markdown guides.

To regenerate from the repository root, run `npm ci`, `npm run docs`, and `node tests/test_diagrams.mjs`. Syntax and packaging checks do not verify browser layout.
