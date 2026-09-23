/* Local Mermaid bundle: no CDN, network fetch or executable diagram links. */
(async () => {
  const blocks = [...document.querySelectorAll('pre.mermaid')];
  if (!blocks.length) return;
  try {
    if (!globalThis.mermaid) throw new Error('Local Mermaid renderer did not load');
    mermaid.initialize({ startOnLoad: false, securityLevel: 'strict', theme: 'neutral',
      flowchart: { htmlLabels: false, useMaxWidth: true }, sequence: { useMaxWidth: true } });
    // Isolate failures: one invalid diagram must not hide the others or their sources.
    for (const block of blocks) {
      const source = block.textContent;
      try { await mermaid.run({ nodes: [block] }); }
      catch (error) {
        block.textContent = source;
        block.classList.add('diagram-error');
        block.parentElement.querySelector('figcaption').textContent = 'Diagram could not render; readable Mermaid source is shown.';
        console.error(error);
      }
    }
  } catch (error) { console.error(error); }
})();
