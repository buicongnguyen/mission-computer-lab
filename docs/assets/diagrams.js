/* Local Mermaid bundle: no CDN, network fetch or executable diagram links. */
(() => {
  const blocks = [...document.querySelectorAll('pre.mermaid')];
  if (!blocks.length) return;
  if (!globalThis.mermaid) { console.error(new Error('Local Mermaid renderer did not load')); return; }
  const sources = blocks.map(block => block.textContent);
  const captions = blocks.map(block => block.parentElement.querySelector('figcaption'));
  const captionText = captions.map(caption => caption && caption.textContent);
  const wanted = () => (window.missionTheme && window.missionTheme.current() === 'dark' ? 'dark' : 'neutral');
  let rendered = null;
  let queue = Promise.resolve();
  const render = async () => {
    const theme = wanted();
    if (theme === rendered) return;
    rendered = theme;
    mermaid.initialize({ startOnLoad: false, securityLevel: 'strict', theme,
      flowchart: { htmlLabels: false, useMaxWidth: true }, sequence: { useMaxWidth: true } });
    // Isolate failures: one invalid diagram must not hide the others or their sources.
    for (const [i, block] of blocks.entries()) {
      block.removeAttribute('data-processed');
      block.classList.remove('diagram-error');
      block.textContent = sources[i];
      if (captions[i]) captions[i].textContent = captionText[i];
      try { await mermaid.run({ nodes: [block] }); }
      catch (error) {
        block.textContent = sources[i];
        block.classList.add('diagram-error');
        if (captions[i]) captions[i].textContent = 'Diagram could not render; readable Mermaid source is shown.';
        console.error(error);
      }
    }
  };
  const schedule = () => { queue = queue.then(render).catch(error => console.error(error)); };
  schedule();
  document.addEventListener('themechange', schedule);
})();
