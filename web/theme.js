/* Light/dark theme: applied before first paint, remembered per browser, toggled from the page nav. */
(() => {
  const KEY = 'mission-computer-lab-theme';
  const root = document.documentElement;
  const system = () => (window.matchMedia && matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
  const stored = () => {
    try {
      const v = localStorage.getItem(KEY);
      return v === 'dark' || v === 'light' ? v : null;
    } catch {
      return null;
    }
  };
  const current = () => root.dataset.theme || system();
  const apply = (theme, remember) => {
    root.dataset.theme = theme;
    if (remember) {
      try {
        localStorage.setItem(KEY, theme);
      } catch {
        /* private mode: still switches for this page */
      }
    }
    const button = document.querySelector('.theme-toggle');
    if (button) {
      button.textContent = theme === 'dark' ? '☀ Light' : '☾ Dark';
      button.setAttribute('aria-label', `Switch to ${theme === 'dark' ? 'light' : 'dark'} theme`);
    }
    document.dispatchEvent(new CustomEvent('themechange', { detail: { theme } }));
  };
  const saved = stored();
  if (saved) root.dataset.theme = saved;
  // Without a saved choice, follow the system live.
  if (window.matchMedia) {
    matchMedia('(prefers-color-scheme: dark)').addEventListener?.('change', () => {
      if (!stored()) apply(system(), false);
    });
  }
  const addToggle = () => {
    const nav = document.querySelector('nav');
    if (!nav || nav.querySelector('.theme-toggle')) return;
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'theme-toggle';
    button.addEventListener('click', () => apply(current() === 'dark' ? 'light' : 'dark', true));
    nav.append(button);
    apply(current(), false);
  };
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', addToggle);
  else addToggle();
  window.missionTheme = { current };
})();
