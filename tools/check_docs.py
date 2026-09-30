"""Static local-link and content checks, not a browser layout test.

Pass a folder to check a built site (for example the Pages `_site`); links must resolve inside it.
"""
from html.parser import HTMLParser
from pathlib import Path
import re
import sys
from urllib.parse import unquote,urlsplit

ROOT=Path(__file__).resolve().parents[1]
class Links(HTMLParser):
    def __init__(self): super().__init__(); self.links=[]; self.text=[]; self.ids=set()
    def handle_starttag(self,tag,attrs):
        for k,v in attrs:
            if k in ('href','src') and v: self.links.append(v)
            if k=='id':
                require(v not in self.ids,'Duplicate HTML id: '+v)
                self.ids.add(v)
    def handle_data(self,data): self.text.append(data)

def require(condition,message):
    if not condition:raise ValueError(message)

# A home directory or a Windows user profile in published text names the author's machine; the guides use
# $HOME, ~ and $SITL_WORKSPACE instead. Vendored bundles and binary media are not scanned.
LOCAL_PATH=re.compile(r'/home/[A-Za-z0-9_.-]+/|/Users/[A-Za-z0-9_.-]+/|\b[A-Za-z]:[\\/]Users[\\/]')
PUBLISHED=('README.md','docs','web','artifacts')
SKIP=('docs/assets','web/vendor')
TEXT=('.md','.html','.js','.json','.jsonl','.csv','.txt','.log','.sdf','.py','.mjs','.yml','.css')

def check_private(root):
    """No local machine path in anything that is published (the repository's shared folders or a built site)."""
    found=[]
    for top in PUBLISHED:
        base=root/top
        for p in ([base] if base.is_file() else base.rglob('*') if base.is_dir() else []):
            rel=p.relative_to(root).as_posix()
            if not p.is_file() or p.suffix not in TEXT or rel.startswith(SKIP):continue
            for number,line in enumerate(p.read_text(encoding='utf-8',errors='replace').splitlines(),1):
                if LOCAL_PATH.search(line):found.append(f'{rel}:{number}')
    require(not found,'Local machine paths in published files: '+', '.join(found[:10]))

def check(root=ROOT):
    root=root.resolve();count=0
    for p in [*root.glob('docs/*.html'),*root.glob('web/*.html')]:
        parser=Links(); parser.feed(p.read_text(encoding='utf-8'))
        for href in parser.links:
            parts=urlsplit(href)
            if parts.scheme or parts.netloc: continue
            resolved=(p.parent/unquote(parts.path)).resolve() if parts.path else p
            require(resolved.is_relative_to(root) and resolved.is_file(), f'{p.name}: missing local link {href}')
            if parts.fragment and resolved.suffix=='.html':
                target=Links();target.feed(resolved.read_text(encoding='utf-8'))
                require(unquote(parts.fragment) in target.ids,f'{p.name}: missing anchor {href}')
        require('<meta name="viewport"' in p.read_text(encoding='utf-8'),f'{p.name}: missing viewport')
        count+=1
    require(count>0,f'No HTML pages found under {root}')
    for script,page in [('app.js','index.html'),('sitl.js','sitl.html'),('fleet.js','fleet.html'),('guardian.js','guardian.html')]:
        app=(root/'web'/script).read_text(encoding='utf-8')
        html=(root/'web'/page).read_text(encoding='utf-8')
        ids=set(re.findall(r'id="([^"]+)"',html))
        # Either quote style, so reformatting the scripts cannot turn this check into one that matches nothing.
        used={m[1] for m in re.findall(r"""\$\(\s*(['"])([^'"]+)\1\s*\)""",app)}
        require(used,f'{script}: no element lookups found; the check would pass vacuously')
        require(used <= ids, f'{script}: JavaScript references missing element IDs: {sorted(used-ids)}')
    check_private(root)
    print(f'Static checks passed for {count} HTML files, dashboard element IDs and published paths')

if __name__=='__main__': check(Path(sys.argv[1]) if len(sys.argv)>1 else ROOT)
