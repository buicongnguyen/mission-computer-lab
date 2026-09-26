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
        require(set(re.findall(r"\$\('([^']+)'\)",app)) <= ids, f'{script}: JavaScript references missing element IDs')
    print(f'Static checks passed for {count} HTML files and dashboard element IDs')

if __name__=='__main__': check(Path(sys.argv[1]) if len(sys.argv)>1 else ROOT)
