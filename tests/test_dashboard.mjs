// Exercise dashboard state/events against a minimal DOM; this does not test layout.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
class Element {
  constructor(){this.value='0';this.children=[];this.events={};this.style={};this.textContent='';this.width=800;this.height=592;this.clientWidth=400;}
  append(...children){this.children.push(...children);}
  replaceChildren(){this.children=[];}
  addEventListener(name,fn){this.events[name]=fn;}
  getContext(){return new Proxy({}, {get:()=>()=>{},set:()=>true});}
}
const elements=new Map();
const document={getElementById(id){if(!elements.has(id))elements.set(id,new Element());return elements.get(id);},createElement(){return new Element();}};
const report=JSON.parse(fs.readFileSync(path.join(root,'artifacts/sample/report.json'),'utf8'));
const context={document,window:{MISSION_REPORT:report,addEventListener(){}},requestAnimationFrame(){},console};
vm.runInNewContext(fs.readFileSync(path.join(root,'web/app.js'),'utf8'),context);
assert.equal(elements.get('scenario').children.length,8);
assert.equal(elements.get('security').children.length,6);
for(let i=0;i<8;i++){
  elements.get('scenario').value=String(i);elements.get('scenario').events.change();
  assert.equal(elements.get('result').textContent,'PASS');
  const frames=report.results[i].trace;
  elements.get('time').events.input({target:{value:String(frames.length-1)}});
  assert.equal(elements.get('mode').textContent,frames.at(-1).mode);
  assert.ok(elements.get('camera').src.startsWith('data:image/png;base64,'));
}
elements.get('play').events.click();assert.equal(elements.get('play').textContent,'Pause');
elements.get('play').events.click();assert.equal(elements.get('play').textContent,'Play');
console.log('Dashboard state checks passed: all 8 scenarios, timeline, images and play/pause; layout untested.');
