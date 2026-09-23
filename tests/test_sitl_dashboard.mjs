// State/event checks only. This test deliberately makes no layout claim.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
class Element{
  constructor(){this.value='0';this.children=[];this.events={};this.style={};this.textContent='';this.width=800;this.height=610;}
  append(...items){this.children.push(...items);}replaceChildren(){this.children=[];}
  addEventListener(name,fn){this.events[name]=fn;}
  getContext(){return new Proxy({}, {get:()=>()=>{},set:()=>true});}
}
const elements=new Map(),document={getElementById(id){if(!elements.has(id))elements.set(id,new Element());return elements.get(id);},createElement(){return new Element();}};
const report=JSON.parse(fs.readFileSync(path.join(root,'artifacts/sitl-sample/report.json'),'utf8'));
vm.runInNewContext(fs.readFileSync(path.join(root,'web/sitl.js'),'utf8'),{document,window:{SITL_REPORT:report},requestAnimationFrame(){}});
assert.equal(elements.get('scenario').children.length,4);
assert.equal(elements.get('overall').textContent,'4 / 4 pass');
for(let i=0;i<4;i++){
  elements.get('scenario').value=String(i);elements.get('scenario').events.change();
  assert.equal(elements.get('checks').children.length,Object.keys(report.results[i].checks).length);
  assert.ok(elements.get('events').children.length>=4);
  elements.get('time').events.input({target:{value:String(report.results[i].trace.length-1)}});
  assert.ok(elements.get('state').textContent.includes('DISARMED'));
}
elements.get('play').events.click();assert.equal(elements.get('play').textContent,'Pause');
elements.get('play').events.click();assert.equal(elements.get('play').textContent,'Play');
console.log('SITL replay checks passed: four scenarios, evidence checks, events, final disarm, play/pause; layout untested.');
