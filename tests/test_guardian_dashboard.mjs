// State checks for the guardian page only. This test deliberately makes no layout, canvas-pixel or WebGL claim.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
class Element{
  constructor(tag){this.tag=tag;this.value='';this.children=[];this.events={};this.textContent='';this.attributes={};this.hidden=false;
    this.dataset={};this.style={setProperty(k,v){this[k]=v;}};this.width=800;this.height=610;this.clientWidth=640;}
  append(...items){this.children.push(...items);}replaceChildren(...items){this.children=[...items];}
  addEventListener(name,fn){this.events[name]=fn;}setAttribute(k,v){this.attributes[k]=v;}
  removeAttribute(k){delete this.attributes[k];if(k==='src')delete this.src;}pause(){this.paused=true;}
  getContext(){return new Proxy({},{get:()=>()=>{},set:()=>true});}
}
const elements=new Map(),document={getElementById(id){if(!elements.has(id))elements.set(id,new Element('div'));return elements.get(id);},
  createElement(tag){return new Element(tag);}};
const $=id=>document.getElementById(id),text=e=>e.textContent+e.children.map(text).join('');
let loaded=null;
const window={devicePixelRatio:1,addEventListener(){},Replay3D:class{load(d){loaded=d;}setTime(){}}};
for(const file of ['artifacts/guardian/report-data.js','artifacts/sitl-sample/report-data.js'])
  vm.runInNewContext(fs.readFileSync(path.join(root,file),'utf8'),{window});
const R=window.GUARDIAN_REPORT,G=window.SITL_REPORT.guardian;
assert.ok(R&&G,'both the simulator report and the PX4 guardian flight are published');
let frame=null;
vm.runInNewContext(fs.readFileSync(path.join(root,'web/guardian.js'),'utf8'),{document,window,requestAnimationFrame(fn){frame=fn;}});
assert.equal(text($('error')),'','no evidence error');
// Headline tiles and the comparison tables come straight from the report.
assert.equal($('kpis').children.length,4);
assert.ok(text($('kpis')).includes(`${R.summary.intruder.hybrid.warning_s.median.toFixed(0)} s`),'hybrid warning tile');
const scenarios=Object.keys(R.summary);
for(const id of ['heatSafe','heatWarn']){
  const rows=$(id).children;assert.equal(rows.length,scenarios.length+1,`${id}: a header and one row per scenario`);
  rows.slice(1).forEach(r=>assert.equal(r.children.length,5));
}
const safeCell=$('heatSafe').children[1+scenarios.indexOf('jamming')].children[4];
assert.equal(safeCell.textContent,Math.round(R.summary.jamming.hybrid.guardians_safe_rate*100)+'%');
assert.ok(safeCell.title.includes('Hybrid'),'cells carry an accessible description');
assert.equal($('layouts').children.length,1+Object.keys(R.layouts.summary).length);
assert.equal($('authority').children.length,4);
// Replay: every scenario loads, only designs with a recorded replay are selectable, and playback advances.
assert.equal($('scenario').children.length,scenarios.length);
for(const s of scenarios){
  $('scenario').value=s;$('scenario').events.change();
  const buttons=$('archButtons').children,enabled=buttons.filter(b=>!b.disabled).map(b=>b.dataset.arch);
  assert.deepEqual(enabled.sort(),Object.keys(R.traces[s]).sort(),`${s}: selectable designs match the recorded replays`);
  assert.ok($('events').children.length>0||s==='spoofing',`${s}: decision log`);
  assert.ok(Number($('time').max)>100,`${s}: replay length`);
}
$('scenario').value='jamming';$('scenario').events.change();
assert.ok($('events').children.some(e=>text(e).includes('lost link hold')),'the jammed picket falls back on its own');
$('play').events.click();let t=0;frame(t);for(let k=0;k<5;k++){t+=100;frame(t);}
assert.equal($('time').value,5,'replay advances one second per 100 ms');
// The PX4 flight: checks, decisions, 3D data and videos.
assert.equal($('checks').children.length,Object.keys(G.checks).length);
assert.ok(Object.values(G.checks).every(Boolean),'every published guardian check passes');
const sitl=$('sitlEvents').children.map(text);
assert.ok(sitl.some(e=>e.includes('keep clear')&&e.includes('onboard')),'the jammed guardian keeps clear on its own');
assert.ok(sitl.some(e=>e.includes('decides recover')),'the center decides recovery');
const links=G.jamming.filter(j=>j.kind==='link');
assert.ok(links.some(l=>l.jammed)&&links.some(l=>!l.jammed),'the link is lost and restored');
assert.ok(loaded&&loaded.vehicles.some(v=>v.hostile&&v.name==='intruder'),'the 3D view draws the intruder');
assert.equal(loaded.zones.length,1,'and the jamming zone');
$('viewClose').events.click();assert.ok($('flightVideo').src.endsWith(`guardian_intruder/${G.videos.close}`));
$('viewOverview').events.click();assert.ok($('flightVideo').src.endsWith(`guardian_intruder/${G.videos.overview}`));
for(const name of Object.values(G.videos))assert.ok(fs.existsSync(path.join(root,'artifacts/sitl-sample/guardian_intruder',name)),`${name} is published`);
console.log(`Guardian page checks passed: ${scenarios.length} scenarios, heatmaps, replay controls, ${Object.keys(G.checks).length} PX4 checks, decisions, 3D data and videos; layout and pixels untested.`);
