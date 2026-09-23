'use strict';
(() => {
  const $=id=>document.getElementById(id), report=window.MISSION_REPORT;
  if(!report){$('error').style.display='block';$('error').textContent='No replay data found. Run scripts/run_all.sh, then python tools/publish_sample.py.';return;}
  let active=report.results[0], index=0, playing=false, previous=0;
  const fmt=(v,n=2)=>Number(v).toFixed(n);
  $('overall').textContent=`${report.results.filter(r=>r.passed).length} / ${report.results.length} scenarios pass`;
  report.results.forEach((r,i)=>{const option=document.createElement('option');option.value=i;option.textContent=r.scenario.replaceAll('_',' ');$('scenario').append(option);});
  const row=(parent,left,right)=>{const div=document.createElement('div');div.className='check';const span=document.createElement('span');span.textContent=left;const b=document.createElement('b');b.textContent=right;div.append(span,b);parent.append(div);};
  report.boot.forEach(s=>row($('boot'),s.stage,s.status));
  report.security.forEach(s=>row($('security'),s.case.replaceAll('_',' '),s.passed?(s.accepted?'ACCEPTED ✓':'REJECTED ✓'):'FAILED'));
  function selectScenario(){active=report.results[Number($('scenario').value)];index=0;playing=false;$('play').textContent='Play';$('time').max=active.trace.length-1;$('time').value=0;
    const s=active.summary;$('result').textContent=active.passed?'PASS':'FAIL';$('terminal').textContent=`Terminal state: ${s.terminal_mode}`;
    $('latency').textContent=fmt(s.onnx_cpu_p95_ms,3)+' ms';$('ipc').textContent=fmt(s.supervisor_roundtrip_p95_ms,3)+' ms';$('rmse').textContent=fmt(s.localization_rmse_m,3)+' m';
    $('events').replaceChildren();active.events.forEach(e=>{const d=document.createElement('div');d.className='event';d.textContent=`${fmt(e.time,2)}s  ${e.mode} / ${e.reason}`;$('events').append(d);});draw();}
  function draw(){const f=active.trace[index];$('time').value=index;$('clock').textContent=fmt(f.time,1)+' s';$('mode').textContent=f.mode;$('reason').textContent=f.reason;$('gnss').textContent=fmt(f.gnss_age)+' s';$('errorValue').textContent=fmt(f.error,3)+' m';$('covariance').textContent=fmt(f.uncertainty,3)+' m²';$('battery').textContent=fmt(f.battery*100,0)+'%';$('camera').src=f.camera;
    const c=$('map'),ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.clearRect(0,0,w,h);
    const scale=Math.min(w/16,h/14),pt=p=>[70+(p[0]+2)*scale,h-42-(p[1]+2)*scale];
    ctx.strokeStyle='#224859';ctx.lineWidth=1;ctx.font='11px system-ui';ctx.fillStyle='#90aebb';
    for(let i=-2;i<=12;i+=2){const a=pt([i,-2]),b=pt([i,12]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();const d=pt([-2,i]),e=pt([12,i]);ctx.beginPath();ctx.moveTo(...d);ctx.lineTo(...e);ctx.stroke();ctx.fillText(String(i),a[0]-4,h-22);}
    ctx.fillText('EAST (m)',w-110,h-18);ctx.fillText('NORTH ↑',15,24);
    report.obstacles.forEach(([x,y,r])=>{ctx.beginPath();ctx.arc(...pt([x,y]),r*scale,0,Math.PI*2);ctx.fillStyle='#78534f';ctx.fill();ctx.strokeStyle='#c98478';ctx.stroke();});
    const line=(points,color,width,dash=[])=>{ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...pt(p)):ctx.moveTo(...pt(p)));ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);};
    line(active.path,'#7ca1b3',2,[7,5]);line(active.trace.slice(0,index+1).map(t=>t.position),'#55d5b4',3);
    ctx.fillStyle='#c3e8ee';f.lidar.forEach(p=>{const [x,y]=pt(p);ctx.fillRect(x-1,y-1,2,2);});
    const mark=(p,color,r)=>{ctx.beginPath();ctx.arc(...pt(p),r,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();};mark(f.target,'#fff',4);mark(f.position,'#55d5b4',7);mark(f.estimate,'#ffb95c',3);
    ctx.fillStyle='#e3f1f3';ctx.font='14px system-ui';ctx.fillText(`Altitude ${fmt(f.position[2],1)} m · ${f.mode}`,24,h-55);
    const overlay=$('bbox'),bctx=overlay.getContext('2d');overlay.width=Math.round(overlay.clientWidth*2);overlay.height=360;bctx.clearRect(0,0,overlay.width,360);
    if(f.bbox){const factor=Math.min(overlay.width/64,360/48),offset=(overlay.width-64*factor)/2;const [x1,y1,x2,y2]=f.bbox;bctx.strokeStyle='#55ffd1';bctx.lineWidth=3;bctx.strokeRect(offset+x1*factor,y1*factor,(x2-x1)*factor,(y2-y1)*factor);}
  }
  $('scenario').addEventListener('change',selectScenario);$('time').addEventListener('input',e=>{index=Number(e.target.value);draw();});$('play').addEventListener('click',()=>{if(index===active.trace.length-1)index=0;playing=!playing;$('play').textContent=playing?'Pause':'Play';});
  function tick(t){if(playing&&t-previous>70){previous=t;index=Math.min(index+1,active.trace.length-1);draw();if(index===active.trace.length-1){playing=false;$('play').textContent='Play';}}requestAnimationFrame(tick);}selectScenario();requestAnimationFrame(tick);window.addEventListener('resize',draw);
})();
