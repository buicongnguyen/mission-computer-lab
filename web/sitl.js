'use strict';
(() => {
  const $=id=>document.getElementById(id), report=window.SITL_REPORT;
  if(!report){$('error').textContent='Reference evidence missing. Run the SITL matrix and tools/publish_sitl.py.';return;}
  const names={4:'AUTO LOITER',14:'OFFBOARD',18:'AUTO LAND'};
  const format=v=>Number(v).toFixed(2);
  let active,index=0,playing=false,last=0;
  $('overall').textContent=`${report.results.filter(r=>r.passed).length} / ${report.results.length} pass`;
  $('provenance').textContent=`Recorded ${report.generated_utc} · ${report.environment.platform}`;
  report.results.forEach((r,i)=>{const o=document.createElement('option');o.value=i;o.textContent=r.scenario.replaceAll('_',' ');$('scenario').append(o);});
  function select(){active=report.results[Number($('scenario').value)];index=0;playing=false;$('play').textContent='Play';$('time').max=active.trace.length-1;
    $('altitude').textContent=format(active.max_altitude_m)+' m';$('clearance').textContent=format(active.minimum_estimated_clearance_m)+' m';$('images').textContent=active.message_counts.image;
    $('checks').replaceChildren();Object.entries(active.checks).forEach(([name,ok])=>{const row=document.createElement('div');row.className='check';const label=document.createElement('span');label.textContent=name.replaceAll('_',' ');const state=document.createElement('b');state.textContent=ok?'PASS':'FAIL';row.append(label,state);$('checks').append(row);});
    const events=[...active.statuses.map(s=>({at:s.wall_time,text:`PX4 ${names[s.nav_state]||s.nav_state} · ${s.arming_state===2?'ARMED':'DISARMED'}${s.failsafe?' · FAILSAFE':''}`})),...active.mission_transitions.map(s=>({at:s.wall_time,text:`C++ ${s.mode} / ${s.reason}`}))];
    if(active.injected_at)events.push({at:active.injected_at,text:'FAULT INJECTED: '+active.scenario.replaceAll('_',' ')});
    events.sort((a,b)=>a.at-b.at);$('events').replaceChildren();events.forEach(e=>{const d=document.createElement('div');d.className='event';d.textContent=`${format(e.at-active.replay_start_wall_time)} s  ${e.text}`;$('events').append(d);});draw();
  }
  function draw(){const frame=active.trace[index],t=frame.wall_time-active.replay_start_wall_time;$('time').value=index;$('clock').textContent=format(t)+' s';
    const state=active.statuses.filter(s=>s.wall_time<=frame.wall_time).at(-1);$('state').textContent=state?`${names[state.nav_state]||state.nav_state} / ${state.arming_state===2?'ARMED':'DISARMED'}`:'Awaiting status';
    const c=$('map'),ctx=c.getContext('2d'),w=c.width,h=c.height,scale=36,pt=(x,y)=>[105+x*scale,h-90-y*scale];ctx.clearRect(0,0,w,h);ctx.font='12px system-ui';ctx.lineWidth=1;
    for(let i=-1;i<=13;i++){ctx.strokeStyle='#244859';ctx.beginPath();ctx.moveTo(...pt(i,-1));ctx.lineTo(...pt(i,13));ctx.moveTo(...pt(-1,i));ctx.lineTo(...pt(13,i));ctx.stroke();if(i%2===0){ctx.fillStyle='#9fb9c5';ctx.fillText(String(i),pt(i,-1)[0],h-35);}}
    ctx.fillStyle='#c5dbe3';ctx.fillText('NORTH ↑',20,25);ctx.fillText('EAST (m)',w-105,h-20);
    report.obstacles.forEach(([x,y,r])=>{ctx.beginPath();ctx.arc(...pt(x,y),r*scale,0,Math.PI*2);ctx.fillStyle='#78534f';ctx.fill();ctx.strokeStyle='#c98478';ctx.stroke();});
    if(active.plan){ctx.beginPath();active.plan.path.forEach(([x,y],i)=>{const p=pt(x+active.plan.origin[0],y+active.plan.origin[1]);i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.setLineDash([7,5]);ctx.strokeStyle='#7ca1b3';ctx.lineWidth=2;ctx.stroke();ctx.setLineDash([]);}
    ctx.beginPath();active.trace.slice(0,index+1).forEach((f,i)=>{const p=pt(f.ned[1],f.ned[0]);i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.strokeStyle='#55d5b4';ctx.lineWidth=3;ctx.stroke();
    ctx.beginPath();ctx.arc(...pt(frame.ned[1],frame.ned[0]),7,0,Math.PI*2);ctx.fillStyle='#55d5b4';ctx.fill();ctx.fillStyle='#e7f4f5';ctx.font='16px system-ui';ctx.fillText(`Estimated altitude ${format(-frame.ned[2])} m`,25,h-60);
    if(active.injected_at&&frame.wall_time>=active.injected_at){ctx.fillStyle='#ffbe6b';ctx.fillText('FAULT INJECTED',w-195,30);}
  }
  $('scenario').addEventListener('change',select);$('time').addEventListener('input',e=>{index=Number(e.target.value);draw();});
  $('play').addEventListener('click',()=>{if(index===active.trace.length-1)index=0;playing=!playing;$('play').textContent=playing?'Pause':'Play';});
  function tick(now){if(playing&&now-last>=100){last=now;index=Math.min(index+1,active.trace.length-1);draw();if(index===active.trace.length-1){playing=false;$('play').textContent='Play';}}requestAnimationFrame(tick);}
  select();requestAnimationFrame(tick);
})();
