'use strict';
(() => {
  const $=id=>document.getElementById(id), report=window.SITL_REPORT;
  if(!report){$('error').textContent='Reference evidence missing. Run the SITL matrix and tools/publish_sitl.py.';return;}
  // PX4 v1.16 VehicleStatus.NAVIGATION_STATE_* values.
  const names={0:'MANUAL',1:'ALTITUDE',2:'POSITION',3:'AUTO MISSION',4:'AUTO LOITER',5:'AUTO RTL',6:'POSITION SLOW',10:'ACRO',
    12:'DESCEND',13:'TERMINATION',14:'OFFBOARD',15:'STABILIZED',17:'AUTO TAKEOFF',18:'AUTO LAND',19:'FOLLOW TARGET',
    20:'PRECISION LAND',21:'ORBIT',22:'VTOL TAKEOFF'};
  const navName=n=>names[n]||`NAV STATE ${n}`;
  const armed=s=>s.arming_state===2?'ARMED':'DISARMED';
  const format=v=>Number(v).toFixed(2);
  let active,index=0,playing=false,last=0;
  const since=t=>format(Math.max(0,t-active.replay_start_wall_time));
  $('overall').textContent=`${report.results.filter(r=>r.passed).length} / ${report.results.length} pass`;
  $('provenance').textContent=`Recorded ${report.generated_utc} · ${report.environment.platform}`;
  report.results.forEach((r,i)=>{const o=document.createElement('option');o.value=i;o.textContent=r.scenario.replaceAll('_',' ');$('scenario').append(o);});
  function select(){active=report.results[Number($('scenario').value)];index=0;playing=false;$('play').textContent='Play';$('time').max=active.trace.length-1;
    $('altitude').textContent=format(active.max_altitude_m)+' m';$('clearance').textContent=format(active.minimum_estimated_clearance_m)+' m';$('images').textContent=active.message_counts.image;
    $('checks').replaceChildren();Object.entries(active.checks).forEach(([name,ok])=>{const row=document.createElement('div');row.className='check';const label=document.createElement('span');label.textContent=name.replaceAll('_',' ');const state=document.createElement('b');state.textContent=ok?'PASS':'FAIL';row.append(label,state);$('checks').append(row);});
    const events=[...active.statuses.map(s=>({at:s.wall_time,text:`PX4 ${navName(s.nav_state)} · ${armed(s)}${s.failsafe?' · FAILSAFE':''}${s.preflight===false?' · PREFLIGHT CHECKS FAILING':''}`})),
      ...active.mission_transitions.map(s=>({at:s.wall_time,text:`C++ ${s.mode} / ${s.reason}`})),
      ...(active.replans||[]).map(r=>({at:r.wall_time,text:r.path.length?`Planner REPLAN · ${r.path.length} cells`:'Planner NO ROUTE'}))];
    if(active.injected_at)events.push({at:active.injected_at,text:'FAULT INJECTED: '+active.scenario.replaceAll('_',' ')});
    events.sort((a,b)=>a.at-b.at);$('events').replaceChildren();
    events.filter((e,i)=>!i||e.text!==events[i-1].text).forEach(e=>{const d=document.createElement('div');d.className='event';d.textContent=`${since(e.at)} s  ${e.text}`;$('events').append(d);});draw();
  }
  // The route in force at a frame: the first plan, then any later replan.
  const routeAt=frame=>(active.replans||[]).filter(r=>r.wall_time<=frame.wall_time&&r.path.length).map(r=>r.path).at(-1)||(active.plan&&active.plan.path);
  function draw(){const frame=active.trace[index];$('time').value=index;$('clock').textContent=since(frame.wall_time)+' s';
    const state=active.statuses.filter(s=>s.wall_time<=frame.wall_time).at(-1);$('state').textContent=state?`${navName(state.nav_state)} / ${armed(state)}${state.failsafe?' / FAILSAFE':''}`:'Awaiting status';
    // Size the bitmap to the displayed size so text stays legible on phones and sharp on high-DPI screens.
    const c=$('map'),dpr=window.devicePixelRatio||1,w=c.clientWidth||800,h=Math.round(w*610/800);
    if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){c.width=Math.round(w*dpr);c.height=Math.round(h*dpr);}
    const ctx=c.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
    const scale=Math.min((w-75)/14,(h-95)/14),pt=(x,y)=>[55+(x+1)*scale,h-55-(y+1)*scale];ctx.font='12px system-ui';ctx.lineWidth=1;
    for(let i=-1;i<=13;i++){ctx.strokeStyle='#244859';ctx.beginPath();ctx.moveTo(...pt(i,-1));ctx.lineTo(...pt(i,13));ctx.moveTo(...pt(-1,i));ctx.lineTo(...pt(13,i));ctx.stroke();if(i%2===0){ctx.fillStyle='#9fb9c5';ctx.fillText(String(i),pt(i,-1)[0]-3,h-35);}}
    ctx.fillStyle='#c5dbe3';ctx.fillText('NORTH ↑',20,25);ctx.fillText('EAST (m)',w-105,h-20);
    report.obstacles.forEach(([x,y,r])=>{ctx.beginPath();ctx.arc(...pt(x,y),r*scale,0,Math.PI*2);ctx.fillStyle='#78534f';ctx.fill();ctx.strokeStyle='#c98478';ctx.stroke();});
    const route=routeAt(frame);
    if(route){ctx.beginPath();route.forEach(([x,y],i)=>{const p=pt(x+active.plan.origin[0],y+active.plan.origin[1]);i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.setLineDash([7,5]);ctx.strokeStyle='#7ca1b3';ctx.lineWidth=2;ctx.stroke();ctx.setLineDash([]);}
    // Estimates PX4 marked invalid (after GPS loss) are drawn grey and dashed, not as real motion.
    const path=active.trace.slice(0,index+1);
    for(let i=1;i<path.length;i++){const a=path[i-1],b=path[i],ok=a.xy_valid!==false&&b.xy_valid!==false;
      ctx.beginPath();ctx.moveTo(...pt(a.ned[1],a.ned[0]));ctx.lineTo(...pt(b.ned[1],b.ned[0]));ctx.strokeStyle=ok?'#55d5b4':'#8a9ba3';ctx.lineWidth=ok?3:2;ctx.setLineDash(ok?[]:[4,4]);ctx.stroke();}
    ctx.setLineDash([]);
    const valid=frame.xy_valid!==false;ctx.beginPath();ctx.arc(...pt(frame.ned[1],frame.ned[0]),7,0,Math.PI*2);
    if(valid){ctx.fillStyle='#55d5b4';ctx.fill();}else{ctx.strokeStyle='#8a9ba3';ctx.lineWidth=2;ctx.stroke();}
    ctx.fillStyle='#e7f4f5';ctx.font='16px system-ui';ctx.fillText(`Estimated altitude ${format(-frame.ned[2])} m${valid?'':' · horizontal estimate invalid'}`,25,h-60);
    if(active.injected_at&&frame.wall_time>=active.injected_at){ctx.fillStyle='#ffbe6b';ctx.fillText('FAULT INJECTED',w-195,30);}
  }
  $('scenario').addEventListener('change',select);$('time').addEventListener('input',e=>{index=Number(e.target.value);draw();});
  $('play').addEventListener('click',()=>{if(index===active.trace.length-1)index=0;playing=!playing;$('play').textContent=playing?'Pause':'Play';});
  function tick(now){if(playing&&now-last>=100){last=now;index=Math.min(index+1,active.trace.length-1);draw();if(index===active.trace.length-1){playing=false;$('play').textContent='Play';}}requestAnimationFrame(tick);}
  window.addEventListener('resize',()=>draw());
  select();requestAnimationFrame(tick);
})();
