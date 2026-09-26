'use strict';
(() => {
  const $=id=>document.getElementById(id);
  const R=window.GUARDIAN_REPORT,S=window.SITL_REPORT,G=S&&S.guardian;
  if(!R){$('error').textContent='Guardian evaluation missing. Run tools/guardian_sim.py --output artifacts/guardian.';return;}
  const ARCH=['station_only','onboard_only','networked','hybrid'];
  const ARCH_LABEL={station_only:'Station only',onboard_only:'Onboard only',networked:'Networked',hybrid:'Hybrid'};
  const SCN_LABEL={intruder:'Low intruder',fast_inbound:'Fast inbound',swarm:'Swarm of five',birds:'Birds and clutter',
    jamming:'Jammed picket',spoofing:'GNSS drag-off',center_loss:'No center link',combined:'Combined'};
  const LAYER_LABEL={onboard:'onboard',station:'station',center:'center','station (delegated)':'delegated'};
  const STATUS={GREEN:'#0ca30c',AMBER:'#fab219',RED:'#d03b3b'};
  const fmt=(v,d=0)=>v==null?'—':Number(v).toFixed(d);
  const el=(tag,text,cls)=>{const e=document.createElement(tag);if(text!=null)e.textContent=text;if(cls)e.className=cls;return e;};
  const scenarios=Object.keys(R.summary),sum=(s,a)=>R.summary[s][a];
  const threatScenarios=scenarios.filter(s=>sum(s,'hybrid').guardians_safe_rate!=null);

  // Headline tiles: the proposed design against the baseline that lacks the feature in question.
  const hy=s=>sum(s,'hybrid'),kpi=(label,value,note)=>{const c=el('div',null,'card');c.append(el('small',label),el('strong',value),el('small',note));$('kpis').append(c);};
  kpi('WARNING BEFORE A LOW INTRUDER ARRIVES',fmt(hy('intruder').warning_s?.median)+' s',
    `Median, hybrid; ${fmt(sum('intruder','station_only').warning_s?.median)} s from the station's own sensor`);
  const allClear=threatScenarios.filter(s=>hy(s).guardians_safe_rate===1);
  kpi('GUARDIANS KEPT CLEAR IN EVERY RUN',`${allClear.length} of ${threatScenarios.length}`,'Threat scenarios, hybrid; the exception is the fast inbound object');
  kpi('FALSE ALARMS FROM BIRDS',String(hy('birds').false_red_total),`RED alerts in ${hy('birds').runs} runs with eight circling birds, hybrid`);
  kpi('GNSS DRAG-OFF CAUGHT',fmt(hy('spoofing').spoof_detect_s?.median)+' s',
    `Median, after ${fmt(hy('spoofing').spoof_drift_m?.median)} m of drift; ${fmt(sum('spoofing','networked').spoof_drift_m?.median)} m unnoticed without the cross-check`);
  $('runsNote').textContent=`${R.seeds} seeded runs per cell, the same threats for every design. Parameters are illustrative round numbers (${R.python}, ${R.platform}).`;

  // Heatmaps: one hue, more is stronger; the value is always printed, so colour never carries it alone.
  function heat(table,key,format,max){
    const head=el('tr');head.append(el('th','Scenario','scn'));
    ARCH.forEach(a=>{const th=el('th',ARCH_LABEL[a],'col');if(a==='hybrid')th.style.color='var(--ink)';head.append(th);});
    table.append(head);
    scenarios.forEach(s=>{const tr=el('tr');tr.append(el('th',SCN_LABEL[s]||s));
      ARCH.forEach(a=>{const r=sum(s,a),v=key(r);const td=el('td');
        if(v==null){td.textContent='—';td.className='na';td.setAttribute('aria-label',`${ARCH_LABEL[a]}, ${SCN_LABEL[s]}: not applicable`);}
        else{const f=Math.max(0,Math.min(1,v/max));const pct=Math.round(8+f*72);
          td.textContent=format(v);td.style.background=`color-mix(in oklab, var(--accent) ${pct}%, var(--surface))`;
          td.style.color=pct>52?'var(--accent-ink)':'var(--ink)';
          const spread=key===safe?'':` (10th-90th percentile ${fmt(r.warning_s?.p10)}-${fmt(r.warning_s?.p90)} s)`;
          td.title=`${ARCH_LABEL[a]}, ${SCN_LABEL[s]}: ${format(v)}${spread}, ${r.runs} runs`;td.setAttribute('aria-label',td.title);td.tabIndex=0;}
        tr.append(td);});table.append(tr);});
  }
  const safe=r=>r.guardians_safe_rate,warn=r=>r.warning_s?r.warning_s.median:null;
  heat($('heatSafe'),safe,v=>Math.round(v*100)+'%',1);
  heat($('heatWarn'),warn,v=>fmt(v)+' s',Math.max(...scenarios.flatMap(s=>ARCH.map(a=>warn(sum(s,a))||0))));

  // Other measures that separate the designs.
  const extras=[['False REDs from birds',r=>r.false_red_total,'birds',v=>String(v)],
    ['Jammed picket acts alone (median s after losing its link)',r=>r.jam_fallback_s?.median,'jamming',v=>fmt(v,1)+' s'],
    ['GNSS drag-off detected (median s)',r=>r.spoof_detect_s?.median,'spoofing',v=>fmt(v)+' s'],
    ['Drift of the spoofed picket (median m)',r=>r.spoof_drift_m?.median,'spoofing',v=>fmt(v)+' m'],
    ['Center could decide before arrival (low intruder)',r=>r.center_in_time_rate,'intruder',v=>Math.round(v*100)+'% of runs'],
    ['Never closed on a threat; stayed within authority',r=>Math.min(...scenarios.map(s=>Math.min(R.summary[s][r].never_closed_rate,R.summary[s][r].authority_ok_rate))),null,v=>Math.round(v*100)+'% of runs']];
  const eh=el('tr');eh.append(el('th','Measure'));ARCH.forEach(a=>eh.append(el('th',ARCH_LABEL[a])));$('extras').append(eh);
  extras.forEach(([label,get,s,f])=>{const tr=el('tr');tr.append(el('td',label));
    ARCH.forEach(a=>{const v=s?get(sum(s,a)):get(a);tr.append(el('td',v==null?'—':f(v)));});$('extras').append(tr);});
  const L=R.layouts,lh=el('tr');['Layout','Low intruder warning','Fast inbound kept clear','Fast inbound warning','Swarm warning'].forEach(t=>lh.append(el('th',t)));
  $('layouts').append(lh);
  Object.keys(L.summary).forEach(k=>{const r=L.summary[k],tr=el('tr');const name=el('td');name.append(el('div',k.replace('_',' ')),el('div',L.notes[k],'sub'));
    tr.append(name,el('td',fmt(r.intruder.warning_s?.median)+' s'),el('td',Math.round(r.fast_inbound.guardians_safe_rate*100)+'%'),
      el('td',fmt(r.fast_inbound.warning_s?.median,1)+' s'),el('td',fmt(r.swarm.warning_s?.median)+' s'));$('layouts').append(tr);});

  // Authority matrix and time budgets.
  const budget={onboard:'under a second',station:'seconds',center:'tens of seconds'};
  const ah=el('tr');['Layer','Time budget','Decides','Falls back to'].forEach(t=>ah.append(el('th',t)));$('authority').append(ah);
  [['Onboard (each guardian)','onboard','Keep clear of a predicted conflict inside the reflex horizon; hold, then return, when the uplink goes quiet','Its own sensor and pre-briefed lost-link procedure'],
   ['Station (on the carrier)','station','Fuse tracks, set GREEN / AMBER / RED, alert the crew, order keep-clear, hold or disperse, relocate, switch a spoofed guardian to station fixes','Delegated recovery (land) if the center stays unreachable after a clear'],
   ['Center (command and control)','center','Acknowledge, authorise recovery or resumption after an event, and anything beyond protection','Nothing crosses a dead link; the station keeps protecting']].forEach(row=>{
    const tr=el('tr');tr.append(el('td',row[0]),el('td',budget[row[1]]),el('td',row[2]),el('td',row[3]));$('authority').append(tr);});

  // Fast-simulator replay.
  const canvas=$('map'),ctx=canvas.getContext('2d');let scenario=scenarios[0],arch='hybrid',index=0,playing=false,last=0;
  scenarios.forEach(s=>{const o=el('option',SCN_LABEL[s]||s);o.value=s;$('scenario').append(o);});
  // Each scenario's replay carries the hybrid design and the baseline that shows what it changes.
  ARCH.forEach(a=>{const b=el('button',ARCH_LABEL[a]);b.dataset.arch=a;
    b.addEventListener('click',()=>{arch=a;select();});$('archButtons').append(b);});
  const POSTURE={G:'GREEN',A:'AMBER',R:'RED'};
  const pick=(flat,i,k)=>flat.slice(i*k,i*k+k);
  const seriesAt=(ser,i)=>{const j=i-ser.first;return j<0||j*3>=ser.p.length?null:ser.p.slice(j*3,j*3+3);};
  function frameAt(tr,i){  // One replay second, rebuilt from the columnar arrays.
    const guardians={},threats={},reported={};
    Object.entries(tr.guardians).forEach(([g,v])=>{guardians[g]=[...pick(v.p,i,3),tr.actions[v.a[i]],v.landed[i]];});
    Object.entries(tr.threats).forEach(([t,v])=>{const q=seriesAt(v,i);if(q)threats[t]=q;});
    Object.entries(tr.reported).forEach(([g,v])=>{const q=seriesAt(v,i);if(q)reported[g]=q;});
    const tracks=[];for(let k=0;k<tr.tracks[i].length;k+=2)tracks.push([tr.tracks[i][k],tr.tracks[i][k+1]]);
    return {t:i,posture:POSTURE[tr.posture[i]],station:pick(tr.station,i,2),guardians,threats,reported,tracks};}
  [['Guardian','#6da7ec'],['Threat','#e66767'],['Bird','#9aa7ad'],['Fused track','#e2eef2'],['Station and protected zone','#f0e6c8']].forEach(([t,c])=>{
    const s=el('span',t);s.style.setProperty('--dot',c);$('mapLegend').append(s);});
  const trace=()=>R.traces[scenario][arch]||R.traces[scenario].hybrid;
  function sizeCanvas(){const w=canvas.clientWidth||800,dpr=window.devicePixelRatio||1;canvas.width=Math.round(w*dpr);canvas.height=Math.round(w*610/800*dpr);}
  function draw(){
    const tr=trace(),smp=frameAt(tr,Math.min(index,tr.n-1)),W=canvas.width,H=canvas.height,k=Math.min(W,H)/3400;
    const X=x=>W/2+x*k,Y=y=>H/2-y*k;ctx.clearRect(0,0,W,H);ctx.fillStyle='#0f2531';ctx.fillRect(0,0,W,H);
    const ring=(x,y,r,color,dash=[],width=1)=>{ctx.beginPath();ctx.setLineDash(dash);ctx.strokeStyle=color;ctx.lineWidth=width*(W/800);ctx.arc(X(x),Y(y),r*k,0,2*Math.PI);ctx.stroke();ctx.setLineDash([]);};
    for(let r=500;r<=1500;r+=500)ring(0,0,r,'rgba(160,190,200,.14)');
    if(tr.jammer){ctx.beginPath();ctx.fillStyle='rgba(227,73,72,.16)';ctx.arc(X(tr.jammer.p[0]),Y(tr.jammer.p[1]),tr.jammer.r*k,0,2*Math.PI);ctx.fill();}
    const [sx,sy]=smp.station;ring(sx,sy,1200,'rgba(240,230,200,.25)',[6,6]);ring(sx,sy,400,'rgba(240,230,200,.35)',[2,4]);ring(sx,sy,100,'#f0e6c8',[],2);
    ctx.fillStyle='#f0e6c8';ctx.fillRect(X(sx)-5*W/800,Y(sy)-5*W/800,10*W/800,10*W/800);
    Object.values(tr.posts).forEach(p=>{ctx.strokeStyle='rgba(109,167,236,.5)';ctx.lineWidth=W/800;ctx.beginPath();
      ctx.moveTo(X(p[0])-5,Y(p[1]));ctx.lineTo(X(p[0])+5,Y(p[1]));ctx.moveTo(X(p[0]),Y(p[1])-5);ctx.lineTo(X(p[0]),Y(p[1])+5);ctx.stroke();});
    // Threat trails over the last minute, then threats and birds.
    const from=Math.max(0,index-60);
    Object.keys(smp.threats).forEach(name=>{const bird=name.startsWith('b');ctx.beginPath();ctx.strokeStyle=bird?'rgba(154,167,173,.35)':'rgba(230,103,103,.55)';ctx.lineWidth=1.5*W/800;
      let started=false;for(let i=from;i<=index;i++){const q=seriesAt(tr.threats[name],i);if(!q)continue;if(!started){ctx.moveTo(X(q[0]),Y(q[1]));started=true;}else ctx.lineTo(X(q[0]),Y(q[1]));}ctx.stroke();});
    smp.tracks.forEach(t=>{ctx.strokeStyle='#e2eef2';ctx.lineWidth=W/800;ctx.strokeRect(X(t[0])-4*W/800,Y(t[1])-4*W/800,8*W/800,8*W/800);});
    Object.entries(smp.threats).forEach(([name,q])=>{const bird=name.startsWith('b');ctx.fillStyle=bird?'#9aa7ad':'#e66767';ctx.beginPath();
      ctx.arc(X(q[0]),Y(q[1]),(bird?3:5)*W/800,0,2*Math.PI);ctx.fill();});
    Object.entries(smp.guardians).forEach(([name,g])=>{
      const rep=smp.reported&&smp.reported[name];
      if(rep){ctx.setLineDash([3,3]);ctx.strokeStyle='#6da7ec';ctx.beginPath();ctx.moveTo(X(g[0]),Y(g[1]));ctx.lineTo(X(rep[0]),Y(rep[1]));ctx.stroke();
        ctx.setLineDash([]);ctx.beginPath();ctx.arc(X(rep[0]),Y(rep[1]),6*W/800,0,2*Math.PI);ctx.stroke();}
      ctx.fillStyle=g[4]?'rgba(109,167,236,.4)':'#6da7ec';ctx.beginPath();ctx.arc(X(g[0]),Y(g[1]),6*W/800,0,2*Math.PI);ctx.fill();
      ctx.fillStyle='#e2eef2';ctx.font=`${12*W/800}px system-ui`;ctx.fillText(`${name} · ${g[4]?'landed':g[3].replace(/_/g,' ')}`,X(g[0])+9*W/800,Y(g[1])-8*W/800);});
    ctx.fillStyle='rgba(226,238,242,.7)';ctx.font=`${11*W/800}px system-ui`;ctx.fillText('500 m rings · north up',10*W/800,H-10*W/800);
    const p=$('posture');p.replaceChildren();const dot=el('i');dot.style.background=STATUS[smp.posture];p.append(dot,el('span',smp.posture));
    $('time').value=index;$('clock').textContent=smp.t+' s';
  }
  function select(){scenario=$('scenario').value||scenario;index=0;
    if(!R.traces[scenario][arch])arch='hybrid';
    [...$('archButtons').children].forEach(b=>{const has=Boolean(R.traces[scenario][b.dataset.arch]);b.disabled=!has;
      b.setAttribute('aria-pressed',String(b.dataset.arch===arch));b.title=has?'':'No replay recorded for this design in this scenario';});
    const tr=trace();$('time').max=tr.n-1;
    $('scenarioNote').textContent=R.scenarios[scenario]+' Replays show the hybrid design and the baseline that differs most; the tables cover all four.';
    const box=$('events');box.replaceChildren();
    tr.events.slice(0,500).forEach(e=>{const row=el('div',null,'event');row.append(el('span',fmt(e.t,1).padStart(6)+' s'),
      el('span',LAYER_LABEL[e.layer]||e.layer,'layer '+(e.layer==='onboard'?'onboard':e.layer==='center'?'center':'')),
      el('span',`${e.who} ${e.action.replace(/_/g,' ')}${e.state?' '+e.state:''}${e.miss!=null?' · predicted miss '+e.miss+' m':''}${e.residual!=null?' · residual '+e.residual+' m':''}`));box.append(row);});
    draw();}
  $('scenario').addEventListener('change',select);$('time').addEventListener('input',e=>{index=Number(e.target.value);draw();});
  $('play').addEventListener('click',()=>{if(index>=trace().n-1)index=0;playing=!playing;$('play').textContent=playing?'Pause':'Play';});

  // PX4 flight of the same logic.
  let sitlIndex=0,sitlPlaying=false,sitlFrames=1,replay=null,view='3d',sitlStart=0;
  const COLORS={px4_0:'#55d5b4',px4_1:'#ffb95c',px4_2:'#a18bff'},color=ns=>COLORS[ns]||'#e2eef2';
  if(!G){$('sitlKpis').textContent='The PX4 guardian flight has not been published yet.';}
  else{
    sitlStart=Math.min(...G.vehicles.map(v=>v.trace[0].wall_time));
    const end=Math.max(...G.vehicles.map(v=>v.trace.at(-1).wall_time));sitlFrames=Math.max(1,Math.ceil((end-sitlStart)/0.2));
    const checks=Object.entries(G.checks),since=t=>fmt(Math.max(0,t-sitlStart),1);
    const sk=(label,value,note)=>{const c=el('div',null,'card');c.append(el('small',label),el('strong',value),el('small',note));$('sitlKpis').append(c);};
    sk('CHECKS',`${checks.filter(([,ok])=>ok).length} / ${checks.length}`,'Independent observations');
    sk('WARNING',fmt(G.warning_s,1)+' s','RED to arrival, simulated time');
    sk('CLOSEST GUARDIAN',fmt(Math.min(...Object.values(G.guardian_separation_m)),2)+' m',`To the intruder; safe radius ${G.layout.safe_radius} m`);
    sk('CARRIER MISS',fmt(G.station_miss_m,1)+' m',`After relocating; protected radius ${G.layout.protect_radius} m`);
    checks.forEach(([name,ok])=>{const row=el('div',null,'check');const b=el('b',ok?'PASS':'FAIL');if(!ok)b.className='fail';row.append(el('span',name.replace(/_/g,' ')),b);$('checks').append(row);});
    const ev=[];
    G.posture.forEach(p=>ev.push({at:p.wall_time,layer:'station',text:`posture ${p.state}`}));
    // Orders to the jammed guardian while the jammer is on never arrive; say so rather than imply they did.
    const links=G.jamming.filter(j=>j.kind==='link');
    const undelivered=o=>{const last=links.filter(l=>l.ns===o.ns&&l.wall_time<=o.wall_time).at(-1);return Boolean(last&&last.jammed);};
    G.orders.filter(o=>o.action!=='watch').forEach(o=>ev.push({at:o.wall_time,layer:'station',text:`orders ${o.ns} to ${o.action.replace(/_/g,' ')}`+
      `${o.miss!=null?' · predicted miss '+fmt(o.miss,1)+' m':''}${undelivered(o)?' · not delivered: link jammed':''}`}));
    G.vehicles.forEach(v=>v.guardian.filter(d=>d.layer!=='station'||d.action!=='watch').forEach(d=>ev.push({at:d.wall_time,layer:d.layer,
      text:`${v.ns} ${d.action.replace(/_/g,' ')}${d.link_age!=null&&d.link_age>1?' · no uplink for '+fmt(d.link_age,1)+' s':''}`})));
    G.center.forEach(c=>ev.push({at:c.wall_time,layer:'center',text:`decides ${c.decision}`}));
    G.relocation.forEach(r=>ev.push({at:r.wall_time,layer:'station',text:r.kind==='relocate'?'relocates the carrier':'carrier relocated'}));
    G.jamming.forEach(j=>ev.push({at:j.wall_time,layer:'environment',text:j.kind==='intruder_start'?'intruder appears; jamming on':
      j.kind==='jammer_off'?'jamming off':`${j.ns} link ${j.jammed?'jammed':'restored'}`}));
    G.vehicles.filter(v=>v.touchdown).forEach(v=>ev.push({at:v.touchdown.wall_time,layer:'station',text:`${v.ns} touchdown · pad error ${fmt(v.touchdown.pad_error,2)} m`}));
    ev.sort((a,b)=>a.at-b.at).forEach(e=>{const row=el('div',null,'event');row.append(el('span',since(e.at).padStart(6)+' s'),
      el('span',LAYER_LABEL[e.layer]||e.layer,'layer '+(e.layer==='onboard'?'onboard':e.layer==='center'?'center':'')),el('span',e.text));$('sitlEvents').append(row);});
    G.vehicles.forEach(v=>{const s=el('span',`${v.ns}: post (${v.goal[0]}, ${v.goal[1]}) at ${v.altitude} m`);s.style.setProperty('--dot',color(v.ns));$('sitlLegend').append(s);});
    [['intruder','#e66767'],['jamming zone','rgba(227,73,72,.6)']].forEach(([t,c])=>{const s=el('span',t);s.style.setProperty('--dot',c);$('sitlLegend').append(s);});
    $('provenance').textContent=`Fast simulator: ${R.seeds} seeds per cell. PX4 flight recorded ${S.generated_utc} · ${S.environment.platform}`;
  }
  const carrierAt=t=>{const c=G.carrier;let lo=0,hi=c.length-1;while(lo<hi){const m=(lo+hi+1)>>1;if(c[m].wall_time<=t)lo=m;else hi=m-1;}return c[lo];};
  function build3d(){if(replay||!window.Replay3D||!G)return;
    try{replay=new window.Replay3D($('scene3d'));}catch(error){$('scene3d').textContent='The 3D view needs WebGL: '+error.message;return;}
    replay.load({obstacles:S.obstacles,view:{from:[-9,-12,14],to:[3,5,1]},
      markers:G.vehicles.map(v=>({e:v.goal[0],n:v.goal[1],color:color(v.ns)})),
      zones:[{e:G.layout.jammer.p[0],n:G.layout.jammer.p[1],r:G.layout.jammer.r,color:0xe34948}],
      carrier:{size:[3.2,1.6,G.deck_height_m],pads:G.vehicles.map(v=>[v.pad,0]),samples:G.carrier.map(c=>({t:c.wall_time,e:c.e,n:c.n,yaw:c.yaw}))},
      vehicles:[...G.vehicles.map(v=>({name:v.ns,color:color(v.ns),ground:G.deck_height_m,samples:v.trace.map(p=>{
          if(v.touchdown&&p.wall_time>=v.touchdown.wall_time){const c=carrierAt(p.wall_time);return{t:p.wall_time,e:c.e+v.pad*Math.cos(c.yaw||0),n:c.n+v.pad*Math.sin(c.yaw||0),u:G.deck_height_m,valid:true};}
          return{t:p.wall_time,e:p.e,n:p.n,u:p.u,valid:p.valid};}),plans:[]})),
        {name:'intruder',color:'#e34948',hostile:true,ground:-1,samples:G.intruder.map(q=>({t:q.wall_time,e:q.e,n:q.n,u:q.u,valid:true})),plans:[]}]});
    replay.active=view==='3d';}
  const videos={overview:G&&G.videos&&G.videos.overview,close:G&&G.videos&&G.videos.close};
  const notes={overview:'Overview camera in the Gazebo world, recorded during this flight in simulation time.',close:'Low close-up of the intruder’s final approach, the jammed guardian keeping clear and the carrier driving away.'};
  const buttons={'3d':$('view3d'),overview:$('viewOverview'),close:$('viewClose')};
  function setView(next){view=next;Object.entries(buttons).forEach(([k,b])=>b.setAttribute('aria-pressed',String(k===next)));
    $('scene3d').hidden=next!=='3d';$('videoPanel').hidden=next==='3d';const video=$('flightVideo');video.pause?.();
    if(next!=='3d'){if(videos[next]){video.src=`../artifacts/sitl-sample/guardian_intruder/${videos[next]}`;$('videoNote').textContent=notes[next];}
      else{video.removeAttribute?.('src');$('videoNote').textContent='No video was recorded for this camera.';}}
    if(next==='3d')build3d();if(replay)replay.active=next==='3d';drawSitl();}
  Object.entries(buttons).forEach(([k,b])=>b.addEventListener('click',()=>setView(k)));
  function drawSitl(){const t=sitlStart+sitlIndex*0.2;$('sitlTime').value=sitlIndex;$('sitlClock').textContent=fmt(sitlIndex*0.2,1)+' s';if(replay)replay.setTime(t);}
  $('sitlTime').max=sitlFrames;$('sitlTime').addEventListener('input',e=>{sitlIndex=Number(e.target.value);drawSitl();});
  $('sitlPlay').addEventListener('click',()=>{if(sitlIndex>=sitlFrames)sitlIndex=0;sitlPlaying=!sitlPlaying;$('sitlPlay').textContent=sitlPlaying?'Pause':'Play';});
  function tick(now){if(now-last>=100){last=now;
      if(playing){index=Math.min(index+1,trace().n-1);draw();if(index>=trace().n-1){playing=false;$('play').textContent='Play';}}
      if(sitlPlaying){sitlIndex=Math.min(sitlIndex+1,sitlFrames);drawSitl();if(sitlIndex>=sitlFrames){sitlPlaying=false;$('sitlPlay').textContent='Play';}}}
    requestAnimationFrame(tick);}
  window.addEventListener('resize',()=>{sizeCanvas();draw();drawSitl();});
  sizeCanvas();$('scenario').value=scenario;select();if(G)setView('3d');requestAnimationFrame(tick);
})();
