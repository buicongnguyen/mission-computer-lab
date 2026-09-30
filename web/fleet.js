'use strict';
(() => {
  const $ = id => document.getElementById(id),
    report = window.SITL_REPORT,
    fleet = report && report.fleet;
  if (!fleet) {
    $('error').textContent = 'Fleet evidence missing. Run the SITL matrix and tools/publish_sitl.py.';
    return;
  }
  const COLORS = { px4_0: '#55d5b4', px4_1: '#ffb95c', px4_2: '#a18bff' },
    color = ns => COLORS[ns] || '#e7f4f5';
  const format = v => Number(v).toFixed(2),
    STEP = 0.2,
    start = fleet.replay_start_wall_time;
  // When the flights ran and on what, from the run's own provenance record.
  const recorded = r => {
    const at = new Date((r.provenance?.captured_at_unix ?? Date.parse(r.generated_utc) / 1000) * 1000);
    const day = at.toLocaleDateString('en-GB', { day: 'numeric', month: 'long', year: 'numeric', timeZone: 'UTC' });
    const px4 = Object.keys(r.environment.upstream_revisions || {}).find(k => k.startsWith('PX4')) || 'PX4';
    const same = r.provenance?.inputs_unchanged
      ? '; every runtime input hashed before and after the run, unchanged'
      : '';
    return `Recorded ${day}, ${at.toISOString().slice(11, 16)} UTC: ${px4} with Gazebo Harmonic and ROS 2 Humble in WSL2${same}.`;
  };
  const end = Math.max(...fleet.vehicles.map(v => v.trace.at(-1).wall_time), fleet.carrier.at(-1).wall_time);
  const frames = Math.max(1, Math.ceil((end - start) / STEP)),
    since = t => format(Math.max(0, t - start));
  let index = 0,
    playing = false,
    last = 0,
    view = '3d',
    replay = null;
  const checks = Object.entries(fleet.checks);
  $('overall').textContent = `${checks.filter(([, ok]) => ok).length} / ${checks.length} pass`;
  $('separation').textContent = format(fleet.min_separation_m) + ' m';
  $('travel').textContent = format(fleet.carrier.at(-1).e - fleet.carrier[0].e) + ' m';
  const errors = fleet.vehicles.map(v => v.touchdown && v.touchdown.pad_error).filter(e => typeof e === 'number');
  $('padError').textContent = errors.length ? format(Math.max(...errors)) + ' m' : '—';
  $('provenance').textContent = recorded(report);
  checks.forEach(([name, ok]) => {
    const row = document.createElement('div');
    row.className = 'check';
    const label = document.createElement('span');
    label.textContent = name.replaceAll('_', ' ');
    const state = document.createElement('b');
    state.textContent = ok ? 'PASS' : 'FAIL';
    if (!ok) state.className = 'fail';
    row.append(label, state);
    $('checks').append(row);
  });
  fleet.vehicles.forEach(v => {
    const s = document.createElement('span');
    s.style.setProperty('--dot', color(v.ns));
    s.textContent = `${v.ns}: goal (${v.goal[0]}, ${v.goal[1]}) at ${v.altitude} m`;
    $('legend').append(s);
  });
  const carrierKey = document.createElement('span');
  carrierKey.style.setProperty('--dot', '#9fb0bb');
  carrierKey.textContent = 'carrier and pads';
  $('legend').append(carrierKey);
  const events = [
    ...fleet.vehicles.flatMap(v =>
      v.phases.map(p => ({ at: p.wall_time, text: `${v.ns} ${p.phase}${p.reason ? ' · ' + p.reason : ''}` }))
    ),
    ...fleet.clearances.map(c => ({ at: c.wall_time, text: `Station grants ${c.grant} to ${c.drone}` })),
    ...fleet.vehicles
      .filter(v => v.touchdown)
      .map(v => ({
        at: v.touchdown.wall_time,
        text: `${v.ns} touchdown · pad error ${format(v.touchdown.pad_error)} m · carrier ${format((v.touchdown.carrier || {}).speed || 0)} m/s`
      }))
  ];
  events
    .sort((a, b) => a.at - b.at)
    .forEach(e => {
      const d = document.createElement('div');
      d.className = 'event';
      d.textContent = `${since(e.at)} s  ${e.text}`;
      $('events').append(d);
    });
  // After touchdown a drone rides its pad, but PX4's estimate stops following a moving deck: EKF2 skips
  // GNSS whose ground drift check fails (see the SITL guide). Draw landed drones on their pads instead.
  const carrierAt = t => {
    let lo = 0,
      hi = fleet.carrier.length - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (fleet.carrier[mid].wall_time <= t) lo = mid;
      else hi = mid - 1;
    }
    return fleet.carrier[lo];
  };
  const samplesOf = v =>
    v.trace.map(p => {
      if (v.touchdown && p.wall_time >= v.touchdown.wall_time) {
        const c = carrierAt(p.wall_time),
          yaw = c.yaw || 0;
        return {
          t: p.wall_time,
          e: c.e + v.pad * Math.cos(yaw),
          n: c.n + v.pad * Math.sin(yaw),
          u: fleet.deck_height_m,
          valid: true
        };
      }
      return { t: p.wall_time, e: p.e, n: p.n, u: p.u, valid: p.valid };
    });
  function build3d() {
    if (replay || !window.Replay3D) return;
    try {
      replay = new window.Replay3D($('scene3d'));
    } catch (error) {
      $('scene3d').textContent = 'The 3D view needs WebGL: ' + error.message;
      return;
    }
    replay.load({
      obstacles: report.obstacles,
      view: { from: [-5, -11, 11], to: [4.5, 3, 1] },
      markers: fleet.vehicles.map(v => ({ e: v.goal[0], n: v.goal[1], color: color(v.ns) })),
      carrier: {
        size: [3.2, 1.6, fleet.deck_height_m],
        pads: fleet.vehicles.map(v => [v.pad, 0]),
        samples: fleet.carrier.map(c => ({ t: c.wall_time, e: c.e, n: c.n, yaw: c.yaw }))
      },
      vehicles: fleet.vehicles.map(v => ({
        name: v.ns,
        color: color(v.ns),
        ground: fleet.deck_height_m,
        samples: samplesOf(v),
        plans: [
          ...(v.plan ? [{ t: -Infinity, points: v.plan.path.map(([x, y]) => [x, y, v.altitude]) }] : []),
          ...v.phases
            .filter(p => p.phase === 'return' && p.path)
            .map(p => ({ t: p.wall_time, points: p.path.map(([x, y]) => [x, y, v.altitude]) })),
          ...(v.replans || [])
            .filter(r => r.path.length)
            .map(r => ({ t: r.wall_time, points: r.path.map(([x, y]) => [x, y, v.altitude]) }))
        ]
      }))
    });
    replay.active = view === '3d';
  }
  const videos = { overview: fleet.videos && fleet.videos.overview, deck: fleet.videos && fleet.videos.deck };
  const notes = {
    overview: 'High overview camera in the Gazebo world, recorded during this exact run in simulation time.',
    deck: 'Camera mounted on the carrier, filming its deck as the drones launch and land, recorded in simulation time.'
  };
  const buttons = { '3d': $('view3d'), overview: $('viewOverview'), deck: $('viewDeck') };
  function setView(next) {
    view = next;
    Object.entries(buttons).forEach(([key, b]) => b.setAttribute('aria-pressed', String(key === next)));
    $('scene3d').hidden = next !== '3d';
    $('videoPanel').hidden = next === '3d';
    const video = $('fleetVideo');
    video.pause?.();
    if (next !== '3d') {
      if (videos[next]) {
        video.src = `../artifacts/sitl-sample/fleet_carrier/${videos[next]}`;
        $('videoNote').textContent = notes[next];
      } else {
        video.removeAttribute?.('src');
        $('videoNote').textContent = 'No video was recorded for this camera.';
      }
    }
    if (next === '3d') build3d();
    if (replay) replay.active = next === '3d';
    draw();
  }
  Object.entries(buttons).forEach(([key, b]) => b.addEventListener('click', () => setView(key)));
  function draw() {
    const t = start + index * STEP;
    $('time').value = index;
    $('clock').textContent = since(t) + ' s';
    if (replay) replay.setTime(t);
  }
  $('time').max = frames;
  $('time').addEventListener('input', e => {
    index = Number(e.target.value);
    draw();
  });
  $('play').addEventListener('click', () => {
    if (index >= frames) index = 0;
    playing = !playing;
    $('play').textContent = playing ? 'Pause' : 'Play';
  });
  // Plays at 2x the recorded pace; one replay step is 0.2 s of recorded time.
  function tick(now) {
    if (playing && now - last >= 100) {
      last = now;
      index = Math.min(index + 1, frames);
      draw();
      if (index >= frames) {
        playing = false;
        $('play').textContent = 'Play';
      }
    }
    requestAnimationFrame(tick);
  }
  window.addEventListener('resize', () => draw());
  setView('3d');
  requestAnimationFrame(tick);
  // Deep links such as ?view=deck&t=0.8 open a view at a fraction of the recorded fleet time.
  const query =
    typeof URLSearchParams === 'function'
      ? new URLSearchParams((window.location && window.location.search) || '')
      : null;
  if (query) {
    if (query.has('t')) {
      index = Math.round(Math.min(1, Math.max(0, Number(query.get('t')) || 0)) * frames);
      draw();
    }
    if (['overview', 'deck'].includes(query.get('view'))) setView(query.get('view'));
  }
})();
