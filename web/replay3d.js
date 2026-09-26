/* Interactive 3D replay of recorded telemetry (three.js 0.147 global build).
   It re-draws the recorded evidence; the simulator footage itself is the Gazebo video. */
'use strict';
(() => {
  if (!window.THREE) return;
  const T = window.THREE;
  // ENU metres (east, north, up) to three.js axes (x east, y up, z south).
  const at = (e, n, u) => new T.Vector3(e, u, -n);
  const DRONE_SCALE = 1.5;  // Drawn 1.5x so a 0.5 m airframe stays visible across a 14 m scene.

  function droneModel(color, hostile) {
    const group = new T.Group();
    // A hostile (simulated intruder) gets a red airframe so it never reads as one of the fleet.
    const shell = new T.MeshStandardMaterial({ color: hostile ? 0xb3261e : 0x30363b, roughness: 0.55, metalness: 0.35 });
    group.add(new T.Mesh(new T.BoxGeometry(0.2, 0.07, 0.2), shell));
    const arm = new T.BoxGeometry(0.52, 0.025, 0.035);
    for (const angle of [Math.PI / 4, -Math.PI / 4]) { const m = new T.Mesh(arm, shell); m.rotation.y = angle; group.add(m); }
    const blade = new T.MeshBasicMaterial({ color: 0xd8eef3, transparent: true, opacity: 0.35, side: T.DoubleSide });
    const rotors = [];
    for (const [x, z] of [[0.18, 0.18], [-0.18, 0.18], [0.18, -0.18], [-0.18, -0.18]]) {
      const motor = new T.Mesh(new T.CylinderGeometry(0.025, 0.025, 0.05, 12), shell); motor.position.set(x, 0.025, z); group.add(motor);
      const rotor = new T.Mesh(new T.CircleGeometry(0.12, 24), blade); rotor.rotation.x = -Math.PI / 2; rotor.position.set(x, 0.055, z);
      group.add(rotor); rotors.push(rotor);
    }
    const led = new T.Mesh(new T.SphereGeometry(0.035, 12, 8), new T.MeshBasicMaterial({ color }));
    led.position.y = -0.05; group.add(led);
    group.scale.setScalar(DRONE_SCALE);
    group.traverse(o => { if (o.isMesh) o.castShadow = true; });
    group.userData.rotors = rotors;
    return group;
  }

  function indexAt(samples, t) {  // Last sample at or before t (binary search).
    let lo = 0, hi = samples.length - 1;
    if (t <= samples[0].t) return 0;
    while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (samples[mid].t <= t) lo = mid; else hi = mid - 1; }
    return lo;
  }

  class Replay3D {
    constructor(container) {
      this.container = container;
      this.renderer = new T.WebGLRenderer({ antialias: true });
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      this.renderer.shadowMap.enabled = true;
      container.append(this.renderer.domElement);
      this.scene = new T.Scene();
      this.scene.background = new T.Color(0x0f2531);
      this.scene.fog = new T.Fog(0x0f2531, 30, 70);
      this.camera = new T.PerspectiveCamera(45, 1, 0.1, 200);
      this.controls = new T.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true; this.controls.maxPolarAngle = Math.PI * 0.49;
      this.scene.add(new T.HemisphereLight(0xdcefff, 0x2b3a2b, 0.75));
      const sun = new T.DirectionalLight(0xffffff, 0.85);
      sun.position.set(-8, 18, 6); sun.castShadow = true;
      Object.assign(sun.shadow.camera, { left: -20, right: 20, top: 20, bottom: -20, far: 60 });
      sun.shadow.mapSize.set(2048, 2048);
      this.scene.add(sun);
      const ground = new T.Mesh(new T.PlaneGeometry(80, 80), new T.MeshStandardMaterial({ color: 0x4d5d4c, roughness: 1 }));
      ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true; this.scene.add(ground);
      const grid = new T.GridHelper(16, 16, 0x6f8f86, 0x3e524d); grid.position.set(5, 0.005, -5); this.scene.add(grid);
      this.world = new T.Group(); this.scene.add(this.world);
      this.vehicles = []; this.carrier = null; this.active = false; this.last = 0;
      new ResizeObserver(() => this.resize()).observe(container);
      const loop = now => { requestAnimationFrame(loop); if (this.active) this.render(now); };
      requestAnimationFrame(loop);
    }
    resize() {
      const w = this.container.clientWidth, h = this.container.clientHeight;
      if (!w || !h) return;
      this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
    }
    clear() {
      this.world.traverse(o => { o.geometry?.dispose?.(); if (o.material && !o.material.shared) o.material.dispose?.(); });
      this.world.clear(); this.vehicles = []; this.carrier = null;
    }
    /** data: {obstacles, markers, zones, vehicles:[{name,color,hostile,ground,samples:[{t,e,n,u,valid}],plans:[{t,points}]}], carrier, view} */
    load(data) {
      this.clear();
      const rock = new T.MeshStandardMaterial({ color: 0xa4533f, roughness: 0.8 });
      for (const [x, y, r] of data.obstacles) {
        const m = new T.Mesh(new T.CylinderGeometry(r, r, 6, 40), rock);
        m.position.copy(at(x, y, 3)); m.castShadow = m.receiveShadow = true; this.world.add(m);
      }
      for (const { e, n, radius = 0.6, color } of data.markers || []) {
        const m = new T.Mesh(new T.CylinderGeometry(radius, radius, 0.02, 40), new T.MeshStandardMaterial({ color }));
        m.position.copy(at(e, n, 0.011)); m.receiveShadow = true; this.world.add(m);
      }
      for (const { e, n, r, color, opacity = 0.28 } of data.zones || []) {  // Flat translucent areas, such as a jamming zone.
        const m = new T.Mesh(new T.CircleGeometry(r, 64), new T.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false }));
        m.rotation.x = -Math.PI / 2; m.position.copy(at(e, n, 0.016)); this.world.add(m);
      }
      if (data.carrier) this.carrier = this.addCarrier(data.carrier);
      for (const v of data.vehicles) this.vehicles.push(this.addVehicle(v));
      const [fe, fn, fu] = data.view?.from || [-6, 15, 9], [te, tn, tu] = data.view?.to || [4.5, 4.5, 1];
      this.camera.position.copy(at(fe, fn, fu)); this.controls.target.copy(at(te, tn, tu)); this.controls.update();
      this.resize();
    }
    addVehicle(v) {
      const n = v.samples.length, positions = new Float32Array(n * 3), colors = new Float32Array(n * 3);
      const ok = new T.Color(v.color), invalid = new T.Color(0x8a9ba3);
      v.samples.forEach((s, i) => { at(s.e, s.n, s.u).toArray(positions, i * 3); (s.valid === false ? invalid : ok).toArray(colors, i * 3); });
      const trailGeometry = new T.BufferGeometry();
      trailGeometry.setAttribute('position', new T.BufferAttribute(positions, 3));
      trailGeometry.setAttribute('color', new T.BufferAttribute(colors, 3));
      const trail = new T.Line(trailGeometry, new T.LineBasicMaterial({ vertexColors: true }));
      trailGeometry.setDrawRange(0, 1); this.world.add(trail);
      const routes = (v.plans || []).map(plan => {
        const line = new T.Line(new T.BufferGeometry().setFromPoints(plan.points.map(([e, n, u]) => at(e, n, u))),
          new T.LineDashedMaterial({ color: 0x7ca1b3, dashSize: 0.35, gapSize: 0.25 }));
        line.computeLineDistances(); line.visible = false; this.world.add(line); return { t: plan.t, line };
      });
      const model = droneModel(v.color, v.hostile); this.world.add(model);
      return { ...v, trail, routes, model };
    }
    addCarrier(c) {
      const group = new T.Group();
      const [length, width, height] = c.size;
      const deck = new T.Mesh(new T.BoxGeometry(length, height, width), new T.MeshStandardMaterial({ color: 0x3c4a55, roughness: 0.7 }));
      deck.position.y = height / 2; deck.castShadow = deck.receiveShadow = true; group.add(deck);
      for (const [pe, pn] of c.pads || []) {
        const pad = new T.Mesh(new T.CylinderGeometry(0.4, 0.4, 0.01, 32), new T.MeshStandardMaterial({ color: 0xeaeaea }));
        pad.position.set(pe, height + 0.006, -pn); group.add(pad);
      }
      this.world.add(group);
      return { ...c, group };
    }
    setTime(t) {
      this.time = t;
      for (const v of this.vehicles) {
        const s = v.samples, i = indexAt(s, t), a = s[i], b = s[Math.min(i + 1, s.length - 1)];
        const f = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 0;
        v.model.position.copy(at(a.e + (b.e - a.e) * f, a.n + (b.n - a.n) * f, a.u + (b.u - a.u) * f));
        // Lean into the direction of travel, as a multirotor does.
        const dt = Math.max(b.t - a.t, 1e-3), ve = (b.e - a.e) / dt, vn = (b.n - a.n) / dt, lean = 0.09;
        v.model.rotation.set(Math.max(-0.35, Math.min(0.35, -vn * lean)), 0, Math.max(-0.35, Math.min(0.35, -ve * lean)));
        v.airborne = a.u > (v.ground || 0) + 0.15;  // Rotors spin only above the vehicle's own ground (deck or field).
        v.trail.geometry.setDrawRange(0, i + 1);
        let shown = null;
        for (const r of v.routes) if (r.t <= t) shown = r;
        v.routes.forEach(r => { r.line.visible = r === shown; });
      }
      if (this.carrier) {
        const s = this.carrier.samples, i = indexAt(s, t), a = s[i], b = s[Math.min(i + 1, s.length - 1)];
        const f = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 0;
        this.carrier.group.position.copy(at(a.e + (b.e - a.e) * f, a.n + (b.n - a.n) * f, 0));
        this.carrier.group.rotation.y = a.yaw || 0;
      }
    }
    render(now) {
      const dt = Math.min(0.1, (now - this.last) / 1000 || 0); this.last = now;
      for (const v of this.vehicles) if (v.airborne) for (const r of v.model.userData.rotors) r.rotation.z += dt * 60;
      this.controls.update(); this.renderer.render(this.scene, this.camera);
    }
  }
  window.Replay3D = Replay3D;
})();
