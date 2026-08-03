// <flight-viewport-3d> — three.js mission viewport for the GCS dashboard.
// Reads telemetry from attributes: frac (0..1 along path), cam-mode (chase|top|orbit).
(function () {
  const WAYPOINTS = [
    { x: 0, z: 0, alt: 60 },
    { x: -120, z: 140, alt: 80 },
    { x: -260, z: 320, alt: 110 },
    { x: -40, z: 460, alt: 120 },
    { x: 220, z: 420, alt: 110 },
    { x: 340, z: 180, alt: 95 },
    { x: 180, z: -40, alt: 75 },
    { x: 0, z: 0, alt: 60 }
  ];

  const ACCENT = 0x4fd8ea;

  function waitForThree() {
    return new Promise(res => {
      if (window.THREE) return res(window.THREE);
      const iv = setInterval(() => { if (window.THREE) { clearInterval(iv); res(window.THREE); } }, 40);
    });
  }

  function labelSprite(THREE, text, color) {
    const c = document.createElement('canvas');
    c.width = 256; c.height = 64;
    const g = c.getContext('2d');
    g.fillStyle = 'rgba(4,8,11,0.72)';
    g.fillRect(0, 0, 256, 64);
    g.strokeStyle = color; g.lineWidth = 3; g.strokeRect(1.5, 1.5, 253, 61);
    g.fillStyle = color;
    g.font = 'bold 36px "JetBrains Mono", monospace';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(text, 128, 34);
    const tex = new THREE.CanvasTexture(c);
    tex.minFilter = THREE.LinearFilter;
    const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false }));
    spr.scale.set(24, 6, 1);
    return spr;
  }

  class FlightViewport3D extends HTMLElement {
    connectedCallback() {
      if (this._booted) return;
      this._booted = true;
      this.style.display = 'block';
      this.style.position = this.style.position || 'absolute';
      this.style.inset = '0';
      waitForThree().then(THREE => this._init(THREE));
    }

    static get observedAttributes() { return ['frac', 'cam-mode']; }
    attributeChangedCallback() { /* read each frame */ }

    disconnectedCallback() {
      this._dead = true;
      if (this._ro) this._ro.disconnect();
      if (this._renderer) this._renderer.dispose();
    }

    _init(THREE) {
      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0x0a1622);
      scene.fog = new THREE.Fog(0x14283a, 420, 2200);

      // gradient sky dome
      const sky = new THREE.Mesh(
        new THREE.SphereGeometry(2600, 32, 24),
        new THREE.ShaderMaterial({
          side: THREE.BackSide, depthWrite: false,
          uniforms: {},
          vertexShader: 'varying float vH; void main(){ vH = normalize(position).y; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
          fragmentShader: 'varying float vH; void main(){ float h = clamp(vH*0.5+0.5,0.0,1.0); vec3 top = vec3(0.016,0.055,0.098); vec3 mid = vec3(0.055,0.145,0.219); vec3 hor = vec3(0.145,0.325,0.415); vec3 c = h > 0.5 ? mix(mid, top, (h-0.5)*2.0) : mix(hor, mid, h*2.0); gl_FragColor = vec4(c,1.0); }'
        })
      );
      scene.add(sky);

      const camera = new THREE.PerspectiveCamera(52, 1, 1, 4000);
      const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, preserveDrawingBuffer: true });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      this.appendChild(renderer.domElement);
      renderer.domElement.style.display = 'block';
      this._renderer = renderer;

      scene.add(new THREE.HemisphereLight(0x88c8ff, 0x0a1018, 0.9));
      const dir = new THREE.DirectionalLight(0xffffff, 0.65);
      dir.position.set(-200, 400, 180);
      scene.add(dir);

      // terrain plane + grid
      const plane = new THREE.Mesh(
        new THREE.PlaneGeometry(3000, 3000),
        new THREE.MeshStandardMaterial({ color: 0x14232c, roughness: 1, metalness: 0 })
      );
      plane.rotation.x = -Math.PI / 2;
      plane.position.y = -0.5;
      scene.add(plane);

      // distant relief for depth cues
      const hillMat = new THREE.MeshStandardMaterial({ color: 0x18303a, roughness: 1, flatShading: true });
      const seed = i => { const s = Math.sin(i * 78.233) * 43758.5453; return s - Math.floor(s); };
      for (let i = 0; i < 18; i++) {
        const a = (i / 18) * Math.PI * 2 + seed(i) * 0.3;
        const r = 620 + seed(i + 40) * 620;
        const h = 40 + seed(i + 80) * 130;
        const hill = new THREE.Mesh(new THREE.ConeGeometry(120 + seed(i + 12) * 180, h, 5 + Math.floor(seed(i) * 3)), hillMat);
        hill.position.set(Math.cos(a) * r, h / 2 - 6, Math.sin(a) * r);
        hill.rotation.y = seed(i + 5) * 3;
        scene.add(hill);
      }

      const grid = new THREE.GridHelper(2400, 48, 0x2a6070, 0x16394a);
      grid.material.transparent = true;
      grid.material.opacity = 0.55;
      scene.add(grid);

      // flight path
      const pathPts = WAYPOINTS.map(w => new THREE.Vector3(w.x, w.alt, w.z));
      const curve = new THREE.CatmullRomCurve3(pathPts, false, 'catmullrom', 0.25);
      const dense = curve.getPoints(600);
      const pathLine = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(dense),
        new THREE.LineBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.85 })
      );
      scene.add(pathLine);
      this._curve = curve;

      // ground track + drop lines
      const trackPts = dense.map(p => new THREE.Vector3(p.x, 0.2, p.z));
      scene.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(trackPts),
        new THREE.LineBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.22 })
      ));

      // waypoint pylons + markers
      WAYPOINTS.forEach((w, i) => {
        if (i === WAYPOINTS.length - 1) return;
        const isHome = i === 0;
        const col = isHome ? 0x35d67c : ACCENT;
        const pylon = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(w.x, 0, w.z), new THREE.Vector3(w.x, w.alt, w.z)]),
          new THREE.LineDashedMaterial({ color: col, dashSize: 8, gapSize: 6, transparent: true, opacity: 0.5 })
        );
        pylon.computeLineDistances();
        scene.add(pylon);

        const ring = new THREE.Mesh(
          new THREE.TorusGeometry(14, 0.9, 8, 40),
          new THREE.MeshBasicMaterial({ color: col })
        );
        ring.position.set(w.x, w.alt, w.z);
        ring.rotation.x = Math.PI / 2;
        scene.add(ring);

        const pad = new THREE.Mesh(
          new THREE.RingGeometry(10, 13, 40),
          new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.35, side: THREE.DoubleSide })
        );
        pad.position.set(w.x, 0.6, w.z);
        pad.rotation.x = -Math.PI / 2;
        scene.add(pad);

        const spr = labelSprite(THREE, isHome ? 'HOME' : 'WP' + i, isHome ? '#35d67c' : '#4fd8ea');
        spr.position.set(w.x, w.alt + 24, w.z);
        scene.add(spr);
      });

      // drone
      const drone = new THREE.Group();
      drone.scale.setScalar(1.25);
      const bodyMat = new THREE.MeshStandardMaterial({ color: 0xe6f1f4, roughness: 0.45, metalness: 0.3 });
      const armMat = new THREE.MeshStandardMaterial({ color: 0x2b3a42, roughness: 0.6 });
      const body = new THREE.Mesh(new THREE.BoxGeometry(9, 3.4, 13), bodyMat);
      drone.add(body);
      const nose = new THREE.Mesh(new THREE.ConeGeometry(2.6, 6, 4), new THREE.MeshStandardMaterial({ color: ACCENT }));
      nose.rotation.x = -Math.PI / 2;
      nose.position.set(0, 0, -9);
      drone.add(nose);
      this._rotors = [];
      [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(([sx, sz]) => {
        const arm = new THREE.Mesh(new THREE.BoxGeometry(1.6, 1.2, 15), armMat);
        arm.position.set(sx * 5, 0, sz * 5);
        arm.rotation.y = sx * sz > 0 ? Math.PI / 4 : -Math.PI / 4;
        drone.add(arm);
        const hub = new THREE.Vector3(sx * 10, 2, sz * 10);
        const rotor = new THREE.Mesh(
          new THREE.CircleGeometry(7, 28),
          new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.28, side: THREE.DoubleSide })
        );
        rotor.position.copy(hub);
        rotor.rotation.x = -Math.PI / 2;
        drone.add(rotor);
        const blade = new THREE.Mesh(new THREE.BoxGeometry(13.5, 0.5, 1.1), bodyMat);
        blade.position.copy(hub);
        drone.add(blade);
        this._rotors.push(blade);
      });
      scene.add(drone);
      this._drone = drone;

      // ground shadow marker + altitude drop line for the drone
      const shadow = new THREE.Mesh(
        new THREE.RingGeometry(5, 7.5, 32),
        new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.28, side: THREE.DoubleSide })
      );
      shadow.rotation.x = -Math.PI / 2;
      scene.add(shadow);
      this._shadow = shadow;
      const dropGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
      const drop = new THREE.Line(dropGeo, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.2 }));
      scene.add(drop);
      this._drop = drop;

      this._scene = scene;
      this._camera = camera;
      this._THREE = THREE;
      this._camPos = new THREE.Vector3(0, 220, 320);
      this._camLook = new THREE.Vector3();

      const resize = () => {
        const w = this.clientWidth || 800, h = this.clientHeight || 500;
        renderer.setSize(w, h);
        renderer.domElement.style.width = w + 'px';
        renderer.domElement.style.height = h + 'px';
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
      };
      resize();
      this._ro = new ResizeObserver(resize);
      this._ro.observe(this);

      this._clock = new THREE.Clock();
      this._loop();
    }

    _loop() {
      if (this._dead) return;
      requestAnimationFrame(() => this._loop());
      const THREE = this._THREE;
      const dt = this._clock.getDelta();
      const frac = Math.max(0, Math.min(0.9999, parseFloat(this.getAttribute('frac')) || 0));
      const mode = this.getAttribute('cam-mode') || 'chase';

      const p = this._curve.getPointAt(frac);
      const tan = this._curve.getTangentAt(frac);
      this._drone.position.copy(p);
      const heading = Math.atan2(tan.x, tan.z);
      this._drone.rotation.y = heading + Math.PI;
      this._drone.rotation.z = -tan.x * 0.12;
      this._rotors.forEach((r, i) => { r.rotation.y += dt * (i % 2 ? 34 : -34); });

      this._shadow.position.set(p.x, 0.4, p.z);
      this._drop.geometry.setFromPoints([new THREE.Vector3(p.x, 0.4, p.z), p.clone()]);

      let want;
      if (mode === 'top') {
        want = new THREE.Vector3(p.x, 620, p.z + 1);
        this._camLook.lerp(p, 0.12);
      } else if (mode === 'orbit') {
        const a = performance.now() * 0.00013;
        want = new THREE.Vector3(p.x + Math.cos(a) * 210, p.y + 110, p.z + Math.sin(a) * 210);
        this._camLook.lerp(p, 0.1);
      } else {
        want = p.clone().sub(tan.clone().multiplyScalar(155)).add(new THREE.Vector3(0, 50, 0));
        this._camLook.lerp(p.clone().add(tan.clone().multiplyScalar(85)), 0.1);
      }
      if (!this._camInit) {
        this._camInit = true;
        this._camPos.copy(want);
        this._camLook.copy(p);
      }
      this._camPos.lerp(want, mode === 'top' ? 0.08 : 0.06);
      this._camera.position.copy(this._camPos);
      this._camera.up.set(0, 1, 0);
      this._camera.lookAt(this._camLook);
      this._renderer.render(this._scene, this._camera);
    }
  }

  if (!customElements.get('flight-viewport-3d')) {
    customElements.define('flight-viewport-3d', FlightViewport3D);
  }
  window.GCS_WAYPOINTS = WAYPOINTS;
})();
