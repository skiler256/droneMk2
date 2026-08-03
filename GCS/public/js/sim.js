/* ============================================================================
   sim.js — source de donnees factice.

   Sert a deux choses :
     1. Faire vivre l'IHM sans backend (demo, travail sur le CSS, capture).
     2. Documenter par l'exemple la forme attendue de l'etat : ce fichier ecrit
        exactement les memes champs que GCS.applyPacket(), dans les memes unites.

   Pour passer en reel : ne pas demarrer le simulateur dans main.js et appeler
   GCS.link.connect(...) a la place. Aucune autre modification.
   ========================================================================== */

(function (global) {
  'use strict';

  var GCS = global.GCS = global.GCS || {};

  /* Meme plan de vol que le viewport 3D (x = est, z = nord, alt en m). */
  var WP = global.GCS_WAYPOINTS || [
    { x: 0, z: 0, alt: 60 }, { x: -120, z: 140, alt: 80 }, { x: -260, z: 320, alt: 110 },
    { x: -40, z: 460, alt: 120 }, { x: 220, z: 420, alt: 110 }, { x: 340, z: 180, alt: 95 },
    { x: 180, z: -40, alt: 75 }, { x: 0, z: 0, alt: 60 }
  ];

  var CUM = [0];
  for (var i = 1; i < WP.length; i++) {
    CUM.push(CUM[i - 1] + Math.hypot(WP[i].x - WP[i - 1].x, WP[i].z - WP[i - 1].z));
  }
  var TOTAL = CUM[CUM.length - 1];

  var FEED = [
    ['INFO',   'MavlinkInterface: heartbeat stream stable, 20 Hz'],
    ['NOTICE', 'SensorFusion: EKF3 attitude + velocity converged'],
    ['INFO',   'SysMonitoring: thermal margin 34 °C, all rails nominal'],
    ['NOTICE', 'Navigation: waypoint captured, sequencing next leg'],
    ['INFO',   'MissionControl: plan checksum verified 0x7A31'],
    ['INFO',   'GNSS: 16 SV tracked, RTK fixed, HDOP 0.7'],
    ['NOTICE', 'RadioLink: primary 2.4 GHz preferred, secondary standby'],
    ['INFO',   'Lidar: 1024 returns/scan, closest obstacle clear'],
    ['INFO',   'BatteryMonitor: cell delta 0.02 V, balanced'],
    ['NOTICE', 'SensorFusion: barometer / GNSS altitude agreement 0.4 m'],
    ['CAUTION','RadioLink: secondary uplink margin reduced (recovered)'],
    ['INFO',   'MissionControl: geofence check passed, 0 breaches']
  ];

  /* Bruit deterministe : meme nuage lidar a chaque chargement. */
  function seed(i) { var s = Math.sin(i * 12.9898) * 43758.5453; return s - Math.floor(s); }

  function genLidar() {
    // [cap central deg, dispersion deg, distance m, profondeur m, nb points]
    var clusters = [
      [-74, 14, 34, 3, 12], [-38, 9, 26, 2, 9], [-6, 6, 41, 2.5, 7],
      [22, 18, 19, 2.5, 14], [61, 12, 30, 3, 10], [88, 8, 38, 2, 6]
    ];
    var pts = [], k = 0;
    clusters.forEach(function (c) {
      for (var j = 0; j < c[4]; j++, k++) {
        var brg  = c[0] + (seed(k) - 0.5) * c[1] * 2;
        var dist = c[2] + (seed(k + 300) - 0.5) * c[3] * 2;
        if (Math.abs(brg) > 100) continue;
        pts.push({ bearingDeg: brg, distM: dist });
      }
    });
    return pts;
  }

  function interp(dist) {
    var i = 0;
    while (i < CUM.length - 2 && CUM[i + 1] < dist) i++;
    var segLen = (CUM[i + 1] - CUM[i]) || 1;
    var f = (dist - CUM[i]) / segLen;
    var a = WP[i], b = WP[i + 1];
    var hdg = (Math.atan2(b.x - a.x, b.z - a.z) * 180 / Math.PI + 360) % 360;
    return {
      x: a.x + (b.x - a.x) * f,
      z: a.z + (b.z - a.z) * f,
      alt: a.alt + (b.alt - a.alt) * f,
      targetAlt: b.alt,
      hdg: hdg, leg: i, remain: segLen * (1 - f)
    };
  }

  GCS.sim = {
    t: 0,
    _lidar: genLidar(),
    _lastEvent: 0,
    _lastIdx: -1,
    speed: 1,          // multiplicateur de temps

    /** Amorce les champs qui ne bougent pas. */
    init: function (state) {
      state.lidar.maxRange = 45;
      state.battery.cycles = 148;
      state.mission.planName = 'COASTAL-LOOP-07';
      state.mission.wpCount  = WP.length - 1;
      state.mission.armed    = true;
      state.sys.thermalLimitC = 85;
      for (var i = 0; i < 9; i++) this._pushEvent(state);
    },

    _pushEvent: function (state) {
      var idx = Math.floor(Math.random() * FEED.length);
      if (idx === this._lastIdx) idx = (idx + 3) % FEED.length;
      if (FEED[idx][0] === 'CAUTION' && Math.random() < 0.72) idx = (idx + 5) % FEED.length;
      this._lastIdx = idx;
      GCS.pushEvent(state, FEED[idx][0], FEED[idx][1]);
    },

    /** @param {number} dt secondes ecoulees depuis l'appel precedent */
    update: function (state, dt, now) {
      this.t += dt * this.speed;
      var t = this.t;

      var gs        = 16 + Math.sin(t * 0.15) * 2.4;
      var traveled  = (t * 16) % TOTAL;
      var pos       = interp(traveled);
      var pitch     = Math.sin(t * 0.33) * 2.4 + Math.sin(t * 0.12) * 1.2;
      var roll      = Math.sin(t * 0.21) * 7 + Math.sin(t * 0.5) * 2;
      var climbRate = (pos.targetAlt - pos.alt) * 0.03 + Math.cos(t * 0.4) * 0.25;
      var battPct   = Math.max(46, 88 - t * 0.018);
      var distHome  = Math.hypot(pos.x, pos.z);

      /* ── GNSS ─────────────────────────────────────────────────────────── */
      state.gnss.fixType    = 6;                       // RTK fix
      state.gnss.satUsed    = 16;
      state.gnss.satVisible = 24;
      state.gnss.hdop       = 0.7;
      state.gnss.vdop       = 1.0;
      state.gnss.lat        = 37.8199;
      state.gnss.lon        = -122.4783;
      state.gnss.altMsl     = pos.alt + 12;
      state.gnss.snr.length = 0;
      for (var s = 0; s < 14; s++) {
        state.gnss.snr.push(28 + 16 * (0.5 + 0.5 * Math.sin(t * 0.4 + s * 1.7)));
      }
      GCS.touch(state, 'gnss');

      /* ── Attitude ─────────────────────────────────────────────────────── */
      state.attitude.roll  = roll;
      state.attitude.pitch = pitch;
      state.attitude.yaw   = pos.hdg;
      GCS.touch(state, 'attitude');

      /* ── Position / vitesse (NED) ─────────────────────────────────────── */
      var hdgRad = pos.hdg * Math.PI / 180;
      state.position.north    = pos.z;
      state.position.east     = pos.x;
      state.position.down     = -pos.alt;
      state.position.aglAlt   = pos.alt;
      state.position.distHome = distHome;
      state.velocity.vn = Math.cos(hdgRad) * gs;
      state.velocity.ve = Math.sin(hdgRad) * gs;
      state.velocity.vd = -climbRate;                  // NED : positif = descente
      state.velocity.groundSpeed = gs;
      GCS.touch(state, 'position');

      /* ── Batterie ─────────────────────────────────────────────────────── */
      state.battery.pct              = battPct;
      state.battery.voltage          = 21.0 + (battPct / 100) * 3.4;
      state.battery.current          = 8.4 + Math.sin(t * 0.6) * 1.5;
      state.battery.tempC            = 32 + Math.sin(t * 0.15) * 2;
      state.battery.consumed_mAh     = 2600 - battPct * 10;
      state.battery.secondsRemaining = Math.max(0, (battPct - 22) * 0.62 * 60);
      state.battery.rtlNeedPct       = 11 + distHome / 90;
      state.battery.cells.length = 0;
      for (var c = 0; c < 6; c++) {
        state.battery.cells.push(Math.max(10, Math.min(100, battPct + Math.sin(t * 0.3 + c) * 2)));
      }
      GCS.touch(state, 'battery');

      /* ── Lidar ────────────────────────────────────────────────────────── */
      state.lidar.points = this._lidar;
      GCS.touch(state, 'lidar');

      /* ── Mission ──────────────────────────────────────────────────────── */
      var nextWp = (pos.leg + 1) % (WP.length - 1);
      var etaSec = pos.remain / Math.max(4, gs);
      var lastLeg = pos.leg === WP.length - 2;
      state.mission.state       = 'MISSION';
      state.mission.legIndex    = pos.leg;
      state.mission.nextWpIndex = nextWp;
      state.mission.nextWpDistM = pos.remain;
      state.mission.etaSec      = etaSec;
      state.mission.pathFrac    = traveled / TOTAL;
      state.mission.lateralMode = {
        engaged: 'NAV WPT' + nextWp,
        armed: pos.remain < 90 ? 'CURVE ' + ((nextWp + 1) % (WP.length - 1)) : 'TRACK HOLD'
      };
      state.mission.verticalMode = {
        engaged: climbRate > 0.35 ? 'CLIMB' : climbRate < -0.35 ? 'DESCEND' : 'ALT HOLD',
        armed: 'ALT ' + pos.targetAlt.toFixed(0)
      };
      state.mission.speedMode = {
        engaged: lastLeg ? 'APPROACH' : 'SPD CONST',
        armed: lastLeg ? 'LAND' : 'SPD 16'
      };
      GCS.touch(state, 'mission');

      /* ── Camera ───────────────────────────────────────────────────────── */
      state.camera.gimbalPitch = -24;
      state.camera.gimbalYaw   = 0;
      state.camera.zoom        = 1 + Math.abs(Math.sin(t * 0.05)) * 1.4;

      /* ── Systeme ──────────────────────────────────────────────────────── */
      state.sys.cpuTempC   = 46 + Math.sin(t * 0.2) * 3.5;
      state.sys.cpuLoadPct = 37 + Math.sin(t * 0.25) * 7;
      state.sys.ramUsedGb  = 2.2 + Math.sin(t * 0.2) * 0.2;
      state.sys.uptimeSec  = t;
      GCS.touch(state, 'sys');

      /* ── Composants ───────────────────────────────────────────────────── */
      state.components = [
        { name: 'SYS MONITORING', hz: 50 + Math.sin(t * 0.5),      note: 'watchdog fed · 0 resets', health: 'NOMINAL' },
        { name: 'MAVLINK IF',     hz: 20 + Math.sin(t * 0.4),      note: 'hb ok · seq 0 drop',      health: 'NOMINAL' },
        { name: 'SENSOR FUSION',  hz: 400 + Math.sin(t * 0.6) * 4, note: 'EKF3 · innov 0.03',       health: 'NOMINAL' },
        { name: 'NAVIGATION',     hz: 50 + Math.sin(t * 0.3),      note: 'xtrack 0.4 m',            health: 'NOMINAL' },
        { name: 'MISSION CTRL',   hz: 10 + Math.sin(t * 0.2),
          note: 'leg ' + (pos.leg + 1) + '/' + (WP.length - 1) + ' · plan valid',                   health: 'NOMINAL' }
      ];
      GCS.touch(state, 'components');

      /* ── Radio ────────────────────────────────────────────────────────── */
      state.radio = [
        {
          name: 'PRIMARY · 2.4 GHz', role: 'ACTIVE', active: true, quality: 0.9,
          rssi: -57 + Math.sin(t * 0.4) * 4, rttMs: 32 + Math.sin(t * 0.6) * 5,
          lossPct: 0.0, downMbps: 2.4 + Math.sin(t * 0.25) * 0.3,
          upMbps: 0.62 + Math.sin(t * 0.33) * 0.07, band: 'LTE BOND OFF'
        },
        {
          name: 'SECONDARY · 900 MHz', role: 'STANDBY', active: false, quality: 0.55,
          rssi: -79 + Math.sin(t * 0.3) * 5, rttMs: 86 + Math.sin(t * 0.5) * 12,
          lossPct: 0.4 + Math.abs(Math.sin(t * 0.2)) * 0.6,
          downMbps: 0.28 + Math.sin(t * 0.3) * 0.05,
          upMbps: 0.11 + Math.sin(t * 0.4) * 0.02, band: 'FAILOVER READY'
        }
      ];
      GCS.touch(state, 'radio');

      /* ── Journal ──────────────────────────────────────────────────────── */
      if (now - this._lastEvent > 2400) {
        this._lastEvent = now;
        this._pushEvent(state);
      }
    }
  };

})(window);
