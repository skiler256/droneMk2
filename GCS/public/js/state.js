/* ============================================================================
   state.js — modele de donnees du GCS.

   Regle : cet objet contient des GRANDEURS PHYSIQUES en unites SI, jamais des
   chaines formatees. Le formatage (toFixed, padStart, couleurs, %) est du
   ressort exclusif de render.js. Consequence pratique : le handler WebSocket
   n'a qu'a ecrire des nombres, il n'a jamais a savoir comment ils s'affichent.

   Unites : metres, m/s, radians->non (degres pour les angles, plus lisible
   cote UI), volts, amperes, degres Celsius, secondes.

   Fraicheur : chaque groupe de champs porte un horodatage dans state.rx.
   La boucle appelle markStale() qui compare a STALE_MS et pose state.stale.X.
   render.js ajoute alors la classe .is-stale sur le panneau concerne.
   ========================================================================== */

(function (global) {
  'use strict';

  var GCS = global.GCS = global.GCS || {};

  /* Au-dela de ce delai sans paquet, le groupe est considere perime.
     Valeurs calees sur les RateHz de TELEMETRY.csv, avec ~3 periodes de marge. */
  GCS.STALE_MS = {
    gnss:       2000,   // GPSFIX ~ 2 Hz
    attitude:    600,   // ATTITUDE ~ 10 Hz
    position:    600,   // POS / VELNED ~ 10 Hz
    battery:    4000,   // BATTERY ~ 1 Hz
    lidar:      1500,
    mission:    4000,   // MISSION_STATE evenementiel + redondance 1 Hz
    sys:        4000,   // HEALTH ~ 1 Hz
    radio:      4000,
    components: 4000,
    hw:         4000   // ESC_RPM a 2 Hz + CODE evenementiel
  };

  GCS.createState = function createState() {
    return {
      /* ── horodatages de reception (ms, base performance.now()) ────────── */
      rx: {
        gnss: 0, attitude: 0, position: 0, battery: 0,
        lidar: 0, mission: 0, sys: 0, radio: 0, components: 0, hw: 0
      },
      stale: {},

      /* ── etat de l'IHM (local, jamais recu du drone) ──────────────────── */
      ui: {
        tab: '3d',            // '3d' | 'cam'
        camMode: 'chase',     // 'chase' | 'orbit' | 'top'
        stageScale: 1,
        stageOx: 0,
        stageOy: 0
      },

      /* ── GNSS ─────────────────────────────────────────────────────────── */
      gnss: {
        fixType: 0,           // 0 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fix
        satUsed: 0,
        satVisible: 0,
        hdop: 99.9,
        vdop: 99.9,
        lat: 0,               // degres decimaux
        lon: 0,
        altMsl: 0,            // m
        snr: []               // dBHz par satellite, pour l'histogramme
      },

      /* ── Attitude (degres) ────────────────────────────────────────────── */
      attitude: { roll: 0, pitch: 0, yaw: 0 },

      /* ── Position / vitesse, repere NED local (origine = HomeCalibrator) ─ */
      position: {
        north: 0, east: 0, down: 0,   // m
        aglAlt: 0,                    // m sol
        distHome: 0                   // m
      },
      velocity: {
        vn: 0, ve: 0, vd: 0,          // m/s (vd > 0 = descente, convention NED)
        groundSpeed: 0                // m/s
      },

      /* ── Batterie ─────────────────────────────────────────────────────── */
      // Seuls pct/voltage/current viennent du drone (paquet BATTERY). Les
      // deux estimations restent null tant que le calcul n'est pas tranche :
      // null -> '--' a l'affichage, jamais 0 qui se lirait comme une mesure.
      battery: {
        pct: null,
        voltage: null,        // V
        current: null,        // A
        minutesRemaining: null,
        kmRemaining: null
      },

      /* ── Etat materiel : alimente le schema drone ─────────────────────
         faults : {forme -> 'warn'|'crit'} deduit des codes actifs (colonne
         Schematic d'INDEX.csv). rpm : null tant qu'ESC_RPM n'a rien donne. */
      hw: {
        faults: {},
        rpm: null              // [4] tr/min, ordre des sorties FC
      },

      /* ── Lidar, repere body, cap = 0 deg ──────────────────────────────── */
      lidar: {
        maxRange: 45,        // m — rayon du scope
        points: []           // [{ bearingDeg: -100..100, distM }]
      },

      /* ── Mission ──────────────────────────────────────────────────────── */
      mission: {
        state: 'IDLE',       // IDLE|PRE_FLIGHT|ARMED|MISSION|MANUAL|HOLD|RTB|LANDING|EMERGENCY
        armed: false,
        planName: '--',
        wpCount: 0,
        legIndex: 0,
        nextWpIndex: 0,
        nextWpDistM: 0,
        etaSec: 0,
        pathFrac: 0,         // 0..1 — progression sur le plan, pour le viewport 3D
        lateralMode:  { engaged: '--', armed: '--' },
        verticalMode: { engaged: '--', armed: '--' },
        speedMode:    { engaged: '--', armed: '--' }
      },

      /* ── Camera / gimbal ──────────────────────────────────────────────── */
      camera: { gimbalPitch: 0, gimbalYaw: 0, zoom: 1 },

      /* ── Sante systeme (Pi 5) ─────────────────────────────────────────── */
      // Seul cpuTempC a une source (paquet CPU_TEMP). Charge CPU, RAM et
      // uptime ne sont transmis par rien : retires avec le panneau thermique.
      sys: {
        cpuTempC: null
      },

      /* ── Composants (un par process) ──────────────────────────────────── */
      components: [],        // [{ name, hz, note, health: 'NOMINAL'|'DEGRADED'|'SICK'|'DEAD', tasks: {id:{hz,target}} }]

      /* ── Liens radio ──────────────────────────────────────────────────── */
      radio: [],             // [{ name, role, active, rssi, rttMs, lossPct, downMbps, upMbps, band, quality 0..1 }]

      /* ── Journal ──────────────────────────────────────────────────────── */
      events: [],            // [{ seq, timeStr, sev, msg }] — le plus recent en tete
      pinned: [],            // evenements CRITICAL persistants
      _eventSeq: 0
    };
  };

  /* Ajoute une ligne au journal. Ring borne : pas de croissance non bornee
     sur une session longue. */
  GCS.pushEvent = function pushEvent(state, sev, msg, maxRows) {
    var d = new Date();
    state._eventSeq += 1;
    state.events.unshift({
      seq: state._eventSeq,
      timeStr: d.toTimeString().slice(0, 8),
      sev: sev,
      msg: msg
    });
    var cap = maxRows || 40;
    if (state.events.length > cap) state.events.length = cap;
  };

  /* Recalcule state.stale.* a partir des horodatages de reception. */
  GCS.markStale = function markStale(state, now) {
    var limits = GCS.STALE_MS;
    for (var key in limits) {
      if (!Object.prototype.hasOwnProperty.call(limits, key)) continue;
      var t = state.rx[key] || 0;
      state.stale[key] = (t === 0) || (now - t > limits[key]);
    }
  };

  /* Helper pour le futur handler WebSocket : horodate un groupe comme frais. */
  GCS.touch = function touch(state, group) {
    state.rx[group] = performance.now();
  };

})(window);
