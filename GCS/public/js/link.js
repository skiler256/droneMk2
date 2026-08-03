/* ============================================================================
   link.js — couture WebSocket.

   C'EST LE SEUL FICHIER A MODIFIER pour brancher le dashboard sur le drone.
   Il ne fait rien d'autre que : recevoir un paquet, ecrire dans state, marquer
   le groupe comme frais. La boucle de rendu ne sait rien de la provenance des
   donnees et n'a pas a le savoir.

   Deux points a retenir :

   - RIEN N'EST SYNCHRONE AVEC LE RENDU. Les paquets arrivent a leur rythme
     (POS a 10 Hz, HEALTH a 1 Hz, MISSION_STATE evenementiel) ; le rendu tourne
     a sa frequence propre. C'est voulu : un pic de trafic ne doit pas faire
     rendre l'UI 200 fois par seconde, et une coupure de lien ne doit pas geler
     l'affichage — elle doit le faire passer en "perime" (cf. state.stale).

   - GCS.touch(state, groupe) A CHAQUE ECRITURE. C'est ce qui alimente
     l'indicateur de fraicheur. Oublier le touch = panneau grise en permanence.
   ========================================================================== */

(function (global) {
  'use strict';

  var GCS = global.GCS = global.GCS || {};

  /* ── Application d'un paquet decode sur l'etat ─────────────────────────── */

  /**
   * Les noms de cas correspondent aux PacketID de TELEMETRY.csv.
   * A adapter au format que ton pont WebSocket emet reellement (JSON, ou
   * binaire decode par un codec equivalent a telemetry_codec.py).
   *
   * @param {object} state
   * @param {{id: string, payload: object}} pkt
   */
  // Enum MissionState reel (SharedCmdMem.hpp) — PAS de RTB en V1 (tranche
  // explicitement plus tot dans le projet), contrairement au commentaire
  // d'origine de state.js qui en listait un.
  var MISSION_STATE_NAMES = [
    'IDLE', 'PRE_FLIGHT', 'ARMED', 'MISSION', 'MANUAL', 'HOLD', 'LANDING', 'EMERGENCY'
  ];

  function codeHex(packed) {
    return packed ? '0x' + (packed >>> 0).toString(16) : '--';
  }

  // Table CODES:: (packed uint32 -> {code, name, severity}), chargee de
  // /codes.csv (alias de INDEX.csv, servi par GCS/server.js) — meme source
  // de verite que tools/gen_codes.py. Chargement async : tant qu'elle n'est
  // pas prete, on retombe sur l'hex plutot que de bloquer le rendu.
  var CODES_TABLE = new Map();
  (function loadCodesTable() {
    var COMPONENTS = { FC: 1, SF: 2, NV: 3, MC: 4, SM: 5, MV: 6, GW: 7 };
    var CATEGORIES = { P: 1, S: 2, A: 3, C: 4, O: 5 };
    fetch('/codes.csv').then(function (r) { return r.text(); }).then(function (text) {
      var lines = text.split(/\r?\n/).filter(function (l) { return l.trim().length > 0; });
      for (var i = 1; i < lines.length; i++) {
        var cols = lines[i].split(';');
        var code = cols[0], name = cols[3], severity = (cols[5] || '').trim();
        var sound = (cols[6] || '').trim(); // colonne Sound, cf. codes_common.py
        var m = code && code.match(/^([A-Z]{2})([A-Z])(\d{3})$/);
        if (!m) continue;
        var packed = ((COMPONENTS[m[1]] || 0) << 24) | ((CATEGORIES[m[2]] || 0) << 16) |
                     parseInt(m[3], 10);
        CODES_TABLE.set(packed >>> 0, { code: code, name: name, severity: severity, sound: sound });
      }
    }).catch(function () { /* pas grave : affichage en hex en repli */ });
  })();

  function lookupCode(packed) {
    return CODES_TABLE.get(packed >>> 0) || null;
  }

  function codeLabel(packed) {
    var info = lookupCode(packed);
    return info ? (info.code + ' ' + info.name) : codeHex(packed);
  }

  // Criticity (INDEX.csv) -> categories attendues par render.js::sevColor().
  var SEVERITY_MAP = { Emergency: 'CRITICAL', Critical: 'CRITICAL', Warning: 'CAUTION', Info: 'NOTICE' };

  // ComponentID (types.hpp) -> nom affiche. GlobalWatchdog exclu : pas
  // pertinent pour ce panneau (cf. kHealthCycle cote SystemMonitoring.cpp).
  var COMPONENT_NAMES = {
    0: 'MAVLINK IF', 1: 'SENSOR FUSION', 2: 'NAVIGATION',
    3: 'MISSION CONTROL', 4: 'SYS MONITORING'
  };
  // ComponentHealth (types.hpp) — ordre exact, ne pas reordonner.
  var HEALTH_NAMES = ['IDLE', 'NOMINAL', 'DEAD', 'SICK', 'DEGRADED'];

  // HEALTH ne porte qu'un composant par paquet (cf. SystemMonitoring.cpp
  // buildHealthPkt, defilement) — on maintient donc un upsert par id plutot
  // que de remplacer toute la liste a chaque reception.
  var componentIndexById = {};

  GCS.applyPacket = function applyPacket(state, pkt) {
    var p = pkt.payload || {};

    switch (pkt.id) {

      // GPSFIX/GPSPOS/SAT_INFO sont trois paquets distincts cote
      // TELEMETRY.csv (pas un seul GPSFIX fusionne) — cf. SYSMONITORING.md.
      case 'GPSFIX':
        state.gnss.fixType    = p.fix_type;
        state.gnss.satUsed    = p.num_sats;   // pas de distinction utilises/visibles cote driver
        state.gnss.satVisible = p.num_sats;
        state.gnss.hdop       = p.hdop;
        GCS.touch(state, 'gnss');
        break;

      case 'GPSPOS':
        state.gnss.lat    = p.lat;            // degres decimaux
        state.gnss.lon    = p.lon;
        state.gnss.altMsl = p.alt_msl;
        state.gnss.vdop   = p.vdop;
        GCS.touch(state, 'gnss');
        break;

      case 'SAT_INFO':
        // strength (cno UBX, dBHz) sert directement de SNR — meme echelle.
        state.gnss.snr = (p.strength || []).slice(0, p.count || 0);
        GCS.touch(state, 'gnss');
        break;

      case 'ATTITUDE': {
        // Quaternion corps->NED (qw,qx,qy,qz) -> Euler ZYX, degres. Pas de
        // paquet roll/pitch/yaw direct cote drone (cf. TELEMETRY.csv).
        var qw = p.qw, qx = p.qx, qy = p.qy, qz = p.qz;
        var roll  = Math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
        var pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (qw * qy - qz * qx))));
        var yaw   = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
        state.attitude.roll  = roll  * 180 / Math.PI;
        state.attitude.pitch = pitch * 180 / Math.PI;
        state.attitude.yaw   = yaw   * 180 / Math.PI;
        GCS.touch(state, 'attitude');
        break;
      }

      case 'POS':
        state.position.north    = p.north;      // NED, metres
        state.position.east     = p.east;
        state.position.down     = p.down;
        // Pas de paquet AGL dedie : origine NED = home (HomeCalibrator),
        // donc -down est une approximation raisonnable de l'altitude sol.
        state.position.aglAlt   = -p.down;
        state.position.distHome = Math.hypot(p.north, p.east);
        GCS.touch(state, 'position');
        break;

      case 'VELNED':
        state.velocity.vn = p.vn;
        state.velocity.ve = p.ve;
        state.velocity.vd = p.vd;               // positif = descente
        state.velocity.groundSpeed = Math.hypot(p.vn, p.ve);
        GCS.touch(state, 'position');
        break;

      case 'BATTERY':
        state.battery.pct     = p.remaining_pct;
        state.battery.voltage = p.voltage;
        state.battery.current = p.current;
        // tempC/consumed_mAh/cycles/cells/secondsRemaining : pas de paquet
        // dedie cote drone actuellement, restent a 0 (cf. README.md).
        GCS.touch(state, 'battery');
        break;

      case 'LIDAR_SCAN': {
        // range_mm[i]/intensity[i] : i = gisement en degres (0=nez, sens
        // direct), pas de retour si range_mm[i]==0 (cf. SharedSFMem.hpp).
        var pts = [];
        var range = p.range_mm || [], inten = p.intensity || [];
        for (var i = 0; i < range.length; i++) {
          var mm = range[i];
          if (!mm) continue;
          var bearing = i <= 180 ? i : i - 360;      // signe, nez = 0 deg
          if (bearing < -100 || bearing > 100) continue; // FOV 200 deg avant (cf. maquette)
          pts.push({ bearingDeg: bearing, distM: mm / 1000 });
        }
        state.lidar.points = pts;
        GCS.touch(state, 'lidar');
        break;
      }

      case 'MISSION_STATE':
        state.mission.state = MISSION_STATE_NAMES[p.state] || 'IDLE';
        GCS.touch(state, 'mission');
        break;

      case 'AUTOPILOT_MODE':
        state.mission.lateralMode.engaged  = codeHex(p.lateral_engaged);
        state.mission.lateralMode.armed    = codeHex(p.lateral_armed);
        state.mission.verticalMode.engaged = codeHex(p.vertical_engaged);
        state.mission.verticalMode.armed   = codeHex(p.vertical_armed);
        GCS.touch(state, 'mission');
        break;

      case 'ARM_STATE':
        // Gap connu : MavlinkInterface ne parse pas encore
        // MAV_MODE_FLAG_SAFETY_ARMED depuis HEARTBEAT -> ce paquet reste a
        // valeurs par defaut cote drone pour l'instant (armed toujours 0).
        state.mission.armed = !!p.armed;
        GCS.touch(state, 'mission');
        break;

      case 'CPU_TEMP':
        state.sys.cpuTempC = p.temp_c;
        GCS.touch(state, 'sys');
        break;

      case 'HEALTH': {
        var name = COMPONENT_NAMES[p.component];
        if (name === undefined) break; // composant hors GCS (ex: GlobalWatchdog) : ignore
        var idx = componentIndexById[p.component];
        if (idx === undefined) {
          idx = state.components.length;
          componentIndexById[p.component] = idx;
          // hz/note : pas de donnee cote drone (pas de paquet de charge par
          // Task) -- hz reste null (affiche '--' cote render.js, pas "0 Hz").
          state.components.push({ name: name, hz: null, note: '', health: 'IDLE' });
        }
        state.components[idx].health = HEALTH_NAMES[p.health] || 'IDLE';
        GCS.touch(state, 'components');
        break;
      }

      case 'RADIO':
        // Aucun paquet TELEM::RADIO n'existe : RSSI/RTT/perte sont mesures
        // localement par le recepteur du GCS (futur H562), pas transmis par
        // le drone. Case gardee pour le jour ou ce sera cable localement.
        state.radio = p.links || [];
        GCS.touch(state, 'radio');
        break;

      case 'CODE': {
        // Relaye uniquement les codes MAJEURS (Critical/Emergency) par
        // polling+diff cote SysMonitoring (cf. scanCodeChanges) -- p.active
        // distingue une levee d'un effacement (pas dans le paquet d'origine).
        var label = codeLabel(p.code);
        var info = lookupCode(p.code);
        var sev = SEVERITY_MAP[info ? info.severity : ''] || 'NOTICE';

        if (p.active) {
          GCS.pushEvent(state, sev, label);
          if (sev === 'CRITICAL' && !state.pinned.some(function (e) { return e.code === p.code; })) {
            state.pinned.push({ code: p.code, sev: 'CRITICAL', msg: label });
          }
        } else {
          GCS.pushEvent(state, 'NOTICE', label + ' -- efface');
          state.pinned = state.pinned.filter(function (e) { return e.code !== p.code; });
        }

        // Alarme sonore (cf. alarm.js) : notifiee sur levee ET effacement --
        // un effacement doit pouvoir arreter une alarme en boucle en cours.
        if (GCS.alarm) GCS.alarm.notify(p.code, info, !!p.active);
        break;
      }

      default:
        // Paquet inconnu : on l'ignore silencieusement plutot que de casser
        // l'affichage. Decommenter pour tracer pendant l'integration.
        // console.debug('[gcs] paquet non gere :', pkt.id);
        break;
    }
  };

  /* ── Transport ─────────────────────────────────────────────────────────── */

  GCS.link = {
    ws: null,
    url: null,
    connected: false,
    _retryMs: 500,

    /**
     * @param {string} url        ex. 'ws://raspberrypi.local:8765'
     * @param {object} state
     */
    connect: function connect(url, state) {
      this.url = url;
      var self = this;

      function open() {
        var ws = new WebSocket(url);
        self.ws = ws;

        ws.onopen = function () {
          self.connected = true;
          self._retryMs = 500;
          GCS.pushEvent(state, 'NOTICE', 'Link: WebSocket etabli sur ' + url);
        };

        ws.onmessage = function (ev) {
          // Format reel de GCS/server.js : {type:'telemetry', link, name,
          // fields, ts} — pas de transport binaire, JSON au fil (le decodage
          // TELEM::* est deja fait cote serveur, cf. telemetry-codec.js).
          var msg;
          try { msg = JSON.parse(ev.data); }
          catch (e) { return; }
          if (msg.type !== 'telemetry') return;
          GCS.applyPacket(state, { id: msg.name, payload: msg.fields });
        };

        ws.onclose = function () {
          self.connected = false;
          // Backoff exponentiel plafonne : evite de marteler un Pi qui reboote.
          self._retryMs = Math.min(self._retryMs * 2, 8000);
          setTimeout(open, self._retryMs);
        };

        ws.onerror = function () { ws.close(); };
      }

      open();
    },

    /** Commande montante (CMD_ARM, CMD_RTL, ...) — format attendu par
     * GCS/server.js : {type:'command', name, fields, link}. */
    send: function send(cmd, payload) {
      if (!this.connected || !this.ws) {
        console.warn('[gcs] lien indisponible, commande ignoree :', cmd);
        return false;
      }
      this.ws.send(JSON.stringify({ type: 'command', name: cmd, fields: payload || {}, link: 'main' }));
      return true;
    }
  };

})(window);
