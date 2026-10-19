/* ============================================================================
   render.js — projection de l'etat sur le DOM.

   Contrat : render(state, now) est une fonction PURE cote lecture. Elle ne
   modifie jamais l'etat telemetrie, elle ne fait que le lire et ecrire dans le
   DOM. Toutes les conversions d'unites et tout le formatage sont ici, nulle
   part ailleurs.

   Elle est idempotente et sans effet de bord : l'appeler deux fois de suite
   avec le meme etat ne produit aucune ecriture DOM la seconde fois (grace au
   dirty-checking de dom.js).
   ========================================================================== */

(function (global) {
  'use strict';

  var GCS = global.GCS = global.GCS || {};
  var D   = GCS.dom;

  /* ── Constantes de mise en page (design fige 1920x1080) ────────────────── */

  var STAGE_W = 1920;
  var STAGE_H = 1080;

  var PX_PER_DEG   = 13;    // HUD : pixels par degre de tangage
  var TAPE_CENTER  = 126;   // centre vertical des bandeaux vitesse/altitude (252/2)
  var HSI_CENTER   = 170;   // centre horizontal du bandeau de cap (340/2)
  var TAPE_PX_STEP = 24;    // pixels entre deux graduations
  var LIDAR_CLOSE_M = 22;   // en-deca : retour lidar signale en ambre
  var LADDER_DEG = [-20, -10, -5, 5, 10, 20];   // graduations de l'echelle de tangage

  /* Tailles max des pools. Surdimensionner ici coute quelques noeuds DOM ;
     sous-dimensionner tronque l'affichage (avertissement console). */
  var CAP = {
    satBars:      32,
    lidarPoints: 256,
    componentCells: 8,
    ribbon:        8,
    pitchLadder:   8,
    speedTicks:   13,
    altTicks:     13,
    headingTicks: 15,
    fmaColumns:    3,
    pinnedEvents:  4,
    feedEvents:   40,
    radioBars:     5
  };

  var P = {};          // pools
  var radioRefs = [];  // refs DOM des deux blocs radio, construits a l'init
  var tabEls = [];     // noeuds [data-tab] / [data-cam], caches a l'init
  var camEls = [];

  /* Tampons reutilises a chaque frame. Les listes derivees (ruban, echelle de
     tangage, colonnes FMA, graduations) ont une taille fixe : on mute des
     objets existants plutot que d'en creer 40 par seconde. */
  var SCRATCH = {
    ribbon:  [], ladder: [], fma: [],
    speed:   [], alt:    [], hdg: []
  };
  function scratch(bucket, n, init) {
    var a = SCRATCH[bucket];
    while (a.length < n) a.push(init());
    a.length = n;
    return a;
  }

  /* ── Formatage ─────────────────────────────────────────────────────────── */

  function fx(v, n)  { return (v == null || !isFinite(v)) ? '--' : v.toFixed(n); }
  function pad(v, n) { return String(Math.round(v)).padStart(n, '0'); }
  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

  function mmss(sec) {
    if (!isFinite(sec) || sec < 0) return '--:--';
    var m = Math.floor(sec / 60), s = Math.floor(sec % 60);
    return m + ':' + pad(s, 2);
  }

  function latStr(v) { return Math.abs(v).toFixed(4) + '°' + (v >= 0 ? 'N' : 'S'); }
  function lonStr(v) { return Math.abs(v).toFixed(4) + '°' + (v >= 0 ? 'E' : 'W'); }

  var FIX_LABEL = ['NO FIX', 'NO FIX', '2D FIX', '3D FIX', 'DGPS', 'RTK FLOAT', 'RTK FIX'];
  function fixLabel(t) { return FIX_LABEL[t] || 'NO FIX'; }

  /* ── Couleurs (variables CSS, jamais de hex en dur ici) ────────────────── */

  var C_OK = 'var(--ok)', C_WARN = 'var(--warn)', C_CRIT = 'var(--crit)';
  var C_ACCENT = 'var(--accent)', C_FG = 'var(--fg-hi)', C_DIM = 'var(--fg-dim)';

  function battColor(pct) {
    if (pct == null || !isFinite(pct)) return C_DIM;
    return pct >= 30 ? C_OK : pct >= 10 ? C_WARN : C_CRIT;
  }
  function healthColor(h) {
    return h === 'NOMINAL' ? C_OK
         : h === 'DEGRADED' ? C_WARN
         : h === 'IDLE' ? C_DIM
         : C_CRIT;
  }
  function sevColor(s) {
    return s === 'CRITICAL' ? C_CRIT
         : s === 'CAUTION'  ? C_WARN
         : s === 'NOTICE'   ? C_OK
         : '#6fa9b8';
  }
  function fixColor(t) { return t >= 5 ? C_OK : t >= 3 ? C_WARN : C_CRIT; }

  /* ── Bandeaux gradues (vitesse / altitude / cap) ───────────────────────── */

  /**
   * Calcule les graduations autour d'une valeur.
   * @param value  valeur courante
   * @param step   pas entre graduations
   * @param half   nombre de graduations de chaque cote
   * @param center coordonnee (px) de la graduation correspondant a value
   * @param wrap   true pour un cap (0-360, avec cardinaux)
   */
  function tape(bucket, value, step, half, center, wrap) {
    var nearest = Math.round(value / step) * step;
    var out = scratch(bucket, 2 * half + 1, function () { return { label: '', px: 0 }; });
    for (var i = -half; i <= half; i++) {
      var tv = nearest + i * step;
      var delta = tv - value;
      if (wrap) {
        delta = ((delta + 180) % 360 + 360) % 360 - 180;
        tv = ((tv % 360) + 360) % 360;
      }
      var slot = out[i + half];
      slot.px = center - delta * (TAPE_PX_STEP / step);
      if (wrap) {
        var card = CARDINALS[Math.round(tv)];
        slot.label = card || pad(tv / 10, 2);
      } else {
        slot.label = String(Math.round(tv));
      }
    }
    return out;
  }

  var CARDINALS = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' };

  /* ── Initialisation des pools + cablage des interactions ───────────────── */

  /**
   * @param {object}   state
   * @param {Function} onCommand  (cmdName, state) => void — point d'entree des
   *                              commandes montantes (ARM, RTL, ...). C'est ici
   *                              que tu brancheras l'emission WebSocket.
   */
  GCS.initRender = function initRender(state, onCommand) {
    D.collect(document);
    onCommand = onCommand || function () {};

    /* --- pools simples --- */

    P.satBars = D.pool('satBars', CAP.satBars, function () {
      return D.make('div', 'gnss__bar');
    });

    P.lidarPoints = D.pool('lidarPoints', CAP.lidarPoints, function () {
      return D.make('div', 'lidar__pt');
    });

    P.componentCells = D.pool('componentCells', CAP.componentCells, function () {
      var cell = D.make('div', 'comp__cell');
      var top  = D.make('div', 'comp__cell-top');
      var name = D.make('div', 'comp__cell-name');
      var hz   = D.make('div', 'comp__cell-hz');
      var note = D.make('div', 'comp__cell-note');
      top.appendChild(name); top.appendChild(hz);
      cell.appendChild(top); cell.appendChild(note);
      cell.__name = name; cell.__hz = hz; cell.__note = note;
      return cell;
    });

    P.ribbon = D.pool('ribbon', CAP.ribbon, function () {
      var box   = D.make('div');
      var label = D.make('div', 'ribbon__label');
      var value = D.make('div', 'ribbon__value');
      var num   = D.make('span');
      var unit  = D.make('span', 'ribbon__unit');
      value.appendChild(num);
      value.appendChild(document.createTextNode(' '));
      value.appendChild(unit);
      box.appendChild(label); box.appendChild(value);
      box.__label = label; box.__value = value; box.__num = num; box.__unit = unit;
      return box;
    });

    P.pitchLadder = D.pool('pitchLadder', CAP.pitchLadder, function () {
      var row = D.make('div', 'hud__ladder');
      var l   = D.make('span', 'hud__ladder-num hud__ladder-num--l');
      var bl  = D.make('div', 'hud__ladder-bar');
      var gap = D.make('div', 'hud__ladder-gap');
      var br  = D.make('div', 'hud__ladder-bar');
      var r   = D.make('span', 'hud__ladder-num');
      [l, bl, gap, br, r].forEach(function (n) { row.appendChild(n); });
      row.__l = l; row.__r = r; row.__bl = bl; row.__br = br;
      return row;
    });

    P.speedTicks   = D.pool('speedTicks',   CAP.speedTicks,   function () { return D.make('div', 'tape__tick'); });
    P.altTicks     = D.pool('altTicks',     CAP.altTicks,     function () { return D.make('div', 'tape__tick'); });
    P.headingTicks = D.pool('headingTicks', CAP.headingTicks, function () { return D.make('div', 'hsi__tick'); });

    P.fmaColumns = D.pool('fmaColumns', CAP.fmaColumns, function () {
      var col = D.make('div', 'fma__col');
      var t   = D.make('div', 't-label-s');
      var e   = D.make('div', 'fma__engaged');
      var a   = D.make('div', 'fma__armed');
      col.appendChild(t); col.appendChild(e); col.appendChild(a);
      col.__t = t; col.__e = e; col.__a = a;
      return col;
    });

    P.pinnedEvents = D.pool('pinnedEvents', CAP.pinnedEvents, function () {
      var row = D.make('div', 'log__pinned-row');
      var sev = D.make('div', 'log__pinned-sev');
      var msg = D.make('div', 'log__pinned-msg');
      row.appendChild(sev); row.appendChild(msg);
      row.__sev = sev; row.__msg = msg;
      return row;
    });

    P.feedEvents = D.pool('feedEvents', CAP.feedEvents, function () {
      var row  = D.make('div', 'log__row');
      var time = D.make('div', 'log__time');
      var sev  = D.make('div', 'log__sev');
      var msg  = D.make('div', 'log__msg');
      row.appendChild(time); row.appendChild(sev); row.appendChild(msg);
      row.__time = time; row.__sev = sev; row.__msg = msg;
      return row;
    });

    /* --- blocs radio : structure construite une fois --- */

    radioRefs = ['radio0', 'radio1'].map(function (name) {
      var host = D.el(name);
      if (!host) return null;
      host.textContent = '';

      var id    = D.make('div', 'radio__id');
      var dot   = D.make('div', 'radio__dot');
      var idTxt = D.make('div');
      var nm    = D.make('div', 'radio__name');
      var role  = D.make('div', 'radio__role');
      idTxt.appendChild(nm); idTxt.appendChild(role);
      id.appendChild(dot); id.appendChild(idTxt);

      var barsHost = D.make('div', 'radio__bars');
      var bars = [];
      for (var i = 0; i < CAP.radioBars; i++) {
        var b = D.make('div', 'radio__bar');
        barsHost.appendChild(b);
        bars.push(b);
      }

      var kpis = D.make('div', 'radio__kpis');
      var vals = {};
      ['RSSI', 'RTT', 'LOSS', 'DOWN', 'UP'].forEach(function (k) {
        var cell = D.make('div');
        cell.appendChild(D.make('div', 'radio__kpi-label', k));
        var v = D.make('div', 't-num-xs');
        cell.appendChild(v);
        kpis.appendChild(cell);
        vals[k] = v;
      });

      var band = D.make('div', 'radio__band');

      host.appendChild(id);
      host.appendChild(barsHost);
      host.appendChild(kpis);
      host.appendChild(band);

      return { host: host, dot: dot, name: nm, role: role, bars: bars, vals: vals, band: band };
    });

    /* --- interactions --- */

    tabEls = Array.prototype.slice.call(document.querySelectorAll('[data-tab]'));
    camEls = Array.prototype.slice.call(document.querySelectorAll('[data-cam]'));

    tabEls.forEach(function (el) {
      el.addEventListener('click', function () { state.ui.tab = el.dataset.tab; });
    });
    camEls.forEach(function (el) {
      el.addEventListener('click', function () { state.ui.camMode = el.dataset.cam; });
    });

    document.querySelectorAll('[data-cmd]').forEach(function (el) {
      el.addEventListener('click', function () { onCommand(el.dataset.cmd, state); });
    });

    /* --- mise a l'echelle de la scene --- */

    function fitStage() {
      var s = Math.min(global.innerWidth / STAGE_W, global.innerHeight / STAGE_H);
      state.ui.stageScale = s;
      state.ui.stageOx = (global.innerWidth  - STAGE_W * s) / 2;
      state.ui.stageOy = (global.innerHeight - STAGE_H * s) / 2;
    }
    global.addEventListener('resize', fitStage);
    fitStage();
  };

  /* ── Sous-rendus ───────────────────────────────────────────────────────── */

  function renderStage(s) {
    D.style('stage', 'transform',
      'translate(' + s.ui.stageOx.toFixed(1) + 'px,' + s.ui.stageOy.toFixed(1) + 'px)' +
      ' scale(' + s.ui.stageScale.toFixed(4) + ')');
  }

  function renderGnss(s) {
    var g = s.gnss;
    D.cls('panelGnss', 'is-stale', s.stale.gnss);

    D.text('gnssFixLabel', fixLabel(g.fixType));
    D.style('gnssFixLabel', 'color', fixColor(g.fixType));
    D.style('gnssFixLabel', 'border-color', fixColor(g.fixType));

    D.text('satCount', g.satUsed + ' / ' + g.satVisible);
    D.text('hdop', fx(g.hdop, 1));
    D.text('vdop', fx(g.vdop, 1));
    D.text('lat', latStr(g.lat));
    D.text('lon', lonStr(g.lon));
    D.text('gpsAlt', fx(g.altMsl, 0) + ' m');

    D.applyPool(P.satBars, g.snr, function (el, snr) {
      var h = clamp(snr / 50 * 100, 8, 100);
      D.nodeStyle(el, 'height', h.toFixed(0) + '%');
      D.nodeStyle(el, 'background', snr >= 30 ? C_OK : C_WARN);
    });
  }

  function renderLidar(s) {
    var l = s.lidar;
    D.cls('panelLidar', 'is-stale', s.stale.lidar);

    D.text('lidarCount', 'FOV 200° · ' + l.points.length + ' RET');
    D.text('lidarRing1', (l.maxRange / 3).toFixed(0) + ' m');
    D.text('lidarRing2', (l.maxRange * 2 / 3).toFixed(0) + ' m');
    D.text('lidarRing3', l.maxRange.toFixed(0) + ' m');

    var min = Infinity;
    for (var i = 0; i < l.points.length; i++) {
      if (l.points[i].distM < min) min = l.points[i].distM;
    }
    var hasMin = isFinite(min);
    D.text('lidarMin', 'MIN ' + (hasMin ? min.toFixed(0) : '--') + ' m');
    D.style('lidarMin', 'color', hasMin && min < 12 ? C_WARN : C_OK);

    D.applyPool(P.lidarPoints, l.points, function (el, pt) {
      var r = clamp(pt.distM / l.maxRange, 0, 1) * 45;  // rayon en unites viewBox (0-100)
      var a = pt.bearingDeg * Math.PI / 180;
      var size = pt.distM < LIDAR_CLOSE_M ? 3.4 : 2.6;
      D.nodeStyle(el, 'left', (50 + Math.sin(a) * r).toFixed(2) + '%');
      D.nodeStyle(el, 'top',  (50 - Math.cos(a) * r).toFixed(2) + '%');
      D.nodeStyle(el, 'width',  size + 'px');
      D.nodeStyle(el, 'height', size + 'px');
      D.nodeStyle(el, 'background', pt.distM < LIDAR_CLOSE_M
        ? 'rgba(255,180,67,.9)'
        : 'rgba(79,216,234,' + (0.9 - 0.45 * (pt.distM / l.maxRange)).toFixed(2) + ')');
    });
  }

  function renderEndurance(s) {
    var b = s.battery;
    D.cls('endur', 'is-stale', s.stale.battery);

    D.text('endurPct', fx(b.pct, 0));
    D.style('endurPct', 'color', battColor(b.pct));

    // Deux estimations non cablees : affichees '--' tant que le calcul n'est
    // pas tranche (capacite du pack, reserve de retour) -- cf. §5 du
    // cahier des charges. Jamais 0, qui se lirait comme une mesure.
    D.text('endurTime', fx(b.minutesRemaining, 0));
    D.text('endurDist', fx(b.kmRemaining, 1));

    D.text('endurVolt', fx(b.voltage, 1) + ' V');
    D.text('endurAmp', fx(b.current, 1) + ' A');
  }

  /* ── Schema drone ──────────────────────────────────────────────────────
     Deux sources par forme : une panne (code de diagnostic, colonne
     Schematic d'INDEX.csv) ou une mesure continue. La panne l'emporte
     toujours -- une batterie a 80 % reste en defaut si un code le dit.
     Toute forme part de 'nodata' : sans donnee, jamais 'bon'. */

  var SCHEME_PARTS = ['batt', 'gps', 'cpu', 'fc', 'lidar', 'mag', 'esc',
                      'ant_main', 'ant_backup', 'motor_front', 'motor_rear'];
  var SCHEME_CLASSES = ['is-ok', 'is-warn', 'is-crit', 'is-nodata', 'is-cont'];

  // Seuils CPU : le Pi 5 commence a se brider vers 80-85 degres.
  var CPU_WARN = 72, CPU_CRIT = 82;
  // Ecart au regime moyen au-dela duquel --dev vaut 1.
  var MOTOR_DEV_MAX = 0.12;

  function setPart(name, level, blink) {
    var el = D.el(name);
    if (!el) return;
    for (var i = 0; i < SCHEME_CLASSES.length; i++) {
      var want = SCHEME_CLASSES[i] === ('is-' + level);
      D.nodeClass(el, SCHEME_CLASSES[i], want);
    }
    D.nodeClass(el, 'is-blink', !!blink);
  }

  // La panne ecrase la mesure, sauf si la mesure est elle-meme plus grave.
  function merge(fault, measured) {
    if (fault === 'crit') return 'crit';
    if (measured === 'crit') return 'crit';
    if (fault === 'warn' || measured === 'warn') return 'warn';
    return measured || (fault ? 'warn' : 'nodata');
  }

  function byThreshold(v, warnAt, critAt) {
    if (v == null || !isFinite(v)) return null;
    return v >= critAt ? 'crit' : v >= warnAt ? 'warn' : 'ok';
  }

  function renderScheme(s) {
    var f = s.hw.faults || {};
    var worst = 'ok', nodata = 0;

    // Batterie : seuils du cahier des charges (>30 / 10-30 / <10).
    var pct = s.battery.pct;
    var battM = (pct == null) ? null : (pct < 10 ? 'crit' : pct < 30 ? 'warn' : 'ok');
    var battLvl = merge(f.BATTERY, battM);
    setPart('batt', battLvl, battLvl === 'crit');
    D.style('batt', '--fill', (pct == null ? 0 : clamp(pct, 0, 100) / 100).toFixed(3));

    // GPS : un fix 3D ne suffit pas, le HDOP decide du degrade.
    var g = s.gnss;
    var gpsM = (g.fixType == null || s.stale.gnss) ? null
             : g.fixType < 3 ? 'crit'
             : (g.hdop > 2.0 || g.satUsed < 7) ? 'warn' : 'ok';
    setPart('gps', merge(f.GPS, gpsM), false);

    var cpuM = s.stale.sys ? null : byThreshold(s.sys.cpuTempC, CPU_WARN, CPU_CRIT);
    setPart('cpu', merge(f.CPU, cpuM), false);
    D.text('cpu_temp', s.sys.cpuTempC == null ? '--' : fx(s.sys.cpuTempC, 0) + ' °C');

    // FC : sante du composant MavlinkInterface, seule fenetre sur le FC.
    var mv = null;
    for (var i = 0; i < s.components.length; i++) {
      if (s.components[i].name === 'MAVLINK IF') { mv = s.components[i].health; break; }
    }
    var fcM = (mv == null || mv === 'IDLE') ? null
            : mv === 'NOMINAL' ? 'ok' : mv === 'DEGRADED' ? 'warn' : 'crit';
    setPart('fc', merge(f.FC, fcM), mv === 'DEAD');

    var lidarM = s.stale.lidar ? null : 'ok';
    setPart('lidar', merge(f.LIDAR, lidarM), false);

    setPart('mag', merge(f.MAG, null), false);

    // ESC : pas de temperature transmise, donc seuls les codes parlent.
    setPart('esc', merge(f.ESC, null), false);
    for (var e = 1; e <= 4; e++) setPart('esc_' + e, 'nodata', false);

    setPart('ant_main', merge(f.ANT_PDL, null), false);
    setPart('ant_backup', merge(f.ANT_SDL, null), false);

    renderMotors(s, f);

    // Synthese : le pire etat de toutes les formes, et le compte des inconnues.
    for (var k = 0; k < SCHEME_PARTS.length; k++) {
      var el = D.el(SCHEME_PARTS[k]);
      if (!el) continue;
      if (el.classList.contains('is-crit')) worst = 'crit';
      else if (el.classList.contains('is-warn') && worst !== 'crit') worst = 'warn';
      if (el.classList.contains('is-nodata')) nodata++;
    }

    var label = nodata === SCHEME_PARTS.length ? 'SANS DONNÉE'
              : worst === 'crit' ? 'DÉFAUT'
              : worst === 'warn' ? 'ATTENTION'
              : nodata > 0 ? nodata + ' SANS DONNÉE' : 'NOMINAL';
    var badgeLvl = nodata === SCHEME_PARTS.length ? 'nodata' : worst;
    setPart('scheme_badge', badgeLvl, false);
    D.text('scheme_badge', label);
  }

  // Le regime absolu suit les gaz : ce qui informe est l'ecart au regime
  // moyen des quatre, cf. SYSMONITORING.md §14.
  function renderMotors(s, f) {
    var rpm = s.hw.rpm;
    var pairs = [['motor_front', 'motor_front_dev'], ['motor_rear', 'motor_rear_dev']];

    if (!rpm || s.stale.hw) {
      for (var i = 0; i < 2; i++) {
        setPart(pairs[i][0], merge(f[i ? 'MOTOR_REAR' : 'MOTOR_FRONT'], null), false);
        D.text(pairs[i][1], '--');
      }
      return;
    }

    var mean = (rpm[0] + rpm[1] + rpm[2] + rpm[3]) / 4;
    // Regroupement AV/AR des 4 sorties FC : quad en X, sorties 1 et 3 a
    // l'avant, 2 et 4 a l'arriere (numerotation ArduPilot, a confirmer au banc).
    var group = [(rpm[0] + rpm[2]) / 2, (rpm[1] + rpm[3]) / 2];

    for (var m = 0; m < 2; m++) {
      var dev = mean > 0 ? (group[m] - mean) / mean : 0;
      var fault = f[m ? 'MOTOR_REAR' : 'MOTOR_FRONT'];
      var el = D.el(pairs[m][0]);

      if (fault) {
        setPart(pairs[m][0], fault, fault === 'crit');
      } else {
        setPart(pairs[m][0], 'cont', false);
        if (el) D.nodeStyle(el, '--dev',
          Math.min(1, Math.abs(dev) / MOTOR_DEV_MAX).toFixed(3));
      }
      D.text(pairs[m][1], (dev >= 0 ? '+' : '\u2212') +
                          Math.abs(dev * 100).toFixed(1) + ' %');
    }
  }

  function renderActions(s) {
    var m = s.mission;
    var col = m.armed ? C_OK : 'rgba(230,241,244,.6)';
    D.text('armLabel', m.armed ? 'ARMED' : 'DISARMED');
    D.style('armLabel', 'color', col);
    D.text('armBtnLabel', m.armed ? 'DISARM MOTORS' : 'ARM MOTORS');
    D.style('armBtn', 'color', col);
    D.style('armBtn', 'border-color', m.armed ? 'rgba(53,214,124,.45)' : 'rgba(255,255,255,.18)');
    D.style('armBtn', 'background', m.armed ? 'rgba(53,214,124,.1)' : 'rgba(255,255,255,.04)');
    D.text('planFooter', 'PLAN · ' + m.planName + ' · ' + m.wpCount + ' WP · ' + m.state);
  }

  function renderComponents(s) {
    var okCount = 0;
    for (var i = 0; i < s.components.length; i++) {
      if (s.components[i].health === 'NOMINAL') okCount++;
    }
    D.text('compSummary', okCount + ' / ' + s.components.length + ' OK');
    D.style('compSummary', 'color', okCount === s.components.length ? C_OK : C_WARN);

    D.applyPool(P.componentCells, s.components, function (el, c) {
      var col = healthColor(c.health);
      D.nodeText(el.__name, c.name);
      D.nodeText(el.__hz, fx(c.hz, 0) + ' Hz');
      D.nodeStyle(el.__hz, 'color', col);
      D.nodeText(el.__note, c.note || '');
      D.nodeStyle(el, 'border-left-color', col);
      D.nodeStyle(el, 'background', c.health === 'NOMINAL'
        ? 'rgba(53,214,124,.05)' : 'rgba(255,180,67,.07)');
      D.nodeClass(el, 'is-stale', s.stale.components);
    });
  }

  function renderTabs(s) {
    var is3d = s.ui.tab === '3d';
    for (var i = 0; i < tabEls.length; i++) {
      D.nodeClass(tabEls[i], 'is-active', tabEls[i].dataset.tab === s.ui.tab);
    }
    for (var j = 0; j < camEls.length; j++) {
      D.nodeClass(camEls[j], 'is-active', camEls[j].dataset.cam === s.ui.camMode);
    }
    D.show('view3d', is3d);
    D.show('viewCam', !is3d);
    D.text('feedLabel', is3d
      ? 'MISSION RENDER · LOCAL NED FRAME'
      : 'EO/IR NOSE CAM · 640 × 480 · 30 FPS · H.265');
  }

  function renderViewport3d(s) {
    /* Le custom element relit ses attributs a chaque frame : on se contente
       de les tenir a jour, il gere son propre rAF interne. */
    D.attr('viewport3d', 'frac', s.mission.pathFrac.toFixed(4));
    D.attr('viewport3d', 'cam-mode', s.ui.camMode);

    var m = s.mission;
    D.text('planCardName', m.planName + ' · LEG ' + (m.legIndex + 1) + '/' + Math.max(1, m.wpCount));
    D.text('planCardNext', 'NEXT WP' + m.nextWpIndex + ' · ' + fx(m.nextWpDistM, 0) +
                           ' m · ETA ' + mmss(m.etaSec));
  }

  function renderRibbon(s) {
    var gs   = s.velocity.groundSpeed;
    var vs   = -s.velocity.vd;                 // NED : vd>0 = descente
    var hdg  = ((s.attitude.yaw % 360) + 360) % 360;

    var items = scratch('ribbon', 7, function () { return { label: '', value: '', unit: '', color: C_FG }; });
    function set(i, label, value, unit, color) {
      items[i].label = label; items[i].value = value;
      items[i].unit = unit;   items[i].color = color;
    }
    set(0, 'Altitude AGL', fx(s.position.aglAlt, 0), 'M', C_ACCENT);
    set(1, 'Ground Speed', fx(gs, 1), 'M/S', C_FG);
    set(2, 'Heading', pad(hdg, 3), '°', C_FG);
    set(3, 'Vert Speed', (vs >= 0 ? '+' : '') + fx(vs, 1), 'M/S', C_FG);
    set(4, 'Dist Home', fx(s.position.distHome, 0), 'M', C_FG);
    set(5, 'Next WP', 'WP' + s.mission.nextWpIndex, fx(s.mission.nextWpDistM, 0) + ' M', C_FG);
    set(6, 'Mode', s.mission.state, s.mission.armed ? 'ARMED' : 'SAFE',
        s.mission.armed ? C_OK : C_DIM);

    D.applyPool(P.ribbon, items, function (el, it) {
      D.nodeText(el.__label, it.label);
      D.nodeText(el.__num, it.value);
      D.nodeText(el.__unit, it.unit);
      D.nodeStyle(el.__value, 'color', it.color);
    });
  }

  function renderHud(s) {
    var att = s.attitude;
    var gs  = s.velocity.groundSpeed;
    var vs  = -s.velocity.vd;
    var hdg = ((att.yaw % 360) + 360) % 360;

    D.style('hudWorld', 'transform', 'rotate(' + (-att.roll).toFixed(2) + 'deg)');
    D.style('hudHorizon', 'margin-top', (att.pitch * PX_PER_DEG).toFixed(1) + 'px');

    var ladder = scratch('ladder', LADDER_DEG.length, function () { return { label: '', dy: 0, w: 0 }; });
    for (var i = 0; i < LADDER_DEG.length; i++) {
      var v = LADDER_DEG[i];
      ladder[i].label = String(Math.abs(v));
      ladder[i].dy = (att.pitch - v) * PX_PER_DEG;
      ladder[i].w = Math.abs(v) >= 20 ? 44 : Math.abs(v) >= 10 ? 32 : 22;
    }
    D.applyPool(P.pitchLadder, ladder, function (el, it) {
      D.nodeStyle(el, 'margin-top', it.dy.toFixed(1) + 'px');
      D.nodeText(el.__l, it.label);
      D.nodeText(el.__r, it.label);
      D.nodeStyle(el.__bl, 'width', it.w + 'px');
      D.nodeStyle(el.__br, 'width', it.w + 'px');
    });

    var fma = scratch('fma', 3, function () { return { title: '', mode: null }; });
    fma[0].title = 'Lateral';  fma[0].mode = s.mission.lateralMode;
    fma[1].title = 'Vertical'; fma[1].mode = s.mission.verticalMode;
    fma[2].title = 'Speed';    fma[2].mode = s.mission.speedMode;
    D.applyPool(P.fmaColumns, fma, function (el, it) {
      D.nodeText(el.__t, it.title);
      D.nodeText(el.__e, it.mode.engaged);
      D.nodeText(el.__a, it.mode.armed);
    });

    D.applyPool(P.speedTicks, tape('speed', gs, 5, 4, TAPE_CENTER, false), function (el, t) {
      D.nodeText(el, t.label);
      D.nodeStyle(el, 'top', t.px.toFixed(1) + 'px');
    });
    D.applyPool(P.altTicks, tape('alt', s.position.aglAlt, 10, 4, TAPE_CENTER, false), function (el, t) {
      D.nodeText(el, t.label);
      D.nodeStyle(el, 'top', t.px.toFixed(1) + 'px');
    });
    D.applyPool(P.headingTicks, tape('hdg', hdg, 10, 5, HSI_CENTER, true), function (el, t) {
      D.nodeText(el, t.label);
      D.nodeStyle(el, 'left', t.px.toFixed(1) + 'px');
    });

    D.text('airspeedStr', fx(gs, 1));
    D.text('altitudeStr', fx(s.position.aglAlt, 0));
    D.text('headingStr', pad(hdg, 3));
    D.text('vspeedStr', (vs >= 0 ? '+' : '') + fx(vs, 1));
    D.style('vsMarker', 'top', (50 - clamp(vs * 16, -42, 42)).toFixed(1) + '%');

    D.text('camStatus', 'CAM ' + fx(s.camera.gimbalPitch, 0) + '° / ' +
                        pad(((s.camera.gimbalYaw % 360) + 360) % 360, 3) + '° · ZOOM ' +
                        fx(s.camera.zoom, 1) + 'x');
    D.text('hudNavWp', 'WP' + s.mission.nextWpIndex + ' · ' + fx(s.mission.nextWpDistM, 0) +
                       ' m · ' + mmss(s.mission.etaSec));
    D.text('hudNavHome', 'HOME ' + fx(s.position.distHome, 0) + ' m');
  }

  function renderRadio(s) {
    for (var i = 0; i < radioRefs.length; i++) {
      var ref = radioRefs[i];
      if (!ref) continue;
      var lk = s.radio[i];
      D.nodeShow(ref.host, !!lk);
      if (!lk) continue;

      var col = lk.active ? C_OK : C_WARN;
      D.nodeClass(ref.host, 'is-stale', s.stale.radio);
      D.nodeStyle(ref.dot, 'background', col);
      D.nodeText(ref.name, lk.name);
      D.nodeText(ref.role, lk.role);
      D.nodeStyle(ref.role, 'color', col);

      for (var b = 0; b < ref.bars.length; b++) {
        var lit = (b + 1) / ref.bars.length <= lk.quality + 1e-6;
        D.nodeStyle(ref.bars[b], 'height', (20 + 16 * b) + '%');
        D.nodeStyle(ref.bars[b], 'background', lit ? col : 'rgba(255,255,255,.12)');
      }

      D.nodeText(ref.vals.RSSI, fx(lk.rssi, 0));
      D.nodeText(ref.vals.RTT,  fx(lk.rttMs, 0) + 'ms');
      D.nodeText(ref.vals.LOSS, fx(lk.lossPct, 1) + '%');
      D.nodeStyle(ref.vals.LOSS, 'color', lk.lossPct < 1 ? C_OK : lk.lossPct < 5 ? C_WARN : C_CRIT);
      D.nodeText(ref.vals.DOWN, fx(lk.downMbps, 2));
      D.nodeText(ref.vals.UP,   fx(lk.upMbps, 2));
      D.nodeText(ref.band, lk.band);
    }
  }

  function renderEvents(s) {
    D.text('pinnedHeader', s.pinned.length
      ? s.pinned.length + ' ACTIVE CRITICAL'
      : 'NO ACTIVE FAULT');
    D.style('pinnedHeader', 'color', s.pinned.length ? C_CRIT : C_OK);

    D.applyPool(P.pinnedEvents, s.pinned, function (el, e) {
      D.nodeText(el.__sev, e.sev);
      D.nodeStyle(el.__sev, 'color', sevColor(e.sev));
      D.nodeText(el.__msg, e.msg);
    });

    D.applyPool(P.feedEvents, s.events, function (el, e) {
      D.nodeText(el.__time, e.timeStr);
      D.nodeText(el.__sev, e.sev);
      D.nodeStyle(el.__sev, 'color', sevColor(e.sev));
      D.nodeStyle(el, 'border-left-color', sevColor(e.sev));
      D.nodeText(el.__msg, e.msg);
    });
  }

  /* ── Point d'entree ────────────────────────────────────────────────────── */

  GCS.render = function render(state) {
    renderStage(state);
    renderGnss(state);
    renderLidar(state);
    renderEndurance(state);
    renderScheme(state);
    renderActions(state);
    renderComponents(state);
    renderTabs(state);
    renderRadio(state);
    renderEvents(state);

    /* Les deux vues centrales sont exclusives : on ne calcule que celle qui
       est visible. Le HUD represente la moitie du cout de rendu. */
    if (state.ui.tab === '3d') {
      renderViewport3d(state);
      renderRibbon(state);
    } else {
      renderHud(state);
    }
  };

})(window);
