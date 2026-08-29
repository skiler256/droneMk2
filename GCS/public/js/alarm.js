/* ============================================================================
   alarm.js — alarme sonore façon cockpit, N voix définies dans ALARMS.csv
   (éditable via tools/codes_editor.py, onglet "Alarmes").

   Deux sources d'alarme :
     - Un code CODE actif (cf. link.js) -> résout vers INDEX.csv::Alarm si
       renseigné, sinon repli générique par catégorie (master_warning si le
       voyant appartient à MW_GROUP -- même règle que server.js pour les
       voyants ESP32 -- sinon master_caution). Type Loop tant que le code
       reste actif, Oneshot pour un simple événement.
     - Une transition MISSION_STATE -> résout via MISSION_ALARMS.csv
       (ex: HOLD -> hold_entered), jouée une fois à l'entrée dans l'état.

   Un seul canal audio (haut-parleur unique) : la plus prioritaire ACTIVE
   joue, préempte IMMEDIATEMENT (à chaque tick, pas seulement quand le canal
   se libère) toute alarme moins prioritaire en cours -- jamais coincée
   derrière une file d'attente.

   Accusé de réception : MW_ACK/MC_ACK (boutons ESP32, cf. link.js) coupent
   toutes les alarmes actives de leur classe (ALARMS.csv::Class), jusqu'à ce
   qu'une NOUVELLE occurrence de cette classe apparaisse (réarme
   automatiquement, sémantique cockpit standard) -- état géré ICI,
   indépendamment de l'ack des voyants (server.js) qui suit ses propres
   règles (cf. server.js::ackedMW/ackedMC).

   Fichiers son : GCS/public/sounds/<SoundFile>.{mp3,wav,mp4} -- un seul des
   trois formats suffit par voix.
   ========================================================================== */

(function (global) {
  'use strict';

  var GCS = global.GCS = global.GCS || {};

  var SCHEDULER_TICK_MS = 200;

  // Même règle que server.js::MW_GROUP -- dupliquée volontairement (table
  // stable et minuscule, même pattern que COMPONENTS/CATEGORIES ailleurs
  // dans ce projet) : un code sans Alarm dédiée retombe sur master_warning
  // si un de ses voyants Panel est dans ce groupe, sinon master_caution.
  var MW_GROUP = new Set(['ENG', 'PWR', 'FC', 'EKF', 'MAG', 'GPS', 'NAV', 'EMG']);

  // ── Chargement ALARMS.csv / MISSION_ALARMS.csv ──────────────────────────
  var ALARMS_TABLE = new Map();       // alarmId -> {label, soundFile, type, priority, periodMs, klass}
  var MISSION_ALARMS_TABLE = new Map(); // state -> alarmId

  function loadCsv(url, onRow) {
    return fetch(url).then(function (r) { return r.text(); }).then(function (text) {
      var lines = text.split(/\r?\n/).filter(function (l) { return l.trim().length > 0; });
      for (var i = 1; i < lines.length; i++) onRow(lines[i].split(';'));
    }).catch(function () { /* pas grave : pas d'alarme tant que non chargé */ });
  }

  var tablesReady = Promise.all([
    loadCsv('/alarms.csv', function (cols) {
      var alarmId = (cols[0] || '').trim().toLowerCase();
      if (!alarmId) return;
      ALARMS_TABLE.set(alarmId, {
        label: cols[1], soundFile: cols[2], type: cols[3],
        priority: parseInt(cols[4], 10) || 0, periodMs: parseInt(cols[5], 10) || 0,
        klass: (cols[6] || '').trim().toUpperCase(),
      });
    }),
    loadCsv('/mission_alarms.csv', function (cols) {
      var state = (cols[0] || '').trim().toUpperCase();
      if (!state) return;
      MISSION_ALARMS_TABLE.set(state, (cols[1] || '').trim().toLowerCase());
    }),
  ]);

  // ── Audio : un <audio> par AlarmId, créé à la demande ───────────────────
  var SOUND_BASE = '/sounds/';
  var SOUND_FORMATS = [
    { ext: 'mp3', type: 'audio/mpeg' }, { ext: 'wav', type: 'audio/wav' },
    { ext: 'mp4', type: 'audio/mp4' },
  ];
  var audioEls = new Map(); // alarmId -> HTMLAudioElement

  function getAudioEl(alarmId, def) {
    var el = audioEls.get(alarmId);
    if (el) return el;
    el = document.createElement('audio');
    el.preload = 'auto';
    SOUND_FORMATS.forEach(function (f) {
      var src = document.createElement('source');
      src.src = SOUND_BASE + def.soundFile + '.' + f.ext;
      src.type = f.type;
      el.appendChild(src);
    });
    el.addEventListener('error', function () {
      console.warn('[gcs] alarme ' + alarmId + ': aucun fichier lisible pour ' + def.soundFile +
        ' (depose ' + def.soundFile + '.mp3/.wav/.mp4 dans GCS/public/sounds/)');
    });
    audioEls.set(alarmId, el);
    return el;
  }

  // ── Déblocage audio (obligatoire : autoplay bloqué sans geste utilisateur) ──
  var unlocked = false;
  var unlockBadge = document.createElement('div');
  unlockBadge.style.cssText = 'position:fixed;right:.6rem;bottom:.6rem;z-index:9999;' +
    'font:11px ui-monospace,monospace;padding:.3rem .6rem;border-radius:5px;' +
    'background:#3a2a00;color:#ffb84f;border:1px solid #7a5a00;pointer-events:none;';
  document.addEventListener('DOMContentLoaded', function () {
    if (!unlocked) { unlockBadge.textContent = 'SON: cliquer pour activer'; document.body.appendChild(unlockBadge); }
  });
  ['click', 'keydown'].forEach(function (evt) {
    global.addEventListener(evt, function once() {
      unlocked = true;
      unlockBadge.remove();
      global.removeEventListener(evt, once);
    }, { once: true });
  });

  // ── État ─────────────────────────────────────────────────────────────────
  var loopSources = new Map();   // alarmId -> Set<packedCode> (refcount des codes qui l'activent)
  var lastPlayedAt = new Map();  // alarmId -> performance.now() du dernier declenchement
  var oneshotQueue = [];         // [{alarmId, since}]
  var ackedClass = { MW: false, MC: false };
  var currentlyPlaying = null;   // {alarmId, def, el} | null

  function armClass(klass) {
    if (klass === 'MW' || klass === 'MC') ackedClass[klass] = false;
  }

  function resolveAlarmId(info) {
    var alarm = (info && info.alarm) || '';
    if (alarm && ALARMS_TABLE.has(alarm)) return alarm;
    var panel = (info && info.panel) || [];
    var isMw = panel.some(function (l) { return MW_GROUP.has(l); });
    return isMw ? 'master_warning' : 'master_caution';
  }

  function queueOneshot(alarmId) {
    // Deduplique : pas la peine d'empiler plusieurs occurrences en attente
    // de la même alarme non encore jouée.
    if (oneshotQueue.some(function (q) { return q.alarmId === alarmId; })) return;
    oneshotQueue.push({ alarmId: alarmId, since: performance.now() });
  }

  function stopCurrent() {
    if (!currentlyPlaying) return;
    try { currentlyPlaying.el.pause(); currentlyPlaying.el.currentTime = 0; } catch (e) { /* deja arrete */ }
    currentlyPlaying = null;
  }

  function stopIfPlaying(alarmId) {
    if (currentlyPlaying && currentlyPlaying.alarmId === alarmId) stopCurrent();
  }

  GCS.alarm = {
    /**
     * @param {number} packedCode
     * @param {{alarm:string, panel:string[]}|null} info  résultat de link.js::lookupCode()
     * @param {boolean} active
     */
    notifyCode: function notifyCode(packedCode, info, active) {
      var alarmId = resolveAlarmId(info);
      var def = ALARMS_TABLE.get(alarmId);
      if (!def) { console.warn('[gcs] alarme "' + alarmId + '" inconnue de ALARMS.csv'); return; }

      if (def.type === 'Loop') {
        var set = loopSources.get(alarmId);
        if (active) {
          if (!set) { set = new Set(); loopSources.set(alarmId, set); }
          var isNew = !set.has(packedCode);
          set.add(packedCode);
          if (isNew) armClass(def.klass); // nouveau defaut de cette classe -> reste jamais coince derriere un ack perime
        } else if (set) {
          set.delete(packedCode);
          if (set.size === 0) { loopSources.delete(alarmId); stopIfPlaying(alarmId); }
        }
      } else if (active) { // Oneshot : rien a faire sur effacement, l'evenement est deja consomme
        armClass(def.klass);
        queueOneshot(alarmId);
      }
    },

    /** @param {string} missionState  ex. 'HOLD' -- appelé uniquement sur un vrai changement d'état (cf. link.js). */
    notifyMissionState: function notifyMissionState(missionState) {
      var alarmId = MISSION_ALARMS_TABLE.get(missionState);
      if (!alarmId) return; // pas de mapping pour cet etat : rien a faire
      var def = ALARMS_TABLE.get(alarmId);
      if (!def) { console.warn('[gcs] alarme "' + alarmId + '" inconnue de ALARMS.csv'); return; }
      // Pas de notion de "source" a decompter pour un etat (contrairement a
      // un code) -- meme une alarme marquee Loop est jouee une seule fois ici,
      // sinon elle ne s'arreterait jamais.
      armClass(def.klass);
      queueOneshot(alarmId);
    },

    /** @param {'MW'|'MC'} klass */
    ack: function ack(klass) {
      if (klass !== 'MW' && klass !== 'MC') return;
      ackedClass[klass] = true;
      if (currentlyPlaying && currentlyPlaying.def.klass === klass) stopCurrent();
    },
  };

  // ── Scheduler : priorité d'abord, "le plus en retard" ensuite pour
  // départager (même principe que PacketScheduler::nextDue, cf.
  // PacketScheduler.hpp) -- préemption vérifiée à CHAQUE tick, pas
  // seulement quand le canal se libère. ────────────────────────────────────
  function loopOverdue(alarmId, def, now) {
    if (def.periodMs <= 0) return 0; // boucle continue : toujours "juste due", pas de cumul d'attente
    var last = lastPlayedAt.get(alarmId) || 0;
    if (last === 0) return now; // jamais joue : tres en retard, gagne les ex-aequo
    return now - last - def.periodMs;
  }

  function pickNext(now) {
    var best = null, bestScore = -Infinity;

    loopSources.forEach(function (set, alarmId) {
      if (set.size === 0) return;
      var def = ALARMS_TABLE.get(alarmId);
      if (!def || ackedClass[def.klass]) return;
      var overdue = loopOverdue(alarmId, def, now);
      if (def.periodMs > 0 && overdue < 0) return; // pas encore du
      var score = def.priority * 1e6 + overdue;
      if (score > bestScore) { bestScore = score; best = { alarmId: alarmId, def: def }; }
    });

    oneshotQueue.forEach(function (q) {
      var def = ALARMS_TABLE.get(q.alarmId);
      if (!def || ackedClass[def.klass]) return;
      var score = def.priority * 1e6 + (now - q.since);
      if (score > bestScore) { bestScore = score; best = { alarmId: q.alarmId, def: def }; }
    });

    return best;
  }

  function startPlaying(candidate) {
    // Rien a muter tant que le son est verrouille (pas de geste utilisateur) :
    // si on posait currentlyPlaying ici sans jouer, tick() croirait que ça
    // joue deja et ne retenterait plus jamais -- laisser currentlyPlaying a
    // null pour que chaque tick retente startPlaying() jusqu'au deblocage.
    if (!unlocked) return;

    var el = getAudioEl(candidate.alarmId, candidate.def);
    var continuous = candidate.def.type === 'Loop' && candidate.def.periodMs <= 0;
    el.loop = continuous;
    el.onended = continuous ? null : function () {
      if (currentlyPlaying && currentlyPlaying.alarmId === candidate.alarmId) currentlyPlaying = null;
    };
    el.currentTime = 0;

    currentlyPlaying = { alarmId: candidate.alarmId, def: candidate.def, el: el };
    if (candidate.def.type === 'Loop') lastPlayedAt.set(candidate.alarmId, performance.now());
    else oneshotQueue = oneshotQueue.filter(function (q) { return q.alarmId !== candidate.alarmId; });

    el.play().catch(function (e) {
      console.warn('[gcs] alarme ' + candidate.alarmId + ': lecture bloquée (' + e.message + ')');
      currentlyPlaying = null;
    });
  }

  function tick() {
    var now = performance.now();
    var best = pickNext(now);

    if (currentlyPlaying) {
      if (best && best.alarmId !== currentlyPlaying.alarmId &&
          best.def.priority > currentlyPlaying.def.priority) {
        stopCurrent();
        startPlaying(best);
      }
      return;
    }

    if (best) startPlaying(best);
  }

  tablesReady.then(function () { setInterval(tick, SCHEDULER_TICK_MS); });

})(window);
