/* ============================================================================
   alarm.js — alarme sonore façon cockpit, pilotée par les paquets CODE.

   Deux régimes, décidés par la sévérité (INDEX.csv/CODES::Severity) :
     - Warning            -> évènementiel : un seul bip à l'arrivée (Master
                             Caution générique, ou son dédié si INDEX.csv en
                             définit un pour ce code).
     - Critical/Emergency -> boucle tant que le code reste actif (Master
                             Warning générique, ou son dédié).
     - Info                -> jamais de son (pas une panne).

   Un seul canal audio (comme un vrai cockpit à un seul haut-parleur
   d'alarme) : scheduler pondéré par sévérité + "le plus en retard gagne",
   même principe que PacketScheduler côté MavlinkInterface/SysMonitoring
   (cf. PacketScheduler.hpp) — priorité d'abord, ancienneté ensuite pour
   départager, plutôt qu'un simple FIFO qui laisserait une alarme Warning
   bloquer indéfiniment une Critical arrivée après.

   Master Warning / Master Caution (bandeau physique du futur H562) :
   uniquement la LOGIQUE ici (quelle sévérité déclenche quoi) — pas
   d'affichage GCS pour l'instant (décidé explicitement, cf. discussion).

   Synthèse des tons en Web Audio pur (pas de fichier .mp3/.wav) : même
   raison que le vendoring de three.js — pas de dépendance réseau au
   décollage.
   ========================================================================== */

(function (global) {
  'use strict';

  var GCS = global.GCS = global.GCS || {};

  // Cadence de répétition d'une alarme Critical/Emergency active — assez
  // court pour rester pressant, assez long pour laisser une autre alarme de
  // même rang s'intercaler (cf. pickNext).
  var LOOP_PERIOD_MS = 4000;
  var SCHEDULER_TICK_MS = 250;

  var SEVERITY_WEIGHT = { Emergency: 4, Critical: 3, Warning: 2, Info: 1 };

  // sound id -> séquence [{freq (Hz, 0=silence), dur (ms)}, ...]. `_master*`
  // sont les deux replis génériques (INDEX.csv::Sound vide) — les autres
  // entrées sont des sons dédiés (INDEX.csv::Sound rempli), à compléter au
  // fil de l'eau : ce ne sont que deux exemples de départ.
  var SOUND_PATTERNS = {
    _masterWarning: [
      { freq: 1000, dur: 150 }, { freq: 0, dur: 100 },
      { freq: 1000, dur: 150 }, { freq: 0, dur: 100 },
      { freq: 1000, dur: 150 }
    ],
    _masterCaution: [{ freq: 800, dur: 250 }],
    obstacle: [{ freq: 1200, dur: 100 }, { freq: 0, dur: 80 }, { freq: 1200, dur: 100 }],
    batt_critical: [{ freq: 600, dur: 400 }, { freq: 0, dur: 120 }, { freq: 600, dur: 400 }]
  };

  function soundPatternFor(severity, soundId) {
    if (soundId && SOUND_PATTERNS[soundId]) return SOUND_PATTERNS[soundId];
    return severity === 'Warning' ? SOUND_PATTERNS._masterCaution : SOUND_PATTERNS._masterWarning;
  }

  // ── Déblocage audio (obligatoire : tout navigateur bloque l'autoplay sans
  // geste utilisateur) — un simple clic ou touche n'importe où suffit. ────
  var audioCtx = null;
  var unlocked = false;
  function ensureAudio() {
    if (!audioCtx) {
      var Ctor = global.AudioContext || global.webkitAudioContext;
      if (!Ctor) return; // navigateur sans Web Audio : alarme silencieuse, pas de crash
      audioCtx = new Ctor();
    }
    if (audioCtx.state === 'suspended') audioCtx.resume();
    unlocked = true;
  }
  ['click', 'keydown'].forEach(function (evt) {
    global.addEventListener(evt, function unlock() {
      ensureAudio();
      global.removeEventListener(evt, unlock);
    }, { once: true });
  });

  function playPattern(pattern, onDone) {
    if (!audioCtx || !unlocked) { onDone(); return null; }
    var t0 = audioCtx.currentTime;
    var osc = audioCtx.createOscillator();
    var gain = audioCtx.createGain();
    osc.type = 'square';
    osc.connect(gain);
    gain.connect(audioCtx.destination);
    gain.gain.setValueAtTime(0, t0);

    var cursor = t0;
    pattern.forEach(function (step) {
      if (step.freq > 0) {
        osc.frequency.setValueAtTime(step.freq, cursor);
        gain.gain.setValueAtTime(0.25, cursor);
      } else {
        gain.gain.setValueAtTime(0, cursor);
      }
      cursor += step.dur / 1000;
    });
    gain.gain.setValueAtTime(0, cursor);
    osc.start(t0);
    osc.stop(cursor + 0.02);

    var stopped = false;
    osc.onended = function () { if (!stopped) onDone(); };

    return {
      stop: function () {
        if (stopped) return;
        stopped = true;
        try {
          gain.gain.cancelScheduledValues(audioCtx.currentTime);
          gain.gain.setValueAtTime(0, audioCtx.currentTime);
          osc.stop();
        } catch (e) { /* déjà arrêté : rien à faire */ }
      }
    };
  }

  // ── État des alarmes ────────────────────────────────────────────────────
  var loopAlarms = new Map();  // packedCode -> {severity, soundId, label, lastPlayedAt}
  var oneShotQueue = [];       // [{severity, soundId, label}] FIFO (Warning uniquement)
  var currentlyPlaying = null; // {stop(), code, severity} | null

  // Appelée par link.js à chaque paquet CODE décodé.
  GCS.alarm = {
    /**
     * @param {number} packedCode  CODES::Code packé (identifiant stable)
     * @param {{severity:string, sound:string, code:string, name:string}|null} info
     *        résultat de link.js::lookupCode() — null si code inconnu de INDEX.csv.
     * @param {boolean} active
     */
    notify: function notify(packedCode, info, active) {
      var severity = info ? info.severity : 'Warning';
      var soundId = info ? info.sound : '';
      var label = info ? (info.code + ' ' + info.name) : ('0x' + (packedCode >>> 0).toString(16));
      var isLooping = severity === 'Critical' || severity === 'Emergency';

      if (isLooping) {
        if (active) {
          if (!loopAlarms.has(packedCode)) {
            loopAlarms.set(packedCode, { severity: severity, soundId: soundId, label: label, lastPlayedAt: 0 });
          }
        } else {
          loopAlarms.delete(packedCode);
          if (currentlyPlaying && currentlyPlaying.code === packedCode) {
            currentlyPlaying.stop();
            currentlyPlaying = null;
          }
        }
      } else if (severity === 'Warning' && active) {
        oneShotQueue.push({ severity: severity, soundId: soundId, label: label });
      }
      // Info : jamais d'alarme sonore (pas une condition de panne).
    }
  };

  // ── Scheduler pondéré : priorité de sévérité d'abord, "le plus en
  // retard" ensuite pour départager (même formule que PacketScheduler::
  // nextDue — cf. PacketScheduler.hpp, "now - lastSent - period"). ────────
  function pickNext(now) {
    var best = null, bestScore = -Infinity;

    loopAlarms.forEach(function (alarm, code) {
      var overdue = now - alarm.lastPlayedAt - LOOP_PERIOD_MS;
      if (alarm.lastPlayedAt !== 0 && overdue < 0) return; // pas encore dû
      var score = SEVERITY_WEIGHT[alarm.severity] * 1e6 + overdue;
      if (score > bestScore) { bestScore = score; best = { kind: 'loop', code: code, entry: alarm }; }
    });

    if (!best && oneShotQueue.length > 0) {
      best = { kind: 'oneshot', entry: oneShotQueue[0] };
    }
    return best;
  }

  function tick() {
    var now = performance.now();

    if (!currentlyPlaying) {
      var next = pickNext(now);
      if (!next) return;

      if (next.kind === 'loop') {
        next.entry.lastPlayedAt = now;
      } else {
        oneShotQueue.shift();
      }
      var pattern = soundPatternFor(next.entry.severity, next.entry.soundId);
      var handle = playPattern(pattern, function () { currentlyPlaying = null; });
      if (handle) {
        currentlyPlaying = { stop: handle.stop, code: next.code, severity: next.entry.severity };
      }
      return;
    }

    // Préemption : une alarme en boucle plus sévère coupe un one-shot en
    // cours (Master Warning prime toujours sur Master Caution, cf. cockpit
    // réel) — on ne préempte jamais un loop par un autre loop en cours de
    // tonalité, la tonalité est courte, le prochain tick suffit.
    var pending = pickNext(now);
    if (pending && pending.kind === 'loop' &&
        SEVERITY_WEIGHT[pending.entry.severity] > SEVERITY_WEIGHT[currentlyPlaying.severity]) {
      currentlyPlaying.stop();
      currentlyPlaying = null;
    }
  }

  setInterval(tick, SCHEDULER_TICK_MS);

})(window);
