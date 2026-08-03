/* ============================================================================
   main.js — cablage et boucle.

   La boucle. C'est tout ce que fait ce fichier.

   Trois cadences independantes, volontairement :

     - ARRIVEE DES DONNEES : asynchrone, imposee par le drone (1 a 20 Hz selon
       le paquet). Le handler WebSocket ecrit dans `state` quand ca lui chante.
     - RENDU : cadence fixe, RENDER_HZ. Un burst de paquets ne declenche pas un
       burst de rendus ; une coupure de lien ne fige pas l'IHM.
     - VIEWPORT 3D : son propre requestAnimationFrame interne (three.js), pilote
       uniquement par ses attributs. On n'y touche pas.

   Le decouplage rendu/reseau est la raison d'etre de ce fichier : c'est ce qui
   permet a l'affichage de rester reactif (survol, onglets, redimensionnement)
   meme quand le lien est mort.
   ========================================================================== */

(function (global) {
  'use strict';

  var GCS = global.GCS;

  /* ── Configuration ─────────────────────────────────────────────────────── */

  var RENDER_HZ = 20;                      // 50 ms — au-dela l'oeil ne suit plus
  var RENDER_PERIOD_MS = 1000 / RENDER_HZ;

  var USE_SIMULATOR = false;
  // GCS/server.js attache le WebSocketServer directement sur le meme port
  // HTTP (3000) que la page — pas de port separe (8765 n'existe pas).
  var WS_URL = 'ws://' + global.location.host;

  /* ── Etat ──────────────────────────────────────────────────────────────── */

  var state = GCS.createState();
  global.gcsState = state;                 // pratique pour bidouiller en console

  /* ── Commandes montantes ───────────────────────────────────────────────── */

  // Altitude de decollage par defaut pour START_MISSION -> CMD_TAKEOFF (le
  // bouton "START MISSION" de la maquette n'a pas de champ de saisie
  // d'altitude) — cf. Conversions.hpp : Armed -> Mission se declenche
  // uniquement sur GCSCommandType::Takeoff, il n'existe pas de commande GCS
  // "demarrer la mission" separee. A ajuster si une vraie saisie est voulue.
  var DEFAULT_TAKEOFF_ALT_M = 10;

  function onCommand(cmd) {
    switch (cmd) {
      case 'ARM_TOGGLE':
        // En reel : laisser le drone confirmer via ARM_STATE plutot que de
        // basculer l'affichage localement (sinon l'IHM ment sur l'etat reel).
        if (USE_SIMULATOR) {
          state.mission.armed = !state.mission.armed;
        } else {
          GCS.link.send(state.mission.armed ? 'CMD_DISARM' : 'CMD_ARM');
        }
        break;
      case 'LOAD_PLAN':
        // Gap connu : aucun mecanisme d'upload/streaming de plan de vol
        // n'existe cote drone (le plan est charge depuis un fichier local
        // fixe par MissionControl) -- rien a envoyer, juste un rappel local.
        GCS.pushEvent(state, 'CAUTION', 'LOAD_PLAN: pas de mecanisme cote drone pour l\'instant');
        break;
      case 'START_MISSION':
        GCS.link.send('CMD_TAKEOFF', { altitude: DEFAULT_TAKEOFF_ALT_M });
        break;
      case 'HOLD':  GCS.link.send('CMD_HOLD'); break;
      case 'RTL':   GCS.link.send('CMD_RTL'); break; // MissionControl rejette RTL en V1 (MCA003)
      default: break;
    }
    GCS.pushEvent(state, 'NOTICE', 'GCS: commande ' + cmd + ' emise');
  }

  /* ── Demarrage ─────────────────────────────────────────────────────────── */

  function boot() {
    GCS.initRender(state, onCommand);

    if (USE_SIMULATOR) {
      GCS.sim.init(state);
    } else {
      GCS.link.connect(WS_URL, state);
    }

    var lastFrame  = performance.now();
    var lastRender = 0;

    function loop(now) {
      requestAnimationFrame(loop);

      var dt = (now - lastFrame) / 1000;
      lastFrame = now;
      if (dt > 0.25) dt = 0.25;            // garde-fou : onglet remis au premier plan

      // Source locale uniquement. En reel, cette ligne disparait : c'est
      // ws.onmessage qui alimente `state`, de facon asynchrone.
      if (USE_SIMULATOR) GCS.sim.update(state, dt, now);

      // Rendu a cadence fixe.
      if (now - lastRender < RENDER_PERIOD_MS) return;
      lastRender = now;

      GCS.markStale(state, now);
      GCS.render(state);
    }

    requestAnimationFrame(loop);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }

})(window);
