#pragma once
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/SystemMonitoring/SharedComMem.hpp"
#include "drone/types.hpp"
#include <optional>
#include <utility>

// Table de transition de MissionState, extraite pour être testable sans
// passer par TStateMachine (loop() est privée) — même principe que
// NAV_CONV/MAVLINK_CONV. Couvre exactement les états et transitions
// décrits par l'utilisateur (pas le diagramme SDD §3.6.2 d'origine, plus
// ambigu sur MISSION<->MANUAL et la reprise HOLD). Pas de RTB en V1 (décidé
// explicitement) : CMD_RTL reste un paquet valide au niveau fil, mais
// TGCSCmdRouter ne le relaie plus ici — cf. MissionControl.cpp.
namespace MC_CONV {

// cf. autres timeouts 30s déjà actés dans le SDD §4 (perte lien double,
// GPS non revenu...) — cohérence de "feel" entre les différents failsafe.
inline constexpr TYPES::Ms kEmergencyTimeout{30000};

struct Inputs {
  // Au plus une commande GCS dépilée ce cycle par TGCSCmdRouter.
  std::optional<GCSCommand> gcsCmd{};

  // nullopt = vérifications en cours, sinon résultat définitif. Valide
  // seulement si current == PreFlight.
  std::optional<bool> preflightResult{};
  bool armError{false}; // valide seulement si current == Armed

  // Deux scénarios distincts (TSafetyMonitor), pas un seul "failure" plat :
  // - recoverable : GPS/EKF/lien FC perdus — peuvent revenir, cf. EMERGENCY
  //   (hold, question au GCS si récupéré, timeout sinon).
  // - unrecoverable : RPM bas, batterie critique — aucune attente possible
  //   (ex: l'ESC coupe les moteurs sous un certain seuil de tension), atterrit
  //   directement sans passer par la logique de reprise d'EMERGENCY.
  bool recoverableFailureActive{false};
  bool unrecoverableFailureActive{false};

  bool obstacleActive{false};     // NVA004, cf. TFSTask (Nav)
  // Hors du volume de vol autorisé (rayon/plafond du plan) — traité comme
  // un obstacle : HOLD immédiat, le drone reste pilotable, pas de RTB en
  // V1 pour l'y ramener automatiquement (cf. discussion).
  bool volumeBreached{false};
  // LIDAR perdu (SFS004) — PAS un critère d'EMERGENCY (pas une panne qui
  // menace le vol lui-même) : HOLD + question au GCS (QueryType::LidarLost,
  // choice 0=reprendre sans évitement d'obstacle, 1=passer en MANUAL).
  bool lidarLost{false};
  bool missionComplete{false};    // séquenceur (TMiSequencer)
  bool landed{false};             // FC posé (LANDING -> IDLE)

  // Réponse à la PendingQuery en cours, si elle correspond au seq passé à
  // step() — {seq, choice}. cf. QueryType pour la sémantique de `choice`.
  std::optional<std::pair<uint32_t, uint8_t>> queryAnswer{};
  TYPES::Ms emergencyElapsed{0}; // depuis l'entrée en EMERGENCY
};

struct Outcome {
  MissionState next;
  // Type de PendingQuery qui devrait être active ce cycle, nullopt si
  // aucune — niveau, pas front : TStateMachine ne (re)publie/ne change que
  // sur front (même pattern edge-triggered que TFSTask::ekfStaleActive_),
  // sinon chaque cycle romprait la corrélation de seq avec CMD_ANSWER.
  std::optional<QueryType> query{};
};

// `previous` : dernier MissionState stable (Mission ou Manual) avant la
// bascule courante — maintenu par TStateMachine, mis à jour UNIQUEMENT en
// quittant Mission/Manual eux-mêmes (jamais en quittant Hold/Emergency,
// pour que "reprise" depuis l'un de ces états retombe toujours sur le
// vrai mode de vol d'origine, pas sur un état transitoire).
// `pendingQuerySeq` : PendingQuery::seq actuellement publié (0 si aucun).
[[nodiscard]] inline Outcome step(MissionState current, MissionState previous,
                                  uint32_t pendingQuerySeq,
                                  const Inputs &in) noexcept {
  auto isCmd = [&](GCSCommandType t) {
    return in.gcsCmd.has_value() && in.gcsCmd->type == t;
  };

  switch (current) {
  case MissionState::Idle:
    if (isCmd(GCSCommandType::Arm))
      return {MissionState::PreFlight};
    return {current};

  case MissionState::PreFlight:
    if (in.preflightResult) {
      if (*in.preflightResult)
        return {MissionState::Armed};
      return {MissionState::Idle}; // échec explicite -> retour à IDLE
    }
    return {current}; // vérifications toujours en cours

  case MissionState::Armed:
    if (in.armError || isCmd(GCSCommandType::Disarm))
      return {MissionState::Idle};
    if (isCmd(GCSCommandType::Takeoff))
      return {MissionState::Mission};
    return {current};

  case MissionState::Mission:
    if (in.unrecoverableFailureActive)
      return {MissionState::Landing};
    if (in.recoverableFailureActive)
      return {MissionState::Emergency};
    if (isCmd(GCSCommandType::Land) || in.missionComplete)
      return {MissionState::Landing};
    if (in.obstacleActive || in.volumeBreached || in.lidarLost || isCmd(GCSCommandType::Hold))
      return {MissionState::Hold};
    if (isCmd(GCSCommandType::Mode) && in.gcsCmd->mode == 1)
      return {MissionState::Manual};
    return {current};

  case MissionState::Manual:
    if (in.unrecoverableFailureActive)
      return {MissionState::Landing};
    if (in.recoverableFailureActive)
      return {MissionState::Emergency};
    if (isCmd(GCSCommandType::Land))
      return {MissionState::Landing};
    if (in.obstacleActive || in.volumeBreached || in.lidarLost || isCmd(GCSCommandType::Hold))
      return {MissionState::Hold};
    if (isCmd(GCSCommandType::Mode) && in.gcsCmd->mode == 0)
      return {MissionState::Mission};
    return {current};

  case MissionState::Hold:
    if (in.unrecoverableFailureActive)
      return {MissionState::Landing};
    if (in.recoverableFailureActive)
      return {MissionState::Emergency};
    if (isCmd(GCSCommandType::Land))
      return {MissionState::Landing};
    // Réponse à la question LidarLost (cf. Inputs::lidarLost) : 0=reprendre
    // sans évitement d'obstacle (previous), 1=passer en MANUAL. Vérifiée
    // avant RESUME(GCS) — une réponse explicite prime sur un simple ordre
    // générique de sortie de hold.
    if (in.queryAnswer && in.queryAnswer->first == pendingQuerySeq) {
      if (in.queryAnswer->second == 1)
        return {MissionState::Manual};
      return {previous};
    }
    if (isCmd(GCSCommandType::Resume))
      return {previous}; // pas de sortie auto sur obstacle dégagé, cf. commentaire ci-dessus
    if (in.lidarLost)
      return {current, QueryType::LidarLost};
    return {current};

  case MissionState::Landing:
    if (in.landed)
      return {MissionState::Idle};
    return {current};

  case MissionState::Emergency:
    // Un échec irrécupérable qui apparaît pendant une EMERGENCY déjà en
    // cours (ex: GPS perdu déclenche EMERGENCY, puis batterie critique)
    // prime sur tout le reste — pas d'attente de réponse GCS ni de timeout.
    if (in.unrecoverableFailureActive)
      return {MissionState::Landing};
    if (in.queryAnswer && in.queryAnswer->first == pendingQuerySeq) {
      if (in.queryAnswer->second == 0)
        return {previous}; // resume
      return {MissionState::Landing}; // land
    }
    if (in.emergencyElapsed >= kEmergencyTimeout)
      return {MissionState::Landing};
    if (!in.recoverableFailureActive)
      return {current, QueryType::EmergencyRecoverable};
    return {current};
  }

  return {current}; // exhaustif par construction, satisfait -Wreturn-type
}

} // namespace MC_CONV
