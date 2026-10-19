#pragma once
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/SystemMonitoring/SharedComMem.hpp"
#include "drone/types.hpp"
#include <optional>
#include <utility>

// Table de transition de MissionState, extraite pour être testable hors de
// TStateMachine — cf. MISSIONCONTROL.md §3. Pas de RTB en V1 : CMD_RTL
// reste valide au niveau fil mais n'est plus relayé.
namespace MC_CONV {

// 30s : aligné sur les autres timeouts failsafe du SDD §4.
inline constexpr TYPES::Ms kEmergencyTimeout{30000};

struct Inputs {
  // Au plus une commande GCS dépilée ce cycle par TGCSCmdRouter.
  std::optional<GCSCommand> gcsCmd{};

  // nullopt = vérifications en cours. Valide si current == PreFlight.
  std::optional<bool> preflightResult{};
  bool armError{false}; // valide seulement si current == Armed

  // recoverable (EMERGENCY, reprise possible) vs unrecoverable (LANDING
  // immédiat) — deux réactions distinctes, cf. MISSIONCONTROL.md §6.
  bool recoverableFailureActive{false};
  bool unrecoverableFailureActive{false};

  bool obstacleActive{false};     // NVA004, cf. TFSTask (Nav)
  // Traité comme un obstacle : HOLD immédiat — MISSIONCONTROL.md §6.
  bool volumeBreached{false};
  // SFS004 — HOLD + question au GCS, pas une EMERGENCY (§6).
  bool lidarLost{false};
  bool missionComplete{false};    // séquenceur (TMiSequencer)
  bool landed{false};             // FC posé (LANDING -> IDLE)

  // {seq, choice} ; sémantique de `choice` : cf. QueryType.
  std::optional<std::pair<uint32_t, uint8_t>> queryAnswer{};
  TYPES::Ms emergencyElapsed{0};  // depuis l'entrée en EMERGENCY
  TYPES::Ms holdQueryElapsed{0};  // depuis la publication de la question en HOLD
};

struct Outcome {
  MissionState next;
  // Signal de NIVEAU, pas de front : TStateMachine ne (re)publie que sur
  // front, sinon la corrélation seq/CMD_ANSWER casse à chaque cycle (§3).
  std::optional<QueryType> query{};
};

// `previous` : dernier état stable, mis à jour UNIQUEMENT en quittant
// Mission/Manual, jamais Hold/Emergency — cf. MISSIONCONTROL.md §3.
// `pendingQuerySeq` : PendingQuery::seq publié (0 si aucun).
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
    // Vérifiée avant RESUME : une réponse explicite prime sur un ordre
    // générique de sortie de hold (choice : cf. QueryType).
    if (in.queryAnswer && in.queryAnswer->first == pendingQuerySeq) {
      if (in.queryAnswer->second == 1)
        return {MissionState::Manual};
      return {previous};
    }
    if (isCmd(GCSCommandType::Resume))
      return {previous}; // pas de sortie auto sur obstacle dégagé, cf. commentaire ci-dessus
    if (in.lidarLost) {
      // Sans réponse du GCS on n'attend pas indéfiniment, même timeout que
      // l'EMERGENCY (cf. MISSIONCONTROL.md §6).
      if (in.holdQueryElapsed >= kEmergencyTimeout)
        return {MissionState::Landing};
      return {current, QueryType::LidarLost};
    }
    return {current};

  case MissionState::Landing:
    if (in.landed)
      return {MissionState::Idle};
    return {current};

  case MissionState::Emergency:
    // Prime même sur une EMERGENCY en cours : ni attente GCS, ni timeout.
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
