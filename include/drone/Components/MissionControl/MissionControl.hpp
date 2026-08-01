#pragma once
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/MissionControl/MissionPlan.hpp"
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/Components/System Monitoring/SharedComMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/watchdog.hpp"
#include <mutex>
#include <optional>

class MissionControl;

// FIFO intra-process (PAS shm) entre TGCSCmdRouter et TStateMachine : les
// deux tournent dans le même process, cf. CLAUDE.md "std::mutex local pour
// éviter les races intra-process entre threads d'un même composant" — la
// CAS-lock shm est le mauvais outil ici, plus lourd et pensé pour de
// l'inter-process. Seul TGCSCmdRouter doit dépiler SharedComMem::gcsCmds
// (cf. discussion : un deuxième consommateur y volerait des commandes qui
// ne lui étaient pas destinées) ; il classe et relaie ici ce qui concerne
// la state machine. Même sémantique que pushFifo/popFifo (échoue plutôt
// qu'écraser le plus ancien) mais sur un RingBuffer local, pas partagé.
class StateMachineOrderQueue {
public:
  [[nodiscard]] bool push(GCSCommand c) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buf_.count >= buf_.buf.size())
      return false;
    buf_.buf[buf_.head] = c;
    buf_.head = (buf_.head + 1) % buf_.buf.size();
    ++buf_.count;
    return true;
  }

  [[nodiscard]] std::optional<GCSCommand> pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buf_.count == 0)
      return std::nullopt;
    auto value = buf_.buf[buf_.tail];
    buf_.tail = (buf_.tail + 1) % buf_.buf.size();
    --buf_.count;
    return value;
  }

private:
  std::mutex mutex_;
  RingBuffer<GCSCommand, 8> buf_{};
};

// Tient l'état système (MissionState, cf. SharedCmdMem.hpp) et ses
// transitions — cf. SDD §3.6.2. Logique pas encore écrite : deux arêtes
// MISSION<->MANUAL restent ambiguës dans le SDD, à trancher avant
// TStateMachine::loop().
class StateMachineTask : public Task {
public:
  explicit StateMachineTask(MissionControl &component, TaskConfig config,
                            ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MissionControl &comp;
  void loop() override;

  // Dernier MissionState stable (Mission ou Manual) avant Hold/Rtb/Emergency
  // — cf. MC_CONV::step() : "reprise" doit toujours retomber sur le vrai
  // mode de vol d'origine, jamais sur un état transitoire.
  MissionState previousFlightMode_{MissionState::Idle};
  TYPES::TimePoint emergencyEnteredAt_{};

  // Vérifications PRE_FLIGHT : faites une seule fois par entrée dans
  // l'état (pas à chaque cycle 20Hz — lecture disque), résultat mis en
  // cache jusqu'à la sortie de PRE_FLIGHT.
  bool preflightChecked_{false};
  bool preflightPassed_{false};

  // Horodatage de la demande d'armement (MCO003) — sert de grâce avant de
  // déclarer armError si le FC ne confirme jamais l'armement (poignée de
  // main matérielle bornée, différent d'un timeout sur la durée d'un
  // segment de mission que l'utilisateur a explicitement écarté).
  TYPES::TimePoint armRequestedAt_{};

  // Désarmé mais toujours en mouvement rapide pendant LANDING -> pas un
  // atterrissage normal, suspicion de crash. Édge-triggered.
  bool crashSuspectedActive_{false};
};

// Interprète le plan de mission (MISSIONPLAN_CONV::MissionPlan, cf.
// MissionPlan.hpp) et arme les segments Nav dans l'ordre — cf. SDD §3.6.3
// (séquence arm/engage déjà validée par le pattern armed/engaged de Nav).
class MiSequencerTask : public Task {
public:
  explicit MiSequencerTask(MissionControl &component, TaskConfig config,
                           ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MissionControl &comp;
  void loop() override;

  // Index du prochain segment de comp.plan_ à armer, et seq du NavSegment
  // actuellement armé/engagé qu'on attend de voir Atteint (0 = rien en
  // cours, prêt à armer le suivant).
  size_t nextIndex_{0};
  uint32_t armedSeqPending_{0};

  // Cible du dernier segment Waypoint armé — sert d'entryWp au suivant
  // (cf. NAV_CONV::canEngage). Absent tant qu'aucun Waypoint n'a été armé
  // cette Mission (le tout premier utilise la position réelle via SF_).
  std::optional<TYPES::Position> lastTargetWp_{};

  // Segment HOLD (pause du plan, pas le MissionState::Hold système) en cours.
  bool inHold_{false};
  TYPES::TimePoint holdUntil_{};

  // Édge-triggered : Nav en bonne santé/publie des NavCommand frais pendant
  // qu'on attend Atteint — pas un timeout sur la durée du segment (décidé
  // explicitement : un segment long avec Nav sain ne doit rien déclencher).
  bool navStaleActive_{false};
};

// Surveille les codes des autres composants (batterie, EKF, liens GCS...) et
// déclenche les réactions failsafe — cf. SDD §4 (table décisions critiques).
class SafetyMonitorTask : public Task {
public:
  explicit SafetyMonitorTask(MissionControl &component, TaskConfig config,
                             ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MissionControl &comp;
  void loop() override;
  bool volumeBreachActive_{false};
};

// Route les commandes reçues du GCS (via SysMonitoring) vers les FIFO
// appropriées (mavlinkCmds/navCmds) ou la state machine — cf. SDD §3.7.2
// "Routage des commandes GCS".
class GCSCmdRouterTask : public Task {
public:
  explicit GCSCmdRouterTask(MissionControl &component, TaskConfig config,
                            ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MissionControl &comp;
  void loop() override;
};

class MissionControl : public ComponentBase<4> {
public:
  explicit MissionControl(ComponenConfig config,
                          SharedSysStateMemHandler &sysState,
                          SharedCmdMemHandler &cmd, SharedFCStatusHandler &fc,
                          SharedNavMemHandler &nav, SharedSFMemHandler &SF,
                          SharedComMemHandler &com);

  SharedCmdMemHandler &cmd_;
  SharedSysStateMemHandler &sysState_;
  // MC touche à tout, contrairement aux autres composants (cf. SDD §3.6.4 —
  // séquencement, RTB, décisions hors-plan, routage GCS lisent/écrivent
  // chacun un sous-ensemble différent de ces segments). Handlers construits
  // par l'appelant avec id=MissionControl, même pattern que le "relay"
  // cross-composant déjà utilisé dans NAV_SITL.cpp.
  SharedFCStatusHandler &fc_;
  SharedNavMemHandler &nav_;
  SharedSFMemHandler &SF_;
  SharedComMemHandler &com_;
  MissionState state_{MissionState::Idle};

  // Plan de mission chargé — rien ne le peuple encore (pipeline upload
  // GCS -> disque -> parse pas construit, cf. discussion). TMiSequencer
  // le consomme tel quel dès qu'il est non vide.
  MISSIONPLAN_CONV::MissionPlan plan_{};
  bool missionComplete_{false}; // écrit par TMiSequencer, lu par TStateMachine

  // Relais TGCSCmdRouter -> TStateMachine — cf. StateMachineOrderQueue.
  StateMachineOrderQueue gcsOrders_{};

  // Publiés par TSafetyMonitor (scan des codes majeurs des autres
  // composants), lus par TStateMachine — même séparation surveillance/
  // décision que TFSTask (Nav/MavlinkInterface) vs. le composant qui agit.
  // Deux scénarios distincts (cf. Conversions.hpp) : recoverable (GPS/EKF/
  // lien FC — EMERGENCY, tentative de reprise) vs. unrecoverable (RPM,
  // batterie critique — LANDING immédiat, aucune attente).
  bool recoverableFailureActive_{false};
  bool unrecoverableFailureActive_{false};
  bool obstacleActive_{false};
  // Hors du volume de vol autorisé (plan_.header.navVolumeRadius/Ceiling).
  bool volumeBreached_{false};
  // LIDAR perdu (SFS004) — cf. Conversions.hpp, ni recoverable ni
  // unrecoverable : HOLD + question au GCS (QueryType::LidarLost).
  bool lidarLost_{false};

private:
  StateMachineTask TStateMachine;
  MiSequencerTask TMiSequencer;
  SafetyMonitorTask TSafetyMonitor;
  GCSCmdRouterTask TGCSCmdRouter;
};
