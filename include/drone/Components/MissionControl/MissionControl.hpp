#pragma once
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/MissionControl/MissionPlan.hpp"
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/Components/SystemMonitoring/SharedComMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/watchdog.hpp"
#include <atomic>
#include <mutex>
#include <optional>

class MissionControl;

// FIFO intra-process (PAS shm), std::mutex local — cf. MISSIONCONTROL.md §7.
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

// État système et transitions — cf. MISSIONCONTROL.md §3-4. Deux arêtes
// MISSION<->MANUAL restent ambiguës dans le SDD, à trancher avant loop().
class StateMachineTask : public Task {
public:
  explicit StateMachineTask(MissionControl &component, TaskConfig config,
                            ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MissionControl &comp;
  void loop() override;

  // Dernier état stable avant Hold/Rtb/Emergency — MISSIONCONTROL.md §3.
  MissionState previousFlightMode_{MissionState::Idle};
  TYPES::TimePoint emergencyEnteredAt_{};
  TYPES::TimePoint holdQueryAt_{}; // publication de la question GCS en HOLD
  bool commandRejectedActive_{false}; // MCA011, edge-triggered

  // Mises en cache : lecture disque, pas à chaque cycle 20Hz.
  bool preflightChecked_{false};
  bool preflightPassed_{false};

  // Délai de grâce avant armError — cf. MISSIONCONTROL.md §4.
  TYPES::TimePoint armRequestedAt_{};

  // Désarmé mais encore en mouvement pendant LANDING (edge-triggered).
  bool crashSuspectedActive_{false};
};

// Arme les segments Nav dans l'ordre — cf. MISSIONCONTROL.md §5.
class MiSequencerTask : public Task {
public:
  explicit MiSequencerTask(MissionControl &component, TaskConfig config,
                           ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MissionControl &comp;
  void loop() override;

  // armedSeqPending_ : 0 = rien en cours, prêt à armer le suivant.
  size_t nextIndex_{0};
  uint32_t armedSeqPending_{0};

  // entryWp du segment suivant ; absent au premier (position réelle via SF_).
  std::optional<TYPES::Position> lastTargetWp_{};

  // Segment HOLD du plan, pas MissionState::Hold système.
  bool inHold_{false};
  TYPES::TimePoint holdUntil_{};

  // Fraîcheur Nav, pas un timeout de segment — MISSIONCONTROL.md §5.
  bool navStaleActive_{false};
};

// Surveillance des codes majeurs, sans décision — MISSIONCONTROL.md §6.
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

// Routage des commandes GCS — cf. MISSIONCONTROL.md §7.
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
  // MC est le seul à détenir tous les segments — MISSIONCONTROL.md §1.
  SharedFCStatusHandler &fc_;
  SharedNavMemHandler &nav_;
  SharedSFMemHandler &SF_;
  SharedComMemHandler &com_;
  // Partagés entre les 4 Tasks — cf. ARCHITECTURE_IPC.md §3.
  std::atomic<MissionState> state_{MissionState::Idle};

  // Chargé au PRE_FLIGHT ; contient un std::vector, donc mutex et non atomique.
  std::mutex planMutex_;
  MISSIONPLAN_CONV::MissionPlan plan_{};

  std::atomic<bool> missionComplete_{false};
  std::atomic<bool> takeoffComplete_{false}; // bloque le 1er segment, §5
  std::atomic<float> takeoffTargetAlt_{0.0f}; // mètres, positif vers le haut

  // Relais TGCSCmdRouter -> TStateMachine — cf. StateMachineOrderQueue.
  StateMachineOrderQueue gcsOrders_{};

  // Écrits par TSafetyMonitor, lus par TStateMachine — MISSIONCONTROL.md §6.
  std::atomic<bool> recoverableFailureActive_{false};
  std::atomic<bool> unrecoverableFailureActive_{false};
  std::atomic<bool> obstacleActive_{false};
  std::atomic<bool> volumeBreached_{false};
  std::atomic<bool> lidarLost_{false};

private:
  StateMachineTask TStateMachine;
  MiSequencerTask TMiSequencer;
  SafetyMonitorTask TSafetyMonitor;
  GCSCmdRouterTask TGCSCmdRouter;
};
