#pragma once
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/Components/SystemMonitoring/Driver.hpp"
#include "drone/Components/SystemMonitoring/PacketScheduler.hpp"
#include "drone/Components/SystemMonitoring/SharedComMem.hpp"
#include "drone/core/ComponentBase.hpp"

class SysMonitoring;

// Composants pertinents pour le GCS, GlobalWatchdog exclu.
inline constexpr size_t kMonitoredComponentCount = 5;

// File locale au process, sans verrou : même thread des deux côtés —
// cf. SYSMONITORING.md §6.
struct PendingCodeEvent {
  uint32_t code{0};
  bool active{false}; // true = code levé, false = code effacé
  TYPES::TimePoint ts{};
};
inline constexpr size_t kPendingCodeCap = 16;

// Lien principal (WFB-NG) : porte tous les paquets descendants + vidéo.
class TelMainTask : public Task {
public:
  explicit TelMainTask(SysMonitoring &component, TaskConfig config,
                       ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
  // Un composant par envoi ; propre à ce lien (§6).
  size_t healthCycleIndex_{0};
  // Un couple (composant, Task) par envoi (§13). TASK_RATE est Links=MAIN,
  // donc ce curseur n'existe pas côté TelSec.
  size_t taskRateCycleIndex_{0};

  // Diff des codes majeurs ; snapshot propre à ce lien (§6).
  std::array<std::array<uint32_t, kMajorCodeSlots>, kMonitoredComponentCount> lastMajorCodes_{};
  bool codePrimed_{false};
  std::array<PendingCodeEvent, kPendingCodeCap> pendingCodes_{};
  size_t pendingCodeCount_{0};
};

// Lien de secours (CC1101) : sous-ensemble léger (Links=BOTH uniquement).
class TelSecTask : public Task {
public:
  explicit TelSecTask(SysMonitoring &component, TaskConfig config,
                      ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
  size_t healthCycleIndex_{0};
  std::array<std::array<uint32_t, kMajorCodeSlots>, kMonitoredComponentCount> lastMajorCodes_{};
  bool codePrimed_{false};
  std::array<PendingCodeEvent, kPendingCodeCap> pendingCodes_{};
  size_t pendingCodeCount_{0};
};

// Réception des commandes montantes (CMD_*/HEARTBEAT_GCS), sur les deux liens.
class GCSRxTask : public Task {
public:
  explicit GCSRxTask(SysMonitoring &component, TaskConfig config,
                     ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
  bool mainLostActive_{false};
  bool secLostActive_{false};
};

// Relai vidéo (WFB-NG uniquement), tolérant à la perte.
class VideoTask : public Task {
public:
  explicit VideoTask(SysMonitoring &component, TaskConfig config,
                     ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
};

class SysMonitoring : public ComponentBase<4> {
public:
  // Drivers injectés par interface, cf. SYSMONITORING.md §7.
  explicit SysMonitoring(ComponenConfig config, SharedSysStateMemHandler &sysState,
                         SharedComMemHandler &com, SharedNavMemHandler &nav,
                         SharedSFMemHandler &sf, SharedFCStatusHandler &fc,
                         SharedCmdMemHandler &cmd, ITelemetryLink &mainLink,
                         ITelemetryLink &secLink, IVideoSource &videoSource,
                         ISystemTempSource &tempSource);

  SharedComMemHandler &com_;
  SharedSysStateMemHandler &sysState_; // pour raiseCode/clearCode depuis les Task
  SharedNavMemHandler &nav_;           // source POS/VELNED/ATTITUDE/AUTOPILOT_MODE (FMA)
  SharedSFMemHandler &sf_;             // source GPSFIX/GPSPOS/SAT_INFO/LIDAR_SCAN
  SharedFCStatusHandler &fc_;          // source BATTERY
  SharedCmdMemHandler &cmd_;           // source MISSION_STATE, AUTOPILOT_MODE (cible)

  ITelemetryLink &mainLink_;
  ITelemetryLink &secLink_;
  IVideoSource &videoSource_;
  ISystemTempSource &tempSource_;

  PacketScheduler mainSched_{TELEM::Link::Main};
  PacketScheduler secSched_{TELEM::Link::Sec};

private:
  TelMainTask TTelMain;
  TelSecTask TTelSec;
  GCSRxTask TGCSRx;
  VideoTask TVideo;
};
