#pragma once
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/Components/System Monitoring/Driver.hpp"
#include "drone/Components/System Monitoring/PacketScheduler.hpp"
#include "drone/Components/System Monitoring/SharedComMem.hpp"
#include "drone/core/ComponentBase.hpp"

class SysMonitoring;

// Lien principal (WFB-NG) : porte tous les paquets descendants + vidéo.
class TelMainTask : public Task {
public:
  explicit TelMainTask(SysMonitoring &component, TaskConfig config,
                       ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
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

// Relai vidéo (WFB-NG uniquement) — RT priority basse, tolérante à la perte.
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
  // Les drivers sont injectés par interface, pas possédés en dur : en dev
  // main.cpp passe des drivers UDP factices, en prod WFB_NG_Driver/
  // CC1101_Driver — SysMonitoring ne connaît jamais le type concret
  // (portabilité inter-véhicules, cf. discussion archi).
  explicit SysMonitoring(ComponenConfig config, SharedSysStateMemHandler &sysState,
                         SharedComMemHandler &com, SharedNavMemHandler &nav,
                         SharedSFMemHandler &sf, SharedFCStatusHandler &fc,
                         SharedCmdMemHandler &cmd, ITelemetryLink &mainLink,
                         ITelemetryLink &secLink, IVideoSource &videoSource);

  SharedComMemHandler &com_;
  SharedSysStateMemHandler &sysState_; // pour raiseCode/clearCode depuis les Task
  SharedNavMemHandler &nav_;           // source POS/VELNED/ATTITUDE/AUTOPILOT_MODE (FMA)
  SharedSFMemHandler &sf_;             // source GPSFIX
  SharedFCStatusHandler &fc_;          // source BATTERY
  SharedCmdMemHandler &cmd_;           // source MISSION_STATE, AUTOPILOT_MODE (cible)

  ITelemetryLink &mainLink_;
  ITelemetryLink &secLink_;
  IVideoSource &videoSource_;

  PacketScheduler mainSched_{TELEM::Link::Main};
  PacketScheduler secSched_{TELEM::Link::Sec};

private:
  TelMainTask TTelMain;
  TelSecTask TTelSec;
  GCSRxTask TGCSRx;
  VideoTask TVideo;
};
