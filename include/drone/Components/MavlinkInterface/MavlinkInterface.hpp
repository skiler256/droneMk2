#pragma once
#include "drone/Components/MavlinkInterface/Driver.hpp"
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/core/ComponentBase.hpp"

class MavlinkInterface;

// Flux routine (GPS_INPUT/SET_POSITION_TARGET_LOCAL_NED événementiels,
// HEARTBEAT périodique) — cf. MAVLINKINTERFACE.md §2/§4/§5.
class TTxTask : public Task {
public:
  explicit TTxTask(MavlinkInterface &component, TaskConfig config,
                   ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MavlinkInterface &comp;
  void loop() override;
};

// Flux ponctuel ACK'd (PARAM_SET, CMD_DO_SET_MODE, init MAV_CMD_SET_
// MESSAGE_INTERVAL) — son propre scheduler, cf. MAVLINKINTERFACE.md §3/§6.
class TTxAckTask : public Task {
public:
  explicit TTxAckTask(MavlinkInterface &component, TaskConfig config,
                      ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MavlinkInterface &comp;
  void loop() override;
};

// Parsing des messages entrants, écrit dans SharedFCStatus.
class TRxTask : public Task {
public:
  explicit TRxTask(MavlinkInterface &component, TaskConfig config,
                   ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MavlinkInterface &comp;
  void loop() override;
};

// Reçoit les décisions de MissionControl, les traduit pour TTx/TTxAck —
// pas de MAVLink direct ici (pattern TCmd générique, cf. autres composants).
class TCmdTask : public Task {
public:
  explicit TCmdTask(MavlinkInterface &component, TaskConfig config,
                    ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MavlinkInterface &comp;
  void loop() override;
};

// Surveille les conditions de failsafe propres au lien FC — loop() vide
// pour l'instant, implémentée en dernier (décision explicite, cf.
// MAVLINKINTERFACE.md §2/§8).
class FSTask : public Task {
public:
  explicit FSTask(MavlinkInterface &component, TaskConfig config,
                  ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MavlinkInterface &comp;
  void loop() override;
};

class MavlinkInterface : public ComponentBase<5> {
public:
  // Le lien UART est injecté par interface, jamais un type concret — même
  // principe que SysMonitoring (portabilité, SITL sans toucher au
  // composant).
  explicit MavlinkInterface(ComponenConfig config,
                            SharedSysStateMemHandler &sysState,
                            SharedFCStatusHandler &fc, SharedNavMemHandler &nav,
                            IMavlinkLink &link);

  SharedFCStatusHandler &fc_;
  SharedSysStateMemHandler &sysState_; // pour raiseCode/clearCode depuis les Task
  SharedNavMemHandler &nav_;           // source consigne vitesse (SET_POSITION_TARGET_LOCAL_NED)
  IMavlinkLink &link_;

private:
  TTxTask TTx;
  TTxAckTask TTxACK;
  TRxTask TRx;
  TCmdTask TCmd;
  FSTask TFailSafe;
};
