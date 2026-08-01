#pragma once
#include "drone/Components/MavlinkInterface/AckFifo.hpp"
#include "drone/Components/MavlinkInterface/Driver.hpp"
#include "drone/Components/MavlinkInterface/PendingAck.hpp"
#include "drone/Components/MavlinkInterface/PendingAckQueue.hpp"
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/Components/System Monitoring/SharedComMem.hpp"
#include "drone/core/ComponentBase.hpp"

class MavlinkInterface;

// Flux routine : HEARTBEAT périodique + GPS_INPUT/SET_POSITION_TARGET_LOCAL_NED événementiels.
class TTxTask : public Task {
public:
  explicit TTxTask(MavlinkInterface &component, TaskConfig config,
                   ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MavlinkInterface &comp;
  void loop() override;
};

// Flux ponctuel ACK'd (PARAM_SET, CMD_DO_SET_MODE, SET_MESSAGE_INTERVAL).
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

// Dépile les ordres MissionControl (SharedCmdMem), les traduit en COMMAND_LONG pour TTxAck.
class TCmdTask : public Task {
public:
  explicit TCmdTask(MavlinkInterface &component, TaskConfig config,
                    ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MavlinkInterface &comp;
  void loop() override;
};

// Surveille la fraîcheur du HEARTBEAT FC (code MVC004) — pas d'action
// autonome ici, le failsafe GCS d'ArduPilot gère déjà le LAND (SDD §4).
class FSTask : public Task {
public:
  explicit FSTask(MavlinkInterface &component, TaskConfig config,
                  ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MavlinkInterface &comp;
  void loop() override;
  bool linkLostActive_{false};
  bool rpmLowActive_{false};
  bool battLowActive_{false};
  bool battCriticalActive_{false};
  // Fallback autonome (LAND) — cf. MAVLINKINTERFACE.md.
  bool mcHeartbeatLostActive_{false};
  bool gcsBothLostActive_{false};
};

class MavlinkInterface : public ComponentBase<5> {
public:
  // Lien injecté par interface, jamais un type concret (portabilité SITL).
  explicit MavlinkInterface(ComponenConfig config,
                            SharedSysStateMemHandler &sysState,
                            SharedFCStatusHandler &fc, SharedNavMemHandler &nav,
                            SharedSFMemHandler &sf, SharedCmdMemHandler &cmd,
                            SharedComMemHandler &com, IMavlinkLink &link);

  // Empile un MAV_CMD_SET_MESSAGE_INTERVAL par message d'intérêt. Déclenchée
  // par TRx au premier HEARTBEAT du FC (cf. initStreamsRequested_), jamais
  // à la construction (lien pas forcément prêt, cf. §7 du .md).
  void requestInitStreams();

  SharedFCStatusHandler &fc_;
  SharedSysStateMemHandler &sysState_; // pour raiseCode/clearCode depuis les Task
  SharedNavMemHandler &nav_;           // source consigne vitesse (SET_POSITION_TARGET_LOCAL_NED)
  SharedSFMemHandler &sf_;             // source GPS_INPUT (position/vitesse brutes + cap fusionné)
  SharedCmdMemHandler &cmd_;           // ordres ponctuels de MissionControl (TCmd), cf. §10
  SharedComMemHandler &com_;           // heartbeat GCS, lu indépendamment par FSTask (fallback autonome)
  IMavlinkLink &link_;

  // Garde requestInitStreams() : déclenchée une seule fois, au premier
  // HEARTBEAT reçu du FC (cf. TRxTask::loop()).
  bool initStreamsRequested_{false};

  // TRx est seule à lire link_ — unique canal par lequel TTxACK apprend qu'un ACK est arrivé.
  AckFifo ackFifo_;

  PendingAck pendingAck_;   // commande ACK'd en vol (TTxACK)
  PendingAckQueue ackQueue_; // commandes ACK'd en attente d'envoi
  TYPES::TimePoint lastHeartbeatSent_{};

  // Horodatage de l'échantillon (pas de l'envoi) déjà transmis — détecte une donnée neuve, pas de renvoi périodique.
  TYPES::TimePoint lastGpsMagSampleTs_{};
  TYPES::TimePoint lastNavCmdSampleTs_{};

private:
  TTxTask TTx;
  TTxAckTask TTxACK;
  TRxTask TRx;
  TCmdTask TCmd;
  FSTask TFailSafe;
};
