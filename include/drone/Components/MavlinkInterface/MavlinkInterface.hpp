#pragma once
#include "drone/Components/MavlinkInterface/AckFifo.hpp"
#include "drone/Components/MavlinkInterface/Driver.hpp"
#include "drone/Components/MavlinkInterface/PendingAck.hpp"
#include "drone/Components/MavlinkInterface/PendingAckQueue.hpp"
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
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
                            SharedSFMemHandler &sf, IMavlinkLink &link);

  // Empile une demande MAV_CMD_SET_MESSAGE_INTERVAL par message d'intérêt
  // dans ackQueue_ — TTxACK les enverra une par une (ACK'd, retry inclus).
  // TEMPORAIRE : à terme cette séquence est ordonnée par MissionControl via
  // TCmd (cf. MAVLINKINTERFACE.md §2) ; en attendant, appelée directement
  // par le binaire SITL en bootstrap manuel (cf. MV_SITL.cpp).
  void requestInitStreams();

  SharedFCStatusHandler &fc_;
  SharedSysStateMemHandler &sysState_; // pour raiseCode/clearCode depuis les Task
  SharedNavMemHandler &nav_;           // source consigne vitesse (SET_POSITION_TARGET_LOCAL_NED)
  SharedSFMemHandler &sf_;             // source GPS_INPUT (position/vitesse brutes + cap fusionné)
  IMavlinkLink &link_;

  // TRx est seule à lire link_ (cf. MAVLINKINTERFACE.md) — c'est l'unique
  // canal par lequel TTxACK apprend qu'un COMMAND_ACK/PARAM_VALUE est arrivé.
  AckFifo ackFifo_;

  // État de la commande ACK'd en vol (TTxACK) et dernier HEARTBEAT envoyé
  // (TTx) — sur comp_ plutôt que sur la Task elle-même, par cohérence avec
  // le reste du code (mainSched_/secSched_ de SysMonitoring).
  PendingAck pendingAck_;
  PendingAckQueue ackQueue_; // commandes ACK'd en attente d'envoi, cf. requestInitStreams()
  TYPES::TimePoint lastHeartbeatSent_{};

  // Horodatage de l'ÉCHANTILLON (pas de l'envoi) le plus récent déjà
  // transmis — sert à détecter une donnée réellement nouvelle chez
  // sf_/nav_ (cf. §5 : événementiel, pas de redondance périodique).
  TYPES::TimePoint lastGpsMagSampleTs_{};
  TYPES::TimePoint lastNavCmdSampleTs_{};

private:
  TTxTask TTx;
  TTxAckTask TTxACK;
  TRxTask TRx;
  TCmdTask TCmd;
  FSTask TFailSafe;
};
