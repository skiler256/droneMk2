#include "drone/Components/MavlinkInterface/MavlinkInterface.hpp"
#include "drone/Components/MavlinkInterface/Conversions.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <cstdlib>
#include <string_view>
#include <cstring>
#include <iostream>
#include <string>

using namespace TYPES;
using namespace MAVLINK_CONV;

namespace {
// Identité unique pour tous les messages émis, cf. MAVLINKINTERFACE.md §3.
constexpr uint8_t kGcsSystemId = 255;
constexpr uint8_t kGcsComponentId = MAV_COMP_ID_MISSIONPLANNER;

// Un canal par thread émetteur, cf. MAVLINKINTERFACE.md §3.
constexpr uint8_t kTxChanTTx = MAVLINK_COMM_1;
constexpr uint8_t kTxChanTTxAck = MAVLINK_COMM_2;
constexpr uint8_t kFcSystemId = 1;
constexpr uint8_t kFcComponentId = MAV_COMP_ID_AUTOPILOT1;

// Échantillon périmé au-delà, cf. MAVLINKINTERFACE.md §4.
constexpr TYPES::Ms kMaxSampleAge{300};

// Aligné sur le failsafe GCS natif d'ArduPilot, cf. MAVLINKINTERFACE.md §9.
constexpr TYPES::Ms kFcLinkTimeout{3000};

// Seuils RPM/batterie à calibrer sur banc, cf. MAVLINKINTERFACE.md §9 / MISSIONCONTROL.md §4.
constexpr float kMinArmedRpm = 1000.0f;
constexpr float kMaxArmedRpm = 10000.0f;
constexpr TYPES::Ms kEscStaleThreshold{500};

#ifdef DRONE_USE_SITL_DRIVERS
// Surcharges réservées au SITL : une variable d'environnement ne doit pas
// pouvoir désactiver une protection de vol — cf. sitl/SITL.md §4.
[[nodiscard]] float envFloatOrDefault(const char *envVar, float defaultV) {
  if (const char *v = std::getenv(envVar))
    return std::strtof(v, nullptr);
  return defaultV;
}

// Comparaison à "1" et non simple présence : DRONE_SKIP_RPM_CHECK=0
// désactivait le check, l'inverse de ce que la variable laisse lire.
[[nodiscard]] bool envFlag(const char *envVar) {
  const char *v = std::getenv(envVar);
  return v && std::string_view(v) == "1";
}

const bool kSkipRpmCheck = envFlag("DRONE_SKIP_RPM_CHECK");
const float kBattLowVoltage = envFloatOrDefault("DRONE_BATT_LOW_V", 19.8f);
const float kBattCriticalVoltage = envFloatOrDefault("DRONE_BATT_CRITICAL_V", 18.0f);
#else
constexpr bool kSkipRpmCheck = false;
// Pack 6S, cf. MAVLINKINTERFACE.md §9.
constexpr float kBattLowVoltage = 19.8f;
constexpr float kBattCriticalVoltage = 18.0f;
#endif

// Durée pendant laquelle le seuil doit rester franchi, par niveau : FCP001
// n'est qu'un voyant, FCP003 déclenche un LANDING — cf. MAVLINKINTERFACE.md §9.
constexpr TYPES::Ms kBattLowSagFilter{500};
constexpr TYPES::Ms kBattCriticalSagFilter{10000};

// Fallback autonome si MissionControl ou le GCS ne répondent plus, cf.
// MAVLINKINTERFACE.md §9.
constexpr TYPES::Ms kMcHeartbeatTimeout{2000};
constexpr TYPES::Ms kGcsIndependentTimeout{30000};

// cf. MAVLINKINTERFACE.md §9.
void enqueueLandCommand(MavlinkInterface &comp, bool fromMissionControl,
                        uint32_t seq, CODES::Code code) {
  mavlink_message_t msg{};
  mavlink_msg_command_long_pack_chan(
      kGcsSystemId, kGcsComponentId, kTxChanTTxAck, &msg, kFcSystemId, kFcComponentId,
      MAV_CMD_DO_SET_MODE, /*confirmation=*/0,
      /*base_mode=*/static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
      /*custom_mode=*/static_cast<float>(COPTER_MODE_LAND), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  PendingAck req{};
  req.kind = PendingKind::Command;
  req.command = MAV_CMD_DO_SET_MODE;
  req.frameLen = static_cast<size_t>(
      mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t *>(req.frame.data()), &msg));
  req.fromMissionControl = fromMissionControl;
  req.cmdSeq = seq;
  req.cmdCode = code;
  if (!comp.ackQueue_.push(req)) {
    comp.sysState_.raiseCode(CODES::MVC003);
    if (fromMissionControl)
      [[maybe_unused]] auto pushed = comp.cmd_.pushMavlinkReport(seq, code, false);
  }
}

// Vitesse pure sur les 3 axes, position/accélérations ignorées -- piège
// ArduCopter et migration du P-controller altitude vers Nav : MAVLINKINTERFACE.md §5.
constexpr uint16_t kPosTargetTypeMask =
    POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE |
    POSITION_TARGET_TYPEMASK_Z_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
    POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
} // namespace

MavlinkInterface::MavlinkInterface(ComponenConfig config,
                                   SharedSysStateMemHandler &sysState,
                                   SharedFCStatusHandler &fc,
                                   SharedNavMemHandler &nav,
                                   SharedSFMemHandler &sf,
                                   SharedCmdMemHandler &cmd,
                                   SharedComMemHandler &com, IMavlinkLink &link)
    : ComponentBase<5>(config, fc, sysState), fc_(fc), sysState_(sysState),
      nav_(nav), sf_(sf), cmd_(cmd), com_(com), link_(link),
      TTx(*this,
          {.id = 0,
           .RTpriority = 75,
           .core = config.CompCore,
           .loopFrequency =
               TYPES::Hz{50}, // couvre SET_POSITION_TARGET_LOCAL_NED (50Hz)
           .timeout = TYPES::Ms{200}},
          localWD),
      TTxACK(
          *this,
          {.id = 1,
           .RTpriority = 70,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{20},
           .timeout = TYPES::Ms{1000}}, // laisse la place au cycle retry 500ms
          localWD),
      TRx(*this,
          {.id = 2,
           .RTpriority = 75,
           .core = config.CompCore,
           .loopFrequency =
               TYPES::Hz{100}, // marge sur le plus rapide des flux RX (50Hz)
           .timeout = TYPES::Ms{200}},
          localWD),
      TCmd(*this,
           {.id = 3,
            .RTpriority = 65,
            .core = config.CompCore,
            .loopFrequency = TYPES::Hz{50},
            .timeout = TYPES::Ms{200}},
           localWD),
      TFailSafe(*this,
                {.id = 4,
                 .RTpriority =
                     80, // supervision failsafe : priorité la plus haute des 5
                 .core = config.CompCore,
                 .loopFrequency = TYPES::Hz{20},
                 .timeout = TYPES::Ms{500}},
                localWD)

{
  registerTask(TTx);
  registerTask(TTxACK);
  registerTask(TRx);
  registerTask(TCmd);
  registerTask(TFailSafe);
  activate();
};

namespace {
// Message d'intérêt + taux voulu. HEARTBEAT absent : ArduPilot l'envoie à
// 1Hz sans le demander (pas un flux au sens SET_MESSAGE_INTERVAL).
struct StreamRequest {
  uint32_t msgid;
  uint32_t intervalUs;
};

// Taux et rationale par message : MAVLINKINTERFACE.md §7.
constexpr std::array<StreamRequest, 8> kInitStreams{{
    {MAVLINK_MSG_ID_ATTITUDE, 50000},              // 20Hz
    {MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000},    // 20Hz
    {MAVLINK_MSG_ID_BATTERY_STATUS, 1000000},      // 1Hz
    {MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, 500000}, // 2Hz
    {MAVLINK_MSG_ID_SCALED_PRESSURE, 500000},      // 2Hz
    {MAVLINK_MSG_ID_SYS_STATUS, 1000000},         // 1Hz — SharedFCStatus.sensorHealth
    // GPS_RAW_INT/EKF_STATUS_REPORT : observabilité bring-up, log console seulement.
    {MAVLINK_MSG_ID_GPS_RAW_INT, 1000000},        // 1Hz
    {MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1000000},  // 1Hz
}};
} // namespace

void MavlinkInterface::requestInitStreams() {
  for (const auto &stream : kInitStreams) {
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack_chan(
        kGcsSystemId, kGcsComponentId, kTxChanTTxAck, &msg, kFcSystemId,
        kFcComponentId, MAV_CMD_SET_MESSAGE_INTERVAL,
        /*confirmation=*/0, static_cast<float>(stream.msgid),
        static_cast<float>(stream.intervalUs), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    PendingAck req{};
    req.kind = PendingKind::Command;
    req.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    req.frameLen = static_cast<size_t>(mavlink_msg_to_send_buffer(
        reinterpret_cast<uint8_t *>(req.frame.data()), &msg));

    if (!ackQueue_.push(req)) {
      // File pleine, marge nulle avec un ordre MC concurrent -- cf. MAVLINKINTERFACE.md §7.
      sysState_.raiseCode(CODES::MVC003);
      break;
    }
  }
}

// Détection seulement, cf. MAVLINKINTERFACE.md §9.
void FSTask::loop() {
  auto hb = comp.fc_.getLastHeartbeat();
  bool lost = !hb || UTILITIES::msBetween(*hb, Clock::now()) > kFcLinkTimeout;

  if (lost && !linkLostActive_) {
    comp.sysState_.raiseCode(CODES::MVC004);
    linkLostActive_ = true;
  } else if (!lost && linkLostActive_) {
    comp.sysState_.clearCode(CODES::MVC004);
    linkLostActive_ = false;
  }

  // Voyants RPM/batterie, et MVC008 si la donnée FC est illisible — §9.
  auto armed = comp.fc_.getArmed();
  auto battery = comp.fc_.getBattery();
  bool fcUnreadable = !armed || !battery;
  if (fcUnreadable && !fcUnreadableActive_) {
    comp.sysState_.raiseCode(CODES::MVC008);
    fcUnreadableActive_ = true;
  } else if (!fcUnreadable && fcUnreadableActive_) {
    comp.sysState_.clearCode(CODES::MVC008);
    fcUnreadableActive_ = false;
  }

  bool rpmFault = false;
  bool rpmHigh = false;
  if (!kSkipRpmCheck && armed.value_or(false)) {
    auto escTs = comp.fc_.getEscStatusTs();
    bool escStale = !escTs || UTILITIES::msBetween(*escTs, Clock::now()) > kEscStaleThreshold;
    if (escStale) {
      rpmFault = true;
    } else if (auto esc = comp.fc_.getEscStatus()) {
      for (float rpm : esc->rpmValue) {
        if (!(rpm >= 0.0f) || rpm < kMinArmedRpm) // !(>=0) attrape aussi NaN
          rpmFault = true;
        if (rpm > kMaxArmedRpm)
          rpmHigh = true;
      }
    }
  }

  if (rpmFault && !rpmLowActive_) {
    comp.sysState_.raiseCode(CODES::MVP001);
    rpmLowActive_ = true;
  } else if (!rpmFault && rpmLowActive_) {
    comp.sysState_.clearCode(CODES::MVP001);
    rpmLowActive_ = false;
  }

  if (rpmHigh && !rpmHighActive_) {
    comp.sysState_.raiseCode(CODES::MVP002);
    rpmHighActive_ = true;
  } else if (!rpmHigh && rpmHighActive_) {
    comp.sysState_.clearCode(CODES::MVP002);
    rpmHighActive_ = false;
  }

  // Confirmé seulement si la condition tient `hold` d'affilée.
  auto sustained = [&](bool raw, TYPES::TimePoint &since, TYPES::Ms hold) {
    if (!raw) {
      since = TYPES::TimePoint{};
      return false;
    }
    if (since == TYPES::TimePoint{}) {
      since = Clock::now();
      return false;
    }
    return UTILITIES::msBetween(since, Clock::now()) >= hold;
  };

  // Pas conditionné à l'armement, contrairement au RPM. voltage<=0 = jamais reçu.
  float voltage = battery.value_or(FCBatteryStatus{}).voltage.v;
  bool battLow = voltage > 0.0f && voltage < kBattLowVoltage;
  bool battCritical = voltage > 0.0f && voltage < kBattCriticalVoltage;

  // Illisible : conserver l'état, ne pas effacer une alarme légitime.
  if (fcUnreadable) {
    battLow = battLowActive_;
    battCritical = battCriticalActive_;
  } else {
    battLow = sustained(battLow, battLowSince_, kBattLowSagFilter);
    battCritical =
        sustained(battCritical, battCriticalSince_, kBattCriticalSagFilter);
  }

  if (battLow && !battLowActive_) {
    comp.sysState_.raiseCode(CODES::FCP001);
    battLowActive_ = true;
  } else if (!battLow && battLowActive_) {
    comp.sysState_.clearCode(CODES::FCP001);
    battLowActive_ = false;
  }

  if (battCritical && !battCriticalActive_) {
    comp.sysState_.raiseCode(CODES::FCP003);
    battCriticalActive_ = true;
  } else if (!battCritical && battCriticalActive_) {
    comp.sysState_.clearCode(CODES::FCP003);
    battCriticalActive_ = false;
  }

  // Fallback autonome, cf. MAVLINKINTERFACE.md §9.
  auto mcHb = comp.cmd_.getHeartbeat();
  bool mcLost = !mcHb || UTILITIES::msBetween(*mcHb, Clock::now()) > kMcHeartbeatTimeout;
  if (mcLost && !mcHeartbeatLostActive_) {
    comp.sysState_.raiseCode(CODES::MVC005);
    mcHeartbeatLostActive_ = true;
    enqueueLandCommand(comp, /*fromMissionControl=*/false, 0, CODES::MVC005);
  } else if (!mcLost && mcHeartbeatLostActive_) {
    comp.sysState_.clearCode(CODES::MVC005);
    mcHeartbeatLostActive_ = false;
  }

  // Défense en profondeur, indépendante de SysMonitoring -- MAVLINKINTERFACE.md §9.
  auto hbMain = comp.com_.getLastHeartbeatMain();
  auto hbSec = comp.com_.getLastHeartbeatSec();
  bool mainLost = !hbMain || UTILITIES::msBetween(*hbMain, Clock::now()) > kGcsIndependentTimeout;
  bool secLost = !hbSec || UTILITIES::msBetween(*hbSec, Clock::now()) > kGcsIndependentTimeout;
  bool bothGcsLost = mainLost && secLost;

  if (bothGcsLost && !gcsBothLostActive_) {
    comp.sysState_.raiseCode(CODES::MVC006);
    gcsBothLostActive_ = true;
    enqueueLandCommand(comp, /*fromMissionControl=*/false, 0, CODES::MVC006);
  } else if (!bothGcsLost && gcsBothLostActive_) {
    comp.sysState_.clearCode(CODES::MVC006);
    gcsBothLostActive_ = false;
  }
}

// cf. MAVLINKINTERFACE.md §8.
void TCmdTask::loop() {
  auto order = comp.cmd_.popMavlinkCommand();
  if (!order)
    return;

  mavlink_message_t msg{};
  uint16_t command = 0;

  if (order->code == CODES::MCO001) {
    // Takeoff : payload = altitude cible en cm (cf. INDEX.csv), param7 de
    // MAV_CMD_NAV_TAKEOFF attend des mètres.
    command = MAV_CMD_NAV_TAKEOFF;
    float altitudeM = static_cast<float>(order->payload) / 100.0f;
    mavlink_msg_command_long_pack_chan(kGcsSystemId, kGcsComponentId, kTxChanTTxAck,
                                       &msg, kFcSystemId, kFcComponentId, command,
                                       /*confirmation=*/0, /*pitch=*/0.0f, 0.0f, 0.0f,
                                       /*yaw=*/0.0f, /*lat=*/0.0f, /*lon=*/0.0f, altitudeM);
  } else if (order->code == CODES::MCO002) {
    command = MAV_CMD_DO_SET_MODE;
    mavlink_msg_command_long_pack_chan(
        kGcsSystemId, kGcsComponentId, kTxChanTTxAck, &msg, kFcSystemId, kFcComponentId,
        command, /*confirmation=*/0,
        /*base_mode=*/static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        /*custom_mode=*/static_cast<float>(COPTER_MODE_GUIDED), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  } else if (order->code == CODES::MCO006) {
    // Même helper que le fallback autonome de FSTask (cf. enqueueLandCommand)
    // — enfile et retourne directement, ne retombe pas dans la queue générique.
    enqueueLandCommand(comp, /*fromMissionControl=*/true, order->seq, order->code);
    return;
  } else if (order->code == CODES::MCO003 || order->code == CODES::MCO004) {
    // Arm/disarm, même commande (param2=0 : pas de bypass des vérifs pré-armement).
    command = MAV_CMD_COMPONENT_ARM_DISARM;
    float arm = (order->code == CODES::MCO003) ? 1.0f : 0.0f;
    mavlink_msg_command_long_pack_chan(kGcsSystemId, kGcsComponentId, kTxChanTTxAck,
                                       &msg, kFcSystemId, kFcComponentId, command,
                                       /*confirmation=*/0, arm, 0.0f, 0.0f, 0.0f, 0.0f,
                                       0.0f, 0.0f);
  } else {
    // Code inconnu : report d'échec quand même, sinon MC attend indéfiniment.
    [[maybe_unused]] auto pushed =
        comp.cmd_.pushMavlinkReport(order->seq, order->code, false);
    return;
  }

  PendingAck req{};
  req.kind = PendingKind::Command;
  req.command = command;
  req.frameLen = static_cast<size_t>(
      mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t *>(req.frame.data()), &msg));
  req.fromMissionControl = true;
  req.cmdSeq = order->seq;
  req.cmdCode = order->code;
  if (!comp.ackQueue_.push(req)) {
    // Queue pleine : report d'échec quand même, même raison que le code
    // inconnu ci-dessus (sinon MC attend indéfiniment).
    comp.sysState_.raiseCode(CODES::MVC003);
    [[maybe_unused]] auto pushed =
        comp.cmd_.pushMavlinkReport(order->seq, order->code, false);
  }
}

namespace {
// Un seul point d'envoi pour les messages TX routine. Renvoie true sur
// succès uniquement — l'appelant n'avance son "dernier envoyé" que dans ce
// cas, un échec laisse la donnée "due" pour le tour suivant.
[[nodiscard]] bool packAndSend(IMavlinkLink &link,
                               const mavlink_message_t &msg) {
  std::array<std::byte, MAVLINK_MAX_PACKET_LEN> buf{};
  uint16_t len =
      mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t *>(buf.data()), &msg);
  // send() renvoie nullopt=succès (convention du projet) — inversé
  // explicitement plutôt qu'un "!" nu au site d'appel.
  std::optional<TYPES::DriverError> err =
      link.send(std::span<const std::byte>(buf.data(), len));
  bool sentOk = !err.has_value();
  return sentOk;
}
} // namespace

// HEARTBEAT : périodique pur (jamais déclencher le failsafe GCS du FC).
// GPS_INPUT/SET_POSITION_TARGET_LOCAL_NED : événementiels, envoyés
// seulement si sf_/nav_ ont un échantillon neuf et pas périmé (§4 du .md).
void TTxTask::loop() {
  auto now = Clock::now();

  if (UTILITIES::msBetween(comp.lastHeartbeatSent_, now) >= Ms{1000}) {
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack_chan(kGcsSystemId, kGcsComponentId, kTxChanTTx,
                                    &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                                    0, 0, MAV_STATE_ACTIVE);
    if (packAndSend(comp.link_, msg))
      comp.lastHeartbeatSent_ = now;
  }

  if (auto sample = comp.sf_.getGpsMag()) {
    bool isNew = sample->ts != comp.lastGpsMagSampleTs_;
    bool isFresh = UTILITIES::msBetween(sample->ts, now) < kMaxSampleAge;

    if (isNew && isFresh) {
      int32_t latE7 = degToE7(sample->latitude);
      int32_t lonE7 = degToE7(sample->longitude);
      // yaw=0 : "cap inconnu" selon la spec MAVLink GPS_INPUT — le
      // magnétomètre est câblé directement sur le FC (EK3_SRC1_YAW=compas),
      // SensorFusion ne fournit plus de cap sur ce lien.
      uint16_t yawCdeg = 0;

      // time_usec accepte le temps depuis le boot, contrairement à
      // time_week/time_week_ms qui exigent le vrai temps GPS (§11).
      auto timeUsec = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              sample->ts.time_since_epoch())
              .count());

      mavlink_message_t msg{};
      mavlink_msg_gps_input_pack_chan(
          kGcsSystemId, kGcsComponentId, kTxChanTTx, &msg, timeUsec,
          /*gps_id=*/0, /*ignore_flags=*/0, sample->gpsTowMs, sample->gpsWeek,
          sample->fixType, latE7, lonE7, sample->altMsl, sample->hdop,
          sample->vdop, sample->velNed.ned.x, sample->velNed.ned.y,
          sample->velNed.ned.z, sample->speedAccuracy, sample->horizAccuracy,
          sample->vertAccuracy, sample->satellitesVisible, yawCdeg);
      if (packAndSend(comp.link_, msg))
        comp.lastGpsMagSampleTs_ = sample->ts;
    }
  }

  if (auto cmd = comp.nav_.getCommand()) {
    bool isNew = cmd->ts != comp.lastNavCmdSampleTs_;
    bool isFresh = UTILITIES::msBetween(cmd->ts, now) < kMaxSampleAge;

    if (isNew && isFresh) {
      auto timeBootMs = static_cast<uint32_t>(
          std::chrono::duration_cast<Ms>(cmd->ts.time_since_epoch()).count());

      mavlink_message_t msg{};
      mavlink_msg_set_position_target_local_ned_pack_chan(
          kGcsSystemId, kGcsComponentId, kTxChanTTx, &msg, timeBootMs,
          kFcSystemId, kFcComponentId, MAV_FRAME_LOCAL_NED, kPosTargetTypeMask,
          /*x=*/0.0f, /*y=*/0.0f, /*z=*/0.0f, cmd->velNed.ned.x,
          cmd->velNed.ned.y, cmd->velNed.ned.z, /*afx=*/0.0f, /*afy=*/0.0f,
          /*afz=*/0.0f, /*yaw=*/cmd->headingCmd.v, /*yaw_rate=*/0.0f);
      bool sent = packAndSend(comp.link_, msg);
      if (sent)
        comp.lastNavCmdSampleTs_ = cmd->ts;
    }
  }
}

// Seule Task à modifier comp.pendingAck_ — pas de mutex nécessaire.
void TTxAckTask::loop() {
  auto now = Clock::now();

  // Résout pendingAck_ : CommandReport (corrélé par seq) si l'ordre vient
  // de MissionControl, sinon MVC002 (ex: requestInitStreams(), personne à
  // qui reporter) — cf. §6 du .md.
  auto resolvePending = [this](bool success) {
    if (comp.pendingAck_.fromMissionControl) {
      if (!comp.cmd_.pushMavlinkReport(comp.pendingAck_.cmdSeq, comp.pendingAck_.cmdCode,
                                       success))
        comp.sysState_.raiseCode(CODES::MVC003); // report perdu (queue pleine)
    } else if (!success) {
      comp.sysState_.raiseCode(CODES::MVC002);
    }
    comp.pendingAck_.kind = PendingKind::None;
  };

  // 1) Traiter les ACK que TRx a routés depuis le dernier tour.
  while (auto msg = comp.ackFifo_.pop()) {
    if (comp.pendingAck_.kind == PendingKind::None) {
      // ACK reçu alors qu'on n'attend rien (timeout déjà épuisé, ou désync).
      comp.sysState_.raiseCode(CODES::MVC001);
      continue;
    }

    if (!ackMatchesPending(*msg, comp.pendingAck_)) {
      comp.sysState_.raiseCode(CODES::MVC001); // ACK d'une autre commande (déjà abandonnée ?)
      continue;
    }

    // Matcher l'identité ne suffit pas : result doit être consulté avant de
    // considérer la commande résolue (ex: DENIED != ACCEPTED).
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      mavlink_command_ack_t ack{};
      mavlink_msg_command_ack_decode(&*msg, &ack);

      if (ack.result == MAV_RESULT_IN_PROGRESS)
        continue; // pas final (ex: takeoff) : on continue d'attendre le vrai résultat

      if (ack.result != MAV_RESULT_ACCEPTED) {
        // N'abandonne la séquence que pour un ordre MissionControl : une
        // commande interne qui échoue ne doit pas emporter un ordre de vol
        // en file derrière elle — cf. MAVLINKINTERFACE.md §6.
        bool wasFromMC = comp.pendingAck_.fromMissionControl;
        resolvePending(false);
        if (wasFromMC)
          comp.ackQueue_.clear();
        continue;
      }
    }

    resolvePending(true); // résolu, prêt pour la prochaine commande
  }

  // 2) Retry/timeout de la commande en attente.
  if (comp.pendingAck_.kind != PendingKind::None) {
    if (UTILITIES::msBetween(comp.pendingAck_.sentAt, now) < Ms{500})
      return; // toujours dans le délai, rien à faire ce tour-ci

    if (comp.pendingAck_.retries >= 3) {
      // Échec définitif : même traitement qu'un résultat négatif explicite
      // (cf. plus haut) — n'abandonne la séquence que si c'était un ordre
      // MissionControl.
      bool wasFromMC = comp.pendingAck_.fromMissionControl;
      resolvePending(false);
      if (wasFromMC)
        comp.ackQueue_.clear();
      return;
    }

    ++comp.pendingAck_.retries;
    comp.pendingAck_.sentAt = now;
    // Échec d'écriture transitoire ignoré ici : le prochain timeout (500ms) retentera.
    [[maybe_unused]] auto writeResult =
        comp.link_.send(std::span<const std::byte>(
            comp.pendingAck_.frame.data(), comp.pendingAck_.frameLen));
    return;
  }

  // 3) Rien en attente : dépiler ackQueue_ (déjà packée), envoyer, amorcer le suivi ACK/retry.
  if (auto req = comp.ackQueue_.pop()) {
    comp.pendingAck_ = *req;
    comp.pendingAck_.sentAt = now;
    comp.pendingAck_.retries = 0;
    [[maybe_unused]] auto sendResult =
        comp.link_.send(std::span<const std::byte>(
            comp.pendingAck_.frame.data(), comp.pendingAck_.frameLen));
  }
}

// Seule Task à appeler link_.poll() : deux lecteurs concurrents du même fd
// mélangeraient le flux d'octets. mavlink_parse_char() garde son état entre
// deux loop() — gère nativement les messages tronqués/concaténés.
void TRxTask::loop() {
  auto now = Clock::now();

  mavlink_message_t msg{};
  mavlink_status_t parseStatus{};

  // Draine tout ce qui est dispo ce tour-ci (pas un seul datagramme), sinon
  // le tampon noyau s'accumule à 100Hz face aux flux ArduPilot. Plafonné
  // pour ne pas monopoliser le coeur RT si le tampon est énorme.
  constexpr int kMaxDatagramsPerTick = 64;
  for (int datagramIdx = 0; datagramIdx < kMaxDatagramsPerTick; ++datagramIdx) {
    auto bytes = comp.link_.poll();
    if (!bytes)
      break; // NoNewData, ou erreur driver — plus rien à parser ce tour-ci

    for (std::byte b : *bytes) {
      if (!mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(b), &msg,
                              &parseStatus))
        continue; // message incomplet ou CRC invalide : pas encore prêt

      switch (msg.msgid) {
      case MAVLINK_MSG_ID_COMMAND_ACK:
        [[fallthrough]];
      case MAVLINK_MSG_ID_PARAM_VALUE:
        // TRx route seulement, TTxACK matche à la commande en attente.
        comp.ackFifo_.push(msg);
        break;

      case MAVLINK_MSG_ID_ATTITUDE: {
        // ArduPilot SITL n'émet jamais ATTITUDE_QUATERNION, seulement Euler.
        mavlink_attitude_t att{};
        mavlink_msg_attitude_decode(&msg, &att);
        comp.fc_.setAttitude(
            Attitude{eulerToQuat(att.roll, att.pitch, att.yaw)});
        break;
      }

      case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t pos{};
        mavlink_msg_local_position_ned_decode(&msg, &pos);
        comp.fc_.setLocalPosVel(Position{Vec3{pos.x, pos.y, pos.z}},
                                Velocity{Vec3{pos.vx, pos.vy, pos.vz}});
        break;
      }

      case MAVLINK_MSG_ID_BATTERY_STATUS: {
        mavlink_battery_status_t bat{};
        mavlink_msg_battery_status_decode(&msg, &bat);
        FCBatteryStatus batStatus{.voltage = mvToVolts(bat.voltages[0]),
                                  .current = caToAmps(bat.current_battery),
                                  .remainingPct =
                                      clampBatteryPct(bat.battery_remaining)};
        comp.fc_.setBattery(batStatus);
        break;
      }

      case MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4: {
        // ArduPilot SITL n'émet jamais ESC_STATUS générique, seulement cette extension.
        mavlink_esc_telemetry_1_to_4_t esc{};
        mavlink_msg_esc_telemetry_1_to_4_decode(&msg, &esc);
        FCEscStatus escStatus{};
        for (size_t i = 0; i < 4; ++i) {
          escStatus.rpmValue[i] = static_cast<float>(esc.rpm[i]);
          escStatus.currentAmps[i] = escCaToAmps(esc.current[i]);
          escStatus.temperatureC[i] = static_cast<float>(esc.temperature[i]);
        }
        comp.fc_.setEscStatus(escStatus);
        break;
      }

      case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t pres{};
        mavlink_msg_scaled_pressure_decode(&msg, &pres);
        FCPressure presStatus{.absHpa = pres.press_abs,
                              .diffHpa = pres.press_diff,
                              .temperatureC = cdegcToC(pres.temperature)};
        comp.fc_.setPressure(presStatus);
        break;
      }

      case MAVLINK_MSG_ID_HEARTBEAT:
        // Ne compte que le HEARTBEAT du FC lui-même — du trafic parasite
        // (sonde MAVProxy, autre GCS) peut arriver sur le même canal.
        if (msg.sysid == kFcSystemId && msg.compid == kFcComponentId) {
          mavlink_heartbeat_t hb{};
          mavlink_msg_heartbeat_decode(&msg, &hb);
          comp.fc_.setArmed((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0);

          comp.fc_.setLastHeartbeat(now);
          // Premier HEARTBEAT réel = preuve que le lien est vivant.
          if (!comp.initStreamsRequested_) {
            comp.requestInitStreams();
            comp.initStreamsRequested_ = true;
          }
        }
        break;

      // ArduPilot y annonce prearm checks/EKF3 source/erreurs capteur —
      // juste loggé pour l'instant, pas de champ SharedFCStatus dédié.
      case MAVLINK_MSG_ID_STATUSTEXT: {
#ifdef DRONE_USE_SITL_DRIVERS
        mavlink_statustext_t st{};
        mavlink_msg_statustext_decode(&msg, &st);
        std::string text(st.text, strnlen(st.text, sizeof(st.text)));
        std::cout << "[FC STATUSTEXT] severity=" << static_cast<int>(st.severity)
                 << " " << text << "\n";
#endif
        break;
      }

      case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t sys{};
        mavlink_msg_sys_status_decode(&msg, &sys);
        comp.fc_.setSensorHealth(
            FCSensorHealth{.present = sys.onboard_control_sensors_present,
                          .enabled = sys.onboard_control_sensors_enabled,
                          .health = sys.onboard_control_sensors_health});
        break;
      }

      case MAVLINK_MSG_ID_GPS_RAW_INT: {
#ifdef DRONE_USE_SITL_DRIVERS
        mavlink_gps_raw_int_t gps{};
        mavlink_msg_gps_raw_int_decode(&msg, &gps);
        std::cout << "[FC GPS_RAW_INT] fix_type=" << static_cast<int>(gps.fix_type)
                 << " satellites=" << static_cast<int>(gps.satellites_visible) << "\n";
#endif
        break;
      }

      case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
#ifdef DRONE_USE_SITL_DRIVERS
        mavlink_ekf_status_report_t ekf{};
        mavlink_msg_ekf_status_report_decode(&msg, &ekf);
        std::cout << "[FC EKF_STATUS_REPORT] flags=0x" << std::hex << ekf.flags
                 << std::dec << " pos_horiz_var=" << ekf.pos_horiz_variance
                 << " compass_var=" << ekf.compass_variance << "\n";
#endif
        break;
      }

      default:
        break; // message reçu mais pas dans notre liste d'intérêt : ignoré
      }
    }
  }
}
