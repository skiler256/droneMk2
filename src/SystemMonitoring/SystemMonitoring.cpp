#include "drone/Components/System Monitoring/Framing.hpp"
#include "drone/Components/System Monitoring/SysMonitoring.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"

using namespace TYPES;

namespace {
// HEARTBEAT_GCS part à 1Hz (cf. TELEMETRY.csv) — seuil avec marge, même
// esprit que kFcLinkTimeout côté MavlinkInterface (3x la période nominale).
constexpr Ms kGcsHeartbeatTimeout{3000};
} // namespace

SysMonitoring::SysMonitoring(ComponenConfig config,
                             SharedSysStateMemHandler &sysState,
                             SharedComMemHandler &com, SharedNavMemHandler &nav,
                             SharedSFMemHandler &sf, SharedFCStatusHandler &fc,
                             SharedCmdMemHandler &cmd, ITelemetryLink &mainLink,
                             ITelemetryLink &secLink, IVideoSource &videoSource)
    : ComponentBase<4>(config, com, sysState), com_(com), sysState_(sysState),
      nav_(nav), sf_(sf), fc_(fc), cmd_(cmd), mainLink_(mainLink), secLink_(secLink),
      videoSource_(videoSource), TTelMain(*this,
                                          {.id = 0,
                                           .RTpriority = 70,
                                           .core = config.CompCore,
                                           .loopFrequency = TYPES::Hz{50},
                                           .timeout = TYPES::Ms{200}},
                                          localWD),
      TTelSec(
          *this,
          {.id = 1,
           .RTpriority = 75, // plus important que le primaire (arret d'urgence)
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{10},
           .timeout = TYPES::Ms{500}},
          localWD),
      TGCSRx(*this,
             {.id = 2,
              .RTpriority = 65,
              .core = config.CompCore,
              .loopFrequency = TYPES::Hz{50},
              .timeout = TYPES::Ms{200}},
             localWD),
      TVideo(*this,
             {.id = 3,
              .RTpriority = 40,
              .core = config.CompCore,
              .loopFrequency = TYPES::Hz{30},
              .timeout = TYPES::Ms{500}},
             localWD)

{
  registerTask(TTelMain);
  registerTask(TTelSec);
  registerTask(TGCSRx);
  registerTask(TVideo);
  activate();
};

// NOTE: POS/VELNED/ATTITUDE/GPSFIX sont câblés sur sf_ (DynamicState/
// GpsMagSample réels). BATTERY/ARM_STATE/MISSION_STATE/AUTOPILOT_MODE
// restent à valeurs par défaut : ils dépendent de handlers que
// SysMonitoring n'a pas encore (SharedFCStatus, SharedCmdMem) — cf.
// SYSMONITORING.md §9.

void TelMainTask::loop() {
  auto now = Clock::now();
  const auto *due = comp.mainSched_.nextDue(now);
  if (!due)
    return;

  switch (due->id) {
  case TELEM::PacketID::POS: {
    TELEM::PosPkt pkt{};
    if (auto state = comp.sf_.getDynamicState()) {
      pkt.north = state->posNed.ned.x;
      pkt.east = state->posNed.ned.y;
      pkt.down = state->posNed.ned.z;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::VELNED: {
    TELEM::VelnedPkt pkt{};
    if (auto state = comp.sf_.getDynamicState()) {
      pkt.vn = state->velNed.ned.x;
      pkt.ve = state->velNed.ned.y;
      pkt.vd = state->velNed.ned.z;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::ATTITUDE: {
    TELEM::AttitudePkt pkt{};
    if (auto state = comp.sf_.getDynamicState()) {
      pkt.qw = state->attitude.q.w;
      pkt.qx = state->attitude.q.x;
      pkt.qy = state->attitude.q.y;
      pkt.qz = state->attitude.q.z;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::HEALTH: {
    auto frame = TELEM::encode(TELEM::HealthPkt{});
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::GPSFIX: {
    TELEM::GpsfixPkt pkt{};
    if (auto gps = comp.sf_.getGpsMag()) {
      pkt.fix_type = gps->fixType;
      pkt.num_sats = gps->satellitesVisible;
      pkt.hdop = gps->hdop;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::BATTERY: {
    TELEM::BatteryPkt pkt{};
    if (auto batt = comp.fc_.getBattery()) {
      pkt.voltage = batt->voltage.v;
      pkt.current = batt->current.v;
      pkt.remaining_pct = batt->remainingPct;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::CODE: {
    auto frame = TELEM::encode(TELEM::CodePkt{});
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::ARM_STATE: {
    // TODO: pas de source — MavlinkInterface ne parse/stocke pas encore
    // le bit MAV_MODE_FLAG_SAFETY_ARMED de HEARTBEAT.base_mode dans
    // SharedFCStatus. Paquet envoyé à valeurs par défaut en attendant.
    auto frame = TELEM::encode(TELEM::ArmStatePkt{});
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::MISSION_STATE: {
    TELEM::MissionStatePkt pkt{};
    if (auto state = comp.cmd_.getMissionState())
      pkt.state = static_cast<uint8_t>(*state);
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::AUTOPILOT_MODE: {
    TELEM::AutopilotModePkt pkt{};
    if (auto c = comp.nav_.getLateralEngaged())
      pkt.lateral_engaged = c->pack();
    if (auto c = comp.nav_.getLateralArmed())
      pkt.lateral_armed = c->pack();
    if (auto c = comp.nav_.getVerticalEngaged())
      pkt.vertical_engaged = c->pack();
    if (auto c = comp.nav_.getVerticalArmed())
      pkt.vertical_armed = c->pack();
    if (auto seg = comp.cmd_.getEngagedSegment()) {
      pkt.target_altitude = seg->targetAltitude.v;
      pkt.target_speed = seg->speed.v;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::MC_QUERY: {
    TELEM::McQueryPkt pkt{};
    if (auto q = comp.cmd_.getPendingQuery()) {
      pkt.seq = q->seq;
      pkt.query_type = static_cast<uint8_t>(q->type);
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  default:
    break; // paquets montants : pas schedulés ici (cf. GCSRxTask)
  }
}

void TelSecTask::loop() {
  auto now = Clock::now();
  const auto *due = comp.secSched_.nextDue(now);
  if (!due)
    return;

  switch (due->id) {
  case TELEM::PacketID::HEALTH: {
    auto frame = TELEM::encode(TELEM::HealthPkt{});
    if (!comp.secLink_.send(frame))
      comp.secSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::BATTERY: {
    TELEM::BatteryPkt pkt{};
    if (auto batt = comp.fc_.getBattery()) {
      pkt.voltage = batt->voltage.v;
      pkt.current = batt->current.v;
      pkt.remaining_pct = batt->remainingPct;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.secLink_.send(frame))
      comp.secSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::CODE: {
    auto frame = TELEM::encode(TELEM::CodePkt{});
    if (!comp.secLink_.send(frame))
      comp.secSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::ARM_STATE: {
    // TODO: cf. même remarque que TelMainTask ci-dessus.
    auto frame = TELEM::encode(TELEM::ArmStatePkt{});
    if (!comp.secLink_.send(frame))
      comp.secSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::MISSION_STATE: {
    TELEM::MissionStatePkt pkt{};
    if (auto state = comp.cmd_.getMissionState())
      pkt.state = static_cast<uint8_t>(*state);
    auto frame = TELEM::encode(pkt);
    if (!comp.secLink_.send(frame))
      comp.secSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::MC_QUERY: {
    TELEM::McQueryPkt pkt{};
    if (auto q = comp.cmd_.getPendingQuery()) {
      pkt.seq = q->seq;
      pkt.query_type = static_cast<uint8_t>(q->type);
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.secLink_.send(frame))
      comp.secSched_.markSent(due->id, now);
    break;
  }
  default:
    break; // paquets MAIN-only ou montants : hors périmètre TelSec
  }
}

namespace {

enum class UplinkSource { Main, Sec };

// Dispatch commun aux deux liens montants — factorisé ici plutôt que
// dupliqué dans chaque branche mainLink_/secLink_ de GCSRxTask::loop().
// CMD_EMERGENCY court-circuite volontairement gcsCmds (cf. SharedComMem.hpp) :
// signalée via un CODES::Code, lu par TSafetyMonitor indépendamment du
// rythme de dépilement du FIFO.
void handleUplink(SysMonitoring &comp, UplinkSource src,
                  std::span<const std::byte> bytes) {
  auto frame = TELEM::decode(bytes);
  if (!frame)
    return;

  auto push = [&](GCSCommand c) {
    if (!comp.com_.pushGCSCommand(c))
      comp.sysState_.raiseCode(CODES::SMC001);
  };

  switch (frame->id) {
  case TELEM::PacketID::CMD_ARM:
    push(GCSCommand{.type = GCSCommandType::Arm});
    break;
  case TELEM::PacketID::CMD_DISARM:
    push(GCSCommand{.type = GCSCommandType::Disarm});
    break;
  case TELEM::PacketID::CMD_MODE: {
    auto pkt = TELEM::CmdModePkt::unpack(frame->payload);
    push(GCSCommand{.type = GCSCommandType::Mode, .mode = pkt.mode});
    break;
  }
  case TELEM::PacketID::CMD_WAYPOINT: {
    auto pkt = TELEM::CmdWaypointPkt::unpack(frame->payload);
    push(GCSCommand{
        .type = GCSCommandType::Waypoint,
        .waypoint = TYPES::Position{TYPES::Vec3{pkt.north, pkt.east, pkt.down}},
        .approachSpeed = TYPES::MetersPerSec{pkt.approach_speed},
        .acceptanceRadius = TYPES::Meters{pkt.acceptance_radius}});
    break;
  }
  case TELEM::PacketID::CMD_RTL:
    push(GCSCommand{.type = GCSCommandType::Rtl});
    break;
  case TELEM::PacketID::CMD_LAND:
    push(GCSCommand{.type = GCSCommandType::Land});
    break;
  case TELEM::PacketID::CMD_EMERGENCY:
    comp.sysState_.raiseCode(CODES::SMO001);
    break;
  case TELEM::PacketID::CMD_TAKEOFF: {
    auto pkt = TELEM::CmdTakeoffPkt::unpack(frame->payload);
    push(GCSCommand{.type = GCSCommandType::Takeoff,
                    .altitude = TYPES::Meters{pkt.altitude}});
    break;
  }
  case TELEM::PacketID::CMD_HOLD:
    push(GCSCommand{.type = GCSCommandType::Hold});
    break;
  case TELEM::PacketID::CMD_RESUME:
    push(GCSCommand{.type = GCSCommandType::Resume});
    break;
  case TELEM::PacketID::CMD_ANSWER: {
    auto pkt = TELEM::CmdAnswerPkt::unpack(frame->payload);
    push(GCSCommand{.type = GCSCommandType::Answer,
                    .answerSeq = pkt.seq,
                    .answerChoice = pkt.choice});
    break;
  }
  case TELEM::PacketID::HEARTBEAT_GCS:
    if (src == UplinkSource::Main)
      comp.com_.markHeartbeatMain();
    else
      comp.com_.markHeartbeatSec();
    break;
  default:
    break; // paquet descendant reçu par erreur : ignoré
  }
}

} // namespace

void GCSRxTask::loop() {
  if (auto bytes = comp.mainLink_.poll())
    handleUplink(comp, UplinkSource::Main, bytes.value());
  if (auto bytes = comp.secLink_.poll())
    handleUplink(comp, UplinkSource::Sec, bytes.value());

  auto now = Clock::now();

  auto hbMain = comp.com_.getLastHeartbeatMain();
  bool mainLost = !hbMain || UTILITIES::msBetween(*hbMain, now) > kGcsHeartbeatTimeout;
  if (mainLost && !mainLostActive_) {
    comp.sysState_.raiseCode(CODES::SMC002);
    mainLostActive_ = true;
  } else if (!mainLost && mainLostActive_) {
    comp.sysState_.clearCode(CODES::SMC002);
    mainLostActive_ = false;
  }

  auto hbSec = comp.com_.getLastHeartbeatSec();
  bool secLost = !hbSec || UTILITIES::msBetween(*hbSec, now) > kGcsHeartbeatTimeout;
  if (secLost && !secLostActive_) {
    comp.sysState_.raiseCode(CODES::SMC003);
    secLostActive_ = true;
  } else if (!secLost && secLostActive_) {
    comp.sysState_.clearCode(CODES::SMC003);
    secLostActive_ = false;
  }
}

void VideoTask::loop() {
  auto frame = comp.videoSource_.pollFrame();
  if (!frame)
    return;
  // TODO: relayer la frame (buffer partagé / socket local) une fois le
  // consommateur (GCS dashboard local ou export réseau) défini.
}
