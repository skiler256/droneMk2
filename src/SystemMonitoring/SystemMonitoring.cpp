#include "drone/Components/System Monitoring/Framing.hpp"
#include "drone/Components/System Monitoring/SysMonitoring.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <iostream>

using namespace TYPES;

SysMonitoring::SysMonitoring(ComponenConfig config,
                             SharedSysStateMemHandler &sysState,
                             SharedComMemHandler &com, SharedNavMemHandler &nav,
                             SharedSFMemHandler &sf, ITelemetryLink &mainLink,
                             ITelemetryLink &secLink, IVideoSource &videoSource)
    : ComponentBase<4>(config, com, sysState), com_(com), sysState_(sysState),
      nav_(nav), sf_(sf), mainLink_(mainLink), secLink_(secLink),
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

// NOTE: les paquets sont encore construits avec des valeurs par défaut —
// nav_/sf_ n'exposent pour l'instant que des champs de stub ("bloup"/
// "blip"), pas encore les payloads POS/VELNED/ATTITUDE/GPSFIX réels. Le
// câblage source -> champ de paquet reste à faire une fois ces shm remplis.

void TelMainTask::loop() {
  auto now = Clock::now();
  const auto *due = comp.mainSched_.nextDue(now);
  if (!due)
    return;

  switch (due->id) {
  case TELEM::PacketID::POS: {
    auto frame = TELEM::encode(TELEM::PosPkt{});
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::VELNED: {
    auto frame = TELEM::encode(TELEM::VelnedPkt{});
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::ATTITUDE: {
    auto frame = TELEM::encode(TELEM::AttitudePkt{});
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
    auto frame = TELEM::encode(TELEM::GpsfixPkt{});
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::BATTERY: {
    auto frame = TELEM::encode(TELEM::BatteryPkt{});
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
    auto frame = TELEM::encode(TELEM::ArmStatePkt{});
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::MISSION_STATE: {
    auto frame = TELEM::encode(TELEM::MissionStatePkt{});
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::AUTOPILOT_MODE: {
    auto frame = TELEM::encode(TELEM::AutopilotModePkt{});
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
    auto frame = TELEM::encode(TELEM::BatteryPkt{});
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
    auto frame = TELEM::encode(TELEM::ArmStatePkt{});
    if (!comp.secLink_.send(frame))
      comp.secSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::MISSION_STATE: {
    auto frame = TELEM::encode(TELEM::MissionStatePkt{});
    if (!comp.secLink_.send(frame))
      comp.secSched_.markSent(due->id, now);
    break;
  }
  default:
    break; // paquets MAIN-only ou montants : hors périmètre TelSec
  }
}

namespace {

// Dispatch commun aux deux liens montants — factorisé ici plutôt que
// dupliqué dans chaque branche mainLink_/secLink_ de GCSRxTask::loop().
void handleUplink(std::span<const std::byte> bytes) {
  auto frame = TELEM::decode(bytes);
  if (!frame)
    return;

  switch (frame->id) {
  case TELEM::PacketID::CMD_ARM:
    std::cout << "GCSRx: CMD_ARM\n"; // TODO: router vers MissionControl
    break;
  case TELEM::PacketID::CMD_DISARM:
    std::cout << "GCSRx: CMD_DISARM\n";
    break;
  case TELEM::PacketID::CMD_MODE: {
    auto pkt = TELEM::CmdModePkt::unpack(frame->payload);
    std::cout << "GCSRx: CMD_MODE mode=" << static_cast<int>(pkt.mode) << "\n";
    break;
  }
  case TELEM::PacketID::CMD_WAYPOINT: {
    auto pkt = TELEM::CmdWaypointPkt::unpack(frame->payload);
    std::cout << "GCSRx: CMD_WAYPOINT north=" << pkt.north << "\n";
    break;
  }
  case TELEM::PacketID::CMD_RTL:
    std::cout << "GCSRx: CMD_RTL\n";
    break;
  case TELEM::PacketID::CMD_LAND:
    std::cout << "GCSRx: CMD_LAND\n";
    break;
  case TELEM::PacketID::HEARTBEAT_GCS:
    break; // sert juste a prouver que le GCS est present, rien a router
  default:
    break; // paquet descendant reçu par erreur : ignoré
  }
}

} // namespace

void GCSRxTask::loop() {
  if (auto bytes = comp.mainLink_.poll())
    handleUplink(bytes.value());
  if (auto bytes = comp.secLink_.poll())
    handleUplink(bytes.value());
}

void VideoTask::loop() {
  auto frame = comp.videoSource_.pollFrame();
  if (!frame)
    return;
  // TODO: relayer la frame (buffer partagé / socket local) une fois le
  // consommateur (GCS dashboard local ou export réseau) défini.
}
