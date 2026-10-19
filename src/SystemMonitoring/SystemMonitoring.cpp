#include "drone/Components/SystemMonitoring/Framing.hpp"
#include "drone/Components/SystemMonitoring/SysMonitoring.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <algorithm>

using namespace TYPES;

namespace {
// HEARTBEAT_GCS part à 1Hz (cf. TELEMETRY.csv) — seuil avec marge, même
// esprit que kFcLinkTimeout côté MavlinkInterface (3x la période nominale).
constexpr Ms kGcsHeartbeatTimeout{3000};

// Les 5 composants défilent, un par envoi — cf. SYSMONITORING.md §6.
constexpr std::array<ComponentID, kMonitoredComponentCount> kMonitoredComponents{
    ComponentID::MavlinkInterface, ComponentID::SensorFusion, ComponentID::Navigation,
    ComponentID::MissionControl, ComponentID::SysMonitoring};

TELEM::HealthPkt buildHealthPkt(SysMonitoring &comp, size_t &cycleIndex) {
  ComponentID target = kMonitoredComponents[cycleIndex % kMonitoredComponents.size()];
  cycleIndex++;
  TELEM::HealthPkt pkt{};
  pkt.component = static_cast<uint8_t>(target);
  if (auto h = comp.sysState_.getHealth(target))
    pkt.health = static_cast<uint8_t>(*h);
  return pkt;
}

// Un seul couple (composant, Task) par envoi, les slots jamais mesurés
// (hz == 0) sont sautés — cf. SYSMONITORING.md §13.
std::optional<TELEM::TaskRatePkt> buildTaskRatePkt(SysMonitoring &comp,
                                                   size_t &cycleIndex) {
  constexpr size_t kSlots = kMonitoredComponentCount * kMaxTasksPerComponent;

  for (size_t scanned = 0; scanned < kSlots; ++scanned) {
    size_t slot = (cycleIndex + scanned) % kSlots;
    ComponentID target = kMonitoredComponents[slot / kMaxTasksPerComponent];
    size_t task = slot % kMaxTasksPerComponent;

    auto rates = comp.sysState_.getTaskHz(target);
    if (!rates || (*rates)[task] == 0.0f)
      continue;

    cycleIndex = slot + 1;
    TELEM::TaskRatePkt pkt{};
    pkt.component = static_cast<uint8_t>(target);
    pkt.task_id = static_cast<uint8_t>(task);
    pkt.hz = (*rates)[task];
    if (auto targets = comp.sysState_.getTaskTargetHz(target))
      pkt.target_hz = (*targets)[task];
    return pkt;
  }

  return std::nullopt; // aucune mesure encore disponible (démarrage)
}

// Polling+diff sur les codes majeurs seulement (cf. SYSMONITORING.md §6).
// Appelé à chaque loop(), que CODE soit "dû" ou non : détection et
// planification sont deux choses distinctes.
void scanCodeChanges(SysMonitoring &comp, PacketScheduler &sched,
                     std::array<std::array<uint32_t, kMajorCodeSlots>,
                                kMonitoredComponentCount> &lastMajor,
                     bool &primed, std::array<PendingCodeEvent, kPendingCodeCap> &queue,
                     size_t &queueCount) {
  for (size_t i = 0; i < kMonitoredComponents.size(); ++i) {
    auto current = comp.sysState_.getMajorCodes(kMonitoredComponents[i]);
    if (!current)
      continue;

    for (const auto &entry : *current) {
      if (entry.code == 0)
        continue;
      bool wasActive = std::find(lastMajor[i].begin(), lastMajor[i].end(), entry.code) !=
                       lastMajor[i].end();
      if (!wasActive && primed && queueCount < queue.size())
        queue[queueCount++] = PendingCodeEvent{entry.code, true, entry.ts};
    }
    for (uint32_t prevCode : lastMajor[i]) {
      if (prevCode == 0)
        continue;
      bool stillActive = false;
      for (const auto &entry : *current)
        if (entry.code == prevCode) {
          stillActive = true;
          break;
        }
      if (!stillActive && primed && queueCount < queue.size())
        queue[queueCount++] = PendingCodeEvent{prevCode, false, Clock::now()};
    }

    for (size_t s = 0; s < kMajorCodeSlots; ++s)
      lastMajor[i][s] = (*current)[s].code;
  }
  // Premier scan : capture juste l'état déjà présent, n'émet rien (sinon un
  // redémarrage de SysMonitoring "découvrirait" tous les codes déjà actifs
  // comme des évènements fraîchement levés).
  primed = true;

  if (queueCount > 0)
    sched.markDirty(TELEM::PacketID::CODE);
}
} // namespace

SysMonitoring::SysMonitoring(ComponenConfig config,
                             SharedSysStateMemHandler &sysState,
                             SharedComMemHandler &com, SharedNavMemHandler &nav,
                             SharedSFMemHandler &sf, SharedFCStatusHandler &fc,
                             SharedCmdMemHandler &cmd, ITelemetryLink &mainLink,
                             ITelemetryLink &secLink, IVideoSource &videoSource,
                             ISystemTempSource &tempSource)
    : ComponentBase<4>(config, com, sysState), com_(com), sysState_(sysState),
      nav_(nav), sf_(sf), fc_(fc), cmd_(cmd), mainLink_(mainLink), secLink_(secLink),
      videoSource_(videoSource), tempSource_(tempSource), TTelMain(*this,
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

// BATTERY/ARM_STATE/MISSION_STATE/AUTOPILOT_MODE restent par défaut : les
// handlers manquent encore — cf. SYSMONITORING.md §9.

void TelMainTask::loop() {
  // Détection indépendante du scheduling : doit tourner à chaque cycle, pas
  // seulement quand CODE se trouve être "dû" (cf. scanCodeChanges).
  scanCodeChanges(comp, comp.mainSched_, lastMajorCodes_, codePrimed_, pendingCodes_,
                  pendingCodeCount_);

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
    auto frame = TELEM::encode(buildHealthPkt(comp, healthCycleIndex_));
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
    TELEM::CodePkt pkt{};
    if (pendingCodeCount_ > 0) {
      PendingCodeEvent ev = pendingCodes_[0];
      for (size_t i = 1; i < pendingCodeCount_; ++i)
        pendingCodes_[i - 1] = pendingCodes_[i];
      pendingCodeCount_--;
      pkt.code = ev.code;
      pkt.active = ev.active ? 1 : 0;
      pkt.ts_ms = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(ev.ts.time_since_epoch())
              .count());
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame)) {
      comp.mainSched_.markSent(due->id, now);
      if (pendingCodeCount_ > 0) // il en reste : re-planifie sans attendre un nouveau changement
        comp.mainSched_.markDirty(TELEM::PacketID::CODE);
    }
    break;
  }
  case TELEM::PacketID::ARM_STATE: {
    // `reason` reste 0 : pas de table de raisons définie (SYSMONITORING.md §9).
    TELEM::ArmStatePkt pkt{};
    pkt.armed = comp.fc_.getArmed().value_or(false) ? 1 : 0;
    auto frame = TELEM::encode(pkt);
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
  case TELEM::PacketID::GPSPOS: {
    TELEM::GpsposPkt pkt{};
    if (auto gps = comp.sf_.getGpsMag()) {
      pkt.lat = static_cast<float>(gps->latitude);
      pkt.lon = static_cast<float>(gps->longitude);
      pkt.alt_msl = gps->altMsl;
      pkt.vdop = gps->vdop;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::SAT_INFO: {
    TELEM::SatInfoPkt pkt{};
    if (auto gps = comp.sf_.getGpsMag()) {
      pkt.count = std::min(gps->satellitesVisible,
                           static_cast<uint8_t>(GpsMagSample::kMaxSatellites));
      pkt.id = gps->satId;
      pkt.strength = gps->satStrength;
      pkt.quality = gps->satQuality;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::LIDAR_SCAN: {
    TELEM::LidarScanPkt pkt{};
    if (auto lidar = comp.sf_.getLidarMap()) {
      pkt.range_mm = lidar->rangeMm;
      pkt.intensity = lidar->intensity;
      uint16_t minDist = 0;
      int16_t minBearing = 0;
      for (std::size_t i = 0; i < lidar->rangeMm.size(); ++i) {
        if (lidar->rangeMm[i] == 0)
          continue; // pas de mesure à cet angle ce tour-ci
        if (minDist == 0 || lidar->rangeMm[i] < minDist) {
          minDist = lidar->rangeMm[i];
          minBearing = static_cast<int16_t>(i);
        }
      }
      pkt.min_dist_mm = minDist;
      pkt.min_bearing_deg = minBearing;
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::CPU_TEMP: {
    TELEM::CpuTempPkt pkt{};
    if (auto temp = comp.tempSource_.readCelsius())
      pkt.temp_c = *temp;
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::ESC_RPM: {
    // Régime par moteur, seule grandeur ESC transmise pour l'instant. Reste
    // à 0 si le FC n'a jamais émis ESC_TELEMETRY (cf. SYSMONITORING.md §14).
    TELEM::EscRpmPkt pkt{};
    if (auto esc = comp.fc_.getEscStatus())
      pkt.rpm = esc->rpmValue;
    auto frame = TELEM::encode(pkt);
    if (!comp.mainLink_.send(frame))
      comp.mainSched_.markSent(due->id, now);
    break;
  }
  case TELEM::PacketID::TASK_RATE: {
    // markSent inconditionnel : sans mesure disponible, ne pas réessayer au
    // cycle suivant (sinon TASK_RATE monopolise le lien au démarrage).
    if (auto pkt = buildTaskRatePkt(comp, taskRateCycleIndex_)) {
      auto frame = TELEM::encode(*pkt);
      if (!comp.mainLink_.send(frame))
        comp.mainSched_.markSent(due->id, now);
      break;
    }
    comp.mainSched_.markSent(due->id, now);
    break;
  }
  default:
    break; // paquets montants : pas schedulés ici (cf. GCSRxTask)
  }
}

void TelSecTask::loop() {
  scanCodeChanges(comp, comp.secSched_, lastMajorCodes_, codePrimed_, pendingCodes_,
                  pendingCodeCount_);

  auto now = Clock::now();
  const auto *due = comp.secSched_.nextDue(now);
  if (!due)
    return;

  switch (due->id) {
  case TELEM::PacketID::HEALTH: {
    auto frame = TELEM::encode(buildHealthPkt(comp, healthCycleIndex_));
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
    TELEM::CodePkt pkt{};
    if (pendingCodeCount_ > 0) {
      PendingCodeEvent ev = pendingCodes_[0];
      for (size_t i = 1; i < pendingCodeCount_; ++i)
        pendingCodes_[i - 1] = pendingCodes_[i];
      pendingCodeCount_--;
      pkt.code = ev.code;
      pkt.active = ev.active ? 1 : 0;
      pkt.ts_ms = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(ev.ts.time_since_epoch())
              .count());
    }
    auto frame = TELEM::encode(pkt);
    if (!comp.secLink_.send(frame)) {
      comp.secSched_.markSent(due->id, now);
      if (pendingCodeCount_ > 0)
        comp.secSched_.markDirty(TELEM::PacketID::CODE);
    }
    break;
  }
  case TELEM::PacketID::ARM_STATE: {
    TELEM::ArmStatePkt pkt{};
    pkt.armed = comp.fc_.getArmed().value_or(false) ? 1 : 0;
    auto frame = TELEM::encode(pkt);
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
  case TELEM::PacketID::CPU_TEMP: {
    TELEM::CpuTempPkt pkt{};
    if (auto temp = comp.tempSource_.readCelsius())
      pkt.temp_c = *temp;
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

// Dispatch commun aux deux liens montants, factorisé plutôt que dupliqué
// dans chaque branche de GCSRxTask::loop(). CMD_EMERGENCY court-circuite
// gcsCmds, cf. SYSMONITORING.md §10.
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
