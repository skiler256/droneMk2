// src/IntegrationTests/UDP_bridge.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Pont shm <-> UDP unique pour les tests d'intégration : s'attache, de façon
// opportuniste et indépendante, aux segments SITL::/HIL::/NAV_SITL::k*ShmPath
// (publiés respectivement par mv_sitl, hil_sf_mv, nav_sitl), ainsi qu'aux
// VRAIS chemins de prod (namespace PROD, kNavShmPath etc. de
// GlobalWatchdog.hpp — publiés par le vrai binaire `drone`, typiquement
// lancé par le SITL_MASTER avec -DDRONE_USE_SITL_DRIVERS). Tous peuvent être
// actifs en même temps ou pas du tout, réessayé périodiquement si un binaire
// démarre après le pont. Réutilise les VRAIS handlers (aucune réimplémentation
// du verrou CAS).
//
// Protocole texte, un datagramme = un segment, ligne 1 "SEGMENT=<NS>_<NOM>"
// (NS = SITL, HIL, NAV_SITL, MC_SITL ou PROD) puis une ligne "cle=valeur" par
// champ — pas de lib JSON, débogable à la main avec `nc -ul <port>`. Deux
// ports localhost (le pont et le serveur web tournent tous les deux sur le
// Pi, seul le navigateur est distant) :
//   - kStatusPort  : le pont ENVOIE ici, ~10Hz, un datagramme par segment actif
//                    — tools/dashboard/dashboard_server.py écoute.
//   - kCommandPort : le pont ÉCOUTE ici les injections de test, "CMD=<NOM>\n
//                    NS=<SITL|HIL|NAV_SITL|MC_SITL|PROD>\ncle=valeur...".
//
// PROD est en lecture seule côté dashboard (pas d'injection GPS/segment :
// les vrais writers — Navigation/SensorFusion/MissionControl réels — sont
// déjà en place, cf. allowGpsMagInject=false).
//
// IMPORTANT : CMD=GPSMAG n'est routable QUE vers SITL. Côté HIL, le vrai
// SensorFusion tourne et est l'unique writer légitime de gpsMag (principe
// un-seul-writer-par-segment) — laisser le pont y écrire aussi créerait une
// race à deux writers. Côté NAV_SITL, aucun SensorFusion ne tourne du tout
// (cf. NAV_SITL.cpp) mais gpsMag n'y sert à rien non plus (GPS simulé laissé
// côté SITL) — rejeté dans les deux cas (log stderr).
//
// Usage : ./udp_bridge (avant ou après mv_sitl/hil_sf_mv/nav_sitl, l'ordre n'importe pas)

#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/Components/System Monitoring/SharedComMem.hpp"
#include "drone/IntegrationTests/ShmPaths.hpp"
#include "drone/core/GlobalWatchdog.hpp" // chemins shm de PROD (kNavShmPath etc.), pas la classe elle-meme
#include "drone/core/SharedSysStateMem.hpp"
#include "drone/core/WatchdogControl.hpp"
#include "drone/generated/codes.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

using namespace TYPES;

namespace {

constexpr uint16_t kStatusPort = 15001;  // pont -> serveur web
constexpr uint16_t kCommandPort = 15000; // serveur web -> pont
constexpr auto kPushPeriod = std::chrono::milliseconds(100);    // 10Hz
constexpr auto kAttachRetryPeriod = std::chrono::seconds(2);    // segments pas encore publiés
constexpr auto kRefreshPeriod = std::chrono::milliseconds(150); // < kMaxSampleAge (300ms) côté TTx

std::atomic<bool> keepRunning{true};

void signalHandler(int) { keepRunning.store(false, std::memory_order_release); }

void installSignalHandler() {
  struct sigaction sa {};
  sa.sa_handler = signalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

int openStatusSocket() {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  return fd; // pas de bind : socket purement émetteur, port source éphémère
}

int openCommandSocket() {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
    return -1;

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(kCommandPort);

  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }

  int flags = ::fcntl(fd, F_GETFL, 0);
  ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  return fd;
}

void sendSegment(int fd, const std::string &text) {
  if (text.empty())
    return;
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(kStatusPort);
  ::sendto(fd, text.data(), text.size(), 0, reinterpret_cast<sockaddr *>(&addr),
          sizeof(addr));
}

std::string serializeFc(std::string_view ns, SharedFCStatusHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=" << ns << "_FC\n";

  if (auto att = h.getAttitude())
    os << "attitude.w=" << att->q.w << "\nattitude.x=" << att->q.x
       << "\nattitude.y=" << att->q.y << "\nattitude.z=" << att->q.z << "\n";

  if (auto pv = h.getLocalPosVel())
    os << "pos.x=" << pv->pos.ned.x << "\npos.y=" << pv->pos.ned.y
       << "\npos.z=" << pv->pos.ned.z << "\nvel.x=" << pv->vel.ned.x
       << "\nvel.y=" << pv->vel.ned.y << "\nvel.z=" << pv->vel.ned.z << "\n";

  if (auto bat = h.getBattery())
    os << "battery.voltage=" << bat->voltage.v << "\nbattery.current=" << bat->current.v
       << "\nbattery.pct=" << static_cast<int>(bat->remainingPct) << "\n";

  if (auto pres = h.getPressure())
    os << "pressure.abs=" << pres->absHpa << "\npressure.diff=" << pres->diffHpa
       << "\npressure.temp=" << pres->temperatureC << "\n";

  if (auto hb = h.getLastHeartbeat()) {
    auto ageMs = UTILITIES::msBetween(*hb, Clock::now());
    os << "heartbeat.age_ms=" << ageMs.count() << "\n";
  }

  return os.str();
}

std::string serializeNav(std::string_view ns, SharedNavMemHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=" << ns << "_NAV\n";

  if (auto cmd = h.getCommand()) {
    auto ageMs = UTILITIES::msBetween(cmd->ts, Clock::now());
    os << "vel.x=" << cmd->velNed.ned.x << "\nvel.y=" << cmd->velNed.ned.y
       << "\nvel.z=" << cmd->velNed.ned.z << "\nheading=" << cmd->headingCmd.v
       << "\naltitude=" << cmd->altitudeCmd.v << "\nage_ms=" << ageMs.count() << "\n";
  }

  // FMA — noms de code seulement (pas de sévérité/composant), suffisant
  // pour un affichage dashboard.
  auto codeName = [](std::optional<CODES::Code> c) -> std::string {
    if (!c || *c == CODES::Code{})
      return "-";
    const auto *info = CODES::find(*c);
    return info ? info->name : "?";
  };
  os << "fma.lateral_armed=" << codeName(h.getLateralArmed()) << "\n";
  os << "fma.lateral_engaged=" << codeName(h.getLateralEngaged()) << "\n";
  os << "fma.vertical_armed=" << codeName(h.getVerticalArmed()) << "\n";
  os << "fma.vertical_engaged=" << codeName(h.getVerticalEngaged()) << "\n";
  os << "fma.speed_engaged=" << codeName(h.getSpeedEngaged()) << "\n";

  return os.str();
}

// Segment/avancement — indépendant de NavCommand, cf. SharedCmdMem.
// `lastReport` : dernier CommandReport dépilé de navReports (TSeq y pousse
// le résultat de chaque ordre navCmds, ex: échec canEngage() sur amorçage)
// — sans ça, un échec de promotion est invisible côté dashboard.
std::string serializeSegment(std::string_view ns, SharedCmdMemHandler &h,
                             const std::optional<CommandReport> &lastReport) {
  std::ostringstream os;
  os << "SEGMENT=" << ns << "_SEG\n";

  if (auto armed = h.getArmedSegment(); armed && armed->seq != 0) {
    os << "armed.seq=" << armed->seq << "\narmed.entry_n=" << armed->entryWp.ned.x
       << "\narmed.entry_e=" << armed->entryWp.ned.y
       << "\narmed.exit_n=" << armed->exitWp.ned.x
       << "\narmed.exit_e=" << armed->exitWp.ned.y << "\n";
  }
  if (auto engaged = h.getEngagedSegment(); engaged && engaged->seq != 0) {
    os << "engaged.seq=" << engaged->seq << "\nengaged.exit_n=" << engaged->exitWp.ned.x
       << "\nengaged.exit_e=" << engaged->exitWp.ned.y << "\n";
  }
  if (auto progress = h.getSegmentProgress()) {
    os << "progress.seq=" << progress->seq
       << "\nprogress.status=" << static_cast<int>(progress->status)
       << "\nprogress.dist=" << progress->distanceRestante.v << "\n";
  }
  if (lastReport) {
    os << "report.seq=" << lastReport->seq
       << "\nreport.success=" << (lastReport->success ? 1 : 0) << "\n";
  }

  return os.str();
}

std::string serializeSf(std::string_view ns, SharedSFMemHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=" << ns << "_SF\n";

  if (auto gps = h.getGpsMag()) {
    auto ageMs = UTILITIES::msBetween(gps->ts, Clock::now());
    os << "gps.lat=" << gps->latitude << "\ngps.lon=" << gps->longitude
       << "\ngps.alt=" << gps->altMsl << "\ngps.vel_n=" << gps->velNed.ned.x
       << "\ngps.vel_e=" << gps->velNed.ned.y << "\ngps.vel_d=" << gps->velNed.ned.z
       << "\ngps.hdop=" << gps->hdop
       << "\ngps.fix_type=" << static_cast<int>(gps->fixType)
       << "\ngps.satellites=" << static_cast<int>(gps->satellitesVisible)
       << "\ngps.age_ms=" << ageMs.count() << "\n";
  }

  if (auto state = h.getDynamicState()) {
    auto ageMs = UTILITIES::msBetween(state->ts, Clock::now());
    os << "state.pos.x=" << state->posNed.ned.x << "\nstate.pos.y=" << state->posNed.ned.y
       << "\nstate.pos.z=" << state->posNed.ned.z << "\nstate.vel.x=" << state->velNed.ned.x
       << "\nstate.vel.y=" << state->velNed.ned.y << "\nstate.vel.z=" << state->velNed.ned.z
       << "\nstate.age_ms=" << ageMs.count() << "\n";
  }

  return os.str();
}

// Points non-nuls seulement (sentinelle "pas de mesure", cf. LidarMap) —
// réduit la taille du datagramme, un tour complet peut sinon dépasser 25Ko.
std::string serializeLidar(std::string_view ns, SharedSFMemHandler &h) {
  auto map = h.getLidarMap();
  if (!map)
    return "";

  std::ostringstream os;
  os << "SEGMENT=" << ns << "_LIDAR\n";
  auto ageMs = UTILITIES::msBetween(map->ts, Clock::now());
  os << "age_ms=" << ageMs.count() << "\n";

  int count = 0;
  for (std::size_t i = 0; i < map->points.size(); ++i) {
    if (map->points[i].isZero())
      continue;
    os << "pt." << count << ".x=" << map->points[i].x << "\npt." << count
       << ".y=" << map->points[i].y << "\npt." << count << ".z=" << map->points[i].z
       << "\npt." << count << ".i=" << static_cast<int>(map->intensity[i]) << "\n";
    ++count;
  }
  os << "count=" << count << "\n";

  return os.str();
}

// Santé + codes majeurs actifs (SF/MavlinkInterface — les seuls composants
// réellement présents dans mv_sitl/hil_sf_mv). Codes mineurs (ring) omis :
// trop bruyants pour un dump périodique (ex: SFS001 relevé à chaque lecture
// GPS valide, cf. SensorFusion.cpp).
std::string serializeSys(std::string_view ns, SharedSysStateMemHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=" << ns << "_SYS\n";

  auto dumpComponent = [&](std::string_view label, ComponentID id) {
    if (auto health = h.getHealth(id))
      os << label << ".health=" << static_cast<int>(*health) << "\n";
    if (auto codes = h.getMajorCodes(id)) {
      int n = 0;
      for (const auto &entry : *codes) {
        if (entry.code == 0)
          continue;
        auto code = CODES::Code::unpack(entry.code);
        const auto *info = CODES::find(code);
        os << label << ".code." << n << "=" << (info ? info->name : "?") << "\n";
        ++n;
      }
    }
  };
  dumpComponent("mavlink", ComponentID::MavlinkInterface);
  dumpComponent("sensorfusion", ComponentID::SensorFusion);
  dumpComponent("navigation", ComponentID::Navigation);
  dumpComponent("sysmonitoring", ComponentID::SysMonitoring);
  dumpComponent("missioncontrol", ComponentID::MissionControl);

  return os.str();
}

[[nodiscard]] std::string_view missionStateName(MissionState s) {
  switch (s) {
  case MissionState::Idle: return "Idle";
  case MissionState::PreFlight: return "PreFlight";
  case MissionState::Armed: return "Armed";
  case MissionState::Mission: return "Mission";
  case MissionState::Manual: return "Manual";
  case MissionState::Hold: return "Hold";
  case MissionState::Landing: return "Landing";
  case MissionState::Emergency: return "Emergency";
  }
  return "?";
}

[[nodiscard]] std::string_view queryTypeName(QueryType t) {
  switch (t) {
  case QueryType::None: return "None";
  case QueryType::ObstacleNonContournable: return "ObstacleNonContournable";
  case QueryType::EmergencyRecoverable: return "EmergencyRecoverable";
  case QueryType::LidarLost: return "LidarLost";
  }
  return "?";
}

// État MissionControl (MissionState + question GCS en cours) — vit dans
// SharedCmdMem comme le reste (armé/engagé/progress, cf. serializeSegment),
// même handler `cmd`, pas de segment shm dédié.
std::string serializeMc(std::string_view ns, SharedCmdMemHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=" << ns << "_MC\n";

  if (auto st = h.getMissionState())
    os << "state=" << missionStateName(*st) << "\n";

  if (auto q = h.getPendingQuery(); q && q->type != QueryType::None)
    os << "query.type=" << queryTypeName(q->type) << "\nquery.seq=" << q->seq << "\n";

  if (auto hb = h.getHeartbeat()) {
    auto ageMs = UTILITIES::msBetween(*hb, Clock::now());
    os << "heartbeat.age_ms=" << ageMs.count() << "\n";
  }

  return os.str();
}

// Heartbeat GCS (SharedComMem) — MC_SITL uniquement, GCSRxTask n'existe
// pas dans ce binaire (pas de SysMonitoring), donc rien d'autre ne pousse
// jamais lastHeartbeatMain/Sec ici (les injecter via CMD=GCSCMD est une
// autre affaire — ce champ mesure l'âge, pas la présence de commandes).
std::string serializeCom(std::string_view ns, SharedComMemHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=" << ns << "_COM\n";

  if (auto hb = h.getLastHeartbeatMain()) {
    auto ageMs = UTILITIES::msBetween(*hb, Clock::now());
    os << "heartbeat_main.age_ms=" << ageMs.count() << "\n";
  }
  if (auto hb = h.getLastHeartbeatSec()) {
    auto ageMs = UTILITIES::msBetween(*hb, Clock::now());
    os << "heartbeat_sec.age_ms=" << ageMs.count() << "\n";
  }

  return os.str();
}

// SharedWatchdogControl n'a pas de handler (pas de verrou CAS/checksum, cf.
// WatchdogControl.hpp) : lecture directe du ptr shm, atomics individuels.
std::string serializeWatchdogControl(std::string_view ns, SharedWatchdogControl &wdc) {
  std::ostringstream os;
  os << "SEGMENT=" << ns << "_WDC\n";

  auto dump = [&](std::string_view label, ComponentID id) {
    bool suppressed = wdc.suppressRespawn[static_cast<std::size_t>(id)].load(
        std::memory_order_acquire);
    os << label << ".suppressed=" << (suppressed ? 1 : 0) << "\n";
  };
  dump("mavlink", ComponentID::MavlinkInterface);
  dump("sensorfusion", ComponentID::SensorFusion);
  dump("navigation", ComponentID::Navigation);
  dump("sysmonitoring", ComponentID::SysMonitoring);
  dump("missioncontrol", ComponentID::MissionControl);

  return os.str();
}

// Parse un datagramme "cle=valeur\n..." en map — utilisé pour les commandes
// entrantes (CMD=NAV/CMD=GPSMAG/CMD=MCCMD).
std::unordered_map<std::string, std::string> parseLines(std::string_view text) {
  std::unordered_map<std::string, std::string> fields;
  size_t pos = 0;
  while (pos < text.size()) {
    size_t eol = text.find('\n', pos);
    if (eol == std::string_view::npos)
      eol = text.size();
    std::string_view line = text.substr(pos, eol - pos);
    size_t eq = line.find('=');
    if (eq != std::string_view::npos)
      fields.emplace(std::string(line.substr(0, eq)), std::string(line.substr(eq + 1)));
    pos = eol + 1;
  }
  return fields;
}

[[nodiscard]] float parseFloatOr(const std::unordered_map<std::string, std::string> &m,
                                 const char *key, float fallback) {
  auto it = m.find(key);
  if (it == m.end())
    return fallback;
  float value = fallback;
  std::from_chars(it->second.data(), it->second.data() + it->second.size(), value);
  return value;
}

// gpsWeek/gpsTowMs pour l'échantillon injecté : pas de vrai récepteur GPS
// ici (SITL simulé), donc on part de l'horloge murale du Pi plutôt que de
// l'UBX NAV-PVT/TIMEUTC (cf. GPSTask::loop() côté SensorFusion) — laisser
// ces champs à 0 réintroduirait le bug "GPS_INPUT.time_week=0" (ArduPilot
// peut rejeter la position comme invalide).
void fillGpsWeek(GpsMagSample &sample) {
  using namespace std::chrono;
  auto sinceGpsEpoch = system_clock::now() - sys_days{year{1980} / January / 6};
  auto msSinceEpoch = duration_cast<milliseconds>(sinceGpsEpoch).count();
  constexpr int64_t kMsPerWeek = 7LL * 24 * 3600 * 1000;
  sample.gpsWeek = static_cast<uint16_t>(msSinceEpoch / kMsPerWeek);
  sample.gpsTowMs = static_cast<uint32_t>(msSinceEpoch % kMsPerWeek);
}

// Un jeu de segments attaché de façon opportuniste (SITL ou HIL) — chaque
// membre est absent tant que le binaire propriétaire (mv_sitl/hil_sf_mv) n'a
// pas encore publié le segment correspondant, ou n'a jamais tourné.
struct TestNamespace {
  std::string_view label; // "SITL" ou "HIL", préfixe des segments envoyés
  bool allowGpsMagInject; // faux pour HIL : SensorFusion réel est déjà writer

  std::string_view navPath, sfPath, sysPath, fcPath, cmdPath, comPath;
  // Absent (vide) partout sauf PROD — seul GlobalWatchdog publie ce segment,
  // cf. WatchdogControl.hpp / GlobalWatchdog.hpp.
  std::string_view wdcPath;

  std::optional<ShmPublisher<SharedNavMem>> navShm;
  std::optional<ShmPublisher<SharedSFMem>> sfShm;
  std::optional<ShmPublisher<SharedSysStateMem>> sysShm;
  std::optional<ShmPublisher<SharedFCStatus>> fcShm;
  std::optional<ShmPublisher<SharedCmdMem>> cmdShm;
  std::optional<ShmPublisher<SharedComMem>> comShm;
  std::optional<ShmPublisher<SharedWatchdogControl>> wdcShm;

  std::optional<SharedNavMemHandler> nav;
  std::optional<SharedSFMemHandler> sf;
  std::optional<SharedSysStateMemHandler> sys;
  std::optional<SharedFCStatusHandler> fc;
  std::optional<SharedCmdMemHandler> cmd;
  // Absent (comPath vide) pour SITL/HIL/NAV_SITL — seul MC_SITL en a besoin
  // (injection de commandes GCS, cf. CMD=GCSCMD).
  std::optional<SharedComMemHandler> com;
  // Pas de handler dédié : SharedWatchdogControl n'a pas de verrou CAS/
  // checksum (cf. WatchdogControl.hpp), lu directement via wdcShm->ptr.

  // Rafraîchissement périodique de la dernière commande injectée — un
  // NavCommand/GpsMagSample écrit une seule fois expire côté ArduPilot
  // (GUIDED/GPS_INPUT attendent un flux continu), le pont simule donc les
  // boucles Navigation/SensorFusion qui réécriraient sinon en continu.
  std::optional<NavCommand> lastNavCmd;
  std::optional<GpsMagSample> lastGpsMag;

  // Dernier report navCmds dépilé — cf. serializeSegment().
  std::optional<CommandReport> lastNavReport;

  // Dépile navReports (non-bloquant) pour affichage — sans ça, un échec
  // silencieux de promotion (canEngage() par ex.) n'est visible nulle part.
  void pollReports() {
    if (cmd) {
      if (auto r = cmd->popNavReport())
        lastNavReport = r;
    }
  }

  // Tente d'attacher les segments pas encore attachés — no-op sur ceux déjà
  // là. Appelé au démarrage puis périodiquement (cf. kAttachRetryPeriod).
  void tryAttachMissing() {
    if (!nav) {
      if (auto r = attachSharedMemory<SharedNavMem>(navPath)) {
        navShm = *r;
        nav.emplace(ComponentID::GlobalWatchdog, Us(2000), *navShm->ptr);
      }
    }
    if (!sf) {
      if (auto r = attachSharedMemory<SharedSFMem>(sfPath)) {
        sfShm = *r;
        sf.emplace(ComponentID::GlobalWatchdog, Us(2000), *sfShm->ptr);
      }
    }
    if (!sys) {
      if (auto r = attachSharedMemory<SharedSysStateMem>(sysPath)) {
        sysShm = *r;
        sys.emplace(*sysShm->ptr, ComponentID::GlobalWatchdog, Us(2000));
      }
    }
    if (!fc) {
      if (auto r = attachSharedMemory<SharedFCStatus>(fcPath)) {
        fcShm = *r;
        fc.emplace(ComponentID::GlobalWatchdog, Us(2000), *fcShm->ptr);
      }
    }
    if (!cmd) {
      if (auto r = attachSharedMemory<SharedCmdMem>(cmdPath)) {
        cmdShm = *r;
        cmd.emplace(ComponentID::GlobalWatchdog, Us(2000), *cmdShm->ptr);
      }
    }
    if (!com && !comPath.empty()) {
      if (auto r = attachSharedMemory<SharedComMem>(comPath)) {
        comShm = *r;
        com.emplace(ComponentID::GlobalWatchdog, Us(2000), *comShm->ptr);
      }
    }
    if (!wdcShm && !wdcPath.empty()) {
      if (auto r = attachSharedMemory<SharedWatchdogControl>(wdcPath))
        wdcShm = *r;
    }
  }

  void pushStatus(int statusFd) {
    if (fc)
      sendSegment(statusFd, serializeFc(label, *fc));
    if (nav)
      sendSegment(statusFd, serializeNav(label, *nav));
    if (sf) {
      sendSegment(statusFd, serializeSf(label, *sf));
      sendSegment(statusFd, serializeLidar(label, *sf));
    }
    if (sys)
      sendSegment(statusFd, serializeSys(label, *sys));
    if (cmd) {
      sendSegment(statusFd, serializeSegment(label, *cmd, lastNavReport));
      sendSegment(statusFd, serializeMc(label, *cmd));
    }
    if (com)
      sendSegment(statusFd, serializeCom(label, *com));
    if (wdcShm)
      sendSegment(statusFd, serializeWatchdogControl(label, *wdcShm->ptr));
  }

  void refreshInjectedCommands(TYPES::TimePoint now) {
    if (lastNavCmd && nav) {
      lastNavCmd->ts = now;
      nav->setCommand(*lastNavCmd);
    }
    if (lastGpsMag && sf) {
      lastGpsMag->ts = now;
      fillGpsWeek(*lastGpsMag); // doit avancer avec le temps réel, pas rester figé
      sf->setGpsMag(*lastGpsMag);
    }
  }
};

// Tables de correspondance texte -> enum pour l'injection MC_SITL — noms
// choisis pour matcher directement les identifiants C++/CODES::, pas de
// couche d'indirection cosmétique côté dashboard.
[[nodiscard]] std::optional<ComponentID> parseComponentId(std::string_view s) {
  if (s == "Navigation") return ComponentID::Navigation;
  if (s == "SensorFusion") return ComponentID::SensorFusion;
  if (s == "MavlinkInterface") return ComponentID::MavlinkInterface;
  if (s == "SysMonitoring") return ComponentID::SysMonitoring;
  if (s == "MissionControl") return ComponentID::MissionControl;
  return std::nullopt;
}

// Liste volontairement fermée aux codes pertinents pour tester le failsafe
// MC (cf. MISSIONCONTROL.md §6) — pas un lookup générique par nom (pas
// construit côté CODES::, et ouvrirait à injecter n'importe quoi).
[[nodiscard]] std::optional<CODES::Code> parseInjectableCode(std::string_view s) {
  if (s == "SFS002") return CODES::SFS002; // GPS perdu (recoverable)
  if (s == "SFS004") return CODES::SFS004; // Lidar perdu (ni l'un ni l'autre)
  if (s == "SFS005") return CODES::SFS005; // EKF mort (recoverable)
  if (s == "MVC004") return CODES::MVC004; // Lien FC perdu (recoverable)
  if (s == "MVP001") return CODES::MVP001; // RPM bas (unrecoverable)
  if (s == "FCP003") return CODES::FCP003; // Batterie critique (unrecoverable)
  if (s == "SMO001") return CODES::SMO001; // E-stop GCS (unrecoverable)
  if (s == "SMC002") return CODES::SMC002; // Lien GCS principal perdu
  if (s == "SMC003") return CODES::SMC003; // Lien GCS secondaire perdu (les 2 = recoverable)
  if (s == "NVA004") return CODES::NVA004; // Obstacle (-> HOLD)
  return std::nullopt;
}

[[nodiscard]] std::optional<ComponentHealth> parseHealth(std::string_view s) {
  if (s == "NOMINAL") return ComponentHealth::NOMINAL;
  if (s == "DEGRADED") return ComponentHealth::DEGRADED;
  if (s == "SICK") return ComponentHealth::SICK;
  if (s == "DEAD") return ComponentHealth::DEAD;
  return std::nullopt;
}

[[nodiscard]] std::optional<GCSCommandType> parseGcsCmdType(std::string_view s) {
  if (s == "Arm") return GCSCommandType::Arm;
  if (s == "Disarm") return GCSCommandType::Disarm;
  if (s == "Takeoff") return GCSCommandType::Takeoff;
  if (s == "Mode") return GCSCommandType::Mode;
  if (s == "Hold") return GCSCommandType::Hold;
  if (s == "Resume") return GCSCommandType::Resume;
  if (s == "Land") return GCSCommandType::Land;
  if (s == "Answer") return GCSCommandType::Answer;
  return std::nullopt;
}

// Applique une commande reçue en UDP (CMD=NAV, CMD=GPSMAG, CMD=MCCMD,
// CMD=SEGMENT, CMD=GCSCMD, CMD=CODE, CMD=HEALTH, CMD=FCARMED) sur le
// namespace ciblé (NS=SITL|HIL|NAV_SITL|MC_SITL) — testable avec :
// echo -e "CMD=NAV\nNS=SITL\nvel.x=1.0" | nc -u -w0 127.0.0.1 15000
// echo -e "CMD=MCCMD\nNS=HIL\ncode=takeoff\npayload=500" | nc -u -w0 127.0.0.1 15000
// echo -e "CMD=SEGMENT\nNS=NAV_SITL\nnorth=20\neast=0\nalt=-10\nspeed=3" | nc -u -w0 127.0.0.1 15000
// echo -e "CMD=GCSCMD\nNS=MC_SITL\ntype=Arm" | nc -u -w0 127.0.0.1 15000
// echo -e "CMD=CODE\nNS=MC_SITL\ncomponent=SensorFusion\ncode=SFS002\naction=raise" | nc -u -w0 127.0.0.1 15000
void handleCommand(std::string_view text, TestNamespace &sitlNs, TestNamespace &hilNs,
                   TestNamespace &navSitlNs, TestNamespace &mcSitlNs,
                   TestNamespace &prodNs) {
  auto fields = parseLines(text);
  auto cmdIt = fields.find("CMD");
  auto nsIt = fields.find("NS");
  if (cmdIt == fields.end() || nsIt == fields.end())
    return;

  TestNamespace &ns = (nsIt->second == "HIL")        ? hilNs
                      : (nsIt->second == "NAV_SITL")  ? navSitlNs
                      : (nsIt->second == "MC_SITL")   ? mcSitlNs
                      : (nsIt->second == "PROD")      ? prodNs
                                                      : sitlNs;

  if (cmdIt->second == "MCCMD") {
    if (!ns.cmd)
      return;
    auto codeIt = fields.find("code");
    if (codeIt == fields.end())
      return;

    // start_segment cible Nav (navCmds), les autres ciblent MavlinkInterface
    // (mavlinkCmds) — deux FIFO distinctes, cf. SharedCmdMem.
    if (codeIt->second == "start_segment") {
      [[maybe_unused]] auto pushed = ns.cmd->pushNavCommand(CODES::MCO005, 0);
      return;
    }

    CODES::Code code{};
    if (codeIt->second == "takeoff")
      code = CODES::MCO001;
    else if (codeIt->second == "guided")
      code = CODES::MCO002;
    else if (codeIt->second == "arm")
      code = CODES::MCO003;
    else if (codeIt->second == "disarm")
      code = CODES::MCO004;
    else
      return; // code inconnu du dashboard, ignoré

    uint32_t payload = static_cast<uint32_t>(parseFloatOr(fields, "payload", 0.0f));
    [[maybe_unused]] auto pushed = ns.cmd->pushMavlinkCommand(code, payload);
  } else if (cmdIt->second == "SEGMENT") {
    if (!ns.cmd)
      return;
    // entry_n/entry_e par défaut à 0 : suffisant pour un test simple où le
    // drone démarre près de l'origine EKF (cf. NAV_SITL.cpp).
    NavSegment seg{
        .type = SegmentType::Waypoint,
        .entryWp = Position{Vec3{parseFloatOr(fields, "entry_n", 0.0f),
                                 parseFloatOr(fields, "entry_e", 0.0f), 0.0f}},
        .exitWp = Position{Vec3{parseFloatOr(fields, "north", 0.0f),
                                parseFloatOr(fields, "east", 0.0f), 0.0f}},
        .speed = MetersPerSec{parseFloatOr(fields, "speed", 2.0f)},
        .targetAltitude = Meters{parseFloatOr(fields, "alt", -5.0f)}};
    ns.cmd->setArmedSegment(seg);
  } else if (cmdIt->second == "NAV") {
    if (!ns.nav)
      return;
    // vel.z n'est plus un paramètre : vz est calculé par MavlinkInterface
    // (P-controller sur l'écart d'altitude, cf. altitudeCmd).
    NavCommand cmd{.ts = Clock::now(),
                  .velNed = Velocity{Vec3{parseFloatOr(fields, "vel.x", 0.0f),
                                          parseFloatOr(fields, "vel.y", 0.0f), 0.0f}},
                  .headingCmd = Radians{parseFloatOr(fields, "heading", 0.0f)},
                  .altitudeCmd = Meters{parseFloatOr(fields, "altitude", 0.0f)}};
    ns.nav->setCommand(cmd);
    ns.lastNavCmd = cmd;
  } else if (cmdIt->second == "GPSMAG") {
    if (!ns.sf)
      return;
    if (!ns.allowGpsMagInject) {
      std::cerr << "UDP_bridge: CMD=GPSMAG refusé pour " << ns.label
               << " (SensorFusion réel est déjà writer de gpsMag)\n";
      return;
    }
    GpsMagSample sample{
        .ts = Clock::now(),
        .latitude = parseFloatOr(fields, "lat", 0.0f),
        .longitude = parseFloatOr(fields, "lon", 0.0f),
        .altMsl = parseFloatOr(fields, "alt", 0.0f),
        .velNed = Velocity{Vec3{parseFloatOr(fields, "vel_n", 0.0f),
                                parseFloatOr(fields, "vel_e", 0.0f),
                                parseFloatOr(fields, "vel_d", 0.0f)}},
        .fixType = static_cast<uint8_t>(parseFloatOr(fields, "fix_type", 3.0f)),
        .satellitesVisible = static_cast<uint8_t>(parseFloatOr(fields, "satellites", 10.0f))};
    fillGpsWeek(sample);
    ns.sf->setGpsMag(sample);
    ns.lastGpsMag = sample;
  } else if (cmdIt->second == "GCSCMD") {
    // Simule une commande GCS -> MissionControl, en direct dans
    // SharedComMem::gcsCmds — remplace GCSRxTask (absent, pas de
    // SysMonitoring dans MC_SITL, cf. MC_SITL.cpp).
    if (!ns.com)
      return;
    auto typeIt = fields.find("type");
    if (typeIt == fields.end())
      return;
    auto type = parseGcsCmdType(typeIt->second);
    if (!type)
      return;

    GCSCommand c{.type = *type};
    c.mode = static_cast<uint8_t>(parseFloatOr(fields, "mode", 0.0f));
    c.altitude = Meters{parseFloatOr(fields, "altitude", 0.0f)};
    c.answerSeq = static_cast<uint32_t>(parseFloatOr(fields, "answerSeq", 0.0f));
    c.answerChoice = static_cast<uint8_t>(parseFloatOr(fields, "answerChoice", 0.0f));
    [[maybe_unused]] auto pushed = ns.com->pushGCSCommand(c);
  } else if (cmdIt->second == "CODE") {
    // Lève/efface un code sous l'identité d'un AUTRE composant (self-report
    // oblige, cf. SharedSysStateMemHandler) — simule "SensorFusion a levé
    // SFS002" sans faire tourner SensorFusion pour de vrai.
    if (!ns.sysShm)
      return;
    auto compIt = fields.find("component");
    auto codeIt = fields.find("code");
    auto actionIt = fields.find("action");
    if (compIt == fields.end() || codeIt == fields.end() || actionIt == fields.end())
      return;
    auto compId = parseComponentId(compIt->second);
    auto code = parseInjectableCode(codeIt->second);
    if (!compId || !code)
      return;

    SharedSysStateMemHandler fake(*ns.sysShm->ptr, *compId, Us(2000));
    if (actionIt->second == "raise")
      fake.raiseCode(*code);
    else if (actionIt->second == "clear")
      fake.clearCode(*code);
  } else if (cmdIt->second == "HEALTH") {
    // Même principe que CODE — simule un composant DEAD/SICK sans le faire
    // vraiment crasher (cf. escalade sur crash, MISSIONCONTROL.md §6).
    if (!ns.sysShm)
      return;
    auto compIt = fields.find("component");
    auto healthIt = fields.find("health");
    if (compIt == fields.end() || healthIt == fields.end())
      return;
    auto compId = parseComponentId(compIt->second);
    auto health = parseHealth(healthIt->second);
    if (!compId || !health)
      return;

    SharedSysStateMemHandler fake(*ns.sysShm->ptr, *compId, Us(2000));
    fake.setHealth(*health);
  } else if (cmdIt->second == "FCARMED") {
    // Rien d'autre ne confirme jamais l'armement dans MC_SITL (pas de
    // MavlinkInterface) — sans ce toggle, ARMED->MISSION resterait bloqué
    // indéfiniment (armError sur timeout de confirmation).
    if (!ns.fc)
      return;
    bool armed = parseFloatOr(fields, "armed", 0.0f) != 0.0f;
    ns.fc->setArmed(armed);
  }
}

} // namespace

int main() {
  installSignalHandler();

  TestNamespace sitlNs{};
  sitlNs.label = "SITL";
  sitlNs.allowGpsMagInject = true;
  sitlNs.navPath = SITL::kNavShmPath;
  sitlNs.sfPath = SITL::kSFShmPath;
  sitlNs.sysPath = SITL::kSysShmPath;
  sitlNs.fcPath = SITL::kFCShmPath;
  sitlNs.cmdPath = SITL::kCmdShmPath;

  TestNamespace hilNs{};
  hilNs.label = "HIL";
  hilNs.allowGpsMagInject = false;
  hilNs.navPath = HIL::kNavShmPath;
  hilNs.sfPath = HIL::kSFShmPath;
  hilNs.sysPath = HIL::kSysShmPath;
  hilNs.fcPath = HIL::kFCShmPath;
  hilNs.cmdPath = HIL::kCmdShmPath;

  TestNamespace navSitlNs{};
  navSitlNs.label = "NAV_SITL";
  navSitlNs.allowGpsMagInject = false; // pas de SensorFusion, gpsMag inutile ici
  navSitlNs.navPath = NAV_SITL::kNavShmPath;
  navSitlNs.sfPath = NAV_SITL::kSFShmPath;
  navSitlNs.sysPath = NAV_SITL::kSysShmPath;
  navSitlNs.fcPath = NAV_SITL::kFCShmPath;
  navSitlNs.cmdPath = NAV_SITL::kCmdShmPath;
  navSitlNs.comPath = NAV_SITL::kComShmPath;

  TestNamespace mcSitlNs{};
  mcSitlNs.label = "MC_SITL";
  mcSitlNs.allowGpsMagInject = false; // pas de SensorFusion dans MC_SITL
  mcSitlNs.navPath = MC_SITL::kNavShmPath;
  mcSitlNs.sfPath = MC_SITL::kSFShmPath;
  mcSitlNs.sysPath = MC_SITL::kSysShmPath;
  mcSitlNs.fcPath = MC_SITL::kFCShmPath;
  mcSitlNs.cmdPath = MC_SITL::kCmdShmPath;
  mcSitlNs.comPath = MC_SITL::kComShmPath;

  // PROD : le vrai binaire `drone`/GlobalWatchdog, lancé par SITL_MASTER
  // (option DRONE_USE_SITL_DRIVERS) — attache les VRAIS chemins de prod
  // (cf. GlobalWatchdog.hpp), pas un namespace de test dédié. Dashboard
  // uniquement : pas d'injection GPS/segment (writers réels déjà en place,
  // même raison que allowGpsMagInject=false sur HIL).
  TestNamespace prodNs{};
  prodNs.label = "PROD";
  prodNs.allowGpsMagInject = false;
  prodNs.navPath = kNavShmPath;
  prodNs.sfPath = kSFShmPath;
  prodNs.sysPath = kSysShmPath;
  prodNs.fcPath = kFCShmPath;
  prodNs.cmdPath = kCmdShmPath;
  prodNs.comPath = kComShmPath;
  prodNs.wdcPath = kWatchdogControlShmPath;

  sitlNs.tryAttachMissing();
  hilNs.tryAttachMissing();
  navSitlNs.tryAttachMissing();
  mcSitlNs.tryAttachMissing();
  prodNs.tryAttachMissing();

  int statusFd = openStatusSocket();
  int commandFd = openCommandSocket();
  if (statusFd < 0 || commandFd < 0) {
    std::cerr << "UDP_bridge: echec ouverture socket\n";
    return 1;
  }

  std::cout << "UDP_bridge: pret — statut vers " << kStatusPort << ", commandes sur "
           << kCommandPort << " — SITL " << (sitlNs.fc ? "actif" : "absent") << ", HIL "
           << (hilNs.fc ? "actif" : "absent") << ", NAV_SITL "
           << (navSitlNs.fc ? "actif" : "absent") << ", MC_SITL "
           << (mcSitlNs.fc ? "actif" : "absent") << ", PROD "
           << (prodNs.fc ? "actif" : "absent") << "\n";

  auto nextPush = Clock::now();
  auto nextRefresh = Clock::now();
  auto nextAttachRetry = Clock::now() + kAttachRetryPeriod;
  std::array<std::byte, 512> cmdBuf{};

  while (keepRunning.load(std::memory_order_acquire)) {
    auto now = Clock::now();

    if (now >= nextPush) {
      sitlNs.pollReports();
      hilNs.pollReports();
      navSitlNs.pollReports();
      mcSitlNs.pollReports();
      prodNs.pollReports();
      sitlNs.pushStatus(statusFd);
      hilNs.pushStatus(statusFd);
      navSitlNs.pushStatus(statusFd);
      mcSitlNs.pushStatus(statusFd);
      prodNs.pushStatus(statusFd);
      nextPush = now + kPushPeriod;
    }

    if (now >= nextRefresh) {
      sitlNs.refreshInjectedCommands(now);
      hilNs.refreshInjectedCommands(now);
      navSitlNs.refreshInjectedCommands(now);
      mcSitlNs.refreshInjectedCommands(now);
      nextRefresh = now + kRefreshPeriod;
    }

    if (now >= nextAttachRetry) {
      sitlNs.tryAttachMissing();
      hilNs.tryAttachMissing();
      navSitlNs.tryAttachMissing();
      mcSitlNs.tryAttachMissing();
      prodNs.tryAttachMissing();
      nextAttachRetry = now + kAttachRetryPeriod;
    }

    ssize_t n = ::recv(commandFd, cmdBuf.data(), cmdBuf.size(), 0);
    if (n > 0) {
      handleCommand(std::string_view(reinterpret_cast<const char *>(cmdBuf.data()),
                                     static_cast<size_t>(n)),
                    sitlNs, hilNs, navSitlNs, mcSitlNs, prodNs);
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      std::cerr << "UDP_bridge: erreur lecture commande\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ::close(statusFd);
  ::close(commandFd);
  std::cout << "UDP_bridge: arret\n";
  return 0;
}
