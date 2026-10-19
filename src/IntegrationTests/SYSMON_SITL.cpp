// Test : SysMonitoring seul, alimenté par injection UDP texte sur le port
// 15020 — observe le flux de télémétrie réel sur le GCS sans monter le reste
// du système. Tourne sur les MÊMES ports que main.cpp, donc le GCS ne
// demande aucune reconfiguration.
// Usage : ./sysmon_sitl — commandes d'injection : SYSMONITORING.md §8.

#include "drone/Components/SystemMonitoring/Driver/LinuxThermalZoneDriver.hpp"
#include "drone/Components/SystemMonitoring/Driver/UdpTelemetryDriver.hpp"
#include "drone/Components/SystemMonitoring/Driver/UdpVideoDriver.hpp"
#include "drone/Components/SystemMonitoring/SysMonitoring.hpp"
#include "drone/IntegrationTests/ShmPaths.hpp"
#include "drone/generated/codes.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"

#include <atomic>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace TYPES;

namespace {

// Mêmes ports que src/main.cpp::runSysMonitoring() — le GCS observe ce
// binaire exactement comme le `drone` réel, sans reconfiguration.
constexpr uint16_t kUdpTelMainLocalPort = 5601;
constexpr uint16_t kUdpTelMainPeerPort = 5602;
constexpr uint16_t kUdpTelSecLocalPort = 5611;
constexpr uint16_t kUdpTelSecPeerPort = 5612;
constexpr uint16_t kUdpVideoLocalPort = 5621;
constexpr uint16_t kInjectPort = 15020;

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

[[nodiscard]] uint32_t parseU32Or(const std::unordered_map<std::string, std::string> &m,
                                  const char *key, uint32_t fallback) {
  auto it = m.find(key);
  if (it == m.end())
    return fallback;
  uint32_t value = fallback;
  std::from_chars(it->second.data(), it->second.data() + it->second.size(), value);
  return value;
}

// Convention ZYX (yaw->pitch->roll), degrés en entrée — seule cette
// direction (RPY -> quaternion) est nécessaire ici, Attitude::roll()/
// pitch() font déjà l'inverse (cf. types.hpp) mais pas ce sens-ci.
[[nodiscard]] Quat quatFromRpyDeg(float rollDeg, float pitchDeg, float yawDeg) {
  float r = rollDeg * 3.14159265f / 180.0f * 0.5f;
  float p = pitchDeg * 3.14159265f / 180.0f * 0.5f;
  float y = yawDeg * 3.14159265f / 180.0f * 0.5f;
  float cr = std::cos(r), sr = std::sin(r);
  float cp = std::cos(p), sp = std::sin(p);
  float cy = std::cos(y), sy = std::sin(y);
  return Quat{cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
             cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

[[nodiscard]] MissionState parseMissionState(std::string_view s) {
  if (s == "PreFlight") return MissionState::PreFlight;
  if (s == "Armed") return MissionState::Armed;
  if (s == "Mission") return MissionState::Mission;
  if (s == "Manual") return MissionState::Manual;
  if (s == "Hold") return MissionState::Hold;
  if (s == "Landing") return MissionState::Landing;
  if (s == "Emergency") return MissionState::Emergency;
  return MissionState::Idle;
}

// Mêmes noms que UDP_bridge.cpp (parseComponentId/parseHealth) — pas de
// réutilisation directe, ce binaire ne dépend pas de UDP_bridge.cpp.
[[nodiscard]] std::optional<ComponentID> parseComponentId(std::string_view s) {
  if (s == "MavlinkInterface") return ComponentID::MavlinkInterface;
  if (s == "SensorFusion") return ComponentID::SensorFusion;
  if (s == "Navigation") return ComponentID::Navigation;
  if (s == "MissionControl") return ComponentID::MissionControl;
  if (s == "SysMonitoring") return ComponentID::SysMonitoring;
  return std::nullopt;
}

// Liste volontairement fermée, même esprit que UDP_bridge.cpp
// parseInjectableCode() — codes majeurs représentatifs pour tester le
// relais CODE (cf. scanCodeChanges, SystemMonitoring.cpp).
[[nodiscard]] std::optional<CODES::Code> parseInjectableCode(std::string_view s) {
  if (s == "SFS002") return CODES::SFS002; // GPS perdu (Emergency)
  if (s == "SFS004") return CODES::SFS004; // Lidar perdu (Critical)
  if (s == "SFS005") return CODES::SFS005; // EKF mort (Emergency)
  if (s == "FCP003") return CODES::FCP003; // Batterie critique (Emergency)
  if (s == "SMO001") return CODES::SMO001; // E-stop GCS (Emergency)
  if (s == "MVP001") return CODES::MVP001; // RPM bas (Critical)
  if (s == "NVA004") return CODES::NVA004; // Obstacle (Critical)
  if (s == "MCA005") return CODES::MCA005; // Nav périmé (Critical)
  return std::nullopt;
}

[[nodiscard]] std::optional<ComponentHealth> parseHealth(std::string_view s) {
  if (s == "IDLE") return ComponentHealth::IDLE;
  if (s == "NOMINAL") return ComponentHealth::NOMINAL;
  if (s == "DEAD") return ComponentHealth::DEAD;
  if (s == "SICK") return ComponentHealth::SICK;
  if (s == "DEGRADED") return ComponentHealth::DEGRADED;
  return std::nullopt;
}

// Toutes les handlers d'injection, sous l'identité du composant qui
// écrirait normalement ce champ en prod — SharedCompMem est single-writer
// par segment, mais rien n'empêche un outil de dev de se faire passer pour
// le bon propriétaire (même principe que UDP_bridge.cpp CMD=CODE/HEALTH).
struct Injectors {
  SharedSFMemHandler &sf;   // identité SensorFusion : gpsMag, lidar, state
  SharedFCStatusHandler &fc; // identité MavlinkInterface : battery
  SharedCmdMemHandler &cmd; // identité MissionControl : missionState
  SharedNavMemHandler &nav; // identité Navigation : lateral/vertical engaged+armed
  SharedCmdMemHandler &cmdNav; // identité Navigation : engagedSegment (target alt/speed)
  SharedSysStateMem &sysMem;   // brut : CMD=HEALTH construit un handler ad-hoc par composant visé
};

void handleCommand(std::string_view text, Injectors &inj) {
  auto fields = parseLines(text);
  auto cmdIt = fields.find("CMD");
  if (cmdIt == fields.end())
    return;

  if (cmdIt->second == "STATE") {
    DynamicState state{};
    state.ts = Clock::now();
    state.posNed = Position{Vec3{parseFloatOr(fields, "north", 0.0f),
                                 parseFloatOr(fields, "east", 0.0f),
                                 parseFloatOr(fields, "down", 0.0f)}};
    state.velNed = Velocity{Vec3{parseFloatOr(fields, "vn", 0.0f),
                                 parseFloatOr(fields, "ve", 0.0f),
                                 parseFloatOr(fields, "vd", 0.0f)}};
    state.attitude.q = quatFromRpyDeg(parseFloatOr(fields, "roll", 0.0f),
                                      parseFloatOr(fields, "pitch", 0.0f),
                                      parseFloatOr(fields, "yaw", 0.0f));
    inj.sf.setDynamicState(state);
  } else if (cmdIt->second == "GPS") {
    auto sample = inj.sf.getGpsMag().value_or(GpsMagSample{});
    sample.ts = Clock::now();
    sample.latitude = static_cast<double>(parseFloatOr(fields, "lat", 0.0f));
    sample.longitude = static_cast<double>(parseFloatOr(fields, "lon", 0.0f));
    sample.altMsl = parseFloatOr(fields, "alt", 0.0f);
    sample.hdop = parseFloatOr(fields, "hdop", 0.0f);
    sample.vdop = parseFloatOr(fields, "vdop", 0.0f);
    sample.fixType = static_cast<uint8_t>(parseU32Or(fields, "fixtype", 0));
    sample.satellitesVisible = static_cast<uint8_t>(parseU32Or(fields, "numsats", 0));
    inj.sf.setGpsMag(sample);
  } else if (cmdIt->second == "SAT") {
    auto sample = inj.sf.getGpsMag().value_or(GpsMagSample{});
    uint32_t index = parseU32Or(fields, "index", 0);
    if (index < GpsMagSample::kMaxSatellites) {
      sample.satId[index] = static_cast<uint8_t>(parseU32Or(fields, "id", 0));
      sample.satStrength[index] = static_cast<uint8_t>(parseU32Or(fields, "strength", 0));
      sample.satQuality[index] = static_cast<uint8_t>(parseU32Or(fields, "quality", 0));
      sample.ts = Clock::now();
      inj.sf.setGpsMag(sample);
    }
  } else if (cmdIt->second == "LIDARPOINT") {
    auto map = inj.sf.getLidarMap().value_or(LidarMap{});
    uint32_t bearing = parseU32Or(fields, "bearing", 0) % 360;
    map.rangeMm[bearing] = static_cast<uint16_t>(parseU32Or(fields, "dist_mm", 0));
    map.intensity[bearing] = static_cast<uint8_t>(parseU32Or(fields, "intensity", 0));
    map.ts = Clock::now();
    inj.sf.setLidarMap(map);
  } else if (cmdIt->second == "LIDARCLEAR") {
    LidarMap map{};
    map.ts = Clock::now();
    inj.sf.setLidarMap(map);
  } else if (cmdIt->second == "BATTERY") {
    FCBatteryStatus batt{};
    batt.voltage = Volts{parseFloatOr(fields, "voltage", 0.0f)};
    batt.current = Amps{parseFloatOr(fields, "current", 0.0f)};
    batt.remainingPct = static_cast<uint8_t>(parseU32Or(fields, "pct", 0));
    inj.fc.setBattery(batt);
  } else if (cmdIt->second == "MISSIONSTATE") {
    auto it = fields.find("state");
    if (it != fields.end())
      inj.cmd.setMissionState(parseMissionState(it->second));
  } else if (cmdIt->second == "FMA") {
    inj.nav.setLateralEngaged(CODES::Code::unpack(parseU32Or(fields, "lateral_engaged", 0)));
    inj.nav.setLateralArmed(CODES::Code::unpack(parseU32Or(fields, "lateral_armed", 0)));
    inj.nav.setVerticalEngaged(CODES::Code::unpack(parseU32Or(fields, "vertical_engaged", 0)));
    inj.nav.setVerticalArmed(CODES::Code::unpack(parseU32Or(fields, "vertical_armed", 0)));
    NavSegment seg{};
    seg.speed = MetersPerSec{parseFloatOr(fields, "target_speed", 0.0f)};
    seg.targetAltitude = Meters{parseFloatOr(fields, "target_altitude", 0.0f)};
    inj.cmdNav.setEngagedSegment(seg);
  } else if (cmdIt->second == "HEALTH") {
    auto compIt = fields.find("component");
    auto healthIt = fields.find("health");
    if (compIt == fields.end() || healthIt == fields.end())
      return;
    auto compId = parseComponentId(compIt->second);
    auto health = parseHealth(healthIt->second);
    if (!compId || !health)
      return;
    // Handler ad-hoc sous l'identité du composant visé — même principe que
    // UDP_bridge.cpp CMD=HEALTH (setHealth() est self-report, ownId_-bound).
    SharedSysStateMemHandler fake(inj.sysMem, *compId, Us(2000));
    fake.setHealth(*health);
  } else if (cmdIt->second == "CODE") {
    auto compIt = fields.find("component");
    auto codeIt = fields.find("code");
    auto actionIt = fields.find("action");
    if (compIt == fields.end() || codeIt == fields.end() || actionIt == fields.end())
      return;
    auto compId = parseComponentId(compIt->second);
    auto code = parseInjectableCode(codeIt->second);
    if (!compId || !code)
      return;
    SharedSysStateMemHandler fake(inj.sysMem, *compId, Us(2000));
    if (actionIt->second == "raise")
      fake.raiseCode(*code);
    else if (actionIt->second == "clear")
      fake.clearCode(*code);
  }
}

} // namespace

int main() {
  auto nav = publishSharedMemory<SharedNavMem>(SYSMON_SITL::kNavShmPath);
  auto sf = publishSharedMemory<SharedSFMem>(SYSMON_SITL::kSFShmPath);
  auto sys = publishSharedMemory<SharedSysStateMem>(SYSMON_SITL::kSysShmPath);
  auto com = publishSharedMemory<SharedComMem>(SYSMON_SITL::kComShmPath);
  auto fc = publishSharedMemory<SharedFCStatus>(SYSMON_SITL::kFCShmPath);
  auto cmd = publishSharedMemory<SharedCmdMem>(SYSMON_SITL::kCmdShmPath);

  if (!nav || !sf || !sys || !com || !fc || !cmd) {
    std::cerr << "SYSMON_SITL: echec publication shm\n";
    return 1;
  }

  installSignalHandler();

  SharedNavMemHandler navHandler(ComponentID::SysMonitoring, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::SysMonitoring, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::SysMonitoring, Us(2000));
  SharedComMemHandler comHandler(ComponentID::SysMonitoring, Us(2000), *com->ptr);
  SharedFCStatusHandler fcHandler(ComponentID::SysMonitoring, Us(2000), *fc->ptr);
  SharedCmdMemHandler cmdHandler(ComponentID::SysMonitoring, Us(2000), *cmd->ptr);

  // Handlers d'injection, sous l'identité "propriétaire" de chaque champ en
  // prod (cf. commentaire sur Injectors) — distincts des handlers ci-dessus
  // (identité SysMonitoring, lecture seule côté paquets TELEM::*).
  SharedSFMemHandler sfInject(ComponentID::SensorFusion, Us(2000), *sf->ptr);
  SharedFCStatusHandler fcInject(ComponentID::MavlinkInterface, Us(2000), *fc->ptr);
  SharedCmdMemHandler cmdInjectMc(ComponentID::MissionControl, Us(2000), *cmd->ptr);
  SharedNavMemHandler navInject(ComponentID::Navigation, Us(2000), *nav->ptr);
  SharedCmdMemHandler cmdInjectNav(ComponentID::Navigation, Us(2000), *cmd->ptr);
  Injectors inj{sfInject, fcInject, cmdInjectMc, navInject, cmdInjectNav, *sys->ptr};

  UdpTelemetryDriver mainLink(kUdpTelMainLocalPort, kUdpTelMainPeerPort);
  UdpTelemetryDriver secLink(kUdpTelSecLocalPort, kUdpTelSecPeerPort);
  UdpVideoDriver videoSource(kUdpVideoLocalPort);
  LinuxThermalZoneDriver tempSource; // lit le vrai capteur thermique de la machine de dev

  int injectFd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (injectFd < 0) {
    std::cerr << "SYSMON_SITL: echec creation socket injection\n";
    return 1;
  }
  sockaddr_in injectAddr{};
  injectAddr.sin_family = AF_INET;
  injectAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  injectAddr.sin_port = htons(kInjectPort);
  if (::bind(injectFd, reinterpret_cast<sockaddr *>(&injectAddr), sizeof(injectAddr)) < 0) {
    std::cerr << "SYSMON_SITL: echec bind port injection " << kInjectPort << "\n";
    ::close(injectFd);
    return 1;
  }
  timeval rxTimeout{.tv_sec = 0, .tv_usec = 500000};
  ::setsockopt(injectFd, SOL_SOCKET, SO_RCVTIMEO, &rxTimeout, sizeof(rxTimeout));

  {
    // Même piège use-after-unmap que les autres SITL : composant détruit
    // avant le nettoyage shm (bloc dédié).
    ComponenConfig config{.id = ComponentID::SysMonitoring, .CompCore = 0};
    SysMonitoring composant(config, sysHandler, comHandler, navHandler, sfHandler,
                            fcHandler, cmdHandler, mainLink, secLink, videoSource,
                            tempSource);

    std::cout << "SYSMON_SITL: pret (TELEM sur " << kUdpTelMainLocalPort << "/"
             << kUdpTelSecLocalPort << ", injection sur " << kInjectPort << ")\n";

    char buf[2048];
    while (keepRunning.load(std::memory_order_acquire)) {
      ssize_t n = ::recv(injectFd, buf, sizeof(buf), 0);
      if (n > 0)
        handleCommand(std::string_view(buf, static_cast<size_t>(n)), inj);
    }
  }

  ::close(injectFd);

  std::cout << "SYSMON_SITL: arret, nettoyage shm\n";
  destroySharedMemory(SYSMON_SITL::kNavShmPath, nav.value());
  destroySharedMemory(SYSMON_SITL::kSFShmPath, sf.value());
  destroySharedMemory(SYSMON_SITL::kSysShmPath, sys.value());
  destroySharedMemory(SYSMON_SITL::kComShmPath, com.value());
  destroySharedMemory(SYSMON_SITL::kFCShmPath, fc.value());
  destroySharedMemory(SYSMON_SITL::kCmdShmPath, cmd.value());

  return 0;
}
