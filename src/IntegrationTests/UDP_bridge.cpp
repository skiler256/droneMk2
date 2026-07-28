// src/SITL/UDP_bridge.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Pont shm <-> UDP pour les tests SITL : s'attache aux MÊMES segments shm que
// MV_SITL.cpp (déjà publiés par lui), réutilise les VRAIS handlers (aucune
// réimplémentation du verrou CAS).
//
// Protocole texte, un datagramme = un segment, ligne 1 "SEGMENT=<NOM>" puis
// une ligne "cle=valeur" par champ — pas de lib JSON, débogable à la main
// avec `nc -ul <port>`. Deux ports en localhost uniquement (le pont et le
// serveur web tournent tous les deux sur le Pi, seul le navigateur est
// distant — cf. discussion de session) :
//   - kStatusPort  : le pont ENVOIE ici, ~10Hz, un datagramme par segment
//                    (FC/NAV/SF) — tools/dashboard/dashboard_server.py écoute.
//   - kCommandPort : le pont ÉCOUTE ici les injections de test (CMD=NAV,
//                    CMD=GPSMAG) — pas encore relié à une page web, mais
//                    déjà fonctionnel (testable avec `nc -u`).
//
// Usage : ./udp_bridge (après avoir lancé mv_sitl, qui publie les segments)

#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/SITL/ShmPaths.hpp"
#include "drone/core/SharedSysStateMem.hpp"
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
constexpr auto kPushPeriod = std::chrono::milliseconds(100); // 10Hz

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
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(kStatusPort);
  ::sendto(fd, text.data(), text.size(), 0, reinterpret_cast<sockaddr *>(&addr),
          sizeof(addr));
}

std::string serializeFc(SharedFCStatusHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=FC\n";

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

std::string serializeNav(SharedNavMemHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=NAV\n";

  if (auto cmd = h.getCommand()) {
    auto ageMs = UTILITIES::msBetween(cmd->ts, Clock::now());
    os << "vel.x=" << cmd->velNed.ned.x << "\nvel.y=" << cmd->velNed.ned.y
       << "\nvel.z=" << cmd->velNed.ned.z << "\nheading=" << cmd->headingCmd.v
       << "\naltitude=" << cmd->altitudeCmd.v << "\nage_ms=" << ageMs.count() << "\n";
  }

  return os.str();
}

std::string serializeSf(SharedSFMemHandler &h) {
  std::ostringstream os;
  os << "SEGMENT=SF\n";

  if (auto gps = h.getGpsMag()) {
    auto ageMs = UTILITIES::msBetween(gps->ts, Clock::now());
    os << "gps.lat=" << gps->latitude << "\ngps.lon=" << gps->longitude
       << "\ngps.alt=" << gps->altMsl << "\ngps.vel_n=" << gps->velNed.ned.x
       << "\ngps.vel_e=" << gps->velNed.ned.y << "\ngps.vel_d=" << gps->velNed.ned.z
       << "\ngps.heading=" << gps->heading.v << "\ngps.hdop=" << gps->hdop
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

// Parse un datagramme "cle=valeur\n..." en map — utilisé pour les commandes
// entrantes (CMD=NAV/CMD=GPSMAG).
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

// Applique une commande reçue en UDP (CMD=NAV ou CMD=GPSMAG) sur les vrais
// handlers — pas encore relié à une page web, testable avec :
// echo -e "CMD=NAV\nvel.x=1.0" | nc -u -w0 127.0.0.1 15000
void handleCommand(std::string_view text, SharedNavMemHandler &navHandler,
                   SharedSFMemHandler &sfHandler) {
  auto fields = parseLines(text);
  auto it = fields.find("CMD");
  if (it == fields.end())
    return;

  if (it->second == "NAV") {
    NavCommand cmd{.ts = Clock::now(),
                  .velNed = Velocity{Vec3{parseFloatOr(fields, "vel.x", 0.0f),
                                          parseFloatOr(fields, "vel.y", 0.0f),
                                          parseFloatOr(fields, "vel.z", 0.0f)}},
                  .headingCmd = Radians{parseFloatOr(fields, "heading", 0.0f)},
                  .altitudeCmd = Meters{parseFloatOr(fields, "altitude", 0.0f)}};
    navHandler.setCommand(cmd);
  } else if (it->second == "GPSMAG") {
    GpsMagSample sample{
        .ts = Clock::now(),
        .latitude = parseFloatOr(fields, "lat", 0.0f),
        .longitude = parseFloatOr(fields, "lon", 0.0f),
        .altMsl = parseFloatOr(fields, "alt", 0.0f),
        .velNed = Velocity{Vec3{parseFloatOr(fields, "vel_n", 0.0f),
                                parseFloatOr(fields, "vel_e", 0.0f),
                                parseFloatOr(fields, "vel_d", 0.0f)}},
        .heading = Radians{parseFloatOr(fields, "heading", 0.0f)},
        .fixType = static_cast<uint8_t>(parseFloatOr(fields, "fix_type", 3.0f)),
        .satellitesVisible = static_cast<uint8_t>(parseFloatOr(fields, "satellites", 10.0f))};
    sfHandler.setGpsMag(sample);
  }
}

} // namespace

int main() {
  // Attache (pas publish) : MV_SITL.cpp doit déjà tourner et avoir publié
  // ces segments.
  auto nav = attachSharedMemory<SharedNavMem>(SITL::kNavShmPath);
  auto sf = attachSharedMemory<SharedSFMem>(SITL::kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(SITL::kSysShmPath);
  auto fc = attachSharedMemory<SharedFCStatus>(SITL::kFCShmPath);

  if (!nav || !sf || !sys || !fc) {
    std::cerr << "UDP_bridge: echec attach shm (mv_sitl tourne-t-il ?)\n";
    return 1;
  }

  installSignalHandler();

  // Identité utilisée comme jeton de verrou CAS — ce pont n'est pas un
  // composant réel, GlobalWatchdog n'existe même pas ici : ComponentID
  // choisi arbitrairement (aucun enjeu de correction, cf. discussion).
  SharedNavMemHandler navHandler(ComponentID::GlobalWatchdog, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::GlobalWatchdog, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::GlobalWatchdog,
                                      Us(2000));
  SharedFCStatusHandler fcHandler(ComponentID::GlobalWatchdog, Us(2000), *fc->ptr);

  int statusFd = openStatusSocket();
  int commandFd = openCommandSocket();
  if (statusFd < 0 || commandFd < 0) {
    std::cerr << "UDP_bridge: echec ouverture socket\n";
    return 1;
  }

  std::cout << "UDP_bridge: pret — statut vers " << kStatusPort << ", commandes sur "
           << kCommandPort << "\n";

  auto nextPush = Clock::now();
  std::array<std::byte, 512> cmdBuf{};

  while (keepRunning.load(std::memory_order_acquire)) {
    auto now = Clock::now();

    if (now >= nextPush) {
      sendSegment(statusFd, serializeFc(fcHandler));
      sendSegment(statusFd, serializeNav(navHandler));
      sendSegment(statusFd, serializeSf(sfHandler));
      nextPush = now + kPushPeriod;
    }

    ssize_t n = ::recv(commandFd, cmdBuf.data(), cmdBuf.size(), 0);
    if (n > 0) {
      handleCommand(std::string_view(reinterpret_cast<const char *>(cmdBuf.data()),
                                     static_cast<size_t>(n)),
                    navHandler, sfHandler);
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
