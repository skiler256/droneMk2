#include "drone/Components/SystemMonitoring/Driver/WFB_NG_Driver.hpp"
#include "drone/utilities.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <charconv>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

// Les stats wfb-ng ne bougent qu'à l'échelle de la seconde : les relire à la
// cadence de la boucle RT ne gagnerait rien et coûterait un syscall par tour.
constexpr TYPES::Ms kStatsPeriod{1000};

// Au-delà, les dernières valeurs lues sont déclarées périmées plutôt que
// laissées à l'affichage : une qualité de lien figée est pire qu'absente.
constexpr TYPES::Ms kStatsStale{5000};

// Cherche "<key>": <nombre> dans une ligne JSON. Pas de parseur complet :
// une seule forme à extraire, aucune dépendance, et -fno-exceptions interdit
// la plupart des bibliothèques.
std::optional<float> jsonNumber(std::string_view line, std::string_view key) {
  auto pos = line.find(key);
  if (pos == std::string_view::npos)
    return std::nullopt;

  pos = line.find(':', pos + key.size());
  if (pos == std::string_view::npos)
    return std::nullopt;
  ++pos;

  while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t'))
    ++pos;

  auto end = pos;
  while (end < line.size() &&
         (line[end] == '-' || line[end] == '.' || (line[end] >= '0' && line[end] <= '9')))
    ++end;
  if (end == pos)
    return std::nullopt;

  float value{};
  auto [ptr, ec] = std::from_chars(line.data() + pos, line.data() + end, value);
  if (ec != std::errc{})
    return std::nullopt;
  return value;
}

} // namespace

WFB_NG_Driver::WFB_NG_Driver(uint16_t localPort, uint16_t peerPort, uint16_t statsPort)
    : UdpTelemetryDriver(localPort, peerPort), statsPort_(statsPort) {}

WFB_NG_Driver::~WFB_NG_Driver() { dropStats(); }

std::expected<std::span<const std::byte>, TYPES::DriverError>
WFB_NG_Driver::poll() {
  auto now = TYPES::Clock::now();
  refreshStats(now);

  if (lastStatsOk_ != TYPES::TimePoint{} &&
      UTILITIES::msBetween(lastStatsOk_, now) > kStatsStale) {
    rssiDbm_.reset();
    lossRatio_.reset();
  }

  return UdpTelemetryDriver::poll();
}

bool WFB_NG_Driver::connectStats() {
  statsFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (statsFd_ < 0)
    return false;

  // Non bloquant avant le connect : appelé depuis une boucle RT, aucun
  // appel ne doit attendre un service qui n'est peut-être pas démarré.
  int flags = ::fcntl(statsFd_, F_GETFL, 0);
  ::fcntl(statsFd_, F_SETFL, flags | O_NONBLOCK);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(statsPort_);

  int rc = ::connect(statsFd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  if (rc < 0 && errno != EINPROGRESS) {
    dropStats();
    return false;
  }
  return true;
}

void WFB_NG_Driver::dropStats() {
  if (statsFd_ >= 0)
    ::close(statsFd_);
  statsFd_ = -1;
  statsLen_ = 0;
}

void WFB_NG_Driver::refreshStats(TYPES::TimePoint now) {
  if (statsPort_ == 0)
    return;
  if (UTILITIES::msBetween(lastStatsAttempt_, now) < kStatsPeriod)
    return;
  lastStatsAttempt_ = now;

  if (statsFd_ < 0 && !connectStats())
    return;

  for (;;) {
    if (statsLen_ >= statsBuf_.size()) {
      // Ligne plus longue que le tampon : on repart propre plutôt que de
      // parser un fragment arbitraire.
      statsLen_ = 0;
      return;
    }

    ssize_t n = ::recv(statsFd_, statsBuf_.data() + statsLen_,
                       statsBuf_.size() - statsLen_, 0);
    if (n > 0) {
      statsLen_ += static_cast<size_t>(n);
      continue;
    }
    if (n == 0) { // pair fermé
      dropStats();
      return;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      break; // plus rien à lire pour l'instant
    dropStats();
    return;
  }

  std::string_view buf(statsBuf_.data(), statsLen_);
  size_t consumed = 0;
  for (;;) {
    auto nl = buf.find('\n', consumed);
    if (nl == std::string_view::npos)
      break;
    parseStatsLine(buf.substr(consumed, nl - consumed));
    consumed = nl + 1;
  }

  // Conserve le reliquat partiel pour la prochaine lecture.
  if (consumed > 0) {
    std::memmove(statsBuf_.data(), statsBuf_.data() + consumed, statsLen_ - consumed);
    statsLen_ -= consumed;
  }
}

void WFB_NG_Driver::parseStatsLine(std::string_view line) {
  // ATTENTION : noms de champs à confirmer contre la sortie réelle de
  // wfb-ng sur le Pi (cf. SYSMONITORING.md §15). Tant qu'ils ne
  // correspondent pas, rssiDbm()/lossRatio() restent vides et le driver se
  // comporte comme un simple transport UDP — c'est voulu.
  auto rssi = jsonNumber(line, "\"rssi\"");
  auto lost = jsonNumber(line, "\"lost\"");
  auto recv = jsonNumber(line, "\"recv\"");

  if (!rssi && !lost)
    return; // ligne d'un autre type, pas une erreur

  lastStatsOk_ = TYPES::Clock::now();

  if (rssi)
    rssiDbm_ = static_cast<int8_t>(*rssi);

  if (lost && recv) {
    float total = *lost + *recv;
    lossRatio_ = total > 0.0f ? *lost / total : 0.0f;
  }
}
