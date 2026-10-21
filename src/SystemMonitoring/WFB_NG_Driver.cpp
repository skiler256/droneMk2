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
// Disposition publiee par tools/wfb_stats_relay.py. Offsets fixes et memcpy,
// meme convention que les paquets TELEM generes — pas d'attribut packed.
//
//   0 u16 magic | 2 u8 version | 3 i8 rssi | 4 i8 snr
//   5 u8 antennes | 6 u32 recus | 10 u32 perdus
//
// Le RSSI est le PIRE des antennes : avec une diversite 2T2R le recepteur
// utilise la meilleure chaine, donc le pire ne mesure pas la marge de liaison
// mais la SANTE du systeme d'antennes. Une antenne debranchee se voit, la ou
// une moyenne la masquerait.
struct StatsRelais {
  static constexpr uint16_t kMagic = 0x5751;
  static constexpr uint8_t kVersion = 1;
  static constexpr size_t kSize = 14;

  int8_t rssi{};
  int8_t snr{};
  uint8_t antennes{};
  uint32_t recus{};
  uint32_t perdus{};

  [[nodiscard]] static bool unpack(const std::byte *buf, size_t len,
                                   StatsRelais &out) noexcept {
    if (len < kSize)
      return false;
    uint16_t magic = 0;
    uint8_t version = 0;
    std::memcpy(&magic, buf + 0, sizeof(magic));
    std::memcpy(&version, buf + 2, sizeof(version));
    if (magic != kMagic || version != kVersion)
      return false;
    std::memcpy(&out.rssi, buf + 3, sizeof(out.rssi));
    std::memcpy(&out.snr, buf + 4, sizeof(out.snr));
    std::memcpy(&out.antennes, buf + 5, sizeof(out.antennes));
    std::memcpy(&out.recus, buf + 6, sizeof(out.recus));
    std::memcpy(&out.perdus, buf + 10, sizeof(out.perdus));
    return true;
  }
};

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
  // Socket UDP en ECOUTE, pas une connexion TCP : c'est le relais qui publie,
  // et un datagramme se perd sans consequence si personne n'ecoute. L'inverse
  // — un tube ou un socket que le producteur doit joindre — ferait dependre la
  // liaison radio de la presence du logiciel de vol.
  statsFd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (statsFd_ < 0)
    return false;

  // Non bloquant : appele depuis une boucle RT, aucun appel ne doit attendre
  // un relais qui n'est peut-etre pas demarre.
  int flags = ::fcntl(statsFd_, F_GETFL, 0);
  ::fcntl(statsFd_, F_SETFL, flags | O_NONBLOCK);

  int reuse = 1;
  ::setsockopt(statsFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(statsPort_);

  if (::bind(statsFd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    dropStats();
    return false;
  }
  return true;
}

void WFB_NG_Driver::dropStats() {
  if (statsFd_ >= 0)
    ::close(statsFd_);
  statsFd_ = -1;
}

void WFB_NG_Driver::refreshStats(TYPES::TimePoint now) {
  if (statsPort_ == 0)
    return;
  if (UTILITIES::msBetween(lastStatsAttempt_, now) < kStatsPeriod)
    return;
  lastStatsAttempt_ = now;

  if (statsFd_ < 0 && !connectStats())
    return;

  // Vider la file et ne garder que le dernier datagramme : une mesure de
  // qualite perimee n'a aucun interet, et accumuler un retard sur le chemin
  // RT serait pire que de perdre des echantillons.
  std::byte buf[64];
  StatsRelais dernier{};
  bool recu = false;
  for (;;) {
    ssize_t n = ::recv(statsFd_, buf, sizeof(buf), 0);
    if (n > 0) {
      StatsRelais st{};
      if (StatsRelais::unpack(buf, static_cast<size_t>(n), st)) {
        dernier = st;
        recu = true;
      }
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      break;
    // Toute autre erreur : on referme et on retentera au prochain tour.
    dropStats();
    return;
  }

  if (!recu)
    return;

  lastStatsOk_ = now;
  rssiDbm_ = dernier.rssi;
  const float total =
      static_cast<float>(dernier.recus) + static_cast<float>(dernier.perdus);
  lossRatio_ = total > 0.0f ? static_cast<float>(dernier.perdus) / total : 0.0f;
}

