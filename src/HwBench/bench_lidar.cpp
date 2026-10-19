// Banc LD06 : vue console du nuage de points réel, par secteurs de 30°.
// 360 valeurs étant illisibles en console, on affiche par secteur la distance
// minimale et le nombre de points valides — suffisant pour vérifier que le
// capteur voit un obstacle dans la bonne direction.
//
// Usage : ./bench_lidar [port]      (défaut /dev/ttyAMA2)

#include "drone/Components/Drivers/LD06.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

constexpr int kSectors = 12; // 30° chacun
constexpr int kSectorWidth = 360 / kSectors;

std::atomic<bool> running{true};
void onSignal(int) { running.store(false); }

void render(const LidarData &scan, std::uint64_t rotations,
            std::uint64_t crcErrors, std::string_view lastError) {
  std::cout << "\033[2J\033[H";
  std::cout << "═══ LD06 ═══ tours:" << rotations
            << "  trames CRC invalides:" << crcErrors << "\n\n";

  int totalValid = 0;
  for (auto d : scan.distance_mm)
    if (d > 0)
      ++totalValid;
  std::cout << "points valides : " << totalValid << " / 360\n\n";
  std::cout << " secteur      min      points   (nez = 0 deg)\n";

  for (int s = 0; s < kSectors; ++s) {
    const int from = s * kSectorWidth;
    std::uint16_t minDist = 0;
    int valid = 0;
    for (int a = from; a < from + kSectorWidth; ++a) {
      const auto d = scan.distance_mm[static_cast<std::size_t>(a)];
      if (d == 0)
        continue;
      ++valid;
      if (minDist == 0 || d < minDist)
        minDist = d;
    }

    std::cout << std::right << std::setw(4) << from << "-" << std::setw(3)
              << from + kSectorWidth - 1 << "  ";
    if (valid == 0) {
      std::cout << std::setw(8) << "-" << std::setw(10) << 0;
    } else {
      std::cout << std::setw(6) << minDist << "mm" << std::setw(10) << valid;
      // Barre : d'autant plus longue que l'obstacle est proche (< 2m).
      const int bar = minDist < 2000 ? (2000 - minDist) / 100 : 0;
      std::cout << "  " << std::string(static_cast<std::size_t>(bar), '#');
    }
    std::cout << "\n";
  }

  if (!lastError.empty())
    std::cout << "\nderniere erreur : " << lastError << "\n";
  std::cout << "\n(Ctrl+C pour quitter)\n" << std::flush;
}

} // namespace

int main(int argc, char **argv) {
  const char *port = (argc > 1) ? argv[1] : "/dev/ttyAMA2";
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  LD06 lidar(port);
  std::cout << "Initialisation sur " << port << " ...\n";
  if (auto err = lidar.init()) {
    std::cerr << "echec init() : " << TYPES::toString(*err) << "\n";
    return 1;
  }
  std::cout << "OK, attente d'un tour complet...\n";

  LidarData last{};
  std::uint64_t rotations = 0, crcErrors = 0;
  std::string lastError;

  while (running.load()) {
    if (auto r = lidar.update()) {
      last = *r;
      ++rotations;
    } else if (r.error() == TYPES::DriverError::InvalidData) {
      ++crcErrors;
    } else if (r.error() != TYPES::DriverError::NoNewData) {
      lastError = TYPES::toString(r.error());
    }
    render(last, rotations, crcErrors, lastError);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "\narret.\n";
  return 0;
}
