// Banc LIS3MDL : lecture directe du magnétomètre.
// Le capteur n'est plus câblé sur le Pi (cf. SENSORFUSION.md §3.5) — cet outil
// sert à le vérifier sur banc s'il y est rebranché.
//
// Usage : ./bench_mag [adresse I2C en hex]   (défaut 0x1C)

#include "drone/Components/Drivers/LIS3MDL.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> running{true};
void onSignal(int) { running.store(false); }
} // namespace

int main(int argc, char **argv) {
  const auto addr = static_cast<std::uint8_t>(
      argc > 1 ? std::strtol(argv[1], nullptr, 16) : 0x1C);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  LIS3MDL mag(addr);
  std::cout << "Initialisation I2C 0x" << std::hex << static_cast<int>(addr)
            << std::dec << " ...\n";
  if (auto err = mag.init()) {
    std::cerr << "echec init() : " << TYPES::toString(*err) << "\n";
    return 1;
  }

  std::uint64_t samples = 0;
  while (running.load()) {
    if (auto r = mag.update()) {
      ++samples;
      const auto &b = r->field.B;
      const float norm = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
      std::cout << "\033[2J\033[H" << std::fixed << std::setprecision(4)
                << "═══ LIS3MDL ═══ echantillons:" << samples << "\n\n"
                << "  Bx " << std::setw(10) << b.x << " G\n"
                << "  By " << std::setw(10) << b.y << " G\n"
                << "  Bz " << std::setw(10) << b.z << " G\n\n"
                << "  norme " << std::setw(10) << norm
                << " G  (attendu ~0.25-0.65 G sur Terre)\n"
                << "\n(Ctrl+C pour quitter)\n"
                << std::flush;
    } else if (r.error() != TYPES::DriverError::NoNewData) {
      std::cerr << "erreur : " << TYPES::toString(r.error()) << "\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "\narret.\n";
  return 0;
}
