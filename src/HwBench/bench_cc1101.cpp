// Banc CC1101 : exerce le pilote du lien secondaire en conditions réelles.
//
// Deux rôles, à lancer un à chaque bout de la liaison. Le pendant côté
// station sol est le firmware ESP32 de GCS/ESP32/cc1101_link.
//
// Usage : ./bench_cc1101 [rx|tx|pingpong] [peripherique]
//   rx        écoute et affiche ce qui arrive      (défaut)
//   tx        émet un compteur une fois par seconde
//   pingpong  émet, puis renvoie tout ce qu'il reçoit
//
// Défaut du périphérique : /dev/spidev0.0

#include "drone/Components/SystemMonitoring/Driver/CC1101_Driver.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> running{true};
void onSignal(int) { running.store(false); }

// Rend lisible une charge utile qui peut contenir n'importe quoi : un
// paquet TELEM::* est binaire, pas du texte.
std::string printable(std::span<const std::byte> data) {
  std::string out;
  for (std::byte b : data) {
    const auto c = static_cast<unsigned char>(b);
    out += (c >= 0x20 && c < 0x7F) ? static_cast<char>(c) : '.';
  }
  return out;
}

} // namespace

int main(int argc, char **argv) {
  const std::string mode = argc > 1 ? argv[1] : "rx";
  const char *device = argc > 2 ? argv[2] : "/dev/spidev0.0";

  if (mode != "rx" && mode != "tx" && mode != "pingpong") {
    std::cerr << "mode inconnu : " << mode << "  (rx | tx | pingpong)\n";
    return 2;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  CC1101_Driver radio(device);
  std::cout << "Initialisation " << device << " ...\n";
  if (auto err = radio.init()) {
    std::cerr << "echec init() : " << TYPES::toString(*err) << "\n";
    if (*err == TYPES::DriverError::InvalidData)
      std::cerr << "  -> PARTNUM/VERSION inattendus : cablage SPI ou puce "
                   "absente.\n";
    return 1;
  }
  std::cout << "puce reconnue, 433.92 MHz, 38.4 kbit/s, mode " << mode
            << "\n\n";

  std::uint64_t sent = 0;
  std::uint64_t received = 0;
  std::uint64_t errors = 0;
  auto lastTx = std::chrono::steady_clock::now();

  while (running.load()) {
    // Réception : interrogation, faute de ligne GDO câblée. La cadence de
    // cette boucle est donc aussi la latence de réception.
    auto frame = radio.poll();
    if (frame) {
      ++received;
      std::cout << "RX  " << frame->size() << " o"
                << "  rssi " << (radio.lastRssiDbm() ? std::to_string(*radio.lastRssiDbm()) : "--")
                << " dBm  lqi "
                << (radio.lastLqi() ? std::to_string(*radio.lastLqi()) : "--")
                << "  | " << printable(*frame) << "\n"
                << std::flush;

      if (mode == "pingpong") {
        std::string echo = "ECHO:" + printable(*frame);
        auto bytes = std::as_bytes(std::span<const char>(echo.data(), echo.size()));
        if (auto err = radio.send(bytes))
          std::cerr << "  echec renvoi : " << TYPES::toString(*err) << "\n";
        else
          ++sent;
      }
    } else if (frame.error() != TYPES::DriverError::NoNewData) {
      ++errors;
      std::cerr << "RX erreur : " << TYPES::toString(frame.error())
                << "   (debordements FIFO : " << radio.overflowCount() << ")\n";
    }

    const auto now = std::chrono::steady_clock::now();
    if ((mode == "tx" || mode == "pingpong") &&
        now - lastTx >= std::chrono::seconds(1)) {
      lastTx = now;
      const std::string msg = "DRONE-" + std::to_string(sent);
      auto bytes = std::as_bytes(std::span<const char>(msg.data(), msg.size()));
      if (auto err = radio.send(bytes)) {
        std::cerr << "TX erreur : " << TYPES::toString(*err) << "\n";
      } else {
        ++sent;
        std::cout << "TX  " << msg << "\n" << std::flush;
      }
    }

    // 20 ms : 50 Hz d'interrogation. Large pour les 8 Hz cumulés du
    // sous-ensemble Links=BOTH, et la FIFO de 64 octets ne risque pas de
    // deborder a cette cadence.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "\n--- bilan ---\n"
            << "  emis      : " << sent << "\n"
            << "  recus     : " << received << "\n"
            << "  erreurs   : " << errors << "\n"
            << "  debordements FIFO : " << radio.overflowCount() << "\n";
  return 0;
}
