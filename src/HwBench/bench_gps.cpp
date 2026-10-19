// Banc GPS : affiche en direct ce que le NEO-M8N remonte réellement.
// Aucun privilège RT requis, aucune shm, aucun dashboard — juste le driver.
//
// Usage : ./bench_gps [port]        (défaut /dev/ttyAMA1)

#include "drone/Components/Drivers/NEO-M8N.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

std::atomic<bool> running{true};
void onSignal(int) { running.store(false); }

void render(const GPSData &d, std::uint64_t frames, std::uint64_t errors,
            std::uint32_t crcErrors, std::string_view lastError) {
  std::cout << "\033[2J\033[H"; // effacement + retour en haut
  std::cout << "═══ NEO-M8N ═══ frames:" << frames << "  erreurs:" << errors
            << "  checksum rejetes:" << crcErrors << "\n\n";

  std::cout << std::left << std::setw(22) << "Fix"
            << (d.gpsFixOk ? "\033[32mOK\033[0m" : "\033[31mNO FIX\033[0m")
            << "  (gpsFix brut UBX = " << static_cast<int>(d.fixType) << ")\n";
  std::cout << std::setw(22) << "Satellites"
            << static_cast<int>(d.satellitesTracked) << " suivis, "
            << static_cast<int>(d.satDetailCount) << " detailles\n\n";

  std::cout << std::fixed << std::setprecision(7);
  std::cout << std::setw(22) << "Latitude" << d.coord.latitude << "\n";
  std::cout << std::setw(22) << "Longitude" << d.coord.longitude << "\n";
  std::cout << std::setprecision(2);
  std::cout << std::setw(22) << "Altitude MSL"
            << static_cast<double>(d.altMslMm) / 1000.0 << " m\n\n";

  std::cout << std::setw(22) << "velNED (m/s)"
            << static_cast<double>(d.velNED[0]) / 100.0 << " / "
            << static_cast<double>(d.velNED[1]) / 100.0 << " / "
            << static_cast<double>(d.velNED[2]) / 100.0 << "\n";
  std::cout << std::setw(22) << "Cap" << d.heading << " deg\n\n";

  std::cout << std::setw(22) << "hAcc / vAcc"
            << static_cast<double>(d.pAcc) / 1000.0 << " / "
            << static_cast<double>(d.vAcc) / 1000.0 << " m\n";
  std::cout << std::setw(22) << "sAcc"
            << static_cast<double>(d.sAcc) / 1000.0 << " m/s\n";
  std::cout << std::setw(22) << "hDOP / vDOP" << d.hdop << " / " << d.vdop
            << "\n\n";

  std::cout << std::setw(22) << "Date UTC" << d.timeArray[0] << "-"
            << std::setfill('0') << std::setw(2) << d.timeArray[1] << "-"
            << std::setw(2) << d.timeArray[2] << " " << std::setw(2)
            << d.timeArray[3] << ":" << std::setw(2) << d.timeArray[4] << ":"
            << std::setw(2) << d.timeArray[5] << std::setfill(' ') << "\n";
  std::cout << std::left << std::setw(22) << "iTOW" << d.iTowMs << " ms\n\n";

  if (d.satDetailCount > 0) {
    std::cout << "  SatID  Signal  Qualite\n";
    for (std::size_t i = 0; i < d.satDetailCount; ++i)
      std::cout << "  " << std::setw(7) << static_cast<int>(d.sats[i].id)
                << std::setw(8) << static_cast<int>(d.sats[i].strength)
                << static_cast<int>(d.sats[i].quality) << "\n";
  }

  if (!lastError.empty())
    std::cout << "\nderniere erreur : " << lastError << "\n";
  std::cout << "\n(Ctrl+C pour quitter)\n" << std::flush;
}

} // namespace

int main(int argc, char **argv) {
  const char *port = (argc > 1) ? argv[1] : "/dev/ttyAMA1";
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  NEO_M8N gps(port);
  std::cout << "Initialisation sur " << port << " ...\n";
  if (auto err = gps.init()) {
    std::cerr << "echec init() : " << TYPES::toString(*err) << "\n";
    return 1;
  }
  std::cout << "OK, attente de donnees...\n";

  GPSData last{};
  std::uint64_t frames = 0, errors = 0;
  std::string lastError;

  while (running.load()) {
    if (auto r = gps.update()) {
      last = *r;
      ++frames;
    } else if (r.error() != TYPES::DriverError::NoNewData) {
      ++errors;
      lastError = TYPES::toString(r.error());
    }
    render(last, frames, errors, gps.checksumErrors(), lastError);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  std::cout << "\narret.\n";
  return 0;
}
