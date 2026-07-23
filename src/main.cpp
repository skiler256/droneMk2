// #include "drone/Components/Navigation/Navigation.hpp"
// #include "drone/Components/SensorFusions/SensorsFusion.hpp"
// #include "drone/Components/SensorFusions/SharedSFMem.hpp"
// #include "drone/core/ComponentBase.hpp"
// #include "drone/types.hpp"
// #include <iostream>
// #include <unistd.h>

// using namespace TYPES;
// using namespace std;

// int main() {

//   SharedNavMem Nav{};
//   SharedSFMem SF{};
//   SharedSysStateMem stateMem{};

//   ComponenConfig config{.id = ComponentID::Navigation, .CompCore = 0};
//   ComponenConfig config_{.id = ComponentID::SensorFusion, .CompCore=0};

//   SharedNavMemHandler navHandler(config.id, Us(2000), Nav);
//   SharedSFMemHandler sfHandler(config_.id, Us(2000),SF);
//   SharedSysStateMemHandler stateHandler(stateMem, Us(2000));

// SensorFusions composant_(config_, stateHandler, sfHandler);
//   Navigation composant(config, stateHandler, navHandler, sfHandler);

//   cout << "salam les roya \n";

//   while (true) {
//     sleep(2);
//   }

//   return 0;
// };

#include "drone/Components/MavlinkInterface/Driver/UartMavlinkLink.hpp"
#include "drone/Components/MavlinkInterface/MavlinkInterface.hpp"
#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/Components/SensorFusions/SensorsFusion.hpp"
#include "drone/Components/System Monitoring/Driver/UdpTelemetryDriver.hpp"
#include "drone/Components/System Monitoring/Driver/UdpVideoDriver.hpp"
#include "drone/Components/System Monitoring/SysMonitoring.hpp"
#include "drone/core/GlobalWatchdog.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"

#include <iostream>
#include <string_view>
#include <unistd.h>

using namespace TYPES;

namespace {

// Ports UDP loopback des drivers factices SysMonitoring (dev uniquement —
// remplacés par WFB_NG_Driver/CC1101_Driver quand le hardware RF sera prêt).
constexpr uint16_t kUdpTelMainLocalPort = 5601;
constexpr uint16_t kUdpTelMainPeerPort = 5602;
constexpr uint16_t kUdpTelSecLocalPort = 5611;
constexpr uint16_t kUdpTelSecPeerPort = 5612;
constexpr uint16_t kUdpVideoLocalPort = 5621;

// Port UART réel vers le FC — placeholder, cohérent avec les autres
// drivers UART du projet (GPS sur ttyAMA1, LD06 sur ttyAMA2).
constexpr const char *kFcUartPort = "/dev/ttyAMA0";

int runNavigation() {
  auto nav = attachSharedMemory<SharedNavMem>(kNavShmPath);
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);

  if (!nav || !sf || !sys) {
    std::cerr << "Navigation: echec attach shm\n";
    return 1;
  }

  SharedNavMemHandler navHandler(ComponentID::Navigation, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::Navigation, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::Navigation,
                                      Us(2000));

  ComponenConfig config{.id = ComponentID::Navigation, .CompCore = 0};
  Navigation composant(config, sysHandler, navHandler, sfHandler);

  while (true)
    sleep(2);
  return 0;
}

int runSensorFusion() {
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);

  if (!sf || !sys) {
    std::cerr << "SensorFusion: echec attach shm\n";
    return 1;
  }

  SharedSFMemHandler sfHandler(ComponentID::SensorFusion, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::SensorFusion,
                                      Us(2000));

  ComponenConfig config{.id = ComponentID::SensorFusion, .CompCore = 0};
  SensorFusions composant(config, sysHandler, sfHandler);

  while (true)
    sleep(2);
  return 0;
}

int runSysMonitoring() {
  auto nav = attachSharedMemory<SharedNavMem>(kNavShmPath);
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);
  auto com = attachSharedMemory<SharedComMem>(kComShmPath);

  if (!nav || !sf || !sys || !com) {
    std::cerr << "SysMonitoring: echec attach shm\n";
    return 1;
  }

  SharedNavMemHandler navHandler(ComponentID::SysMonitoring, Us(2000),
                                 *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::SysMonitoring, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::SysMonitoring,
                                      Us(2000));
  SharedComMemHandler comHandler(ComponentID::SysMonitoring, Us(2000),
                                 *com->ptr);

  // Drivers factices (UDP loopback) — a remplacer par WFB_NG_Driver/
  // CC1101_Driver quand le hardware RF sera cable. SysMonitoring ne
  // connait que les interfaces ITelemetryLink/IVideoSource, donc ce
  // remplacement ne touchera pas SysMonitoring.hpp/.cpp.
  UdpTelemetryDriver mainLink(kUdpTelMainLocalPort, kUdpTelMainPeerPort);
  UdpTelemetryDriver secLink(kUdpTelSecLocalPort, kUdpTelSecPeerPort);
  UdpVideoDriver videoSource(kUdpVideoLocalPort);

  ComponenConfig config{.id = ComponentID::SysMonitoring, .CompCore = 3};
  SysMonitoring composant(config, sysHandler, comHandler, navHandler, sfHandler,
                          mainLink, secLink, videoSource);

  while (true)
    sleep(2);
  return 0;
}

int runMavlinkInterface() {
  auto nav = attachSharedMemory<SharedNavMem>(kNavShmPath);
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);
  auto fc = attachSharedMemory<SharedFCStatus>(kFCShmPath);

  if (!nav || !sf || !sys || !fc) {
    std::cerr << "MavlinkInterface: echec attach shm\n";
    return 1;
  }

  SharedNavMemHandler navHandler(ComponentID::MavlinkInterface, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::MavlinkInterface, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::MavlinkInterface,
                                      Us(2000));
  SharedFCStatusHandler fcHandler(ComponentID::MavlinkInterface, Us(2000), *fc->ptr);

  UartMavlinkLink link(kFcUartPort);

  ComponenConfig config{.id = ComponentID::MavlinkInterface, .CompCore = 2};
  MavlinkInterface composant(config, sysHandler, fcHandler, navHandler, sfHandler, link);

  while (true)
    sleep(2);
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  std::string_view role = (argc > 1) ? argv[1] : "globalwatchdog";

  if (role == "navigation")
    return runNavigation();
  if (role == "sensorfusion")
    return runSensorFusion();
  if (role == "sysmonitoring")
    return runSysMonitoring();
  if (role == "mavlinkinterface")
    return runMavlinkInterface();

  // point d'entree par defaut : process parent
  GlobalWatchdog gwd(argv[0]);
  gwd.run();
  return 0;
}

// #include "drone/Components/Drivers/NEO-M8N.hpp"

// #include <chrono>
// #include <cstdio>
// #include <iomanip>
// #include <iostream>
// #include <thread>

// namespace {

// const char *errorToString(TYPES::DriverError err) {
//   switch (err) {
//   case TYPES::DriverError::None:
//     return "None";
//   case TYPES::DriverError::I2COpenFailed:
//     return "I2COpenFailed";
//   case TYPES::DriverError::I2CAddressFailed:
//     return "I2CAddressFailed";
//   case TYPES::DriverError::I2CReadFailed:
//     return "I2CReadFailed";
//   case TYPES::DriverError::I2CWriteFailed:
//     return "I2CWriteFailed";
//   case TYPES::DriverError::UARTOpenFailed:
//     return "UARTOpenFailed";
//   case TYPES::DriverError::UARTAttrGetFailed:
//     return "UARTAttrGetFailed";
//   case TYPES::DriverError::UARTAttrSetFailed:
//     return "UARTAttrSetFailed";
//   case TYPES::DriverError::UARTReadFailed:
//     return "UARTReadFailed";
//   case TYPES::DriverError::UARTWriteFailed:
//     return "UARTWriteFailed";
//   case TYPES::DriverError::NotInitialized:
//     return "NotInitialized";
//   case TYPES::DriverError::Timeout:
//     return "Timeout";
//   case TYPES::DriverError::InvalidData:
//     return "InvalidData";
//   case TYPES::DriverError::ConfigFailed:
//     return "ConfigFailed";
//   case TYPES::DriverError::NoNewData:
//     return "NoNewData";
//   }
//   return "Unknown";
// }

// void printTable(const GPSData &d, std::uint64_t frameCount,
//                 std::uint64_t errorCount) {
//   std::cout << "\033[2J\033[H"; // clear + retour en haut (ANSI)

//   std::cout
//       << "==================== NEO-M8N GPS — Live Data
//       ====================\n";
//   std::cout << std::left;

//   std::cout << std::setw(20) << "Fix 3D" << ": "
//             << (d.gpsFixOk ? "\033[32mOK\033[0m" : "\033[31mNO FIX\033[0m")
//             << "\n";

//   std::cout << std::fixed << std::setprecision(7);
//   std::cout << std::setw(20) << "Latitude" << ": " << d.coord.latitude <<
//   "\n"; std::cout << std::setw(20) << "Longitude" << ": " <<
//   d.coord.longitude
//             << "\n";
//   std::cout << std::setprecision(2);

//   std::cout << std::setw(20) << "Heading (deg)" << ": " << d.heading << "\n";
//   std::cout << std::setw(20) << "Speed (cm/s)" << ": " << d.speed << "\n";
//   std::cout << std::setw(20) << "Ground spd(cm/s)" << ": " << d.groundSpeed
//             << "\n";

//   std::cout << std::setw(20) << "VelNED N/E/D"
//             << ": " << d.velNED[0] << " / " << d.velNED[1] << " / "
//             << d.velNED[2] << " (cm/s)\n";

//   std::cout << std::setw(20) << "pAcc" << ": " << d.pAcc << " mm\n";
//   std::cout << std::setw(20) << "sAcc" << ": " << d.sAcc << " cm/s\n";

//   std::cout << std::setw(20) << "UTC Time"
//             << ": " << d.timeArray[0] << "-" << std::setfill('0')
//             << std::setw(2) << d.timeArray[1] << "-" << std::setw(2)
//             << d.timeArray[2] << " " << std::setw(2) << d.timeArray[3] << ":"
//             << std::setw(2) << d.timeArray[4] << ":" << std::setw(2)
//             << d.timeArray[5] << "\n"
//             << std::setfill(' ');

//   std::cout <<
//   "---------------------------------------------------------------"
//                "----\n";
//   std::cout << std::left << std::setw(6) << "SatID" << std::setw(12)
//             << "Strength"
//             << "Quality\n";
//   std::cout <<
//   "---------------------------------------------------------------"
//                "----\n";
//   for (const auto &sat : d.sats) {
//     std::cout << std::setw(6) << static_cast<int>(sat.id) << std::setw(12)
//               << static_cast<int>(sat.strength) <<
//               static_cast<int>(sat.quality)
//               << "\n";
//   }

//   std::cout <<
//   "---------------------------------------------------------------"
//                "----\n";
//   std::cout << "Frames OK: " << frameCount
//             << "   |   Erreurs read: " << errorCount << "\n";
//   std::cout <<
//   "==============================================================="
//                "====\n";
// }

// } // namespace

// int main() {
//   NEO_M8N gps("/dev/ttyAMA1");

//   std::cout << "Initialisation du GPS...\n";
//   if (auto err = gps.init()) {
//     std::cerr << "Echec init(): " << errorToString(*err) << "\n";
//     return 1;
//   }
//   std::cout << "GPS initialise. En attente de donnees...\n";
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));

//   GPSData lastData{};
//   std::uint64_t frameCount = 0;
//   std::uint64_t errorCount = 0;
//   bool hasData = false;

//   while (true) {
//     auto result = gps.update();

//     if (result.has_value()) {
//       lastData = *result;
//       hasData = true;
//       ++frameCount;
//       printTable(lastData, frameCount, errorCount);
//     } else if (result.error() != TYPES::DriverError::NoNewData) {
//       ++errorCount;
//       if (hasData) {
//         printTable(lastData, frameCount, errorCount);
//       }
//       std::cerr << "\n[Erreur update()] " << errorToString(result.error())
//                 << "\n";
//     }
//     // NoNewData : rien à afficher, on boucle simplement plus vite que le
//     débit
//     // UART.

//     std::this_thread::sleep_for(
//         std::chrono::milliseconds(20)); // ~50 Hz de polling
//   }
// }

// #include "drone/Components/Drivers/LIS3MDL.hpp"
// #include "drone/types.hpp"
// #include <iostream>
// #include <unistd.h>

// using namespace std;
// using namespace TYPES;

// int main() {
//   LIS3MDL sensor(0x1C, "/dev/i2c-1");

//   auto init = sensor.init();
//   if (init != DriverError::None) {
//     cout << "pb initialisation \n";
//   }

//   while (true) {
//     auto result = sensor.update();
//     if (!result) {
//       // std::cout << "pb lecture \n";
//       continue;
//     }
//     cout << "x :" << result->x << " y : " << result->y << " z : " <<
//     result->z
//          << "\n";
//     usleep(1000);
//   }

//   return 0;
// };

// #include "drone/Components/Drivers/LD06.hpp"

// #include <algorithm>
// #include <chrono>
// #include <cstdio>
// #include <iomanip>
// #include <iostream>
// #include <thread>

// namespace {

// const char* errorToString(TYPES::DriverError err) {
//     switch (err) {
//         case TYPES::DriverError::None:             return "None";
//         case TYPES::DriverError::I2COpenFailed:     return "I2COpenFailed";
//         case TYPES::DriverError::I2CAddressFailed:  return
//         "I2CAddressFailed"; case TYPES::DriverError::I2CReadFailed: return
//         "I2CReadFailed"; case TYPES::DriverError::I2CWriteFailed:    return
//         "I2CWriteFailed"; case TYPES::DriverError::UARTOpenFailed:    return
//         "UARTOpenFailed"; case TYPES::DriverError::UARTAttrGetFailed: return
//         "UARTAttrGetFailed"; case TYPES::DriverError::UARTAttrSetFailed:
//         return "UARTAttrSetFailed"; case TYPES::DriverError::UARTReadFailed:
//         return "UARTReadFailed"; case TYPES::DriverError::UARTWriteFailed:
//         return "UARTWriteFailed"; case TYPES::DriverError::NotInitialized:
//         return "NotInitialized"; case TYPES::DriverError::Timeout: return
//         "Timeout"; case TYPES::DriverError::InvalidData:       return
//         "InvalidData"; case TYPES::DriverError::ConfigFailed:      return
//         "ConfigFailed"; case TYPES::DriverError::NoNewData:         return
//         "NoNewData";
//     }
//     return "Unknown";
// }

// // Représente une distance (mm) par une barre proportionnelle, tronquée à
// maxBarLen. std::string distanceBar(std::uint16_t distanceMm, int maxBarLen,
// std::uint16_t maxRangeMm) {
//     if (distanceMm == 0) {
//         return "  (pas de mesure)";
//     }
//     int len = static_cast<int>((static_cast<double>(distanceMm) / maxRangeMm)
//     * maxBarLen); len = std::clamp(len, 1, maxBarLen); return
//     std::string(static_cast<std::size_t>(len), '#');
// }

// void printRadar(const LidarData& scan, std::uint64_t scanCount, std::uint64_t
// crcErrorCount) {
//     std::cout << "\033[2J\033[H"; // clear écran (ANSI)

//     constexpr int kSectorSizeDeg = 10;
//     constexpr int kNumSectors = 360 / kSectorSizeDeg;
//     constexpr int kMaxBarLen = 40;
//     constexpr std::uint16_t kMaxRangeMm = 8000; // 8m, échelle d'affichage
//     seulement

//     std::cout << "======================= LD06 Lidar — Scan #" << scanCount
//     << " =======================\n";

//     std::uint16_t minDist = 0xFFFF;
//     int minDistAngle = -1;
//     std::uint16_t maxDist = 0;
//     int validPoints = 0;

//     for (int sector = 0; sector < kNumSectors; ++sector) {
//         // Moyenne (sur les points valides) des distances du secteur pour
//         lisser l'affichage. std::uint32_t sum = 0; int count = 0; for (int a
//         = sector * kSectorSizeDeg; a < (sector + 1) * kSectorSizeDeg; ++a) {
//             std::uint16_t d = scan.distance_mm[static_cast<std::size_t>(a)];
//             if (d > 0) {
//                 sum += d;
//                 ++count;
//                 ++validPoints;
//                 if (d < minDist) { minDist = d; minDistAngle = a; }
//                 if (d > maxDist) { maxDist = d; }
//             }
//         }

//         std::uint16_t avgDist = count > 0 ? static_cast<std::uint16_t>(sum /
//         static_cast<std::uint32_t>(count)) : 0;

//         std::cout << std::setw(3) << (sector * kSectorSizeDeg) << "deg | "
//                    << std::setw(5) << (avgDist > 0 ? std::to_string(avgDist)
//                    + "mm" : "  --  ") << " | "
//                    << distanceBar(avgDist, kMaxBarLen, kMaxRangeMm) << "\n";
//     }

//     std::cout <<
//     "-------------------------------------------------------------------------\n";
//     std::cout << "Points valides   : " << validPoints << " / 360\n";
//     if (minDistAngle >= 0) {
//         std::cout << "Obstacle le + proche : " << minDist << " mm  a " <<
//         minDistAngle << " deg\n";
//     } else {
//         std::cout << "Obstacle le + proche : aucun\n";
//     }
//     std::cout << "Distance max     : " << maxDist << " mm\n";
//     std::cout << "Erreurs CRC (total) : " << crcErrorCount << "\n";
//     std::cout <<
//     "===========================================================================\n";
// }

// } // namespace

// int main() {
//     LD06 lidar("/dev/ttyAMA2");

//     std::cout << "Initialisation du LD06...\n";
//     if (auto err = lidar.init()) {
//         std::cerr << "Echec init(): " << errorToString(*err) << "\n";
//         return 1;
//     }
//     std::cout << "LD06 initialise. En attente d'un tour complet...\n";

//     std::uint64_t scanCount = 0;
//     std::uint64_t crcErrorCount = 0;

//     while (true) {
//         auto result = lidar.update();

//         if (result.has_value()) {
//             ++scanCount;
//             printRadar(*result, scanCount, crcErrorCount);
//         } else if (result.error() == TYPES::DriverError::InvalidData) {
//             ++crcErrorCount;
//             // Pas d'affichage à chaque paquet jeté, juste le compteur global
//             (voir tableau).
//         } else if (result.error() != TYPES::DriverError::NoNewData) {
//             std::cerr << "\n[Erreur update()] " <<
//             errorToString(result.error()) << "\n";
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(5)); // le LD06
//         débite vite (~4500 pts/s)
//     }
// }