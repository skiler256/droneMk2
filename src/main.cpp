#ifdef DRONE_USE_SITL_DRIVERS
#include "drone/Components/Drivers/UdpGpsDriver.hpp"
#include "drone/Components/Drivers/UdpLidarDriver.hpp"
#include "drone/Components/MavlinkInterface/Driver/UdpMavlinkLink.hpp"
#else
#include "drone/Components/Drivers/LD06.hpp"
#include "drone/Components/Drivers/NEO-M8N.hpp"
#include "drone/Components/MavlinkInterface/Driver/UartMavlinkLink.hpp"
#endif
#include "drone/Components/MavlinkInterface/MavlinkInterface.hpp"
#include "drone/Components/MissionControl/MissionControl.hpp"
#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/Components/SensorFusions/SensorsFusion.hpp"
#include "drone/Components/SystemMonitoring/Driver/LinuxThermalZoneDriver.hpp"
#include "drone/Components/SystemMonitoring/Driver/UdpVideoDriver.hpp"
#ifdef DRONE_USE_SITL_DRIVERS
#include "drone/Components/SystemMonitoring/Driver/UdpTelemetryDriver.hpp"
#else
#include "drone/Components/SystemMonitoring/Driver/CC1101_Driver.hpp"
#include "drone/Components/SystemMonitoring/Driver/WFB_NG_Driver.hpp"
#endif
#include "drone/Components/SystemMonitoring/SysMonitoring.hpp"
#include "drone/core/GlobalWatchdog.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"

#include <iostream>
#include <string_view>
#include <unistd.h>

using namespace TYPES;

namespace {

// Le PDL garde les mêmes ports en SITL et en vol : wfb-ng EST un transport
// UDP, et c'est tout l'intérêt du montage — wfb_tx lit 5602 et diffuse,
// wfb_rx livre sur 5601. Cf. deploy/wfb.conf.drone, qui doit rester d'accord
// avec ces deux nombres.
constexpr uint16_t kTelMainLocalPort = 5601;
constexpr uint16_t kTelMainPeerPort = 5602;
constexpr uint16_t kUdpVideoLocalPort = 5621;

#ifndef DRONE_USE_SITL_DRIVERS
// Port UART réel vers le FC — placeholder, cohérent avec les autres
// drivers UART du projet (GPS sur ttyAMA1, LD06 sur ttyAMA2).
constexpr const char *kFcUartPort = "/dev/ttyAMA0";

// Santé du PDL : le relais de statistiques publie ici (tools/wfb_stats_relay.py).
// 0 désactiverait la lecture ; le driver se dégraderait alors en UDP nu, donc
// en lien dont le drone ne sait plus s'il émet dans le vide.
constexpr uint16_t kWfbStatsPort = 5699;

// SDL : pas de ports, un bus SPI. 500 kHz est la vitesse à laquelle le lien a
// été CARACTÉRISÉ (8 Hz, 1,7 à 2,5 % de perte, cf. BRINGUP_PDL.md) ; la monter
// sans remesurer remplacerait un chiffre connu par une supposition. À ne pas
// confondre avec le côté ESP32, lui défectueux en dessous de 1 MHz.
constexpr const char *kSdlSpiDevice = "/dev/spidev0.0";
constexpr uint32_t kSdlSpiHz = 500000;
#else
// Ports UDP du lien secondaire, factices : en SITL les deux liens sont deux
// paires de ports loopback.
constexpr uint16_t kUdpTelSecLocalPort = 5611;
constexpr uint16_t kUdpTelSecPeerPort = 5612;
// Ports des drivers SITL ; 14550 = port MAVLink conventionnel, cf.
// sitl/SITL.md §2.
constexpr uint16_t kSitlMavlinkUdpPort = 14550;
constexpr uint16_t kSitlGpsUdpPort = 15100;
constexpr uint16_t kSitlLidarUdpPort = 15101;
#endif

int runNavigation(bool firstBoot) {
  auto nav = attachSharedMemory<SharedNavMem>(kNavShmPath);
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);
  auto cmd = attachSharedMemory<SharedCmdMem>(kCmdShmPath);

  if (!nav || !sf || !sys || !cmd) {
    std::cerr << "Navigation: echec attach shm\n";
    return 1;
  }

  SharedNavMemHandler navHandler(ComponentID::Navigation, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::Navigation, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::Navigation,
                                      Us(2000));
  SharedCmdMemHandler cmdHandler(ComponentID::Navigation, Us(2000), *cmd->ptr);

  ComponenConfig config{.id = ComponentID::Navigation, .CompCore = 0, .firstBoot = firstBoot};
  Navigation composant(config, sysHandler, navHandler, sfHandler, cmdHandler);

  if (auto err = composant.startupError()) {
    std::cerr << "Navigation: thread RT non demarre (errno "
              << *err << ") -- sortie, GlobalWatchdog escaladera\n";
    return 1;
  }
  while (true)
    sleep(2);
  return 0;
}

int runSensorFusion(bool firstBoot) {
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);
  auto fc = attachSharedMemory<SharedFCStatus>(kFCShmPath);

  if (!sf || !sys || !fc) {
    std::cerr << "SensorFusion: echec attach shm\n";
    return 1;
  }

  SharedSFMemHandler sfHandler(ComponentID::SensorFusion, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::SensorFusion,
                                      Us(2000));
  SharedFCStatusHandler fcHandler(ComponentID::SensorFusion, Us(2000), *fc->ptr);

#ifdef DRONE_USE_SITL_DRIVERS
  UdpGpsDriver gps(kSitlGpsUdpPort);
  UdpLidarDriver lidar(kSitlLidarUdpPort);
#else
  NEO_M8N gps;
  LD06 lidar;
#endif
  if (auto err = gps.init()) {
    std::cerr << "SensorFusion: echec init GPS\n";
    return 1;
  }
  if (auto err = lidar.init()) {
    std::cerr << "SensorFusion: echec init Lidar\n";
    return 1;
  }

  ComponenConfig config{.id = ComponentID::SensorFusion, .CompCore = 0, .firstBoot = firstBoot};
  SensorFusions composant(config, sysHandler, sfHandler, gps, fcHandler, lidar);

  if (auto err = composant.startupError()) {
    std::cerr << "SensorFusions: thread RT non demarre (errno "
              << *err << ") -- sortie, GlobalWatchdog escaladera\n";
    return 1;
  }
  while (true)
    sleep(2);
  return 0;
}

int runSysMonitoring(bool firstBoot) {
  auto nav = attachSharedMemory<SharedNavMem>(kNavShmPath);
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);
  auto com = attachSharedMemory<SharedComMem>(kComShmPath);
  auto fc = attachSharedMemory<SharedFCStatus>(kFCShmPath);
  auto cmd = attachSharedMemory<SharedCmdMem>(kCmdShmPath);

  if (!nav || !sf || !sys || !com || !fc || !cmd) {
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
  SharedFCStatusHandler fcHandler(ComponentID::SysMonitoring, Us(2000), *fc->ptr);
  SharedCmdMemHandler cmdHandler(ComponentID::SysMonitoring, Us(2000), *cmd->ptr);

#ifdef DRONE_USE_SITL_DRIVERS
  UdpTelemetryDriver mainLink(kTelMainLocalPort, kTelMainPeerPort);
  UdpTelemetryDriver secLink(kUdpTelSecLocalPort, kUdpTelSecPeerPort);
#else
  WFB_NG_Driver mainLink(kTelMainLocalPort, kTelMainPeerPort, kWfbStatsPort);
  CC1101_Driver secLink(kSdlSpiDevice, kSdlSpiHz);

  // Un SDL absent ne doit PAS empêcher SysMonitoring de démarrer : le drone a
  // deux liens justement pour survivre à la perte d'un seul, et refuser de
  // démarrer ferait boucler le GlobalWatchdog en respawn, ce qui coûterait
  // aussi le PDL. On démarre donc dégradé — mais bruyamment.
  //
  // Le signalement reste imparfait et c'est su : quality() rendra des
  // compteurs à zéro et un RSSI vide, que buildLinkQuality() écrit en
  // sdl_rssi = 0. Au sol, 0 dBm n'est pas une mesure plausible sur ce lien
  // (le réel va de -44 à -90), donc le tableau de bord peut le lire comme
  // « pas de mesure » — mais rien ne l'impose encore dans le format.
  if (auto err = secLink.init()) {
    std::cerr << "SysMonitoring: ATTENTION, init SDL (CC1101) echouee -- "
                 "demarrage avec le seul PDL, plus aucune redondance de lien\n";
  }
#endif
  // Pas de vraie source vidéo pour l'instant : la chaîne vidéo n'est pas
  // tranchée (HIL.md §5). Ce driver reste donc factice dans les deux cas.
  UdpVideoDriver videoSource(kUdpVideoLocalPort);
  // Pas de variante factice : /sys/class/thermal existe sur toute machine
  // Linux avec capteur (dev x86 compris), cf. LinuxThermalZoneDriver.hpp.
  LinuxThermalZoneDriver tempSource;

  ComponenConfig config{.id = ComponentID::SysMonitoring, .CompCore = 3, .firstBoot = firstBoot};
  SysMonitoring composant(config, sysHandler, comHandler, navHandler, sfHandler,
                          fcHandler, cmdHandler, mainLink, secLink, videoSource,
                          tempSource);

  if (auto err = composant.startupError()) {
    std::cerr << "SysMonitoring: thread RT non demarre (errno "
              << *err << ") -- sortie, GlobalWatchdog escaladera\n";
    return 1;
  }
  while (true)
    sleep(2);
  return 0;
}

int runMavlinkInterface(bool firstBoot) {
  auto nav = attachSharedMemory<SharedNavMem>(kNavShmPath);
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);
  auto fc = attachSharedMemory<SharedFCStatus>(kFCShmPath);
  auto cmd = attachSharedMemory<SharedCmdMem>(kCmdShmPath);
  auto com = attachSharedMemory<SharedComMem>(kComShmPath);

  if (!nav || !sf || !sys || !fc || !cmd || !com) {
    std::cerr << "MavlinkInterface: echec attach shm\n";
    return 1;
  }

  SharedNavMemHandler navHandler(ComponentID::MavlinkInterface, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::MavlinkInterface, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::MavlinkInterface,
                                      Us(2000));
  SharedFCStatusHandler fcHandler(ComponentID::MavlinkInterface, Us(2000), *fc->ptr);
  SharedCmdMemHandler cmdHandler(ComponentID::MavlinkInterface, Us(2000), *cmd->ptr);
  SharedComMemHandler comHandler(ComponentID::MavlinkInterface, Us(2000), *com->ptr);

#ifdef DRONE_USE_SITL_DRIVERS
  UdpMavlinkLink link(kSitlMavlinkUdpPort);
#else
  UartMavlinkLink link(kFcUartPort);
#endif

  ComponenConfig config{.id = ComponentID::MavlinkInterface, .CompCore = 2, .firstBoot = firstBoot};
  MavlinkInterface composant(config, sysHandler, fcHandler, navHandler, sfHandler,
                             cmdHandler, comHandler, link);

  if (auto err = composant.startupError()) {
    std::cerr << "MavlinkInterface: thread RT non demarre (errno "
              << *err << ") -- sortie, GlobalWatchdog escaladera\n";
    return 1;
  }
  while (true)
    sleep(2);
  return 0;
}

int runMissionControl(bool firstBoot) {
  auto nav = attachSharedMemory<SharedNavMem>(kNavShmPath);
  auto sf = attachSharedMemory<SharedSFMem>(kSFShmPath);
  auto sys = attachSharedMemory<SharedSysStateMem>(kSysShmPath);
  auto fc = attachSharedMemory<SharedFCStatus>(kFCShmPath);
  auto cmd = attachSharedMemory<SharedCmdMem>(kCmdShmPath);
  auto com = attachSharedMemory<SharedComMem>(kComShmPath);

  if (!nav || !sf || !sys || !fc || !cmd || !com) {
    std::cerr << "MissionControl: echec attach shm\n";
    return 1;
  }

  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::MissionControl, Us(2000));
  SharedCmdMemHandler cmdHandler(ComponentID::MissionControl, Us(2000), *cmd->ptr);
  SharedFCStatusHandler fcHandler(ComponentID::MissionControl, Us(2000), *fc->ptr);
  SharedNavMemHandler navHandler(ComponentID::MissionControl, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::MissionControl, Us(2000), *sf->ptr);
  SharedComMemHandler comHandler(ComponentID::MissionControl, Us(2000), *com->ptr);

  ComponenConfig config{.id = ComponentID::MissionControl, .CompCore = 1, .firstBoot = firstBoot};
  MissionControl composant(config, sysHandler, cmdHandler, fcHandler, navHandler,
                           sfHandler, comHandler);

  if (auto err = composant.startupError()) {
    std::cerr << "MissionControl: thread RT non demarre (errno "
              << *err << ") -- sortie, GlobalWatchdog escaladera\n";
    return 1;
  }
  while (true)
    sleep(2);
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  std::string_view role = (argc > 1) ? argv[1] : "globalwatchdog";

  // Absent = premier démarrage, y compris en lancement manuel (§4.3).
  const bool firstBoot =
      !(argc > 2 && std::string_view(argv[2]) == "respawn");

  if (role == "navigation")
    return runNavigation(firstBoot);
  if (role == "sensorfusion")
    return runSensorFusion(firstBoot);
  if (role == "sysmonitoring")
    return runSysMonitoring(firstBoot);
  if (role == "mavlinkinterface")
    return runMavlinkInterface(firstBoot);
  if (role == "missioncontrol")
    return runMissionControl(firstBoot);

  // point d'entree par defaut : process parent
  GlobalWatchdog gwd(argv[0]);
  gwd.run();
  return 0;
}
