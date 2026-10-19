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
#include "drone/Components/SystemMonitoring/Driver/UdpTelemetryDriver.hpp"
#include "drone/Components/SystemMonitoring/Driver/UdpVideoDriver.hpp"
#include "drone/Components/SystemMonitoring/SysMonitoring.hpp"
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

#ifndef DRONE_USE_SITL_DRIVERS
// Port UART réel vers le FC — placeholder, cohérent avec les autres
// drivers UART du projet (GPS sur ttyAMA1, LD06 sur ttyAMA2).
constexpr const char *kFcUartPort = "/dev/ttyAMA0";
#else
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

  // À remplacer par WFB_NG_Driver/CC1101_Driver : SysMonitoring ne connaît
  // que les interfaces, le remplacement ne le touchera pas.
  UdpTelemetryDriver mainLink(kUdpTelMainLocalPort, kUdpTelMainPeerPort);
  UdpTelemetryDriver secLink(kUdpTelSecLocalPort, kUdpTelSecPeerPort);
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
