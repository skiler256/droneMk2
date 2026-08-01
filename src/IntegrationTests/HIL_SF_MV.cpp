// src/IntegrationTests/HIL_SF_MV.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Binaire de test HIL (Hardware In the Loop) : fait tourner SensorFusion et
// MavlinkInterface dans le même process, face aux VRAIS capteurs (GPS/
// LIDAR) et au vrai FC (UART), sans GlobalWatchdog ni les autres composants.
// Publie ses propres segments shm dédiés (cf. drone/IntegrationTests/ShmPaths.hpp).
// Pas de magnétomètre ici : câblé directement sur le FC (cf. SENSORFUSION.md).
//
// Usage : ./hil_sf_mv
// Prérequis : GPS sur /dev/ttyAMA1, LIDAR sur /dev/ttyAMA2, FC sur
// /dev/ttyAMA0 — mêmes défauts que main.cpp.

#include "drone/Components/Drivers/LD06.hpp"
#include "drone/Components/Drivers/NEO-M8N.hpp"
#include "drone/Components/MavlinkInterface/Driver/UartMavlinkLink.hpp"
#include "drone/Components/MavlinkInterface/MavlinkInterface.hpp"
#include "drone/Components/SensorFusions/SensorsFusion.hpp"
#include "drone/IntegrationTests/ShmPaths.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <unistd.h>

using namespace TYPES;

namespace {

constexpr const char *kFcUartPort = "/dev/ttyAMA0";

std::atomic<bool> keepRunning{true};

void signalHandler(int) { keepRunning.store(false, std::memory_order_release); }

void installSignalHandler() {
  struct sigaction sa{};
  sa.sa_handler = signalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0; // pas de SA_RESTART, cf. GlobalWatchdog (même piège EINTR)
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

} // namespace

int main() {
  auto nav = publishSharedMemory<SharedNavMem>(HIL::kNavShmPath);
  auto sf = publishSharedMemory<SharedSFMem>(HIL::kSFShmPath);
  auto sys = publishSharedMemory<SharedSysStateMem>(HIL::kSysShmPath);
  auto fc = publishSharedMemory<SharedFCStatus>(HIL::kFCShmPath);
  auto cmd = publishSharedMemory<SharedCmdMem>(HIL::kCmdShmPath);
  auto com = publishSharedMemory<SharedComMem>(HIL::kComShmPath);

  if (!nav || !sf || !sys || !fc || !cmd || !com) {
    std::cerr << "HIL_SF_MV: echec publication shm\n";
    return 1;
  }

  installSignalHandler();

  SharedNavMemHandler navHandler(ComponentID::MavlinkInterface, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandlerMv(ComponentID::MavlinkInterface, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandlerMv(*sys->ptr, ComponentID::MavlinkInterface, Us(2000));
  SharedFCStatusHandler fcHandlerMv(ComponentID::MavlinkInterface, Us(2000), *fc->ptr);
  SharedCmdMemHandler cmdHandler(ComponentID::MavlinkInterface, Us(2000), *cmd->ptr);
  SharedComMemHandler comHandler(ComponentID::MavlinkInterface, Us(2000), *com->ptr);

  SharedSFMemHandler sfHandlerSf(ComponentID::SensorFusion, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandlerSf(*sys->ptr, ComponentID::SensorFusion, Us(2000));
  SharedFCStatusHandler fcHandlerSf(ComponentID::SensorFusion, Us(2000), *fc->ptr);

  UartMavlinkLink link(kFcUartPort);

  NEO_M8N gps;
  LD06 lidar;
  if (auto err = gps.init()) {
    std::cerr << "HIL_SF_MV: echec init GPS\n";
    return 1;
  }
  if (auto err = lidar.init()) {
    std::cerr << "HIL_SF_MV: echec init Lidar\n";
    return 1;
  }

  {
    // Bloc dédié : les composants doivent être détruits (Tasks stoppées et
    // jointes) AVANT le nettoyage shm ci-dessous — même piège que MV_SITL
    // (SEGV use-after-unmap constaté en session).
    ComponenConfig mvConfig{.id = ComponentID::MavlinkInterface, .CompCore = 0};
    MavlinkInterface mvComposant(mvConfig, sysHandlerMv, fcHandlerMv, navHandler,
                                 sfHandlerMv, cmdHandler, comHandler, link);
    // requestInitStreams() se déclenche automatiquement au premier
    // HEARTBEAT reçu du FC (cf. TRxTask::loop()), plus de bootstrap manuel.

    ComponenConfig sfConfig{.id = ComponentID::SensorFusion, .CompCore = 1};
    SensorFusions sfComposant(sfConfig, sysHandlerSf, sfHandlerSf, gps,
                              fcHandlerSf, lidar);

    std::cout << "HIL_SF_MV: pret (FC=" << kFcUartPort << ")\n";

    while (keepRunning.load(std::memory_order_acquire))
      sleep(1);
  }

  std::cout << "HIL_SF_MV: arret, nettoyage shm\n";
  destroySharedMemory(HIL::kNavShmPath, nav.value());
  destroySharedMemory(HIL::kSFShmPath, sf.value());
  destroySharedMemory(HIL::kSysShmPath, sys.value());
  destroySharedMemory(HIL::kFCShmPath, fc.value());
  destroySharedMemory(HIL::kCmdShmPath, cmd.value());
  destroySharedMemory(HIL::kComShmPath, com.value());

  return 0;
}
