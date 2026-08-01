// src/SITL/MV_SITL.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Binaire de dev : fait tourner MavlinkInterface seule face à ArduPilot
// SITL, sans GlobalWatchdog. Publie ses propres segments shm (dédiés SITL,
// cf. drone/IntegrationTests/ShmPaths.hpp) — c'est lui le propriétaire ici, contrairement
// à runMavlinkInterface() de main.cpp qui s'attache à des segments publiés
// par GlobalWatchdog.
//
// Usage : ./mv_sitl
// Prérequis : SITL doit pousser son flux vers ce Pi, cf. MAVLINKINTERFACE.md
// et la doc de session — sim_vehicle.py ... --out=udp:<IP_PI>:14550, ou
// "output add <IP_PI>:14550" dans la console MAVProxy.

#include "drone/Components/MavlinkInterface/Driver/UdpMavlinkLink.hpp"
#include "drone/Components/MavlinkInterface/MavlinkInterface.hpp"
#include "drone/IntegrationTests/ShmPaths.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <unistd.h>

using namespace TYPES;

namespace {

constexpr uint16_t kSitlUdpLocalPort = 14550; // vérifié manuellement en session

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
  auto nav = publishSharedMemory<SharedNavMem>(SITL::kNavShmPath);
  auto sf = publishSharedMemory<SharedSFMem>(SITL::kSFShmPath);
  auto sys = publishSharedMemory<SharedSysStateMem>(SITL::kSysShmPath);
  auto fc = publishSharedMemory<SharedFCStatus>(SITL::kFCShmPath);
  auto cmd = publishSharedMemory<SharedCmdMem>(SITL::kCmdShmPath);
  auto com = publishSharedMemory<SharedComMem>(SITL::kComShmPath);

  if (!nav || !sf || !sys || !fc || !cmd || !com) {
    std::cerr << "MV_SITL: echec publication shm\n";
    return 1;
  }

  installSignalHandler();

  SharedNavMemHandler navHandler(ComponentID::MavlinkInterface, Us(2000),
                                 *nav->ptr);
  SharedSFMemHandler sfHandler(ComponentID::MavlinkInterface, Us(2000),
                               *sf->ptr);
  SharedSysStateMemHandler sysHandler(*sys->ptr, ComponentID::MavlinkInterface,
                                      Us(2000));
  SharedFCStatusHandler fcHandler(ComponentID::MavlinkInterface, Us(2000),
                                  *fc->ptr);
  SharedCmdMemHandler cmdHandler(ComponentID::MavlinkInterface, Us(2000),
                                 *cmd->ptr);
  SharedComMemHandler comHandler(ComponentID::MavlinkInterface, Us(2000),
                                 *com->ptr);

  UdpMavlinkLink link(kSitlUdpLocalPort);

  {
    // Bloc dédié : composant doit être détruit (donc ses Tasks stoppées et
    // jointes, cf. Task::~Task()) AVANT le nettoyage shm ci-dessous — sinon
    // les threads TTx/TTxACK/TRx continuent d'accéder à la shm pendant son
    // démontage (SEGV constaté en session, use-after-unmap).
    ComponenConfig config{.id = ComponentID::MavlinkInterface, .CompCore = 0};
    MavlinkInterface composant(config, sysHandler, fcHandler, navHandler,
                               sfHandler, cmdHandler, comHandler, link);
    // requestInitStreams() se déclenche automatiquement au premier
    // HEARTBEAT reçu du FC (cf. TRxTask::loop()), plus de bootstrap manuel.

    std::cout << "MV_SITL: pret, en attente de SITL sur le port "
              << kSitlUdpLocalPort << "\n";

    while (keepRunning.load(std::memory_order_acquire))
      sleep(1);
  }

  std::cout << "MV_SITL: arret, nettoyage shm\n";
  destroySharedMemory(SITL::kNavShmPath, nav.value());
  destroySharedMemory(SITL::kSFShmPath, sf.value());
  destroySharedMemory(SITL::kSysShmPath, sys.value());
  destroySharedMemory(SITL::kFCShmPath, fc.value());
  destroySharedMemory(SITL::kCmdShmPath, cmd.value());
  destroySharedMemory(SITL::kComShmPath, com.value());

  return 0;
}
