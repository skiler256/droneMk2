// Test intermédiaire : Navigation + MavlinkInterface face à ArduPilot SITL,
// SANS SensorFusion — valide le séquencement/guidage sans monter le moteur
// 3D ni le modèle capteur. Le GPS simulé reste côté SITL.
// SharedSFMem.state, normalement rempli par EKFTask, l'est ici par un relais
// minimal (mêmes données et même garde de fraîcheur) — jamais en prod.
// Usage : ./nav_sitl — injection via udp_bridge (namespace NAV_SITL).

#include "drone/Components/MavlinkInterface/Driver/UdpMavlinkLink.hpp"
#include "drone/Components/MavlinkInterface/MavlinkInterface.hpp"
#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/IntegrationTests/ShmPaths.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace TYPES;

namespace {

constexpr uint16_t kSitlUdpLocalPort = 14550;
constexpr Ms kAgeThrEKF{200}; // même seuil que EKFTask côté SF

std::atomic<bool> keepRunning{true};

void signalHandler(int) { keepRunning.store(false, std::memory_order_release); }

void installSignalHandler() {
  struct sigaction sa {};
  sa.sa_handler = signalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

// Relais FC -> SharedSFMem.state, remplace EKFTask pour ce test — même
// garde de fraîcheur, pas de HomeCalibrator (offset toujours nul, comme
// en prod pour l'instant).
void relayFcToSF(SharedFCStatusHandler &fc, SharedSFMemHandler &sf) {
  auto attitude = fc.getAttitude();
  auto attitudeTs = fc.getAttitudeTs();
  auto posVel = fc.getLocalPosVel();
  auto posVelTs = fc.getLocalPosVelTs();
  if (!attitude || !attitudeTs || !posVel || !posVelTs)
    return;

  auto now = Clock::now();
  bool stale = UTILITIES::msBetween(*attitudeTs, now) > kAgeThrEKF ||
              UTILITIES::msBetween(*posVelTs, now) > kAgeThrEKF;
  if (stale)
    return;

  DynamicState state{.ts = now,
                     .posNed = posVel->pos,
                     .velNed = posVel->vel,
                     .attitude = *attitude};
  sf.setDynamicState(state);
}

} // namespace

int main() {
  auto nav = publishSharedMemory<SharedNavMem>(NAV_SITL::kNavShmPath);
  auto sf = publishSharedMemory<SharedSFMem>(NAV_SITL::kSFShmPath);
  auto sys = publishSharedMemory<SharedSysStateMem>(NAV_SITL::kSysShmPath);
  auto fc = publishSharedMemory<SharedFCStatus>(NAV_SITL::kFCShmPath);
  auto cmd = publishSharedMemory<SharedCmdMem>(NAV_SITL::kCmdShmPath);
  auto com = publishSharedMemory<SharedComMem>(NAV_SITL::kComShmPath);

  if (!nav || !sf || !sys || !fc || !cmd || !com) {
    std::cerr << "NAV_SITL: echec publication shm\n";
    return 1;
  }

  installSignalHandler();

  SharedNavMemHandler navHandlerMv(ComponentID::MavlinkInterface, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandlerMv(ComponentID::MavlinkInterface, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandlerMv(*sys->ptr, ComponentID::MavlinkInterface, Us(2000));
  SharedFCStatusHandler fcHandlerMv(ComponentID::MavlinkInterface, Us(2000), *fc->ptr);
  SharedCmdMemHandler cmdHandlerMv(ComponentID::MavlinkInterface, Us(2000), *cmd->ptr);
  SharedComMemHandler comHandlerMv(ComponentID::MavlinkInterface, Us(2000), *com->ptr);

  SharedNavMemHandler navHandlerNav(ComponentID::Navigation, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandlerNav(ComponentID::Navigation, Us(2000), *sf->ptr);
  SharedSysStateMemHandler sysHandlerNav(*sys->ptr, ComponentID::Navigation, Us(2000));
  SharedCmdMemHandler cmdHandlerNav(ComponentID::Navigation, Us(2000), *cmd->ptr);

  // Le relais utilise son propre jeton (Navigation, cohérent avec le fait
  // qu'il remplace EKFTask côté SF — écrit dans le même segment que
  // sfHandlerNav, jamais lu en écriture concurrente par MV).
  SharedFCStatusHandler fcHandlerRelay(ComponentID::Navigation, Us(2000), *fc->ptr);
  SharedSFMemHandler sfHandlerRelay(ComponentID::Navigation, Us(2000), *sf->ptr);

  UdpMavlinkLink link(kSitlUdpLocalPort);

  {
    // Même piège use-after-unmap que MV_SITL/HIL_SF_MV : composants
    // détruits avant le nettoyage shm.
    ComponenConfig mvConfig{.id = ComponentID::MavlinkInterface, .CompCore = 0};
    MavlinkInterface mvComposant(mvConfig, sysHandlerMv, fcHandlerMv, navHandlerMv,
                                 sfHandlerMv, cmdHandlerMv, comHandlerMv, link);

    ComponenConfig navConfig{.id = ComponentID::Navigation, .CompCore = 1};
    Navigation navComposant(navConfig, sysHandlerNav, navHandlerNav, sfHandlerNav,
                            cmdHandlerNav);

    std::cout << "NAV_SITL: pret, en attente de SITL sur le port "
              << kSitlUdpLocalPort << "\n";

    while (keepRunning.load(std::memory_order_acquire)) {
      relayFcToSF(fcHandlerRelay, sfHandlerRelay);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  std::cout << "NAV_SITL: arret, nettoyage shm\n";
  destroySharedMemory(NAV_SITL::kNavShmPath, nav.value());
  destroySharedMemory(NAV_SITL::kSFShmPath, sf.value());
  destroySharedMemory(NAV_SITL::kSysShmPath, sys.value());
  destroySharedMemory(NAV_SITL::kFCShmPath, fc.value());
  destroySharedMemory(NAV_SITL::kCmdShmPath, cmd.value());
  destroySharedMemory(NAV_SITL::kComShmPath, com.value());

  return 0;
}
