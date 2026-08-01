// src/IntegrationTests/MC_SITL.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Binaire de dev : MissionControl SEUL, face à un plan de mission prédéfini
// et un faux Nav minimal (pas de vol réel, pas de SITL ArduPilot) — test
// ciblé de la state machine/séquenceur/failsafe, pas du pilotage. Le SITL
// général avec moteur 3D viendra après (cf. INTEGRATION_TEST.md).
//
// Le "faux Nav" ci-dessous simule uniquement la progression des segments
// (arme -> engage -> Atteint après un délai fixe) — jamais dans le code de
// prod. La santé de Nav (ComponentHealth) et l'armement FC
// (SharedFCStatus::armed) sont initialisés une seule fois au démarrage puis
// laissés à la main du dashboard (UDP_bridge, namespace MC_SITL) : les
// re-écrire en boucle ici empêcherait toute injection de panne de tenir.
//
// Usage : ./mc_sitl
// Injection : via UDP_bridge (CMD=GCSCMD/CODE/HEALTH/FCARMED, namespace MC_SITL).

#include "drone/Components/MissionControl/MissionControl.hpp"
#include "drone/IntegrationTests/ShmPaths.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace TYPES;

namespace {

// Simule le temps de vol jusqu'à un segment — arbitraire, choisi court pour
// itérer vite sur les tests (pas un modèle physique, cf. commentaire en tête).
constexpr Ms kFakeFlightDuration{2000};

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

// Plan prédéfini : 2 waypoints, un HOLD, un 3e waypoint de retour — assez
// pour exercer amorçage + auto-promotion + segment temporisé + fin de
// mission. Valeurs arbitraires, cf. MISSIONPLAN.csv pour le format.
[[nodiscard]] MISSIONPLAN_CONV::MissionPlan buildTestPlan() {
  MISSIONPLAN_CONV::MissionPlan plan{};
  plan.segments.push_back(MISSIONPLAN::WaypointSegment{
      .north = 10.0f, .east = 0.0f, .alt = -10.0f, .speed = 2.0f, .acceptRadius = 2.0f});
  plan.segments.push_back(MISSIONPLAN::HoldSegment{.duration = 3.0f});
  plan.segments.push_back(MISSIONPLAN::WaypointSegment{
      .north = 10.0f, .east = 10.0f, .alt = -10.0f, .speed = 2.0f, .acceptRadius = 2.0f});
  plan.segments.push_back(MISSIONPLAN::WaypointSegment{
      .north = 0.0f, .east = 0.0f, .alt = -10.0f, .speed = 2.0f, .acceptRadius = 2.0f});
  return plan;
}

// Écrit le plan de test au chemin lu par MissionControl (cf.
// DRONE_MISSION_PLAN_PATH, positionné plus bas avant construction du
// composant) — même sérialisation que la prod (MISSIONPLAN_CONV::serialize).
[[nodiscard]] bool writeTestPlan(const std::string &path) {
  auto bytes = MISSIONPLAN_CONV::serialize(buildTestPlan());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file)
    return false;
  file.write(reinterpret_cast<const char *>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
  return static_cast<bool>(file);
}

// État du faux Nav — un seul segment "en vol" à la fois, comme le vrai Nav.
struct FakeNavState {
  uint32_t engagedSeq{0};
  TimePoint engagedAt{};
};

void fakeNavTick(SharedCmdMemHandler &cmdNav, SharedNavMemHandler &navNav,
                 FakeNavState &state) {
  auto now = Clock::now();

  // Amorçage (MCO005) : rien à valider ici (TMiSequencer ne vérifie que
  // segmentProgress, pas ce report), on draine juste pour ne pas remplir
  // navCmds indéfiniment.
  if (auto order = cmdNav.popNavCommand())
    [[maybe_unused]] auto pushed = cmdNav.pushNavReport(order->seq, order->code, true);

  if (auto armed = cmdNav.getArmedSegment()) {
    if (armed->seq != 0 && armed->seq != state.engagedSeq) {
      cmdNav.setEngagedSegment(*armed);
      state.engagedSeq = armed->seq;
      state.engagedAt = now;
    }
  }

  if (state.engagedSeq != 0) {
    bool arrived = UTILITIES::msBetween(state.engagedAt, now) > kFakeFlightDuration;
    SegmentProgress p{.seq = state.engagedSeq,
                      .status = arrived ? SegmentStatus::Atteint : SegmentStatus::EnCours,
                      .ts = now};
    cmdNav.setSegmentProgress(p);
  }

  // Fraîcheur pour TMiSequencer (cf. MISSIONCONTROL.md §5) — indépendant de
  // ComponentHealth, qui lui reste à la main du dashboard une fois posé.
  navNav.setCommand(NavCommand{.ts = now});
}

} // namespace

int main() {
  auto nav = publishSharedMemory<SharedNavMem>(MC_SITL::kNavShmPath);
  auto sf = publishSharedMemory<SharedSFMem>(MC_SITL::kSFShmPath);
  auto sys = publishSharedMemory<SharedSysStateMem>(MC_SITL::kSysShmPath);
  auto fc = publishSharedMemory<SharedFCStatus>(MC_SITL::kFCShmPath);
  auto cmd = publishSharedMemory<SharedCmdMem>(MC_SITL::kCmdShmPath);
  auto com = publishSharedMemory<SharedComMem>(MC_SITL::kComShmPath);

  if (!nav || !sf || !sys || !fc || !cmd || !com) {
    std::cerr << "MC_SITL: echec publication shm\n";
    return 1;
  }

  installSignalHandler();

  // Chemin de plan dédié à ce binaire — cf. MissionControl.cpp,
  // DRONE_MISSION_PLAN_PATH (évite d'écrire dans /etc/drone en dev).
  const std::string planPath = "/tmp/drone_mc_sitl_plan.bin";
  if (!writeTestPlan(planPath)) {
    std::cerr << "MC_SITL: echec ecriture plan de test (" << planPath << ")\n";
    return 1;
  }
  setenv("DRONE_MISSION_PLAN_PATH", planPath.c_str(), /*overwrite=*/1);

  SharedSysStateMemHandler sysHandlerMc(*sys->ptr, ComponentID::MissionControl, Us(2000));
  SharedCmdMemHandler cmdHandlerMc(ComponentID::MissionControl, Us(2000), *cmd->ptr);
  SharedFCStatusHandler fcHandlerMc(ComponentID::MissionControl, Us(2000), *fc->ptr);
  SharedNavMemHandler navHandlerMc(ComponentID::MissionControl, Us(2000), *nav->ptr);
  SharedSFMemHandler sfHandlerMc(ComponentID::MissionControl, Us(2000), *sf->ptr);
  SharedComMemHandler comHandlerMc(ComponentID::MissionControl, Us(2000), *com->ptr);

  SharedCmdMemHandler cmdHandlerNav(ComponentID::Navigation, Us(2000), *cmd->ptr);
  SharedNavMemHandler navHandlerNav(ComponentID::Navigation, Us(2000), *nav->ptr);
  SharedSysStateMemHandler sysHandlerNav(*sys->ptr, ComponentID::Navigation, Us(2000));

  // Baseline une seule fois — cf. commentaire en tête de fichier, le
  // dashboard prend le relais ensuite (injection de panne).
  sysHandlerNav.setHealth(ComponentHealth::NOMINAL);
  fcHandlerMc.setArmed(false);

  FakeNavState fakeNav{};

  {
    // Même piège use-after-unmap que MV_SITL/NAV_SITL : composant détruit
    // avant le nettoyage shm.
    ComponenConfig config{.id = ComponentID::MissionControl, .CompCore = 0};
    MissionControl composant(config, sysHandlerMc, cmdHandlerMc, fcHandlerMc,
                             navHandlerMc, sfHandlerMc, comHandlerMc);

    std::cout << "MC_SITL: pret (plan: " << planPath << ")\n";

    while (keepRunning.load(std::memory_order_acquire)) {
      fakeNavTick(cmdHandlerNav, navHandlerNav, fakeNav);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::cout << "MC_SITL: arret, nettoyage shm\n";
  destroySharedMemory(MC_SITL::kNavShmPath, nav.value());
  destroySharedMemory(MC_SITL::kSFShmPath, sf.value());
  destroySharedMemory(MC_SITL::kSysShmPath, sys.value());
  destroySharedMemory(MC_SITL::kFCShmPath, fc.value());
  destroySharedMemory(MC_SITL::kCmdShmPath, cmd.value());
  destroySharedMemory(MC_SITL::kComShmPath, com.value());

  return 0;
}
