// tests/test_watchdog.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Couvre ComponentWatchdog (ring buffer de restart, escalade vers exit(1),
// polling des erreurs shm), Task (câblage TaskConfig -> ComponentWatchdog),
// et ComponentBase (décision hot-start/cold-start/DEAD).
//
// Note d'environnement : ComponentWatchdog/Task lancent un thread réel via
// SCHED_FIFO (temps réel) dans leur constructeur/start(). La plupart des
// tests ci-dessous pilotent loop()/heartbeat() à la main et n'ont pas
// besoin de ce thread réel. La section "exécution réelle" en bas EN a
// besoin (RLIMIT_RTPRIO suffisant, ou root) — elle se skip proprement
// (GTEST_SKIP) si SCHED_FIFO n'est pas disponible, pour rester utilisable
// dans un sandbox sans privilège RT.

#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <pthread.h>
#include <thread>

using namespace TYPES;

namespace {
// Sonde rapide : peut-on réellement créer un thread SCHED_FIFO ici ? Évite
// que les tests d'exécution réelle échouent silencieusement/bizarrement
// dans un environnement sans CAP_SYS_NICE / RLIMIT_RTPRIO.
bool rtSchedulingAvailable() {
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  struct sched_param param {};
  param.sched_priority = 1;
  pthread_attr_setschedparam(&attr, &param);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

  pthread_t t;
  int err = pthread_create(
      &t, &attr, [](void *) -> void * { return nullptr; }, nullptr);
  pthread_attr_destroy(&attr);

  if (err != 0)
    return false;
  pthread_join(t, nullptr);
  return true;
}
} // namespace

// ─── ComponentWatchdog : ring buffer de restart ────────────────────────────

TEST(ComponentWatchdogTest, RestartsBelowThresholdDoNotEscalate) {
  ComponentWatchdog wd(0, 0);
  int startCount = 0;

  // timeout=0 : le prochain loop() considère systématiquement la task comme
  // morte (elapsed > 0 dès qu'un peu de temps s'est écoulé).
  wd.registerTask(
      1, [&] { ++startCount; }, [] {}, Ms(0), /*maxRestart=*/3, Ms(60000));

  for (int i = 0; i < 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    wd.loop();
  }

  EXPECT_EQ(startCount, 2);
}

TEST(ComponentWatchdogDeathTest, EscalatesToExitAfterTooManyRestarts) {
  EXPECT_EXIT(
      {
        ComponentWatchdog wd(0, 0);
        wd.registerTask(
            1, [] {}, [] {}, Ms(0), /*maxRestart=*/3, Ms(60000));

        for (int i = 0; i < 3; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          wd.loop();
        }

        // Ne doit jamais être atteint : le 3e loop() doit avoir escaladé
        // vers shutdown()/_exit(1) avant.
        _exit(0);
      },
      ::testing::ExitedWithCode(1), "");
}

TEST(ComponentWatchdogTest, RestartWindowPruningPreventsEscalation) {
  // Fenêtre très courte : chaque restart sort de la fenêtre avant le
  // suivant (on dort plus longtemps que restartWindow entre deux loop()),
  // donc restartHistory ne dépasse jamais 1 entrée malgré de nombreux
  // redémarrages étalés dans le temps.
  ComponentWatchdog wd(0, 0);
  int startCount = 0;
  constexpr Ms window(20);

  wd.registerTask(
      1, [&] { ++startCount; }, [] {}, Ms(0), /*maxRestart=*/3, window);

  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30)); // > window
    wd.loop();
  }

  EXPECT_EQ(startCount, 5);
}

TEST(ComponentWatchdogTest, HeartbeatPreventsRestart) {
  ComponentWatchdog wd(0, 0);
  int startCount = 0;

  wd.registerTask(
      1, [&] { ++startCount; }, [] {}, Ms(1000), /*maxRestart=*/3, Ms(60000));

  wd.heartbeat(1);
  wd.loop();

  EXPECT_EQ(startCount, 0);
}

// ─── ComponentWatchdog : polling des erreurs shm ───────────────────────────

TEST(ComponentWatchdogTest, PicksUpShmErrorsFromRegisteredSources) {
  ComponentWatchdog wd(0, 0);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysHandler(sys, ComponentID::Navigation, Us(20000));
  wd.registerShmSource(sysHandler);

  sysHandler.setHealth(ComponentHealth::NOMINAL);
  // Corruption directe, hors API — simule un torn write.
  sys.compHealth[static_cast<size_t>(ComponentID::Navigation)] =
      ComponentHealth::DEAD;

  auto result = sysHandler.getHealth(ComponentID::Navigation);
  EXPECT_FALSE(result.has_value());

  wd.loop(); // doit consommer l'erreur en attente sur la source enregistrée

  EXPECT_EQ(wd.lastShmError(), shmError::corrupt);
  EXPECT_EQ(wd.shmErrorCount(), 1u);
}

// ─── Task : câblage TaskConfig -> ComponentWatchdog ────────────────────────

class TestTask : public Task {
public:
  using Task::Task;
  std::atomic<int> loopCount{0};

protected:
  void loop() override { loopCount.fetch_add(1, std::memory_order_relaxed); }
};

TEST(TaskTest, ConstructorWiresRestartConfigIntoWatchdog) {
  ComponentWatchdog wd(0, 0);
  TaskConfig cfg{
      .id = 7,
      .RTpriority = 0,
      .core = 0,
      .loopFrequency = Hz(50.0f),
      .timeout = Ms(0),
      .maxRestart = 2,
      .restartWindow = Ms(60000),
  };
  TestTask task(cfg, wd);

  // 1er restart (< maxRestart) : ne doit pas escalader.
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  wd.loop();
  SUCCEED(); // si on arrive ici, pas d'exit(1) prématuré
}

namespace {
// Extrait dans une fonction séparée : le préprocesseur ne suit pas les
// accolades `{}` (seulement les parenthèses) pour découper les arguments
// d'une macro — les virgules d'un TaskConfig{...} inline dans EXPECT_EXIT
// seraient (à tort) comptées comme séparateurs d'arguments de la macro.
void escalateAfterTwoRestarts() {
  ComponentWatchdog wd(0, 0);
  TaskConfig cfg{
      .id = 7,
      .RTpriority = 0,
      .core = 0,
      .loopFrequency = Hz(50.0f),
      .timeout = Ms(0),
      .maxRestart = 2,
      .restartWindow = Ms(60000),
  };
  TestTask task(cfg, wd);

  for (int i = 0; i < 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    wd.loop();
  }
  _exit(0);
}
} // namespace

TEST(TaskDeathTest, ConstructorWiresRestartThresholdCorrectly) {
  // Vérifie que maxRestart=2 transmis via TaskConfig déclenche bien
  // l'escalade au 2e restart, pas avant ni après — preuve que Task
  // transmet fidèlement sa config à ComponentWatchdog::registerTask.
  EXPECT_EXIT(escalateAfterTwoRestarts(), ::testing::ExitedWithCode(1), "");
}

// ─── ComponentBase : décision hot-start/cold-start/DEAD ────────────────────

namespace {
ComponenConfig makeConfig() {
  return ComponenConfig{
      .max_cold_start = 1,
      .max_cold_start_interval = Ms(60000),
      .max_hot_start = 3,
      .max_hot_start_interval = Ms(60000),
      .shm_timeout = Us(20000),
      .id = ComponentID::Navigation,
      .CompCore = 0,
  };
}
} // namespace

TEST(ComponentBaseTest, FreshStartIsNominal) {
  SharedNavMem nav{};
  SharedNavMemHandler comp(ComponentID::Navigation, Us(20000), nav);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::Navigation, Us(20000));

  ComponentBase<0> base(makeConfig(), comp, sysState);

  auto health = sysState.getHealth(ComponentID::Navigation);
  ASSERT_TRUE(health.has_value());
  EXPECT_EQ(*health, ComponentHealth::NOMINAL);
}

TEST(ComponentBaseTest, SingleHotStartIsDegraded) {
  SharedNavMem nav{};
  SharedNavMemHandler comp(ComponentID::Navigation, Us(20000), nav);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::Navigation, Us(20000));

  comp.recordHotStart(Clock::now());

  ComponentBase<0> base(makeConfig(), comp, sysState);

  auto health = sysState.getHealth(ComponentID::Navigation);
  ASSERT_TRUE(health.has_value());
  EXPECT_EQ(*health, ComponentHealth::DEGRADED);
}

TEST(ComponentBaseTest, TooManyHotStartsEscalatesToSick) {
  SharedNavMem nav{};
  SharedNavMemHandler comp(ComponentID::Navigation, Us(20000), nav);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::Navigation, Us(20000));

  // max_hot_start=3 : il faut que le 3e plus récent (index 2) existe et
  // soit dans la fenêtre pour déclencher l'escalade cold-start.
  comp.recordHotStart(Clock::now());
  comp.recordHotStart(Clock::now());
  comp.recordHotStart(Clock::now());

  ComponentBase<0> base(makeConfig(), comp, sysState);

  auto health = sysState.getHealth(ComponentID::Navigation);
  ASSERT_TRUE(health.has_value());
  EXPECT_EQ(*health, ComponentHealth::SICK);
}

TEST(ComponentBaseTest, RecentColdStartIsDead) {
  SharedNavMem nav{};
  SharedNavMemHandler comp(ComponentID::Navigation, Us(20000), nav);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::Navigation, Us(20000));

  comp.recordColdStart(Clock::now());

  ComponentBase<0> base(makeConfig(), comp, sysState);

  auto health = sysState.getHealth(ComponentID::Navigation);
  ASSERT_TRUE(health.has_value());
  EXPECT_EQ(*health, ComponentHealth::DEAD);
}

// ─── Exécution réelle (nécessite SCHED_FIFO — skip sinon) ──────────────────
//
// Contrairement aux tests ci-dessus (loop()/heartbeat() pilotés à la main),
// ceux-ci laissent tourner de vrais threads RT et observent le comportement
// sur des fenêtres de temps réelles. Bornés par construction (aucun
// pthread_join sur un thread qui ne rendrait jamais la main) pour ne
// jamais accrocher la suite.

TEST(TaskRealExecutionTest, LoopRunsAtConfiguredFrequency) {
  if (!rtSchedulingAvailable())
    GTEST_SKIP() << "SCHED_FIFO indisponible ici (pas de privilège RT)";

  ComponentWatchdog wd(0, 0);
  TaskConfig cfg{
      .id = 1,
      .RTpriority = 1,
      .core = 0,
      .loopFrequency = Hz(100.0f), // période 10ms
      .timeout = Ms(2000),         // large : le watchdog ne doit pas intervenir
      .maxRestart = 3,
      .restartWindow = Ms(60000),
  };
  TestTask task(cfg, wd);
  task.start();

  std::this_thread::sleep_for(std::chrono::milliseconds(105));
  task.stop();

  // ~10 itérations attendues (105ms / 10ms) ; tolérance large pour
  // absorber le jitter de scheduling.
  int count = task.loopCount.load();
  EXPECT_GE(count, 5);
  EXPECT_LE(count, 15);
}

TEST(ComponentWatchdogRealThreadTest, DetectsAndRestartsATrulyDeadTask) {
  if (!rtSchedulingAvailable())
    GTEST_SKIP() << "SCHED_FIFO indisponible ici (pas de privilège RT)";

  ComponentWatchdog wd(1, 0); // thread de fond réel, poll ~100Hz
  std::atomic<int> startCount{0};

  wd.registerTask(
      1, [&] { startCount.fetch_add(1); }, [] {}, Ms(30),
      /*maxRestart=*/5, Ms(60000));

  // On ne heartbeat jamais et on n'appelle jamais loop() nous-même : c'est
  // le vrai thread de fond qui doit détecter le timeout et redémarrer.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  EXPECT_GE(startCount.load(), 2);
}

TEST(ComponentWatchdogRealThreadTest, RespectsHeartbeatFromAnotherThread) {
  if (!rtSchedulingAvailable())
    GTEST_SKIP() << "SCHED_FIFO indisponible ici (pas de privilège RT)";

  ComponentWatchdog wd(1, 0);
  std::atomic<int> startCount{0};
  std::atomic<bool> stop{false};

  wd.registerTask(
      1, [&] { startCount.fetch_add(1); }, [] {}, Ms(30),
      /*maxRestart=*/5, Ms(60000));

  std::thread heartbeatThread([&] {
    while (!stop.load(std::memory_order_relaxed)) {
      wd.heartbeat(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  stop.store(true, std::memory_order_relaxed);
  heartbeatThread.join();

  EXPECT_EQ(startCount.load(), 0);
}

class StallOnceTask : public Task {
public:
  using Task::Task;
  std::atomic<int> loopCount{0};

protected:
  void loop() override {
    int n = loopCount.fetch_add(1, std::memory_order_relaxed);
    if (n == 2) {
      // Blocage transitoire (borné) assez long pour dépasser le timeout du
      // watchdog — simule un stall récupérable, pas un hang infini.
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
  }
};

TEST(TaskRealExecutionTest, WatchdogRestartsATransientlyStalledTask) {
  if (!rtSchedulingAvailable())
    GTEST_SKIP() << "SCHED_FIFO indisponible ici (pas de privilège RT)";

  ComponentWatchdog wd(1, 0);
  TaskConfig cfg{
      .id = 1,
      .RTpriority = 1,
      .core = 0,
      .loopFrequency = Hz(200.0f), // période 5ms
      .timeout = Ms(50),
      .maxRestart = 5,
      .restartWindow = Ms(60000),
  };
  StallOnceTask task(cfg, wd);
  task.start();
  ASSERT_EQ(task.generation(), 1u);

  // Assez long pour couvrir : itérations normales + stall (150ms) +
  // détection/restart + reprise.
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  task.stop();

  EXPECT_GT(task.loopCount.load(), 5); // la Task a repris et continue

  // generation() prouve une vraie intervention du watchdog (rappel de
  // start()) — loopCount seul ne suffit pas : un stall qui finit par
  // revenir (loop() retourne) fait continuer le MÊME thread sans aucune
  // intervention, donc loopCount grimperait pareil dans les deux cas.
  EXPECT_GT(task.generation(), 1u);
}
