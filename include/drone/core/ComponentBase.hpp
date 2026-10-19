#pragma once
#include "drone/Components/SharedCompMem.hpp"
#include "drone/core/SharedSysStateMem.hpp"
#include "drone/core/watchdog.hpp"
#include "drone/types.hpp"
#include <array>
#include <cstddef>
#include <ctime>
#include <expected>
#include <optional>

// 90 : strictement au-dessus de la Task la plus prioritaire du système
// (TSafetyMonitor, 85). Sans cette marge, un watchdog partageant un cœur avec
// la Task qu'il surveille pourrait en être affamé. cf. ARCHITECTURE_IPC.md §4.1.
#define RT_PRIORITY_COMPONENT_WATCHDOG 90
// Cœur dédié, volontairement distinct de ComponenConfig::CompCore (qui épingle
// les Task du composant).
#define CORE_COMPONENT_WATCHDOG 3

struct TaskConfig {
  TYPES::TaskID id;
  int RTpriority;
  int core;
  TYPES::Hz loopFrequency;
  TYPES::Ms timeout;

  // Escalade Task -> process entier si dépassé, cf. ARCHITECTURE_IPC.md §4.1.
  size_t maxRestart = 3;
  TYPES::Ms restartWindow{60000};
};

class Task {
public:
  explicit Task(TaskConfig config, ComponentWatchdog &WD)
      : Tconfig(config), TlocalWD(WD) {
    WD.registerTask(
        Tconfig.id, [this] { (void)start(); }, [this] { stop(); },
        [this] { return loopCount_.load(std::memory_order_relaxed); },
        Tconfig.loopFrequency.v, Tconfig.timeout, Tconfig.maxRestart, Tconfig.restartWindow);
  };

  // Preuve externe de restart, PAS pthread_self() -- cf. ARCHITECTURE_IPC.md §3.
  [[nodiscard]] size_t generation() const noexcept {
    return generation_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::expected<void, int> start() {
    generation_.fetch_add(1, std::memory_order_acq_rel);
    running_.store(true, std::memory_order_release);

    auto result = UTILITIES::launchRTThread(
        [this] { run(); }, Tconfig.RTpriority, Tconfig.core);

    if (!result) {
      running_.store(false, std::memory_order_release);
      return std::unexpected(result.error());
    }
    thread_ = result.value();
    threadValid_ = true;
    return {};
  }

  // Idempotent (cf. ARCHITECTURE_IPC.md §3).
  void stop() {
    running_.store(false, std::memory_order_release);
    if (threadValid_) {
      pthread_join(thread_, nullptr);
      threadValid_ = false;
    }
  }

  virtual ~Task() { stop(); } // RAII, cf. ARCHITECTURE_IPC.md §3

protected:
  virtual void loop() = 0;
  TaskConfig Tconfig;
  ComponentWatchdog &TlocalWD;

private:
  static void *threadEntry(void *self) {
    static_cast<Task *>(self)->run();
    return nullptr;
  }

  void run() {
    using namespace std::chrono_literals;

    auto period =
        TYPES::Us(static_cast<int64_t>(1'000'000.0f / Tconfig.loopFrequency.v));

    auto next_time = TYPES::Clock::now();

    while (running_.load(std::memory_order_acquire)) {
      loop();
      loopCount_.fetch_add(1, std::memory_order_relaxed);
      TlocalWD.heartbeat(Tconfig.id);

      next_time += period;
      std::this_thread::sleep_until(next_time);
    }
  }

  pthread_t thread_{};
  std::atomic<bool> running_{false};
  bool threadValid_{false};
  std::atomic<size_t> generation_{0};
  std::atomic<uint32_t> loopCount_{0};
};

// Décision hot/cold/DEAD à la construction : cf. ARCHITECTURE_IPC.md §4.3.
struct ComponenConfig {
  size_t max_cold_start = 1;
  TYPES::Ms max_cold_start_interval{60000};
  size_t max_hot_start = 3;
  TYPES::Ms max_hot_start_interval{60000};

  TYPES::ComponentID id;
  int CompCore;

  // Transmis par GlobalWatchdog, jamais déduit — cf. ARCHITECTURE_IPC.md §4.3.
  bool firstBoot = true;
};

template <size_t MaxTasks>
class ComponentBase {
public:
  explicit ComponentBase(ComponenConfig config, SharedCompMemHandler &compMem,
                         SharedSysStateMemHandler &sysState)
      : localWD(RT_PRIORITY_COMPONENT_WATCHDOG, CORE_COMPONENT_WATCHDOG),
        config_(config), compMem_(compMem), sysState_(sysState) {
    localWD.registerShmSource(compMem_);
    localWD.registerShmSource(sysState_);
    localWD.setRatePublisher(sysState_);

    auto lastCold = compMem_.getColdStartTs(config_.max_cold_start - 1);
    auto lastHot = compMem_.getHotStartTs(config_.max_hot_start - 1);

    // Lecture shm échouée : on suppose le pire plutôt que de rater une
    // escalade — perdre un cran est plus grave qu'un diagnostic pessimiste.
    // cf. ARCHITECTURE_IPC.md §4.3.
    // MissionControl ne cold-start jamais : sa state machine ne peut pas être
    // resetée en sécurité (CLAUDE.md) — cf. ARCHITECTURE_IPC.md §4.3.
    const bool coldStartForbidden =
        config_.id == TYPES::ComponentID::MissionControl;
    auto escalateToColdStart = [&] {
      if (coldStartForbidden) {
        sysState_.setHealth(TYPES::ComponentHealth::DEAD);
        return;
      }
      compMem_.recordColdStart(TYPES::Clock::now());
      sysState_.setHealth(TYPES::ComponentHealth::SICK);
      coldstart = true;
    };

    auto readFailed = [](const auto &r) { return !r.has_value(); };
    if (readFailed(lastCold) || readFailed(lastHot)) {
      escalateToColdStart();
      return;
    }

    // Slot porteur d'un horodatage réel (la sentinelle est filtrée par le
    // handler, qui seul connaît la convention).
    auto isRecorded = [](const auto &r) { return r->has_value(); };

    if (isRecorded(lastCold)) {
      if (UTILITIES::msBetween(lastCold->value(), TYPES::Clock::now()) <
          config_.max_cold_start_interval) {
        sysState_.setHealth(TYPES::ComponentHealth::DEAD);
        return;
      }
    }

    if (isRecorded(lastHot)) {
      if (UTILITIES::msBetween(lastHot->value(), TYPES::Clock::now()) <
          config_.max_hot_start_interval) {
        escalateToColdStart();
        return;
      }
    }

    // Pas un premier démarrage = hot-start par définition (§4.3).
    if (!config_.firstBoot) {
      compMem_.recordHotStart(TYPES::Clock::now());
      sysState_.setHealth(TYPES::ComponentHealth::DEGRADED);
      hotstart = true;
      return;
    }

    sysState_.setHealth(TYPES::ComponentHealth::NOMINAL);
    coldstart = true;
  };

  virtual ~ComponentBase() = default;

  // errno du thread RT qui n'a pas démarré. Un constructeur ne peut pas
  // échouer, donc le rôle appelant doit tester ceci et sortir en erreur :
  // GlobalWatchdog verra l'enfant mourir et déclenchera l'escalade normale.
  // cf. AUDIT.md §5.
  [[nodiscard]] std::optional<int> startupError() const { return startupError_; }

protected:
  ComponentWatchdog localWD;

  bool hotstart{false};
  bool coldstart{false};

  // À appeler en fin de ctor du dérivé, cf. ARCHITECTURE_IPC.md §4.3.
  void registerTask(Task &t) {
    if (taskCount_ < tasks_.size())
      tasks_[taskCount_++] = &t;
  }

  // Démarre le watchdog AVANT les Task, pour qu'aucun heartbeat ne parte avant
  // qu'il y ait quelqu'un pour l'observer.
  void activate() {
    if (!hotstart && !coldstart)
      return; // DEAD : rien à démarrer

    if (auto r = localWD.start(); !r) {
      startupError_ = r.error();
      return;
    }
    if (hotstart)
      restore();
    if (auto r = init(); !r)
      startupError_ = r.error();
  }

  virtual void restore() {} // override si un état est à réhydrater
  virtual std::expected<void, int> init() {
    for (size_t i = 0; i < taskCount_; ++i)
      if (auto r = tasks_[i]->start(); !r)
        return r;
    return {};
  }

private:
  ComponenConfig config_;
  SharedCompMemHandler &compMem_;
  SharedSysStateMemHandler &sysState_;

  std::array<Task *, MaxTasks> tasks_{};
  size_t taskCount_ = 0;
  std::optional<int> startupError_{};
};
