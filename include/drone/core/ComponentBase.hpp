#pragma once
#include "drone/Components/SharedCompMem.hpp"
#include "drone/core/SharedSysStateMem.hpp"
#include "drone/core/watchdog.hpp"
#include "drone/types.hpp"
#include <array>
#include <cstddef>
#include <ctime>
#include <expected>

#define RT_PRIORITY_COMPONENT_WATCHDOG 80
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
        Tconfig.id, [this] { start(); }, [this] { stop(); }, Tconfig.timeout,
        Tconfig.maxRestart, Tconfig.restartWindow);
  };

  // Preuve externe de restart, PAS pthread_self() -- cf. ARCHITECTURE_IPC.md §3.
  [[nodiscard]] size_t generation() const noexcept {
    return generation_.load(std::memory_order_acquire);
  }

  void start() {
    generation_.fetch_add(1, std::memory_order_acq_rel);
    running_.store(true, std::memory_order_release);

    auto result =
        UTILITIES::launchRTThread([this] { run(); }, Tconfig.RTpriority,
                                  Tconfig.core // à ajouter dans TaskConfig
        );

    if (!result) {
      // errno dans result.error()
      running_.store(false, std::memory_order_release);
    } else {
      thread_ = result.value();
      threadValid_ = true;
    }
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
      TlocalWD.heartbeat(Tconfig.id);

      next_time += period;
      std::this_thread::sleep_until(next_time);
    }
  }

  pthread_t thread_{};
  std::atomic<bool> running_{false};
  bool threadValid_{false};
  std::atomic<size_t> generation_{0};
};

// Décision hot/cold/DEAD à la construction : cf. ARCHITECTURE_IPC.md §4.3.
struct ComponenConfig {
  size_t max_cold_start = 1;
  TYPES::Ms max_cold_start_interval{60000};
  size_t max_hot_start = 3;
  TYPES::Ms max_hot_start_interval{60000};
  TYPES::Us shm_timeout{2000};

  TYPES::ComponentID id;
  int CompCore;
};

// MaxTasks : nombre de Task du composant dérivé, connu à la compilation
// (varie d'un composant à l'autre — Navigation en a 1, un futur composant
// plus riche pourra en avoir plus).
template <size_t MaxTasks>
class ComponentBase {
public:
  explicit ComponentBase(ComponenConfig config, SharedCompMemHandler &compMem,
                         SharedSysStateMemHandler &sysState)
      : localWD(RT_PRIORITY_COMPONENT_WATCHDOG, CORE_COMPONENT_WATCHDOG),
        config_(config), compMem_(compMem), sysState_(sysState) {
    localWD.registerShmSource(compMem_);
    localWD.registerShmSource(sysState_);

    auto lastCold = compMem_.getColdtStartTs(config_.max_cold_start - 1);
    auto lastHot = compMem_.getHotStartTs(config_.max_hot_start - 1);

    // has_value() seul ne suffit pas ici, cf. piège documenté ARCHITECTURE_IPC.md §4.3.
    auto isRecorded = [](const std::optional<TYPES::TimePoint> &ts) {
      return ts.has_value() && ts.value() != TYPES::TimePoint{};
    };

    // Trop de cold-starts dans la fenêtre → DEAD
    if (isRecorded(lastCold)) {
      if (UTILITIES::msBetween(lastCold.value(), TYPES::Clock::now()) <
          config_.max_cold_start_interval) {
        sysState_.setHealth(TYPES::ComponentHealth::DEAD);
        return;
      }
    }

    // Trop de hot-starts dans la fenêtre → cold-start
    if (isRecorded(lastHot)) {
      if (UTILITIES::msBetween(lastHot.value(), TYPES::Clock::now()) <
          config_.max_hot_start_interval) {
        compMem_.recordColdStart(TYPES::Clock::now());
        sysState_.setHealth(TYPES::ComponentHealth::SICK);
        coldstart = true;
        return;
      }
    }

    // Hot-start si au moins un redémarrage existe
    if (isRecorded(compMem_.getHotStartTs(0))) {
      compMem_.recordHotStart(TYPES::Clock::now());
      sysState_.setHealth(TYPES::ComponentHealth::DEGRADED);
      hotstart = true;
      return;
    }

    // Premier démarrage nominal
    sysState_.setHealth(TYPES::ComponentHealth::NOMINAL);
    coldstart = true;
  };

  virtual ~ComponentBase() = default;

protected:
  ComponentWatchdog localWD;

  bool hotstart{false};
  bool coldstart{false}; // séquence de démarrage du composant

  // registerTask() puis activate(), en fin de ctor du composant dérivé --
  // pourquoi ComponentBase ne peut pas le faire lui-même : ARCHITECTURE_IPC.md §4.3.
  void registerTask(Task &t) {
    if (taskCount_ < tasks_.size())
      tasks_[taskCount_++] = &t;
  }

  void activate() {
    if (!hotstart && !coldstart)
      return; // DEAD : rien à démarrer
    if (hotstart)
      restore();
    init();
  }

  virtual void restore() {} // no-op par défaut, override si état à réhydrater
  virtual void init() {     // défaut : démarre toutes les tasks enregistrées
    for (size_t i = 0; i < taskCount_; ++i)
      tasks_[i]->start();
  }

private:
  ComponenConfig config_;
  SharedCompMemHandler &compMem_;
  SharedSysStateMemHandler &sysState_;

  std::array<Task *, MaxTasks> tasks_{};
  size_t taskCount_ = 0;
};
