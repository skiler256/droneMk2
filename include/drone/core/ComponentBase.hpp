#pragma once
#include "drone/Components/SharedCompMem.hpp"
#include "drone/core/SharedSysStateMem.hpp"
#include "drone/core/watchdog.hpp"
#include "drone/types.hpp"
#include <cstddef>
#include <ctime>
#include <expected>

#define RT_PRIORITY_COMPONENT_WATCHDOG 80

struct TaskConfig {
  TYPES::TaskID id;
  int RTpriority;
  TYPES::Hz loopFrequency;
  TYPES::Ms timeout;
};

class Task {
public:
  explicit Task(TaskConfig config, ComponentWatchdog &WD)
      : Tconfig(config), TlocalWD(WD) {
    WD.registerTask(
        Tconfig.id, [this] { start(); }, [this] { stop(); }, Tconfig.timeout);
  };

  void start() {
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    struct sched_param param;
    param.sched_priority = Tconfig.RTpriority;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    running_.store(true, std::memory_order_release);
    pthread_create(&thread_, &attr, &Task::threadEntry, this);
    pthread_attr_destroy(&attr);
  }

  void stop() {
    running_.store(false, std::memory_order_release);
    pthread_join(thread_, nullptr);
  }

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

    // Conversion sécurisée Hz -> période
    auto period =
        TYPES::Ms(static_cast<int64_t>(1'000.0f / Tconfig.loopFrequency.v));

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
};

struct ComponenConfig {
  size_t max_cold_start = 1; // si le composant dépasse la limite de cold start,
                             // il sera considéré comme "fou"
  TYPES::Ms max_cold_start_interval{60000};
  size_t max_hot_start = 3; // limite de crash a partir de laquelle on considère
                            // la mémoire du composant comme "malade"
  TYPES::Ms max_hot_start_interval{60000};
  TYPES::Us shm_timeout{2000};

  TYPES::ComponentID id;
};

class ComponentBase {
public:
  explicit ComponentBase(ComponenConfig config, SharedCompMemHandler &compMem,
                         SharedSysStateMemHandler &sysState)
      : localWD(RT_PRIORITY_COMPONENT_WATCHDOG), config_(config),
        compMem_(compMem), sysState_(sysState) {
    auto lastCold = compMem_.getColdtStartTs(config_.max_cold_start - 1);
    auto lastHot = compMem_.getHotStartTs(config_.max_hot_start - 1);

    // Trop de cold-starts dans la fenêtre → DEAD
    if (lastCold.has_value()) {
      if (UTILITIES::msBetween(lastCold.value(), TYPES::Clock::now()) <
          config_.max_cold_start_interval) {
        sysState_.setHealth(config_.id, TYPES::ComponentHealth::DEAD);
        return;
      }
    }

    // Trop de hot-starts dans la fenêtre → cold-start
    if (lastHot.has_value()) {
      if (UTILITIES::msBetween(lastHot.value(), TYPES::Clock::now()) <
          config_.max_hot_start_interval) {
        compMem_.recordColdStart(TYPES::Clock::now());
        sysState_.setHealth(config_.id, TYPES::ComponentHealth::SICK);
        // init();
        coldstart = true;
        return;
      }
    }

    // Hot-start si au moins un redémarrage existe
    if (compMem_.getHotStartTs(0).has_value()) {
      compMem_.recordHotStart(TYPES::Clock::now());
      sysState_.setHealth(config_.id, TYPES::ComponentHealth::DEGRADED);
      // if (!restore()) {
      //     compMem_.recordColdStart(TYPES::Clock::now());
      //     sysState_.setHealth(config_.id, TYPES::ComponentHealth::SICK);
      //     init();
      // }
      hotstart = true;
      return;
    }

    // Premier démarrage nominal
    sysState_.setHealth(config_.id, TYPES::ComponentHealth::NOMINAL);
    // init();
    coldstart = true;
  };

protected:
  ComponentWatchdog localWD;

  bool hotstart{false};
  bool coldstart{false}; // séquence de démarage du composant
  //  [[nodiscard]]virtual bool restore(); // récupération des données du
  //  composant sur le segment shm, sert au hot_start

private:
  ComponenConfig config_;
  SharedCompMemHandler &compMem_;
  SharedSysStateMemHandler &sysState_; // Les handler shm, sont stocké loclament
};
