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

  // Si la Task redémarre maxRestart fois dans restartWindow, le watchdog
  // local tue le process entier (exit(1)) au lieu de la relancer sans fin.
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

  // Incrémenté à chaque appel : preuve externe qu'un restart a eu lieu
  // (pthread_self() n'est pas fiable pour ça, glibc recycle les tid).
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

  // Idempotent : no-op si pas de thread valide (déjà stoppé, ou jamais démarré).
  void stop() {
    running_.store(false, std::memory_order_release);
    if (threadValid_) {
      pthread_join(thread_, nullptr);
      threadValid_ = false;
    }
  }

  // Évite qu'un thread RT survive à la destruction de l'objet.
  virtual ~Task() { stop(); }

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

struct ComponenConfig {
  size_t max_cold_start = 1; // si le composant dépasse la limite de cold start,
                             // il sera considéré comme "fou"
  TYPES::Ms max_cold_start_interval{60000};
  size_t max_hot_start = 3; // limite de crash a partir de laquelle on considère
                            // la mémoire du composant comme "malade"
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

    // has_value() seul ne suffit pas : le tableau a une taille fixe, donc
    // un slot jamais écrit renvoie quand même un optional valide (à
    // l'epoch). Sans ce check, un composant neuf serait vu comme DEGRADED.
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

  // A appeler pour chaque Task membre, dans le ctor du composant dérivé,
  // avant activate().
  void registerTask(Task &t) {
    if (taskCount_ < tasks_.size())
      tasks_[taskCount_++] = &t;
    
  }

  // A appeler UNE fois, en fin de ctor du composant dérivé (une fois ses
  // Tasks enregistrées) — ComponentBase ne peut pas le faire lui-même,
  // ses membres se construisent avant ceux de la classe dérivée.
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
