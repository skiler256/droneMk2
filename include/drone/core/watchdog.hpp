#pragma once
#include "drone/core/SharedMemory.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <deque>
#include <functional>
#include <vector>

class ComponentWatchdog {
public:
  explicit ComponentWatchdog(int RTpriority, int core)
      : rt_priority_(RTpriority), core_(core) {
    auto result =
        UTILITIES::launchRTThread([this] { run(); }, rt_priority_, core_);

    if (result) {
      thread_ = result.value();
      started_ = true;
    }
  }

  // Évite qu'un thread RT survive à la destruction de l'objet (this
  // pendouillant, capturé par [this]{run();}).
  ~ComponentWatchdog() { stop(); }

  struct TaskHandle {
    std::function<void()> stop;
    std::function<void()> start;
    TYPES::TimePoint lastHeartbeat;
    TYPES::Ms timeout;
    TYPES::TaskID id;

    // Ring buffer d'horodatages, pas un compteur absolu.
    size_t maxRestart;
    TYPES::Ms restartWindow;
    std::deque<TYPES::TimePoint> restartHistory;
  };

  void registerTask(TYPES::TaskID id, std::function<void()> start,
                    std::function<void()> stop, TYPES::Ms timeout,
                    size_t maxRestart, TYPES::Ms restartWindow) {
    tasks_.push_back({
        .stop = std::move(stop),
        .start = std::move(start),
        .lastHeartbeat = TYPES::Clock::now(),
        .timeout = timeout,
        .id = id,
        .maxRestart = maxRestart,
        .restartWindow = restartWindow,
        .restartHistory = {},
    });
  }

  void heartbeat(TYPES::TaskID id) {
    for (auto &t : tasks_) {
      if (t.id == id) {
        t.lastHeartbeat = TYPES::Clock::now();
        return;
      }
    }
  }

  // loop() consomme la dernière erreur de chaque source à fréquence fixe,
  // pour ne pas en rater une écrasée entre deux accès du composant.
  void registerShmSource(SharedMemoryHandler &handler) {
    shmSources_.push_back(&handler);
  }

  [[nodiscard]] TYPES::shmError lastShmError() const noexcept {
    return lastShmError_.load(std::memory_order_acquire);
  }

  [[nodiscard]] uint32_t shmErrorCount() const noexcept {
    return shmErrorCount_.load(std::memory_order_acquire);
  }

  // Idempotent : safe depuis le destructeur ET un appel explicite.
  void stop() {
    if (!started_ || stopped_.exchange(true, std::memory_order_acq_rel))
      return;
    running_.store(false, std::memory_order_release);
    pthread_join(thread_, nullptr);
  }

  void loop() {
    auto now = TYPES::Clock::now();

    for (auto &t : tasks_) {
      TYPES::Ms elapsed = UTILITIES::msBetween(t.lastHeartbeat, now);
      if (elapsed > t.timeout)
        handleDeadTask(t);
    }

    for (auto *src : shmSources_) {
      auto err = src->consumeError();
      if (err != TYPES::shmError::None) {
        lastShmError_.store(err, std::memory_order_release);
        shmErrorCount_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

private:
  void run() {
    using namespace std::chrono_literals;

    auto next = TYPES::Clock::now();
    const auto period = std::chrono::milliseconds(10); // 100Hz max

    while (running_.load(std::memory_order_acquire)) {
      loop();
      next += period;
      std::this_thread::sleep_until(next);
    }
  }

  void handleDeadTask(TaskHandle &t) {
    auto now = TYPES::Clock::now();

    // Purge la fenêtre glissante puis enregistre ce redémarrage.
    while (!t.restartHistory.empty() &&
           UTILITIES::msBetween(t.restartHistory.front(), now) >
               t.restartWindow)
      t.restartHistory.pop_front();
    t.restartHistory.push_back(now);

    if (t.restartHistory.size() >= t.maxRestart) {
      // Trop de redémarrages : on tue le process entier plutôt que de
      // continuer à relancer la Task seule. GlobalWatchdog respawn, et
      // ComponentBase décide hot-start/cold-start/DEAD.
      shutdown();
      return;
    }

    // stop() ici uniquement dans le cas restart simple : shutdown() stoppe
    // déjà toutes les tasks lui-même (double pthread_join serait UB).
    t.stop();
    t.start();
    t.lastHeartbeat = now;
  }

  void shutdown() {
    for (auto it = tasks_.rbegin(); it != tasks_.rend(); ++it)
      it->stop();
    _exit(1);
  }

  std::vector<TaskHandle> tasks_;
  std::vector<SharedMemoryHandler *> shmSources_;
  std::atomic<TYPES::shmError> lastShmError_{TYPES::shmError::None};
  std::atomic<uint32_t> shmErrorCount_{0};
  pthread_t thread_{};
  std::atomic<bool> running_{true};
  bool started_{false};
  std::atomic<bool> stopped_{false};
  int rt_priority_;
  int core_;
};