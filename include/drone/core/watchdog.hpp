#pragma once
#include "drone/core/SharedMemory.hpp"
#include "drone/core/SharedSysStateMem.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <atomic>
#include <deque>
#include <expected>
#include <functional>
#include <vector>

class ComponentWatchdog {
public:
  explicit ComponentWatchdog(int RTpriority, int core)
      : rt_priority_(RTpriority), core_(core) {}

  // Lancé hors du constructeur : sinon le thread itérerait tasks_ pendant que
  // les Task s'y enregistrent, et l'échec ne serait pas remontable.
  // cf. ARCHITECTURE_IPC.md §4.1.
  [[nodiscard]] std::expected<void, int> start() {
    auto result =
        UTILITIES::launchRTThread([this] { run(); }, rt_priority_, core_);
    if (!result)
      return std::unexpected(result.error());

    thread_ = result.value();
    started_ = true;
    return {};
  }

  ~ComponentWatchdog() { stop(); } // RAII, cf. ARCHITECTURE_IPC.md §3

  struct TaskHandle {
    std::function<void()> stop;
    std::function<void()> start;
    TYPES::TimePoint lastHeartbeat;
    TYPES::Ms timeout;
    TYPES::TaskID id;

    // Fenêtre glissante, cf. ARCHITECTURE_IPC.md §4.1.
    size_t maxRestart;
    TYPES::Ms restartWindow;
    std::deque<TYPES::TimePoint> restartHistory;

    // Mesure de fréquence : la Task ne fait qu'incrémenter, toute
    // l'arithmétique vit ici (mono-thread). cf. SYSMONITORING.md §13.
    std::function<uint32_t()> loopCount;
    float targetHz{0.0f};
    bool targetPublished{false};
    uint32_t lastCount{0};
    TYPES::TimePoint lastRateSample{};
  };

  void registerTask(TYPES::TaskID id, std::function<void()> start,
                    std::function<void()> stop,
                    std::function<uint32_t()> loopCount, float targetHz,
                    TYPES::Ms timeout, size_t maxRestart,
                    TYPES::Ms restartWindow) {
    tasks_.push_back({
        .stop = std::move(stop),
        .start = std::move(start),
        .lastHeartbeat = TYPES::Clock::now(),
        .timeout = timeout,
        .id = id,
        .maxRestart = maxRestart,
        .restartWindow = restartWindow,
        .restartHistory = {},
        .loopCount = std::move(loopCount),
        .targetHz = targetHz,
        .lastRateSample = TYPES::Clock::now(),
    });
  }

  // atomic_ref et non un membre atomic : TaskHandle doit rester copiable pour
  // vivre dans un std::vector.
  void heartbeat(TYPES::TaskID id) {
    for (auto &t : tasks_) {
      if (t.id == id) {
        std::atomic_ref<TYPES::TimePoint>(t.lastHeartbeat)
            .store(TYPES::Clock::now(), std::memory_order_release);
        return;
      }
    }
  }

  // Destinataire des fréquences mesurées, posé par ComponentBase.
  void setRatePublisher(SharedSysStateMemHandler &h) { ratePublisher_ = &h; }

  // Erreur sticky consommée par loop(), cf. ARCHITECTURE_IPC.md §4.1.
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
      auto last = std::atomic_ref<TYPES::TimePoint>(t.lastHeartbeat)
                      .load(std::memory_order_acquire);
      if (UTILITIES::msBetween(last, now) > t.timeout)
        handleDeadTask(t);
    }

    publishRates(now);

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

  // Fenêtre d'une seconde : assez longue pour une moyenne stable, assez
  // courte pour voir un décrochage.
  void publishRates(TYPES::TimePoint now) {
    if (ratePublisher_ == nullptr)
      return;
    for (auto &t : tasks_) {
      if (!t.loopCount)
        continue;
      auto elapsed = UTILITIES::msBetween(t.lastRateSample, now);
      if (elapsed < TYPES::Ms{1000})
        continue;
      if (!t.targetPublished) {
        ratePublisher_->setTaskTargetHz(t.id, t.targetHz);
        t.targetPublished = true;
      }

      const uint32_t count = t.loopCount();
      const auto delta = static_cast<float>(count - t.lastCount);
      ratePublisher_->setTaskHz(
          t.id, delta * 1000.0f / static_cast<float>(elapsed.count()));
      t.lastCount = count;
      t.lastRateSample = now;
    }
  }

  void handleDeadTask(TaskHandle &t) {
    auto now = TYPES::Clock::now();

    while (!t.restartHistory.empty() &&
           UTILITIES::msBetween(t.restartHistory.front(), now) >
               t.restartWindow)
      t.restartHistory.pop_front();
    t.restartHistory.push_back(now);

    if (t.restartHistory.size() >= t.maxRestart) {
      // Escalade -> process entier, cf. ARCHITECTURE_IPC.md §4.1.
      shutdown();
      return;
    }

    // stop() ici uniquement dans le cas restart simple : shutdown() stoppe
    // déjà toutes les tasks lui-même (double pthread_join serait UB).
    t.stop();
    // Échec de relance non remonté ici : l'absence de heartbeat ramènera la
    // Task dans handleDeadTask, jusqu'à l'escalade vers le process entier.
    t.start();
    std::atomic_ref<TYPES::TimePoint>(t.lastHeartbeat)
        .store(now, std::memory_order_release);
  }

  void shutdown() {
    for (auto it = tasks_.rbegin(); it != tasks_.rend(); ++it)
      it->stop();
    _exit(1);
  }

  std::vector<TaskHandle> tasks_;
  std::vector<SharedMemoryHandler *> shmSources_;
  SharedSysStateMemHandler *ratePublisher_{nullptr};
  std::atomic<TYPES::shmError> lastShmError_{TYPES::shmError::None};
  std::atomic<uint32_t> shmErrorCount_{0};
  pthread_t thread_{};
  std::atomic<bool> running_{true};
  bool started_{false};
  std::atomic<bool> stopped_{false};
  int rt_priority_;
  int core_;
};