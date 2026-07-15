#pragma once
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <functional>
#include <vector>

class ComponentWatchdog {
public:
  explicit ComponentWatchdog(int RTpriority, int core)
      : rt_priority_(RTpriority), core_(core) {
    auto result =
        UTILITIES::launchRTThread([this] { run(); }, rt_priority_, core_);

    if (result)
      thread_ = result.value();
  }

  struct TaskHandle {
    std::function<void()> stop;
    std::function<void()> start;
    TYPES::TimePoint lastHeartbeat;
    TYPES::Ms timeout;
    TYPES::TaskID id;
  };

  void registerTask(TYPES::TaskID id, std::function<void()> start,
                    std::function<void()> stop, TYPES::Ms timeout) {
    tasks_.push_back({
        .stop = std::move(stop),
        .start = std::move(start),
        .lastHeartbeat = TYPES::Clock::now(),
        .timeout = timeout,
        .id = id,
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

  void stop() {
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
    t.stop();

    // TODO : ring buffer restart Task
    // Si seuil dépassé → shutdown()

    t.start();
    t.lastHeartbeat = TYPES::Clock::now();
  }

  void shutdown() {
    for (auto it = tasks_.rbegin(); it != tasks_.rend(); ++it)
      it->stop();
    _exit(1);
  }

  std::vector<TaskHandle> tasks_;
  pthread_t thread_{};
  std::atomic<bool> running_{true};
  int rt_priority_;
  int core_;
};