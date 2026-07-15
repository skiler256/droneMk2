#pragma once
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <functional>
#include <vector>

class ComponentWatchdog {
public:

    explicit ComponentWatchdog(int RTpriority):rt_priority_(RTpriority){
         pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

        struct sched_param param;
        param.sched_priority = rt_priority_;
        pthread_attr_setschedparam(&attr, &param);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

        running_.store(true, std::memory_order_release);
        pthread_create(&thread_, &attr, &ComponentWatchdog::threadEntry, this);
        pthread_attr_destroy(&attr);
    };

    struct TaskHandle {
        std::function<void()>    stop;
        std::function<void()>    start;
        TYPES::TimePoint lastHeartbeat;
        TYPES::Ms                timeout;
        TYPES::TaskID            id;
        
    };

    void registerTask(TYPES::TaskID        id,
                      std::function<void()> start,
                      std::function<void()> stop,
                      TYPES::Ms             timeout)
    {
        tasks_.push_back({
            .stop          = std::move(stop),
            .start         = std::move(start),
            .lastHeartbeat = TYPES::Clock::now(),
            .timeout       = timeout,
            .id            = id,
        });
    }

    void heartbeat(TYPES::TaskID id) {
        for (auto& t : tasks_) {
            if (t.id == id) {
                t.lastHeartbeat=TYPES::Clock::now();
                return;
            }
        }
    }

    void loop() {
        auto now = TYPES::Clock::now();

        for (auto& t : tasks_) {
            TYPES::TimePoint last = t.lastHeartbeat;
            TYPES::Ms elapsed = UTILITIES::msBetween(last, now);

            if (elapsed > t.timeout) {
                handleDeadTask(t);
            }
        }
    }

private:

static void* threadEntry(void* self) {
        static_cast<ComponentWatchdog*>(self)->run();
        return nullptr;
    };

        void run() {
        while (running_.load(std::memory_order_acquire)) {
            loop();
        }
    };

    void handleDeadTask(TaskHandle& t) {
        // Arrêt de la task fautive
        t.stop();

        // TODO : ring buffer restart Task — même logique que ComponentBase
        // Si seuil dépassé → arrêt propre de tout le composant

        t.start();
        t.lastHeartbeat=TYPES::Clock::now();
                              
    }

    void shutdown() {
        // Arrêt propre de toutes les Tasks dans l'ordre inverse
        for (auto it = tasks_.rbegin(); it != tasks_.rend(); ++it) {
            it->stop();
        }
        _exit(1);
    }

    std::vector<TaskHandle> tasks_;
    pthread_t          thread_     {};
    std::atomic<bool>  running_    {false};
    int                rt_priority_;
};