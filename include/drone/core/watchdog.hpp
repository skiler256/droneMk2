#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// drone/core/watchdog.hpp
//
// Chaque bloc appelle beat() dans sa boucle principale.
// Le Watchdog surveille et relance les threads qui ne battent plus.
// ─────────────────────────────────────────────────────────────────────────────

#include "drone/interfaces.hpp"
#include <functional>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

using namespace TYPES;

namespace CORE {

class Watchdog final : public IWatchdogClient {
public:
    struct Entry {
        TimePoint              last_beat;
        Ms                     timeout;
        std::function<void()>  restart_fn;
        int                    miss_count  = 0;
    };

    static constexpr int MAX_RESTARTS = 3;

    // ── Enregistrement (avant start()) ───────────────────────────────────
    void registerComponent(std::string         name,
                           Ms                  timeout,
                           std::function<void()> restart_fn) {
        std::unique_lock lock(mutex_);
        entries_[name] = Entry{
            .last_beat  = Clock::now(),
            .timeout    = timeout,
            .restart_fn = std::move(restart_fn),
        };
    }

    // ── IWatchdogClient ───────────────────────────────────────────────────
    void beat(std::string_view name) noexcept override {
        std::unique_lock lock(mutex_);
        auto it = entries_.find(std::string(name));
        if (it != entries_.end()) {
            it->second.last_beat  = Clock::now();
            it->second.miss_count = 0;
        }
    }

    // ── Cycle de vie ──────────────────────────────────────────────────────
    void start() {
        running_ = true;
        thread_  = std::thread(&Watchdog::run, this);

        // Le watchdog tourne à la priorité la plus haute
        sched_param p{ .sched_priority = 95 };
        pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &p);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        while (running_) {
            std::this_thread::sleep_for(Ms{100});

            auto now = Clock::now();
            std::unique_lock lock(mutex_);

            for (auto& [name, entry] : entries_) {
                if (now - entry.last_beat <= entry.timeout) continue;

                entry.miss_count++;
                std::cerr << "[watchdog] " << name
                          << " mort (miss " << entry.miss_count << "/"
                          << MAX_RESTARTS << ")\n";

                if (entry.miss_count <= MAX_RESTARTS) {
                    std::cerr << "[watchdog] Redémarrage de " << name << "\n";
                    entry.last_beat = Clock::now(); // évite redémarrage en boucle
                    entry.restart_fn();
                } else {
                    std::cerr << "[watchdog] CRITIQUE : " << name
                              << " en échec permanent — failsafe actif\n";
                    // ArduPilot prend la main via perte de heartbeat MAVLink
                }
            }
        }
    }

    mutable std::mutex                       mutex_;
    std::unordered_map<std::string, Entry>   entries_;
    std::thread                              thread_;
    std::atomic_bool                         running_{false};
};

} // namespace drone