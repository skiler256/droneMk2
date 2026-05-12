#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// drone/core/block_base.hpp
//
// Classe de base commune aux 4 blocs.
// Gère : démarrage/arrêt du thread, priorité RT, enregistrement watchdog.
//
// Un bloc concret hérite de BlockBase et implémente loop().
// ─────────────────────────────────────────────────────────────────────────────

#include "drone/core/watchdog.hpp"
#include <pthread.h>
#include <string>
#include <thread>
#include <atomic>
#include <iostream>

namespace drone {

class BlockBase {
public:
    BlockBase(std::string name, int rt_priority, Ms watchdog_timeout,
              Watchdog& watchdog)
        : name_(std::move(name))
        , rt_priority_(rt_priority)
        , watchdog_(watchdog)
    {
        // Enregistrement watchdog : la restart_fn relance start()
        watchdog_.registerComponent(name_, watchdog_timeout,
            [this] { this->start(); });
    }

    virtual ~BlockBase() { stop(); }

    void start() {
        if (running_.exchange(true)) return; // déjà lancé

        thread_ = std::thread(&BlockBase::run, this);

        // Priorité RT si disponible
        if (rt_priority_ > 0) {
            sched_param p{ .sched_priority = rt_priority_ };
            if (pthread_setschedparam(thread_.native_handle(),
                                      SCHED_FIFO, &p) != 0) {
                std::cerr << "[" << name_ << "] SCHED_FIFO non disponible\n";
            }
        }
        std::cout << "[" << name_ << "] démarré (prio=" << rt_priority_ << ")\n";
    }

    void stop() {
        if (!running_.exchange(false)) return; // déjà arrêté
        if (thread_.joinable()) thread_.join();
        std::cout << "[" << name_ << "] arrêté\n";
    }

    [[nodiscard]] bool isRunning() const noexcept { return running_; }
    [[nodiscard]] const std::string& name() const noexcept { return name_; }

protected:
    // Implémenté par chaque bloc — appelé en boucle tant que running_
    virtual void loop() = 0;

    // Appelé une fois avant la boucle (init optionnelle dans le thread RT)
    virtual void onStart() {}

    // Appelé une fois après la boucle (cleanup)
    virtual void onStop() {}

    std::atomic_bool running_{false};
    Watchdog&        watchdog_;
    std::string      name_;

private:
    void run() {
        onStart();
        while (running_) {
            loop();
            watchdog_.beat(name_);  // je suis vivant
        }
        onStop();
    }

    int         rt_priority_;
    std::thread thread_;
};

} // namespace drone