#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// drone/shared_state.hpp
//
// Implémentation concrète des interfaces IStateProvider, INavOutput, IFCStatus.
// Un seul objet SharedState est créé au démarrage et partagé entre tous
// les threads via leurs interfaces respectives.
//
// Thread-safety :
//   - shared_mutex : N lecteurs simultanés, 1 seul écrivain
//   - Chaque groupe de données a son propre mutex pour minimiser la contention
// ─────────────────────────────────────────────────────────────────────────────

#include "drone/interfaces.hpp"
#include <shared_mutex>
#include <mutex>

namespace drone {

// ─── SharedStateProvider ─────────────────────────────────────────────────────
// Écrit par ① Drivers+Fusion, lu par tous les autres blocs.

class SharedStateProvider final : public IStateProvider {
public:
    // ── Lecture (thread-safe, N lecteurs simultanés) ──────────────────────
    [[nodiscard]] Position getPosition() const noexcept override {
        std::shared_lock lock(mutex_);
        return position_;
    }
    [[nodiscard]] Velocity getVelocity() const noexcept override {
        std::shared_lock lock(mutex_);
        return velocity_;
    }
    [[nodiscard]] Attitude getAttitude() const noexcept override {
        std::shared_lock lock(mutex_);
        return attitude_;
    }
    [[nodiscard]] SensorHealth getSensorHealth() const noexcept override {
        std::shared_lock lock(mutex_);
        return health_;
    }
    [[nodiscard]] TimePoint getLastUpdateTime() const noexcept override {
        std::shared_lock lock(mutex_);
        return last_update_;
    }

    // ── Écriture (bloc ① uniquement) ──────────────────────────────────────
    void setState(Position pos, Velocity vel,
                  Attitude att, SensorHealth health) noexcept {
        std::unique_lock lock(mutex_);
        position_    = pos;
        velocity_    = vel;
        attitude_    = att;
        health_      = health;
        last_update_ = Clock::now();
    }

private:
    mutable std::shared_mutex mutex_;
    Position     position_    {};
    Velocity     velocity_    {};
    Attitude     attitude_    {};
    SensorHealth health_      {};
    TimePoint    last_update_ {};
};

// ─── SharedNavOutput ─────────────────────────────────────────────────────────
// Écrit par ② Navigation, lu par ③ MAVLink.

class SharedNavOutput final : public INavOutput {
public:
    // ── Lecture ───────────────────────────────────────────────────────────
    [[nodiscard]] VelocityCmd getVelocityCmd() const noexcept override {
        std::shared_lock lock(mutex_);
        return cmd_;
    }
    [[nodiscard]] NavMode getNavMode() const noexcept override {
        std::shared_lock lock(mutex_);
        return mode_;
    }
    [[nodiscard]] TimePoint getCmdTime() const noexcept override {
        std::shared_lock lock(mutex_);
        return cmd_.timestamp;
    }

    // ── Écriture (bloc ② uniquement) ──────────────────────────────────────
    void setCmd(VelocityCmd cmd, NavMode mode) noexcept {
        // Double vérification de sécurité avant d'écrire
        if (!cmd.isValid()) {
            cmd  = VelocityCmd::hold();
            mode = NavMode::HOLD;
        }
        std::unique_lock lock(mutex_);
        cmd_  = cmd;
        mode_ = mode;
    }

private:
    mutable std::shared_mutex mutex_;
    VelocityCmd cmd_  = VelocityCmd::hold();
    NavMode     mode_ = NavMode::IDLE;
};

// ─── SharedFCStatus ──────────────────────────────────────────────────────────
// Écrit par ③ MAVLink, lu par ② Navigation et ④ Monitoring.

class SharedFCStatus final : public IFCStatus {
public:
    // ── Lecture ───────────────────────────────────────────────────────────
    [[nodiscard]] float   getBaroAltitude()  const noexcept override {
        std::shared_lock lock(mutex_);
        return baro_alt_;
    }
    [[nodiscard]] bool    isArmed()          const noexcept override {
        std::shared_lock lock(mutex_);
        return armed_;
    }
    [[nodiscard]] uint8_t getFCMode()        const noexcept override {
        std::shared_lock lock(mutex_);
        return fc_mode_;
    }
    [[nodiscard]] bool    isFCReachable()    const noexcept override {
        std::shared_lock lock(mutex_);
        return reachable_;
    }
    [[nodiscard]] TimePoint getLastFCMessage() const noexcept override {
        std::shared_lock lock(mutex_);
        return last_msg_;
    }

    // ── Écriture (bloc ③ uniquement) ──────────────────────────────────────
    void update(float baro_alt, bool armed,
                uint8_t fc_mode, bool reachable) noexcept {
        std::unique_lock lock(mutex_);
        baro_alt_  = baro_alt;
        armed_     = armed;
        fc_mode_   = fc_mode;
        reachable_ = reachable;
        last_msg_  = Clock::now();
    }

private:
    mutable std::shared_mutex mutex_;
    float    baro_alt_  = 0.0f;
    bool     armed_     = false;
    uint8_t  fc_mode_   = 0;
    bool     reachable_ = false;
    TimePoint last_msg_ {};
};

// ─── AppState ────────────────────────────────────────────────────────────────
// Agrège les 3 états partagés — créé une seule fois dans le main().
// Chaque bloc reçoit uniquement l'interface dont il a besoin.

struct AppState {
    SharedStateProvider state;    // ① écrit, tous lisent
    SharedNavOutput     nav;      // ② écrit, ③ lit
    SharedFCStatus      fc;       // ③ écrit, ②④ lisent
};

} // namespace drone