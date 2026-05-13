#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// drone/interfaces.hpp
//
// Interfaces abstraites entre les 4 blocs.
// Aucun bloc ne dépend directement d'un autre — seulement de ces contrats.
// En vol réel : implémentations concrètes (SharedState, drivers…)
// En test/SIL : mocks injectés par les tests.
// ─────────────────────────────────────────────────────────────────────────────

#include "drone/types.hpp"

namespace drone {

// ─── IStateProvider ──────────────────────────────────────────────────────────
// Fourni par : ① Drivers+Fusion → lu par : ② Navigation, ③ MAVLink, ④ Monitoring

struct IStateProvider {
    virtual ~IStateProvider() = default;

    [[nodiscard]] virtual Position     getPosition()     const noexcept = 0;
    [[nodiscard]] virtual Velocity     getVelocity()     const noexcept = 0;
    [[nodiscard]] virtual Attitude     getAttitude()     const noexcept = 0;
    [[nodiscard]] virtual SensorHealth getSensorHealth() const noexcept = 0;
    [[nodiscard]] virtual TimePoint    getLastUpdateTime() const noexcept = 0;

    // Données fraîches = mise à jour il y a moins de MAX_STALENESS
    static constexpr Ms MAX_STALENESS{200};
    [[nodiscard]] bool isFresh() const noexcept {
        return (Clock::now() - getLastUpdateTime()) < MAX_STALENESS;
    }
};

// ─── INavOutput ──────────────────────────────────────────────────────────────
// Fourni par : ② Navigation → lu par : ③ MAVLink

struct INavOutput {
    virtual ~INavOutput() = default;

    [[nodiscard]] virtual VelocityCmd  getVelocityCmd()  const noexcept = 0;
    [[nodiscard]] virtual NavMode      getNavMode()      const noexcept = 0;
    [[nodiscard]] virtual TimePoint    getCmdTime()      const noexcept = 0;

    static constexpr Ms MAX_CMD_AGE{200};
    [[nodiscard]] bool isCmdFresh() const noexcept {
        return (Clock::now() - getCmdTime()) < MAX_CMD_AGE;
    }
};

// ─── IFCStatus ───────────────────────────────────────────────────────────────
// Fourni par : ③ MAVLink → lu par : ② Navigation, ④ Monitoring

struct IFCStatus {
    virtual ~IFCStatus() = default;

    [[nodiscard]] virtual float     getBaroAltitude()  const noexcept = 0;
    [[nodiscard]] virtual bool      isArmed()          const noexcept = 0;
    [[nodiscard]] virtual uint8_t   getFCMode()        const noexcept = 0;
    [[nodiscard]] virtual bool      isFCReachable()    const noexcept = 0;
    [[nodiscard]] virtual TimePoint getLastFCMessage() const noexcept = 0;

    static constexpr Ms FC_TIMEOUT{2000};
    [[nodiscard]] bool isFCAlive() const noexcept {
        return (Clock::now() - getLastFCMessage()) < FC_TIMEOUT;
    }
};

// ─── IHealthSink ─────────────────────────────────────────────────────────────
// Consommé par : ④ Monitoring — tous les blocs peuvent y écrire

struct IHealthSink {
    virtual ~IHealthSink() = default;

    virtual void reportHealth(std::string_view component,
                              HealthStatus     status,
                              std::string_view detail = "") noexcept = 0;

    virtual void reportMetric(std::string_view key,
                              float            value) noexcept = 0;
};

// ─── IWatchdogClient ─────────────────────────────────────────────────────────
// Chaque bloc l'utilise pour signaler qu'il est vivant

struct IWatchdogClient {
    virtual ~IWatchdogClient() = default;
    virtual void beat(std::string_view component_name) noexcept = 0;
};

} // namespace drone