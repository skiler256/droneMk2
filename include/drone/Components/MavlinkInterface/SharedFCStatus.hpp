#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"
#include <array>
#include <cstdint>

// LOCAL_POSITION_NED (position + vitesse) : BRUTES, pas calibrées — le
// recalage par rapport au point home (HomeCalibrator) est appliqué par
// SensorFusion en aval, pas ici (cf. Document de Conception Logicielle
// §3.3.3). Bundlées dans un seul champ pour que set/get soit atomique
// (évite de lire une position et une vitesse d'instants différents).
struct FCLocalPosVel {
  TYPES::Position pos{};
  TYPES::Velocity vel{};
};

struct FCBatteryStatus {
  TYPES::Volts voltage{0.0f};
  TYPES::Amps current{0.0f};
  uint8_t remainingPct{0};
};

// Pas de Volts/Amps ici : un std::array<Scalar<Tag>, N> n'a pas de
// constructeur par défaut exploitable (Scalar est explicite à dessein) —
// float nu + commentaire d'unité, plus simple que d'initialiser les 4
// éléments à la main pour un array interne à ce seul composant.
struct FCEscStatus {
  std::array<float, 4> rpmValue{};       // tr/min
  std::array<float, 4> currentAmps{};    // A
  std::array<float, 4> temperatureC{};   // °C
};

struct FCPressure {
  float absHpa{0.0f};
  float diffHpa{0.0f};
  float temperatureC{0.0f};
};

struct SharedFCStatus {
  SharedCompMem compMem;

  struct payload {
    TYPES::Attitude attitude{};       // ATTITUDE_QUATERNION
    FCLocalPosVel localPosVel{};      // LOCAL_POSITION_NED
    FCBatteryStatus battery{};        // BATTERY_STATUS
    FCEscStatus esc{};                // ESC_STATUS
    FCPressure pressure{};            // SCALED_PRESSURE
    TYPES::TimePoint lastHeartbeat{}; // HEARTBEAT — pour TimeoutMonitor/FSTask
  };
  payload data;
};

class SharedFCStatusHandler : public SharedCompMemHandler {
public:
  explicit SharedFCStatusHandler(TYPES::ComponentID id, TYPES::Us timeout,
                                 SharedFCStatus &fc)
      : SharedCompMemHandler(fc.compMem, id, timeout), fc_(fc) {};

  // Écriture — appelée par TRx uniquement, mais pas de garde ici (le
  // segment SharedCompMem est mono-writer par convention, pas imposé par
  // le code ; cf. reset() ci-dessous pour la seule vérification d'identité).
  void setAttitude(TYPES::Attitude a) {
    setData(id_, timeout_, fc_.data.attitude, a);
  }
  void setLocalPosVel(TYPES::Position pos, TYPES::Velocity vel) {
    FCLocalPosVel v{pos, vel};
    setData(id_, timeout_, fc_.data.localPosVel, v);
  }
  void setBattery(FCBatteryStatus b) {
    setData(id_, timeout_, fc_.data.battery, b);
  }
  void setEscStatus(FCEscStatus e) {
    setData(id_, timeout_, fc_.data.esc, e);
  }
  void setPressure(FCPressure p) {
    setData(id_, timeout_, fc_.data.pressure, p);
  }
  void setLastHeartbeat(TYPES::TimePoint t) {
    setData(id_, timeout_, fc_.data.lastHeartbeat, t);
  }

  // Lecture — SensorFusion (attitude/position/vitesse), MissionControl
  // (batterie/ESC/pression), FSTask (lastHeartbeat).
  [[nodiscard]] std::optional<TYPES::Attitude> getAttitude() {
    return getData(id_, timeout_, fc_.data.attitude);
  }
  [[nodiscard]] std::optional<FCLocalPosVel> getLocalPosVel() {
    return getData(id_, timeout_, fc_.data.localPosVel);
  }
  [[nodiscard]] std::optional<FCBatteryStatus> getBattery() {
    return getData(id_, timeout_, fc_.data.battery);
  }
  [[nodiscard]] std::optional<FCEscStatus> getEscStatus() {
    return getData(id_, timeout_, fc_.data.esc);
  }
  [[nodiscard]] std::optional<FCPressure> getPressure() {
    return getData(id_, timeout_, fc_.data.pressure);
  }
  [[nodiscard]] std::optional<TYPES::TimePoint> getLastHeartbeat() {
    return getData(id_, timeout_, fc_.data.lastHeartbeat);
  }

private:
  SharedFCStatus &fc_;

  uint32_t computeChecksum(TYPES::ComponentID) {
    return UTILITIES::crc32(fc_.data) ^ historyChecksum();
  };

  void reset(TYPES::ComponentID id) {
    if (id == TYPES::ComponentID::MavlinkInterface) {
      fc_.data = SharedFCStatus::payload{};
      sanitizeHistory(comp_.HotStartHistory);
      sanitizeHistory(comp_.ColdStartHistory);
      updateChecksum(id);
    }
  };
};
