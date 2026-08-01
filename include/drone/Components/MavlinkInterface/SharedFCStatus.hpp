#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"
#include <array>
#include <cstdint>

// LOCAL_POSITION_NED brute, pas calibrée — le recalage HomeCalibrator est
// appliqué par SensorFusion en aval. Bundlée pour un set/get atomique.
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

// Bitmasks MAV_SYS_STATUS_SENSOR bruts (SYS_STATUS) — santé vue par le FC
// lui-même, complémentaire aux codes SFS0xx/MVC0xx qui mesurent côté Pi.
struct FCSensorHealth {
  uint32_t present{0};
  uint32_t enabled{0};
  uint32_t health{0};
};

struct SharedFCStatus {
  SharedCompMem compMem;

  struct payload {
    TYPES::Attitude attitude{};       // ATTITUDE_QUATERNION
    TYPES::TimePoint attitudeTs{};    // horodatage de réception, pour staleness (ex: TEKF)
    FCLocalPosVel localPosVel{};      // LOCAL_POSITION_NED
    TYPES::TimePoint localPosVelTs{}; // idem, LOCAL_POSITION_NED
    FCBatteryStatus battery{};        // BATTERY_STATUS
    FCEscStatus esc{};                // ESC_STATUS
    TYPES::TimePoint escTs{};         // idem, pour détecter une donnée absente (TFSTask)
    FCPressure pressure{};            // SCALED_PRESSURE
    FCSensorHealth sensorHealth{};    // SYS_STATUS
    TYPES::TimePoint lastHeartbeat{}; // HEARTBEAT — pour TimeoutMonitor/FSTask
    bool armed{false};                // HEARTBEAT.base_mode & MAV_MODE_FLAG_SAFETY_ARMED
    TYPES::TimePoint armedTs{};
  };
  payload data;
};

class SharedFCStatusHandler : public SharedCompMemHandler {
public:
  explicit SharedFCStatusHandler(TYPES::ComponentID id, TYPES::Us timeout,
                                 SharedFCStatus &fc)
      : SharedCompMemHandler(fc.compMem, id, timeout), fc_(fc) {};

  // Écriture — appelée par TRx uniquement (mono-writer par convention, pas
  // imposé par le code ; cf. reset() pour la seule vérification d'identité).
  void setAttitude(TYPES::Attitude a) {
    setData(id_, timeout_, fc_.data.attitude, a);
    auto now = TYPES::Clock::now();
    setData(id_, timeout_, fc_.data.attitudeTs, now);
  }
  void setLocalPosVel(TYPES::Position pos, TYPES::Velocity vel) {
    FCLocalPosVel v{pos, vel};
    setData(id_, timeout_, fc_.data.localPosVel, v);
    auto now = TYPES::Clock::now();
    setData(id_, timeout_, fc_.data.localPosVelTs, now);
  }
  void setBattery(FCBatteryStatus b) {
    setData(id_, timeout_, fc_.data.battery, b);
  }
  void setEscStatus(FCEscStatus e) {
    setData(id_, timeout_, fc_.data.esc, e);
    auto now = TYPES::Clock::now();
    setData(id_, timeout_, fc_.data.escTs, now);
  }
  void setPressure(FCPressure p) {
    setData(id_, timeout_, fc_.data.pressure, p);
  }
  void setSensorHealth(FCSensorHealth h) {
    setData(id_, timeout_, fc_.data.sensorHealth, h);
  }
  void setLastHeartbeat(TYPES::TimePoint t) {
    setData(id_, timeout_, fc_.data.lastHeartbeat, t);
  }
  void setArmed(bool a) {
    setData(id_, timeout_, fc_.data.armed, a);
    auto now = TYPES::Clock::now();
    setData(id_, timeout_, fc_.data.armedTs, now);
  }

  // Lecture — SensorFusion (attitude/position/vitesse), MissionControl
  // (batterie/ESC/pression), FSTask (lastHeartbeat).
  [[nodiscard]] std::optional<TYPES::Attitude> getAttitude() {
    return getData(id_, timeout_, fc_.data.attitude);
  }
  [[nodiscard]] std::optional<TYPES::TimePoint> getAttitudeTs() {
    return getData(id_, timeout_, fc_.data.attitudeTs);
  }
  [[nodiscard]] std::optional<FCLocalPosVel> getLocalPosVel() {
    return getData(id_, timeout_, fc_.data.localPosVel);
  }
  [[nodiscard]] std::optional<TYPES::TimePoint> getLocalPosVelTs() {
    return getData(id_, timeout_, fc_.data.localPosVelTs);
  }
  [[nodiscard]] std::optional<FCBatteryStatus> getBattery() {
    return getData(id_, timeout_, fc_.data.battery);
  }
  [[nodiscard]] std::optional<FCEscStatus> getEscStatus() {
    return getData(id_, timeout_, fc_.data.esc);
  }
  [[nodiscard]] std::optional<TYPES::TimePoint> getEscStatusTs() {
    return getData(id_, timeout_, fc_.data.escTs);
  }
  [[nodiscard]] std::optional<FCPressure> getPressure() {
    return getData(id_, timeout_, fc_.data.pressure);
  }
  [[nodiscard]] std::optional<FCSensorHealth> getSensorHealth() {
    return getData(id_, timeout_, fc_.data.sensorHealth);
  }
  [[nodiscard]] std::optional<TYPES::TimePoint> getLastHeartbeat() {
    return getData(id_, timeout_, fc_.data.lastHeartbeat);
  }
  [[nodiscard]] std::optional<bool> getArmed() {
    return getData(id_, timeout_, fc_.data.armed);
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
