#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// drone/types.hpp
//
// Types forts pour toutes les grandeurs physiques du drone.
//
// Principe : deux grandeurs de types différents ne peuvent pas être
// confondues — le compilateur refuse le mélange. Un float brut ne peut
// pas être passé là où un Meters est attendu.
//
// Convention de repère : NED (North-East-Down)
//   x = Nord, y = Est, z = bas (altitude négative en vol)
// ─────────────────────────────────────────────────────────────────────────────

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <arm_neon.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <expected>
#include <string_view>

namespace TYPES {

// ─── Horloge commune ────────────────────────────────────────────────────────

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Ms = std::chrono::milliseconds;
using Us = std::chrono::microseconds;



// ─── Types scalaires forts ──────────────────────────────────────────────────
//
// Chaque type encapsule un float et n'est constructible que de façon
// explicite — impossible de passer un float nu par accident.

template <typename Tag> struct Scalar {
  float v;
  explicit constexpr Scalar(float f) noexcept : v(f) {}
  constexpr bool operator==(const Scalar &) const noexcept = default;
  constexpr bool operator<(const Scalar &o) const noexcept { return v < o.v; }
  constexpr bool operator<=(const Scalar &o) const noexcept { return v <= o.v; }
  constexpr bool operator>(const Scalar &o) const noexcept { return v > o.v; }
  constexpr bool operator>=(const Scalar &o) const noexcept { return v >= o.v; }
  constexpr Scalar operator-() const noexcept { return Scalar{-v}; }
};

// Grandeurs de distance / position
struct TagMeters {};
using Meters = Scalar<TagMeters>; // distance en mètres

// Grandeurs de vitesse
struct TagMetersPerSec {};
using MetersPerSec = Scalar<TagMetersPerSec>; // vitesse en m/s

// Grandeurs magnétiques
struct TagGauss {};
using Gauss = Scalar<TagGauss>;

// Grandeurs d'angle
struct TagRadians {};
struct TagDegrees {};
using Radians = Scalar<TagRadians>;
using Degrees = Scalar<TagDegrees>;

// Conversions explicites
[[nodiscard]] constexpr Radians toRadians(Degrees d) noexcept {
  return Radians{d.v * (3.14159265358979323846f / 180.0f)};
}
[[nodiscard]] constexpr Degrees toDegrees(Radians r) noexcept {
  return Degrees{r.v * (180.0f / 3.14159265358979323846f)};
}

// Grandeurs de fréquence / temps
struct TagHz {};
using Hz = Scalar<TagHz>; // fréquence en hertz

// ─── Vecteurs 3D NED ────────────────────────────────────────────────────────

struct Position { // mètres, NED
  Eigen::Vector3f ned = Eigen::Vector3f::Zero();
  ; // ned.x=Nord, ned.y=Est, ned.z=Bas
  [[nodiscard]] Meters norm() const noexcept { return Meters{ned.norm()}; }
};

struct Velocity { // m/s, NED
  Eigen::Vector3f ned = Eigen::Vector3f::Zero();
  ;
  [[nodiscard]] MetersPerSec norm() const noexcept {
    return MetersPerSec{ned.norm()};
  }
};

struct Acceleration { // m/s², NED
  Eigen::Vector3f ned = Eigen::Vector3f::Zero();
  ;
};

// -- Vecteurs ------------------------------------------------

struct magnetic_field {
  Eigen::Vector3f B = Eigen::Vector3f::Zero();
  ;
  [[nodiscard]] Gauss norm() const noexcept { return Gauss{B.norm()}; }
};
// ─── Attitude ───────────────────────────────────────────────────────────────

struct Attitude {
  Eigen::Quaternionf q; // quaternion unitaire, corps→NED

  [[nodiscard]] Radians roll() const noexcept;
  [[nodiscard]] Radians pitch() const noexcept;
  [[nodiscard]] Radians yaw() const noexcept;
};

// ─── Waypoint ───────────────────────────────────────────────────────────────

struct Waypoint {
  Position target;
  MetersPerSec approach_speed{1.5f}; // vitesse d'approche
  Meters acceptance_radius{0.3f};    // rayon d'acceptation
};

// ─── Consigne de vitesse (sortie Navigation → FC) ────────────────────────────

struct VelocityCmd {
  Velocity velocity;   // consigne en m/s NED
  TimePoint timestamp; // quand la consigne a été calculée

  // Limites physiques — vérifiées avant envoi au FC
  static constexpr float MAX_HORIZONTAL_MS = 5.0f; // m/s
  static constexpr float MAX_DESCENT_MS = 2.0f;    // m/s (descente)
  static constexpr float MAX_CLIMB_MS = 1.5f;      // m/s (montée)

  [[nodiscard]] bool isValid() const noexcept {
    const float vx = velocity.ned.x();
    const float vy = velocity.ned.y();
    const float vz = velocity.ned.z();
    if (std::isnan(vx) || std::isnan(vy) || std::isnan(vz))
      return false;
    if (std::isinf(vx) || std::isinf(vy) || std::isinf(vz))
      return false;
    const float horiz = std::sqrt(vx * vx + vy * vy);
    if (horiz > MAX_HORIZONTAL_MS)
      return false;
    if (vz > MAX_DESCENT_MS)
      return false; // NED : z>0 = descente
    if (vz < -MAX_CLIMB_MS)
      return false;
    return true;
  }

  // Consigne nulle de sécurité (HOLD)
  [[nodiscard]] static VelocityCmd hold() noexcept {
    return VelocityCmd{.velocity = Velocity{Eigen::Vector3f::Zero()},
                       .timestamp = Clock::now()};
  }
};

// ─── Erreurs typées ─────────────────────────────────────────────────────────

enum class SensorError {
  OutOfRange,
  Timeout,
  InvalidCRC,
  StaleData,
  NotInitialized,
  Connection,
};

enum class NavError {
  InvalidState,
  NoWaypoint,
  StagnationTimeout,
  InvalidCmd,
};

enum class MavlinkError {
  Timeout,
  InvalidMessage,
  FCUnreachable,
};

enum class SensorsT {
  GPS,
  Mag,
  Tel,
  Acc,
  Gyro,
  Baro,
};

[[nodiscard]] constexpr std::string_view toString(SensorsT sens) noexcept {
  switch (sens) {
  case SensorsT::GPS:
    return "GPS";
  case SensorsT::Mag:
    return "Mag";
  case SensorsT::Tel:
    return "Tel";
  case SensorsT::Acc:
    return "Acc";
  case SensorsT::Gyro:
    return "Gyro";
  case SensorsT::Baro:
    return "Baro";
  default:
    return "Unknown";
  }
}

// Alias pratique pour les retours faillibles
template <typename T, typename E> using Result = std::expected<T, E>;

// ─── État de santé du système ────────────────────────────────────────────────

enum class HealthStatus { OK, DEGRADED, CRITICAL };

struct SensorHealth {
  bool gps_ok = false;
  bool mag_ok = false;
  bool lidar_ok = false;
  uint8_t gps_reject_streak = 0; // mesures GPS rejetées consécutives
  uint8_t mag_reject_streak = 0;
  uint8_t lidar_reject_streak = 0;

  [[nodiscard]] HealthStatus status() const noexcept {
    if (!gps_ok && !lidar_ok)
      return HealthStatus::CRITICAL;
    if (!gps_ok || !mag_ok)
      return HealthStatus::DEGRADED;
    return HealthStatus::OK;
  }
};

// ─── Mode de navigation ──────────────────────────────────────────────────────

enum class NavMode {
  IDLE,      // au sol, pas armé
  TAKEOFF,   // décollage en cours
  WAYPOINT,  // suivi de waypoint
  HOLD,      // maintien de position
  LAND,      // atterrissage en cours
  EMERGENCY, // urgence — descente immédiate
};

[[nodiscard]] constexpr std::string_view toString(NavMode m) noexcept {
  switch (m) {
  case NavMode::IDLE:
    return "IDLE";
  case NavMode::TAKEOFF:
    return "TAKEOFF";
  case NavMode::WAYPOINT:
    return "WAYPOINT";
  case NavMode::HOLD:
    return "HOLD";
  case NavMode::LAND:
    return "LAND";
  case NavMode::EMERGENCY:
    return "EMERGENCY";
  }
  return "UNKNOWN";
}

// namespace TYPES

enum class ComponentID : uint8_t {
  MavlinkInterface = 0,
  SensorFusion,
  Navigation,
  MissionControl,
  SysMonitoring,
  GlobalWatchdog,

  Count // toujours en dernier : sert a dimensionner le tableau, jamais utilise
        // comme index reel
};

enum class ComponentHealth : uint8_t {
  IDLE = 0,
  NOMINAL,
  DEAD,
  SICK,
  DEGRADED,

  Count
};

enum class shmError : uint8_t { 
  timeout = 0, 
  corrupt, 
  Count
 };

using TaskID = uint8_t;

}; // namespace TYPES