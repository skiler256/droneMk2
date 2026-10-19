#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <expected>
#include <string_view>

#ifdef DRONE_USE_SIM_CLOCK
#include "drone/core/SimClock.hpp"
#endif

namespace TYPES {

// ─── Horloge commune ────────────────────────────────────────────────────────
// Résolue à la compilation, pas de dispatch virtuel — cf. sitl/SITL.md.
#ifdef DRONE_USE_SIM_CLOCK
using Clock = SimClock;
#else
using Clock = std::chrono::steady_clock;
#endif
using TimePoint = Clock::time_point;
using Ms = std::chrono::milliseconds;
using Us = std::chrono::microseconds;

// ─── Types scalaires forts ──────────────────────────────────────────────────

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

struct TagMeters {};
using Meters = Scalar<TagMeters>;

struct TagMetersPerSec {};
using MetersPerSec = Scalar<TagMetersPerSec>;

struct TagGauss {};
using Gauss = Scalar<TagGauss>;

struct TagRadians {};
struct TagDegrees {};
using Radians = Scalar<TagRadians>;
using Degrees = Scalar<TagDegrees>;

[[nodiscard]] constexpr Radians toRadians(Degrees d) noexcept {
  return Radians{d.v * (3.14159265358979323846f / 180.0f)};
}
[[nodiscard]] constexpr Degrees toDegrees(Radians r) noexcept {
  return Degrees{r.v * (180.0f / 3.14159265358979323846f)};
}

struct TagHz {};
using Hz = Scalar<TagHz>;

struct TagVolts {};
struct TagAmps {};
using Volts = Scalar<TagVolts>;
using Amps = Scalar<TagAmps>;

// ─── Vecteurs 3D POD ─────────────────────────────────────────────────────────
// Floats bruts, jamais de type Eigen stocké : contrainte des payloads shm
// (cf. ARCHITECTURE_IPC.md §2.1). asEigen() rend une vue à la demande.

struct Vec3 {
  float x{0.0f}, y{0.0f}, z{0.0f};

  [[nodiscard]] float norm() const noexcept {
    return std::sqrt(x * x + y * y + z * z);
  }
  [[nodiscard]] bool isZero() const noexcept {
    return x == 0.0f && y == 0.0f && z == 0.0f;
  }

  [[nodiscard]] Eigen::Map<const Eigen::Vector3f> asEigen() const noexcept {
    return Eigen::Map<const Eigen::Vector3f>(&x);
  }
  [[nodiscard]] Eigen::Map<Eigen::Vector3f> asEigen() noexcept {
    return Eigen::Map<Eigen::Vector3f>(&x);
  }
};

// Identité par défaut (w=1), contrairement à Eigen::Quaternionf.
struct Quat {
  float w{1.0f}, x{0.0f}, y{0.0f}, z{0.0f};

  [[nodiscard]] Eigen::Map<const Eigen::Quaternionf> asEigen() const noexcept {
    return Eigen::Map<const Eigen::Quaternionf>(&x); // Eigen: (x,y,z,w) en mémoire
  }
};

struct Position { // mètres, NED (x=Nord, y=Est, z=Bas)
  Vec3 ned{};
  [[nodiscard]] Meters norm() const noexcept { return Meters{ned.norm()}; }
};

struct Velocity { // m/s, NED
  Vec3 ned{};
  [[nodiscard]] MetersPerSec norm() const noexcept {
    return MetersPerSec{ned.norm()};
  }
};

struct Acceleration { // m/s², NED
  Vec3 ned{};
};

// Repère capteur, pas NED : c'est SensorFusion qui projette.
struct MagneticField {
  Vec3 B{};
  [[nodiscard]] Gauss norm() const noexcept { return Gauss{B.norm()}; }
};

// ─── Attitude ───────────────────────────────────────────────────────────────

struct Attitude {
  Quat q{}; // quaternion unitaire, corps→NED

  [[nodiscard]] Radians roll() const noexcept {
    return Radians{std::atan2(2.0f * (q.w * q.x + q.y * q.z),
                              1.0f - 2.0f * (q.x * q.x + q.y * q.y))};
  }
  [[nodiscard]] Radians pitch() const noexcept {
    float s = 2.0f * (q.w * q.y - q.z * q.x);
    s = std::clamp(s, -1.0f, 1.0f); // domaine d'asin invalide au gimbal lock
    return Radians{std::asin(s)};
  }
  [[nodiscard]] Radians yaw() const noexcept {
    return Radians{std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                              1.0f - 2.0f * (q.y * q.y + q.z * q.z))};
  }
};

// ─── Waypoint ───────────────────────────────────────────────────────────────

struct Waypoint {
  Position target;
  MetersPerSec approach_speed{1.5f};
  Meters acceptance_radius{0.3f};
};

// ─── Mode de navigation ──────────────────────────────────────────────────────
// Spécifiée dans Design.md, pas encore câblée à du code actif.

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

template <typename T, typename E> using Result = std::expected<T, E>;

// ─── IPC : identité des composants et de leurs segments partagés ────────────

enum class ComponentID : uint8_t {
  MavlinkInterface = 0,
  SensorFusion,
  Navigation,
  MissionControl,
  SysMonitoring,
  GlobalWatchdog,

  Count // dimensionne les tableaux, jamais un index réel
};

enum class ComponentHealth : uint8_t {
  IDLE = 0,
  NOMINAL,
  DEAD,
  SICK,
  DEGRADED,

  Count
};

enum class shmError : uint8_t { None = 0, timeout, corrupt, Count };

using TaskID = uint8_t;

// ─── Drivers (capteurs bas niveau, I2C/UART) ────────────────────────────────

enum class DriverError : std::uint8_t {
  None = 0,
  I2COpenFailed,
  I2CAddressFailed,
  I2CReadFailed,
  I2CWriteFailed,
  UARTOpenFailed,
  UARTAttrGetFailed,
  UARTAttrSetFailed,
  UARTReadFailed,
  UARTWriteFailed,
  SocketOpenFailed,
  SocketBindFailed,
  SocketReadFailed,
  SocketWriteFailed,
  NotInitialized,
  Timeout,
  InvalidData,
  ConfigFailed,
  NoNewData, // aucun paquet complet reçu depuis le dernier update()
};

[[nodiscard]] constexpr std::string_view toString(DriverError e) noexcept {
  switch (e) {
  case DriverError::None:              return "None";
  case DriverError::I2COpenFailed:     return "I2COpenFailed";
  case DriverError::I2CAddressFailed:  return "I2CAddressFailed";
  case DriverError::I2CReadFailed:     return "I2CReadFailed";
  case DriverError::I2CWriteFailed:    return "I2CWriteFailed";
  case DriverError::UARTOpenFailed:    return "UARTOpenFailed";
  case DriverError::UARTAttrGetFailed: return "UARTAttrGetFailed";
  case DriverError::UARTAttrSetFailed: return "UARTAttrSetFailed";
  case DriverError::UARTReadFailed:    return "UARTReadFailed";
  case DriverError::UARTWriteFailed:   return "UARTWriteFailed";
  case DriverError::SocketOpenFailed:  return "SocketOpenFailed";
  case DriverError::SocketBindFailed:  return "SocketBindFailed";
  case DriverError::SocketReadFailed:  return "SocketReadFailed";
  case DriverError::SocketWriteFailed: return "SocketWriteFailed";
  case DriverError::NotInitialized:    return "NotInitialized";
  case DriverError::Timeout:           return "Timeout";
  case DriverError::InvalidData:       return "InvalidData";
  case DriverError::ConfigFailed:      return "ConfigFailed";
  case DriverError::NoNewData:         return "NoNewData";
  }
  return "UNKNOWN";
}

enum class DriverHealth : uint8_t {
  Unconnected = 0,
  Connected,
  Dead,

  Count
};

}; // namespace TYPES
