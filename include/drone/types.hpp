#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// drone/types.hpp
//
// Types forts pour toutes les grandeurs physiques du drone.
//
// Principe : deux grandeurs de types différents ne peuvent pas être
// confondues — le compilateur refuse le mélange. Un float brut ne peut
// pas être passé là où un Meters est attendu.
// ─────────────────────────────────────────────────────────────────────────────

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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

// ─── Vecteurs 3D POD ─────────────────────────────────────────────────────────
// Décision actée (CLAUDE.md) : pas de type Eigen dans un payload shared
// memory — Eigen ne garantit ni trivially-copyable ni standard-layout, et
// c'est requis par crc32<T>()/le memcpy implicite entre process via mmap.
// Vec3/Quat sont trois/quatre floats bruts, rien d'autre. `asEigen()` donne
// une vue Eigen::Map à la demande pour du calcul (Kalman, etc.) SANS jamais
// stocker le type Eigen lui-même — la struct qui l'utilise reste POD.

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

// Quaternion unitaire, identité par défaut (w=1) — contrairement à
// Eigen::Quaternionf, dont le défaut est non-initialisé.
struct Quat {
  float w{1.0f}, x{0.0f}, y{0.0f}, z{0.0f};

  [[nodiscard]] Eigen::Map<const Eigen::Quaternionf> asEigen() const noexcept {
    return Eigen::Map<const Eigen::Quaternionf>(&x); // Eigen: (x,y,z,w) en mémoire
  }
};

// Repère NED (North-East-Down) pour Position/Velocity/Acceleration : x=Nord,
// y=Est, z=Bas (altitude négative en vol). MagneticField reste en repère
// capteur (pas de rotation appliquée) — c'est le rôle de SensorFusion de la
// projeter en NED, pas de ce type.

struct Position { // mètres, NED
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

struct MagneticField { // Gauss, repère capteur
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
    s = std::clamp(s, -1.0f, 1.0f); // évite un domaine invalide pour asin au gimbal lock
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
  MetersPerSec approach_speed{1.5f}; // vitesse d'approche
  Meters acceptance_radius{0.3f};    // rayon d'acceptation
};

// ─── Mode de navigation ──────────────────────────────────────────────────────
// Cf. Design.md — state machine de Navigation, pas encore câblée mais déjà
// spécifiée : gardé ici même si pas encore consommé par du code actif.

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

// Alias pratique pour les retours faillibles
template <typename T, typename E> using Result = std::expected<T, E>;

// ─── IPC : identité des composants et de leurs segments partagés ────────────

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

enum class DriverHealth : uint8_t {
  Unconnected = 0,
  Connected,
  Dead,

  Count
};

}; // namespace TYPES
