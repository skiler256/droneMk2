#pragma once
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include <algorithm>
#include <cmath>

// Lois de correction extraites d'APTask::loop() pour être testables hors
// de la Task — cf. NAVIGATION.md §5.
namespace NAV_CONV {

// Même gain que l'axe vertical, par cohérence de "feel".
constexpr float kApproachKp = 0.6f;

struct WaypointGuidance {
  NavCommand cmd;
  TYPES::Meters distanceRemaining;
  bool arrived;
};

// P-controller saturé à segment.speed : pas de dépassement sur cible
// proche. Ne remplit pas velNed.z — cf. computeVerticalGuidance().
[[nodiscard]] inline WaypointGuidance
computeWaypointGuidance(const NavSegment &segment, const DynamicState &state,
                        TYPES::TimePoint now,
                        TYPES::Meters arrivalTolerance) noexcept {
  float dx = segment.exitWp.ned.x - state.posNed.ned.x; // Nord
  float dy = segment.exitWp.ned.y - state.posNed.ned.y; // Est
  float dist = std::sqrt(dx * dx + dy * dy);

  bool arrived = dist <= arrivalTolerance.v;
  float speed = std::clamp(kApproachKp * dist, 0.0f, segment.speed.v);

  TYPES::Vec3 velNed{};
  if (!arrived && dist > 0.0f) {
    velNed.x = speed * dx / dist;
    velNed.y = speed * dy / dist;
  }

  NavCommand cmd{.ts = now,
                 .velNed = TYPES::Velocity{velNed},
                 .headingCmd = TYPES::Radians{std::atan2(dy, dx)},
                 .altitudeCmd = segment.targetAltitude};

  return WaypointGuidance{
      .cmd = cmd, .distanceRemaining = TYPES::Meters{dist}, .arrived = arrived};
}

// Sous ce seuil, la direction de velNed est trop bruitée (GPS) : repli sur
// le cap vers la cible, TFS ayant toujours besoin d'une direction.
constexpr float kVelocityDirectionThreshold = 0.5f; // m/s

// Zone alignée sur la trajectoire réelle — cf. NAVIGATION.md §6.
constexpr float kObstacleBoxForwardM = 2.0f; // 0 à 2m devant le drone
constexpr float kObstacleBoxLateralM = 1.0f; // ±1m de part et d'autre

// Direction "avant" de TFS, recalculée indépendamment de TAP (sécurité
// redondante). DEUX INVARIANTS : composante verticale exclue du calcul de
// `speed`, et vecteur de sortie unitaire — cf. NAVIGATION.md §6, bug SITL.
[[nodiscard]] inline TYPES::Vec3
travelDirection(const NavSegment &segment, const DynamicState &state) noexcept {
  // HORIZONTALE seulement, jamais velNed.norm() : cf. l'invariant ci-dessus.
  float speed = std::sqrt(state.velNed.ned.x * state.velNed.ned.x +
                          state.velNed.ned.y * state.velNed.ned.y);
  if (speed > kVelocityDirectionThreshold) {
    return TYPES::Vec3{state.velNed.ned.x / speed, state.velNed.ned.y / speed, 0.0f};
  }

  float dx = segment.exitWp.ned.x - state.posNed.ned.x;
  float dy = segment.exitWp.ned.y - state.posNed.ned.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  if (dist > 0.0f)
    return TYPES::Vec3{dx / dist, dy / dist, 0.0f};

  return TYPES::Vec3{1.0f, 0.0f, 0.0f}; // sur la cible, vitesse nulle : Nord par défaut
}

// `forward` DOIT être unitaire (cf. travelDirection()). Test de boîte et
// non de cône : les points sont déjà en NED relatif au drone.
[[nodiscard]] inline bool obstacleInBox(const LidarMap &map,
                                        const TYPES::Vec3 &forward) noexcept {
  TYPES::Vec3 right{forward.y, -forward.x, 0.0f}; // perpendiculaire, rotation -90°

  for (const auto &p : map.points) {
    if (p.isZero())
      continue; // sentinelle "pas de mesure à cet angle"

    float fwd = p.x * forward.x + p.y * forward.y;
    float lat = p.x * right.x + p.y * right.y;
    if (fwd >= 0.0f && fwd <= kObstacleBoxForwardM &&
        std::abs(lat) <= kObstacleBoxLateralM)
      return true;
  }
  return false;
}

// Plus large que kArrivalTolerance : démarrer tolère plus qu'arriver.
constexpr float kEngageToleranceM = 2.0f;

// Distance à entryWp, pas à la cible — cf. NAVIGATION.md §3.
[[nodiscard]] inline bool canEngage(const NavSegment &segment,
                                    const DynamicState &state) noexcept {
  float dx = segment.entryWp.ned.x - state.posNed.ned.x;
  float dy = segment.entryWp.ned.y - state.posNed.ned.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  return dist <= kEngageToleranceM;
}

// Axe vertical, indépendant du latéral. NED : vz<0 = monte (§5).
constexpr float kAltitudeKp = 0.6f;
constexpr float kMaxClimbRateMs = 2.0f;

// Évite un mode FMA qui papillonne avec le bruit autour de la cible.
constexpr float kAltitudeHoldToleranceM = 0.3f;

struct VerticalGuidance {
  TYPES::MetersPerSec vz;
  CODES::Code modeCode;
};

[[nodiscard]] inline VerticalGuidance
computeVerticalGuidance(TYPES::Meters targetAltitude,
                        TYPES::Meters currentAltitude) noexcept {
  float error = targetAltitude.v - currentAltitude.v; // NED : cible plus haute -> négatif
  float vz = std::clamp(kAltitudeKp * error, -kMaxClimbRateMs, kMaxClimbRateMs);

  CODES::Code mode;
  if (std::abs(error) <= kAltitudeHoldToleranceM)
    mode = CODES::NVA007; // Hold
  else if (error < 0.0f)
    mode = CODES::NVA008; // Climb
  else
    mode = CODES::NVA009; // Descend

  return VerticalGuidance{.vz = TYPES::MetersPerSec{vz}, .modeCode = mode};
}

// Switch exhaustif sans default : le compilateur forcera une mise à jour
// quand Line/Curve seront ajoutés.
[[nodiscard]] inline CODES::Code lateralModeCode(SegmentType type) noexcept {
  switch (type) {
  case SegmentType::Waypoint:
    return CODES::NVA006;
  }
  return CODES::Code{}; // inatteignable tant que -Wswitch reste vert (cas exhaustifs ci-dessus)
}

} // namespace NAV_CONV
