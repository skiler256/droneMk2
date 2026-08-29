#pragma once
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include <algorithm>
#include <cmath>

// Lois de correction extraites d'APTask::loop() pour être testables sans
// passer par la Task (loop() est privée, appelée uniquement par le thread
// RT de la Task) — même principe que MavlinkInterface/Conversions.hpp et
// SensorFusions/Conversions.hpp.
namespace NAV_CONV {

// Même gain que le P-controller altitude ci-dessous (computeVerticalGuidance)
// — cohérence de "feel" entre les deux axes.
constexpr float kApproachKp = 0.6f;

struct WaypointGuidance {
  NavCommand cmd;
  TYPES::Meters distanceRemaining;
  bool arrived;
};

// Cap direct sur la cible, vitesse d'approche en P-controller saturé à
// segment.speed (pas de dépassement même sur une cible proche) — l'axe
// vertical (velNed.z) n'est pas rempli ici, cf. computeVerticalGuidance().
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

// Sous ce seuil, la direction de state.velNed est trop bruitée (bruit GPS)
// pour être fiable — repli sur le cap vers la cible (il faut toujours une
// direction à observer pour TFS).
constexpr float kVelocityDirectionThreshold = 0.5f; // m/s

// Zone de détection devant le drone, alignée sur la trajectoire réelle
// (angle mort du LIDAR ~200° centré sur le nez, cf. NAVIGATION.md §6).
constexpr float kObstacleBoxForwardM = 2.0f; // 0 à 2m devant le drone
constexpr float kObstacleBoxLateralM = 1.0f; // ±1m de part et d'autre

// Direction "avant" pour TFS — recalculée indépendamment de
// computeWaypointGuidance (TFS est une sécurité redondante, elle ne fait
// confiance à aucun calcul déjà fait par TAP).
[[nodiscard]] inline TYPES::Vec3
travelDirection(const NavSegment &segment, const DynamicState &state) noexcept {
  // Vitesse HORIZONTALE seulement -- pas state.velNed.norm() (norme 3D
  // complète, cf. Vec3::norm()) : en montée/descente quasi verticale, cette
  // norme est dominée par vz, "speed" passe le seuil alors que vx/vy sont
  // quasi nuls, et le vecteur retourné (vx/speed, vy/speed, 0) n'est alors
  // plus unitaire (norme << 1). obstacleInBox() suppose `forward` unitaire
  // pour projeter les points LIDAR -- avec un vecteur ecrase, un obstacle
  // reel a plusieurs metres se projette a quelques centimetres et tombe
  // faussement dans la boite de securite (bug constate en SITL : hold_obstacle
  // se declenchait en pleine montee sans aucun obstacle reel a proximite).
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

// Un point LidarMap tombe-t-il dans la zone de sécurité ? `forward` doit
// être unitaire (cf. travelDirection()). Test de boîte (projection sur les
// axes avant/latéral) plutôt qu'un cône — plus simple, et les points sont
// déjà en NED relatif au drone donc directement exploitables.
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

// Plus large qu'à l'arrivée (kArrivalTolerance, Navigation.cpp) — démarrer
// un segment tolère un peu plus de marge qu'"être arrivé".
constexpr float kEngageToleranceM = 2.0f;

// Sécurité avant promotion armé->engagé (TSeq) : distance à entryWp (pas
// exitWp, la cible) — cf. NAVIGATION.md §3.
[[nodiscard]] inline bool canEngage(const NavSegment &segment,
                                    const DynamicState &state) noexcept {
  float dx = segment.entryWp.ned.x - state.posNed.ned.x;
  float dy = segment.entryWp.ned.y - state.posNed.ned.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  return dist <= kEngageToleranceM;
}

// Axe vertical, indépendant du latéral — migré depuis
// MavlinkInterface::altitudeToVz() (mêmes gains, cf. NAVIGATION.md §5).
// NED : vz<0 = monte.
constexpr float kAltitudeKp = 0.6f;
constexpr float kMaxClimbRateMs = 2.0f;

// En dessous de cet écart, considéré Hold plutôt que Climb/Descend — évite
// d'afficher un mode qui change à chaque bruit de mesure autour de la cible.
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

// Code FMA latéral pour un type de segment donné — switch exhaustif
// (pas de default) : le compilateur force une mise à jour ici quand
// Line/Curve seront ajoutés.
[[nodiscard]] inline CODES::Code lateralModeCode(SegmentType type) noexcept {
  switch (type) {
  case SegmentType::Waypoint:
    return CODES::NVA006;
  }
  return CODES::Code{}; // inatteignable tant que -Wswitch reste vert (cas exhaustifs ci-dessus)
}

} // namespace NAV_CONV
