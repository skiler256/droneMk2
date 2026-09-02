// Navigation : loi de correction pure extraite d'APTask::loop() (cf.
// Conversions.hpp) — même contrainte que test_mavlink.cpp/test_sensorfusion.cpp :
// loop() est privée, appelée uniquement par le thread RT de la Task, donc pas
// testée directement ici. Lance : cd build && ctest -v

#include "drone/Components/Navigation/Conversions.hpp"
#include <gtest/gtest.h>
#include <numbers>

using namespace TYPES;
using namespace NAV_CONV;

namespace {

NavSegment makeWaypoint(Position target, MetersPerSec speed, Meters altitude) {
  return NavSegment{.seq = 1,
                    .type = SegmentType::Waypoint,
                    .exitWp = target,
                    .speed = speed,
                    .targetAltitude = altitude};
}

DynamicState makeState(Position pos) {
  return DynamicState{.ts = Clock::now(), .posNed = pos};
}

} // namespace

// ─── Cap ─────────────────────────────────────────────────────────────────

TEST(ComputeWaypointGuidanceTest, HeadingPointsDueNorth) {
  auto segment = makeWaypoint(Position{Vec3{10.0f, 0.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_NEAR(g.cmd.headingCmd.v, 0.0f, 1e-4f); // Nord = cap 0
}

TEST(ComputeWaypointGuidanceTest, HeadingPointsDueEast) {
  auto segment = makeWaypoint(Position{Vec3{0.0f, 10.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_NEAR(g.cmd.headingCmd.v, std::numbers::pi_v<float> / 2.0f, 1e-4f);
}

// ─── Vitesse : P-controller saturé ───────────────────────────────────────

TEST(ComputeWaypointGuidanceTest, SaturatesAtSegmentSpeedWhenFar) {
  // dist=100m, Kp=0.6 -> 60 m/s brut, saturé à segment.speed=5 m/s.
  auto segment = makeWaypoint(Position{Vec3{100.0f, 0.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_NEAR(g.cmd.velNed.norm().v, 5.0f, 1e-4f);
}

TEST(ComputeWaypointGuidanceTest, ProportionalWhenClose) {
  // dist=2m, Kp=0.6 -> 1.2 m/s, sous le plafond de 5 m/s.
  auto segment = makeWaypoint(Position{Vec3{2.0f, 0.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_NEAR(g.cmd.velNed.norm().v, kApproachKp * 2.0f, 1e-4f);
}

TEST(ComputeWaypointGuidanceTest, VelocityDirectedTowardTarget) {
  auto segment = makeWaypoint(Position{Vec3{0.0f, -8.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_GT(0.0f, g.cmd.velNed.ned.y); // vers l'Ouest (y négatif)
  EXPECT_NEAR(g.cmd.velNed.ned.x, 0.0f, 1e-4f);
}

// ─── Arrivée ─────────────────────────────────────────────────────────────

TEST(ComputeWaypointGuidanceTest, NotArrivedOutsideTolerance) {
  auto segment = makeWaypoint(Position{Vec3{5.0f, 0.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_FALSE(g.arrived);
}

TEST(ComputeWaypointGuidanceTest, ArrivedWithinTolerance) {
  auto segment = makeWaypoint(Position{Vec3{10.0f, 0.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{9.7f, 0.0f, 0.0f}}); // dist=0.3m < 0.5m

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_TRUE(g.arrived);
  EXPECT_NEAR(g.cmd.velNed.norm().v, 0.0f, 1e-4f); // hold, pas de vitesse résiduelle
}

TEST(ComputeWaypointGuidanceTest, ExactlyOnTargetIsArrivedWithoutNaN) {
  // dist=0 exactement : la division par dist dans le calcul de direction ne
  // doit jamais produire de NaN (garde dist > 0.0f dans computeWaypointGuidance).
  auto segment = makeWaypoint(Position{Vec3{3.0f, 4.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{3.0f, 4.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_TRUE(g.arrived);
  EXPECT_FALSE(std::isnan(g.cmd.velNed.ned.x));
  EXPECT_FALSE(std::isnan(g.cmd.velNed.ned.y));
  EXPECT_NEAR(g.cmd.velNed.norm().v, 0.0f, 1e-4f);
}

// ─── distanceRemaining / altitude ────────────────────────────────────────

TEST(ComputeWaypointGuidanceTest, DistanceRemainingMatchesPythagoras) {
  auto segment = makeWaypoint(Position{Vec3{3.0f, 4.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_NEAR(g.distanceRemaining.v, 5.0f, 1e-4f); // 3-4-5
}

TEST(ComputeWaypointGuidanceTest, AltitudeIsDirectPassthrough) {
  auto segment = makeWaypoint(Position{Vec3{10.0f, 0.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-42.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  auto g = computeWaypointGuidance(segment, state, Clock::now(), Meters{0.5f});
  EXPECT_FLOAT_EQ(g.cmd.altitudeCmd.v, -42.0f);
}

// ─── travelDirection (TFS) ────────────────────────────────────────────────

TEST(TravelDirectionTest, UsesVelocityWhenAboveThreshold) {
  auto segment = makeWaypoint(Position{Vec3{0.0f, 100.0f, 0.0f}}, // Est
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});
  state.velNed = Velocity{Vec3{2.0f, 0.0f, 0.0f}}; // Nord, > seuil 0.5 m/s

  auto dir = travelDirection(segment, state);
  EXPECT_NEAR(dir.x, 1.0f, 1e-4f);
  EXPECT_NEAR(dir.y, 0.0f, 1e-4f);
}

TEST(TravelDirectionTest, FallsBackToTargetBearingBelowThreshold) {
  auto segment = makeWaypoint(Position{Vec3{0.0f, 10.0f, 0.0f}}, // Est
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});
  state.velNed = Velocity{Vec3{0.1f, 0.0f, 0.0f}}; // < seuil 0.5 m/s

  auto dir = travelDirection(segment, state);
  EXPECT_NEAR(dir.x, 0.0f, 1e-4f);
  EXPECT_NEAR(dir.y, 1.0f, 1e-4f);
}

TEST(TravelDirectionTest, VerticalClimbDoesNotFalselyPassHorizontalThreshold) {
  // Bug reel trouve en SITL (session) : state.velNed.norm() est la norme 3D
  // complete -- en montee quasi verticale (vz domine), cette norme depasse
  // le seuil alors que vx/vy sont sous le seuil, et l'ancien code prenait
  // la branche "vitesse" en renvoyant un vecteur horizontal ECRASE (pas
  // unitaire) -- obstacleInBox() projetait alors des obstacles reels
  // lointains comme s'ils etaient tout pres (fausse alerte hold_obstacle
  // en pleine montee, sans aucun obstacle a proximite).
  auto segment = makeWaypoint(Position{Vec3{0.0f, 10.0f, 0.0f}}, // Est
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});
  state.velNed = Velocity{Vec3{0.1f, 0.05f, -3.0f}}; // horizontale << seuil, vz = 3 m/s

  auto dir = travelDirection(segment, state);
  // Doit retomber sur le cap vers la cible (est) -- pas un vecteur ecrase.
  EXPECT_NEAR(dir.x, 0.0f, 1e-4f);
  EXPECT_NEAR(dir.y, 1.0f, 1e-4f);
}

TEST(TravelDirectionTest, DegenerateCaseOnTargetWithNoVelocityHasNoNaN) {
  auto segment = makeWaypoint(Position{Vec3{5.0f, 5.0f, 0.0f}},
                              MetersPerSec{5.0f}, Meters{-10.0f});
  auto state = makeState(Position{Vec3{5.0f, 5.0f, 0.0f}}); // sur la cible, vitesse nulle

  auto dir = travelDirection(segment, state);
  EXPECT_FALSE(std::isnan(dir.x));
  EXPECT_FALSE(std::isnan(dir.y));
}

// ─── obstacleInBox (TFS) ──────────────────────────────────────────────────

TEST(ObstacleInBoxTest, EmptyMapIsNoObstacle) {
  LidarMap map{}; // tous les points à zéro (sentinelle "pas de mesure")
  EXPECT_FALSE(obstacleInBox(map, Vec3{1.0f, 0.0f, 0.0f}));
}

TEST(ObstacleInBoxTest, PointDirectlyAheadInBoxIsDetected) {
  LidarMap map{};
  map.points[0] = Vec3{1.0f, 0.0f, 0.0f}; // 1m devant, dans la boîte 2x2m
  EXPECT_TRUE(obstacleInBox(map, Vec3{1.0f, 0.0f, 0.0f}));
}

TEST(ObstacleInBoxTest, PointBeyondForwardRangeIsIgnored) {
  LidarMap map{};
  map.points[0] = Vec3{3.0f, 0.0f, 0.0f}; // au-delà des 2m de portée
  EXPECT_FALSE(obstacleInBox(map, Vec3{1.0f, 0.0f, 0.0f}));
}

TEST(ObstacleInBoxTest, PointTooFarLaterallyIsIgnored) {
  LidarMap map{};
  map.points[0] = Vec3{1.0f, 1.5f, 0.0f}; // dans la portée avant, hors ±1m latéral
  EXPECT_FALSE(obstacleInBox(map, Vec3{1.0f, 0.0f, 0.0f}));
}

TEST(ObstacleInBoxTest, PointBehindIsIgnored) {
  LidarMap map{};
  map.points[0] = Vec3{-1.0f, 0.0f, 0.0f}; // derrière le drone
  EXPECT_FALSE(obstacleInBox(map, Vec3{1.0f, 0.0f, 0.0f}));
}

TEST(ObstacleInBoxTest, WorksWithArbitraryForwardDirection) {
  // forward = Est : un point à l'Est doit être détecté, un point au Nord
  // (latéral pour cette direction) ne doit pas l'être.
  LidarMap map{};
  map.points[0] = Vec3{0.0f, 1.0f, 0.0f}; // Est
  EXPECT_TRUE(obstacleInBox(map, Vec3{0.0f, 1.0f, 0.0f}));

  LidarMap map2{};
  map2.points[0] = Vec3{1.5f, 0.0f, 0.0f}; // Nord, latéral pour forward=Est
  EXPECT_FALSE(obstacleInBox(map2, Vec3{0.0f, 1.0f, 0.0f}));
}

// ─── canEngage (TSeq) ─────────────────────────────────────────────────────

TEST(CanEngageTest, AcceptsWithinTolerance) {
  NavSegment segment{.entryWp = Position{Vec3{1.5f, 0.0f, 0.0f}}};
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}}); // dist=1.5m < 2m

  EXPECT_TRUE(canEngage(segment, state));
}

TEST(CanEngageTest, RejectsBeyondTolerance) {
  NavSegment segment{.entryWp = Position{Vec3{5.0f, 0.0f, 0.0f}}};
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}}); // dist=5m > 2m

  EXPECT_FALSE(canEngage(segment, state));
}

TEST(CanEngageTest, ChecksEntryWpNotExitWp) {
  // Loin de exitWp (la cible) mais proche de entryWp (le point de départ
  // attendu) — doit accepter : c'est bien entryWp qui compte ici.
  NavSegment segment{.entryWp = Position{Vec3{0.5f, 0.0f, 0.0f}},
                     .exitWp = Position{Vec3{100.0f, 0.0f, 0.0f}}};
  auto state = makeState(Position{Vec3{0.0f, 0.0f, 0.0f}});

  EXPECT_TRUE(canEngage(segment, state));
}

// ─── lateralModeCode ───────────────────────────────────────────────────────

TEST(LateralModeCodeTest, WaypointMapsToNVA006) {
  EXPECT_EQ(lateralModeCode(SegmentType::Waypoint), CODES::NVA006);
}

// ─── computeVerticalGuidance ────────────────────────────────────────────

TEST(ComputeVerticalGuidanceTest, HoldWithinTolerance) {
  // écart 0.1m < kAltitudeHoldToleranceM (0.3m)
  auto g = computeVerticalGuidance(Meters{-10.0f}, Meters{-10.1f});
  EXPECT_EQ(g.modeCode, CODES::NVA007);
  EXPECT_NEAR(g.vz.v, kAltitudeKp * 0.1f, 1e-4f);
}

TEST(ComputeVerticalGuidanceTest, ClimbsWhenTargetHigher) {
  // NED : cible plus haute = plus négative -> vz<0, mode Climb.
  auto g = computeVerticalGuidance(Meters{-15.0f}, Meters{-10.0f});
  EXPECT_EQ(g.modeCode, CODES::NVA008);
  EXPECT_LT(g.vz.v, 0.0f);
}

TEST(ComputeVerticalGuidanceTest, DescendsWhenTargetLower) {
  auto g = computeVerticalGuidance(Meters{-5.0f}, Meters{-10.0f});
  EXPECT_EQ(g.modeCode, CODES::NVA009);
  EXPECT_GT(g.vz.v, 0.0f);
}

TEST(ComputeVerticalGuidanceTest, SaturatesAtMaxClimbRate) {
  // écart 100m, Kp=0.6 -> 60 m/s brut, saturé à kMaxClimbRateMs (2 m/s).
  auto g = computeVerticalGuidance(Meters{-110.0f}, Meters{-10.0f});
  EXPECT_NEAR(g.vz.v, -kMaxClimbRateMs, 1e-4f);
}
