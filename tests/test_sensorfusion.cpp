// tests/test_sensorfusion.cpp
// ─────────────────────────────────────────────────────────────────────────────
// SensorFusion : conversions pures extraites de GPSTask/LidarTask (cf.
// Conversions.hpp) — mêmes contraintes que test_mavlink.cpp : les loop() des
// Tasks (GPSTask/EKFTask/LidarTask) sont privées, appelées uniquement par le
// thread RT de chaque Task, donc pas testées directement ici. Lance :
// cd build && ctest -v

#include "drone/Components/SensorFusions/Conversions.hpp"
#include <gtest/gtest.h>
#include <numbers>

using namespace TYPES;
using namespace SF_CONV;

// ─── mapUbxFixType ───────────────────────────────────────────────────────

TEST(MapUbxFixTypeTest, PassesThroughKnownValues) {
  EXPECT_EQ(mapUbxFixType(0), 0); // no fix
  EXPECT_EQ(mapUbxFixType(2), 2); // 2D
  EXPECT_EQ(mapUbxFixType(3), 3); // 3D
}

TEST(MapUbxFixTypeTest, DegradesUnmappedValuesToNoFix) {
  EXPECT_EQ(mapUbxFixType(1), 1); // dead reckoning seul
  EXPECT_EQ(mapUbxFixType(4), 1); // GNSS+dead reckoning
  EXPECT_EQ(mapUbxFixType(5), 1); // time only
  EXPECT_EQ(mapUbxFixType(255), 1); // valeur inconnue
}

// ─── gpsWeekFromUbxDate ──────────────────────────────────────────────────

TEST(GpsWeekFromUbxDateTest, EpochDateIsWeekZero) {
  int timeArray[6] = {1980, 1, 6, 0, 0, 0}; // époque GPS elle-même
  EXPECT_EQ(gpsWeekFromUbxDate(timeArray), 0);
}

TEST(GpsWeekFromUbxDateTest, OneWeekAfterEpochIsWeekOne) {
  int timeArray[6] = {1980, 1, 13, 0, 0, 0};
  EXPECT_EQ(gpsWeekFromUbxDate(timeArray), 1);
}

TEST(GpsWeekFromUbxDateTest, PlausibleModernDate) {
  // Valeur croisée manuellement (script Python indépendant) — sert surtout
  // de garde non-régression sur l'algorithme sys_days.
  int timeArray[6] = {2026, 7, 25, 0, 0, 0};
  EXPECT_EQ(gpsWeekFromUbxDate(timeArray), 2428);
}

// ─── gpsCoordPlausible ───────────────────────────────────────────────────

TEST(GpsCoordPlausibleTest, AcceptsRealisticCoordinates) {
  EXPECT_TRUE(gpsCoordPlausible(44.1, 0.7));
  EXPECT_TRUE(gpsCoordPlausible(-90.0, -180.0)); // bornes incluses
  EXPECT_TRUE(gpsCoordPlausible(90.0, 180.0));
}

TEST(GpsCoordPlausibleTest, RejectsOutOfRangeCoordinates) {
  EXPECT_FALSE(gpsCoordPlausible(90.1, 0.0));
  EXPECT_FALSE(gpsCoordPlausible(0.0, 180.1));
  EXPECT_FALSE(gpsCoordPlausible(-91.0, 0.0));
}

// ─── gpsVelNedMs ─────────────────────────────────────────────────────────

TEST(GpsVelNedMsTest, ConvertsCentimetersPerSecondToMetersPerSecond) {
  Vec3 v = gpsVelNedMs(150, -50, 1000);
  EXPECT_FLOAT_EQ(v.x, 1.5f);
  EXPECT_FLOAT_EQ(v.y, -0.5f);
  EXPECT_FLOAT_EQ(v.z, 10.0f);
}

// ─── gpsAccuracyOk ───────────────────────────────────────────────────────

TEST(GpsAccuracyOkTest, AcceptsWithinThresholds) {
  EXPECT_TRUE(gpsAccuracyOk(2000, 500, 5000, 1000));
  EXPECT_TRUE(gpsAccuracyOk(5000, 1000, 5000, 1000)); // bornes incluses
}

TEST(GpsAccuracyOkTest, RejectsBeyondThresholds) {
  EXPECT_FALSE(gpsAccuracyOk(5001, 500, 5000, 1000));
  EXPECT_FALSE(gpsAccuracyOk(2000, 1001, 5000, 1000));
}

// ─── lidarPointNed ───────────────────────────────────────────────────────

TEST(LidarPointNedTest, ZeroAngleWithIdentityAttitudeIsPureForward) {
  Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
  Vec3 p = lidarPointNed(2.0f, 0.0f, identity);
  EXPECT_NEAR(p.x, 2.0f, 1e-4f);
  EXPECT_NEAR(p.y, 0.0f, 1e-4f);
  EXPECT_NEAR(p.z, 0.0f, 1e-4f);
}

TEST(LidarPointNedTest, NinetyDegreesWithIdentityAttitudeIsPureRight) {
  Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
  Vec3 p = lidarPointNed(2.0f, 90.0f, identity);
  EXPECT_NEAR(p.x, 0.0f, 1e-4f);
  EXPECT_NEAR(p.y, 2.0f, 1e-4f);
  EXPECT_NEAR(p.z, 0.0f, 1e-4f);
}

TEST(LidarPointNedTest, AttitudeRotationIsApplied) {
  // Cap arbitraire (yaw 90° autour de Z) : le point doit suivre la même
  // rotation que celle appliquée directement via Eigen, pas rester au repère
  // capteur — vérifie que lidarPointNed applique bien attitudeQ, sans
  // présumer d'une convention de repère particulière au-delà de ça.
  Eigen::Quaternionf yaw90(Eigen::AngleAxisf(std::numbers::pi_v<float> / 2.0f,
                                             Eigen::Vector3f::UnitZ()));
  Vec3 p = lidarPointNed(1.0f, 0.0f, yaw90);

  Eigen::Vector3f expected = yaw90 * Eigen::Vector3f{1.0f, 0.0f, 0.0f};
  EXPECT_NEAR(p.x, expected.x(), 1e-4f);
  EXPECT_NEAR(p.y, expected.y(), 1e-4f);
  EXPECT_NEAR(p.z, expected.z(), 1e-4f);
}
