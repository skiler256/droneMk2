// // tests/test_types.cpp
// //
// // Tests unitaires des types forts et de VelocityCmd.
// // Lance : cd build && ctest -v
// // ─────────────────────────────────────────────────────────────────────────────

// #include "drone/types.hpp"
// #include <gtest/gtest.h>

// using namespace TYPES;

// // ─── Types forts ─────────────────────────────────────────────────────────────

// TEST(TypesTest, MetersIsExplicit) {
//   // Ce test compile uniquement si Meters est bien un type fort.
//   // La ligne suivante ne doit PAS compiler (décommenter pour vérifier) :
//   //   Meters m = 5.0f;  // erreur : pas de conversion implicite
//   Meters m{5.0f};
//   EXPECT_FLOAT_EQ(m.v, 5.0f);
// }

// TEST(TypesTest, MetersPerSecIsExplicit) {
//   MetersPerSec spd{3.0f};
//   EXPECT_FLOAT_EQ(spd.v, 3.0f);
// }

// TEST(TypesTest, RadiansDegreesConversion) {
//   Degrees deg{180.0f};
//   Radians rad = toRadians(deg);
//   EXPECT_NEAR(rad.v, 3.14159f, 1e-4f);

//   Radians rad2{3.14159f};
//   Degrees deg2 = toDegrees(rad2);
//   EXPECT_NEAR(deg2.v, 180.0f, 1e-3f);
// }

// TEST(TypesTest, ScalarComparisons) {
//   Meters a{1.0f}, b{2.0f};
//   EXPECT_TRUE(a < b);
//   EXPECT_TRUE(b > a);
//   EXPECT_TRUE(a <= a);
//   EXPECT_FALSE(a == b);
// }

// TEST(TypesTest, ScalarNegation) {
//   Meters a{3.0f};
//   Meters neg = -a;
//   EXPECT_FLOAT_EQ(neg.v, -3.0f);
// }

// // ─── Position et Velocity
// // ─────────────────────────────────────────────────────

// TEST(PositionTest, Norm) {
//   Position p;
//   p.ned = Eigen::Vector3f{3.0f, 4.0f, 0.0f};
//   EXPECT_FLOAT_EQ(p.norm().v, 5.0f);
// }

// TEST(VelocityTest, Norm) {
//   Velocity v;
//   v.ned = Eigen::Vector3f{0.0f, 3.0f, 4.0f};
//   EXPECT_FLOAT_EQ(v.norm().v, 5.0f);
// }

// // ─── VelocityCmd::isValid ────────────────────────────────────────────────────

// TEST(VelocityCmdTest, ZeroIsValid) {
//   auto cmd = VelocityCmd::hold();
//   EXPECT_TRUE(cmd.isValid());
// }

// TEST(VelocityCmdTest, NominalIsValid) {
//   VelocityCmd cmd{.velocity = Velocity{Eigen::Vector3f{1.0f, 1.0f, 0.5f}},
//                   .timestamp = Clock::now()};
//   EXPECT_TRUE(cmd.isValid());
// }

// TEST(VelocityCmdTest, TooFastHorizontalIsInvalid) {
//   VelocityCmd cmd{
//       .velocity = Velocity{Eigen::Vector3f{10.0f, 0.0f, 0.0f}}, // 10 m/s > 5
//       .timestamp = Clock::now()};
//   EXPECT_FALSE(cmd.isValid());
// }

// TEST(VelocityCmdTest, TooFastDescentIsInvalid) {
//   VelocityCmd cmd{.velocity = Velocity{Eigen::Vector3f{
//                       0.0f, 0.0f, 5.0f}}, // 5 m/s descente > 2
//                   .timestamp = Clock::now()};
//   EXPECT_FALSE(cmd.isValid());
// }

// TEST(VelocityCmdTest, TooFastClimbIsInvalid) {
//   VelocityCmd cmd{.velocity = Velocity{Eigen::Vector3f{
//                       0.0f, 0.0f, -3.0f}}, // -3 m/s montée > 1.5
//                   .timestamp = Clock::now()};
//   EXPECT_FALSE(cmd.isValid());
// }

// TEST(VelocityCmdTest, NaNIsInvalid) {
//   VelocityCmd cmd{.velocity = Velocity{Eigen::Vector3f{
//                       std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}},
//                   .timestamp = Clock::now()};
//   EXPECT_FALSE(cmd.isValid());
// }

// TEST(VelocityCmdTest, InfIsInvalid) {
//   VelocityCmd cmd{.velocity = Velocity{Eigen::Vector3f{
//                       std::numeric_limits<float>::infinity(), 0.0f, 0.0f}},
//                   .timestamp = Clock::now()};
//   EXPECT_FALSE(cmd.isValid());
// }

// // ─── NavMode ─────────────────────────────────────────────────────────────────

// TEST(NavModeTest, ToStringCoversAllModes) {
//   // Vérifie que toString ne retourne pas "UNKNOWN" pour un mode valide
//   const NavMode all[] = {NavMode::IDLE, NavMode::TAKEOFF, NavMode::WAYPOINT,
//                          NavMode::HOLD, NavMode::LAND,    NavMode::EMERGENCY};
//   for (auto m : all) {
//     EXPECT_NE(toString(m), "UNKNOWN") << "Mode manquant dans toString()";
//   }
// }

// // ─── SensorHealth ────────────────────────────────────────────────────────────

// TEST(SensorHealthTest, AllOkIsOK) {
//   SensorHealth h;
//   h.gps_ok = h.mag_ok = h.lidar_ok = true;
//   EXPECT_EQ(h.status(), HealthStatus::OK);
// }

// TEST(SensorHealthTest, NoGpsIsDegraded) {
//   SensorHealth h;
//   h.gps_ok = false;
//   h.mag_ok = true;
//   h.lidar_ok = true;
//   EXPECT_EQ(h.status(), HealthStatus::DEGRADED);
// }

// TEST(SensorHealthTest, NoGpsNoLidarIsCritical) {
//   SensorHealth h;
//   h.gps_ok = false;
//   h.mag_ok = true;
//   h.lidar_ok = false;
//   EXPECT_EQ(h.status(), HealthStatus::CRITICAL);
// }