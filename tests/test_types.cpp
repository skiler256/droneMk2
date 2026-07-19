// tests/test_types.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Types forts (drone/types.hpp). Lance : cd build && ctest -v

#include "drone/types.hpp"
#include <gtest/gtest.h>
#include <thread>
#include <type_traits>

using namespace TYPES;

// ─── Scalar<Tag> / types forts ──────────────────────────────────────────────

TEST(TypesTest, MetersIsExplicit) {
  // Ce test compile uniquement si Meters est bien un type fort.
  // La ligne suivante ne doit PAS compiler (décommenter pour vérifier) :
  //   Meters m = 5.0f;  // erreur : pas de conversion implicite
  Meters m{5.0f};
  EXPECT_FLOAT_EQ(m.v, 5.0f);
}

TEST(TypesTest, MetersPerSecIsExplicit) {
  MetersPerSec spd{3.0f};
  EXPECT_FLOAT_EQ(spd.v, 3.0f);
}

TEST(TypesTest, GaussIsExplicit) {
  Gauss g{0.42f};
  EXPECT_FLOAT_EQ(g.v, 0.42f);
}

TEST(TypesTest, HzIsExplicit) {
  Hz f{100.0f};
  EXPECT_FLOAT_EQ(f.v, 100.0f);
}

TEST(TypesTest, RadiansDegreesConversion) {
  Degrees deg{180.0f};
  Radians rad = toRadians(deg);
  EXPECT_NEAR(rad.v, 3.14159f, 1e-4f);

  Radians rad2{3.14159f};
  Degrees deg2 = toDegrees(rad2);
  EXPECT_NEAR(deg2.v, 180.0f, 1e-3f);
}

TEST(TypesTest, ScalarComparisons) {
  Meters a{1.0f}, b{2.0f};
  EXPECT_TRUE(a < b);
  EXPECT_TRUE(b > a);
  EXPECT_TRUE(a <= a);
  EXPECT_TRUE(b >= a);
  EXPECT_FALSE(a == b);
}

TEST(TypesTest, ScalarNegation) {
  Meters a{3.0f};
  Meters neg = -a;
  EXPECT_FLOAT_EQ(neg.v, -3.0f);
}

TEST(TypesTest, DifferentTagsAreDifferentTypes) {
  // Ce test compile uniquement si Meters et MetersPerSec sont bien deux
  // types distincts malgré la même représentation (float) — la ligne
  // suivante ne doit PAS compiler (décommenter pour vérifier) :
  //   Meters m{1.0f}; MetersPerSec s = m; // erreur : types incompatibles
  Meters m{1.0f};
  MetersPerSec s{1.0f};
  EXPECT_FLOAT_EQ(m.v, s.v); // même valeur numérique, types différents
}

// ─── Vecteurs 3D typés ───────────────────────────────────────────────────────

TEST(TypesTest, PositionNorm) {
  Position p;
  p.ned = Vec3{3.0f, 4.0f, 0.0f};
  EXPECT_FLOAT_EQ(p.norm().v, 5.0f);
}

TEST(TypesTest, VelocityNorm) {
  Velocity v;
  v.ned = Vec3{0.0f, 3.0f, 4.0f};
  EXPECT_FLOAT_EQ(v.norm().v, 5.0f);
}

TEST(TypesTest, AccelerationDefaultsToZero) {
  Acceleration a;
  EXPECT_TRUE(a.ned.isZero());
}

TEST(TypesTest, MagneticFieldNorm) {
  MagneticField f;
  f.B = Vec3{0.3f, 0.4f, 0.0f};
  EXPECT_FLOAT_EQ(f.norm().v, 0.5f);
}

TEST(TypesTest, Vec3IsTriviallyCopyable) {
  // Exigence architecturale : les types placés dans un payload shared
  // memory doivent être trivially-copyable (crc32<T>() l'exige, et le
  // memcpy implicite entre process via mmap aussi). Contrairement à
  // Eigen::Vector3f, qui ne le garantit pas.
  static_assert(std::is_trivially_copyable_v<Vec3>);
  static_assert(std::is_trivially_copyable_v<Quat>);
  static_assert(std::is_trivially_copyable_v<Position>);
  static_assert(std::is_trivially_copyable_v<Velocity>);
  static_assert(std::is_trivially_copyable_v<Acceleration>);
  static_assert(std::is_trivially_copyable_v<MagneticField>);
  static_assert(std::is_trivially_copyable_v<Attitude>);
  static_assert(std::is_trivially_copyable_v<Waypoint>);
  SUCCEED();
}

TEST(TypesTest, Vec3AsEigenMatchesComponents) {
  Vec3 v{1.0f, 2.0f, 3.0f};
  auto e = v.asEigen();
  EXPECT_FLOAT_EQ(e.x(), 1.0f);
  EXPECT_FLOAT_EQ(e.y(), 2.0f);
  EXPECT_FLOAT_EQ(e.z(), 3.0f);
}

TEST(TypesTest, QuatDefaultsToIdentity) {
  Attitude a;
  EXPECT_NEAR(a.roll().v, 0.0f, 1e-5f);
  EXPECT_NEAR(a.pitch().v, 0.0f, 1e-5f);
  EXPECT_NEAR(a.yaw().v, 0.0f, 1e-5f);
}

TEST(TypesTest, QuatYaw90Degrees) {
  // Rotation de 90° autour de Z (yaw) : w=cos(45°), z=sin(45°).
  Attitude a;
  a.q = Quat{0.70710678f, 0.0f, 0.0f, 0.70710678f};
  EXPECT_NEAR(toDegrees(a.yaw()).v, 90.0f, 1e-2f);
  EXPECT_NEAR(toDegrees(a.roll()).v, 0.0f, 1e-2f);
  EXPECT_NEAR(toDegrees(a.pitch()).v, 0.0f, 1e-2f);
}

TEST(TypesTest, WaypointDefaults) {
  Waypoint w;
  EXPECT_TRUE(w.target.ned.isZero());
  EXPECT_FLOAT_EQ(w.approach_speed.v, 1.5f);
  EXPECT_FLOAT_EQ(w.acceptance_radius.v, 0.3f);
}

TEST(TypesTest, VectorTypesDefaultToZero) {
  // Nécessaire pour que les struct Data des drivers (ex: MagData) restent
  // sûres par défaut avant la première lecture capteur.
  EXPECT_TRUE(Position{}.ned.isZero());
  EXPECT_TRUE(Velocity{}.ned.isZero());
  EXPECT_TRUE(MagneticField{}.B.isZero());
}

// ─── Horloge commune ─────────────────────────────────────────────────────────

TEST(TypesTest, ClockAdvances) {
  TimePoint t0 = Clock::now();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  TimePoint t1 = Clock::now();
  EXPECT_GT(t1, t0);
}

TEST(TypesTest, MsUsConversion) {
  Ms fiveMs{5};
  Us asUs = std::chrono::duration_cast<Us>(fiveMs);
  EXPECT_EQ(asUs.count(), 5000);
}

// ─── NavMode ─────────────────────────────────────────────────────────────────

TEST(NavModeTest, ToStringCoversAllModes) {
  const NavMode all[] = {NavMode::IDLE, NavMode::TAKEOFF, NavMode::WAYPOINT,
                         NavMode::HOLD, NavMode::LAND,    NavMode::EMERGENCY};
  for (auto m : all) {
    EXPECT_NE(toString(m), "UNKNOWN") << "Mode manquant dans toString()";
  }
}

TEST(NavModeTest, DistinctModesHaveDistinctStrings) {
  EXPECT_NE(toString(NavMode::HOLD), toString(NavMode::EMERGENCY));
}

// ─── Result<T, E> ────────────────────────────────────────────────────────────

TEST(TypesTest, ResultAliasWrapsExpected) {
  Result<int, DriverError> ok = 42;
  ASSERT_TRUE(ok.has_value());
  EXPECT_EQ(*ok, 42);

  Result<int, DriverError> err = std::unexpected(DriverError::Timeout);
  ASSERT_FALSE(err.has_value());
  EXPECT_EQ(err.error(), DriverError::Timeout);
}

// ─── Enums IPC / drivers : juste la structure (Count en dernier, valeurs
// distinctes), le comportement réel est couvert par test_shared_state.cpp /
// test_watchdog.cpp qui les utilisent en conditions réelles. ──────────────

TEST(TypesTest, ComponentIDCountIsLastAndSized) {
  // Count sert à dimensionner des tableaux (cf. SharedSysStateMem) — doit
  // rester strictement supérieur à tout ComponentID réel.
  EXPECT_GT(static_cast<uint8_t>(ComponentID::Count),
           static_cast<uint8_t>(ComponentID::GlobalWatchdog));
}

TEST(TypesTest, ShmErrorNoneIsZero) {
  // None doit valoir 0 : c'est le sentinel "pas d'erreur" utilisé partout
  // (valeur par défaut de lastError_ dans SharedMemoryHandler).
  EXPECT_EQ(static_cast<uint8_t>(shmError::None), 0);
}

TEST(TypesTest, DriverHealthValuesAreDistinct) {
  EXPECT_NE(DriverHealth::Unconnected, DriverHealth::Connected);
  EXPECT_NE(DriverHealth::Connected, DriverHealth::Dead);
}
