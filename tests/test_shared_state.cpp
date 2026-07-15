// // tests/test_shared_state.cpp
// // ─────────────────────────────────────────────────────────────────────────────

// #include "drone/shared_state.hpp"
// #include <atomic>
// #include <gtest/gtest.h>
// #include <thread>

// using namespace TYPES;
// using namespace CORE;

// // ─── SharedStateProvider ─────────────────────────────────────────────────────

// TEST(SharedStateTest, DefaultPositionIsZero) {
//   SharedStateProvider s;
//   EXPECT_TRUE(s.getPosition().ned.isZero());
// }

// TEST(SharedStateTest, WriteAndReadBack) {
//   SharedStateProvider s;

//   Position pos;
//   pos.ned = {1.0f, 2.0f, -3.0f};
//   Velocity vel;
//   vel.ned = {0.5f, 0.0f, 0.0f};
//   Attitude att;
//   att.q = Eigen::Quaternionf::Identity();
//   SensorHealth h;
//   h.gps_ok = h.mag_ok = h.lidar_ok = true;

//   s.setState(pos, vel, att, h);

//   EXPECT_TRUE(s.getPosition().ned.isApprox(pos.ned));
//   EXPECT_TRUE(s.getVelocity().ned.isApprox(vel.ned));
//   EXPECT_EQ(s.getSensorHealth().status(), HealthStatus::OK);
// }

// TEST(SharedStateTest, IsFreshAfterWrite) {
//   SharedStateProvider s;
//   Position p;
//   Velocity v;
//   Attitude a;
//   SensorHealth h;
//   s.setState(p, v, a, h);
//   EXPECT_TRUE(s.isFresh());
// }

// TEST(SharedStateTest, IsNotFreshByDefault) {
//   // Par défaut last_update_ = TimePoint{} = epoch → très vieux
//   SharedStateProvider s;
//   EXPECT_FALSE(s.isFresh());
// }

// TEST(SharedStateTest, ConcurrentReadWrite) {
//   // Plusieurs threads lisent pendant qu'un thread écrit — ne doit pas crasher
//   SharedStateProvider s;
//   std::atomic_bool stop{false};

//   // Thread écrivain
//   std::thread writer([&] {
//     Position p;
//     Velocity v;
//     Attitude a;
//     SensorHealth h;
//     while (!stop) {
//       p.ned = {1.0f, 2.0f, -3.0f};
//       s.setState(p, v, a, h);
//     }
//   });

//   // 3 threads lecteurs
//   std::vector<std::thread> readers;
//   for (int i = 0; i < 3; ++i) {
//     readers.emplace_back([&] {
//       while (!stop) {
//         [[maybe_unused]] auto pos = s.getPosition();
//         [[maybe_unused]] auto vel = s.getVelocity();
//       }
//     });
//   }

//   std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   stop = true;
//   writer.join();
//   for (auto &r : readers)
//     r.join();
//   // Si on arrive ici sans crash ni data race → OK
//   SUCCEED();
// }

// // ─── SharedNavOutput ─────────────────────────────────────────────────────────

// TEST(SharedNavOutputTest, DefaultIsHold) {
//   SharedNavOutput nav;
//   EXPECT_TRUE(nav.getVelocityCmd().velocity.ned.isZero());
//   EXPECT_EQ(nav.getNavMode(), NavMode::IDLE);
// }

// TEST(SharedNavOutputTest, ValidCmdIsStored) {
//   SharedNavOutput nav;
//   VelocityCmd cmd{.velocity = Velocity{Eigen::Vector3f{1.0f, 0.0f, 0.0f}},
//                   .timestamp = Clock::now()};
//   nav.setCmd(cmd, NavMode::WAYPOINT);

//   EXPECT_EQ(nav.getNavMode(), NavMode::WAYPOINT);
//   EXPECT_NEAR(nav.getVelocityCmd().velocity.ned.x(), 1.0f, 1e-5f);
// }

// TEST(SharedNavOutputTest, InvalidCmdFallsBackToHold) {
//   SharedNavOutput nav;

//   // Consigne trop rapide → doit être remplacée par HOLD
//   VelocityCmd bad_cmd{.velocity = Velocity{Eigen::Vector3f{99.0f, 0.0f, 0.0f}},
//                       .timestamp = Clock::now()};
//   nav.setCmd(bad_cmd, NavMode::WAYPOINT);

//   // Le SharedNavOutput doit avoir refusé la commande invalide
//   EXPECT_TRUE(nav.getVelocityCmd().velocity.ned.isZero());
//   EXPECT_EQ(nav.getNavMode(), NavMode::HOLD);
// }

// TEST(SharedNavOutputTest, NaNCmdFallsBackToHold) {
//   SharedNavOutput nav;
//   VelocityCmd nan_cmd{.velocity = Velocity{Eigen::Vector3f{
//                           std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}},
//                       .timestamp = Clock::now()};
//   nav.setCmd(nan_cmd, NavMode::WAYPOINT);
//   EXPECT_TRUE(nav.getVelocityCmd().velocity.ned.isZero());
// }

// // ─── SharedFCStatus ──────────────────────────────────────────────────────────

// TEST(SharedFCStatusTest, DefaultNotReachable) {
//   SharedFCStatus fc;
//   EXPECT_FALSE(fc.isFCReachable());
//   EXPECT_FALSE(fc.isFCAlive()); // last_msg_ = epoch = trop vieux
// }

// TEST(SharedFCStatusTest, UpdateAndRead) {
//   SharedFCStatus fc;
//   fc.update(10.5f, true, 4, true);

//   EXPECT_NEAR(fc.getBaroAltitude(), 10.5f, 1e-4f);
//   EXPECT_TRUE(fc.isArmed());
//   EXPECT_EQ(fc.getFCMode(), 4);
//   EXPECT_TRUE(fc.isFCReachable());
//   EXPECT_TRUE(fc.isFCAlive());
// }

// // ─── IStateProvider::isFresh (via interface)
// // ──────────────────────────────────

// TEST(InterfaceTest, IsFreshViaInterface) {
//   SharedStateProvider s;
//   const IStateProvider &iface = s; // accès via interface abstraite

//   EXPECT_FALSE(iface.isFresh());

//   Position p;
//   Velocity v;
//   Attitude a;
//   SensorHealth h;
//   s.setState(p, v, a, h);
//   EXPECT_TRUE(iface.isFresh());
// }

// // ─── INavOutput::isCmdFresh (via interface) ──────────────────────────────────

// TEST(InterfaceTest, IsCmdFreshAfterSet) {
//   SharedNavOutput nav;
//   const INavOutput &iface = nav;

//   VelocityCmd cmd = VelocityCmd::hold();
//   nav.setCmd(cmd, NavMode::HOLD);
//   EXPECT_TRUE(iface.isCmdFresh());
// }