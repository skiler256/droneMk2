// tests/test_missioncontrol.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Table de transition MC_CONV::step() — logique pure, testable sans passer
// par TStateMachine (loop() est privée). Lance : cd build && ctest -v

#include "drone/Components/MissionControl/Conversions.hpp"
#include <gtest/gtest.h>

using namespace TYPES;
using namespace MC_CONV;

namespace {

GCSCommand cmd(GCSCommandType type, uint8_t mode = 0) {
  return GCSCommand{.type = type, .mode = mode};
}

} // namespace

TEST(MissionControlTest, IdleArmsOnGcsArm) {
  Inputs in{.gcsCmd = cmd(GCSCommandType::Arm)};
  auto out = step(MissionState::Idle, MissionState::Idle, 0, in);
  EXPECT_EQ(out.next, MissionState::PreFlight);
}

TEST(MissionControlTest, IdleStaysWithoutArm) {
  auto out = step(MissionState::Idle, MissionState::Idle, 0, Inputs{});
  EXPECT_EQ(out.next, MissionState::Idle);
}

TEST(MissionControlTest, PreFlightPendingStays) {
  auto out = step(MissionState::PreFlight, MissionState::Idle, 0, Inputs{});
  EXPECT_EQ(out.next, MissionState::PreFlight);
}

TEST(MissionControlTest, PreFlightFailureGoesIdle) {
  auto out = step(MissionState::PreFlight, MissionState::Idle, 0,
                  Inputs{.preflightResult = false});
  EXPECT_EQ(out.next, MissionState::Idle);
}

TEST(MissionControlTest, PreFlightOkGoesArmed) {
  auto out = step(MissionState::PreFlight, MissionState::Idle, 0,
                  Inputs{.preflightResult = true});
  EXPECT_EQ(out.next, MissionState::Armed);
}

TEST(MissionControlTest, ArmedErrorGoesIdle) {
  auto out = step(MissionState::Armed, MissionState::Idle, 0, Inputs{.armError = true});
  EXPECT_EQ(out.next, MissionState::Idle);
}

TEST(MissionControlTest, ArmedDisarmGoesIdle) {
  Inputs in{.gcsCmd = cmd(GCSCommandType::Disarm)};
  auto out = step(MissionState::Armed, MissionState::Idle, 0, in);
  EXPECT_EQ(out.next, MissionState::Idle);
}

TEST(MissionControlTest, ArmedTakeoffGoesMission) {
  Inputs in{.gcsCmd = cmd(GCSCommandType::Takeoff)};
  auto out = step(MissionState::Armed, MissionState::Idle, 0, in);
  EXPECT_EQ(out.next, MissionState::Mission);
}

TEST(MissionControlTest, MissionToManualAndBack) {
  Inputs toManual{.gcsCmd = cmd(GCSCommandType::Mode, 1)};
  auto out1 = step(MissionState::Mission, MissionState::Idle, 0, toManual);
  EXPECT_EQ(out1.next, MissionState::Manual);

  Inputs toMission{.gcsCmd = cmd(GCSCommandType::Mode, 0)};
  auto out2 = step(MissionState::Manual, MissionState::Idle, 0, toMission);
  EXPECT_EQ(out2.next, MissionState::Mission);
}

TEST(MissionControlTest, MissionObstacleGoesHold) {
  auto out = step(MissionState::Mission, MissionState::Idle, 0, Inputs{.obstacleActive = true});
  EXPECT_EQ(out.next, MissionState::Hold);
}

TEST(MissionControlTest, VolumeBreachGoesHold) {
  auto out = step(MissionState::Mission, MissionState::Idle, 0, Inputs{.volumeBreached = true});
  EXPECT_EQ(out.next, MissionState::Hold);
}

TEST(MissionControlTest, HoldResumeReturnsToPrevious) {
  Inputs in{.gcsCmd = cmd(GCSCommandType::Resume)};
  auto out = step(MissionState::Hold, MissionState::Manual, 0, in);
  EXPECT_EQ(out.next, MissionState::Manual);
}

TEST(MissionControlTest, HoldWithoutResumeStays) {
  auto out = step(MissionState::Hold, MissionState::Mission, 0, Inputs{});
  EXPECT_EQ(out.next, MissionState::Hold);
}

TEST(MissionControlTest, MissionCompleteGoesLanding) {
  auto out = step(MissionState::Mission, MissionState::Idle, 0, Inputs{.missionComplete = true});
  EXPECT_EQ(out.next, MissionState::Landing);
}

// Pas de RTB en V1 (décidé explicitement) — CMD_RTL n'atteint même plus
// step() dans le vrai pipeline (TGCSCmdRouter le filtre, cf.
// MissionControl.cpp), mais on vérifie ici qu'un Rtl parvenu par erreur
// jusqu'à step() ne provoque aucune transition (pas d'état Rtb à atteindre).
TEST(MissionControlTest, RtlRequestIsNoOpInV1) {
  Inputs in{.gcsCmd = cmd(GCSCommandType::Rtl)};
  auto out = step(MissionState::Mission, MissionState::Idle, 0, in);
  EXPECT_EQ(out.next, MissionState::Mission);
}

TEST(MissionControlTest, LandedGoesIdle) {
  auto out = step(MissionState::Landing, MissionState::Idle, 0, Inputs{.landed = true});
  EXPECT_EQ(out.next, MissionState::Idle);
}

TEST(MissionControlTest, RecoverableFailureFromMissionGoesEmergency) {
  auto out = step(MissionState::Mission, MissionState::Idle, 0,
                  Inputs{.recoverableFailureActive = true});
  EXPECT_EQ(out.next, MissionState::Emergency);
}

TEST(MissionControlTest, UnrecoverableFailureFromMissionGoesLandingDirectly) {
  auto out = step(MissionState::Mission, MissionState::Idle, 0,
                  Inputs{.unrecoverableFailureActive = true});
  EXPECT_EQ(out.next, MissionState::Landing);
}

TEST(MissionControlTest, UnrecoverableFailureDuringEmergencyGoesLandingImmediately) {
  // Pas d'attente de réponse GCS ni de timeout si un échec irrécupérable
  // apparaît pendant une EMERGENCY déjà en cours.
  auto out = step(MissionState::Emergency, MissionState::Mission, 0,
                  Inputs{.recoverableFailureActive = true, .unrecoverableFailureActive = true});
  EXPECT_EQ(out.next, MissionState::Landing);
}

TEST(MissionControlTest, EmergencyPublishesQueryOnceFailureClears) {
  auto out = step(MissionState::Emergency, MissionState::Mission, 0,
                  Inputs{.recoverableFailureActive = false});
  EXPECT_EQ(out.next, MissionState::Emergency);
  ASSERT_TRUE(out.query.has_value());
  EXPECT_EQ(*out.query, QueryType::EmergencyRecoverable);
}

TEST(MissionControlTest, EmergencyNoQueryWhileFailureStillActive) {
  auto out = step(MissionState::Emergency, MissionState::Mission, 0,
                  Inputs{.recoverableFailureActive = true});
  EXPECT_EQ(out.next, MissionState::Emergency);
  EXPECT_FALSE(out.query.has_value());
}

TEST(MissionControlTest, LidarLostFromMissionGoesHold) {
  auto out = step(MissionState::Mission, MissionState::Idle, 0, Inputs{.lidarLost = true});
  EXPECT_EQ(out.next, MissionState::Hold);
}

TEST(MissionControlTest, HoldPublishesLidarLostQuery) {
  auto out = step(MissionState::Hold, MissionState::Mission, 0, Inputs{.lidarLost = true});
  EXPECT_EQ(out.next, MissionState::Hold);
  ASSERT_TRUE(out.query.has_value());
  EXPECT_EQ(*out.query, QueryType::LidarLost);
}

TEST(MissionControlTest, HoldLidarLostAnswerResumeReturnsToPrevious) {
  Inputs in{.lidarLost = true, .queryAnswer = std::pair<uint32_t, uint8_t>{7, 0}};
  auto out = step(MissionState::Hold, MissionState::Mission, 7, in);
  EXPECT_EQ(out.next, MissionState::Mission);
}

TEST(MissionControlTest, HoldLidarLostAnswerManualGoesManual) {
  Inputs in{.lidarLost = true, .queryAnswer = std::pair<uint32_t, uint8_t>{7, 1}};
  auto out = step(MissionState::Hold, MissionState::Mission, 7, in);
  EXPECT_EQ(out.next, MissionState::Manual);
}

TEST(MissionControlTest, EmergencyAnswerResumeReturnsToPrevious) {
  Inputs in{.queryAnswer = std::pair<uint32_t, uint8_t>{5, 0}};
  auto out = step(MissionState::Emergency, MissionState::Manual, 5, in);
  EXPECT_EQ(out.next, MissionState::Manual);
}

TEST(MissionControlTest, EmergencyAnswerLandGoesLanding) {
  Inputs in{.queryAnswer = std::pair<uint32_t, uint8_t>{5, 1}};
  auto out = step(MissionState::Emergency, MissionState::Manual, 5, in);
  EXPECT_EQ(out.next, MissionState::Landing);
}

TEST(MissionControlTest, EmergencyAnswerWrongSeqIgnored) {
  Inputs in{.queryAnswer = std::pair<uint32_t, uint8_t>{4, 0}}; // seq périmé
  auto out = step(MissionState::Emergency, MissionState::Manual, 5, in);
  EXPECT_EQ(out.next, MissionState::Emergency);
}

TEST(MissionControlTest, EmergencyTimeoutGoesLandingEvenWithoutAnswer) {
  Inputs in{.emergencyElapsed = kEmergencyTimeout};
  auto out = step(MissionState::Emergency, MissionState::Mission, 0, in);
  EXPECT_EQ(out.next, MissionState::Landing);
}

TEST(MissionControlTest, EmergencyFromHoldReturnsToOriginalFlightMode) {
  // previous = Mission (mode d'origine avant Hold), pas Hold lui-même.
  Inputs in{.queryAnswer = std::pair<uint32_t, uint8_t>{9, 0}};
  auto out = step(MissionState::Emergency, MissionState::Mission, 9, in);
  EXPECT_EQ(out.next, MissionState::Mission);
}
