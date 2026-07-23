// tests/test_mavlink.cpp
// ─────────────────────────────────────────────────────────────────────────────
// MavlinkInterface : conversions pures, matching ACK, AckFifo, segments shm
// (SharedFCStatus/SharedSFMem/SharedNavMem), et le mécanisme de parsing
// mavlink_parse_char() dont TRx dépend (tronqué/concaténé/CRC invalide).
//
// Les loop() des Tasks (TTx/TTxACK/TRx) sont privées, appelées uniquement
// par le thread RT de chaque Task — pas testées directement ici. Ce fichier
// couvre la logique qui en a été extraite pour rester testable (cf.
// Conversions.hpp) et les briques indépendantes (AckFifo, shm). Lance :
// cd build && ctest -v

#include "drone/Components/MavlinkInterface/AckFifo.hpp"
#include "drone/Components/MavlinkInterface/Conversions.hpp"
#include "drone/Components/MavlinkInterface/PendingAck.hpp"
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include <gtest/gtest.h>

using namespace TYPES;
using namespace MAVLINK_CONV;

// ─── Conversions ─────────────────────────────────────────────────────────

TEST(ConversionsTest, HeadingToYawCdegBasic) {
  EXPECT_EQ(headingToYawCdeg(Radians{0.0f}), 36000); // 0° = "non disponible" -> remap plein nord
  EXPECT_NEAR(headingToYawCdeg(Radians{3.14159265f}), 18000, 2); // 180°
}

TEST(ConversionsTest, HeadingToYawCdegWrapsNegative) {
  // -90° doit devenir 270°, jamais une valeur négative.
  auto cdeg = headingToYawCdeg(Radians{-3.14159265f / 2.0f});
  EXPECT_NEAR(cdeg, 27000, 2);
}

TEST(ConversionsTest, HeadingToYawCdegWrapsAboveFullCircle) {
  // 370° -> 10°
  float rad370 = 370.0f * (3.14159265f / 180.0f);
  auto cdeg = headingToYawCdeg(Radians{rad370});
  EXPECT_NEAR(cdeg, 1000, 2);
}

TEST(ConversionsTest, DegToE7RoundTrip) {
  EXPECT_EQ(degToE7(44.123456), 441234560);
  EXPECT_EQ(degToE7(-1.5), -15000000);
}

TEST(ConversionsTest, MvToVolts) {
  EXPECT_FLOAT_EQ(mvToVolts(12600).v, 12.6f);
  EXPECT_FLOAT_EQ(mvToVolts(0).v, 0.0f);
}

TEST(ConversionsTest, CaToAmpsHandlesUnknown) {
  EXPECT_FLOAT_EQ(caToAmps(150).v, 1.5f);   // 150 cA = 1.5A
  EXPECT_FLOAT_EQ(caToAmps(-1).v, 0.0f);    // -1 = non mesuré -> 0
}

TEST(ConversionsTest, ClampBatteryPctHandlesUnknown) {
  EXPECT_EQ(clampBatteryPct(42), 42);
  EXPECT_EQ(clampBatteryPct(-1), 0); // -1 = inconnu -> 0
}

TEST(ConversionsTest, CdegcToC) {
  EXPECT_FLOAT_EQ(cdegcToC(2550), 25.5f);
  EXPECT_FLOAT_EQ(cdegcToC(-500), -5.0f);
}

// ─── ackMatchesPending ───────────────────────────────────────────────────

TEST(AckMatchTest, CommandAckMatchesSameCommand) {
  PendingAck pending{.kind = PendingKind::Command, .command = MAV_CMD_DO_SET_MODE};

  mavlink_message_t msg{};
  mavlink_msg_command_ack_pack(1, 1, &msg, MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED,
                               255, 0, 0, 0);

  EXPECT_TRUE(ackMatchesPending(msg, pending));
}

TEST(AckMatchTest, CommandAckDoesNotMatchDifferentCommand) {
  PendingAck pending{.kind = PendingKind::Command, .command = MAV_CMD_DO_SET_MODE};

  mavlink_message_t msg{};
  mavlink_msg_command_ack_pack(1, 1, &msg, MAV_CMD_SET_MESSAGE_INTERVAL,
                               MAV_RESULT_ACCEPTED, 255, 0, 0, 0);

  EXPECT_FALSE(ackMatchesPending(msg, pending));
}

TEST(AckMatchTest, ParamValueMatchesSameParamId) {
  PendingAck pending{.kind = PendingKind::Param};
  std::strcpy(pending.paramId.data(), "ARMING_CHECK");

  mavlink_message_t msg{};
  mavlink_msg_param_value_pack(1, 1, &msg, "ARMING_CHECK", 1.0f, MAV_PARAM_TYPE_REAL32,
                               1, 0);

  EXPECT_TRUE(ackMatchesPending(msg, pending));
}

TEST(AckMatchTest, ParamValueDoesNotMatchDifferentParamId) {
  PendingAck pending{.kind = PendingKind::Param};
  std::strcpy(pending.paramId.data(), "ARMING_CHECK");

  mavlink_message_t msg{};
  mavlink_msg_param_value_pack(1, 1, &msg, "BATT_LOW_VOLT", 1.0f, MAV_PARAM_TYPE_REAL32,
                               1, 0);

  EXPECT_FALSE(ackMatchesPending(msg, pending));
}

TEST(AckMatchTest, WrongMessageKindNeverMatches) {
  // PARAM_VALUE reçu alors qu'on attend un COMMAND_ACK (ou l'inverse) :
  // jamais un match, même si les identifiants "coïncidaient" par hasard.
  PendingAck pending{.kind = PendingKind::Command, .command = MAV_CMD_DO_SET_MODE};

  mavlink_message_t msg{};
  mavlink_msg_param_value_pack(1, 1, &msg, "ARMING_CHECK", 1.0f, MAV_PARAM_TYPE_REAL32,
                               1, 0);

  EXPECT_FALSE(ackMatchesPending(msg, pending));
}

TEST(AckMatchTest, NoPendingCommandNeverMatches) {
  PendingAck pending{.kind = PendingKind::None};

  mavlink_message_t msg{};
  mavlink_msg_command_ack_pack(1, 1, &msg, MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED,
                               255, 0, 0, 0);

  EXPECT_FALSE(ackMatchesPending(msg, pending));
}

// ─── AckFifo ─────────────────────────────────────────────────────────────

TEST(AckFifoTest, EmptyPopReturnsNullopt) {
  AckFifo fifo;
  EXPECT_FALSE(fifo.pop().has_value());
}

TEST(AckFifoTest, PushThenPopPreservesOrder) {
  AckFifo fifo;
  mavlink_message_t a{}, b{};
  mavlink_msg_command_ack_pack(1, 1, &a, 1, MAV_RESULT_ACCEPTED, 255, 0, 0, 0);
  mavlink_msg_command_ack_pack(1, 1, &b, 2, MAV_RESULT_ACCEPTED, 255, 0, 0, 0);

  fifo.push(a);
  fifo.push(b);

  auto first = fifo.pop();
  auto second = fifo.pop();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  mavlink_command_ack_t d1{}, d2{};
  mavlink_msg_command_ack_decode(&*first, &d1);
  mavlink_msg_command_ack_decode(&*second, &d2);
  EXPECT_EQ(d1.command, 1u);
  EXPECT_EQ(d2.command, 2u);

  EXPECT_FALSE(fifo.pop().has_value());
}

TEST(AckFifoTest, OverwritesOldestWhenFull) {
  AckFifo fifo;
  // kCapacity == 8 : on en pousse 10, les 2 plus anciens (command=0,1)
  // doivent avoir été écrasés.
  for (uint16_t i = 0; i < 10; ++i) {
    mavlink_message_t msg{};
    mavlink_msg_command_ack_pack(1, 1, &msg, i, MAV_RESULT_ACCEPTED, 255, 0, 0, 0);
    fifo.push(msg);
  }

  auto first = fifo.pop();
  ASSERT_TRUE(first.has_value());
  mavlink_command_ack_t d{};
  mavlink_msg_command_ack_decode(&*first, &d);
  EXPECT_EQ(d.command, 2u); // 0 et 1 écrasés, le plus ancien restant est 2
}

// ─── SharedFCStatus ──────────────────────────────────────────────────────

TEST(SharedFCStatusTest, AttitudeRoundTrip) {
  SharedFCStatus fc{};
  SharedFCStatusHandler h(ComponentID::MavlinkInterface, Us(20000), fc);

  h.setAttitude(Attitude{Quat{0.7071f, 0.0f, 0.0f, 0.7071f}});
  auto got = h.getAttitude();
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(got->q.w, 0.7071f);
  EXPECT_FLOAT_EQ(got->q.z, 0.7071f);
}

TEST(SharedFCStatusTest, LocalPosVelRoundTrip) {
  SharedFCStatus fc{};
  SharedFCStatusHandler h(ComponentID::MavlinkInterface, Us(20000), fc);

  h.setLocalPosVel(Position{Vec3{1.0f, 2.0f, 3.0f}}, Velocity{Vec3{0.1f, 0.2f, 0.3f}});
  auto got = h.getLocalPosVel();
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(got->pos.ned.x, 1.0f);
  EXPECT_FLOAT_EQ(got->pos.ned.z, 3.0f);
  EXPECT_FLOAT_EQ(got->vel.ned.y, 0.2f);
}

TEST(SharedFCStatusTest, BatteryRoundTrip) {
  SharedFCStatus fc{};
  SharedFCStatusHandler h(ComponentID::MavlinkInterface, Us(20000), fc);

  h.setBattery(FCBatteryStatus{.voltage = Volts{12.6f}, .current = Amps{5.5f}, .remainingPct = 80});
  auto got = h.getBattery();
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(got->voltage.v, 12.6f);
  EXPECT_EQ(got->remainingPct, 80);
}

TEST(SharedFCStatusTest, EscStatusRoundTrip) {
  SharedFCStatus fc{};
  SharedFCStatusHandler h(ComponentID::MavlinkInterface, Us(20000), fc);

  FCEscStatus esc{};
  for (size_t i = 0; i < 4; ++i)
    esc.rpmValue[i] = static_cast<float>(1000 * (i + 1));
  h.setEscStatus(esc);

  auto got = h.getEscStatus();
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(got->rpmValue[3], 4000.0f);
}

TEST(SharedFCStatusTest, PressureRoundTrip) {
  SharedFCStatus fc{};
  SharedFCStatusHandler h(ComponentID::MavlinkInterface, Us(20000), fc);

  h.setPressure(FCPressure{.absHpa = 1013.25f, .diffHpa = 0.5f, .temperatureC = 21.0f});
  auto got = h.getPressure();
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(got->absHpa, 1013.25f);
}

TEST(SharedFCStatusTest, LastHeartbeatRoundTrip) {
  SharedFCStatus fc{};
  SharedFCStatusHandler h(ComponentID::MavlinkInterface, Us(20000), fc);

  auto now = Clock::now();
  h.setLastHeartbeat(now);
  auto got = h.getLastHeartbeat();
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(*got, now);
}

TEST(SharedFCStatusTest, CorruptionIsDetected) {
  SharedFCStatus fc{};
  SharedFCStatusHandler h(ComponentID::MavlinkInterface, Us(20000), fc);

  h.setAttitude(Attitude{});
  fc.data.battery.remainingPct = 250; // corruption directe, hors API

  auto got = h.getAttitude();
  EXPECT_FALSE(got.has_value());
  EXPECT_EQ(h.consumeError(), shmError::corrupt);
}

// ─── SharedSFMem ─────────────────────────────────────────────────────────

TEST(SharedSFMemTest, GpsMagRoundTrip) {
  SharedSFMem sf{};
  SharedSFMemHandler h(ComponentID::SensorFusion, Us(20000), sf);

  GpsMagSample sample{.ts = Clock::now(),
                      .latitude = 44.1,
                      .longitude = 0.7,
                      .altMsl = 120.0f,
                      .fixType = 3,
                      .satellitesVisible = 12};
  h.setGpsMag(sample);

  auto got = h.getGpsMag();
  ASSERT_TRUE(got.has_value());
  EXPECT_DOUBLE_EQ(got->latitude, 44.1);
  EXPECT_EQ(got->fixType, 3);
  EXPECT_EQ(got->satellitesVisible, 12);
}

TEST(SharedSFMemTest, LidarMapRoundTrip) {
  SharedSFMem sf{};
  SharedSFMemHandler h(ComponentID::SensorFusion, Us(20000), sf);

  LidarMap map{};
  map.ts = Clock::now();
  map.distanceMm[90] = 1500;
  map.intensity[90] = 200;
  h.setLidarMap(map);

  auto got = h.getLidarMap();
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->distanceMm[90], 1500);
  EXPECT_EQ(got->distanceMm[0], 0); // sentinelle "pas de mesure"
}

TEST(SharedSFMemTest, DynamicStateRoundTrip) {
  SharedSFMem sf{};
  SharedSFMemHandler h(ComponentID::SensorFusion, Us(20000), sf);

  DynamicState state{.ts = Clock::now(),
                     .posNed = Position{Vec3{1.0f, 0.0f, -5.0f}},
                     .velNed = Velocity{Vec3{2.0f, 0.0f, 0.0f}},
                     .attitude = Attitude{}};
  h.setDynamicState(state);

  auto got = h.getDynamicState();
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(got->posNed.ned.z, -5.0f);
}

// ─── SharedNavMem ────────────────────────────────────────────────────────

TEST(SharedNavMemTest, CommandRoundTrip) {
  SharedNavMem nav{};
  SharedNavMemHandler h(ComponentID::Navigation, Us(20000), nav);

  NavCommand cmd{.ts = Clock::now(),
                .velNed = Velocity{Vec3{1.5f, 0.0f, 0.0f}},
                .headingCmd = Radians{1.0f},
                .altitudeCmd = Meters{-10.0f}};
  h.setCommand(cmd);

  auto got = h.getCommand();
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(got->velNed.ned.x, 1.5f);
  EXPECT_FLOAT_EQ(got->headingCmd.v, 1.0f);
  EXPECT_FLOAT_EQ(got->altitudeCmd.v, -10.0f);
}

TEST(SharedNavMemTest, CorruptionIsDetected) {
  SharedNavMem nav{};
  SharedNavMemHandler h(ComponentID::Navigation, Us(20000), nav);

  h.setCommand(NavCommand{});
  nav.data.cmd.headingCmd = Radians{999.0f}; // corruption directe, hors API

  auto got = h.getCommand();
  EXPECT_FALSE(got.has_value());
  EXPECT_EQ(h.consumeError(), shmError::corrupt);
}

// ─── mavlink_parse_char() : ce sur quoi TRx s'appuie ────────────────────

TEST(MavlinkParseTest, RoundTripSingleMessage) {
  mavlink_message_t packed{};
  mavlink_msg_heartbeat_pack(1, 1, &packed, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                             MAV_STATE_ACTIVE);

  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
  uint16_t len = mavlink_msg_to_send_buffer(buf.data(), &packed);

  mavlink_message_t parsed{};
  mavlink_status_t status{};
  bool gotMessage = false;
  for (uint16_t i = 0; i < len; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &parsed, &status))
      gotMessage = true;
  }

  ASSERT_TRUE(gotMessage);
  EXPECT_EQ(parsed.msgid, static_cast<uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
}

TEST(MavlinkParseTest, TruncatedMessageIsNotYetComplete) {
  mavlink_message_t packed{};
  mavlink_msg_heartbeat_pack(1, 1, &packed, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                             MAV_STATE_ACTIVE);

  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
  uint16_t len = mavlink_msg_to_send_buffer(buf.data(), &packed);

  mavlink_message_t parsed{};
  mavlink_status_t status{};
  bool gotMessage = false;
  // On ne fournit que la première moitié des octets.
  for (uint16_t i = 0; i < len / 2; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &parsed, &status))
      gotMessage = true;
  }
  EXPECT_FALSE(gotMessage);

  // Le reste arrive ensuite (même canal = même état de reconstruction) :
  // le message doit maintenant se compléter, exactement le scénario que
  // TRx doit gérer sur un flux UART découpé arbitrairement par le noyau.
  for (uint16_t i = len / 2; i < len; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &parsed, &status))
      gotMessage = true;
  }
  EXPECT_TRUE(gotMessage);
}

TEST(MavlinkParseTest, ConcatenatedMessagesBothParsed) {
  mavlink_message_t hb{};
  mavlink_msg_heartbeat_pack(1, 1, &hb, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                             MAV_STATE_ACTIVE);
  mavlink_message_t ack{};
  mavlink_msg_command_ack_pack(1, 1, &ack, 42, MAV_RESULT_ACCEPTED, 255, 0, 0, 0);

  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf1{}, buf2{};
  uint16_t len1 = mavlink_msg_to_send_buffer(buf1.data(), &hb);
  uint16_t len2 = mavlink_msg_to_send_buffer(buf2.data(), &ack);

  std::vector<uint8_t> combined(buf1.begin(), buf1.begin() + len1);
  combined.insert(combined.end(), buf2.begin(), buf2.begin() + len2);

  mavlink_message_t parsed{};
  mavlink_status_t status{};
  std::vector<uint32_t> receivedIds;
  for (uint8_t byte : combined) {
    if (mavlink_parse_char(MAVLINK_COMM_3, byte, &parsed, &status))
      receivedIds.push_back(parsed.msgid);
  }

  ASSERT_EQ(receivedIds.size(), 2u);
  EXPECT_EQ(receivedIds[0], static_cast<uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
  EXPECT_EQ(receivedIds[1], static_cast<uint32_t>(MAVLINK_MSG_ID_COMMAND_ACK));
}

TEST(MavlinkParseTest, CorruptedByteStreamIsIgnoredNotCrashed) {
  // Un flux de bruit pur ne doit jamais faire "réussir" un parse, ni
  // planter — mavlink_parse_char() doit rester dans un état récupérable.
  mavlink_message_t parsed{};
  mavlink_status_t status{};
  std::array<uint8_t, 64> noise{};
  for (size_t i = 0; i < noise.size(); ++i)
    noise[i] = static_cast<uint8_t>(i * 37 + 11); // motif arbitraire, pas de STX valide

  bool gotMessage = false;
  for (uint8_t byte : noise) {
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &parsed, &status))
      gotMessage = true;
  }
  EXPECT_FALSE(gotMessage);

  // Le canal doit être toujours utilisable ensuite pour un vrai message.
  // Envoyé deux fois : le bruit peut avoir laissé le parseur en plein
  // milieu d'une trame bogus qui "consomme" le premier message avant de
  // resynchroniser — ce n'est pas un bug, juste un désalignement
  // temporaire. On vérifie la récupération, pas un délai de resync précis.
  mavlink_message_t hb{};
  mavlink_msg_heartbeat_pack(1, 1, &hb, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                             MAV_STATE_ACTIVE);
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
  uint16_t len = mavlink_msg_to_send_buffer(buf.data(), &hb);

  // Le bruit peut avoir laissé le parseur croire qu'une trame v2 de
  // longueur max (~280 octets) est en cours — il faut dépasser cette
  // taille pour garantir la resync, pas juste un ou deux messages.
  bool recovered = false;
  for (int rep = 0; rep < 20; ++rep) {
    for (uint16_t i = 0; i < len; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &parsed, &status))
        recovered = true;
    }
  }
  EXPECT_TRUE(recovered);
}
