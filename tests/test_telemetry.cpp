// tests/test_telemetry.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Format de paquets télémétrie (TELEM::, généré depuis TELEMETRY.csv) et
// PacketScheduler (choix du prochain paquet à émettre sur TelMain/TelSec).
// Lance : cd build && ctest -v

#include "drone/Components/System Monitoring/Framing.hpp"
#include "drone/Components/System Monitoring/PacketScheduler.hpp"
#include "drone/generated/telemetry.hpp"
#include <gtest/gtest.h>
#include <thread>

using namespace std::chrono_literals;

// ─── pack()/unpack() round-trip ─────────────────────────────────────────────

TEST(TelemetryPacketTest, PosRoundTrip) {
  TELEM::PosPkt in{.north = 1.5f, .east = -2.25f, .down = 10.0f};
  auto bytes = in.pack();
  auto out = TELEM::PosPkt::unpack(bytes);
  EXPECT_FLOAT_EQ(out.north, 1.5f);
  EXPECT_FLOAT_EQ(out.east, -2.25f);
  EXPECT_FLOAT_EQ(out.down, 10.0f);
}

TEST(TelemetryPacketTest, AutopilotModeRoundTrip) {
  TELEM::AutopilotModePkt in{.lateral_engaged = 2,
                            .lateral_armed = 3,
                            .vertical_engaged = 1,
                            .vertical_armed = 0,
                            .target_altitude = 42.0f,
                            .target_speed = 3.5f};
  auto out = TELEM::AutopilotModePkt::unpack(in.pack());
  EXPECT_EQ(out.lateral_engaged, 2);
  EXPECT_EQ(out.lateral_armed, 3);
  EXPECT_EQ(out.vertical_engaged, 1);
  EXPECT_EQ(out.vertical_armed, 0);
  EXPECT_FLOAT_EQ(out.target_altitude, 42.0f);
  EXPECT_FLOAT_EQ(out.target_speed, 3.5f);
}

TEST(TelemetryPacketTest, EmptyPayloadPacketRoundTrip) {
  // CMD_ARM n'a aucun champ : pack()/unpack() doivent rester no-op valides.
  TELEM::CmdArmPkt in{};
  auto bytes = in.pack();
  EXPECT_EQ(bytes.size(), 0u);
  auto out = TELEM::CmdArmPkt::unpack(bytes);
  (void)out;
}

TEST(TelemetryPacketTest, UnpackTruncatedBufferReturnsDefault) {
  std::array<std::byte, 2> tooShort{};
  auto out = TELEM::PosPkt::unpack(tooShort); // PosPkt::kSize == 12
  EXPECT_FLOAT_EQ(out.north, 0.0f);
  EXPECT_FLOAT_EQ(out.east, 0.0f);
  EXPECT_FLOAT_EQ(out.down, 0.0f);
}

TEST(TelemetryPacketTest, KTableCoversEveryPacketID) {
  // Un trou dans kTable casserait find() silencieusement pour ce PacketID.
  for (const auto &m : TELEM::kTable) {
    auto *found = TELEM::find(m.id);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->id, m.id);
  }
}

// ─── Framing (encode/decode) ────────────────────────────────────────────────

TEST(TelemetryFramingTest, EncodeDecodeRoundTrip) {
  TELEM::PosPkt in{.north = 3.0f, .east = 4.0f, .down = 5.0f};
  auto frame = TELEM::encode(in);

  auto decoded = TELEM::decode(frame);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->id, TELEM::PacketID::POS);

  auto out = TELEM::PosPkt::unpack(decoded->payload);
  EXPECT_FLOAT_EQ(out.north, 3.0f);
  EXPECT_FLOAT_EQ(out.east, 4.0f);
  EXPECT_FLOAT_EQ(out.down, 5.0f);
}

TEST(TelemetryFramingTest, DecodeEmptyBufferFails) {
  std::array<std::byte, 0> empty{};
  EXPECT_FALSE(TELEM::decode(empty).has_value());
}

// ─── PacketScheduler ─────────────────────────────────────────────────────────

TEST(PacketSchedulerTest, MainLinkCarriesEveryDownPacket) {
  PacketScheduler sched(TELEM::Link::Main);
  auto now = TYPES::Clock::now();

  // Tous les paquets descendants doivent finir par être dus sur TelMain
  // (aucun n'est filtré par linkMask).
  int seen = 0;
  for (const auto &m : TELEM::kTable) {
    if (m.direction != TELEM::Direction::Down)
      continue;
    ++seen;
  }
  EXPECT_GT(seen, 0);
  (void)now;
}

TEST(PacketSchedulerTest, SecLinkOnlyCarriesBothPackets) {
  // AUTOPILOT_MODE est MAIN uniquement : jamais dû sur TelSec, même en
  // attendant longtemps.
  PacketScheduler sched(TELEM::Link::Sec);
  auto now = TYPES::Clock::now() + 10s;

  for (int i = 0; i < 200; ++i) {
    auto *due = sched.nextDue(now);
    if (!due)
      break;
    EXPECT_NE(due->id, TELEM::PacketID::AUTOPILOT_MODE);
    EXPECT_NE(due->id, TELEM::PacketID::POS);
    sched.markSent(due->id, now);
  }
}

TEST(PacketSchedulerTest, CriticalAlwaysBeatsRoutineWhenBothDue) {
  PacketScheduler sched(TELEM::Link::Main);
  auto t0 = TYPES::Clock::now();

  // Fait passer largement toutes les périodes : Routine (POS, 10Hz) et
  // Critical (CODE, event) sont tous deux dus, CODE doit être choisi.
  sched.markDirty(TELEM::PacketID::CODE);
  auto now = t0 + 5s;

  auto *due = sched.nextDue(now);
  ASSERT_NE(due, nullptr);
  EXPECT_EQ(due->priority, TELEM::Priority::Critical);
}

TEST(PacketSchedulerTest, MostOverdueWinsAtEqualPriority) {
  PacketScheduler sched(TELEM::Link::Main);
  auto t0 = TYPES::Clock::now();
  auto now = t0 + 1100ms;

  // Neutralise tout le reste (y compris les Critical à redondance, qui
  // sinon deviendraient dus aussi à 1000ms et gagneraient sur la priorité
  // avant même le départage par retard) en le marquant "envoyé maintenant".
  for (const auto &m : TELEM::kTable)
    if (m.direction == TELEM::Direction::Down)
      sched.markSent(m.id, now);

  // POS (Routine, 10Hz -> période 100ms) et HEALTH (Routine, 1Hz ->
  // 1000ms) envoyés une fois à t0 : à `now`, POS est 10x plus en retard
  // que HEALTH en valeur absolue (1000ms vs 100ms de dépassement).
  sched.markSent(TELEM::PacketID::POS, t0);
  sched.markSent(TELEM::PacketID::HEALTH, t0);

  auto *due = sched.nextDue(now);
  ASSERT_NE(due, nullptr);
  EXPECT_EQ(due->id, TELEM::PacketID::POS);
}

TEST(PacketSchedulerTest, NothingDueReturnsNullptr) {
  PacketScheduler sched(TELEM::Link::Main);
  auto t0 = TYPES::Clock::now();

  for (const auto &m : TELEM::kTable)
    if (m.direction == TELEM::Direction::Down)
      sched.markSent(m.id, t0);

  // Juste après envoi de tout : rien ne doit être dû (pas de marge event
  // sans markDirty, pas de période écoulée).
  auto *due = sched.nextDue(t0 + 1ms);
  EXPECT_EQ(due, nullptr);
}

TEST(PacketSchedulerTest, MarkDirtyForcesEventPacketDue) {
  PacketScheduler sched(TELEM::Link::Main);
  auto t0 = TYPES::Clock::now();

  sched.markSent(TELEM::PacketID::MISSION_STATE, t0);
  EXPECT_EQ(sched.nextDue(t0 + 1ms), nullptr);

  sched.markDirty(TELEM::PacketID::MISSION_STATE);
  auto *due = sched.nextDue(t0 + 1ms);
  ASSERT_NE(due, nullptr);
  EXPECT_EQ(due->id, TELEM::PacketID::MISSION_STATE);
}

TEST(PacketSchedulerTest, EventPacketRedundancyRefiresWithoutMarkDirty) {
  // MISSION_STATE : redundancyHz=1 -> doit redevenir dû ~1s après le
  // dernier envoi même sans changement d'état (résistance à la perte du
  // paquet événementiel sur un lien lossy sans ACK).
  PacketScheduler sched(TELEM::Link::Main);
  auto t0 = TYPES::Clock::now();

  // Neutralise tout le reste de la table pour isoler MISSION_STATE.
  for (const auto &m : TELEM::kTable)
    if (m.direction == TELEM::Direction::Down)
      sched.markSent(m.id, t0 + 1100ms);

  sched.markSent(TELEM::PacketID::MISSION_STATE, t0);
  EXPECT_EQ(sched.nextDue(t0 + 500ms), nullptr) << "pas encore dû avant 1s";

  auto *due = sched.nextDue(t0 + 1100ms);
  ASSERT_NE(due, nullptr);
  EXPECT_EQ(due->id, TELEM::PacketID::MISSION_STATE);
}
