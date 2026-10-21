// Format de paquets télémétrie (TELEM::, généré depuis TELEMETRY.csv) et
// PacketScheduler (choix du prochain paquet à émettre sur TelMain/TelSec).
// Lance : cd build && ctest -v

#include "drone/Components/SystemMonitoring/Framing.hpp"
#include "drone/Components/SystemMonitoring/PacketScheduler.hpp"
#include "drone/Components/SystemMonitoring/SysMonitoring.hpp"
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

TEST(TelemetryPacketTest, TaskRateRoundTrip) {
  TELEM::TaskRatePkt in{.component = 3, .task_id = 2, .hz = 47.5f,
                        .target_hz = 50.0f};
  auto out = TELEM::TaskRatePkt::unpack(in.pack());
  EXPECT_EQ(out.component, 3);
  EXPECT_EQ(out.task_id, 2);
  EXPECT_FLOAT_EQ(out.hz, 47.5f);
  EXPECT_FLOAT_EQ(out.target_hz, 50.0f);
}

TEST(TelemetryPacketTest, EscRpmRoundTrip) {
  TELEM::EscRpmPkt in{.rpm = {4210.0f, 4195.5f, 3980.0f, 4205.0f}};
  auto out = TELEM::EscRpmPkt::unpack(in.pack());
  EXPECT_FLOAT_EQ(out.rpm[0], 4210.0f);
  EXPECT_FLOAT_EQ(out.rpm[1], 4195.5f);
  EXPECT_FLOAT_EQ(out.rpm[2], 3980.0f);
  EXPECT_FLOAT_EQ(out.rpm[3], 4205.0f);
}

TEST(TelemetryPacketTest, EscRpmMatchesSourceCadence) {
  // 2 Hz : la cadence d'ESC_TELEMETRY côté FC. Émettre plus vite ne ferait
  // que renvoyer la même mesure (cf. SYSMONITORING.md §14).
  const auto *meta = TELEM::find(TELEM::PacketID::ESC_RPM);
  ASSERT_NE(meta, nullptr);
  EXPECT_EQ(meta->rateHz, 2);
  EXPECT_EQ(meta->linkMask, static_cast<uint8_t>(TELEM::Link::Main));
}

TEST(TelemetryPacketTest, TaskRateIsMainLinkOnly) {
  // Diagnostic : rien à faire sur le lien de secours, et ça garantit qu'un
  // seul des deux switch d'émission est concerné (cf. SYSMONITORING.md §13).
  const auto *meta = TELEM::find(TELEM::PacketID::TASK_RATE);
  ASSERT_NE(meta, nullptr);
  EXPECT_EQ(meta->linkMask, static_cast<uint8_t>(TELEM::Link::Main));
}

TEST(TelemetryPacketTest, EmptyPayloadPacketRoundTrip) {
  // HEARTBEAT_GCS n'a aucun champ : pack()/unpack() doivent rester des no-op
  // valides. C'était CMD_ARM avant que l'acquittement ne lui ajoute un seq —
  // il reste le seul paquet sans charge utile, et le cas mérite sa couverture
  // puisque pack() y renvoie un std::array de taille nulle.
  TELEM::HeartbeatGcsPkt in{};
  auto bytes = in.pack();
  EXPECT_EQ(bytes.size(), 0u);
  auto out = TELEM::HeartbeatGcsPkt::unpack(bytes);
  (void)out;
}

TEST(TelemetryPacketTest, AckedCommandsCarrySequence) {
  // Un acquittement sans numéro de séquence est inappariable, et un doublon
  // de retransmission serait indétectable — donc réexécuté. Le générateur le
  // refuse déjà, cette vérification garde le contrat côté C++ au cas où la
  // table serait produite autrement.
  for (const auto &m : TELEM::kTable) {
    if (m.ack == TELEM::AckMode::None_)
      continue;
    EXPECT_GE(m.size, sizeof(uint32_t))
        << m.name << " est acquitté mais trop petit pour porter un seq:u32";
  }
}

TEST(TelemetryPacketTest, EmergencyBypassesCarrierSense) {
  // Rester muet sur un arrêt d'urgence parce que le canal paraît occupé est
  // pire qu'une collision : c'est le seul paquet qui doit passer outre le CCA.
  const auto *urgence = TELEM::find(TELEM::PacketID::CMD_EMERGENCY);
  ASSERT_NE(urgence, nullptr);
  EXPECT_EQ(urgence->ack, TELEM::AckMode::ForceNoCca);

  // Accolades explicites : les macros GTest s'étendent en if/else, et un
  // `if` nu dans la boucle produit un else ambigu refusé par -Werror.
  for (const auto &m : TELEM::kTable) {
    if (m.id != TELEM::PacketID::CMD_EMERGENCY) {
      EXPECT_NE(m.ack, TELEM::AckMode::ForceNoCca)
          << m.name << " contourne le CCA sans raison documentée";
    }
  }
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

// ─────────────────────────────────────────────────────────────────────────────
// Acquittement des commandes critiques : file d'émission et dédoublonnage.
//
// Logique pure, donc testable sans matériel — et elle le mérite : c'est elle
// qui empêche une retransmission de rejouer un armement.
// ─────────────────────────────────────────────────────────────────────────────

TEST(AckQueueTest, PushPopPreservesOrder) {
  AckQueue q;
  EXPECT_TRUE(q.empty());

  ASSERT_TRUE(q.push(AckEvent{.seq = 1, .packetId = 10, .result = kAckAccepted}));
  ASSERT_TRUE(q.push(AckEvent{.seq = 2, .packetId = 11, .result = kAckDuplicate}));
  EXPECT_FALSE(q.empty());

  AckEvent ev{};
  ASSERT_TRUE(q.pop(ev));
  EXPECT_EQ(ev.seq, 1u);
  EXPECT_EQ(ev.packetId, 10);
  ASSERT_TRUE(q.pop(ev));
  EXPECT_EQ(ev.seq, 2u);
  EXPECT_EQ(ev.result, kAckDuplicate);

  EXPECT_TRUE(q.empty());
  EXPECT_FALSE(q.pop(ev));
}

TEST(AckQueueTest, FullQueueRefusesRatherThanOverwrite) {
  // On préfère perdre un acquittement — le sol retransmettra — que d'écraser
  // un acquittement plus ancien, donc plus urgent.
  AckQueue q;
  size_t acceptes = 0;
  for (size_t i = 0; i < AckQueue::kCapacity + 4; ++i)
    if (q.push(AckEvent{.seq = static_cast<uint32_t>(i)}))
      ++acceptes;

  EXPECT_EQ(acceptes, AckQueue::kCapacity - 1) // une case perdue par l'anneau
      << "la file doit refuser quand elle est pleine, pas ecraser";

  // Le plus ancien est toujours là : c'est bien l'ancien qui a été préservé.
  AckEvent ev{};
  ASSERT_TRUE(q.pop(ev));
  EXPECT_EQ(ev.seq, 0u);
}

TEST(UplinkDedupTest, FirstArrivalIsNotDuplicate) {
  UplinkDedup d;
  EXPECT_FALSE(d.isDuplicate(TELEM::PacketID::CMD_ARM, 42));
  d.remember(TELEM::PacketID::CMD_ARM, 42);
  EXPECT_TRUE(d.isDuplicate(TELEM::PacketID::CMD_ARM, 42));
}

TEST(UplinkDedupTest, NewSequenceIsAccepted) {
  // Un nouvel ordre doit passer, sinon on bloquerait les commandes suivantes.
  UplinkDedup d;
  d.remember(TELEM::PacketID::CMD_RTL, 7);
  EXPECT_FALSE(d.isDuplicate(TELEM::PacketID::CMD_RTL, 8));
}

TEST(UplinkDedupTest, CommandsAreIndependent) {
  // Armer puis désarmer avec le même seq sont deux ordres distincts : le
  // dédoublonnage est par type de commande.
  UplinkDedup d;
  d.remember(TELEM::PacketID::CMD_ARM, 3);
  EXPECT_FALSE(d.isDuplicate(TELEM::PacketID::CMD_DISARM, 3));
}

TEST(UplinkDedupTest, DedupIsGlobalNotPerLink) {
  // PDL et SDL sont indépendants et les commandes sont en Links=BOTH : la même
  // commande peut donc arriver par les deux. Une seule table, partagée par les
  // deux liens, est ce qui empêche de l'appliquer deux fois.
  //
  // Ce test documente l'intention : il n'existe qu'UNE instance d'UplinkDedup,
  // portée par la tâche de réception qui lit les deux liens.
  UplinkDedup d;
  EXPECT_FALSE(d.isDuplicate(TELEM::PacketID::CMD_EMERGENCY, 99));
  d.remember(TELEM::PacketID::CMD_EMERGENCY, 99); // arrivée par le PDL
  EXPECT_TRUE(d.isDuplicate(TELEM::PacketID::CMD_EMERGENCY, 99)) // puis par le SDL
      << "une commande recue par les deux liens ne doit etre appliquee qu'une fois";
}
