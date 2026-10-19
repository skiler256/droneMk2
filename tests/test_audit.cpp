// Non-régression des anomalies d'AUDIT.md. Un test rouge prouve que
// l'anomalie existe ; il passe au vert quand elle est traitée, et sert alors
// de garde. AuditNN_* renvoie à l'entrée NN (numérotation stable). Les
// entrées non couvertes par un test unitaire sont listées en fin de fichier.

#include "drone/Components/Drivers/NEO-M8N.hpp"
#include "drone/Components/MissionControl/MissionControl.hpp"
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/SystemMonitoring/SharedComMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/SharedSysStateMem.hpp"
#include "drone/generated/codes.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace TYPES;

namespace {

// ─── Fabrication de trames UBX ──────────────────────────────────────────────

// Fletcher 8 bits sur class/id/len/payload (spec u-blox).
void appendUbxChecksum(std::vector<std::uint8_t> &frame) {
  std::uint8_t ckA = 0, ckB = 0;
  for (std::size_t i = 2; i < frame.size(); ++i) {
    ckA = static_cast<std::uint8_t>(ckA + frame[i]);
    ckB = static_cast<std::uint8_t>(ckB + ckA);
  }
  frame.push_back(ckA);
  frame.push_back(ckB);
}

std::vector<std::uint8_t> makeUbx(std::uint8_t cls, std::uint8_t id,
                                  std::vector<std::uint8_t> payload) {
  std::vector<std::uint8_t> frame{
      0xB5, 0x62, cls, id,
      static_cast<std::uint8_t>(payload.size() & 0xFF),
      static_cast<std::uint8_t>((payload.size() >> 8) & 0xFF)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  appendUbxChecksum(frame);
  return frame;
}

// fd_ est protected dans IDriver : une sous-classe peut l'alimenter par un
// pipe, sans toucher au code de production ni brancher d'UART.
class UbxFeeder : public NEO_M8N {
public:
  UbxFeeder() : NEO_M8N("/dev/null") {}

  // Le fd est fermé par ~IDriver(), pas ici.
  void feed(const std::vector<std::uint8_t> &bytes) {
    int fds[2];
    ASSERT_EQ(::pipe(fds), 0);
    ASSERT_EQ(::write(fds[1], bytes.data(), bytes.size()),
              static_cast<ssize_t>(bytes.size()));
    ::close(fds[1]);
    fd_ = fds[0];
  }
};

ComponenConfig makeConfig(bool firstBoot) {
  return ComponenConfig{
      .max_cold_start = 1,
      .max_cold_start_interval = Ms(60000),
      .max_hot_start = 3,
      .max_hot_start_interval = Ms(60000),
      .id = ComponentID::Navigation,
      .CompCore = 0,
      .firstBoot = firstBoot,
  };
}

} // namespace

// ─── AUDIT.md §1 — l'échelle hot/cold/DEAD doit s'engager ──────────────────────

// Un respawn doit être rapporté DEGRADED, pas NOMINAL.
TEST(AuditTest, Audit01_RespawnIsNotNominal) {
  SharedNavMem nav{};
  SharedNavMemHandler comp(ComponentID::Navigation, Us(20000), nav);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::Navigation, Us(20000));

  ComponentBase<0> base(makeConfig(/*firstBoot=*/false), comp, sysState);

  auto health = sysState.getHealth(ComponentID::Navigation);
  ASSERT_TRUE(health.has_value());
  EXPECT_NE(*health, ComponentHealth::NOMINAL)
      << "un composant redémarré est rapporté sain : l'escalade "
         "DEGRADED/SICK/DEAD ne peut jamais s'engager";
}

// Sans trace écrite, l'escalade ne peut pas compter les respawns.
TEST(AuditTest, Audit01_RespawnRecordsHistory) {
  SharedNavMem nav{};
  SharedNavMemHandler comp(ComponentID::Navigation, Us(20000), nav);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::Navigation, Us(20000));

  { ComponentBase<0> base(makeConfig(/*firstBoot=*/false), comp, sysState); }

  auto hot = comp.getHotStartTs(0);
  ASSERT_TRUE(hot.has_value()) << "lecture shm échouée";
  EXPECT_TRUE(hot->has_value())
      << "aucun hot-start enregistré au respawn : l'escalade reste aveugle";
}

// Garde-fou : MissionControl.cpp:286 exige STRICTEMENT NOMINAL, sinon MCA005
// se lèverait au premier segment d'un système neuf.
TEST(AuditTest, Audit01_FirstBootStaysNominal) {
  SharedNavMem nav{};
  SharedNavMemHandler comp(ComponentID::Navigation, Us(20000), nav);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::Navigation, Us(20000));

  ComponentBase<0> base(makeConfig(/*firstBoot=*/true), comp, sysState);

  auto health = sysState.getHealth(ComponentID::Navigation);
  ASSERT_TRUE(health.has_value());
  EXPECT_EQ(*health, ComponentHealth::NOMINAL)
      << "premier démarrage non NOMINAL : navHealthy serait faux en "
         "permanence et MCA005 se lèverait à chaque mission";
}

// max_hot_start=3 : 3 respawns tolérés, le 4e escalade (cf. §4.3).
TEST(AuditTest, Audit01_SuccessiveRespawnsEscalate) {
  SharedNavMem nav{};
  SharedNavMemHandler comp(ComponentID::Navigation, Us(20000), nav);
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::Navigation, Us(20000));

  { ComponentBase<0> b(makeConfig(/*firstBoot=*/true), comp, sysState); }
  EXPECT_EQ(sysState.getHealth(ComponentID::Navigation).value(),
            ComponentHealth::NOMINAL);

  for (int i = 0; i < 3; ++i) {
    ComponentBase<0> b(makeConfig(/*firstBoot=*/false), comp, sysState);
    EXPECT_EQ(sysState.getHealth(ComponentID::Navigation).value(),
              ComponentHealth::DEGRADED)
        << "respawn " << i + 1 << " : encore dans la tolérance";
  }

  { ComponentBase<0> b(makeConfig(/*firstBoot=*/false), comp, sysState); }
  EXPECT_EQ(sysState.getHealth(ComponentID::Navigation).value(),
            ComponentHealth::SICK)
      << "4e respawn : max_hot_start=3 atteint, doit escalader en cold-start";

  auto cold = comp.getColdStartTs(0);
  ASSERT_TRUE(cold.has_value()) << "lecture shm échouée";
  EXPECT_TRUE(cold->has_value())
      << "l'escalade en cold-start doit laisser sa propre trace, sans quoi "
         "le passage à DEAD ne pourra jamais se produire";
}

// ─── AUDIT.md §4 — deux raiseCode() concurrents ne doivent pas en perdre un ────

// getData() puis setData() = deux verrous, donc read-modify-write non atomique.
TEST(AuditTest, Audit04_ConcurrentRaiseCodeKeepsBothCodes) {
  SharedSysStateMem sys{};
  SharedSysStateMemHandler sysState(sys, ComponentID::MissionControl,
                                    Us(50000));

  constexpr int kIterations = 3000;
  int lost = 0;

  for (int i = 0; i < kIterations; ++i) {
    sysState.clearCode(CODES::MCA005);
    sysState.clearCode(CODES::MCA007);

    std::atomic<bool> go{false};
    std::thread t1([&] {
      while (!go.load(std::memory_order_acquire)) {
      }
      sysState.raiseCode(CODES::MCA005);
    });
    std::thread t2([&] {
      while (!go.load(std::memory_order_acquire)) {
      }
      sysState.raiseCode(CODES::MCA007);
    });
    go.store(true, std::memory_order_release);
    t1.join();
    t2.join();

    auto slots = sysState.getMajorCodes(ComponentID::MissionControl);
    ASSERT_TRUE(slots.has_value());
    bool has5 = false, has7 = false;
    for (const auto &s : *slots) {
      if (s.code == CODES::MCA005.pack())
        has5 = true;
      if (s.code == CODES::MCA007.pack())
        has7 = true;
    }
    if (!has5 || !has7)
      ++lost;
  }

  EXPECT_EQ(lost, 0) << lost << " perte(s) sur " << kIterations
                     << " itérations : lost update entre getData() et "
                        "setData(), aucun mutex local sur le handler";
}

// ─── AUDIT.md §10 / §15 — NAV-SVINFO doit borner le nombre de satellites ───────

// Trame courte annonçant 255 satellites : sous ASan ce test signale la
// lecture hors bornes, sinon l'assertion sur la borne suffit.
TEST(AuditTest, Audit10_SvinfoSatelliteCountIsBounded) {
  std::vector<std::uint8_t> svinfo(20, 0);
  svinfo[4] = 255; // numCh incohérent avec une charge utile de 20 octets

  // NAV-STATUS valide à la suite : un rejet correct ne décode aucun paquet.
  std::vector<std::uint8_t> status(16, 0);
  status[4] = 0x03;

  auto stream = makeUbx(0x01, 0x30, svinfo);
  auto second = makeUbx(0x01, 0x03, status);
  stream.insert(stream.end(), second.begin(), second.end());

  UbxFeeder gps;
  gps.feed(stream);
  auto result = gps.update();

  ASSERT_TRUE(result.has_value());
  EXPECT_LE(result->satDetailCount, GPSData::kMaxSatDetail)
      << int(result->satDetailCount)
      << " satellites décodés depuis une trame de 20 octets : numCh n'est pas "
         "validé, lecture hors bornes de payloadBuffer_[1024]";
  EXPECT_EQ(result->satDetailCount, 0)
      << "une trame dont numCh contredit la taille annoncée doit être rejetée "
         "en entier";
}

// Une trame NAV-SVINFO cohérente (8 + 12*N) doit, elle, être acceptée.
TEST(AuditTest, Audit10_WellFormedSvinfoIsAccepted) {
  constexpr std::uint8_t kSats = 3;
  std::vector<std::uint8_t> payload(8 + 12 * kSats, 0);
  payload[4] = kSats;

  UbxFeeder gps;
  gps.feed(makeUbx(0x01, 0x30, payload));
  auto result = gps.update();

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->satDetailCount, kSats);
  EXPECT_EQ(result->satellitesTracked, kSats);
}

// Le détail est plafonné à 16, le compte de canaux suivis ne doit pas l'être.
TEST(AuditTest, Audit10_TrackedCountIsNotCappedByDetailLimit) {
  constexpr std::uint8_t kSats = 24;
  std::vector<std::uint8_t> payload(8 + 12 * kSats, 0);
  payload[4] = kSats;

  UbxFeeder gps;
  gps.feed(makeUbx(0x01, 0x30, payload));
  auto result = gps.update();

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->satellitesTracked, kSats)
      << "compte de satellites tronqué au plafond du détail";
  EXPECT_EQ(result->satDetailCount, GPSData::kMaxSatDetail);
}

// ─── AUDIT.md §11 — une trame au checksum invalide doit être rejetée ───────────

// Trame valide par ailleurs, seuls les deux octets de checksum sont corrompus.
TEST(AuditTest, Audit11_FrameWithBadChecksumIsRejected) {
  std::vector<std::uint8_t> payload(16, 0);
  payload[4] = 0x03; // fix 3D

  auto frame = makeUbx(0x01, 0x03, payload);
  frame[frame.size() - 2] ^= 0xFF; // CK_A corrompu
  frame[frame.size() - 1] ^= 0xFF; // CK_B corrompu

  UbxFeeder gps;
  gps.feed(frame);
  auto result = gps.update();

  EXPECT_FALSE(result.has_value())
      << "trame au checksum invalide acceptée : le checksum UBX n'est jamais "
         "calculé ni comparé";
}

// Inoffensif sauf si CK_B vaut 0xB5 ; le dernier octet du payload le décalant
// d'autant, une recherche sur cet octet trouve toujours ce cas.
TEST(AuditTest, Audit11_SecondChecksumByteDoesNotEatNextFrame) {
  std::vector<std::uint8_t> status(16, 0);
  status[4] = 0x03; // fix 3D, à préserver

  bool found = false;
  for (int filler = 0; filler <= 0xFF && !found; ++filler) {
    status[15] = static_cast<std::uint8_t>(filler);
    auto candidate = makeUbx(0x01, 0x03, status);
    found = (candidate.back() == 0xB5);
  }
  ASSERT_TRUE(found) << "aucun payload produisant CK_B == 0xB5";

  std::vector<std::uint8_t> dop(18, 0);
  dop[12] = 90; // hDOP = 0.90, porté par la SECONDE trame

  auto stream = makeUbx(0x01, 0x03, status);
  ASSERT_EQ(stream.back(), 0xB5);
  auto second = makeUbx(0x01, 0x04, dop);
  stream.insert(stream.end(), second.begin(), second.end());

  UbxFeeder gps;
  gps.feed(stream);
  auto result = gps.update();

  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->hdop, 0.90f)
      << "trame suivante perdue : CK_B valant 0xB5 a été pris pour un octet de "
         "synchronisation, un seul des deux octets de checksum étant consommé";
}

// ─── AUDIT.md §13 — un fix « time only » n'est pas un fix exploitable ──────────

// 0x05 = time only, aucune position — franchissait le filtre de plausibilité.
TEST(AuditTest, Audit13_TimeOnlyFixIsNotValid) {
  std::vector<std::uint8_t> payload(16, 0);
  payload[4] = 0x05; // time only fix

  UbxFeeder gps;
  gps.feed(makeUbx(0x01, 0x03, payload));
  auto result = gps.update();

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->gpsFixOk)
      << "fix « time only » (0x05) rapporté comme valide alors qu'il ne porte "
         "aucune position";
}

TEST(AuditTest, Audit13_ThreeDAndDeadReckoningFixesStayValid) {
  for (std::uint8_t fix : {std::uint8_t{0x03}, std::uint8_t{0x04}}) {
    std::vector<std::uint8_t> payload(16, 0);
    payload[4] = fix;

    UbxFeeder gps;
    gps.feed(makeUbx(0x01, 0x03, payload));
    auto result = gps.update();

    ASSERT_TRUE(result.has_value()) << "fix = " << int(fix);
    EXPECT_TRUE(result->gpsFixOk) << "fix = " << int(fix);
  }
}

// ─── AUDIT.md §14 — NAV-TIMEUTC doit respecter ses drapeaux de validité ────────

// La date alimente la semaine GPS envoyée à l'EKF3 : inexploitable si valid=0.
TEST(AuditTest, Audit14_InvalidTimeUtcIsIgnored) {
  std::vector<std::uint8_t> timeutc(20, 0);
  timeutc[12] = 0x2B; // année = 0x082B, valeur arbitraire
  timeutc[13] = 0x08;
  timeutc[14] = 13;   // mois absurde
  timeutc[19] = 0x00; // drapeaux de validité : rien n'est acquis

  // NAV-STATUS valide à la suite, sinon un rejet correct remonte NoNewData.
  std::vector<std::uint8_t> status(16, 0);
  status[4] = 0x03;

  auto stream = makeUbx(0x01, 0x21, timeutc);
  auto second = makeUbx(0x01, 0x03, status);
  stream.insert(stream.end(), second.begin(), second.end());

  UbxFeeder gps;
  gps.feed(stream);
  auto result = gps.update();

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->timeArray[0], 0)
      << "date UTC (année " << result->timeArray[0] << ", mois "
      << result->timeArray[1]
      << ") exploitée alors que les drapeaux de validité la déclarent non "
         "acquise — alimente la semaine GPS envoyée à l'EKF3";
}

// ─── AUDIT.md §17 — le refus d'une FIFO pleine doit être observable ────────────

// Précondition seule : la conséquence (TStateMachine avance quand même) vit
// dans loop(), privée et pilotée par un thread RT. Cf. AUDIT.md §17.
TEST(AuditTest, Audit17_FullCommandFifoRefusesPush) {
  SharedCmdMem cmd{};
  SharedCmdMemHandler handler(ComponentID::MissionControl, Us(20000), cmd);

  int accepted = 0;
  for (int i = 0; i < 32; ++i) {
    if (handler.pushMavlinkCommand(CODES::MCO006, 0))
      ++accepted;
  }

  EXPECT_LT(accepted, 32) << "la FIFO accepte sans fin : le refus sur file "
                             "pleine ne protège plus rien";
  EXPECT_FALSE(handler.pushMavlinkCommand(CODES::MCO006, 0))
      << "une FIFO pleine doit refuser explicitement";
}

// ─── AUDIT.md §17 — intégration : la transition est réellement bloquée ───────

namespace {

// Les Task de MissionControl sont des threads SCHED_FIFO : sans privilège RT
// elles ne démarrent pas et le scénario n'a aucun sens.
bool rtAvailable() {
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  sched_param param{};
  param.sched_priority = 1;
  pthread_attr_setschedparam(&attr, &param);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_t t;
  bool ok = pthread_create(&t, &attr, [](void *) -> void * { return nullptr; },
                           nullptr) == 0;
  if (ok)
    pthread_join(t, nullptr);
  pthread_attr_destroy(&attr);
  return ok;
}

// Tout le décor shm d'un MissionControl réel, sur la pile.
struct McFixture {
  SharedNavMem nav{};
  SharedSFMem sf{};
  SharedSysStateMem sys{};
  SharedFCStatus fc{};
  SharedCmdMem cmd{};
  SharedComMem com{};

  SharedNavMemHandler navH{ComponentID::MissionControl, Us(20000), nav};
  SharedSFMemHandler sfH{ComponentID::MissionControl, Us(20000), sf};
  SharedSysStateMemHandler sysH{sys, ComponentID::MissionControl, Us(20000)};
  SharedFCStatusHandler fcH{ComponentID::MissionControl, Us(20000), fc};
  SharedCmdMemHandler cmdH{ComponentID::MissionControl, Us(20000), cmd};
  SharedComMemHandler comH{ComponentID::MissionControl, Us(20000), com};

  ComponenConfig config{.id = ComponentID::MissionControl, .CompCore = 0};
};

bool hasMajor(SharedSysStateMemHandler &h, CODES::Code code) {
  auto slots = h.getMajorCodes(ComponentID::MissionControl);
  if (!slots)
    return false;
  for (const auto &s : *slots)
    if (s.code == code.pack())
      return true;
  return false;
}

// Plan vide => TMiSequencer pose missionComplete_, ce qui demande
// Mission -> Landing sans avoir à injecter de commande GCS.
MissionState runUntilSettled(McFixture &f, bool fillFifo) {
  // FC armé : sinon `landed` est vrai d'emblée et Landing file vers Idle,
  // ce qui masquerait le point testé.
  f.fcH.setArmed(true);

  MissionControl mc(f.config, f.sysH, f.cmdH, f.fcH, f.navH, f.sfH, f.comH);
  if (fillFifo)
    while (f.cmdH.pushMavlinkCommand(CODES::MCO006, 0)) {
    }

  mc.state_.store(MissionState::Mission);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  return mc.state_.load();
}

} // namespace

// FIFO pleine : l'ordre LAND ne peut pas partir, donc l'état NE DOIT PAS
// annoncer Landing — sinon le GCS afficherait un atterrissage qui n'a pas lieu.
TEST(AuditTest, Audit17_FullFifoBlocksLandingTransition) {
  if (!rtAvailable())
    GTEST_SKIP() << "SCHED_FIFO indisponible (pas de privilège RT)";

  McFixture f;
  auto state = runUntilSettled(f, /*fillFifo=*/true);

  EXPECT_NE(state, MissionState::Landing)
      << "transition annoncée alors que la commande LAND n'a pas pu partir";
  EXPECT_TRUE(hasMajor(f.sysH, CODES::MCA010))
      << "commande FC perdue sans MCA010 : la perte reste invisible";
}

// Contrôle positif : FIFO libre, la transition doit bien avoir lieu.
TEST(AuditTest, Audit17_FreeFifoAllowsLandingTransition) {
  if (!rtAvailable())
    GTEST_SKIP() << "SCHED_FIFO indisponible (pas de privilège RT)";

  McFixture f;
  auto state = runUntilSettled(f, /*fillFifo=*/false);

  EXPECT_EQ(state, MissionState::Landing);
  EXPECT_FALSE(hasMajor(f.sysH, CODES::MCA010));
}

// ─── Entrées d'AUDIT.md non couvertes par un test unitaire ─────────────────────
//
// §2  course thread watchdog / registerTask : data race sur std::vector —
//     relève de ThreadSanitizer (-fsanitize=thread), pas d'une assertion.
// §3  politique de reprise après un fork() échoué : exigerait d'épuiser
//     RLIMIT_NPROC, trop invasif et instable pour la suite.
// §5  erreurs launchRTThread avalées : exige de faire échouer SCHED_FIFO, et
//     surtout AUCUNE API ne permet d'observer l'échec — c'est précisément
//     l'anomalie.
// §6  CompCore renseigné mais ignoré : propriété statique du code.
// §8  dérive de codes.hpp : concerne le système de build.
// §12 couverture des drivers : ce fichier couvre le parseur UBX ; LD06 et
//     LIS3MDL restent sans test.
// §19 keepRunning_ static : observable seulement via deux instances de
//     GlobalWatchdog dans un process, or run() bloque et fork — déjà
//     contourné par le harnais de test_global_watchdog.cpp.
// §20 variables d'environnement : lues à l'initialisation statique, donc
//     figées avant l'entrée dans main() comme dans les tests.
