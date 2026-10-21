// Parseurs matériels : LD06 (LDROBOT, CRC8) et NEO-M8N (UBX).
//
// Alimentés par un pipe via une sous-classe de test — `fd_` est protected dans
// IDriver, donc le vrai update() est exercé de bout en bout sans UART ni
// modification du code de production. init() n'est jamais appelé.
//
// Les anomalies d'AUDIT.md sur le parseur UBX ont leurs propres gardes dans
// test_audit.cpp ; ce fichier couvre le fonctionnement nominal et les trames
// malformées, tronquées ou concaténées.

#include "drone/Components/Drivers/LD06.hpp"
#include "drone/Components/Drivers/NEO-M8N.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <tuple>
#include <unistd.h>
#include <vector>

namespace {

template <typename Driver> class PipeFed : public Driver {
public:
  // Le fd est fermé par ~IDriver().
  void feed(const std::vector<std::uint8_t> &bytes) {
    int fds[2];
    ASSERT_EQ(::pipe(fds), 0);
    ASSERT_EQ(::write(fds[1], bytes.data(), bytes.size()),
              static_cast<ssize_t>(bytes.size()));
    ::close(fds[1]);
    this->fd_ = fds[0];
  }
};

// ─── LD06 ───────────────────────────────────────────────────────────────────

// Table CRC8 du protocole LD06/LD19 (polynôme 0x4D), recopiée du driver : un
// test qui réutiliserait la fonction du driver ne validerait rien.
std::uint8_t ld06Crc(const std::uint8_t *d, std::size_t n) {
  static constexpr std::uint8_t kTable[256] = {
      0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
      0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
      0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
      0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
      0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
      0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
      0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
      0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
      0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
      0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
      0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
      0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
      0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
      0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
      0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
      0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
      0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
      0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
      0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
      0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
      0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
      0x7f, 0x32, 0xe5, 0xa8};
  std::uint8_t c = 0;
  for (std::size_t i = 0; i < n; ++i)
    c = kTable[(c ^ d[i]) & 0xFF];
  return c;
}

// Paquet LD06 : [0x54][0x2C][speed:2][start:2][12×(dist:2,intens:1)][end:2]
// [timestamp:2][crc:1] = 47 octets. Angles en centi-degrés.
std::vector<std::uint8_t> ld06Packet(double startDeg, double endDeg,
                                     std::uint16_t distanceMm,
                                     std::uint8_t intensity = 200) {
  std::vector<std::uint8_t> p(47, 0);
  p[0] = 0x54;
  p[1] = 0x2C;
  const auto s = static_cast<std::uint16_t>(startDeg * 100.0);
  const auto e = static_cast<std::uint16_t>(endDeg * 100.0);
  p[4] = static_cast<std::uint8_t>(s & 0xFF);
  p[5] = static_cast<std::uint8_t>(s >> 8);
  p[42] = static_cast<std::uint8_t>(e & 0xFF);
  p[43] = static_cast<std::uint8_t>(e >> 8);
  for (int i = 0; i < 12; ++i) {
    const std::size_t off = 6 + static_cast<std::size_t>(i) * 3;
    p[off] = static_cast<std::uint8_t>(distanceMm & 0xFF);
    p[off + 1] = static_cast<std::uint8_t>(distanceMm >> 8);
    p[off + 2] = intensity;
  }
  p[46] = ld06Crc(p.data(), 46);
  return p;
}

} // namespace

// Un tour n'est rendu que lorsque l'angle de départ « revient en arrière ».
// Il faut donc un second paquet pour clore le premier.
TEST(LD06Test, CompletedRotationIsReturned) {
  PipeFed<LD06> lidar;
  auto stream = ld06Packet(10.0, 20.0, 1500);
  auto next = ld06Packet(5.0, 15.0, 1500); // 5 < 10 : nouveau tour
  stream.insert(stream.end(), next.begin(), next.end());
  lidar.feed(stream);

  auto scan = lidar.update();
  ASSERT_TRUE(scan.has_value());
  EXPECT_EQ(scan->distance_mm[10], 1500);
  EXPECT_EQ(scan->intensity[10], 200);
}

// Tant que le tour n'est pas clos, rien ne doit remonter.
TEST(LD06Test, IncompleteRotationYieldsNoNewData) {
  PipeFed<LD06> lidar;
  lidar.feed(ld06Packet(10.0, 20.0, 1500));

  auto scan = lidar.update();
  ASSERT_FALSE(scan.has_value());
  EXPECT_EQ(scan.error(), TYPES::DriverError::NoNewData);
}

TEST(LD06Test, BadCrcIsRejected) {
  PipeFed<LD06> lidar;
  auto pkt = ld06Packet(10.0, 20.0, 1500);
  pkt[46] ^= 0xFF;
  lidar.feed(pkt);

  auto scan = lidar.update();
  ASSERT_FALSE(scan.has_value());
  EXPECT_EQ(scan.error(), TYPES::DriverError::InvalidData);
}

// Une distance nulle est la sentinelle « aucune mesure » : elle ne doit pas
// écraser un angle déjà renseigné.
TEST(LD06Test, ZeroDistanceIsNotStored) {
  PipeFed<LD06> lidar;
  auto stream = ld06Packet(10.0, 20.0, 0);
  auto next = ld06Packet(5.0, 15.0, 1500);
  stream.insert(stream.end(), next.begin(), next.end());
  lidar.feed(stream);

  auto scan = lidar.update();
  ASSERT_TRUE(scan.has_value());
  EXPECT_EQ(scan->distance_mm[10], 0);
}

// Octets parasites avant l'en-tête : le parseur doit se resynchroniser.
TEST(LD06Test, ResyncsAfterGarbage) {
  PipeFed<LD06> lidar;
  std::vector<std::uint8_t> stream{0x12, 0x54, 0x99, 0xAB, 0x54};
  auto first = ld06Packet(10.0, 20.0, 1500);
  auto second = ld06Packet(5.0, 15.0, 1500);
  stream.insert(stream.end(), first.begin(), first.end());
  stream.insert(stream.end(), second.begin(), second.end());
  lidar.feed(stream);

  auto scan = lidar.update();
  ASSERT_TRUE(scan.has_value());
  EXPECT_EQ(scan->distance_mm[10], 1500);
}

// Un paquet tronqué ne doit ni planter ni être interprété.
TEST(LD06Test, TruncatedPacketIsIgnored) {
  PipeFed<LD06> lidar;
  auto pkt = ld06Packet(10.0, 20.0, 1500);
  pkt.resize(30);
  lidar.feed(pkt);

  auto scan = lidar.update();
  EXPECT_FALSE(scan.has_value());
}

// Le paquet traverse 0° : endDeg < startDeg sans que ce soit un nouveau tour.
// Le second paquet (start 10 > 355 impossible, donc retour en arrière) clôt
// le tour et le rend.
TEST(LD06Test, PacketWrappingZeroDegreesIsHandled) {
  PipeFed<LD06> lidar;
  auto stream = ld06Packet(355.0, 5.0, 1200); // traverse 0
  auto next = ld06Packet(10.0, 20.0, 1300);   // 10 < 355 : clôt le tour
  stream.insert(stream.end(), next.begin(), next.end());
  lidar.feed(stream);

  auto scan = lidar.update();
  ASSERT_TRUE(scan.has_value());
  EXPECT_EQ(scan->distance_mm[355], 1200) << "début du paquet à cheval sur 0";
  EXPECT_EQ(scan->distance_mm[2], 1200) << "fin du paquet, après passage par 0";
}

// Contrat à connaître : update() rend le tour le plus RÉCENT. Plusieurs tours
// clos pendant un même appel (Task en retard, ou flux accumulé) sont écrasés —
// comportement souhaitable pour un lidar, où la fraîcheur primes sur
// l'exhaustivité, mais qui doit être explicite.
TEST(LD06Test, OnlyTheMostRecentRotationIsReturned) {
  PipeFed<LD06> lidar;
  auto stream = ld06Packet(10.0, 20.0, 1111); // tour A
  for (auto [start, end, dist] :
       {std::tuple{5.0, 15.0, 2222}, std::tuple{3.0, 13.0, 3333}}) {
    auto pkt = ld06Packet(start, end, static_cast<std::uint16_t>(dist));
    stream.insert(stream.end(), pkt.begin(), pkt.end());
  }
  lidar.feed(stream);

  auto scan = lidar.update();
  ASSERT_TRUE(scan.has_value());
  EXPECT_EQ(scan->distance_mm[5], 2222)
      << "le tour rendu doit être celui clos en dernier, pas le premier";
}

// ─── NEO-M8N : nominal et robustesse ────────────────────────────────────────

namespace {

void appendUbxChecksum(std::vector<std::uint8_t> &f) {
  std::uint8_t a = 0, b = 0;
  for (std::size_t i = 2; i < f.size(); ++i) {
    a = static_cast<std::uint8_t>(a + f[i]);
    b = static_cast<std::uint8_t>(b + a);
  }
  f.push_back(a);
  f.push_back(b);
}

std::vector<std::uint8_t> ubx(std::uint8_t cls, std::uint8_t id,
                              std::vector<std::uint8_t> payload) {
  std::vector<std::uint8_t> f{
      0xB5, 0x62, cls, id,
      static_cast<std::uint8_t>(payload.size() & 0xFF),
      static_cast<std::uint8_t>((payload.size() >> 8) & 0xFF)};
  f.insert(f.end(), payload.begin(), payload.end());
  appendUbxChecksum(f);
  return f;
}

void putI4(std::vector<std::uint8_t> &p, std::size_t off, std::int32_t v) {
  auto u = static_cast<std::uint32_t>(v);
  for (int i = 0; i < 4; ++i)
    p[off + static_cast<std::size_t>(i)] =
        static_cast<std::uint8_t>((u >> (8 * i)) & 0xFF);
}

} // namespace

// NAV-POSLLH : lon @4, lat @8 (degE7), hMSL @16 (mm), vAcc @24.
TEST(NeoM8nTest, PosllhIsDecoded) {
  std::vector<std::uint8_t> p(28, 0);
  putI4(p, 4, 23000000);   // 2.3°
  putI4(p, 8, 448000000);  // 44.8°
  putI4(p, 16, 142000);    // 142 m
  putI4(p, 24, 3000);      // 3 m

  PipeFed<NEO_M8N> gps;
  gps.feed(ubx(0x01, 0x02, p));
  auto d = gps.update();

  ASSERT_TRUE(d.has_value());
  EXPECT_NEAR(d->coord.longitude, 2.3, 1e-6);
  EXPECT_NEAR(d->coord.latitude, 44.8, 1e-6);
  EXPECT_EQ(d->altMslMm, 142000);
  EXPECT_EQ(d->vAcc, 3000u);
}

// NAV-VELNED : velN/E/D @4/8/12 (cm/s), heading @24 (1e-5 deg).
TEST(NeoM8nTest, VelnedIsDecoded) {
  std::vector<std::uint8_t> p(36, 0);
  putI4(p, 4, 250);       // 2.5 m/s nord
  putI4(p, 8, -100);      // -1.0 m/s est
  putI4(p, 12, 30);       // 0.3 m/s bas
  putI4(p, 24, 9000000);  // 90°

  PipeFed<NEO_M8N> gps;
  gps.feed(ubx(0x01, 0x12, p));
  auto d = gps.update();

  ASSERT_TRUE(d.has_value());
  EXPECT_EQ(d->velNED[0], 250);
  EXPECT_EQ(d->velNED[1], -100);
  EXPECT_EQ(d->velNED[2], 30);
  EXPECT_NEAR(d->heading, 90.0, 1e-3);
}

// Un message de classe non-NAV ne doit rien produire.
TEST(NeoM8nTest, NonNavClassIsIgnored) {
  PipeFed<NEO_M8N> gps;
  gps.feed(ubx(0x05, 0x01, std::vector<std::uint8_t>(2, 0))); // ACK-ACK
  auto d = gps.update();

  ASSERT_FALSE(d.has_value());
  EXPECT_EQ(d.error(), TYPES::DriverError::NoNewData);
}

// Payload de taille inattendue : rejeté, pas interprété partiellement.
TEST(NeoM8nTest, WrongPayloadSizeIsRejected) {
  PipeFed<NEO_M8N> gps;
  gps.feed(ubx(0x01, 0x02, std::vector<std::uint8_t>(20, 0xFF))); // POSLLH = 28
  auto d = gps.update();

  ASSERT_FALSE(d.has_value());
  EXPECT_EQ(d.error(), TYPES::DriverError::NoNewData);
}

// Longueur annoncée au-delà du tampon : resynchronisation, pas de débordement.
TEST(NeoM8nTest, OversizedLengthResyncs) {
  std::vector<std::uint8_t> f{0xB5, 0x62, 0x01, 0x02, 0xFF, 0xFF};
  std::vector<std::uint8_t> p(16, 0);
  p[4] = 0x03;
  auto valid = ubx(0x01, 0x03, p);
  f.insert(f.end(), valid.begin(), valid.end());

  PipeFed<NEO_M8N> gps;
  gps.feed(f);
  auto d = gps.update();

  ASSERT_TRUE(d.has_value()) << "la trame valide suivante doit être décodée";
  EXPECT_TRUE(d->gpsFixOk);
}

// Trames concaténées de types différents : toutes décodées en un update().
TEST(NeoM8nTest, ConcatenatedFramesAreAllDecoded) {
  std::vector<std::uint8_t> status(16, 0);
  status[4] = 0x03;
  std::vector<std::uint8_t> dop(18, 0);
  dop[12] = 90; // hDOP 0.90
  std::vector<std::uint8_t> posllh(28, 0);
  putI4(posllh, 16, 50000);

  auto stream = ubx(0x01, 0x03, status);
  for (auto *f : {&dop, &posllh}) {
    auto next = ubx(0x01, f == &dop ? 0x04 : 0x02, *f);
    stream.insert(stream.end(), next.begin(), next.end());
  }

  PipeFed<NEO_M8N> gps;
  gps.feed(stream);
  auto d = gps.update();

  ASSERT_TRUE(d.has_value());
  EXPECT_TRUE(d->gpsFixOk);
  EXPECT_FLOAT_EQ(d->hdop, 0.90f);
  EXPECT_EQ(d->altMslMm, 50000);
}

// Compteur de trames rejetées : outil de diagnostic du câblage au bring-up.
TEST(NeoM8nTest, ChecksumErrorsAreCounted) {
  std::vector<std::uint8_t> p(16, 0);
  p[4] = 0x03;
  auto f = ubx(0x01, 0x03, p);
  f[f.size() - 1] ^= 0xFF;

  PipeFed<NEO_M8N> gps;
  EXPECT_EQ(gps.checksumErrors(), 0u);
  gps.feed(f);
  (void)gps.update();
  EXPECT_EQ(gps.checksumErrors(), 1u);
}

// ─── WFB_NG_Driver : lecture des statistiques wfb-ng ───────────────────────
//
// Le transport est celui d'UdpTelemetryDriver, déjà couvert. Ce qui est
// propre à ce driver est la lecture des statistiques : un faux relais publie
// en UDP la même disposition que tools/wfb_stats_relay.py, sans radio ni
// matériel.

#include "drone/Components/SystemMonitoring/Driver/WFB_NG_Driver.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <thread>

namespace {

// Faux relais : publie en UDP la disposition de tools/wfb_stats_relay.py.
//
// Le pilote ECOUTE desormais ce port au lieu de se connecter en TCP : c'est
// le relais qui publie, et un datagramme se perd sans consequence si personne
// n'ecoute. Le montage du test suit donc le meme sens que la production.
class FauxRelais {
public:
  // Le pilote lie le port, donc le test doit en choisir un libre AVANT de le
  // construire : on en fait attribuer un par le noyau, puis on le relache.
  static uint16_t portLibre() {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    ::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len);
    const uint16_t port = ntohs(addr.sin_port);
    ::close(fd);
    return port;
  }

  FauxRelais() { fd_ = ::socket(AF_INET, SOCK_DGRAM, 0); }
  ~FauxRelais() {
    if (fd_ >= 0)
      ::close(fd_);
  }

  // Disposition : 0 u16 magic | 2 u8 version | 3 i8 rssi | 4 i8 snr
  //               5 u8 antennes | 6 u32 recus | 10 u32 perdus
  void publier(uint16_t port, int8_t rssi, uint32_t recus, uint32_t perdus,
               uint16_t magic = 0x5751, uint8_t version = 1,
               size_t taille = 14) {
    std::array<std::byte, 14> t{};
    std::memcpy(t.data() + 0, &magic, sizeof(magic));
    std::memcpy(t.data() + 2, &version, sizeof(version));
    std::memcpy(t.data() + 3, &rssi, sizeof(rssi));
    const int8_t snr = 20;
    std::memcpy(t.data() + 4, &snr, sizeof(snr));
    const uint8_t ant = 2;
    std::memcpy(t.data() + 5, &ant, sizeof(ant));
    std::memcpy(t.data() + 6, &recus, sizeof(recus));
    std::memcpy(t.data() + 10, &perdus, sizeof(perdus));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    ::sendto(fd_, t.data(), taille, 0,
             reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  }

private:
  int fd_{-1};
};

// Le pilote ne relit ses statistiques qu'une fois par kStatsPeriod : il faut
// donc franchir la fenetre pour qu'un nouveau datagramme soit pris en compte.
void waitForNextStatsWindow() {
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
}

} // namespace

TEST(WfbNgDriverTest, LitLeDatagrammeDuRelais) {
  const uint16_t port = FauxRelais::portLibre();
  WFB_NG_Driver drv(0, 0, port);
  FauxRelais relais;

  std::ignore = drv.poll(); // lie le socket de statistiques
  relais.publier(port, -62, 980, 20);

  waitForNextStatsWindow();
  std::ignore = drv.poll();

  ASSERT_TRUE(drv.rssiDbm().has_value());
  EXPECT_EQ(*drv.rssiDbm(), -62);
  ASSERT_TRUE(drv.lossRatio().has_value());
  EXPECT_NEAR(*drv.lossRatio(), 0.02f, 0.001f);
}

TEST(WfbNgDriverTest, SeulLeDernierDatagrammeCompte) {
  // Une mesure de qualite perimee n'a aucun interet, et accumuler du retard
  // sur le chemin RT serait pire que de perdre des echantillons : le pilote
  // vide la file et ne garde que le dernier.
  const uint16_t port = FauxRelais::portLibre();
  WFB_NG_Driver drv(0, 0, port);
  FauxRelais relais;

  std::ignore = drv.poll();
  relais.publier(port, -80, 100, 0);
  relais.publier(port, -70, 100, 0);
  relais.publier(port, -41, 100, 0);

  waitForNextStatsWindow();
  std::ignore = drv.poll();

  ASSERT_TRUE(drv.rssiDbm().has_value());
  EXPECT_EQ(*drv.rssiDbm(), -41) << "le pilote doit retenir le plus recent";
}

TEST(WfbNgDriverTest, DatagrammeInvalideEstIgnore) {
  // Magic ou version faux, ou trame tronquee : on prefere rester sans valeur
  // qu'en remonter une interpretee de travers.
  const uint16_t port = FauxRelais::portLibre();
  WFB_NG_Driver drv(0, 0, port);
  FauxRelais relais;

  std::ignore = drv.poll();
  relais.publier(port, -50, 100, 0, /*magic=*/0x1234);
  relais.publier(port, -50, 100, 0, /*magic=*/0x5751, /*version=*/99);
  relais.publier(port, -50, 100, 0, 0x5751, 1, /*taille=*/8);

  waitForNextStatsWindow();
  std::ignore = drv.poll();

  EXPECT_FALSE(drv.rssiDbm().has_value())
      << "un datagramme invalide ne doit pas produire de valeur";
}

TEST(WfbNgDriverTest, AbsentStatsServiceIsNotFatal) {
  // wfb-ng pas demarre : le driver doit rester un transport UDP utilisable,
  // sans valeur de qualite plutot qu'avec une valeur inventee.
  WFB_NG_Driver drv(0, 0, /*statsPort=*/1); // port ou personne n'ecoute

  for (int i = 0; i < 3; ++i) {
    auto r = drv.poll();
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), TYPES::DriverError::NoNewData);
  }

  EXPECT_FALSE(drv.rssiDbm().has_value());
  EXPECT_FALSE(drv.lossRatio().has_value());
}

TEST(WfbNgDriverTest, StatsPortZeroDisablesTheFeature) {
  WFB_NG_Driver drv(0, 0, /*statsPort=*/0);
  std::ignore = drv.poll();
  EXPECT_FALSE(drv.rssiDbm().has_value());
}
