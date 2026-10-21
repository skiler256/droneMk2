#include "drone/Components/SystemMonitoring/Driver/CC1101_Driver.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

// ─── Bits de l'octet d'en-tête SPI ────────────────────────────────────────
constexpr uint8_t kRead = 0x80;
constexpr uint8_t kBurst = 0x40;

// ─── Commandes (strobes) ──────────────────────────────────────────────────
constexpr uint8_t kSRES = 0x30;  // reset logiciel
constexpr uint8_t kSRX = 0x34;   // passer en réception
constexpr uint8_t kSTX = 0x35;   // passer en émission
constexpr uint8_t kSIDLE = 0x36; // repos
constexpr uint8_t kSFRX = 0x3A;  // vider la FIFO de réception
constexpr uint8_t kSFTX = 0x3B;  // vider la FIFO d'émission

// ─── Registres d'état et FIFO ─────────────────────────────────────────────
constexpr uint8_t kPARTNUM = 0x30;
constexpr uint8_t kVERSION = 0x31;
constexpr uint8_t kMARCSTATE = 0x35;
constexpr uint8_t kRXBYTES = 0x3B;
constexpr uint8_t kFIFO = 0x3F;
constexpr uint8_t kPATABLE = 0x3E;

// Identité attendue de la puce. Une lecture différente signifie un câblage
// SPI faux ou un composant absent, pas une erreur de configuration.
constexpr uint8_t kExpectedPartnum = 0x00;
constexpr uint8_t kExpectedVersion = 0x14;

constexpr uint8_t kRxBytesOverflow = 0x80;
constexpr uint8_t kRxBytesCountMask = 0x7F;
constexpr uint8_t kCrcOkBit = 0x80; // bit 7 du second octet de statut
constexpr uint8_t kLqiMask = 0x7F;

// MARCSTATE : la FIFO d'émission ne doit être vidée que depuis un état
// franchement calme, sous peine de tronquer la trame en cours.
//
// Lister les états INTERDITS est un piège : entre le strobe STX et l'état TX
// la puce traverse STARTCAL, BWBOOST, FS_LOCK, IFADCON, ENDCAL puis FSTXON.
// Une garde qui ne refusait que TX et TX_END laissait donc passer tous ces
// états transitoires, et le SIDLE+SFTX suivant tronquait l'émission en cours
// sans remonter la moindre erreur. On liste les états PERMIS.
constexpr uint8_t kMarcStateMask = 0x1F;
constexpr uint8_t kMarcIdle = 0x01;
constexpr uint8_t kMarcRx = 0x0D;
constexpr uint8_t kMarcTxFifoUnderflow = 0x16;

// MCSM1 avec et sans détection de porteuse. Les deux valeurs ne diffèrent que
// par CCA_MODE : le retour en RX après émission est conservé dans les deux cas.
constexpr uint8_t kMCSM1 = 0x17;
constexpr uint8_t kMcsm1WithCca = 0x3F;
constexpr uint8_t kMcsm1NoCca = 0x0F;

// ─── Configuration radio ──────────────────────────────────────────────────
//
// 433,80 MHz, ASK/OOK, 10 kbit/s, quartz 26 MHz — configuration reprise
// d'un montage anterieur qui fonctionnait avec ces memes modules.
//
// Le débit n'est pas un réglage de confort : à 8 Hz cumulés et ~32 octets
// par trame, 10 kbit/s donnerait 20 % de rapport cyclique — au-dessus de la
// limite ETSI de la bande 433 MHz. À 38,4 kbit/s on tombe à 5 %.
// cf. SYSMONITORING.md §16.
//
// FREQ = f × 2^16 / f_quartz = 433,92e6 × 65536 / 26e6 = 0x10B071.
struct RegInit {
  uint8_t addr;
  uint8_t value;
};

constexpr RegInit kConfig[] = {
    {0x00, 0x2E}, // IOCFG2  — GDO non cables
    {0x02, 0x2E}, // IOCFG0
    {0x03, 0x07}, // FIFOTHR
    {0x04, 0x12}, // SYNC1   — mot de synchro 0x1234
    {0x05, 0x34}, // SYNC0
    {0x06, 0x3D}, // PKTLEN
    {0x07, 0x04}, // PKTCTRL1 — ajoute RSSI et LQI/CRC
    {0x08, 0x45}, // PKTCTRL0 — blanchiment + CRC + longueur variable
    {0x0B, 0x06}, // FSCTRL1
    {0x0C, 0x00}, // FSCTRL0
    {0x0D, 0x10}, // FREQ2   ┐
    {0x0E, 0xAF}, // FREQ1   ├ 433,8203 MHz, quartz 26 MHz — optimum
    {0x0F, 0x76}, // FREQ0   ┘ mesure par balayage, cf. BRINGUP_PDL.md.
                  // La station sol ecoute 60 kHz plus haut (0x10B00C) :
                  // cet ecart compense la difference de quartz entre les
                  // deux modules. Ne PAS aligner les deux valeurs.

    {0x10, 0xF8}, // MDMCFG4 — filtre RX 68 kHz, 9,6 kbit/s
    {0x11, 0x83}, // MDMCFG3
    {0x12, 0x32}, // MDMCFG2 — ASK/OOK, synchro 16/16
    {0x13, 0x62}, // MDMCFG1 — preambule 24 octets : l'acquisition OOK en
                  // demande beaucoup plus que le GFSK.
    {0x14, 0xF8}, // MDMCFG0
    {0x15, 0x15}, // DEVIATN — sans objet en OOK
    // CCA_MODE = 11 : n'emettre que si le canal est libre, et jamais pendant
    // une reception en cours. C'est le plus prudent des quatre reglages.
    // Attention : la puce ignore alors STX en silence quand le canal est
    // occupe — send() doit verifier que l'emission a reellement demarre,
    // sinon la perte devient invisible au lieu de diminuer.
    {kMCSM1, kMcsm1WithCca}, // MCSM1 — CCA actif, retour en RX apres TX
    {0x18, 0x18}, // MCSM0   — auto-calibration
    {0x19, 0x16}, // FOCCFG
    {0x1A, 0x6C}, // BSCFG
    {0x1B, 0x07}, // AGCCTRL2 — reglages AGC propres a l'OOK
    {0x1C, 0x00}, // AGCCTRL1
    {0x1D, 0x91}, // AGCCTRL0
    {0x21, 0x56}, // FREND1
    {0x22, 0x11}, // FREND0  — PA_POWER=1, impose par l'OOK
    {0x23, 0xE9}, // FSCAL3
    {0x24, 0x2A}, // FSCAL2
    {0x25, 0x00}, // FSCAL1
    {0x26, 0x1F}, // FSCAL0
    {0x2C, 0x81}, // TEST2
    {0x2D, 0x35}, // TEST1
    {0x2E, 0x09}, // TEST0
};

// 10 dBm, soit les 10 mW ERP autorisés sur 433,05–434,79 MHz en ETSI.
// En OOK, PATABLE[0] porte le niveau « 0 » et PATABLE[1] le niveau « 1 ».
// Ecrire un seul octet, comme en GFSK, laisse l'emetteur muet.
constexpr uint8_t kPaTable[2] = {0x00, 0xC0}; // 0 dBm eteint, 10 dBm allume

// Conversion de l'octet RSSI brut en dBm, formule de la fiche technique.
int8_t rssiToDbm(uint8_t raw) {
  constexpr int kOffset = 74; // dB, valeur pour 433 MHz
  int rssi = raw >= 128 ? (static_cast<int>(raw) - 256) / 2 : static_cast<int>(raw) / 2;
  return static_cast<int8_t>(rssi - kOffset);
}

} // namespace

CC1101_Driver::CC1101_Driver(const char *spiDevice, uint32_t speedHz)
    : device_(spiDevice), speedHz_(speedHz) {}

CC1101_Driver::~CC1101_Driver() {
  if (fd_ >= 0)
    ::close(fd_);
}

std::optional<TYPES::DriverError> CC1101_Driver::transfer(const uint8_t *tx,
                                                          uint8_t *rx,
                                                          size_t len) {
  if (fd_ < 0)
    return TYPES::DriverError::NotInitialized;

  spi_ioc_transfer tr{};
  tr.tx_buf = reinterpret_cast<__u64>(tx);
  tr.rx_buf = reinterpret_cast<__u64>(rx);
  tr.len = static_cast<__u32>(len);
  tr.speed_hz = speedHz_;
  tr.bits_per_word = 8;

  if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 1)
    return TYPES::DriverError::SPITransferFailed;
  return std::nullopt;
}

std::optional<TYPES::DriverError> CC1101_Driver::writeReg(uint8_t addr,
                                                           uint8_t value) {
  const uint8_t tx[2] = {addr, value};
  uint8_t rx[2]{};
  return transfer(tx, rx, 2);
}

std::expected<uint8_t, TYPES::DriverError> CC1101_Driver::readReg(uint8_t addr) {
  const uint8_t tx[2] = {static_cast<uint8_t>(addr | kRead), 0x00};
  uint8_t rx[2]{};
  if (auto err = transfer(tx, rx, 2))
    return std::unexpected(*err);
  return rx[1];
}

std::expected<uint8_t, TYPES::DriverError>
CC1101_Driver::readStatusReg(uint8_t addr) {
  // Sans le bit de rafale, un registre d'état renvoie une valeur périmée.
  const uint8_t tx[2] = {static_cast<uint8_t>(addr | kRead | kBurst), 0x00};
  uint8_t rx[2]{};
  if (auto err = transfer(tx, rx, 2))
    return std::unexpected(*err);
  return rx[1];
}

std::optional<TYPES::DriverError> CC1101_Driver::strobe(uint8_t cmd) {
  uint8_t rx{};
  return transfer(&cmd, &rx, 1);
}

std::optional<TYPES::DriverError> CC1101_Driver::applyConfig() {
  for (const auto &reg : kConfig)
    if (auto err = writeReg(reg.addr, reg.value))
      return err;
  const uint8_t tx[3] = {static_cast<uint8_t>(kPATABLE | kBurst), kPaTable[0],
                         kPaTable[1]};
  uint8_t rx[3]{};
  return transfer(tx, rx, 3);
}

std::optional<TYPES::DriverError> CC1101_Driver::init() {
  fd_ = ::open(device_, O_RDWR);
  if (fd_ < 0) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return TYPES::DriverError::SPIOpenFailed;
  }

  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
      ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speedHz_) < 0) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return TYPES::DriverError::SPIConfigFailed;
  }

  if (auto err = strobe(kSRES)) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return err;
  }

  // Vérifier l'identité avant de configurer : une puce absente ou un câblage
  // SPI faux donnerait sinon une configuration « réussie » sur du vide.
  //
  // La puce ne répond pas juste après le strobe de reset : lire PARTNUM
  // immédiatement renvoie n'importe quoi et faisait échouer init() sur une
  // puce parfaitement saine. On réessaie pendant 50 ms plutôt que de dormir
  // un délai arbitraire, ce qui échoue toujours franchement si elle est
  // réellement absente.
  constexpr int kTentatives = 50;
  std::expected<uint8_t, TYPES::DriverError> partnum{0};
  std::expected<uint8_t, TYPES::DriverError> version{0};
  bool identifiee = false;
  for (int i = 0; i < kTentatives; ++i) {
    ::usleep(1000);
    partnum = readStatusReg(kPARTNUM);
    version = readStatusReg(kVERSION);
    if (partnum && version && *partnum == kExpectedPartnum &&
        *version == kExpectedVersion) {
      identifiee = true;
      break;
    }
  }
  if (!partnum || !version) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return partnum ? version.error() : partnum.error();
  }
  if (!identifiee) {
    setHealth(TYPES::DriverHealth::Dead);
    return TYPES::DriverError::InvalidData;
  }

  if (auto err = applyConfig()) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return err;
  }

  if (auto err = strobe(kSFRX))
    return err;
  if (auto err = strobe(kSRX))
    return err;

  setHealth(TYPES::DriverHealth::Connected);
  return std::nullopt;
}

std::expected<uint8_t, TYPES::DriverError> CC1101_Driver::readRxBytesStable() {
  auto first = readStatusReg(kRXBYTES);
  if (!first)
    return std::unexpected(first.error());
  uint8_t previous = *first;
  for (int i = 0; i < 4; ++i) {
    auto again = readStatusReg(kRXBYTES);
    if (!again)
      return std::unexpected(again.error());
    if (*again == previous)
      return previous;
    previous = *again;
  }
  // Compte jamais stable : ne pas toucher à la FIFO sur une base douteuse.
  return std::unexpected(TYPES::DriverError::NoNewData);
}

std::optional<TYPES::DriverError> CC1101_Driver::flushRxAndListen() {
  pendingLength_ = 0;
  if (auto err = strobe(kSIDLE))
    return err;
  if (auto err = strobe(kSFRX))
    return err;
  return strobe(kSRX);
}

std::expected<std::span<const std::byte>, TYPES::DriverError>
CC1101_Driver::poll() {
  // Un paquet est deja annonce et pas encore entierement arrive : ne toucher
  // a RIEN jusqu'a l'echeance, pas meme un registre d'etat.
  //
  // C'est la difference decisive avec le recepteur de la station sol, qui ne
  // perd quasiment rien : lui attend en une seule pause, sans aucun acces SPI
  // pendant que la radio remplit la FIFO. Le pilote, appele en boucle serree,
  // relisait MARCSTATE et RXBYTES toutes les 200 us pendant les 30 ms
  // d'arrivee — des centaines d'acces concurrents aux ecritures de la radio,
  // ce que l'erratum CC1101 designe comme la cause du dereglement du
  // pointeur de FIFO. Ce test est purement local : aucun cout, aucun blocage.
  if (pendingLength_ != 0 &&
      std::chrono::steady_clock::now() < pendingReadyAt_)
    return std::unexpected(TYPES::DriverError::NoNewData);

  auto rxBytes = readRxBytesStable();
  if (!rxBytes)
    return std::unexpected(rxBytes.error());

  // Débordement : la FIFO est corrompue dans son ensemble, il n'y a rien à
  // sauver. On la vide et on repart en réception plutôt que de livrer des
  // octets dont on ne sait pas à quel paquet ils appartiennent.
  if ((*rxBytes & kRxBytesOverflow) != 0) {
    ++overflows_;
    if (auto err = flushRxAndListen())
      return std::unexpected(*err);
    return std::unexpected(TYPES::DriverError::InvalidData);
  }

  // La puce ne doit pas être en train de changer d'état : juste après une
  // émission elle traverse plusieurs états, et la FIFO lue à ce moment-là ne
  // veut rien dire.
  auto marcRx = readStatusReg(kMARCSTATE);
  if (!marcRx)
    return std::unexpected(marcRx.error());
  if ((*marcRx & kMarcStateMask) != kMarcRx)
    return std::unexpected(TYPES::DriverError::NoNewData);

  const uint8_t available = *rxBytes & kRxBytesCountMask;

  // Longueur déjà retirée lors d'un appel précédent, ou à lire maintenant.
  uint8_t frameLength = pendingLength_;
  if (frameLength == 0) {
    // Un paquet complet porte sa longueur plus deux octets de statut : en
    // dessous de trois octets, rien de ce qui est là ne peut être une trame.
    if (available < 3)
      return std::unexpected(TYPES::DriverError::NoNewData);

    auto length = readReg(kFIFO);
    if (!length)
      return std::unexpected(length.error());

    if (*length == 0 || *length > kMaxPayload) {
      // Longueur aberrante : trame corrompue avant le contrôle CRC.
      ++lengthErrors_;
      if (auto err = flushRxAndListen())
        return std::unexpected(*err);
      return std::unexpected(TYPES::DriverError::InvalidData);
    }
    frameLength = *length;
    pendingLength_ = frameLength;
    // 9,6 kbit/s (MDMCFG4/MDMCFG3) : un octet occupe l'air 833 us en theorie.
    // On compte 900 us et on ajoute 2 ms, pour deux raisons :
    //  - l'octet de longueur est lu un peu apres son arrivee, donc l'echeance
    //    part deja en retard par rapport au debut du paquet ;
    //  - le debit reel derive de la tolerance du quartz, et lire un seul
    //    octet trop tot suffit a vider la FIFO : la puce renvoie alors son
    //    dernier octet en boucle et le CRC tombe.
    // Ces valeurs sont celles du firmware ESP32, qui ne perd quasiment rien
    // (90 trames sur 90) la ou le pilote sans marge en rejetait 15 a 26 %.
    constexpr auto kByteTime = std::chrono::microseconds(900);
    constexpr auto kMarge = std::chrono::milliseconds(2);
    pendingReadyAt_ = std::chrono::steady_clock::now() + kMarge +
                      kByteTime * (static_cast<int64_t>(frameLength) + 2);
  }

  // Garde-fou : le test en tete de fonction couvre le cas normal, celui-ci
  // rattrape l'appel ou la longueur vient tout juste d'etre lue.
  if (std::chrono::steady_clock::now() < pendingReadyAt_)
    return std::unexpected(TYPES::DriverError::NoNewData);

  // Charge utile + les deux octets de statut ajoutés par la puce (PKTCTRL1).
  const size_t toRead = static_cast<size_t>(frameLength) + 2;

  // La FIFO se remplit pendant que le paquet arrive. Lire maintenant
  // livrerait du vide à la suite des données ; on retient la longueur et on
  // reprend au tour suivant plutôt que d'attendre sur le chemin RT.
  if (available < toRead) {
    pendingLength_ = frameLength;
    return std::unexpected(TYPES::DriverError::NoNewData);
  }
  pendingLength_ = 0;

  // Lecture en RAFALE. Le firmware de la station sol a du passer en acces
  // simple — son bus SPI etant defaillant aux horloges lentes — mais sur le
  // Pi la MESURE favorise la rafale : en reception seule, 50 trames sur 50
  // sans une erreur. L'acces simple n'a rien ameliore ici, les deux hotes
  // n'ont donc pas le meme optimum et il ne faut pas transposer.
  std::array<uint8_t, kMaxPayload + 3> tx{};
  std::array<uint8_t, kMaxPayload + 3> rx{};
  tx[0] = kFIFO | kRead | kBurst;
  if (auto err = transfer(tx.data(), rx.data(), toRead + 1))
    return std::unexpected(*err);
  // Les donnees commencent apres l'octet d'adresse.
  std::array<uint8_t, kMaxPayload + 2> brut{};
  std::memcpy(brut.data(), rx.data() + 1, toRead);

  const uint8_t statusRssi = brut[frameLength];
  const uint8_t statusLqi = brut[frameLength + 1];

  if ((statusLqi & kCrcOkBit) == 0) {
    ++crcErrors_;
    rejectedLen_ = std::min(static_cast<size_t>(frameLength), kMaxPayload);
    std::memcpy(rejectedBuf_.data(), brut.data(), rejectedLen_);
    // Sans ce vidage, les octets du paquet rejeté restent en FIFO et le
    // premier d'entre eux est relu comme un octet de longueur au tour
    // suivant — d'où des longueurs aberrantes en cascade.
    if (auto err = flushRxAndListen())
      return std::unexpected(*err);
    return std::unexpected(TYPES::DriverError::InvalidData);
  }

  rssiDbm_ = rssiToDbm(statusRssi);
  lqi_ = static_cast<uint8_t>(statusLqi & kLqiMask);

  std::memcpy(rxBuf_.data(), brut.data(), frameLength);
  return std::span<const std::byte>(rxBuf_.data(), frameLength);
}

std::optional<TYPES::DriverError>
CC1101_Driver::sendForced(std::span<const std::byte> payload) {
  // Le CCA vit dans MCSM1 : on l'éteint, on émet, on le rallume. Passer par
  // send() garantit que tout le reste — garde d'état, vérification de départ,
  // compteurs — s'applique identiquement.
  //
  // Le rétablissement est fait quel que soit le sort de l'émission : laisser
  // le CCA éteint après un échec transformerait une exception ponctuelle en
  // comportement permanent, et le lien perdrait sa discipline sans que rien
  // ne le signale.
  ++forcedSends_;
  if (auto err = writeReg(kMCSM1, kMcsm1NoCca))
    return err;

  auto resultat = send(payload);

  if (auto err = writeReg(kMCSM1, kMcsm1WithCca)) {
    // L'échec du rétablissement est plus grave que celui de l'émission : on
    // le remonte en priorité pour qu'il ne passe pas inaperçu.
    return err;
  }
  return resultat;
}

std::optional<TYPES::DriverError>
CC1101_Driver::send(std::span<const std::byte> payload) {
  if (payload.empty() || payload.size() > kMaxPayload)
    return TYPES::DriverError::InvalidData;

  // N'émettre que depuis IDLE ou RX : dans tout autre état la puce est en
  // train de calibrer, de basculer ou d'émettre, et vider la FIFO
  // tronquerait la trame en cours. On refuse plutôt que d'attendre — send()
  // est appelé depuis le chemin RT et ne doit jamais bloquer.
  auto marc = readStatusReg(kMARCSTATE);
  if (!marc)
    return marc.error();
  const uint8_t state = *marc & kMarcStateMask;

  if (state == kMarcTxFifoUnderflow) {
    // Sous-alimentation : la puce reste coincée là jusqu'à ce qu'on vide la
    // FIFO. Sans ce rattrapage, toutes les émissions suivantes échoueraient
    // définitivement.
    if (auto err = strobe(kSFTX))
      return err;
    if (auto err = strobe(kSRX))
      return err;
    return TYPES::DriverError::Timeout;
  }

  if (state != kMarcIdle && state != kMarcRx)
    return TYPES::DriverError::Timeout;

  // SIDLE avorte une réception en cours. Si un paquet était annoncé et
  // partiellement arrivé, sa longueur mémorisée ne correspond plus à rien :
  // le poll() suivant lirait len+2 octets dans une FIFO repartie de zéro, et
  // le CRC échouerait systématiquement.
  //
  // Mesuré : en réception seule le pilote ne perd rien (50 trames sur 50),
  // mais dès qu'il émet aussi, 100 % des trames reçues sont rejetées. Ce
  // défaut n'apparaît donc QUE lorsque les deux sens cohabitent — c'est-à-dire
  // en exploitation réelle.
  if (auto err = strobe(kSIDLE))
    return err;
  pendingLength_ = 0;
  if (auto err = strobe(kSFRX))
    return err;
  if (auto err = strobe(kSFTX))
    return err;

  std::array<uint8_t, kMaxPayload + 2> tx{};
  std::array<uint8_t, kMaxPayload + 2> rx{};
  tx[0] = kFIFO | kBurst;
  tx[1] = static_cast<uint8_t>(payload.size());
  std::memcpy(tx.data() + 2, payload.data(), payload.size());

  if (auto err = transfer(tx.data(), rx.data(), payload.size() + 2))
    return err;

  if (auto err = strobe(kSTX))
    return err;

  // Le CCA peut refuser l'émission : la puce ignore alors STX et reste en
  // réception. Rendre « succès » ici livrerait une trame jamais émise, et le
  // CCA aggraverait le silence au lieu de réduire les collisions. On vérifie
  // donc que la machine d'état a bien quitté RX.
  //
  // Le coût est borné : après STX la puce calibre (~720 us) avant d'émettre,
  // et tout état hors RX prouve que le départ a eu lieu.
  constexpr int kVerifications = 8;
  for (int i = 0; i < kVerifications; ++i) {
    auto apres = readStatusReg(kMARCSTATE);
    if (!apres)
      return apres.error();
    if ((*apres & kMarcStateMask) != kMarcRx)
      return std::nullopt; // l'émission a démarré
  }

  // Toujours en réception après huit lectures : le canal était occupé. On
  // vide la FIFO d'émission pour ne pas laisser la trame s'émettre plus tard
  // à contretemps, et on rend Timeout pour que l'appelant puisse réessayer.
  ++ccaRefusals_;
  if (auto err = strobe(kSFTX))
    return err;
  return TYPES::DriverError::Timeout;
}
