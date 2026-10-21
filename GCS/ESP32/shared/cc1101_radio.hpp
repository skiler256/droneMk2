#pragma once
// Pilote CC1101 partagé par les deux projets ESP32 de la station sol.
//
// Pourquoi partagé plutôt que recopié : la table de registres a déjà existé en
// deux exemplaires — le banc et le pilote du drone — et elle a DIVERGÉ sur la
// fréquence et la longueur de préambule. Les deux bouts ne se parlaient plus,
// sans aucun message d'erreur, et le diagnostic a coûté des heures.
// Une seule source de vérité, donc.
//
// Les valeurs ci-dessous sont celles validées sur le matériel : liaison
// bidirectionnelle, motif de 40 octets vérifié octet par octet, 50 trames sur
// 50 en réception seule. Cf. BRINGUP_PDL.md.

#include <Arduino.h>
#include <SPI.h>

namespace cc1101 {

// ─────────────── Câblage et horloge ───────────────
constexpr int kPinSck = 14;
constexpr int kPinMiso = 35; // entrée seule sur ESP32, ce qui convient à MISO
constexpr int kPinMosi = 13;
constexpr int kPinCs = 4;

// 2 MHz. Mesuré sur ce montage : à 100 kHz 94 % des aller-retours PATABLE sont
// fautifs, 30 % à 500 kHz, 0 % à partir de 1 MHz. Le SPI de l'ESP32 n'est pas
// fiable aux horloges lentes sur ce câblage — c'est ce qui perdait des octets
// à l'écriture de la FIFO et bloquait le pointeur à la lecture.
// NE PAS redescendre sous 1 MHz.
constexpr uint32_t kSpiHz = 2000000;

// ─────────────── Registres et commandes ───────────────
constexpr uint8_t kRead = 0x80;
constexpr uint8_t kBurst = 0x40;

constexpr uint8_t kSRES = 0x30;
constexpr uint8_t kSRX = 0x34;
constexpr uint8_t kSTX = 0x35;
constexpr uint8_t kSIDLE = 0x36;
constexpr uint8_t kSFRX = 0x3A;
constexpr uint8_t kSFTX = 0x3B;

constexpr uint8_t kPARTNUM = 0x30;
constexpr uint8_t kVERSION = 0x31;
constexpr uint8_t kMARCSTATE = 0x35;
constexpr uint8_t kRSSI = 0x34;
constexpr uint8_t kRXBYTES = 0x3B;
constexpr uint8_t kFIFO = 0x3F;
constexpr uint8_t kPATABLE = 0x3E;

constexpr uint8_t kMCSM1 = 0x17;
constexpr uint8_t kMcsm1WithCca = 0x3F; // CCA_MODE = 11, retour en RX après TX
constexpr uint8_t kMcsm1NoCca = 0x0F;   // CCA_MODE = 00

constexpr uint8_t kRxOverflow = 0x80;
constexpr uint8_t kRxCountMask = 0x7F;
constexpr uint8_t kCrcOkBit = 0x80;
constexpr uint8_t kLqiMask = 0x7F;
constexpr uint8_t kMarcMask = 0x1F;
constexpr uint8_t kMarcIdle = 0x01;
constexpr uint8_t kMarcRx = 0x0D;
constexpr uint8_t kMarcTxFifoUnderflow = 0x16;

// Plafond théorique : 64 octets de FIFO moins longueur et octets de statut.
// MESURÉ : au-delà de ~50 octets la réception s'effondre, parce qu'à 61 la
// FIFO contient exactement sa capacité sans marge de synchronisation.
constexpr size_t kMaxPayload = 61;
constexpr size_t kUsablePayload = 50;

// À 9,6 kbit/s un octet occupe l'air 833 µs. On compte 900 et on ajoute une
// marge : l'octet de longueur est lu après son arrivée, donc l'échéance part
// déjà en retard, et lire un seul octet trop tôt vide la FIFO — la puce
// renvoie alors son dernier octet en boucle et le CRC tombe.
constexpr uint32_t kByteTimeUs = 900;
constexpr uint32_t kMargeUs = 2000;

struct RegInit {
  uint8_t addr;
  uint8_t value;
};

// 433,88 MHz (FREQ 0x10B00C), ASK/OOK, 9,6 kbit/s, quartz 26 MHz.
//
// Le drone émet à 0x10AF76 (433,82 MHz) : l'écart de 60 kHz entre les deux
// registres compense la différence de quartz des deux modules. Mesuré par
// balayage corrélé à l'horodatage absolu de la capture.
// NE PAS « aligner » les deux valeurs : ce serait les désaccorder.
inline const RegInit kConfig[] = {
    {0x00, 0x2E}, {0x02, 0x2E}, {0x03, 0x07}, {0x04, 0x12}, {0x05, 0x34},
    {0x06, 0x3D}, {0x07, 0x04}, {0x08, 0x45}, {0x0B, 0x06}, {0x0C, 0x00},
    {0x0D, 0x10}, {0x0E, 0xB0}, {0x0F, 0x0C}, {0x10, 0xF8}, {0x11, 0x83},
    {0x12, 0x32}, {0x13, 0x62}, {0x14, 0xF8}, {0x15, 0x15},
    {kMCSM1, kMcsm1WithCca},
    {0x18, 0x18}, {0x19, 0x16}, {0x1A, 0x6C}, {0x1B, 0x07}, {0x1C, 0x00},
    {0x1D, 0x91}, {0x21, 0x56}, {0x22, 0x11}, {0x23, 0xE9}, {0x24, 0x2A},
    {0x25, 0x00}, {0x26, 0x1F}, {0x2C, 0x81}, {0x2D, 0x35}, {0x2E, 0x09},
};

// OOK : PATABLE[0] = niveau bas, PATABLE[1] = niveau haut (10 dBm). Écrire un
// seul octet laisse l'émetteur muet, FREND0 pointant sur l'index 1.
constexpr uint8_t kPaTable[2] = {0x00, 0xC0};

// Compteurs de santé, remontés au sol dans LINK_QUALITY. Sans eux, une
// liaison qui se dégrade est indiscernable d'une liaison silencieuse.
struct Stats {
  uint32_t received = 0;
  uint32_t crcErrors = 0;
  uint32_t lengthErrors = 0;
  uint32_t overflows = 0;
  uint32_t ccaRefusals = 0;
  uint32_t forcedSends = 0;
  int8_t lastRssiDbm = 0;
  uint8_t lastLqi = 0;
  bool haveQuality = false;
};

class Radio {
public:
  explicit Radio(SPIClass &spi) : spi_(spi) {}

  [[nodiscard]] Stats &stats() { return stats_; }

  // Renvoie false si la puce ne répond pas ou n'est pas un CC1101 : vérifier
  // l'identité avant de configurer évite de programmer du vide et de chercher
  // ensuite pourquoi la liaison reste muette.
  [[nodiscard]] bool begin() {
    pinMode(kPinCs, OUTPUT);
    digitalWrite(kPinCs, HIGH);
    // CS à -1 : piloté uniquement en logiciel. Le confier au périphérique en
    // même temps lui fait relever CS en cours de transaction, ce qui coupe les
    // accès en rafale — octets perdus à l'écriture, pointeur bloqué en lecture.
    spi_.begin(kPinSck, kPinMiso, kPinMosi, -1);

    if (!strobe(kSRES))
      return false;
    delay(100);

    uint8_t partnum = 0xFF, version = 0xFF;
    readStatus(kPARTNUM, partnum);
    readStatus(kVERSION, version);
    if (partnum != 0x00 || version != 0x14)
      return false;

    for (const auto &reg : kConfig)
      writeReg(reg.addr, reg.value);

    csLow();
    if (waitReady()) {
      spi_.transfer(static_cast<uint8_t>(kPATABLE | kBurst));
      spi_.transfer(kPaTable[0]);
      spi_.transfer(kPaTable[1]);
    }
    csHigh();

    flushRxAndListen();
    return true;
  }

  // Émet en respectant le CCA. Renvoie false si le canal était occupé — ce
  // n'est pas une panne, c'est de la contention, et l'appelant peut réessayer.
  [[nodiscard]] bool send(const uint8_t *data, uint8_t len) {
    if (len == 0 || len > kMaxPayload)
      return false;

    uint8_t marc = 0;
    if (!readStatus(kMARCSTATE, marc))
      return false;
    const uint8_t etat = marc & kMarcMask;

    if (etat == kMarcTxFifoUnderflow) {
      // La puce reste coincée là jusqu'à ce qu'on vide la FIFO : sans ce
      // rattrapage toutes les émissions suivantes échoueraient définitivement.
      strobe(kSFTX);
      strobe(kSRX);
      return false;
    }
    // Liste des états PERMIS, pas des états interdits : entre STX et TX la
    // puce traverse CALIBRATE, SETTLING et FSTXON, et une liste noire les
    // laisse passer — le SIDLE+SFTX suivant tronque alors l'émission en cours
    // sans remonter d'erreur.
    if (etat != kMarcIdle && etat != kMarcRx)
      return false;

    strobe(kSIDLE);
    strobe(kSFTX);

    // Ecriture en RAFALE : c'est la configuration du firmware de banc, seule
    // eprouvee. L'acces simple a ete essaye et rend les choses PIRES (aucune
    // trame acceptee, contre 8 et 12 octets qui passaient). Ne pas y revenir
    // sans mesure.
    csLow();
    if (!waitReady()) {
      csHigh();
      return false;
    }
    spi_.transfer(static_cast<uint8_t>(kFIFO | kBurst));
    spi_.transfer(len);
    for (uint8_t i = 0; i < len; ++i)
      spi_.transfer(data[i]);
    csHigh();

    if (!strobe(kSTX))
      return false;

    // Avec le CCA actif la puce ignore STX en silence quand le canal est
    // occupé. Rendre « succès » ici compterait une trame jamais émise.
    for (int i = 0; i < 8; ++i) {
      uint8_t apres = 0;
      if (!readStatus(kMARCSTATE, apres))
        return false;
      if ((apres & kMarcMask) != kMarcRx)
        return true; // l'émission a démarré
    }
    ++stats_.ccaRefusals;
    strobe(kSFTX);
    return false;
  }

  // Émet sans s'effacer devant une occupation du canal. Pour les ordres où
  // rester muet est pire qu'une collision — l'arrêt d'urgence.
  //
  // Le CCA est rétabli quel que soit le sort de l'émission : le laisser éteint
  // transformerait une exception ponctuelle en comportement permanent, sans
  // que rien ne le signale.
  [[nodiscard]] bool sendForced(const uint8_t *data, uint8_t len) {
    ++stats_.forcedSends;
    writeReg(kMCSM1, kMcsm1NoCca);
    const bool ok = send(data, len);
    writeReg(kMCSM1, kMcsm1WithCca);
    return ok;
  }

  // Non bloquant tant qu'aucune trame n'arrive. Renvoie la longueur reçue, ou
  // 0. `buf` doit faire au moins kMaxPayload octets.
  [[nodiscard]] uint8_t poll(uint8_t *buf) {
    // Ne rien lire tant que la radio n'est pas franchement en réception :
    // juste après une émission elle traverse plusieurs états, et la FIFO lue
    // pendant cette transition ne veut rien dire.
    uint8_t marc = 0;
    if (!readStatus(kMARCSTATE, marc) || (marc & kMarcMask) != kMarcRx)
      return 0;

    uint8_t rxBytes = 0;
    if (!readRxBytes(rxBytes))
      return 0;

    if (rxBytes & kRxOverflow) {
      ++stats_.overflows;
      flushRxAndListen();
      return 0;
    }

    // Un paquet complet porte sa longueur plus deux octets d'état : en dessous
    // de trois octets, rien de ce qui est là ne peut être une trame.
    if ((rxBytes & kRxCountMask) < 3)
      return 0;

    uint8_t len = 0;
    if (!readReg(kFIFO, len))
      return 0;
    if (len == 0 || len > kMaxPayload) {
      ++stats_.lengthErrors;
      flushRxAndListen();
      return 0;
    }

    // RXBYTES annonce le paquet complet bien avant qu'il le soit — il a
    // annoncé 42 octets alors que la FIFO n'en contenait qu'une dizaine.
    // Aucune confirmation ne rattrape un compteur qui mente de façon
    // constante, et lire en avance dérègle le pointeur de la puce. On n'attend
    // donc pas un compte mais une durée. C'est ce que ferait une ligne GDO,
    // faute de l'avoir câblée.
    delayMicroseconds((static_cast<uint32_t>(len) + 2) * kByteTimeUs + kMargeUs);

    // L'attente ci-dessus est une ESTIMATION : le debit reel derive de la
    // tolerance du quartz, et le compte peut etre transitoirement court. Une
    // seule verification jetterait alors la trame — c'est exactement la
    // regression introduite en extrayant ce code, qui faisait perdre la
    // quasi-totalite des trames. On reessaie donc, avec une echeance.
    const unsigned long limite = millis() + 100;
    for (;;) {
      uint8_t dispo = 0;
      if (!readRxBytes(dispo) || (dispo & kRxOverflow) ||
          millis() > limite) {
        flushRxAndListen();
        return 0;
      }
      if ((dispo & kRxCountMask) >= len + 2)
        break;
      delayMicroseconds(kByteTimeUs);
    }

    // Acces SIMPLE, une transaction par octet, CS relache entre chaque.
    //
    // La fiche technique recommande la rafale pour la FIFO de reception, mais
    // la MESURE dit le contraire sur ce montage : en rafale le pointeur de la
    // puce cesse d'avancer et renvoie son dernier octet en boucle, et tout
    // echoue au CRC. En acces simple, 90 trames sur 90 passent.
    //
    // C'est exactement la regression que l'extraction de ce code avait
    // introduite — ne pas « optimiser » en rafale sans remesurer.
    uint8_t brut[kMaxPayload + 2];
    for (uint8_t i = 0; i < len + 2; ++i) {
      if (!readReg(kFIFO, brut[i])) {
        flushRxAndListen();
        return 0;
      }
    }

    const uint8_t statutRssi = brut[len];
    const uint8_t statutLqi = brut[len + 1];

    if ((statutLqi & kCrcOkBit) == 0) {
      ++stats_.crcErrors;
      // Sans ce vidage, les octets du paquet rejeté restent en FIFO et le
      // premier est relu comme un octet de longueur au tour suivant — d'où des
      // longueurs aberrantes en cascade.
      flushRxAndListen();
      return 0;
    }

    stats_.lastRssiDbm = rssiToDbm(statutRssi);
    stats_.lastLqi = static_cast<uint8_t>(statutLqi & kLqiMask);
    stats_.haveQuality = true;
    ++stats_.received;
    memcpy(buf, brut, len);
    return len;
  }

  void flushRxAndListen() {
    strobe(kSIDLE);
    strobe(kSFRX);
    strobe(kSRX);
  }

private:
  // Le CC1101 signale qu'il est prêt en relâchant MISO après l'abaissement de
  // CS. Sans cette attente, les premiers accès renvoient n'importe quoi.
  bool waitReady(uint32_t timeoutMs = 50) {
    const uint32_t debut = millis();
    while (digitalRead(kPinMiso) == HIGH)
      if (millis() - debut > timeoutMs)
        return false;
    return true;
  }

  void csLow() {
    spi_.beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(kPinCs, LOW);
  }
  void csHigh() {
    digitalWrite(kPinCs, HIGH);
    spi_.endTransaction();
  }

  bool strobe(uint8_t cmd) {
    csLow();
    const bool ok = waitReady();
    if (ok)
      spi_.transfer(cmd);
    csHigh();
    return ok;
  }

  bool writeReg(uint8_t addr, uint8_t value) {
    csLow();
    const bool ok = waitReady();
    if (ok) {
      spi_.transfer(addr);
      spi_.transfer(value);
    }
    csHigh();
    return ok;
  }

  bool readReg(uint8_t addr, uint8_t &out) {
    csLow();
    const bool ok = waitReady();
    if (ok) {
      spi_.transfer(static_cast<uint8_t>(addr | kRead));
      out = spi_.transfer(0x00);
    }
    csHigh();
    return ok;
  }

  // Les registres d'état exigent le bit de rafale, sans quoi la puce renvoie
  // une valeur périmée — piège classique du CC1101.
  bool readStatus(uint8_t addr, uint8_t &out) {
    csLow();
    const bool ok = waitReady();
    if (ok) {
      spi_.transfer(static_cast<uint8_t>(addr | kRead | kBurst));
      out = spi_.transfer(0x00);
    }
    csHigh();
    return ok;
  }

  // Erratum CC1101 : RXBYTES lu pendant que la puce le met à jour renvoie un
  // compte corrompu. Deux lectures concordantes, espacées d'un temps d'octet —
  // coup sur coup elles concorderaient même en pleine réception.
  bool readRxBytes(uint8_t &out) {
    uint8_t a = 0;
    if (!readStatus(kRXBYTES, a))
      return false;
    for (int i = 0; i < 8; ++i) {
      delayMicroseconds(kByteTimeUs);
      uint8_t b = 0;
      if (!readStatus(kRXBYTES, b))
        return false;
      if (a == b) {
        out = a;
        return true;
      }
      a = b;
    }
    return false; // compte jamais stable : on ne touche pas à la FIFO
  }

  static int8_t rssiToDbm(uint8_t raw) {
    const int rssi = raw >= 128 ? (static_cast<int>(raw) - 256) / 2
                                : static_cast<int>(raw) / 2;
    return static_cast<int8_t>(rssi - 74);
  }

  SPIClass &spi_;
  Stats stats_;
};

} // namespace cc1101
