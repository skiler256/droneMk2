// Liaison CC1101 côté station sol — pendant du banc ./bench_cc1101 côté drone.
//
// Configure la radio, émet un compteur chaque seconde et affiche tout ce qui
// arrive, sur le port série à 115200 bauds. Ne touche ni aux MCP23017 ni aux
// boutons du panneau d'alarme.
//
// ATTENTION, DETTE CONNUE : ce projet garde sa propre copie du pilote radio.
// La référence est désormais GCS/ESP32/shared/cc1101_radio.hpp, utilisée par
// le firmware de production. Cette copie a déjà divergé une fois (fréquence et
// longueur de préambule) et les deux bouts ne se parlaient plus, sans aucun
// message d'erreur. À migrer sur l'en-tête partagé.
//
// Ce banc reste utile pour le bring-up (mode écho, pingpong) ; le firmware de
// production, lui, ne fait que du pont.

#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr int kPinSck = 14;
constexpr int kPinMiso = 35; // entrée seule sur ESP32, ce qui convient à MISO
constexpr int kPinMosi = 13;
constexpr int kPinCs = 4;

// 2 MHz. Mesure sur ce montage (cc1101_test) : a 100 kHz 94 % des
// aller-retours PATABLE sont fautifs, 30 % a 500 kHz, et 0 % des 1 MHz.
// Le SPI de l'ESP32 n'est pas fiable aux horloges lentes sur ce cablage ;
// c'est ce qui perdait des octets a l'ecriture de la FIFO d'emission et
// bloquait le pointeur a la lecture. Ne pas redescendre sous 1 MHz.
constexpr uint32_t kSpiHz = 2000000;

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
constexpr uint8_t kRXBYTES = 0x3B;
constexpr uint8_t kFIFO = 0x3F;
constexpr uint8_t kPATABLE = 0x3E;
constexpr uint8_t kRSSI = 0x34;

constexpr uint8_t kRxOverflow = 0x80;
constexpr uint8_t kRxCountMask = 0x7F;
constexpr uint8_t kCrcOkBit = 0x80;
constexpr uint8_t kLqiMask = 0x7F;
constexpr uint8_t kMarcMask = 0x1F;
constexpr uint8_t kMarcTx = 0x13;
constexpr uint8_t kMarcTxEnd = 0x14;
constexpr uint8_t kMarcRx = 0x0D;

constexpr size_t kMaxPayload = 61;

// ─────────── repondeur du banc ./bench_link ───────────
//
// Disposition identique a BenchFrame dans src/HwBench/bench_link.cpp :
//   0 u16 magic | 2 u8 version | 3 u8 mode | 4 u32 seq | 8 u64 txNs
//  16 i8 farRssi | 17 u8 farLqi | 18 u8 farFlags | 19 u8 reserved
//
// L'horodatage est renvoye tel quel, sans etre interprete : c'est le drone
// qui mesure, avec sa seule horloge. On n'ecrit que la qualite de reception
// vue d'ici, ce qui donne au drone les deux sens en une seule mesure.
constexpr uint16_t kBenchMagic = 0x4B4C;
constexpr uint8_t kBenchVersion = 1;
constexpr size_t kBenchHeader = 20;
constexpr uint8_t kBenchFarValid = 0x01;
constexpr uint8_t kBenchModeFlood = 2;

uint32_t benchEchoes = 0;
uint32_t benchFlood = 0;
unsigned long benchDernier = 0;   // derniere trame de banc vue
unsigned long benchDernierBilan = 0;

// Tant que le banc tourne, la station sol ne doit pas emettre son propre
// compteur : sur un lien semi-duplex il collisionnerait avec la mesure.
constexpr unsigned long kBenchActifMs = 5000;
bool benchActif() { return benchDernier != 0 &&
                           millis() - benchDernier < kBenchActifMs; }

// 433,88 MHz (FREQ 0x10B00C), ASK/OOK, 9,6 kbit/s, quartz 26 MHz.
// Le drone emet a 0x10AF76 (433,82 MHz) : l'ecart de 60 kHz entre les deux
// registres compense la difference de quartz des deux modules. Mesure par
// balayage — ne pas "aligner" les deux valeurs, ce serait les desaccorder.
// Copie conforme de CC1101_Driver.cpp — toute modification doit être faite
// des deux côtés en même temps.
struct RegInit {
  uint8_t addr;
  uint8_t value;
};

const RegInit kConfig[] = {
    {0x00, 0x2E},
    {0x02, 0x2E},
    {0x03, 0x07},
    {0x04, 0x12},
    {0x05, 0x34},
    {0x06, 0x3D},
    {0x07, 0x04},
    {0x08, 0x45},
    {0x0B, 0x06},
    {0x0C, 0x00},
    {0x0D, 0x10},
    {0x0E, 0xB0},
    {0x0F, 0x0C},
    {0x10, 0xF8},
    {0x11, 0x83},
    {0x12, 0x32},
    {0x13, 0x62},
    {0x14, 0xF8},
    {0x15, 0x15},
    // CCA_MODE = 11 : n'emettre que si le canal est libre. Doit etre actif
    // des DEUX cotes, sinon un seul des deux s'efface et l'autre continue de
    // couper la parole.
    {0x17, 0x3F},
    {0x18, 0x18},
    {0x19, 0x16},
    {0x1A, 0x6C},
    {0x1B, 0x07},
    {0x1C, 0x00},
    {0x1D, 0x91},
    {0x21, 0x56},
    {0x22, 0x11},
    {0x23, 0xE9},
    {0x24, 0x2A},
    {0x25, 0x00},
    {0x26, 0x1F},
    {0x2C, 0x81},
    {0x2D, 0x35},
    {0x2E, 0x09},
};

// OOK : PATABLE[0] = niveau bas, PATABLE[1] = niveau haut (10 dBm).
// Ecrire un seul octet laisse l'emetteur muet, FREND0 pointant sur l'index 1.
constexpr uint8_t kPaTable[2] = {0x00, 0xC0};

SPIClass spi(HSPI);
uint32_t sent = 0;
uint32_t received = 0;
uint32_t overflows = 0;
uint32_t ccaRefus = 0;
unsigned long lastTx = 0;

// Le CC1101 signale qu'il est prêt en relâchant MISO après l'abaissement de
// CS. Sans cette attente, les premiers accès renvoient n'importe quoi.
bool waitReady(uint32_t timeoutMs = 50) {
  const uint32_t start = millis();
  while (digitalRead(kPinMiso) == HIGH)
    if (millis() - start > timeoutMs)
      return false;
  return true;
}

void csLow() {
  spi.beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(kPinCs, LOW);
}

void csHigh() {
  digitalWrite(kPinCs, HIGH);
  spi.endTransaction();
}

bool strobe(uint8_t cmd) {
  csLow();
  const bool ok = waitReady();
  if (ok)
    spi.transfer(cmd);
  csHigh();
  return ok;
}

bool writeReg(uint8_t addr, uint8_t value) {
  csLow();
  const bool ok = waitReady();
  if (ok) {
    spi.transfer(addr);
    spi.transfer(value);
  }
  csHigh();
  return ok;
}

// Les registres d'état exigent le bit de rafale, sans quoi la puce renvoie
// une valeur périmée.
bool readStatus(uint8_t addr, uint8_t &out) {
  csLow();
  const bool ok = waitReady();
  if (ok) {
    spi.transfer(static_cast<uint8_t>(addr | kRead | kBurst));
    out = spi.transfer(0x00);
  }
  csHigh();
  return ok;
}

bool readReg(uint8_t addr, uint8_t &out) {
  csLow();
  const bool ok = waitReady();
  if (ok) {
    spi.transfer(static_cast<uint8_t>(addr | kRead));
    out = spi.transfer(0x00);
  }
  csHigh();
  return ok;
}

int8_t rssiToDbm(uint8_t raw) {
  const int rssi = raw >= 128 ? (static_cast<int>(raw) - 256) / 2
                              : static_cast<int>(raw) / 2;
  return static_cast<int8_t>(rssi - 74);
}

// A 9,6 kbit/s un octet arrive toutes les 830 us. Deux lectures de RXBYTES
// coup sur coup sont separees de ~40 us : elles concordent donc meme en
// pleine reception, et valident un compte encore en train de croitre. On
// lirait alors la FIFO plus vite qu'elle ne se remplit, et la puce renvoie
// indefiniment son dernier octet valide. Il faut espacer les lectures d'au
// moins un temps d'octet pour que la concordance prouve vraiment que plus
// rien n'arrive.
constexpr uint32_t kByteTimeUs = 900;

// Erratum CC1101 : RXBYTES lu pendant que la puce le met a jour renvoie un
// compte corrompu. Deux lectures concordantes, espacees, repondent aux deux
// problemes a la fois.
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
  return false; // compte jamais stable : on ne touche pas a la FIFO
}

void flushRxAndListen() {
  strobe(kSIDLE);
  strobe(kSFRX);
  strobe(kSRX);
}

bool sendPacket(const uint8_t *data, uint8_t len) {
  if (len == 0 || len > kMaxPayload)
    return false;

  // Émission en cours : vider la FIFO maintenant tronquerait la trame.
  uint8_t marc = 0;
  if (!readStatus(kMARCSTATE, marc))
    return false;
  const uint8_t state = marc & kMarcMask;
  if (state == kMarcTx || state == kMarcTxEnd)
    return false;

  strobe(kSIDLE);
  strobe(kSFTX);

  csLow();
  if (!waitReady()) {
    csHigh();
    return false;
  }
  spi.transfer(static_cast<uint8_t>(kFIFO | kBurst));
  spi.transfer(len);
  for (uint8_t i = 0; i < len; ++i)
    spi.transfer(data[i]);
  csHigh();

  if (!strobe(kSTX))
    return false;

  // Le CCA peut refuser : la puce ignore alors STX et reste en reception.
  // Rendre vrai ici compterait une trame jamais emise.
  for (int i = 0; i < 8; ++i) {
    uint8_t apres = 0;
    if (!readStatus(kMARCSTATE, apres))
      return false;
    if ((apres & kMarcMask) != kMarcRx)
      return true; // l'emission a demarre
  }
  ++ccaRefus;
  strobe(kSFTX);
  return false;
}

void pollRx() {
  // Ne rien lire tant que la radio n'est pas franchement en reception : juste
  // apres une emission elle traverse plusieurs etats, et la FIFO lue pendant
  // cette transition ne veut rien dire.
  uint8_t marc = 0;
  if (!readStatus(kMARCSTATE, marc) || (marc & kMarcMask) != kMarcRx)
    return;

  uint8_t rxBytes = 0;
  if (!readRxBytes(rxBytes))
    return;

  if (rxBytes & kRxOverflow) {
    ++overflows;
    Serial.printf("RX debordement FIFO (total %lu)\n",
                  static_cast<unsigned long>(overflows));
    flushRxAndListen();
    return;
  }

  // Un paquet complet porte sa longueur plus deux octets d'etat : en dessous
  // de trois octets, rien de ce qui est la ne peut etre une trame.
  if ((rxBytes & kRxCountMask) < 3)
    return;

  uint8_t len = 0;
  if (!readReg(kFIFO, len))
    return;
  if (len == 0 || len > kMaxPayload) {
    uint8_t rssiRaw = 0;
    readStatus(kRSSI, rssiRaw);
    Serial.printf("RX longueur aberrante (%u)  rssi %d dBm\n", len,
                  rssiToDbm(rssiRaw));
    flushRxAndListen();
    return;
  }

  // RXBYTES annonce le paquet complet bien avant qu'il le soit : il a
  // annonce 42 octets alors que la FIFO n'en contenait qu'une dizaine. Aucune
  // confirmation ne rattrape un compteur qui ment de facon constante, et lire
  // en avance deregle le pointeur de la puce, qui renvoie alors son dernier
  // octet en boucle.
  //
  // On n'attend donc plus un compte mais une duree : a 9,6 kbit/s un octet
  // prend 833 us, et la longueur annoncee dit combien il en reste a venir.
  // C'est ce que ferait une ligne GDO, en mieux — faute de l'avoir cablee.
  delayMicroseconds((static_cast<uint32_t>(len) + 2) * kByteTimeUs + 2000);

  uint8_t compteAccepte = 0;
  if (!readRxBytes(compteAccepte) || (compteAccepte & kRxOverflow) ||
      (compteAccepte & kRxCountMask) < len + 2) {
    flushRxAndListen();
    return;
  }
  compteAccepte &= kRxCountMask;

  uint8_t buf[kMaxPayload + 2];
  // Essai : acces simple, une transaction par octet, CS relache entre chaque.
  // Si le pointeur avance ainsi alors qu'il calait en rafale, le probleme
  // etait la transaction groupee et non le contenu de la FIFO.
  csHigh();
  const size_t aLire = static_cast<size_t>(len) + 2;
  for (size_t i = 0; i < aLire; ++i)
    if (!readReg(kFIFO, buf[i])) {
      flushRxAndListen();
      return;
    }

  const uint8_t statusRssi = buf[len];
  const uint8_t statusLqi = buf[len + 1];

  if ((statusLqi & kCrcOkBit) == 0) {
    Serial.printf("RX CRC invalide  %u o  compte %u  rssi %d dBm  lqi %u  | ",
                  len, compteAccepte, rssiToDbm(statusRssi),
                  statusLqi & kLqiMask);
    for (uint8_t i = 0; i < len; ++i)
      Serial.printf("%02X ", buf[i]);
    Serial.print("| ");
    for (uint8_t i = 0; i < len; ++i)
      Serial.print(buf[i] >= 0x20 && buf[i] < 0x7F ? static_cast<char>(buf[i])
                                                   : '.');
    Serial.println();
    flushRxAndListen();
    return;
  }

  ++received;

  // ── trame de banc : repondre, pas afficher ──
  if (len >= kBenchHeader) {
    uint16_t magic = 0;
    memcpy(&magic, buf + 0, sizeof(magic));
    if (magic == kBenchMagic && buf[2] == kBenchVersion) {
      benchDernier = millis();
      if (buf[3] == kBenchModeFlood) {
        // Saturation : on compte seulement. Repondre doublerait le trafic
        // sur un lien semi-duplex et fausserait la mesure de debit.
        ++benchFlood;
      } else {
        buf[16] = static_cast<uint8_t>(rssiToDbm(statusRssi));
        buf[17] = static_cast<uint8_t>(statusLqi & kLqiMask);
        buf[18] = kBenchFarValid;
        if (sendPacket(buf, len))
          ++benchEchoes;
      }
      return;
    }
  }

  Serial.printf("RX  %u o  rssi %d dBm  lqi %u  | ", len,
                rssiToDbm(statusRssi), statusLqi & kLqiMask);
  for (uint8_t i = 0; i < len; ++i)
    Serial.print(buf[i] >= 0x20 && buf[i] < 0x7F ? static_cast<char>(buf[i])
                                                 : '.');
  Serial.println();
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(kPinCs, OUTPUT);
  digitalWrite(kPinCs, HIGH);
  // CS a -1 : on le pilote uniquement en logiciel. Le confier au peripherique
  // en meme temps lui fait relever CS en cours de transaction, ce qui coupe
  // les acces en rafale du CC1101 — octets perdus a l'ecriture, pointeur de
  // FIFO bloque a la lecture.
  spi.begin(kPinSck, kPinMiso, kPinMosi, -1);

  Serial.println();
  Serial.println("=== Liaison CC1101 — station sol ===");

  if (!strobe(kSRES)) {
    Serial.println("ECHEC: la puce ne relache jamais MISO (CS, 3V3, masse).");
    return;
  }
  delay(100);

  // Vérifier l'identité avant de configurer : sinon on programme du vide et
  // la liaison reste muette sans qu'on sache pourquoi.
  uint8_t partnum = 0xFF, version = 0xFF;
  readStatus(kPARTNUM, partnum);
  readStatus(kVERSION, version);
  if (partnum != 0x00 || version != 0x14) {
    Serial.printf("ECHEC: PARTNUM=0x%02X VERSION=0x%02X, attendu 0x00/0x14\n",
                  partnum, version);
    return;
  }
  Serial.println("puce reconnue (0x00 / 0x14)");

  for (const auto &reg : kConfig)
    writeReg(reg.addr, reg.value);
  csLow();
  if (waitReady()) {
    spi.transfer(static_cast<uint8_t>(kPATABLE | kBurst));
    spi.transfer(kPaTable[0]);
    spi.transfer(kPaTable[1]);
  }
  csHigh();

  flushRxAndListen();
  Serial.println("configuree : 433.88 MHz, ASK/OOK, 9.6 kbit/s, 10 dBm, SPI 2 MHz");
  Serial.println("emission d'un compteur chaque seconde, ecoute en continu\n");
}

void loop() {
  pollRx();

  const unsigned long now = millis();

  if (benchActif()) {
    if (now - benchDernierBilan >= 1000) {
      benchDernierBilan = now;
      Serial.printf("BANC  echos %lu   saturation %lu   cca %lu\n",
                    static_cast<unsigned long>(benchEchoes),
                    static_cast<unsigned long>(benchFlood),
                    static_cast<unsigned long>(ccaRefus));
    }
    delay(2);
    return;   // ne pas emettre : le banc occupe le lien
  }

  if (now - lastTx >= 500) {   // 2 paquets/s
    lastTx = now;
    char msg[32];
    const int len = snprintf(msg, sizeof(msg), "GCS-%lu",
                             static_cast<unsigned long>(sent));
    if (len > 0 && sendPacket(reinterpret_cast<const uint8_t *>(msg),
                              static_cast<uint8_t>(len))) {
      ++sent;
      Serial.printf("TX  %s\n", msg);
    } else {
      Serial.println("TX echec");
    }
  }

  delay(20); // 50 Hz d'interrogation, faute de ligne GDO cablee
}
