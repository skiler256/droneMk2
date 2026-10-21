// Integrite du bus SPI entre l'ESP32 et le CC1101 — aucune radio en jeu.
//
// Le lien perd des octets dans les deux sens : a l'ecriture de la FIFO
// d'emission (40 octets ecrits, 17 emis) et a la lecture de celle de
// reception (pointeur bloque). Les ecritures de registres isoles, elles,
// se relisent juste. Le soupcon porte donc sur les acces en rafale.
//
// PATABLE fait 8 octets et accepte la rafale dans les deux sens : c'est le
// banc d'essai ideal, sans interference de la radio. On ecrit un motif, on
// le relit, on compte les ecarts — et on recommence a plusieurs vitesses
// d'horloge pour voir si le taux d'erreur en depend.

#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr int kPinSck = 14, kPinMiso = 35, kPinMosi = 13, kPinCs = 4;
constexpr uint8_t kRead = 0x80, kBurst = 0x40;
constexpr uint8_t kSRES = 0x30, kSIDLE = 0x36;
constexpr uint8_t kPARTNUM = 0x30, kVERSION = 0x31, kPATABLE = 0x3E;

SPIClass spi(HSPI);
uint32_t vitesse = 500000;

bool waitReady(uint32_t t = 50) {
  const uint32_t s = millis();
  while (digitalRead(kPinMiso) == HIGH)
    if (millis() - s > t)
      return false;
  return true;
}
void csLow() {
  spi.beginTransaction(SPISettings(vitesse, MSBFIRST, SPI_MODE0));
  digitalWrite(kPinCs, LOW);
}
void csHigh() {
  digitalWrite(kPinCs, HIGH);
  spi.endTransaction();
}
void strobe(uint8_t c) {
  csLow();
  if (waitReady())
    spi.transfer(c);
  csHigh();
}
uint8_t readStatus(uint8_t a) {
  uint8_t o = 0xFF;
  csLow();
  if (waitReady()) {
    spi.transfer(static_cast<uint8_t>(a | kRead | kBurst));
    o = spi.transfer(0x00);
  }
  csHigh();
  return o;
}

// Ecriture puis relecture de PATABLE en rafale. Renvoie le nombre d'octets
// qui diffèrent.
int allerRetour(const uint8_t *motif, uint8_t *relu) {
  strobe(kSIDLE);
  csLow();
  if (!waitReady()) {
    csHigh();
    return -1;
  }
  spi.transfer(static_cast<uint8_t>(kPATABLE | kBurst));
  for (int i = 0; i < 8; ++i)
    spi.transfer(motif[i]);
  csHigh();

  csLow();
  if (!waitReady()) {
    csHigh();
    return -1;
  }
  spi.transfer(static_cast<uint8_t>(kPATABLE | kRead | kBurst));
  for (int i = 0; i < 8; ++i)
    relu[i] = spi.transfer(0x00);
  csHigh();

  int ecarts = 0;
  for (int i = 0; i < 8; ++i)
    if (relu[i] != motif[i])
      ++ecarts;
  return ecarts;
}

void campagne(uint32_t hz, int tours) {
  vitesse = hz;
  int trames = 0, octets = 0, erreursSpi = 0;
  uint8_t motif[8], relu[8];
  uint8_t premier[8] = {0}, premierRelu[8] = {0};
  bool capture = false;

  for (int n = 0; n < tours; ++n) {
    for (int i = 0; i < 8; ++i)
      motif[i] = static_cast<uint8_t>(random(256));
    const int e = allerRetour(motif, relu);
    if (e < 0) {
      ++erreursSpi;
      continue;
    }
    if (e > 0) {
      ++trames;
      octets += e;
      if (!capture) {
        memcpy(premier, motif, 8);
        memcpy(premierRelu, relu, 8);
        capture = true;
      }
    }
  }
  Serial.printf("  %7lu Hz : %4d/%d aller-retours fautifs, %d octet(s) "
                "errone(s), %d acces impossibles\n",
                static_cast<unsigned long>(hz), trames, tours, octets,
                erreursSpi);
  if (capture) {
    Serial.print("            premier ecart  ecrit :");
    for (int i = 0; i < 8; ++i)
      Serial.printf(" %02X", premier[i]);
    Serial.print("\n                           relu  :");
    for (int i = 0; i < 8; ++i)
      Serial.printf(" %02X", premierRelu[i]);
    Serial.println();
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(600);
  pinMode(kPinCs, OUTPUT);
  digitalWrite(kPinCs, HIGH);
  // CS pilote en logiciel seul : confie au peripherique, il serait releve en
  // cours de transaction et couperait les acces en rafale.
  spi.begin(kPinSck, kPinMiso, kPinMosi, -1);

  Serial.println("\n=== Integrite SPI ESP32 <-> CC1101 (sans radio) ===");
  strobe(kSRES);
  delay(100);
  Serial.printf("PARTNUM 0x%02X  VERSION 0x%02X  (attendu 0x00 / 0x14)\n\n",
                readStatus(kPARTNUM), readStatus(kVERSION));
  Serial.println("aller-retour sur PATABLE (8 octets, rafale) :");

  for (uint32_t hz : {100000u, 500000u, 1000000u, 2000000u, 4000000u})
    campagne(hz, 500);

  Serial.println("\nUn taux d'erreur nul partout disculpe le bus SPI.");
}

void loop() { delay(1000); }
