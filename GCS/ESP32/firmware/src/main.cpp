// Firmware "bête" du boîtier GCS : aucune décision ici (quel code allume quel
// voyant, priorité d'alarme...), tout ça vit côté Node.js
// (GCS/serial-bridge.js). Ce firmware fait seulement :
//   - lit le port série USB, applique l'état de voyants demandé
//   - lit les boutons, remonte les appuis sur le port série
//
// Protocole série (ASCII, une commande par ligne, '\n' terminé) :
//   PC -> ESP32 :  LED:label1,label2,...\n
//                  Etat COMPLET des voyants actifs (liste vide = tout eteint) --
//                  pas de delta, l'ESP32 ne garde aucun etat metier.
//   ESP32 -> PC :  BTN:label\n
//                  Envoye une fois par appui (front descendant, debounce
//                  logiciel), jamais sur le relachement.
//
// Cette carte porte AUSSI la radio CC1101 du lien secondaire (SDL). Le
// firmware en est le pont : il ne decode aucun paquet de telemetrie, il
// transporte des octets. Toute la logique reste cote Node.js.
//
//   PC -> ESP32 :  TX:<hex>\n    emettre, en respectant le CCA
//                  TXF:<hex>\n   emettre en FORCANT, CCA contourne
//                  Le second est reserve aux ordres ou rester muet est pire
//                  qu'une collision (arret d'urgence).
//   ESP32 -> PC :  RX:<hex>:<rssi>:<lqi>\n   trame recue et sa qualite
//                  LQ:<rssi>:<lqi>:<crc>:<len>:<ovf>:<cca>:<forced>\n
//                  sante du lien, une fois par seconde
//                  TXERR:<raison>\n  emission refusee (canal occupe, trame
//                  invalide) -- jamais un echec silencieux
//
// L'hexadecimal double le volume mais le lien SDL ne porte que ~5 trames de
// 50 octets par seconde : a 115200 bauds c'est negligeable, et une trace au
// moniteur serie reste lisible a l'oeil. Le gain d'un cadrage binaire ne
// justifierait pas la perte de lisibilite.
//
// Cablage : voir GCS/ESP32/mapingESP.csv -> genere esp_map_generated.h (ne
// pas editer ce header a la main, cf. GCS/ESP32/gen_esp_map.py).

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <Wire.h>

#include "cc1101_radio.hpp"
#include "esp_map_generated.h"

Adafruit_MCP23X17 mcp[sizeof(kMcpAddresses) / sizeof(kMcpAddresses[0])];

// Debounce bouton : ignore les rebonds mecaniques pendant ce delai apres un
// front descendant detecte (pas de la "logique metier", juste du filtrage
// electrique -- meme un firmware "bete" doit filtrer ca).
constexpr unsigned long kDebounceMs = 30;

bool buttonWasPressed[kNumButtons] = {};
unsigned long buttonLastChangeMs[kNumButtons] = {};

String serialBuffer;

// HSPI : le bus SPI de la radio, distinct de l'I2C des MCP23017.
SPIClass radioSpi(HSPI);
cc1101::Radio radio(radioSpi);
bool radioPresente = false;
unsigned long derniereSanteMs = 0;

constexpr unsigned long kPeriodeSanteMs = 1000;

uint8_t hexNibble(char c) {
  if (c >= '0' && c <= '9')
    return static_cast<uint8_t>(c - '0');
  if (c >= 'a' && c <= 'f')
    return static_cast<uint8_t>(c - 'a' + 10);
  if (c >= 'A' && c <= 'F')
    return static_cast<uint8_t>(c - 'A' + 10);
  return 0xFF;
}

// Renvoie la longueur decodee, ou 0 si l'hexa est invalide. Une trame mal
// formee est refusee plutot que tronquee : emettre la moitie d'une commande
// est pire que de ne rien emettre.
size_t decodeHex(const String &hex, uint8_t *out, size_t capacite) {
  const size_t n = hex.length();
  if (n == 0 || (n % 2) != 0 || (n / 2) > capacite)
    return 0;
  for (size_t i = 0; i < n; i += 2) {
    const uint8_t hi = hexNibble(hex[i]);
    const uint8_t lo = hexNibble(hex[i + 1]);
    if (hi == 0xFF || lo == 0xFF)
      return 0;
    out[i / 2] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return n / 2;
}

void printHex(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] < 0x10)
      Serial.print('0');
    Serial.print(data[i], HEX);
  }
}

// Trame en attente d'emission.
//
// MESURE : emettre depuis l'interieur de la boucle qui vide le port serie
// echoue au-dela de ~12 octets — 0 trame acceptee sur 13 a 32 octets. La
// meme trame, emise quelques millisecondes plus tard, passe a 12 sur 14. Le
// defaut est donc une question de TIMING, pas de donnees : l'interruption
// UART est encore active pendant l'ecriture de la FIFO d'emission.
//
// On separe donc l'analyse de la commande de son execution : pollSerial()
// ne fait que deposer la trame, et loop() l'emet apres avoir fini de vider
// le port. C'est aussi la bonne forme en soi — une boucle de lecture d'entree
// n'a pas a porter une operation longue.
uint8_t txEnAttente[cc1101::kMaxPayload];
size_t txEnAttenteLen = 0;
bool txEnAttenteForce = false;

// NOTE : un delai artificiel avant l'emission a ete essaye (5, 20, 60 ms).
// Les resultats sont incoherents entre eux — 4/15, 0/12, 0/12 — et des
// echantillons de cette taille ne permettent pas de conclure. Aucun delai
// n'est conserve : il n'etait justifie que par du bruit.

// `forcer` contourne le CCA. Toute issue est rapportee : un echec silencieux
// ferait croire au sol que l'ordre est parti.
void handleTx(const String &hex, bool forcer) {
  if (!radioPresente) {
    Serial.println("TXERR:radio_absente");
    return;
  }
  uint8_t trame[cc1101::kMaxPayload];
  const size_t len = decodeHex(hex, trame, sizeof(trame));
  if (len == 0) {
    Serial.println("TXERR:hex_invalide");
    return;
  }
  // Une trame deja en attente est ecrasee : le sol retransmet de lui-meme, et
  // garder une file ici melangerait l'ordre des ordres critiques.
  if (txEnAttenteLen > 0)
    Serial.println("TXERR:ecrasee");
  memcpy(txEnAttente, trame, len);
  txEnAttenteLen = len;
  txEnAttenteForce = forcer;
}

// Emet la trame deposee par pollSerial(), hors du contexte de l'interruption
// UART. Appele depuis loop().
void pollTx() {
  if (txEnAttenteLen == 0)
    return;
  const size_t len = txEnAttenteLen;
  const bool forcer = txEnAttenteForce;
  txEnAttenteLen = 0;

  const bool ok = forcer
                      ? radio.sendForced(txEnAttente, static_cast<uint8_t>(len))
                      : radio.send(txEnAttente, static_cast<uint8_t>(len));
  if (!ok)
    Serial.println(forcer ? "TXERR:echec_force" : "TXERR:canal_occupe");
}

void pollRadio() {
  if (!radioPresente)
    return;


  uint8_t trame[cc1101::kMaxPayload];
  const uint8_t len = radio.poll(trame);
  if (len > 0) {
    const auto &st = radio.stats();
    Serial.print("RX:");
    printHex(trame, len);
    Serial.print(':');
    Serial.print(st.lastRssiDbm);
    Serial.print(':');
    Serial.println(st.lastLqi);
  }

  const unsigned long maintenant = millis();
  if (maintenant - derniereSanteMs >= kPeriodeSanteMs) {
    derniereSanteMs = maintenant;
    const auto &st = radio.stats();
    // Qualite absente tant que rien n'a ete recu : on envoie 0 mais le sol
    // sait distinguer grace aux compteurs, qui restent a zero eux aussi.
    Serial.print("LQ:");
    Serial.print(st.haveQuality ? st.lastRssiDbm : 0);
    Serial.print(':');
    Serial.print(st.haveQuality ? st.lastLqi : 0);
    Serial.print(':');
    Serial.print(st.crcErrors);
    Serial.print(':');
    Serial.print(st.lengthErrors);
    Serial.print(':');
    Serial.print(st.overflows);
    Serial.print(':');
    Serial.print(st.ccaRefusals);
    Serial.print(':');
    Serial.println(st.forcedSends);
  }
}

void applyLedState(const String &labelsCsv) {
  // Repart de zero a chaque commande : LED:... porte l'etat COMPLET voulu,
  // pas un delta (cf. protocole en tete de fichier).
  for (const auto &led : kLeds) {
    bool on = false;
    int start = 0;
    while (start < static_cast<int>(labelsCsv.length())) {
      int comma = labelsCsv.indexOf(',', start);
      if (comma < 0) comma = labelsCsv.length();
      if (labelsCsv.substring(start, comma) == led.label) {
        on = true;
        break;
      }
      start = comma + 1;
    }

    if (led.kind == LedKind::MCP) {
      mcp[led.mcpChipIndex].digitalWrite(led.mcpPin, on ? HIGH : LOW);
    } else {
      digitalWrite(led.gpio, on ? HIGH : LOW);
    }
  }
}

void handleLine(const String &line) {
  if (line.startsWith("LED:")) {
    applyLedState(line.substring(4));
  } else if (line.startsWith("TXF:")) {
    handleTx(line.substring(4), true);
  } else if (line.startsWith("TX:")) {
    handleTx(line.substring(3), false);
  }
  // Commande inconnue : ignoree silencieusement -- firmware "bete", pas de
  // remontee d'erreur cote protocole pour l'instant (POC).
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for (size_t i = 0; i < sizeof(kMcpAddresses) / sizeof(kMcpAddresses[0]); ++i) {
    mcp[i].begin_I2C(kMcpAddresses[i]);
    for (uint8_t pin = 0; pin < 16; ++pin) {
      mcp[i].pinMode(pin, OUTPUT);
      mcp[i].digitalWrite(pin, LOW);
    }
  }

  for (const auto &led : kLeds) {
    if (led.kind == LedKind::DIRECT) {
      pinMode(led.gpio, OUTPUT);
      digitalWrite(led.gpio, LOW);
    }
  }

  for (const auto &btn : kButtons) {
    pinMode(btn.gpio, INPUT_PULLUP);
  }

  // La radio est optionnelle : le panneau doit fonctionner meme si le module
  // est absent ou mal cable. On le dit une fois, puis on continue.
  radioPresente = radio.begin();
  Serial.println(radioPresente ? "RADIO:ok" : "RADIO:absente");
}

void pollButtons() {
  unsigned long now = millis();
  for (size_t i = 0; i < kNumButtons; ++i) {
    // Actif bas (INPUT_PULLUP) : appuye = LOW.
    bool pressed = digitalRead(kButtons[i].gpio) == LOW;

    if (pressed != buttonWasPressed[i] && (now - buttonLastChangeMs[i]) > kDebounceMs) {
      buttonLastChangeMs[i] = now;
      buttonWasPressed[i] = pressed;
      if (pressed) {
        Serial.print("BTN:");
        Serial.println(kButtons[i].label);
      }
    }
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) handleLine(serialBuffer);
      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
    }
  }
}

void loop() {
  pollSerial();
  pollButtons();
  // La reception radio bloque jusqu'a ~50 ms le temps qu'un paquet arrive
  // entierement (faute de ligne GDO). Les boutons sont donc scrutes un peu
  // moins souvent — imperceptible a l'echelle d'un appui humain, et le
  // debounce de 30 ms n'en souffre pas puisqu'il mesure un ecart de temps,
  // pas un nombre de tours de boucle.
  pollRadio();
  // Hors de la boucle de lecture serie : une boucle qui vide une entree n'a
  // pas a porter une operation longue. L'ordre vis-a-vis de pollRadio n'a en
  // revanche PAS d'effet demontre — les mesures qui semblaient le montrer ne
  // se sont pas reproduites.
  pollTx();
}
