// GCS/ESP32/firmware/src/main.cpp
// ─────────────────────────────────────────────────────────────────────────────
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
// Cablage : voir GCS/ESP32/mapingESP.csv -> genere esp_map_generated.h (ne
// pas editer ce header a la main, cf. GCS/ESP32/gen_esp_map.py).

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <Wire.h>

#include "esp_map_generated.h"

Adafruit_MCP23X17 mcp[sizeof(kMcpAddresses) / sizeof(kMcpAddresses[0])];

// Debounce bouton : ignore les rebonds mecaniques pendant ce delai apres un
// front descendant detecte (pas de la "logique metier", juste du filtrage
// electrique -- meme un firmware "bete" doit filtrer ca).
constexpr unsigned long kDebounceMs = 30;

bool buttonWasPressed[kNumButtons] = {};
unsigned long buttonLastChangeMs[kNumButtons] = {};

String serialBuffer;

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
}
