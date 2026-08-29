// GCS/serial-bridge.js
// ─────────────────────────────────────────────────────────────────────────────
// Pont USB série <-> ESP32 (boîtier GCS). Le firmware (GCS/ESP32/firmware) est
// volontairement "bête" : aucune décision ici côté ESP32, tout vit ici.
//
// Protocole (ASCII, une commande par ligne '\n', cf. main.cpp) :
//   -> ESP32 : LED:label1,label2,...\n   (état COMPLET des voyants actifs)
//   <- ESP32 : BTN:label\n               (un appui bouton, front descendant)

const { SerialPort } = require("serialport");
const { ReadlineParser } = require("@serialport/parser-readline");
const { EventEmitter } = require("events");

class SerialBridge extends EventEmitter {
  /**
   * @param {string} path ex. "/dev/ttyUSB0"
   * @param {number} baudRate
   */
  constructor(path, baudRate = 115200) {
    super();
    this.path = path;
    this.baudRate = baudRate;
    this.port = null;
    this._retryMs = 1000;
    this._lastLeds = null; // évite de spammer le port si l'état ne change pas
    this._open();
  }

  _open() {
    const port = new SerialPort({ path: this.path, baudRate: this.baudRate }, (err) => {
      if (err) {
        console.error(`GCS: liaison ESP32 (${this.path}) indisponible: ${err.message}`);
        this._scheduleRetry();
      }
    });
    this.port = port;

    const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));
    parser.on("data", (line) => this._handleLine(line.trim()));

    port.on("open", () => {
      this._retryMs = 1000;
      this._lastLeds = null; // renvoyer l'état complet après une reconnexion
      console.log(`GCS: liaison ESP32 établie sur ${this.path}`);
      this.emit("connected");
    });

    // "close" couvre aussi bien un débranchement qu'une erreur port fermé.
    port.on("close", () => {
      this.emit("disconnected");
      this._scheduleRetry();
    });
    port.on("error", (err) => {
      console.error(`GCS: erreur port série ESP32: ${err.message}`);
    });
  }

  _scheduleRetry() {
    // Backoff plafonné : le port peut rester absent longtemps (ESP32
    // débranché), pas la peine de marteler l'ouverture.
    this._retryMs = Math.min(this._retryMs * 2, 8000);
    setTimeout(() => this._open(), this._retryMs);
  }

  _handleLine(line) {
    if (line.startsWith("BTN:")) {
      this.emit("button", line.slice(4));
    }
    // Ligne inconnue : ignorée (firmware "bête", pas d'autre trafic prévu).
  }

  /**
   * Envoie l'état COMPLET des voyants actifs (pas de delta : plus simple,
   * plus robuste à une reconnexion ou un paquet perdu).
   * @param {string[]} labels
   */
  setLeds(labels) {
    const key = [...labels].sort().join(",");
    if (key === this._lastLeds) return; // pas de changement, pas d'écriture
    this._lastLeds = key;

    if (!this.port || !this.port.isOpen) return; // pas connecté : rien à faire
    this.port.write(`LED:${key}\n`, (err) => {
      if (err) console.error("GCS: écriture ESP32 échouée:", err.message);
    });
  }
}

module.exports = { SerialBridge };
