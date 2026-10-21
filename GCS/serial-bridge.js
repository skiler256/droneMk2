// Pont USB série <-> ESP32 (boîtier GCS, firmware "bête" -- cf. GCS/README.md §2).
//
// Protocole (ASCII, une commande par ligne '\n') :
//   -> ESP32 : LED:label1,label2,...\n   (état COMPLET des voyants actifs)
//   <- ESP32 : BTN:label\n               (un appui bouton, front descendant)
//
// La même carte porte la radio CC1101 du lien secondaire (SDL), dont ce pont
// est le transport. Les trames de télémétrie passent en hexadécimal : le lien
// ne porte que ~5 trames de 50 octets par seconde, donc doubler le volume est
// sans conséquence à 115200 bauds, et une trace reste lisible à l'œil.
//
//   -> ESP32 : TX:<hex>\n     émettre en respectant le CCA
//              TXF:<hex>\n    émettre en FORÇANT (CCA contourné)
//   <- ESP32 : RX:<hex>:<rssi>:<lqi>\n            trame reçue
//              LQ:<rssi>:<lqi>:<crc>:<len>:<ovf>:<cca>:<forced>\n
//              TXERR:<raison>\n                   émission refusée
//              RADIO:ok | RADIO:absente\n         au démarrage

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
    // Prouvée par les lignes LQ reçues, pas par l'annonce de démarrage.
    this.radioReady = false;
    this._lastLq = 0;
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
      return;
    }
    if (line.startsWith("RX:")) {
      // RX:<hex>:<rssi>:<lqi>
      const parts = line.slice(3).split(":");
      const buf = Buffer.from(parts[0], "hex");
      // Un hex tronqué donnerait un Buffer silencieusement court : on vérifie
      // plutôt que de livrer une trame amputée au décodeur.
      if (buf.length * 2 !== parts[0].length) {
        console.error("GCS: trame RX hexadécimale invalide, ignorée");
        return;
      }
      this.emit("frame", {
        data: buf,
        rssi: Number(parts[1]),
        lqi: Number(parts[2]),
      });
      return;
    }
    if (line.startsWith("LQ:")) {
      // Le firmware n'émet LQ que si la radio répond : recevoir cette ligne
      // EST la preuve qu'elle est là. On ne peut pas se fier au seul RADIO:ok
      // du démarrage — le serveur se connecte presque toujours après le boot
      // de l'ESP32 et manquerait l'annonce, croirait la radio absente, et
      // enverrait les commandes SDL dans le vide.
      this.radioReady = true;
      this._lastLq = Date.now();
      const [rssi, lqi, crc, len, ovf, cca, forced] = line
        .slice(3)
        .split(":")
        .map(Number);
      this.emit("quality", {
        rssi,
        lqi,
        crcErrors: crc,
        lengthErrors: len,
        overflows: ovf,
        ccaRefusals: cca,
        forcedSends: forced,
      });
      return;
    }
    if (line.startsWith("TXERR:")) {
      // Jamais silencieux : une émission refusée doit se savoir, sinon on
      // croit l'ordre parti.
      this.emit("txerror", line.slice(6));
      return;
    }
    if (line.startsWith("RADIO:")) {
      this.radioReady = line.slice(6) === "ok";
      this.emit("radio", this.radioReady);
      return;
    }
    // Ligne inconnue : ignorée (firmware "bête").
  }

  /**
   * Émet une trame de télémétrie par la radio SDL.
   * @param {Buffer} frame
   * @param {boolean} forced contourne le CCA (arrêt d'urgence)
   * @returns {boolean} false si le port n'est pas ouvert — l'appelant doit en
   *   tenir compte plutôt que de supposer que la trame est partie.
   */
  sendFrame(frame, forced = false) {
    if (!this.port || !this.port.isOpen) return false;
    // Le firmware publie sa santé chaque seconde : trois secondes de silence
    // et on ne considère plus la radio comme disponible.
    if (this._lastLq && Date.now() - this._lastLq > 3000) {
      this.radioReady = false;
      return false;
    }
    const prefixe = forced ? "TXF:" : "TX:";
    this.port.write(`${prefixe}${frame.toString("hex")}\n`, (err) => {
      if (err) console.error("GCS: écriture trame SDL échouée:", err.message);
    });
    return true;
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
