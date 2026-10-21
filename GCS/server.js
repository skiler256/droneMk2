// Sert le dashboard statique (public/) et relaie la télémétrie UDP <->
// WebSocket dans les deux sens. Architecture, ports, limite de duplication
// avec tools/dashboard/dashboard_server.py : cf. GCS/README.md §1.

const { execFile } = require("child_process");
const dgram = require("dgram");
const express = require("express");
const path = require("path");
const { WebSocketServer } = require("ws");
const { loadPackets, decode, encode } = require("./telemetry-codec");
const { loadCodesTable } = require("./codes-table");
const { SerialBridge } = require("./serial-bridge");

const PORT = Number(process.env.GCS_HTTP_PORT) || 3000;

// Boîtier ESP32 (voyants + boutons), cf. GCS/README.md §2.
const ESP32_PORT = process.env.GCS_ESP32_PORT || "/dev/ttyUSB0";

// cf. src/main.cpp : kUdpTelMainLocalPort/PeerPort, kUdpTelSecLocalPort/PeerPort.
// Surchargeables par l'environnement, comme GCS_ESP32_PORT : permet de faire
// tourner une instance de test a cote du dashboard sans conflit de port.
const envPort = (nom, defaut) => Number(process.env[nom]) || defaut;

const TELMAIN_RX_PORT = envPort("GCS_TELMAIN_RX", 5602); // le drone ENVOIE ici
const TELMAIN_TX_PORT = envPort("GCS_TELMAIN_TX", 5601); // le drone ÉCOUTE ici
const TELSEC_RX_PORT = envPort("GCS_TELSEC_RX", 5612);
const TELSEC_TX_PORT = envPort("GCS_TELSEC_TX", 5611);

const TELEMETRY_CSV = path.join(__dirname, "..", "data", "TELEMETRY.csv");
const { byId, byName } = loadPackets(TELEMETRY_CSV);

const INDEX_CSV = path.join(__dirname, "..", "data", "INDEX.csv");
const codesTable = loadCodesTable(INDEX_CSV);

const serial = new SerialBridge(ESP32_PORT);
// Santé du lien SDL, publiée une fois par seconde par le firmware. Alimente
// les antennes du schéma du drone, qui étaient grises faute de données.
const sdlQuality = {
  rssi: null, lqi: null, crcErrors: 0, lengthErrors: 0,
  overflows: 0, ccaRefusals: 0, forcedSends: 0,
};

// Trames SDL reçues par la radio : même chemin que l'UDP.
serial.on("frame", ({ data, rssi, lqi }) => {
  sdlQuality.rssi = rssi;
  sdlQuality.lqi = lqi;
  handleTelemetryFrame(data, "sec");
});

serial.on("quality", (q) => {
  Object.assign(sdlQuality, q);
  broadcast({ type: "link_quality", link: "sec", ...q, ts: Date.now() });
});

serial.on("txerror", (raison) => {
  console.error(`GCS: émission SDL refusée (${raison})`);
  broadcast({ type: "sdl_txerror", reason: raison, ts: Date.now() });
});

serial.on("radio", (ok) => {
  console.log(ok ? "GCS: radio CC1101 détectée" : "GCS: radio CC1101 absente");
});

serial.on("button", (label) => {
  console.log(`GCS: bouton ESP32 pressé: ${label}`);
  if (label === "MW_ACK") ackedMW = true;
  if (label === "MC_ACK") ackedMC = true;
  updateLeds();
  broadcast({ type: "esp_button", label });
});

// Codes CODE actuellement actifs (packed -> Panel[]) -- ground truth
// indépendante des navigateurs connectés : le boîtier doit refléter l'état
// réel même sans page ouverte. Recalculé à chaque paquet CODE, pas de delta
// (cf. SerialBridge.setLeds : état complet envoyé, plus robuste).
const activeCodePanels = new Map(); // packed code -> string[] (labels)

// Les 4 voyants directs (DISARMED/ARMED/HB_Main/HB_Sec, cf. mapingESP.csv) --
// pas des défauts, donc pas dans INDEX.csv::Panel : état/heartbeat séparé.
let armed = false;
let lastRxMain = 0;
let lastRxSec = 0;
const HEARTBEAT_STALE_MS = 3000; // au-delà, le lien est considéré mort -> pas de clignotement
let blinkOn = false;

// Agrégats MW/MC pour le VOYANT ESP32 uniquement (son : alarm.js, ack
// indépendant) -- règle MW_GROUP et sémantique d'ack : GCS/README.md §1.
const MW_GROUP = new Set(["ENG", "PWR", "FC", "EKF", "MAG", "GPS", "NAV", "EMG"]);
let prevFaultLabels = new Set();
let ackedMW = false;
let ackedMC = false;

function computeMasterState(faultLabels) {
  const rising = [...faultLabels].filter((l) => !prevFaultLabels.has(l));
  if (rising.some((l) => MW_GROUP.has(l))) ackedMW = false;
  if (rising.some((l) => !MW_GROUP.has(l))) ackedMC = false;
  prevFaultLabels = faultLabels;

  const mwOn = !ackedMW && [...faultLabels].some((l) => MW_GROUP.has(l));
  const mcOn = !ackedMC && [...faultLabels].some((l) => !MW_GROUP.has(l));
  return { mwOn, mcOn };
}

function stateLeds(now) {
  const s = new Set();
  s.add(armed ? "ARMED" : "DISARMED");
  if (blinkOn && now - lastRxMain < HEARTBEAT_STALE_MS) s.add("HB_Main");
  if (blinkOn && now - lastRxSec < HEARTBEAT_STALE_MS) s.add("HB_Sec");
  return s;
}

function updateLeds() {
  const faultLabels = new Set();
  for (const panel of activeCodePanels.values()) {
    for (const label of panel) faultLabels.add(label);
  }

  const { mwOn, mcOn } = computeMasterState(faultLabels);

  const labels = stateLeds(Date.now());
  for (const label of faultLabels) labels.add(label);
  if (mwOn) labels.add("MW");
  if (mcOn) labels.add("MC");
  serial.setLeds([...labels]);
}

// Clignotement HB_Main/HB_Sec (2 Hz) -- confirme que le process tourne
// vraiment plutôt qu'un voyant fixe qui pourrait rester allumé en cas de blocage.
setInterval(() => {
  blinkOn = !blinkOn;
  updateLeds();
}, 250);

function handleCodePacket(fields) {
  const packed = fields.code >>> 0;
  if (fields.active) {
    const info = codesTable.get(packed);
    activeCodePanels.set(packed, info ? info.panel : []);
  } else {
    activeCodePanels.delete(packed);
  }
  updateLeds();
}

const app = express();
app.use(express.static(path.join(__dirname, "public")));
// Table de codes diagnostics (INDEX.csv) — servie telle quelle, décodée
// côté navigateur (public/codes.js), même source de vérité que
// tools/gen_codes.py.
app.get("/codes.csv", (_req, res) => res.sendFile(path.join(__dirname, "..", "data", "INDEX.csv")));
// Idem pour les définitions d'alarme (cf. tools/alarms_common.py, éditées
// via l'onglet "Alarmes" de codes_editor.py) -- décodées côté navigateur
// par GCS/public/js/alarm.js.
app.get("/alarms.csv", (_req, res) => res.sendFile(path.join(__dirname, "..", "data", "ALARMS.csv")));
app.get("/mission_alarms.csv", (_req, res) => res.sendFile(path.join(__dirname, "..", "data", "MISSION_ALARMS.csv")));

// Volume systeme (PulseAudio), cf. GCS/README.md §4/§5 (le service doit
// tourner sous le meme utilisateur que la session graphique).
function pactl(args, cb) {
  execFile("pactl", args, (err, stdout) => cb(err, (stdout || "").trim()));
}

app.get("/api/volume", (_req, res) => {
  pactl(["get-sink-volume", "@DEFAULT_SINK@"], (err, out) => {
    if (err) { res.status(500).json({ error: err.message }); return; }
    const m = out.match(/(\d+)%/);
    pactl(["get-sink-mute", "@DEFAULT_SINK@"], (err2, out2) => {
      if (err2) { res.status(500).json({ error: err2.message }); return; }
      res.json({ percent: m ? parseInt(m[1], 10) : null, muted: /yes/i.test(out2) });
    });
  });
});

app.post("/api/volume/:percent", (req, res) => {
  const pct = parseInt(req.params.percent, 10);
  if (!Number.isInteger(pct) || pct < 0 || pct > 100) {
    res.status(400).json({ error: "percent doit être un entier 0-100" });
    return;
  }
  pactl(["set-sink-volume", "@DEFAULT_SINK@", `${pct}%`], (err) => {
    if (err) { res.status(500).json({ error: err.message }); return; }
    res.json({ ok: true, percent: pct });
  });
});

app.post("/api/mute/:state", (req, res) => {
  const state = req.params.state === "1" ? "1" : req.params.state === "0" ? "0" : null;
  if (state === null) { res.status(400).json({ error: "state doit être 0 ou 1" }); return; }
  pactl(["set-sink-mute", "@DEFAULT_SINK@", state], (err) => {
    if (err) { res.status(500).json({ error: err.message }); return; }
    res.json({ ok: true, muted: state === "1" });
  });
});

const server = app.listen(PORT, () => {
  console.log(`GCS dashboard sur http://localhost:${PORT}`);
});

const wss = new WebSocketServer({ server });

function broadcast(msg) {
  const text = JSON.stringify(msg);
  for (const client of wss.clients) {
    if (client.readyState === client.OPEN) client.send(text);
  }
}

// Traitement commun aux deux transports : une trame SDL arrivée par la radio
// doit suivre exactement le même chemin qu'une trame arrivée en UDP, sinon
// l'acquittement et le tableau de bord se comporteraient différemment selon
// le transport.
function handleTelemetryFrame(data, link) {
  const pkt = decode(byId, data);
  if (!pkt) return; // trame corrompue/inconnue : on droppe, pas de crash

  if (link === "main") lastRxMain = Date.now();
  else lastRxSec = Date.now();

  if (pkt.name === "CODE") handleCodePacket(pkt.fields);
  if (pkt.name === "CMD_ACK") handleAck(pkt.fields, link);
  if (pkt.name === "ARM_STATE") {
    armed = !!pkt.fields.armed;
    updateLeds();
  }
  broadcast({ type: "telemetry", link, name: pkt.name, fields: pkt.fields, ts: Date.now() });
}

function listenTelemetry(port, link) {
  const sock = dgram.createSocket("udp4");
  sock.on("message", (data) => {
    const pkt = decode(byId, data);
    if (!pkt) return; // trame corrompue/inconnue : on droppe, pas de crash
    handleTelemetryFrame(data, link);
  });
  sock.on("error", (err) => console.error(`GCS: erreur socket UDP ${link} (${port}):`, err.message));
  sock.bind(port, "127.0.0.1");
  return sock;
}

const mainRxSock = listenTelemetry(TELMAIN_RX_PORT, "main");
const secRxSock = listenTelemetry(TELSEC_RX_PORT, "sec");

const txSock = dgram.createSocket("udp4");

// ─────────────── Retransmission des commandes acquittées ───────────────
//
// Une liaison radio perd des trames : c'est normal et ça ne se corrige pas
// au niveau physique. La fiabilité d'une commande vient d'ici — on réémet
// jusqu'à recevoir l'acquittement du drone, ou on prévient l'opérateur.
//
// Le drone acquitte sur le lien d'arrivée, et PDL/SDL sont indépendants : on
// suit donc un envoi en cours par couple (lien, paquet).
//
// Le drone déduplique par `seq` : une retransmission d'une commande qu'il a
// déjà appliquée est réacquittée sans être rejouée. C'est ce qui rend la
// retransmission sûre sur un armement ou un waypoint.

// Aller-retour mesuré sur SDL : ~101 ms pour un paquet de 21 octets, et le
// lien ne tient que ~5 trames/s. Un délai plus court réémettrait avant que
// l'acquittement ait eu le temps d'arriver, et saturerait le lien.
const ACK_TIMEOUT_MS = 300;
const ACK_MAX_ATTEMPTS = 4; // fenêtre totale ~1,2 s

// Un arrêt d'urgence réessaie plus vite et plus longtemps : le coût d'une
// trame de plus est sans commune mesure avec celui d'un ordre perdu.
const ACK_TIMEOUT_FORCE_MS = 200;
const ACK_MAX_ATTEMPTS_FORCE = 8;

let nextSeq = 1;
const pendingAcks = new Map(); // "lien:packetId" -> { timer, attempts, ... }

function ackKey(link, packetId) {
  return `${link === "sec" ? "sec" : "main"}:${packetId}`;
}

function clearPending(key) {
  const entry = pendingAcks.get(key);
  if (entry) {
    clearTimeout(entry.timer);
    pendingAcks.delete(key);
  }
}

// Le SDL passe par la radio dès que l'ESP32 est là. Sinon on retombe sur
// l'UDP de développement, ce qui permet de travailler sans matériel — mais le
// basculement est journalisé une fois, pour qu'on ne croie pas émettre par
// radio alors qu'on parle à un bouchon.
let sdlParRadioAnnonce = false;

function rawSend(frame, link, forced = false) {
  if (link === "sec" && serial.radioReady) {
    if (!sdlParRadioAnnonce) {
      console.log("GCS: SDL émis par la radio CC1101 (ESP32)");
      sdlParRadioAnnonce = true;
    }
    if (serial.sendFrame(frame, forced)) return true;
    // Port fermé entre-temps : on le dit plutôt que de perdre la trame en
    // silence, et on laisse la retransmission faire son travail.
    console.error("GCS: pont ESP32 indisponible, trame SDL non émise");
    return false;
  }
  const port = link === "sec" ? TELSEC_TX_PORT : TELMAIN_TX_PORT;
  txSock.send(frame, port, "127.0.0.1");
  return true;
}

function sendCommand(name, fields, link) {
  const meta = byName[name];
  if (!meta) {
    console.error(`GCS: paquet inconnu: ${name}`);
    return false;
  }

  const acquitte = meta.ack === "ACK" || meta.ack === "ACK_FORCE";
  const force = meta.ack === "ACK_FORCE";
  const payload = { ...(fields || {}) };
  let seq = 0;
  if (acquitte) {
    seq = nextSeq++;
    payload.seq = seq;
  }

  let frame;
  try {
    frame = encode(byName, name, payload);
  } catch (e) {
    console.error("GCS: encodage commande échoué:", e.message);
    return false;
  }

  if (!acquitte) {
    rawSend(frame, link, false);
    return true;
  }

  // Un nouvel ordre du même type remplace celui en vol : l'opérateur vient de
  // décider autre chose, réémettre l'ancien n'aurait aucun sens.
  const key = ackKey(link, meta.id);
  clearPending(key);

  const limite = force ? ACK_MAX_ATTEMPTS_FORCE : ACK_MAX_ATTEMPTS;
  const delai = force ? ACK_TIMEOUT_FORCE_MS : ACK_TIMEOUT_MS;

  const essayer = (attempts) => {
    // ACK_FORCE : contourne le CCA de la radio. C'est ici que la colonne Ack
    // de TELEMETRY.csv se traduit en comportement physique.
    rawSend(frame, link, force);
    const timer = setTimeout(() => {
      if (attempts + 1 >= limite) {
        // Abandonner en silence serait le pire des comportements : l'opérateur
        // croirait sa commande passée. On le prévient explicitement.
        pendingAcks.delete(key);
        console.error(
          `GCS: ${name} NON ACQUITTEE apres ${limite} tentatives sur ${link}`);
        broadcast({
          type: "ack",
          state: "failed",
          name,
          link,
          seq,
          attempts: limite,
          ts: Date.now(),
        });
        return;
      }
      essayer(attempts + 1);
    }, delai);
    pendingAcks.set(key, { timer, attempts: attempts + 1, seq, name, link, force });
    broadcast({
      type: "ack",
      state: "pending",
      name,
      link,
      seq,
      attempts: attempts + 1,
      ts: Date.now(),
    });
  };

  essayer(0);
  return true;
}

// Appelé à la réception d'un CMD_ACK. L'acquittement ne vaut que pour le lien
// qui l'apporte, et seulement si le seq correspond : un acquittement en
// retard, pour un ordre déjà remplacé, ne doit pas annuler le nouveau.
function handleAck(fields, link) {
  const key = ackKey(link, fields.packet_id);
  const entry = pendingAcks.get(key);
  if (!entry || entry.seq !== fields.seq) return;

  clearPending(key);
  const nom = byId[fields.packet_id] ? byId[fields.packet_id].name : "?";
  broadcast({
    type: "ack",
    state: fields.result === 1 ? "duplicate" : "confirmed",
    name: nom,
    link,
    seq: fields.seq,
    result: fields.result,
    attempts: entry.attempts,
    ts: Date.now(),
  });
}

wss.on("connection", (ws) => {
  ws.on("message", (raw) => {
    let msg;
    try {
      msg = JSON.parse(raw);
    } catch {
      return; // message mal formé : ignoré, pas de crash du serveur
    }
    if (msg.type === "command" && typeof msg.name === "string") {
      sendCommand(msg.name, msg.fields, msg.link);
    }
  });
});

process.on("SIGINT", () => {
  for (const key of [...pendingAcks.keys()]) clearPending(key);
  mainRxSock.close();
  secRxSock.close();
  txSock.close();
  if (serial.port && serial.port.isOpen) serial.port.close();
  server.close(() => process.exit(0));
});
