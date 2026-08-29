// GCS/server.js
// ─────────────────────────────────────────────────────────────────────────────
// Sert le dashboard statique (public/) et relaie la télémétrie réelle :
//   drone (SysMonitoring, UdpTelemetryDriver) --UDP loopback--> ce serveur
//   ce serveur --WebSocket--> navigateur(s)
// et dans l'autre sens, commandes UP navigateur -> WebSocket -> UDP -> drone.
//
// Ports identiques à tools/dashboard/dashboard_server.py (même émulation
// LAN de TelMain/TelSec tant que le H562 physique n'existe pas, cf.
// mémoire "GCS architecture") — ne PAS lancer les deux dashboards en même
// temps, ils se disputeraient les mêmes ports UDP.

const dgram = require("dgram");
const express = require("express");
const path = require("path");
const { WebSocketServer } = require("ws");
const { loadPackets, decode, encode } = require("./telemetry-codec");
const { loadCodesTable } = require("./codes-table");
const { SerialBridge } = require("./serial-bridge");

const PORT = 3000;

// Boîtier ESP32 (voyants + boutons) -- cf. GCS/ESP32/. "Bête" côté firmware,
// toute la logique (quel code allume quel voyant) est ici.
const ESP32_PORT = process.env.GCS_ESP32_PORT || "/dev/ttyUSB0";

// cf. src/main.cpp : kUdpTelMainLocalPort/PeerPort, kUdpTelSecLocalPort/PeerPort.
const TELMAIN_RX_PORT = 5602; // le drone ENVOIE ici (son "peer port")
const TELMAIN_TX_PORT = 5601; // le drone ÉCOUTE ici (son "local port")
const TELSEC_RX_PORT = 5612;
const TELSEC_TX_PORT = 5611;

const TELEMETRY_CSV = path.join(__dirname, "..", "TELEMETRY.csv");
const { byId, byName } = loadPackets(TELEMETRY_CSV);

const INDEX_CSV = path.join(__dirname, "..", "INDEX.csv");
const codesTable = loadCodesTable(INDEX_CSV);

const serial = new SerialBridge(ESP32_PORT);
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

// MW (Master Warning) / MC (Master Caution) ne sont PAS dans Panel (contrairement
// aux voyants de défaut individuels) : ce sont des agrégats calculés ici, pour
// le VOYANT ESP32 uniquement -- le son suit désormais sa propre logique côté
// navigateur (cf. alarm.js, ALARMS.csv), ack indépendant de celui-ci.
// Règle donnée par Jules : ENG/PWR/FC/EKF/MAG/GPS/NAV/EMG -> MW, tout le reste
// des voyants de défaut -> MC. Accusé de réception (MW_ACK/MC_ACK) éteint
// l'agrégat SANS éteindre le voyant de défaut sous-jacent (dark cockpit : seul
// un code qui redevient inactif l'éteint) -- et un NOUVEAU voyant de la
// catégorie qui apparaît réarme l'ack (sémantique cockpit standard).
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
app.get("/codes.csv", (_req, res) => res.sendFile(path.join(__dirname, "..", "INDEX.csv")));
// Idem pour les définitions d'alarme (cf. tools/alarms_common.py, éditées
// via l'onglet "Alarmes" de codes_editor.py) -- décodées côté navigateur
// par GCS/public/js/alarm.js.
app.get("/alarms.csv", (_req, res) => res.sendFile(path.join(__dirname, "..", "ALARMS.csv")));
app.get("/mission_alarms.csv", (_req, res) => res.sendFile(path.join(__dirname, "..", "MISSION_ALARMS.csv")));

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

function listenTelemetry(port, link) {
  const sock = dgram.createSocket("udp4");
  sock.on("message", (data) => {
    const pkt = decode(byId, data);
    if (!pkt) return; // trame corrompue/inconnue : on droppe, pas de crash

    if (link === "main") lastRxMain = Date.now();
    else lastRxSec = Date.now();

    if (pkt.name === "CODE") handleCodePacket(pkt.fields);
    if (pkt.name === "ARM_STATE") {
      armed = !!pkt.fields.armed;
      updateLeds();
    }
    broadcast({ type: "telemetry", link, name: pkt.name, fields: pkt.fields, ts: Date.now() });
  });
  sock.on("error", (err) => console.error(`GCS: erreur socket UDP ${link} (${port}):`, err.message));
  sock.bind(port, "127.0.0.1");
  return sock;
}

const mainRxSock = listenTelemetry(TELMAIN_RX_PORT, "main");
const secRxSock = listenTelemetry(TELSEC_RX_PORT, "sec");

const txSock = dgram.createSocket("udp4");

function sendCommand(name, fields, link) {
  const port = link === "sec" ? TELSEC_TX_PORT : TELMAIN_TX_PORT;
  let frame;
  try {
    frame = encode(byName, name, fields || {});
  } catch (e) {
    console.error("GCS: encodage commande échoué:", e.message);
    return false;
  }
  txSock.send(frame, port, "127.0.0.1");
  return true;
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
  mainRxSock.close();
  secRxSock.close();
  txSock.close();
  if (serial.port && serial.port.isOpen) serial.port.close();
  server.close(() => process.exit(0));
});
