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

const PORT = 3000;

// cf. src/main.cpp : kUdpTelMainLocalPort/PeerPort, kUdpTelSecLocalPort/PeerPort.
const TELMAIN_RX_PORT = 5602; // le drone ENVOIE ici (son "peer port")
const TELMAIN_TX_PORT = 5601; // le drone ÉCOUTE ici (son "local port")
const TELSEC_RX_PORT = 5612;
const TELSEC_TX_PORT = 5611;

const TELEMETRY_CSV = path.join(__dirname, "..", "TELEMETRY.csv");
const { byId, byName } = loadPackets(TELEMETRY_CSV);

const app = express();
app.use(express.static(path.join(__dirname, "public")));
// Table de codes diagnostics (INDEX.csv) — servie telle quelle, décodée
// côté navigateur (public/codes.js), même source de vérité que
// tools/gen_codes.py.
app.get("/codes.csv", (_req, res) => res.sendFile(path.join(__dirname, "..", "INDEX.csv")));

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
  server.close(() => process.exit(0));
});
