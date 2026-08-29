// GCS/debug-injector.js
// ─────────────────────────────────────────────────────────────────────────────
// Page de debug séparée (autre port que le dashboard réel) pour injecter de
// faux paquets CODE, comme le ferait le drone (SysMonitoring) -- teste tout
// le chemin réel : UDP -> server.js -> voyants ESP32 + alarme sonore
// navigateur, sans toucher au code du dashboard ni à celui du drone.
//
// Volontairement un process Node séparé (pas une route ajoutée à
// server.js) : un outil de debug ne doit jamais pouvoir planter ou
// ralentir le serveur GCS réel.
//
// Usage : node debug-injector.js  (port 3001 par défaut, GCS_DEBUG_PORT pour changer)

const dgram = require("dgram");
const express = require("express");
const path = require("path");
const { loadPackets, encode } = require("./telemetry-codec");

const PORT = process.env.GCS_DEBUG_PORT || 3001;

// Même port que server.js::TELMAIN_RX_PORT -- on imite exactement ce que le
// drone enverrait, aucun raccourci qui contournerait le pipeline réel.
const TELMAIN_RX_PORT = 5602;

const TELEMETRY_CSV = path.join(__dirname, "..", "TELEMETRY.csv");
const { byName } = loadPackets(TELEMETRY_CSV);

const sock = dgram.createSocket("udp4");

function sendCode(packedCode, active) {
  const frame = encode(byName, "CODE", { code: packedCode, active: active ? 1 : 0, ts_ms: Date.now() & 0xffffffff });
  sock.send(frame, TELMAIN_RX_PORT, "127.0.0.1");
}

const app = express();
app.use(express.json());
app.use(express.static(path.join(__dirname, "public-debug")));

app.post("/api/code", (req, res) => {
  const { code, active } = req.body || {};
  const packed = Number(code);
  if (!Number.isInteger(packed)) {
    res.status(400).json({ error: "code (uint32 packé) requis" });
    return;
  }
  sendCode(packed, !!active);
  res.json({ ok: true });
});

app.listen(PORT, () => {
  console.log(`GCS debug injector sur http://localhost:${PORT}`);
});
