// Test de la retransmission des commandes acquittees de server.js.
//
// Simule le drone en UDP : server.js emet les commandes, ce script decide
// d'acquitter ou non, et on verifie le nombre d'emissions et les etats
// annonces a l'interface.
//
// Lancement (ports decales pour cohabiter avec le dashboard) :
//
//   cd GCS
//   GCS_HTTP_PORT=13000 GCS_TELMAIN_RX=15602 GCS_TELMAIN_TX=15601 \
//   GCS_TELSEC_RX=15612 GCS_TELSEC_TX=15611 GCS_ESP32_PORT=/dev/null \
//     node server.js &
//   node test/test_ack.js
//
const dgram = require("dgram");
const path = require("path");
const { loadPackets, decode, encode } = require("../telemetry-codec");
const WebSocket = require("ws");

const { byId, byName } = loadPackets(path.join(__dirname, "..", "..", "data", "TELEMETRY.csv"));
const TX = 15601;   // la ou server.js emet (le drone ecoute)
const RX = 15602;   // la ou le drone emet (server.js ecoute)

const drone = dgram.createSocket("udp4");
const vers = dgram.createSocket("udp4");
let recues = [];
let politique = "toujours";
let compteur = 0;

drone.on("message", (data) => {
  const pkt = decode(byId, data);
  if (!pkt || !pkt.name.startsWith("CMD_")) return;
  recues.push({ name: pkt.name, seq: pkt.fields.seq, t: Date.now() });
  compteur++;
  const acquitter =
    politique === "toujours" ||
    (politique === "apres2" && compteur > 2);
  if (!acquitter) return;
  const ack = encode(byName, "CMD_ACK",
    { seq: pkt.fields.seq, packet_id: byName[pkt.name].id, result: 0 });
  vers.send(ack, RX, "127.0.0.1");
});
drone.bind(TX, "127.0.0.1");

const ws = new WebSocket("ws://127.0.0.1:13000");
const evenements = [];
ws.on("message", (m) => {
  const msg = JSON.parse(m);
  if (msg.type === "ack") evenements.push(msg);
});

const attendre = (ms) => new Promise((r) => setTimeout(r, ms));

(async () => {
  await new Promise((r) => ws.on("open", r));

  const scenario = async (nom, pol, cmd, patience) => {
    politique = pol; compteur = 0; recues = []; evenements.length = 0;
    ws.send(JSON.stringify({ type: "command", name: cmd, fields: {}, link: "main" }));
    await attendre(patience);
    const etats = evenements.map((e) => e.state);
    console.log(`${nom}`);
    console.log(`  emissions recues par le drone : ${recues.length}`);
    console.log(`  etats annonces a l'interface   : ${etats.join(" -> ") || "(aucun)"}`);
    return { recues: recues.length, etats };
  };

  const a = await scenario("1. le drone acquitte du premier coup", "toujours", "CMD_ARM", 700);
  console.log(`  ATTENDU 1 emission + confirmed : ${a.recues === 1 && a.etats.includes("confirmed") ? "OK" : "ECHEC"}\n`);

  const b = await scenario("2. le drone ignore les 2 premieres", "apres2", "CMD_RTL", 1500);
  console.log(`  ATTENDU 3 emissions + confirmed : ${b.recues === 3 && b.etats.includes("confirmed") ? "OK" : "ECHEC"}\n`);

  const c = await scenario("3. le drone n'acquitte jamais", "jamais", "CMD_LAND", 2000);
  console.log(`  ATTENDU 4 emissions + failed    : ${c.recues === 4 && c.etats.includes("failed") ? "OK" : "ECHEC"}\n`);

  const d = await scenario("4. urgence : plus de tentatives", "jamais", "CMD_EMERGENCY", 2600);
  console.log(`  ATTENDU 8 emissions + failed    : ${d.recues === 8 && d.etats.includes("failed") ? "OK" : "ECHEC"}`);

  drone.close(); vers.close(); ws.close(); process.exit(0);
})();
