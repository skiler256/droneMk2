// Encode/décode les paquets TELEM::* (voir ../TELEMETRY.csv) côté Node.
// Port JS de tools/dashboard/telemetry_codec.py — même source de vérité
// (TELEMETRY.csv), même format sur le fil : [PacketID:1][champs LE, ordre
// du CSV]. Un champ 'u16[360]' est un tableau taille fixe (cf.
// SYSMONITORING.md §11) — même principe que le bytesN de MISSIONPLAN.csv,
// mais typé élément par élément.

const fs = require("fs");

const FIELD_TYPES = {
  u8: { size: 1, read: (b, o) => b.readUInt8(o), write: (b, o, v) => b.writeUInt8(v & 0xff, o) },
  u16: { size: 2, read: (b, o) => b.readUInt16LE(o), write: (b, o, v) => b.writeUInt16LE(v & 0xffff, o) },
  u32: { size: 4, read: (b, o) => b.readUInt32LE(o), write: (b, o, v) => b.writeUInt32LE(v >>> 0, o) },
  i8: { size: 1, read: (b, o) => b.readInt8(o), write: (b, o, v) => b.writeInt8(v, o) },
  i16: { size: 2, read: (b, o) => b.readInt16LE(o), write: (b, o, v) => b.writeInt16LE(v, o) },
  i32: { size: 4, read: (b, o) => b.readInt32LE(o), write: (b, o, v) => b.writeInt32LE(v, o) },
  f32: { size: 4, read: (b, o) => b.readFloatLE(o), write: (b, o, v) => b.writeFloatLE(v, o) },
};

const ARRAY_RE = /^(u8|u16|u32|i8|i16|i32|f32)\[(\d+)\]$/;

function parseField(token) {
  const [name, typeToken] = token.split(":").map((s) => s.trim());
  const m = typeToken.match(ARRAY_RE);
  if (m) {
    const elemToken = m[1];
    const count = parseInt(m[2], 10);
    return { name, elemToken, count, isArray: true, size: FIELD_TYPES[elemToken].size * count };
  }
  return { name, elemToken: typeToken, count: 1, isArray: false, size: FIELD_TYPES[typeToken].size };
}

function loadPackets(csvPath) {
  const text = fs.readFileSync(csvPath, "utf-8");
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0);
  const header = lines[0].split(";").map((h) => h.trim());
  const byId = {};
  const byName = {};
  let id = 0;
  for (let i = 1; i < lines.length; i++) {
    const cols = lines[i].split(";");
    const row = {};
    header.forEach((h, idx) => (row[h] = (cols[idx] || "").trim()));
    if (!row.PacketID) continue;

    const fields = (row.Fields || "")
      .split(",")
      .map((t) => t.trim())
      .filter((t) => t.length > 0)
      .map(parseField);
    const size = fields.reduce((s, f) => s + f.size, 0);

    const pkt = { id, name: row.PacketID, direction: row.Direction, fields, size };
    byId[id] = pkt;
    byName[pkt.name] = pkt;
    id++;
  }
  return { byId, byName };
}

// Décode une trame [PacketID][payload]. null si PacketID inconnu ou trame
// trop courte (silencieux : un paquet corrompu ne doit pas planter le
// listener UDP — même philosophie que telemetry_codec.py::decode).
function decode(byId, buf) {
  if (buf.length < 1) return null;
  const pkt = byId[buf[0]];
  if (!pkt) return null;

  const fields = {};
  let offset = 1;
  const short = buf.length < 1 + pkt.size;
  for (const f of pkt.fields) {
    const elem = FIELD_TYPES[f.elemToken];
    if (short) {
      fields[f.name] = f.isArray ? new Array(f.count).fill(0) : 0;
    } else if (f.isArray) {
      const arr = new Array(f.count);
      for (let i = 0; i < f.count; i++) arr[i] = elem.read(buf, offset + i * elem.size);
      fields[f.name] = arr;
    } else {
      fields[f.name] = elem.read(buf, offset);
    }
    offset += f.size;
  }
  return { id: pkt.id, name: pkt.name, fields };
}

function encode(byName, name, fieldValues) {
  const pkt = byName[name];
  if (!pkt) throw new Error(`paquet '${name}' inconnu`);

  const buf = Buffer.alloc(1 + pkt.size);
  buf[0] = pkt.id;
  let offset = 1;
  for (const f of pkt.fields) {
    const elem = FIELD_TYPES[f.elemToken];
    const raw = fieldValues[f.name];
    if (f.isArray) {
      const arr = Array.isArray(raw) ? raw : [];
      for (let i = 0; i < f.count; i++) elem.write(buf, offset + i * elem.size, arr[i] || 0);
    } else {
      elem.write(buf, offset, raw || 0);
    }
    offset += f.size;
  }
  return buf;
}

module.exports = { loadPackets, decode, encode };
