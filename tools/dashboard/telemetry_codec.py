#!/usr/bin/env python3
# tools/dashboard/telemetry_codec.py
# ─────────────────────────────────────────────────────────────────────────────
# Encode/décode les paquets TELEM::* (voir TELEMETRY.csv) côté Python, pour
# le dashboard. Réutilise tools/telemetry_common.py — même source de vérité
# que le générateur C++ (tools/gen_telemetry.py), donc les PacketID
# assignés ici (index = ordre du CSV) correspondent exactement à
# include/drone/generated/telemetry.hpp::PacketID.
#
# Format sur le fil : [PacketID:1 octet][champs en little-endian, ordre du
# CSV] — cf. include/drone/Components/System Monitoring/Framing.hpp.

import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from telemetry_common import read_csv  # noqa: E402

STRUCT_FMT = {
    "u8": "B", "u16": "H", "u32": "I",
    "i8": "b", "i16": "h", "i32": "i",
    "f32": "f",
}


class PacketDef:
    def __init__(self, packet_id: int, packet):
        self.id = packet_id
        self.name = packet.packet_id
        self.direction = packet.direction
        self.priority = packet.priority
        self.fields = packet.fields
        # Un champ tableau ('u16[360]') compte pour `count` éléments répétés
        # dans le format struct — le préfixe numérique de struct fait ça
        # nativement (ex: "360H").
        fmt = "<" + "".join(
            (str(f.count) if f.is_array else "") + STRUCT_FMT[f.elem_token] for f in packet.fields)
        self.struct = struct.Struct(fmt)


def load_packets(csv_path: Path):
    """-> (packets_by_id: {int: PacketDef}, packets_by_name: {str: PacketDef})"""
    packets = read_csv(csv_path)
    by_id = {i: PacketDef(i, p) for i, p in enumerate(packets)}
    by_name = {p.name: p for p in by_id.values()}
    return by_id, by_name


def decode(packets_by_id, data: bytes):
    """Décode une trame [PacketID][payload]. None si PacketID inconnu ou
    trame trop courte (silencieux : un paquet corrompu ne doit pas planter
    le listener UDP, cf. philosophie erreurs typées côté C++ — ici on
    droppe juste, il n'y a personne à qui remonter l'erreur côté script)."""
    if len(data) < 1:
        return None
    pdef = packets_by_id.get(data[0])
    if pdef is None:
        return None

    payload = data[1:1 + pdef.struct.size]
    if len(payload) < pdef.struct.size:
        flat = (0,) * sum(f.count for f in pdef.fields)
    else:
        flat = pdef.struct.unpack(payload)

    fields = {}
    idx = 0
    for f in pdef.fields:
        fields[f.name] = list(flat[idx:idx + f.count]) if f.is_array else flat[idx]
        idx += f.count

    return {"id": pdef.id, "name": pdef.name, "fields": fields}


def encode(packets_by_name, name: str, field_values: dict) -> bytes:
    pdef = packets_by_name[name]
    ordered = []
    for f in pdef.fields:
        v = field_values.get(f.name, 0)
        conv = float if f.elem_token == "f32" else int
        if f.is_array:
            vals = list(v) if isinstance(v, (list, tuple)) else []
            vals = (vals + [0] * f.count)[:f.count]
            ordered.extend(conv(x) for x in vals)
        else:
            ordered.append(conv(v))
    return bytes([pdef.id]) + pdef.struct.pack(*ordered)
