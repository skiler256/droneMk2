#!/usr/bin/env python3
# tools/missionplan_wire.py
# ─────────────────────────────────────────────────────────────────────────────
# Codec Python du format binaire plan de mission consomme par MissionControl
# (cf. include/drone/Components/MissionControl/MissionPlan.hpp/.cpp,
# include/drone/generated/missionplan.hpp). Source de verite = le C++ --
# meme esprit que sitl/gps_wire.py / sitl/lidar_wire.py (miroir Python d'un
# format C++, valide par round-trip contre le vrai decodeur).
#
# Trame : [magic "MPLN":4][version:1][MissionHeader:21][segments TLV...][crc32c:4]
# Segment TLV : [kind:u8][len:u8][payload...]
#
# CRC32-C (Castagnoli, poly 0x1EDC6F41 reflechi 0x82F63B78) -- PAS le CRC32
# standard de zlib (poly different). Reimplemente ici pour matcher exactement
# UTILITIES::crc32 (utilities.hpp) sans nouvelle dependance externe.

import struct

MAGIC = b"MPLN"
VERSION = 1

# battWarningV,battCriticalV,rtbTransitAlt,gcsLossAction,navVolumeRadius,navVolumeCeiling,takeoffAlt
_HEADER_FMT = "<fffBfff"
HEADER_SIZE = struct.calcsize(_HEADER_FMT)
assert HEADER_SIZE == 25

HEADER_DEFAULTS = {
    "battWarningV": 14.0,
    "battCriticalV": 12.5,
    "rtbTransitAlt": 30.0,
    "gcsLossAction": 0,
    "navVolumeRadius": 200.0,
    "navVolumeCeiling": 100.0,
    "takeoffAlt": 10.0,
}


def pack_header(fields: dict) -> bytes:
    h = {**HEADER_DEFAULTS, **fields}
    return struct.pack(
        _HEADER_FMT,
        h["battWarningV"], h["battCriticalV"], h["rtbTransitAlt"],
        h["gcsLossAction"], h["navVolumeRadius"], h["navVolumeCeiling"],
        h["takeoffAlt"],
    )


# kind -> (SegmentKind numerique, format struct, ordre des champs)
# cf. include/drone/generated/missionplan.hpp -- NE PAS reordonner (identifiants
# TLV stables), ajouter seulement en fin (meme contrainte que le C++ genere).
_SEGMENT_SPECS = {
    "Waypoint": (0, "<5f", ["north", "east", "alt", "speed", "acceptRadius"]),
    "Corridor": (1, "<7f", ["northStart", "eastStart", "northEnd", "eastEnd", "alt", "speed", "width"]),
    "Curve": (2, "<7f", ["centerNorth", "centerEast", "radius", "alt", "speed", "startAngle", "endAngle"]),
    "Landing": (3, "<ffB", ["north", "east", "category"]),
    "Hold": (4, "<f", ["duration"]),
    # PayloadAction : actionId (u8) + payload brut 16 octets (liste de 16 int 0-255).
    "PayloadAction": (5, "<B16s", ["actionId", "payload"]),
}

SEGMENT_DEFAULTS = {
    "Waypoint": {"north": 0.0, "east": 0.0, "alt": -10.0, "speed": 2.0, "acceptRadius": 2.0},
    "Corridor": {"northStart": 0.0, "eastStart": 0.0, "northEnd": 0.0, "eastEnd": 0.0,
                "alt": -10.0, "speed": 2.0, "width": 5.0},
    "Curve": {"centerNorth": 0.0, "centerEast": 0.0, "radius": 10.0, "alt": -10.0,
             "speed": 2.0, "startAngle": 0.0, "endAngle": 1.5708},
    "Landing": {"north": 0.0, "east": 0.0, "category": 2},
    "Hold": {"duration": 0.0},
    "PayloadAction": {"actionId": 0, "payload": bytes(16)},
}


def encode_segment(seg_type: str, fields: dict) -> bytes:
    if seg_type not in _SEGMENT_SPECS:
        raise ValueError(f"type de segment inconnu: {seg_type!r} (valides: {list(_SEGMENT_SPECS)})")
    kind, fmt, field_names = _SEGMENT_SPECS[seg_type]
    merged = {**SEGMENT_DEFAULTS[seg_type], **fields}
    values = []
    for name in field_names:
        v = merged[name]
        if name == "payload":
            v = bytes(v) if not isinstance(v, (bytes, bytearray)) else v
            v = v.ljust(16, b"\x00")[:16]
        values.append(v)
    payload = struct.pack(fmt, *values)
    if len(payload) > 255:
        raise ValueError(f"payload {seg_type} trop long ({len(payload)}o, max 255)")
    return bytes([kind, len(payload)]) + payload


# ─────────────────────────────────────────────────────────────────────────────
# CRC32-C -- meme polynome/init/xor final que UTILITIES::crc32 (utilities.hpp).
# ─────────────────────────────────────────────────────────────────────────────

def _make_crc32c_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0x82F63B78 if (crc & 1) else (crc >> 1)
        table.append(crc & 0xFFFFFFFF)
    return table


_CRC32C_TABLE = _make_crc32c_table()


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc = _CRC32C_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF


def encode(header: dict, segments: list) -> bytes:
    """header : dict de champs MissionHeader (manquants = defauts).
    segments : liste de (type: str, fields: dict)."""
    out = bytearray()
    out += MAGIC
    out.append(VERSION)
    out += pack_header(header)
    for seg_type, fields in segments:
        out += encode_segment(seg_type, fields)
    crc = crc32c(bytes(out))
    out += struct.pack("<I", crc)
    return bytes(out)
