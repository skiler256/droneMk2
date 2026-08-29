#!/usr/bin/env python3
# sitl/raycast_wire.py
# ─────────────────────────────────────────────────────────────────────────────
# Codec du protocole prive requete/reponse entre sensor_bridge.py (bloc [4])
# et world_engine/LidarServer.gd (bloc [3]) -- JAMAIS vu par le systeme sous
# test (port et format distincts de lidar_wire.py/UdpLidarDriver). Source de
# verite = le commentaire d'en-tete de LidarServer.gd, mirrore ici a la main.
#
# Requete (34 octets) : [magic:4]["RCQ1"][version:1][kind:1][seq:4]
#                        [north:f32][east:f32][down:f32][roll:f32][pitch:f32][yaw:f32]
# Reponse : [magic:4]["RCR1"][version:1][kind:1][seq:4][donnees]
#   kind LIDAR360(0) : 360 * f32 metres. kind TFLUNA(1) : 1 * f32 metres.
#
# Distances en METRES bruts (pas mm/quantifie, pas de bruit) -- meme
# convention que World.gd, la quantification/le bruit sont a sensor_bridge.py
# (bloc [4]) au moment d'encoder vers lidar_wire.py/DISTANCE_SENSOR, pas ici.

import struct

MAGIC_REQ = b"RCQ1"
MAGIC_REP = b"RCR1"
VERSION = 1
KIND_LIDAR360 = 0
KIND_TFLUNA = 1

_REQ_FMT = "<BBI6f"  # version, kind, seq, north, east, down, roll, pitch, yaw
_REP_HEADER_FMT = "<BBI"


def encode_request(kind, seq, north, east, down, roll, pitch, yaw):
    return MAGIC_REQ + struct.pack(_REQ_FMT, VERSION, kind, seq, north, east, down, roll, pitch, yaw)


MAGIC_POSE = b"POS1"
_POSE_FMT = "<B6f"  # version, north, east, down, roll, pitch, yaw


def encode_pose(north, east, down, roll, pitch, yaw):
    """Diffusion "fire-and-forget" (pas de reponse attendue) vers
    world_engine/Viewer.gd (bloc [3], viewer optionnel non-headless,
    DECOUPLE de LidarServer.gd -- cf. discussion roadmap etape 4, jamais de
    rendu sur le chemin critique du raycast). Meme pose que celle envoyee a
    LidarServer, juste un destinataire different."""
    return MAGIC_POSE + struct.pack(_POSE_FMT, VERSION, north, east, down, roll, pitch, yaw)


def decode_reply(data):
    """-> (kind, seq, values). values : tuple de 360 floats (LIDAR360) ou 1 (TFLUNA)."""
    if data[:4] != MAGIC_REP:
        raise ValueError(f"magic reponse invalide: {data[:4]!r}")
    version, kind, seq = struct.unpack_from(_REP_HEADER_FMT, data, 4)
    if version != VERSION:
        raise ValueError(f"version reponse inattendue: {version}")
    payload = data[10:]
    if kind == KIND_LIDAR360:
        values = struct.unpack("<360f", payload)
    elif kind == KIND_TFLUNA:
        values = struct.unpack("<f", payload)
    else:
        raise ValueError(f"kind reponse inconnu: {kind}")
    return kind, seq, values
