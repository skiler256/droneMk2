#!/usr/bin/env python3
# sitl/lidar_wire.py
# ─────────────────────────────────────────────────────────────────────────────
# Codec du format UDP lidar simulé consommé par UdpLidarDriver (cf.
# include/drone/Components/Drivers/UdpLidarDriver.hpp / src/Drivers/
# UdpLidarDriver.cpp). Source de vérité = le .cpp.
#
# Trame : [magic:4]["LID1"][version:1][distance_mm:360*u16][intensity:360*u8]

import struct

MAGIC = b"LID1"
VERSION = 1
_FMT = "<360H360B"
PAYLOAD_SIZE = struct.calcsize(_FMT)
FRAME_SIZE = len(MAGIC) + 1 + PAYLOAD_SIZE


def encode(distance_mm=None, intensity=None):
    """distance_mm/intensity : listes de 360 valeurs (index = degré, 0 =
    nez). None -> tout à zéro ("rien détecté"), cf. INTEGRATION_TEST.md
    roadmap étape 4 (pas de moteur 3D/modèle capteur encore, étape 2/3)."""
    distance_mm = list(distance_mm) if distance_mm is not None else [0] * 360
    intensity = list(intensity) if intensity is not None else [0] * 360
    if len(distance_mm) != 360 or len(intensity) != 360:
        raise ValueError("distance_mm et intensity doivent faire exactement 360 elements")
    payload = struct.pack(_FMT, *distance_mm, *intensity)
    return MAGIC + bytes([VERSION]) + payload
