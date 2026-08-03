#!/usr/bin/env python3
# sitl/gps_wire.py
# ─────────────────────────────────────────────────────────────────────────────
# Codec du format UDP GPS simulé consommé par UdpGpsDriver (cf.
# include/drone/Components/Drivers/UdpGpsDriver.hpp / src/Drivers/
# UdpGpsDriver.cpp). Source de vérité = le .cpp — si l'un change, l'autre
# doit changer avec (pas de générateur pour 2 formats, contrairement à
# TELEMETRY.csv/INDEX.csv qui en ont vraiment besoin).
#
# Trame : [magic:4]["GPS1"][version:1][payload struct <ddiiiiiiBBIIIffiIiiiIId]

import struct

MAGIC = b"GPS1"
VERSION = 1
_FMT = "<ddiiiiiiBBIIIffiIiiiIId"
PAYLOAD_SIZE = struct.calcsize(_FMT)
FRAME_SIZE = len(MAGIC) + 1 + PAYLOAD_SIZE


def encode(*, latitude, longitude, time_array, gps_fix_ok, fix_type,
          p_acc_mm, s_acc_mm_s, v_acc_mm, hdop, vdop, alt_msl_mm, itow_ms,
          vel_ned_cm_s, speed_cm_s, ground_speed_cm_s, heading_deg):
    """time_array: (year,month,day,hour,min,sec). vel_ned_cm_s: (vn,ve,vd), cm/s."""
    payload = struct.pack(
        _FMT,
        latitude, longitude,
        *time_array,
        1 if gps_fix_ok else 0, fix_type,
        p_acc_mm, s_acc_mm_s, v_acc_mm,
        hdop, vdop,
        alt_msl_mm,
        itow_ms,
        *vel_ned_cm_s,
        speed_cm_s, ground_speed_cm_s,
        heading_deg,
    )
    return MAGIC + bytes([VERSION]) + payload
