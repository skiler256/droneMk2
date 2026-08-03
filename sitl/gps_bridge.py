#!/usr/bin/env python3
# sitl/gps_bridge.py
# ─────────────────────────────────────────────────────────────────────────────
# Pont SITL_MASTER : lit SIM_STATE depuis ArduPilot SITL (état vrai, non
# bruité, hors EKF — cf. INTEGRATION_TEST.md §3.2) et le relaie en UDP vers
# UdpGpsDriver (cf. gps_wire.py) à la cadence d'un vrai NEO-M8N (~10 Hz).
#
# "SITL nu" (roadmap étape 2/3) : pas de bruit/biais/dropout (bloc [4], pas
# encore construit — étape 4) — fixType/hdop/vdop/accuracy sont des valeurs
# fixes représentant un fix propre idéal, PAS un modèle capteur réaliste.
#
# Envoie aussi un flux lidar "rien détecté" constant (pas de moteur 3D avant
# l'étape 4, cf. discussion) à la même cadence, vers UdpLidarDriver.
#
# ⚠️ Écrit contre les champs MAVLink SIM_STATE documentés (pymavlink), mais
# PAS testé contre une vraie instance ArduPilot SITL (indisponible dans cet
# environnement de dev) — seul gps_wire.py/lidar_wire.py (l'encodage lui-même)
# a été validé en bout en bout contre le vrai décodeur C++ (UdpGpsDriver).
# À vérifier sur ta machine avec `sim_vehicle.py` avant de faire confiance
# aux valeurs relayées.
#
# Usage : sitl/.venv/bin/python3 sitl/gps_bridge.py [--sim-udp 127.0.0.1:14551]
#                                                    [--gps-port 15100] [--lidar-port 15101]

import argparse
import datetime
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gps_wire  # noqa: E402
import lidar_wire  # noqa: E402

from pymavlink import mavutil  # noqa: E402

GPS_EPOCH = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)
SECONDS_PER_WEEK = 7 * 24 * 3600

# Cadence cible, alignée sur un vrai NEO-M8N (10 Hz) — cf. gps_bridge.py en-tête.
SEND_PERIOD_S = 0.1

# "SITL nu" : fix idéal fixe, pas de modèle de bruit (bloc [4], étape 4).
FIXED_FIX_TYPE = 3
FIXED_HDOP = 1.0
FIXED_VDOP = 1.0
FIXED_P_ACC_MM = 300
FIXED_S_ACC_MM_S = 100
FIXED_V_ACC_MM = 500


def gps_time_now():
    """-> (year,month,day,hour,min,sec), itow_ms — dérivés de l'horloge murale,
    même principe que UDP_bridge.cpp::fillGpsWeek() (pas de vrai récepteur GPS
    ici, cf. commentaire équivalent côté C++)."""
    now = datetime.datetime.now(datetime.timezone.utc)
    elapsed = (now - GPS_EPOCH).total_seconds()
    tow_s = elapsed % SECONDS_PER_WEEK
    time_array = (now.year, now.month, now.day, now.hour, now.minute, now.second)
    return time_array, int(tow_s * 1000)


def build_gps_frame(sim_state):
    """sim_state : message pymavlink SIM_STATE. cf. champs documentés dans
    common.xml (lat/lon en degres float, alt en m, vn/ve/vd en m/s)."""
    time_array, itow_ms = gps_time_now()
    heading_deg = (sim_state.yaw * 180.0 / 3.14159265) % 360.0
    speed_cm_s = int(((sim_state.vn ** 2 + sim_state.ve ** 2) ** 0.5) * 100)

    return gps_wire.encode(
        latitude=sim_state.lat,
        longitude=sim_state.lon,
        time_array=time_array,
        gps_fix_ok=True,
        fix_type=FIXED_FIX_TYPE,
        p_acc_mm=FIXED_P_ACC_MM,
        s_acc_mm_s=FIXED_S_ACC_MM_S,
        v_acc_mm=FIXED_V_ACC_MM,
        hdop=FIXED_HDOP,
        vdop=FIXED_VDOP,
        alt_msl_mm=int(sim_state.alt * 1000),
        itow_ms=itow_ms,
        vel_ned_cm_s=(
            int(sim_state.vn * 100),
            int(sim_state.ve * 100),
            int(sim_state.vd * 100),
        ),
        speed_cm_s=speed_cm_s,
        ground_speed_cm_s=speed_cm_s,
        heading_deg=heading_deg,
    )


def run(sim_udp: str, gps_port: int, lidar_port: int):
    mav = mavutil.mavlink_connection(f"udp:{sim_udp}")
    print(f"gps_bridge: connexion MAVLink sur {sim_udp}, attente HEARTBEAT...")
    mav.wait_heartbeat()
    print("gps_bridge: HEARTBEAT recu, demande du flux SIM_STATE...")

    # 10 Hz : cf. SEND_PERIOD_S — MAV_CMD_SET_MESSAGE_INTERVAL, pas l'ancien
    # REQUEST_DATA_STREAM (deprecie, moins fiable sur les dialectes recents).
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE,
        int(SEND_PERIOD_S * 1_000_000), 0, 0, 0, 0, 0,
    )

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lidar_frame = lidar_wire.encode()  # "rien detecte" constant, cf. en-tete
    last_send = 0.0

    while True:
        msg = mav.recv_match(type="SIM_STATE", blocking=True, timeout=1.0)
        now = time.monotonic()
        if msg is None:
            continue
        if now - last_send < SEND_PERIOD_S:
            continue
        last_send = now

        out.sendto(build_gps_frame(msg), ("127.0.0.1", gps_port))
        out.sendto(lidar_frame, ("127.0.0.1", lidar_port))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-udp", default="127.0.0.1:14551",
                        help="endpoint MAVLink d'ArduPilot SITL (PAS le port 14550 du drone lui-meme)")
    parser.add_argument("--gps-port", type=int, default=15100)
    parser.add_argument("--lidar-port", type=int, default=15101)
    args = parser.parse_args()
    run(args.sim_udp, args.gps_port, args.lidar_port)


if __name__ == "__main__":
    main()
