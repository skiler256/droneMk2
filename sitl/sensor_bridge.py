#!/usr/bin/env python3
# sitl/sensor_bridge.py
# ─────────────────────────────────────────────────────────────────────────────
# Pont SITL_MASTER, bloc [4] "SITL nu" (INTEGRATION_TEST.md §3.4 -- pas
# encore de bruit/biais/latence/dropout, juste un passthrough du bloc [3]).
# Remplace gps_bridge.py : une seule lecture SIM_STATE, fusionnee pour les
# trois capteurs (decision roadmap etape 4 -- "fusionne tout, c'est plus ce
# que j'avais en tete") :
#
#   GPS (NEO-M8N, 10Hz)     -> gps_wire.py   -> UdpGpsDriver   ([4] -> [1])
#   LD06 (rotatif, 10Hz)    -> lidar_wire.py -> UdpLidarDriver ([4] -> [1])
#   TF Luna (telemetre, 100Hz) -> DISTANCE_SENSOR (MAVLink)    -> ArduPilot SITL ([4] -> [2])
#
# Le routage TF Luna n'est PAS symetrique au reste (cf. §4 "Routage des
# capteurs") : il passe par ArduPilot lui-meme (comme sur le vrai FC via
# UART), pas par notre UDP_bridge.
#
# LD06/TF Luna interrogent world_engine/LidarServer.gd (bloc [3], requete/
# reponse synchrone UDP, cf. raycast_wire.py) avec la pose ET l'attitude
# complete (roll/pitch/yaw) recue de SIM_STATE -- LD06 et TF Luna sont montes
# rigides sur le chassis, leur mesure suit toute l'attitude, pas seulement
# le cap (cf. discussion roadmap etape 4).
#
# ⚠️ Ecrit contre les champs MAVLink SIM_STATE/DISTANCE_SENSOR documentes
# (pymavlink) mais le flux DISTANCE_SENSOR -> ArduPilot n'a pas encore ete
# verifie contre une vraie instance SITL avec RNGFND1_TYPE=10 configure --
# a valider avant de faire confiance a l'altitude EKF/rangefinder relayee.
#
# ⚠️⚠️ ORIGINE FIXE ET CODEE EN DUR (cf. HOME_LAT/HOME_LON/HOME_ALT_M plus
# bas) : SIM_STATE ne donne QUE lat/lon/alt absolus, jamais de position NED
# locale -- or le raycast (World.gd) a besoin d'une position en metres dans
# le repere du monde JSON. On convertit lat/lon -> NED par approximation
# terre-plate autour d'une origine FIXE = le home par defaut d'ArduPilot
# SITL (Canberra/CMAC). Decision explicite (pas une approximation qu'on a
# oublie de generaliser) : SI UN SCENARIO DEMARRE LE SITL AILLEURS (home
# different), CETTE CONVERSION SERA FAUSSE SANS AUCUNE ERREUR VISIBLE --
# le raycast utilisera silencieusement le mauvais repere. A generaliser
# (capture dynamique de GPS_GLOBAL_ORIGIN/HOME_POSITION) le jour ou un
# scenario a besoin d'un home different.
#
# Usage : sitl/.venv/bin/python3 sitl/sensor_bridge.py
#             [--sim-udp 127.0.0.1:14551]
#             [--gps-port 15100] [--lidar-port 15101]
#             [--lidar-server 127.0.0.1:15102]
#             [--fake-fixed]
#
# --fake-fixed : ignore MAVLink/SIM_STATE/LidarServer entierement, envoie une
# trame GPS+LD06 fixe valide a 10Hz en continu (pas de DISTANCE_SENSOR, pas
# de connexion MAVLink en mode fake-fixed). Sert a isoler une panne "GPS/LD06
# jamais mis a jour dans la shm", cf. gps_bridge.py historique.

import argparse
import datetime
import math
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gps_wire  # noqa: E402
import lidar_wire  # noqa: E402
import raycast_wire  # noqa: E402

from pymavlink import mavutil  # noqa: E402

GPS_EPOCH = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)
SECONDS_PER_WEEK = 7 * 24 * 3600

# GPS et LD06 : cadence d'un vrai NEO-M8N/LD06 (10 Hz).
GPS_LIDAR_PERIOD_S = 0.1
# TF Luna : jusqu'a 250Hz reel (datasheet), 100Hz = cadence usine par defaut
# -- viser ca plutot que 10Hz pour rester fidele au capteur (contrairement
# au LD06 ou 10Hz EST la cadence native, pas une limitation qu'on invente).
TFLUNA_PERIOD_S = 0.01

# "SITL nu" : fix idéal fixe, pas de modele de bruit (bloc [4] reel, futur).
FIXED_FIX_TYPE = 3
FIXED_HDOP = 1.0
FIXED_VDOP = 1.0
FIXED_P_ACC_MM = 300
FIXED_S_ACC_MM_S = 100
FIXED_V_ACC_MM = 500

# TF Luna, cf. INTEGRATION_TEST.md §5.3 (RNGFND1_MIN_CM/MAX_CM) -- memes
# valeurs des deux cotes (bridge et params ArduPilot) sinon deux ecreteurs
# divergents rendent le diagnostic impossible (meme raisonnement que §5.5).
RNGFND_MIN_CM = 20
RNGFND_MAX_CM = 800

# Intensite : aucune notion de materiau/reflectivite dans le raycast
# geometrique pur (bloc [3], §3.3) -- constante tant que le bloc [4] n'a pas
# de vrai modele capteur, juste distingue "touche"/"rien detecte" comme
# gps_bridge.py::build_fixed_lidar_frame() le faisait deja.
LIDAR_INTENSITY_HIT = 128
LIDAR_INTENSITY_MISS = 0

# Origine FIXE pour la conversion lat/lon -> NED local (cf. avertissement
# ⚠️⚠️ en-tete de fichier) -- home par defaut d'ArduPilot SITL, memes valeurs
# que build_fixed_gps_frame() ci-dessous.
HOME_LAT_DEG = -35.363262
HOME_LON_DEG = 149.165237
HOME_ALT_M = 584.0

# ⚠️⚠️⚠️ APPROXIMATION TERRE-PLATE, PAS DE LA VRAIE GEODESIE -- valable
# seulement a courte portee. Pas d'ellipsoide WGS84 ici (pas de conversion
# geodetique -> ECEF -> plan tangent local, pas de rayon de courbure
# N(lat) = a / sqrt(1 - e^2 sin^2(lat))) : juste "1 degre de latitude =
# 111320m partout" et "1 degre de longitude = 111320m * cos(lat_origine)"
# (le seul terme geodesique garde -- le retrecissement des meridiens).
# L'erreur croit avec le carre de la distance a l'origine : negligeable
# (mm) sur les quelques dizaines de metres d'un monde de test SITL
# (MAX_RANGE_M = 30 dans World.gd) -- MAIS PAS VALABLE au-dela de quelques
# km. Meme simplification qu'ArduPilot en interne (AP_Math/Location::
# offset(), LOCATION_SCALING_FACTOR = cos(lat)), donc coherent avec le
# modele de reference SITL (§3.2) -- mais si cette base sert un jour a un
# projet longue portee (UAV longue distance, fusee/missile balistique --
# courbure terrestre et variation de cos(lat) le long de la trajectoire
# non negligeables), remplacer par une vraie transformation ellipsoidale
# (geodetique -> ECEF -> NED local, ou une bibliotheque type pyproj/GeographicLib)
# avant de faire confiance a cette fonction.
_METERS_PER_DEG_LAT = 111320.0


def sim_state_pos_ned(msg):
    """SIM_STATE (lat/lon/alt absolus) -> (north_m, east_m, down_m) autour de
    l'origine FIXE HOME_LAT_DEG/HOME_LON_DEG/HOME_ALT_M -- cf. avertissement
    ⚠️⚠️ en-tete de fichier (origine figee) ET ⚠️⚠️⚠️ ci-dessus (terre plate,
    courte portee seulement), faux si le SITL demarre ailleurs ou sur de
    longues distances."""
    lat_deg = msg.lat / 1.0e7
    lon_deg = msg.lon / 1.0e7
    north_m = (lat_deg - HOME_LAT_DEG) * _METERS_PER_DEG_LAT
    east_m = (lon_deg - HOME_LON_DEG) * _METERS_PER_DEG_LAT * math.cos(math.radians(HOME_LAT_DEG))
    down_m = -(msg.alt - HOME_ALT_M)
    return north_m, east_m, down_m


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
    """sim_state : message pymavlink SIM_STATE. cf. champs documentes dans
    common.xml (lat/lon en degres float, alt en m, vn/ve/vd en m/s).

    Bug ArduPilot (verifie dans libraries/SITL/SITL.cpp::sim_state_send(),
    V4.8.0-dev 1f2fe6fc) : les champs FLOTTANTS lat/lon (documentes en
    degres dans le XML MAVLink officiel) sont envoyes multiplies par 1e7
    -- copie-colle rate depuis le calcul de lat_int/lon_int (eux corrects,
    degE7 comme documente). On divise ici pour compenser, cote ArduPilot
    c'est un vrai bug (pas une question d'interpretation)."""
    time_array, itow_ms = gps_time_now()
    heading_deg = (sim_state.yaw * 180.0 / 3.14159265) % 360.0
    speed_cm_s = int(((sim_state.vn ** 2 + sim_state.ve ** 2) ** 0.5) * 100)

    return gps_wire.encode(
        latitude=sim_state.lat / 1.0e7,
        longitude=sim_state.lon / 1.0e7,
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


def build_lidar_frame(distances_m):
    """distances_m : 360 floats metres (reponse LidarServer, kind=LIDAR360).
    0.0 = rien detecte, meme convention que lidar_wire -- conversion m->mm
    directe, pas de quantification/bruit reel (bloc [4] pas encore construit).

    DEBUG TEMPORAIRE : round() leve une exception NON RATTRAPEE sur
    NaN/inf (ValueError/OverflowError) -- plausible en geometrie degeneree
    tout pres d'une surface d'obstacle. Non protege avant, ça crashait tout
    sensor_bridge.py en silence (bug lidar shm fige pres d'un obstacle).
    """
    distance_mm = []
    for i, d in enumerate(distances_m):
        if not math.isfinite(d):
            print(f"DEBUG-LIDAR: distance non finie a l'angle {i}: {d!r} -- traitee comme 0 (rien detecte)")
            d = 0.0
        distance_mm.append(max(0, min(65535, round(d * 1000.0))))
    intensity = [LIDAR_INTENSITY_HIT if d > 0.0 else LIDAR_INTENSITY_MISS for d in distances_m]
    return lidar_wire.encode(distance_mm=distance_mm, intensity=intensity)


def send_distance_sensor(mav, distance_m, time_boot_ms):
    """distance_m : reponse LidarServer, kind=TFLUNA (0.0 = rien detecte).
    0.0 encode "hors de portee" en envoyant RNGFND_MAX_CM (convention ArduPilot
    usuelle pour un rangefinder qui ne voit rien -- a verifier en SITL reel,
    cf. avertissement en-tete)."""
    current_cm = RNGFND_MAX_CM if distance_m <= 0.0 else int(round(distance_m * 100.0))
    current_cm = max(RNGFND_MIN_CM, min(RNGFND_MAX_CM, current_cm))
    mav.mav.distance_sensor_send(
        time_boot_ms,
        RNGFND_MIN_CM,
        RNGFND_MAX_CM,
        current_cm,
        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        0,  # id
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,  # vers le bas, cf. RNGFND1_ORIENT=25
        0,  # covariance inconnue -- hors fusion EKF de toute facon (EK3_RNG_USE_HGT=-1)
    )


def build_fixed_gps_frame():
    """Trame GPS fixe valide (position/vitesse ArduPilot SITL par defaut,
    Canberra) -- pour --fake-fixed, bypass complet de SIM_STATE."""
    time_array, itow_ms = gps_time_now()
    return gps_wire.encode(
        latitude=-35.363262,
        longitude=149.165237,
        time_array=time_array,
        gps_fix_ok=True,
        fix_type=FIXED_FIX_TYPE,
        p_acc_mm=FIXED_P_ACC_MM,
        s_acc_mm_s=FIXED_S_ACC_MM_S,
        v_acc_mm=FIXED_V_ACC_MM,
        hdop=FIXED_HDOP,
        vdop=FIXED_VDOP,
        alt_msl_mm=584000,
        itow_ms=itow_ms,
        vel_ned_cm_s=(0, 0, 0),
        speed_cm_s=0,
        ground_speed_cm_s=0,
        heading_deg=353.0,
    )


def build_fixed_lidar_frame():
    """Anneau complet a 3000mm, intensite 128 -- pas "rien detecte", pour
    verifier que count>0 se propage bien jusqu'au dashboard. Rappel :
    LidarTask a besoin d'un DynamicState frais (donc ArduPilot connecte + EKF
    converge) avant de publier en shm, meme avec ce mode."""
    return lidar_wire.encode(distance_mm=[3000] * 360, intensity=[128] * 360)


def run_fake_fixed(gps_port: int, lidar_port: int):
    print("sensor_bridge: --fake-fixed actif, pas de connexion MAVLink/LidarServer -- "
         "trame GPS+LD06 fixe envoyee en continu a 10Hz (pas de TF Luna)")
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lidar_frame = build_fixed_lidar_frame()
    while True:
        out.sendto(build_fixed_gps_frame(), ("127.0.0.1", gps_port))
        out.sendto(lidar_frame, ("127.0.0.1", lidar_port))
        time.sleep(GPS_LIDAR_PERIOD_S)


# Le flux SIM_STATE peut se perdre au demarrage (paquet UDP perdu, ArduPilot
# pas encore pret) -- on retente tant que rien n'est arrive, meme logique que
# STREAM_STALE_S ci-dessous pour une interruption en cours de vol.
REQUEST_RETRY_PERIOD_S = 2.0
STREAM_STALE_S = 1.0


def _request_sim_state(mav):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE,
        int(TFLUNA_PERIOD_S * 1_000_000), 0, 0, 0, 0, 0,
    )


class RaycastClient:
    """Requete/reponse synchrone UDP vers LidarServer.gd (bloc [3]), cf.
    raycast_wire.py. Un socket dedie (pas celui des trames GPS/lidar/
    DISTANCE_SENSOR sortantes) pour pouvoir bloquer sur la reponse."""

    def __init__(self, server_addr, timeout_s=0.2):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout_s)
        self._server_addr = server_addr
        self._seq = 0

    def query(self, kind, north, east, down, roll, pitch, yaw):
        """-> tuple de floats (360 ou 1) ou None (timeout/reponse invalide --
        LidarServer pas encore pret, ou paquet perdu)."""
        self._seq += 1
        req = raycast_wire.encode_request(kind, self._seq, north, east, down, roll, pitch, yaw)
        self._sock.sendto(req, self._server_addr)
        try:
            data, _ = self._sock.recvfrom(65535)
        except socket.timeout:
            # DEBUG TEMPORAIRE -- bug lidar shm fige pres d'un obstacle.
            print(f"DEBUG-LIDAR: TIMEOUT kind={kind} seq={self._seq} "
                 f"pos=({north:.2f},{east:.2f},{down:.2f})")
            return None
        try:
            got_kind, got_seq, values = raycast_wire.decode_reply(data)
        except ValueError as e:
            print(f"DEBUG-LIDAR: decode_reply invalide kind={kind} seq={self._seq}: {e}")
            return None
        if got_kind != kind or got_seq != self._seq:
            print(f"DEBUG-LIDAR: MISMATCH attendu kind={kind} seq={self._seq}, "
                 f"recu kind={got_kind} seq={got_seq}")
            return None
        return values


def run(sim_udp: str, gps_port: int, lidar_port: int, lidar_server_addr, viewer_addr=None):
    mav = mavutil.mavlink_connection(f"udp:{sim_udp}")
    print(f"sensor_bridge: connexion MAVLink sur {sim_udp}, attente HEARTBEAT...")
    hb = mav.wait_heartbeat()
    # wait_heartbeat() ne remplit PAS mav.target_system/target_component
    # (verifie sur pymavlink) -- sans ca, MAV_CMD_SET_MESSAGE_INTERVAL et
    # DISTANCE_SENSOR partent vers un systeme invalide, silencieusement ignores.
    mav.target_system = hb.get_srcSystem()
    mav.target_component = hb.get_srcComponent()
    print(f"sensor_bridge: HEARTBEAT recu (sysid={mav.target_system} compid={mav.target_component}), "
         f"demande du flux SIM_STATE a {1.0/TFLUNA_PERIOD_S:.0f}Hz...")

    _request_sim_state(mav)
    last_request = time.monotonic()

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    raycaster = RaycastClient(lidar_server_addr)

    last_gps_lidar_send = 0.0
    last_tfluna_send = 0.0
    last_sim_state_at = None
    sim_state_seen = False
    bridge_start = time.monotonic()

    while True:
        msg = mav.recv_match(type="SIM_STATE", blocking=True, timeout=TFLUNA_PERIOD_S)
        now = time.monotonic()

        stale = last_sim_state_at is None or (now - last_sim_state_at) > STREAM_STALE_S
        if msg is None:
            if stale and now - last_request >= REQUEST_RETRY_PERIOD_S:
                _request_sim_state(mav)
                last_request = now
            continue

        last_sim_state_at = now
        if not sim_state_seen:
            sim_state_seen = True
            print("sensor_bridge: SIM_STATE recu, flux GPS/LD06/TF Luna actif")

        # cf. avertissement ⚠️⚠️ en-tete de fichier : origine FIXE, faux si le
        # SITL demarre ailleurs que le home ArduPilot par defaut.
        north_m, east_m, down_m = sim_state_pos_ned(msg)

        # DEBUG TEMPORAIRE -- bug lidar shm fige pres d'un obstacle : verifie
        # si l'attitude/position brute d'ArduPilot est deja invalide avant
        # meme d'atteindre LidarServer (NaN/inf propagerait dans tout le
        # raycast, cf. build_lidar_frame()).
        if not all(math.isfinite(v) for v in (north_m, east_m, down_m, msg.roll, msg.pitch, msg.yaw)):
            print(f"DEBUG-LIDAR: pose/attitude non finie recue -- "
                 f"pos=({north_m},{east_m},{down_m}) roll={msg.roll} pitch={msg.pitch} yaw={msg.yaw}")

        if now - last_tfluna_send >= TFLUNA_PERIOD_S:
            last_tfluna_send = now
            reply = raycaster.query(
                raycast_wire.KIND_TFLUNA,
                north_m, east_m, down_m,
                msg.roll, msg.pitch, msg.yaw,
            )
            if reply is not None:
                time_boot_ms = int((now - bridge_start) * 1000.0)
                send_distance_sensor(mav, reply[0], time_boot_ms)

        if now - last_gps_lidar_send >= GPS_LIDAR_PERIOD_S:
            last_gps_lidar_send = now
            print(f"sensor_bridge: SIM_STATE lat={msg.lat:.7f} lon={msg.lon:.7f} "
                 f"alt={msg.alt:.2f} vn={msg.vn:.2f} ve={msg.ve:.2f} vd={msg.vd:.2f} "
                 f"roll={msg.roll:.3f} pitch={msg.pitch:.3f} yaw={msg.yaw:.3f}")
            out.sendto(build_gps_frame(msg), ("127.0.0.1", gps_port))

            reply = raycaster.query(
                raycast_wire.KIND_LIDAR360,
                north_m, east_m, down_m,
                msg.roll, msg.pitch, msg.yaw,
            )
            if reply is not None:
                out.sendto(build_lidar_frame(reply), ("127.0.0.1", lidar_port))

            # Viewer optionnel (bloc [3] non-headless, DECOUPLE de
            # LidarServer.gd -- cf. discussion roadmap etape 4) :
            # fire-and-forget, pas de reponse attendue, aucun impact sur la
            # cadence si personne n'ecoute (paquet UDP perdu dans le vide).
            if viewer_addr is not None:
                out.sendto(
                    raycast_wire.encode_pose(north_m, east_m, down_m, msg.roll, msg.pitch, msg.yaw),
                    viewer_addr,
                )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-udp", default="127.0.0.1:14551",
                        help="endpoint MAVLink d'ArduPilot SITL (PAS le port 14550 du drone lui-meme)")
    parser.add_argument("--gps-port", type=int, default=15100)
    parser.add_argument("--lidar-port", type=int, default=15101)
    parser.add_argument("--lidar-server", default="127.0.0.1:15102",
                        help="host:port de world_engine/LidarServer.gd (bloc [3])")
    parser.add_argument("--fake-fixed", action="store_true",
                        help="bypass MAVLink/SIM_STATE/LidarServer, envoie une trame GPS+LD06 fixe "
                            "en continu (diagnostic : isole UdpGpsDriver/UdpLidarDriver/SensorFusion/shm)")
    parser.add_argument("--viewer", default=None,
                        help="host:port de world_engine/Viewer.gd (optionnel, debug -- diffusion "
                            "fire-and-forget de la pose, DECOUPLE du bloc [3] critique/LidarServer)")
    args = parser.parse_args()
    if args.fake_fixed:
        run_fake_fixed(args.gps_port, args.lidar_port)
    else:
        host, port = args.lidar_server.split(":")
        viewer_addr = None
        if args.viewer:
            v_host, v_port = args.viewer.split(":")
            viewer_addr = (v_host, int(v_port))
        run(args.sim_udp, args.gps_port, args.lidar_port, (host, int(port)), viewer_addr)


if __name__ == "__main__":
    main()
