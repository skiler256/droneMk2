#!/usr/bin/env python3
# sitl/mission_driver.py
# ─────────────────────────────────────────────────────────────────────────────
# Envoie des commandes GCS au vrai système sous test, via le VRAI canal
# TELEM::* (UDP loopback TelMain, cf. UdpTelemetryDriver/SysMonitoring) —
# pas un canal MAVLink séparé : le système parle par ses interfaces externes
# (cf. INTEGRATION_TEST.md §1.3), et ce canal existe déjà (réutilise
# tools/dashboard/telemetry_codec.py, même source de vérité que le C++).
#
# kUdpTelMainLocalPort (5601, cf. src/main.cpp) : le drone ECOUTE ici les
# commandes montantes (CMD_ARM etc.) — c'est la cible d'envoi ici.
#
# kUdpTelSecLocalPort (5611) : lien secondaire (433MHz cote reel) --
# heartbeat seulement, pas de commandes dessus (le sec sert de redondance
# de detection de lien, cf. SharedComMem/GCSRxTask, pas de canal de
# commande separe cote systeme).

import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools" / "dashboard"))
from telemetry_codec import load_packets, encode  # noqa: E402

TELMAIN_TX_PORT = 5601
TELSEC_TX_PORT = 5611
TELEMETRY_CSV = Path(__file__).resolve().parent.parent / "TELEMETRY.csv"

_BY_ID, _BY_NAME = load_packets(TELEMETRY_CSV)


class MissionDriver:
    def __init__(self, tx_port: int = TELMAIN_TX_PORT, sec_tx_port: int = TELSEC_TX_PORT):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tx_port = tx_port
        self._sec_tx_port = sec_tx_port
        self._hb_stop = threading.Event()
        self._hb_thread = None

    def _send(self, name: str, fields: dict | None = None):
        frame = encode(_BY_NAME, name, fields or {})
        self._sock.sendto(frame, ("127.0.0.1", self._tx_port))

    def _send_sec(self, name: str, fields: dict | None = None):
        frame = encode(_BY_NAME, name, fields or {})
        self._sock.sendto(frame, ("127.0.0.1", self._sec_tx_port))

    def start_heartbeat(self, period_s: float = 1.0):
        """HEARTBEAT_GCS periodique (1Hz cf. TELEMETRY.csv) sur les DEUX
        liens (main + sec) -- sans ca, GCSRxTask leve SMC002/SMC003 (lien
        GCS perdu) apres kGcsHeartbeatTimeout (3s), meme si le scenario
        n'envoie aucune commande pendant un moment. Le sec (433MHz cote
        reel) n'a pas de canal de commande separe ici, juste le heartbeat
        de detection de lien -- matche une vraie GCS avec redondance."""
        def loop():
            while not self._hb_stop.is_set():
                self._send("HEARTBEAT_GCS")
                self._send_sec("HEARTBEAT_GCS")
                time.sleep(period_s)
        self._hb_thread = threading.Thread(target=loop, daemon=True)
        self._hb_thread.start()

    def stop_heartbeat(self):
        self._hb_stop.set()
        if self._hb_thread:
            self._hb_thread.join(timeout=2)

    def arm(self):
        self._send("CMD_ARM")

    def disarm(self):
        self._send("CMD_DISARM")

    def takeoff(self, altitude_m: float):
        self._send("CMD_TAKEOFF", {"altitude": altitude_m})

    def hold(self):
        self._send("CMD_HOLD")

    def resume(self):
        self._send("CMD_RESUME")

    def land(self):
        self._send("CMD_LAND")

    def answer(self, seq: int, choice: int):
        self._send("CMD_ANSWER", {"seq": seq, "choice": choice})
