#!/usr/bin/env python3
# tools/dashboard/dashboard_server.py
# ─────────────────────────────────────────────────────────────────────────────
# Tableau de bord de dev pour SysMonitoring — se connecte aux ports UDP
# loopback des drivers factices (UdpTelemetryDriver, cf. src/main.cpp) :
# écoute la télémétrie descendante, relaie les commandes montantes en
# binaire TELEM::* vers le drone.
#
# Stdlib uniquement (mêmes contraintes que tools/codes_editor.py) : serveur
# HTTP local qui sert dashboard.html et expose /api/telemetry (poll côté
# navigateur) + /api/command (POST -> forward UDP). Pas de WebSocket — un
# polling toutes les ~300ms suffit pour du debug, et évite d'implémenter
# le framing RFC6455 à la main pour un outil jetable.
#
# Usage : python3 tools/dashboard/dashboard_server.py [--port 8766]
# Prérequis : le binaire `drone` doit tourner (role sysmonitoring lancé par
# GlobalWatchdog) pour qu'il y ait quelque chose à l'autre bout des sockets.

import argparse
import json
import socket
import struct
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

from telemetry_codec import decode, encode, load_packets

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
HTML_PATH = Path(__file__).resolve().parent / "dashboard.html"
DEFAULT_CSV = REPO_ROOT / "TELEMETRY.csv"

# Doivent correspondre à src/main.cpp (kUdpTelMain*/kUdpTelSec*). Le drone
# BIND sur les ports *_TX (il y écoute nos commandes) et ENVOIE vers les
# ports *_RX (on y écoute sa télémétrie) — miroir exact des constructeurs
# UdpTelemetryDriver(localPort, peerPort) côté C++.
TELMAIN_RX_PORT = 5602
TELMAIN_TX_PORT = 5601
TELSEC_RX_PORT = 5612
TELSEC_TX_PORT = 5611


class TelemetryState:
    """Dernière valeur connue par paquet, thread-safe (un thread par lien
    UDP écrit, le thread HTTP lit)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._latest = {}

    def update(self, name, fields, link):
        with self._lock:
            self._latest[name] = {"fields": fields, "ts": time.time(), "link": link}

    def snapshot(self):
        with self._lock:
            return dict(self._latest)


def udp_listener(port, packets_by_id, state, link_label, stop_event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    sock.settimeout(0.5)  # pour repasser régulièrement voir stop_event

    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        decoded = decode(packets_by_id, data)
        if decoded:
            state.update(decoded["name"], decoded["fields"], link_label)

    sock.close()


def make_handler(state, packets_by_name):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send_json(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                html = HTML_PATH.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return

            if self.path == "/api/telemetry":
                self._send_json(200, state.snapshot())
                return

            if self.path == "/api/commands":
                # Décrit les paquets montants pour que la page construise
                # ses formulaires sans dupliquer la liste des champs.
                cmds = {
                    name: [{"name": f.name, "type": f.type_token} for f in p.fields]
                    for name, p in packets_by_name.items()
                    if p.direction == "UP"
                }
                self._send_json(200, cmds)
                return

            self.send_response(404)
            self.end_headers()

        def do_POST(self):
            if self.path != "/api/command":
                self.send_response(404)
                self.end_headers()
                return

            length = int(self.headers.get("Content-Length", 0))
            try:
                payload = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                self._send_json(400, {"error": "JSON invalide"})
                return

            name = payload.get("name")
            link = payload.get("link", "main")
            fields = payload.get("fields", {})

            if name not in packets_by_name or packets_by_name[name].direction != "UP":
                self._send_json(400, {"error": f"commande '{name}' inconnue"})
                return

            try:
                frame = encode(packets_by_name, name, fields)
            except (struct.error, KeyError, ValueError) as e:
                self._send_json(400, {"error": f"encodage: {e}"})
                return

            port = TELMAIN_TX_PORT if link == "main" else TELSEC_TX_PORT
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(frame, ("127.0.0.1", port))
            sock.close()

            self._send_json(200, {"ok": True})

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--csv", default=str(DEFAULT_CSV), type=Path)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args()

    csv_path = args.csv if Path(args.csv).is_absolute() else REPO_ROOT / args.csv
    if not csv_path.exists():
        raise SystemExit(f"erreur: {csv_path} introuvable")

    packets_by_id, packets_by_name = load_packets(csv_path)

    state = TelemetryState()
    stop_event = threading.Event()
    listeners = [
        threading.Thread(target=udp_listener,
                         args=(TELMAIN_RX_PORT, packets_by_id, state, "main", stop_event),
                         daemon=True),
        threading.Thread(target=udp_listener,
                         args=(TELSEC_RX_PORT, packets_by_id, state, "sec", stop_event),
                         daemon=True),
    ]
    for t in listeners:
        t.start()

    server = HTTPServer((args.host, args.port), make_handler(state, packets_by_name))
    url = f"http://{args.host}:{args.port}/"
    print(f"Dashboard sur {url} (Ctrl+C pour arrêter)")
    print(f"UDP rx: TelMain={TELMAIN_RX_PORT} TelSec={TELSEC_RX_PORT} "
         f"— tx commandes: TelMain={TELMAIN_TX_PORT} TelSec={TELSEC_TX_PORT}")

    if not args.no_browser:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
