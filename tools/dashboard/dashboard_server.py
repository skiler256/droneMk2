#!/usr/bin/env python3
# tools/dashboard/dashboard_server.py
# ─────────────────────────────────────────────────────────────────────────────
# Serveur web unique pour tous les outils de dev droneMk2 — un menu ("/"),
# et une page isolée par cas de test :
#   /sysmonitoring  — télémétrie TelMain/TelSec (UDP loopback), cf. §SysMonitoring
#   /sitl/mv        — état shm FC/Nav/SF de MavlinkInterface face au SITL
#   /hil/sf_mv      — état shm FC/SF/Lidar/Sys de SensorFusion+MavlinkInterface face au vrai matériel
#   /nav/sitl       — état shm FC/Nav/Seg de Navigation+MavlinkInterface face au SITL (sans SensorFusion)
#
# /sitl/mv, /hil/sf_mv et /nav/sitl lisent tous les trois le MÊME pont
# UDP_bridge.cpp (un seul process, namespaces SITL_*/HIL_*/NAV_SITL_* — cf.
# IntegrationTests/UDP_bridge.cpp), distingués par le préfixe du segment.
#
# Stdlib uniquement (mêmes contraintes que tools/codes_editor.py). Pas de
# WebSocket — un polling toutes les ~300ms suffit pour du debug.
#
# Usage : python3 tools/dashboard/dashboard_server.py [--port 8766]

import argparse
import json
import socket
import struct
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from telemetry_codec import decode, encode, load_packets

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
HTML_DIR = Path(__file__).resolve().parent
DEFAULT_CSV = REPO_ROOT / "TELEMETRY.csv"

# ─── SysMonitoring (UdpTelemetryDriver, cf. src/main.cpp) ───────────────────
TELMAIN_RX_PORT = 5602
TELMAIN_TX_PORT = 5601
TELSEC_RX_PORT = 5612
TELSEC_TX_PORT = 5611

# ─── Pont d'intégration SITL/HIL (cf. src/IntegrationTests/UDP_bridge.cpp) ──
BRIDGE_STATUS_PORT = 15001   # le pont ENVOIE ici, on écoute
BRIDGE_COMMAND_PORT = 15000  # on ENVOIE ici pour injecter (champ NS=SITL|HIL requis)


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


class BridgeState:
    """Dernier snapshot connu par segment (SITL_FC/HIL_SF/...), thread-safe."""

    def __init__(self):
        self._lock = threading.Lock()
        self._latest = {}

    def update(self, segment, fields):
        with self._lock:
            self._latest[segment] = {"fields": fields, "ts": time.time()}

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


def _coerce_number(value: str):
    """UDP_bridge envoie tout en texte — reconvertit en nombre côté JS si
    possible, sinon garde la chaîne telle quelle (ex: pas de valeur numérique
    inattendue)."""
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def parse_bridge_datagram(data: bytes):
    """Format UDP_bridge : ligne 1 'SEGMENT=<NOM>', puis 'cle=valeur' par
    ligne. Renvoie (segment, fields) ou None si mal formé."""
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        return None

    lines = text.splitlines()
    if not lines or not lines[0].startswith("SEGMENT="):
        return None

    segment = lines[0][len("SEGMENT="):]
    fields = {}
    for line in lines[1:]:
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        fields[key] = _coerce_number(value)

    return segment, fields


def bridge_listener(state: BridgeState, stop_event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", BRIDGE_STATUS_PORT))
    sock.settimeout(0.5)

    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        parsed = parse_bridge_datagram(data)
        if parsed:
            segment, fields = parsed
            state.update(segment, fields)

    sock.close()


def make_handler(state, packets_by_name, bridge_state):
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

        def _send_html(self, filename):
            path = HTML_DIR / filename
            if not path.exists():
                self.send_response(404)
                self.end_headers()
                return
            html = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)

        def do_GET(self):
            # ─── Pages ───────────────────────────────────────────────────
            if self.path == "/":
                self._send_html("menu.html")
                return
            if self.path == "/sysmonitoring":
                self._send_html("sysmonitoring.html")
                return
            if self.path == "/sitl/mv":
                self._send_html("sitl_mv.html")
                return
            if self.path == "/hil/sf_mv":
                self._send_html("hil_sf_mv.html")
                return
            if self.path == "/nav/sitl":
                self._send_html("nav_sitl.html")
                return

            # ─── API SysMonitoring ──────────────────────────────────────
            if self.path == "/api/sysmonitoring/telemetry":
                self._send_json(200, state.snapshot())
                return

            if self.path == "/api/sysmonitoring/commands":
                cmds = {
                    name: [{"name": f.name, "type": f.type_token} for f in p.fields]
                    for name, p in packets_by_name.items()
                    if p.direction == "UP"
                }
                self._send_json(200, cmds)
                return

            # ─── API pont d'intégration (SITL_*/HIL_*, segments préfixés) ──
            if self.path == "/api/bridge/status":
                self._send_json(200, bridge_state.snapshot())
                return

            self.send_response(404)
            self.end_headers()

        def do_POST(self):
            if self.path == "/api/sysmonitoring/command":
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
                return

            if self.path == "/api/bridge/inject":
                length = int(self.headers.get("Content-Length", 0))
                try:
                    payload = json.loads(self.rfile.read(length) or b"{}")
                except json.JSONDecodeError:
                    self._send_json(400, {"error": "JSON invalide"})
                    return

                ns = payload.get("ns")
                cmd = payload.get("cmd")
                fields = payload.get("fields", {})
                if ns not in ("SITL", "HIL", "NAV_SITL"):
                    self._send_json(400, {"error": f"namespace '{ns}' inconnu"})
                    return
                if cmd not in ("NAV", "GPSMAG", "MCCMD", "SEGMENT"):
                    self._send_json(400, {"error": f"commande '{cmd}' inconnue"})
                    return

                # Même protocole texte que UDP_bridge.cpp attend :
                # "CMD=<NOM>\nNS=<SITL|HIL|NAV_SITL>\ncle=valeur\n..." — cf. handleCommand().
                # CMD=GPSMAG+NS=HIL est rejeté côté pont (SensorFusion réel
                # est déjà writer de gpsMag) — pas de garde ici, un seul
                # endroit pour cette règle plutôt que la dupliquer.
                lines = [f"CMD={cmd}", f"NS={ns}"] + [f"{k}={v}" for k, v in fields.items()]
                text = "\n".join(lines).encode("utf-8")

                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(text, ("127.0.0.1", BRIDGE_COMMAND_PORT))
                sock.close()

                self._send_json(200, {"ok": True})
                return

            self.send_response(404)
            self.end_headers()

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
    bridge_state = BridgeState()
    stop_event = threading.Event()
    listeners = [
        threading.Thread(target=udp_listener,
                         args=(TELMAIN_RX_PORT, packets_by_id, state, "main", stop_event),
                         daemon=True),
        threading.Thread(target=udp_listener,
                         args=(TELSEC_RX_PORT, packets_by_id, state, "sec", stop_event),
                         daemon=True),
        threading.Thread(target=bridge_listener,
                         args=(bridge_state, stop_event),
                         daemon=True),
    ]
    for t in listeners:
        t.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(state, packets_by_name, bridge_state))
    url = f"http://{args.host}:{args.port}/"
    print(f"Outils de dev sur {url} (Ctrl+C pour arrêter)")
    print(f"SysMonitoring — UDP rx: TelMain={TELMAIN_RX_PORT} TelSec={TELSEC_RX_PORT} "
         f"— tx commandes: TelMain={TELMAIN_TX_PORT} TelSec={TELSEC_TX_PORT}")
    print(f"Pont intégration (SITL/HIL) — UDP rx statut: {BRIDGE_STATUS_PORT} "
         f"— tx commandes: {BRIDGE_COMMAND_PORT}")

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
