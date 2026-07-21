#!/usr/bin/env python3
# tools/codes_editor.py
# ─────────────────────────────────────────────────────────────────────────────
# Petit serveur web local (stdlib uniquement, pas de dépendance) pour
# visualiser/éditer INDEX.csv (codes de diagnostic) ET TELEMETRY.csv
# (paquets télémétrie). Ne touche QUE les CSV — la régénération de
# codes.hpp/telemetry.hpp reste la responsabilité du build (cf.
# CMakeLists.txt / tools/gen_codes.py / tools/gen_telemetry.py).
#
# Usage : python3 tools/codes_editor.py [--port 8765] [--csv INDEX.csv] [--telemetry-csv TELEMETRY.csv]
# Puis : cmake --build build/  (ou make) pour régénérer les deux headers.
#
# IMPORTANT pour les paquets télémétrie : contrairement aux codes (dont
# l'identifiant est la chaîne "FCP001" elle-même), le PacketID d'un paquet
# est son INDEX dans TELEMETRY.csv (cf. gen_telemetry.py) — l'ordre des
# lignes fait donc partie du format binaire. Cette page préserve toujours
# l'ordre existant (ajout en fin, édition en place) ; supprimer un paquet
# décale les ID de tous ceux qui le suivent et casse la compatibilité avec
# un binaire déjà compilé — à éviter après un premier déploiement réel.

import argparse
import json
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from codes_common import (
    COMPONENTS, CATEGORIES, SEVERITIES, CodeValidationError, parse_row,
    read_csv, write_csv,
)
from telemetry_common import (
    DIRECTIONS, FIELD_TYPES, LINKS, PRIORITIES, TelemetryValidationError,
)
from telemetry_common import parse_row as parse_telemetry_row
from telemetry_common import read_csv as read_telemetry_csv
from telemetry_common import write_csv as write_telemetry_csv

REPO_ROOT = Path(__file__).resolve().parent.parent
HTML_PATH = Path(__file__).resolve().parent / "codes_editor.html"


def codes_to_json(codes):
    return [
        {
            "code": c.code,
            "component_label": c.component_label,
            "component_name": COMPONENTS[c.component_label][1],
            "category_label": c.category_label,
            "category_name": CATEGORIES[c.category_label][1],
            "number": c.number,
            "name": c.name,
            "description": c.description,
            "severity": c.severity,
            "is_major": c.is_major,
        }
        for c in sorted(codes, key=lambda c: c.code)
    ]


def next_free_number(codes, component_label, category_label):
    used = {
        c.number for c in codes
        if c.component_label == component_label and c.category_label == category_label
    }
    n = 1
    while n in used:
        n += 1
    return n


def resolve_number(payload, codes, component_label, category_label):
    """Renvoie (number, error) — error est un message str si invalide."""
    number = payload.get("number")
    if number in (None, ""):
        return next_free_number(codes, component_label, category_label), None
    try:
        number = int(number)
    except (TypeError, ValueError):
        return None, f"numéro invalide: {number!r}"
    if not (0 <= number <= 999):
        return None, "numéro hors plage (000-999)"
    return number, None


def build_code_from_payload(payload, codes_for_dup_check):
    """Valide `payload` (dict JSON du formulaire) et renvoie (DiagCode, None)
    ou (None, message_erreur)."""
    component_label = (payload.get("component") or "").strip().upper()
    category_label = (payload.get("category") or "").strip().upper()

    number, err = resolve_number(payload, codes_for_dup_check, component_label,
                                 category_label)
    if err:
        return None, err

    code_str = f"{component_label}{category_label}{number:03d}"
    seen = {c.code for c in codes_for_dup_check}

    try:
        code = parse_row(
            code_str, payload.get("component"), payload.get("category"),
            payload.get("name"), payload.get("description"),
            payload.get("severity"), seen=seen,
        )
    except CodeValidationError as e:
        return None, str(e)
    return code, None


def packets_to_json(packets):
    return [
        {
            "id": i,
            "packet_id": p.packet_id,
            "direction": p.direction,
            "priority": p.priority,
            "rate_hz": p.rate_hz,
            "redundancy_hz": p.redundancy_hz,
            "links": p.links,
            "is_event": p.is_event,
            "size": p.size,
            "fields": [{"name": f.name, "type": f.type_token} for f in p.fields],
        }
        for i, p in enumerate(packets)
    ]


def _fields_payload_to_raw(fields_payload):
    """[{"name":..,"type":..}, ...] -> "name:type,name:type" attendu par
    telemetry_common.parse_row."""
    parts = []
    for f in fields_payload or []:
        name = (f.get("name") or "").strip()
        type_token = (f.get("type") or "").strip()
        if not name or not type_token:
            continue
        parts.append(f"{name}:{type_token}")
    return ",".join(parts)


def build_packet_from_payload(payload, packets_for_dup_check):
    """Valide `payload` et renvoie (TelemetryPacket, None) ou (None, erreur)."""
    seen = {p.packet_id for p in packets_for_dup_check}
    fields_raw = _fields_payload_to_raw(payload.get("fields"))

    try:
        packet = parse_telemetry_row(
            payload.get("packet_id"), payload.get("direction"),
            payload.get("priority"), payload.get("rate_hz"),
            payload.get("redundancy_hz"), payload.get("links"),
            fields_raw, seen=seen,
        )
    except TelemetryValidationError as e:
        return None, str(e)
    return packet, None


def make_handler(csv_path: Path, telemetry_csv_path: Path):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # silencieux — outil local, pas besoin de logs d'accès

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

            if self.path == "/api/codes":
                codes = read_csv(csv_path)
                self._send_json(200, codes_to_json(codes))
                return

            if self.path == "/api/meta":
                self._send_json(200, {
                    "components": {k: v[1] for k, v in COMPONENTS.items()},
                    "categories": {k: v[1] for k, v in CATEGORIES.items()},
                    "severities": sorted(set(SEVERITIES.values())),
                })
                return

            if self.path.startswith("/api/next_number"):
                qs = parse_qs(urlparse(self.path).query)
                comp = (qs.get("component") or [""])[0]
                cat = (qs.get("category") or [""])[0]
                codes = read_csv(csv_path)
                self._send_json(200, {"number": next_free_number(codes, comp, cat)})
                return

            if self.path == "/api/packets":
                packets = read_telemetry_csv(telemetry_csv_path)
                self._send_json(200, packets_to_json(packets))
                return

            if self.path == "/api/packets/meta":
                self._send_json(200, {
                    "directions": sorted(DIRECTIONS),
                    "priorities": sorted(PRIORITIES),
                    "links": sorted(LINKS),
                    "field_types": list(FIELD_TYPES.keys()),
                })
                return

            self.send_response(404)
            self.end_headers()

        def _read_json_body(self):
            length = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(length) or b"{}")

        def do_POST(self):
            if self.path == "/api/codes":
                try:
                    payload = self._read_json_body()
                except json.JSONDecodeError:
                    self._send_json(400, {"error": "JSON invalide"})
                    return

                codes = read_csv(csv_path)
                new_code, err = build_code_from_payload(payload, codes)
                if err:
                    self._send_json(400, {"error": err})
                    return

                codes.append(new_code)
                write_csv(csv_path, codes)
                self._send_json(201, codes_to_json(codes))
                return

            if self.path == "/api/packets":
                # Toujours ajouté EN FIN de liste : préserve les PacketID
                # déjà attribués aux paquets existants.
                try:
                    payload = self._read_json_body()
                except json.JSONDecodeError:
                    self._send_json(400, {"error": "JSON invalide"})
                    return

                packets = read_telemetry_csv(telemetry_csv_path)
                new_packet, err = build_packet_from_payload(payload, packets)
                if err:
                    self._send_json(400, {"error": err})
                    return

                packets.append(new_packet)
                write_telemetry_csv(telemetry_csv_path, packets)
                self._send_json(201, packets_to_json(packets))
                return

            self.send_response(404)
            self.end_headers()

        def do_PUT(self):
            path = urlparse(self.path).path

            if path.startswith("/api/codes/"):
                # PUT /api/codes/<code> — modifie un code existant. Le
                # nouveau payload peut changer n'importe quel champ, y
                # compris composant/catégorie/numéro (donc l'identifiant
                # lui-même) : on retire l'ancien puis revalide le nouveau
                # comme un ajout, en excluant l'ancien de la vérification
                # de doublon.
                old_code = unquote(path[len("/api/codes/"):]).strip().upper()

                try:
                    payload = self._read_json_body()
                except json.JSONDecodeError:
                    self._send_json(400, {"error": "JSON invalide"})
                    return

                codes = read_csv(csv_path)
                if not any(c.code == old_code for c in codes):
                    self._send_json(404, {"error": f"code '{old_code}' introuvable"})
                    return

                remaining = [c for c in codes if c.code != old_code]
                updated, err = build_code_from_payload(payload, remaining)
                if err:
                    self._send_json(400, {"error": err})
                    return

                remaining.append(updated)
                write_csv(csv_path, remaining)
                self._send_json(200, codes_to_json(remaining))
                return

            if path.startswith("/api/packets/"):
                # PUT /api/packets/<packet_id> — édite EN PLACE (même
                # index), donc son PacketID (et ceux des autres paquets)
                # ne bouge pas, même si on renomme l'identifiant texte.
                old_name = unquote(path[len("/api/packets/"):]).strip().upper()

                try:
                    payload = self._read_json_body()
                except json.JSONDecodeError:
                    self._send_json(400, {"error": "JSON invalide"})
                    return

                packets = read_telemetry_csv(telemetry_csv_path)
                idx = next((i for i, p in enumerate(packets) if p.packet_id == old_name), None)
                if idx is None:
                    self._send_json(404, {"error": f"paquet '{old_name}' introuvable"})
                    return

                others = packets[:idx] + packets[idx + 1:]
                updated, err = build_packet_from_payload(payload, others)
                if err:
                    self._send_json(400, {"error": err})
                    return

                packets[idx] = updated
                write_telemetry_csv(telemetry_csv_path, packets)
                self._send_json(200, packets_to_json(packets))
                return

            self.send_response(404)
            self.end_headers()

        def do_DELETE(self):
            path = urlparse(self.path).path

            if path.startswith("/api/codes/"):
                code = unquote(path[len("/api/codes/"):]).strip().upper()

                codes = read_csv(csv_path)
                remaining = [c for c in codes if c.code != code]
                if len(remaining) == len(codes):
                    self._send_json(404, {"error": f"code '{code}' introuvable"})
                    return

                write_csv(csv_path, remaining)
                self._send_json(200, codes_to_json(remaining))
                return

            if path.startswith("/api/packets/"):
                # Décale les PacketID de tout ce qui suit — cf. avertissement
                # en tête de fichier. Accepté volontairement tant qu'aucun
                # binaire réel n'est déployé avec ce format.
                name = unquote(path[len("/api/packets/"):]).strip().upper()

                packets = read_telemetry_csv(telemetry_csv_path)
                remaining = [p for p in packets if p.packet_id != name]
                if len(remaining) == len(packets):
                    self._send_json(404, {"error": f"paquet '{name}' introuvable"})
                    return

                write_telemetry_csv(telemetry_csv_path, remaining)
                self._send_json(200, packets_to_json(remaining))
                return

            self.send_response(404)
            self.end_headers()

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--csv", default="INDEX.csv", type=Path)
    parser.add_argument("--telemetry-csv", default="TELEMETRY.csv", type=Path)
    parser.add_argument("--no-browser", action="store_true")
    # 0.0.0.0 : accessible depuis tout le réseau local, sans authentification
    # — pratique en dev quand le port-forwarding SSH pose souci, mais garder
    # 127.0.0.1 par défaut (accès local / tunnel SSH uniquement).
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args()

    csv_path = args.csv if args.csv.is_absolute() else REPO_ROOT / args.csv
    if not csv_path.exists():
        raise SystemExit(f"erreur: {csv_path} introuvable")

    telemetry_csv_path = (args.telemetry_csv if args.telemetry_csv.is_absolute()
                          else REPO_ROOT / args.telemetry_csv)
    if not telemetry_csv_path.exists():
        raise SystemExit(f"erreur: {telemetry_csv_path} introuvable")

    server = HTTPServer((args.host, args.port), make_handler(csv_path, telemetry_csv_path))
    url = f"http://{args.host}:{args.port}/"
    print(f"Éditeur codes + paquets sur {url} (Ctrl+C pour arrêter)")
    print(f"CSV codes : {csv_path}")
    print(f"CSV paquets : {telemetry_csv_path}")
    print("Après modification, régénère : cmake --build build/ (ou make)")

    if not args.no_browser:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
