#!/usr/bin/env python3
# tools/codes_editor.py
# ─────────────────────────────────────────────────────────────────────────────
# Petit serveur web local (stdlib uniquement, pas de dépendance) pour
# visualiser et ajouter des codes dans INDEX.csv. Ne touche QUE le CSV — la
# régénération de codes.hpp reste la responsabilité du build (cf.
# CMakeLists.txt / tools/gen_codes.py), pas de cette page.
#
# Usage : python3 tools/codes_editor.py [--port 8765] [--csv INDEX.csv]
# Puis : cmake --build build/  (ou make, ou python3 tools/gen_codes.py)
# pour régénérer codes.hpp avant de recompiler.

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


def make_handler(csv_path: Path):
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

            self.send_response(404)
            self.end_headers()

        def _read_json_body(self):
            length = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(length) or b"{}")

        def do_POST(self):
            if self.path != "/api/codes":
                self.send_response(404)
                self.end_headers()
                return

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

        def do_PUT(self):
            # PUT /api/codes/<code> — modifie un code existant. Le nouveau
            # payload peut changer n'importe quel champ, y compris
            # composant/catégorie/numéro (donc l'identifiant lui-même) : on
            # retire l'ancien puis revalide le nouveau comme un ajout, en
            # excluant l'ancien de la vérification de doublon.
            path = urlparse(self.path).path
            if not path.startswith("/api/codes/"):
                self.send_response(404)
                self.end_headers()
                return
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

        def do_DELETE(self):
            path = urlparse(self.path).path
            if not path.startswith("/api/codes/"):
                self.send_response(404)
                self.end_headers()
                return
            code = unquote(path[len("/api/codes/"):]).strip().upper()

            codes = read_csv(csv_path)
            remaining = [c for c in codes if c.code != code]
            if len(remaining) == len(codes):
                self._send_json(404, {"error": f"code '{code}' introuvable"})
                return

            write_csv(csv_path, remaining)
            self._send_json(200, codes_to_json(remaining))

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--csv", default="INDEX.csv", type=Path)
    parser.add_argument("--no-browser", action="store_true")
    # 0.0.0.0 : accessible depuis tout le réseau local, sans authentification
    # — pratique en dev quand le port-forwarding SSH pose souci, mais garder
    # 127.0.0.1 par défaut (accès local / tunnel SSH uniquement).
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args()

    csv_path = args.csv if args.csv.is_absolute() else REPO_ROOT / args.csv
    if not csv_path.exists():
        raise SystemExit(f"erreur: {csv_path} introuvable")

    server = HTTPServer((args.host, args.port), make_handler(csv_path))
    url = f"http://{args.host}:{args.port}/"
    print(f"Éditeur de codes sur {url} (Ctrl+C pour arrêter)")
    print(f"CSV : {csv_path}")
    print("Après modification, régénère codes.hpp : cmake --build build/ (ou make)")

    if not args.no_browser:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
