#!/usr/bin/env python3
# tools/telemetry_common.py
# ─────────────────────────────────────────────────────────────────────────────
# Logique partagée pour le format de paquets télémétrie TelMain/TelSec.
# TELEMETRY.csv est la source de vérité unique, lue par gen_telemetry.py.
# Même esprit que codes_common.py pour les codes de diagnostic.

import csv
import sys
from pathlib import Path

# token CSV -> (type C++, taille en octets)
FIELD_TYPES = {
    "u8": ("uint8_t", 1),
    "u16": ("uint16_t", 2),
    "u32": ("uint32_t", 4),
    "i8": ("int8_t", 1),
    "i16": ("int16_t", 2),
    "i32": ("int32_t", 4),
    "f32": ("float", 4),
}

DIRECTIONS = {"DOWN", "UP"}
PRIORITIES = {"Routine", "Critical"}
LINKS = {"MAIN", "BOTH"}  # pas de "SEC" seul : TelSec est un sous-ensemble de TelMain

CSV_FIELDS = ["PacketID", "Direction", "Priority", "RateHz", "RedundancyHz", "Links", "Fields"]


class TelemetryValidationError(Exception):
    pass


class Field:
    def __init__(self, name, type_token):
        self.name = name
        self.type_token = type_token
        self.cpp_type, self.size = FIELD_TYPES[type_token]


class TelemetryPacket:
    def __init__(self, packet_id, direction, priority, rate_hz, redundancy_hz, links, fields):
        self.packet_id = packet_id
        self.direction = direction
        self.priority = priority
        self.rate_hz = rate_hz
        self.redundancy_hz = redundancy_hz
        self.links = links
        self.fields = fields

    @property
    def is_event(self):
        return self.rate_hz == 0

    @property
    def size(self):
        return sum(f.size for f in self.fields)

    def to_row(self):
        return {
            "PacketID": self.packet_id,
            "Direction": self.direction,
            "Priority": self.priority,
            "RateHz": self.rate_hz,
            "RedundancyHz": self.redundancy_hz,
            "Links": self.links,
            "Fields": ",".join(f"{f.name}:{f.type_token}" for f in self.fields),
        }


def _parse_fields(raw, where):
    raw = (raw or "").strip()
    if not raw:
        return []
    fields = []
    seen = set()
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        if ":" not in token:
            raise TelemetryValidationError(f"{where}champ '{token}' invalide, attendu name:type")
        name, type_token = (p.strip() for p in token.split(":", 1))
        if type_token not in FIELD_TYPES:
            raise TelemetryValidationError(
                f"{where}type '{type_token}' inconnu pour le champ '{name}' "
                f"(attendu: {', '.join(FIELD_TYPES)})")
        if name in seen:
            raise TelemetryValidationError(f"{where}champ '{name}' dupliqué")
        seen.add(name)
        fields.append(Field(name, type_token))
    return fields


def parse_row(packet_id, direction, priority, rate_hz_raw, redundancy_hz_raw, links_raw,
              fields_raw, *, seen=None, where=""):
    packet_id = (packet_id or "").strip().upper()
    if not packet_id or not packet_id.replace("_", "").isalnum():
        raise TelemetryValidationError(f"{where}PacketID '{packet_id}' invalide")

    direction = (direction or "").strip().upper()
    if direction not in DIRECTIONS:
        raise TelemetryValidationError(
            f"{where}direction '{direction}' inconnue (attendu: {', '.join(DIRECTIONS)})")

    priority = (priority or "").strip()
    if priority not in PRIORITIES:
        raise TelemetryValidationError(
            f"{where}priorité '{priority}' inconnue (attendu: {', '.join(PRIORITIES)})")

    try:
        rate_hz = int((rate_hz_raw or "0").strip())
        redundancy_hz = int((redundancy_hz_raw or "0").strip())
    except ValueError as e:
        raise TelemetryValidationError(f"{where}RateHz/RedundancyHz doivent être des entiers") from e

    if rate_hz < 0 or redundancy_hz < 0:
        raise TelemetryValidationError(f"{where}RateHz/RedundancyHz doivent être >= 0")
    if rate_hz > 0 and redundancy_hz > 0:
        raise TelemetryValidationError(
            f"{where}RedundancyHz n'a de sens que pour un paquet événementiel (RateHz=0)")

    links = (links_raw or "").strip().upper()
    if links not in LINKS:
        raise TelemetryValidationError(
            f"{where}Links '{links}' inconnu (attendu: {', '.join(LINKS)})")

    fields = _parse_fields(fields_raw, where)

    if seen is not None:
        if packet_id in seen:
            raise TelemetryValidationError(f"{where}PacketID '{packet_id}' dupliqué")
        seen.add(packet_id)

    return TelemetryPacket(packet_id, direction, priority, rate_hz, redundancy_hz, links, fields)


def read_csv(path: Path):
    raw = path.read_bytes()
    for enc in ("utf-8", "cp850", "cp1252"):
        try:
            text = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    else:
        sys.exit(f"erreur: impossible de decoder {path} (essaye utf-8/cp1252)")

    reader = csv.DictReader(text.splitlines(), delimiter=";")
    if reader.fieldnames:
        reader.fieldnames = [f.strip() for f in reader.fieldnames]

    packets = []
    seen = set()
    for lineno, row in enumerate(reader, start=2):
        pid_raw = (row.get("PacketID") or "").strip()
        if not pid_raw:
            continue
        try:
            packets.append(parse_row(
                pid_raw, row.get("Direction"), row.get("Priority"),
                row.get("RateHz"), row.get("RedundancyHz"), row.get("Links"),
                row.get("Fields"),
                seen=seen, where=f"{path}:{lineno}: ",
            ))
        except TelemetryValidationError as e:
            sys.exit(f"erreur: {e}")

    return packets


def write_csv(path: Path, packets):
    """Réécrit le CSV dans l'ORDRE FOURNI — surtout PAS trié (contrairement
    à codes_common.write_csv) : PacketID = index de la ligne dans le CSV
    (cf. gen_telemetry.py), donc réordonner ici renumérote silencieusement
    tous les paquets et casse la compatibilité avec un binaire déjà
    compilé. L'appelant est responsable de l'ordre (ajout en fin, édition
    en place, cf. dashboard_editor.py)."""
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, delimiter=";")
        writer.writeheader()
        for p in packets:
            writer.writerow(p.to_row())
