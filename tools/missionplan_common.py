#!/usr/bin/env python3
# Logique partagée pour le format binaire du plan de mission (MissionControl).
# MISSIONPLAN.csv est la source de vérité unique (header + types de segment),
# lue par gen_missionplan.py. Même esprit que telemetry_common.py/codes_common.py
# — une seule source de vérité pour que la future page d'édition (si besoin)
# ne puisse jamais produire un plan que le générateur rejetterait.

import csv
import re
import sys
from pathlib import Path

from telemetry_common import FIELD_TYPES as SCALAR_FIELD_TYPES  # u8/u16/.../f32

CSV_FIELDS = ["Kind", "Name", "Fields"]
KINDS = {"HEADER", "SEGMENT"}

_BYTES_RE = re.compile(r"^bytes(\d+)$")


class MissionPlanValidationError(Exception):
    pass


def _resolve_type(type_token, where, field_name):
    """Type scalaire (table partagée avec la télémétrie) ou blob brut
    'bytesN' (ex: payload:bytes16) — seul le type ajouté ici, pas de champ
    nommé à l'intérieur : l'interprétation du blob dépend de actionId,
    tranchée plus tard côté routage (cf. PAYLOAD_ACTION, pas encore routé)."""
    m = _BYTES_RE.match(type_token)
    if m:
        n = int(m.group(1))
        if n == 0:
            raise MissionPlanValidationError(
                f"{where}taille de blob nulle pour le champ '{field_name}'")
        return f"std::array<uint8_t, {n}>", n
    if type_token in SCALAR_FIELD_TYPES:
        return SCALAR_FIELD_TYPES[type_token]
    raise MissionPlanValidationError(
        f"{where}type '{type_token}' inconnu pour le champ '{field_name}' "
        f"(attendu: {', '.join(SCALAR_FIELD_TYPES)}, ou bytesN)")


class Field:
    def __init__(self, name, type_token, default_raw, where):
        self.name = name
        self.type_token = type_token
        self.cpp_type, self.size = _resolve_type(type_token, where, name)
        self.default_raw = default_raw
        self.is_blob = self.type_token.startswith("bytes")

    @property
    def cpp_default(self):
        # Un blob n'a pas de littéral scalaire simple — toujours
        # zéro-initialisé, la valeur "par défaut" du CSV n'a ici que le
        # rôle d'un commentaire humain (pas de sémantique).
        if self.is_blob:
            return "{}"
        if self.cpp_type == "float":
            return f"{float(self.default_raw)}f"
        return str(int(self.default_raw, 0))


class Entity:
    """HEADER (singleton) ou un type de SEGMENT."""

    def __init__(self, kind, name, fields):
        self.kind = kind
        self.name = name
        self.fields = fields

    @property
    def size(self):
        return sum(f.size for f in self.fields)

    def to_row(self):
        return {
            "Kind": self.kind,
            "Name": self.name,
            "Fields": ",".join(
                f"{f.name}:{f.type_token}={f.default_raw}" for f in self.fields),
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
        default_raw = "0"
        if "=" in token:
            token, default_raw = (p.strip() for p in token.split("=", 1))
        if ":" not in token:
            raise MissionPlanValidationError(
                f"{where}champ '{token}' invalide, attendu name:type[=default]")
        name, type_token = (p.strip() for p in token.split(":", 1))
        if name in seen:
            raise MissionPlanValidationError(f"{where}champ '{name}' dupliqué")
        seen.add(name)
        fields.append(Field(name, type_token, default_raw, where))
    return fields


def parse_row(kind_raw, name_raw, fields_raw, *, seen_names, header_seen, where=""):
    kind = (kind_raw or "").strip().upper()
    if kind not in KINDS:
        raise MissionPlanValidationError(
            f"{where}Kind '{kind}' inconnu (attendu: {', '.join(KINDS)})")

    fields = _parse_fields(fields_raw, where)

    if kind == "HEADER":
        if header_seen[0]:
            raise MissionPlanValidationError(
                f"{where}HEADER dupliqué (une seule ligne HEADER autorisée)")
        header_seen[0] = True
        name = "HEADER"
    else:
        name = (name_raw or "").strip().upper()
        if not name or not name.replace("_", "").isalnum():
            raise MissionPlanValidationError(f"{where}Name '{name}' invalide pour un SEGMENT")
        if name in seen_names:
            raise MissionPlanValidationError(f"{where}segment '{name}' dupliqué")
        seen_names.add(name)

    return Entity(kind, name, fields)


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

    entities = []
    seen_names = set()
    header_seen = [False]
    for lineno, row in enumerate(reader, start=2):
        kind_raw = (row.get("Kind") or "").strip()
        if not kind_raw:
            continue
        try:
            entities.append(parse_row(
                kind_raw, row.get("Name"), row.get("Fields"),
                seen_names=seen_names, header_seen=header_seen,
                where=f"{path}:{lineno}: ",
            ))
        except MissionPlanValidationError as e:
            sys.exit(f"erreur: {e}")

    if not header_seen[0]:
        sys.exit(f"erreur: {path} ne définit aucune ligne HEADER")

    return entities


def write_csv(path: Path, entities):
    """Même remarque que telemetry_common.write_csv : l'ordre des lignes
    SEGMENT est l'ordre du CSV = valeur de l'enum SegmentKind généré (cf.
    gen_missionplan.py). Ne PAS trier ici, l'appelant gère l'ordre."""
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, delimiter=";")
        writer.writeheader()
        for e in entities:
            writer.writerow(e.to_row())
