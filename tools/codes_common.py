#!/usr/bin/env python3
# tools/codes_common.py
# ─────────────────────────────────────────────────────────────────────────────
# Logique partagée entre gen_codes.py (générateur C++) et codes_editor.py
# (éditeur web local) — une seule source de vérité pour le format/la
# validation des codes, pour que la page web ne puisse jamais produire un
# code que le générateur rejetterait.

import csv
import re
import sys
from pathlib import Path

# ─── Tables stables (composant/catégorie -> id binaire). NE PAS réordonner
# des entrées existantes : ça changerait le binaire de codes déjà déployés.
# Ajouter seulement de nouvelles entrées à la fin.
COMPONENTS = {
    "FC": (1, "Flight Controller"),
    "SF": (2, "Sensor Fusion"),
    "NV": (3, "Navigation"),
    "MC": (4, "Mission Control"),
    "SM": (5, "Sys Monitoring"),
    "MV": (6, "Mavlink Interface"),
    "GW": (7, "Global Watchdog"),
}

CATEGORIES = {
    "P": (1, "Power"),
    "S": (2, "Sensor"),
    "A": (3, "Action"),
}

SEVERITIES = {"info": "Info", "warning": "Warning", "critical": "Critical",
             "emergency": "Emergency"}
# Sévérité >= Critical -> stockage "majeur" (ensemble actif) ; sinon "mineur"
# (ring buffer). Cf. GUIDE_COMPOSANTS.md / discussion architecture.
MAJOR_SEVERITIES = {"Critical", "Emergency"}

CODE_RE = re.compile(r"^([A-Z]{2})([A-Z])(\d{3})$")

CSV_FIELDS = ["Code", "Component", "Category", "Name", "Description", "Criticity"]


class DiagCode:
    def __init__(self, code, component_label, category_label, name, description, severity):
        self.code = code
        self.component_label = component_label
        self.category_label = category_label
        self.name = name
        self.description = description
        self.severity = severity

    @property
    def component_id(self):
        return COMPONENTS[self.component_label][0]

    @property
    def category_id(self):
        return CATEGORIES[self.category_label][0]

    @property
    def number(self):
        return int(self.code[3:6])

    @property
    def is_major(self):
        return self.severity in MAJOR_SEVERITIES

    def to_row(self):
        return {
            "Code": self.code,
            "Component": COMPONENTS[self.component_label][1],
            "Category": CATEGORIES[self.category_label][1],
            "Name": self.name,
            "Description": self.description,
            "Criticity": self.severity,
        }


class CodeValidationError(Exception):
    pass


def _label_from_full_name(table, full_name):
    for label, (_, full) in table.items():
        if full == full_name or label == full_name:
            return label
    return None


def parse_row(code_raw, component_raw, category_raw, name, description, severity_raw,
              *, seen=None, where=""):
    """Valide une ligne brute (types str) et renvoie un DiagCode. Lève
    CodeValidationError avec un message clair sinon — utilisé à la fois par
    le parsing CSV (gen_codes.py) et par l'API web (codes_editor.py)."""
    code_raw = (code_raw or "").strip().upper()
    m = CODE_RE.match(code_raw)
    if not m:
        raise CodeValidationError(
            f"{where}code '{code_raw}' invalide, attendu [2 lettres][1 lettre]"
            f"[3 chiffres] (ex FCP001)")

    comp_label, cat_label, _ = m.groups()

    # Accepte soit le libellé court (FC) soit le nom complet (Flight Controller).
    if comp_label not in COMPONENTS:
        found = _label_from_full_name(COMPONENTS, component_raw)
        if found:
            comp_label = found
    if comp_label not in COMPONENTS:
        raise CodeValidationError(
            f"{where}composant '{comp_label}' inconnu "
            f"(ajoute-le dans COMPONENTS de tools/codes_common.py)")

    if cat_label not in CATEGORIES:
        found = _label_from_full_name(CATEGORIES, category_raw)
        if found:
            cat_label = found
    if cat_label not in CATEGORIES:
        raise CodeValidationError(
            f"{where}catégorie '{cat_label}' inconnue "
            f"(ajoute-la dans CATEGORIES de tools/codes_common.py)")

    severity_key = (severity_raw or "").strip().lower()
    if severity_key not in SEVERITIES:
        raise CodeValidationError(
            f"{where}sévérité '{severity_raw}' inconnue "
            f"(attendu: {', '.join(SEVERITIES.values())})")

    if seen is not None:
        if code_raw in seen:
            raise CodeValidationError(f"{where}code '{code_raw}' dupliqué")
        seen.add(code_raw)

    return DiagCode(
        code=code_raw,
        component_label=comp_label,
        category_label=cat_label,
        name=(name or "").strip(),
        description=(description or "").strip(),
        severity=SEVERITIES[severity_key],
    )


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
    codes = []
    seen = set()

    for lineno, row in enumerate(reader, start=2):
        code_raw = (row.get("Code") or "").strip()
        if not code_raw:
            continue  # ligne vide / garbage (ex: fin de fichier Excel)

        try:
            codes.append(parse_row(
                code_raw, row.get("Component"), row.get("Category"),
                row.get("Name"), row.get("Description"), row.get("Criticity"),
                seen=seen, where=f"{path}:{lineno}: ",
            ))
        except CodeValidationError as e:
            sys.exit(f"erreur: {e}")

    return codes


def write_csv(path: Path, codes):
    """Réécrit le CSV en entier, trié par Code, encodage UTF-8 propre."""
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, delimiter=";")
        writer.writeheader()
        for c in sorted(codes, key=lambda c: c.code):
            writer.writerow(c.to_row())
