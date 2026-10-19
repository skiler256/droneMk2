#!/usr/bin/env python3
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
    "C": (4, "Communication"),
    "O": (5, "Command"),  # Ordre — C déjà pris par Communication
}

SEVERITIES = {"info": "Info", "warning": "Warning", "critical": "Critical",
             "emergency": "Emergency"}
# Sévérité >= Critical -> stockage "majeur" (ensemble actif) ; sinon "mineur"
# (ring buffer). Cf. GUIDE_COMPOSANTS.md / discussion architecture.
MAJOR_SEVERITIES = {"Critical", "Emergency"}

CODE_RE = re.compile(r"^([A-Z]{2})([A-Z])(\d{3})$")

CSV_FIELDS = ["Code", "Component", "Category", "Name", "Description", "Criticity", "Alarm",
              "Panel", "Schematic"]

# Formes du schéma drone du dashboard (cf. GCS/SCHEMA_DRONE_SPEC.md). La
# couleur n'est pas stockée : elle découle de Criticity, sinon un code pourrait
# allumer en rouge sur le schéma ce qu'il signale en ambre au panneau ESP32.
SCHEMATIC_ELEMENTS = {
    "CPU", "FC", "GPS", "MAG", "ESC", "LIDAR",
    "MOTOR_FRONT", "MOTOR_REAR", "BATTERY", "ANT_PDL", "ANT_SDL",
}


class DiagCode:
    # alarm="" : pas d'alarme dédiée -- le GCS retombe sur l'alarme générique
    # de catégorie (master_warning/master_caution, cf. ALARMS.csv et
    # server.js::MW_GROUP), le détail restant de toute façon affiché en
    # clair sur la page (event log). Sinon, AlarmId référençant une ligne de
    # ALARMS.csv (cf. tools/alarms_common.py) -- plusieurs codes peuvent
    # partager la même alarme (ex: SFS002/SFS003/SFS005 -> "airbus_caution").
    # panel="" : aucun voyant physique dédié (pas relayé sur le boîtier ESP32).
    # Sinon liste de labels mapingESP.csv séparés par virgule (ex: "GPS,EKF"),
    # cf. GCS/ESP32/mapingESP.csv — un code peut allumer plusieurs voyants
    # ("panne cause plusieurs voyants", décision explicite : politique dark
    # cockpit, chaque code décrit lui-même sa cascade plutôt qu'une règle
    # implicite basée sur la sévérité).
    # schematic="" : le code n'allume aucune forme du schéma drone. Sinon liste
    # de SCHEMATIC_ELEMENTS séparés par virgule (ex: "MOTOR_FRONT,MOTOR_REAR").
    def __init__(self, code, component_label, category_label, name, description, severity,
                alarm="", panel="", schematic=""):
        self.code = code
        self.component_label = component_label
        self.category_label = category_label
        self.name = name
        self.description = description
        self.severity = severity
        self.alarm = alarm
        self.panel = panel
        self.schematic = schematic

    @property
    def panel_labels(self):
        return [p.strip() for p in self.panel.split(",") if p.strip()]

    @property
    def schematic_labels(self):
        return [e.strip() for e in self.schematic.split(",") if e.strip()]

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
            "Alarm": self.alarm,
            "Panel": self.panel,
            "Schematic": self.schematic,
        }


class CodeValidationError(Exception):
    pass


def _label_from_full_name(table, full_name):
    for label, (_, full) in table.items():
        if full == full_name or label == full_name:
            return label
    return None


def parse_row(code_raw, component_raw, category_raw, name, description, severity_raw,
              alarm_raw=None, panel_raw=None, schematic_raw=None, *, seen=None,
              known_alarm_ids=None, where=""):
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

    alarm = (alarm_raw or "").strip().lower()
    if alarm and known_alarm_ids is not None and alarm not in known_alarm_ids:
        raise CodeValidationError(
            f"{where}Alarm '{alarm}' inconnu de ALARMS.csv "
            f"(ajoute-le d'abord dans l'onglet Alarmes)")

    schematic = (schematic_raw or "").strip().upper()
    unknown = [e for e in (x.strip() for x in schematic.split(",")) if e and e not in SCHEMATIC_ELEMENTS]
    if unknown:
        raise CodeValidationError(
            f"{where}Schematic {unknown} inconnu "
            f"(attendu parmi: {', '.join(sorted(SCHEMATIC_ELEMENTS))})")

    return DiagCode(
        code=code_raw,
        component_label=comp_label,
        category_label=cat_label,
        name=(name or "").strip(),
        description=(description or "").strip(),
        severity=SEVERITIES[severity_key],
        alarm=alarm,
        panel=(panel_raw or "").strip().upper(),
        schematic=schematic,
    )


def read_csv(path: Path, known_alarm_ids=None):
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
                row.get("Alarm"), row.get("Panel"), row.get("Schematic"),
                seen=seen, known_alarm_ids=known_alarm_ids, where=f"{path}:{lineno}: ",
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
