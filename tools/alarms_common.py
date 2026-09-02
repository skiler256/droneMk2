#!/usr/bin/env python3
# Logique partagée pour ALARMS.csv (définitions d'alarme) et
# MISSION_ALARMS.csv (alarme déclenchée par une transition MISSION_STATE) --
# même principe que codes_common.py/telemetry_common.py : une seule source de
# vérité pour le format/la validation, utilisée par codes_editor.py et (côté
# navigateur) GCS/public/js/alarm.js via /alarms.csv, /mission_alarms.csv.
#
# ALARMS.csv : AlarmId;Label;SoundFile;Type;Priority;PeriodMs;Class
#   AlarmId    identifiant stable (snake_case), référencé par INDEX.csv::Alarm
#              et MISSION_ALARMS.csv::AlarmId.
#   SoundFile  base de fichier dans GCS/public/sounds/ (sans extension) --
#              <SoundFile>.mp3/.wav/.mp4, cf. alarm.js::makeAudio.
#   Type       Loop (rejoue tant que la source reste active) ou Oneshot
#              (joue une fois au déclenchement, ne se répète pas seul).
#   Priority   entier, plus haut = plus prioritaire. Un seul canal audio :
#              la plus prioritaire active joue, préempte immédiatement une
#              moins prioritaire en cours (jamais coincée derrière une file).
#   PeriodMs   Loop uniquement. 0 = boucle native continue (<audio loop>).
#              >0 = rejoué toutes les PeriodMs (clip fini avant repli).
#              Ignoré pour Oneshot.
#   Class      MW ou MC -- quel bouton (MW_ACK/MC_ACK du boîtier ESP32)
#              acquitte cette alarme. Indépendant du calcul MW/MC des
#              voyants (server.js) : ici c'est juste le routage d'ack son.

import csv
import re
import sys
from pathlib import Path

ALARM_TYPES = {"loop": "Loop", "oneshot": "Oneshot"}
ALARM_CLASSES = {"mw": "MW", "mc": "MC"}

ALARM_ID_RE = re.compile(r"^[a-z][a-z0-9_]*$")
ALARMS_CSV_FIELDS = ["AlarmId", "Label", "SoundFile", "Type", "Priority", "PeriodMs", "Class"]
MISSION_ALARMS_CSV_FIELDS = ["State", "AlarmId"]

# Cf. GCS/public/js/link.js::MISSION_STATE_NAMES -- ordre non significatif ici
# (juste une liste de valeurs valides), contrairement à l'enum C++ réel.
MISSION_STATES = [
    "IDLE", "PRE_FLIGHT", "ARMED", "MISSION", "MANUAL", "HOLD", "LANDING", "EMERGENCY",
]


class AlarmValidationError(Exception):
    pass


class Alarm:
    def __init__(self, alarm_id, label, sound_file, type_, priority, period_ms, klass):
        self.alarm_id = alarm_id
        self.label = label
        self.sound_file = sound_file
        self.type = type_
        self.priority = priority
        self.period_ms = period_ms
        self.klass = klass

    def to_row(self):
        return {
            "AlarmId": self.alarm_id,
            "Label": self.label,
            "SoundFile": self.sound_file,
            "Type": self.type,
            "Priority": self.priority,
            "PeriodMs": self.period_ms,
            "Class": self.klass,
        }


class MissionAlarm:
    def __init__(self, state, alarm_id):
        self.state = state
        self.alarm_id = alarm_id

    def to_row(self):
        return {"State": self.state, "AlarmId": self.alarm_id}


def parse_alarm_row(alarm_id, label, sound_file, type_raw, priority_raw, period_ms_raw,
                    class_raw, *, seen=None, where=""):
    alarm_id = (alarm_id or "").strip().lower()
    if not ALARM_ID_RE.match(alarm_id):
        raise AlarmValidationError(
            f"{where}AlarmId '{alarm_id}' invalide (attendu snake_case, ex: fire_bell)")

    if seen is not None:
        if alarm_id in seen:
            raise AlarmValidationError(f"{where}AlarmId '{alarm_id}' dupliqué")
        seen.add(alarm_id)

    type_key = (type_raw or "").strip().lower()
    if type_key not in ALARM_TYPES:
        raise AlarmValidationError(
            f"{where}Type '{type_raw}' inconnu (attendu {', '.join(ALARM_TYPES.values())})")

    class_key = (class_raw or "").strip().lower()
    if class_key not in ALARM_CLASSES:
        raise AlarmValidationError(
            f"{where}Class '{class_raw}' inconnue (attendu {', '.join(ALARM_CLASSES.values())})")

    # priority_raw/period_ms_raw : str depuis le CSV ou le formulaire HTML,
    # mais peuvent arriver en int si l'appelant est une API JSON (ex: script
    # d'injection) -- accepter les deux plutôt que de planter sur .strip().
    try:
        priority = int(priority_raw)
    except (TypeError, ValueError):
        raise AlarmValidationError(f"{where}Priority '{priority_raw}' doit être un entier")
    if not (0 <= priority <= 999):
        raise AlarmValidationError(f"{where}Priority hors plage (0-999)")

    try:
        period_ms = int(period_ms_raw) if str(period_ms_raw or "").strip() else 0
    except (TypeError, ValueError):
        raise AlarmValidationError(f"{where}PeriodMs '{period_ms_raw}' doit être un entier")
    if period_ms < 0:
        raise AlarmValidationError(f"{where}PeriodMs ne peut pas être négatif")

    return Alarm(
        alarm_id=alarm_id,
        label=(label or "").strip(),
        sound_file=(sound_file or "").strip(),
        type_=ALARM_TYPES[type_key],
        priority=priority,
        period_ms=period_ms,
        klass=ALARM_CLASSES[class_key],
    )


def parse_mission_alarm_row(state_raw, alarm_id_raw, *, known_alarm_ids=None, seen_states=None,
                            where=""):
    state = (state_raw or "").strip().upper()
    if state not in MISSION_STATES:
        raise AlarmValidationError(
            f"{where}State '{state}' inconnu (attendu {', '.join(MISSION_STATES)})")
    if seen_states is not None:
        if state in seen_states:
            raise AlarmValidationError(f"{where}State '{state}' dupliqué (un seul par état)")
        seen_states.add(state)

    alarm_id = (alarm_id_raw or "").strip().lower()
    if known_alarm_ids is not None and alarm_id not in known_alarm_ids:
        raise AlarmValidationError(f"{where}AlarmId '{alarm_id}' inconnu de ALARMS.csv")

    return MissionAlarm(state=state, alarm_id=alarm_id)


def _read_text(path: Path) -> str:
    raw = path.read_bytes()
    for enc in ("utf-8", "cp850", "cp1252"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    sys.exit(f"erreur: impossible de decoder {path} (essaye utf-8/cp1252)")


def read_alarms_csv(path: Path):
    if not path.exists():
        return []
    reader = csv.DictReader(_read_text(path).splitlines(), delimiter=";")
    if reader.fieldnames:
        reader.fieldnames = [f.strip() for f in reader.fieldnames]
    alarms, seen = [], set()
    for lineno, row in enumerate(reader, start=2):
        alarm_id = (row.get("AlarmId") or "").strip()
        if not alarm_id:
            continue
        try:
            alarms.append(parse_alarm_row(
                alarm_id, row.get("Label"), row.get("SoundFile"), row.get("Type"),
                row.get("Priority"), row.get("PeriodMs"), row.get("Class"),
                seen=seen, where=f"{path}:{lineno}: ",
            ))
        except AlarmValidationError as e:
            sys.exit(f"erreur: {e}")
    return alarms


def write_alarms_csv(path: Path, alarms):
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=ALARMS_CSV_FIELDS, delimiter=";")
        writer.writeheader()
        for a in sorted(alarms, key=lambda a: a.alarm_id):
            writer.writerow(a.to_row())


def read_mission_alarms_csv(path: Path, known_alarm_ids=None):
    if not path.exists():
        return []
    reader = csv.DictReader(_read_text(path).splitlines(), delimiter=";")
    if reader.fieldnames:
        reader.fieldnames = [f.strip() for f in reader.fieldnames]
    rows, seen_states = [], set()
    for lineno, row in enumerate(reader, start=2):
        state = (row.get("State") or "").strip()
        if not state:
            continue
        try:
            rows.append(parse_mission_alarm_row(
                state, row.get("AlarmId"), known_alarm_ids=known_alarm_ids,
                seen_states=seen_states, where=f"{path}:{lineno}: ",
            ))
        except AlarmValidationError as e:
            sys.exit(f"erreur: {e}")
    return rows


def write_mission_alarms_csv(path: Path, mission_alarms):
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=MISSION_ALARMS_CSV_FIELDS, delimiter=";")
        writer.writeheader()
        for m in sorted(mission_alarms, key=lambda m: MISSION_STATES.index(m.state)):
            writer.writerow(m.to_row())
