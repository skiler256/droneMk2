#!/usr/bin/env python3
# sitl/scenario.py
# ─────────────────────────────────────────────────────────────────────────────
# Format de scénario (cf. INTEGRATION_TEST.md §6), allégé pour la roadmap
# étape 2/3 : pas de `monde`/`modele_capteur` (blocs [3]/[4], pas encore
# construits — étape 4). `graine` gardé pour quand un modèle capteur bruité
# existera, ignoré pour l'instant si présent.
#
# id/description/mission/criteres : mêmes noms que §6, pas de renommage
# arbitraire pour "faire plus Pythonique" — un scénario doit se lire pareil
# qu'on regarde le YAML ou ce module.

from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class MissionStep:
    type: str
    params: dict = field(default_factory=dict)


@dataclass
class Criterion:
    metrique: str
    attendu: object


@dataclass
class Scenario:
    id: str
    description: str
    mission: list[MissionStep]
    criteres: list[Criterion]
    graine: int | None = None


def load(path: Path) -> Scenario:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))

    mission = []
    for step in raw.get("mission", []):
        step = dict(step)
        step_type = step.pop("type", None)
        if not step_type:
            raise ValueError(f"{path}: étape de mission sans 'type': {step}")
        mission.append(MissionStep(type=step_type, params=step))

    criteres = [Criterion(metrique=c["metrique"], attendu=c["attendu"])
               for c in raw.get("criteres", [])]

    return Scenario(
        id=raw["id"],
        description=(raw.get("description") or "").strip(),
        mission=mission,
        criteres=criteres,
        graine=raw.get("graine"),
    )
