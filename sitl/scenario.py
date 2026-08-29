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

# Racine du repo, pour resoudre `monde:` -- contrairement a `plan` (a cote du
# scenario), les mondes vivent dans world_engine/worlds/ (partages entre
# scenarios, et c'est aussi res://worlds/ cote Godot), donc resolus depuis
# la racine du repo et non depuis le dossier du scenario.
_REPO_ROOT = Path(__file__).resolve().parent.parent


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
    # Plan de mission (YAML, cf. tools/mission_compiler.py) que MissionControl
    # execute reellement -- distinct du scenario lui-meme (qui pilote
    # SITL_MASTER : ARM/TAKEOFF/pannes, jamais la trajectoire). Chemin resolu
    # relatif au dossier du scenario. None = pas de plan associe (le run
    # echouera au PRE_FLIGHT, MCA004).
    plan: Path | None = None
    # Monde 3D (JSON, cf. world_engine/World.gd), obligatoire -- meme
    # parallelisme que `plan` : un scenario = une combinaison plan de vol +
    # monde + sequence SITL_MASTER, jamais un des trois implicite (cf.
    # discussion roadmap etape 4). Resolu depuis la racine du repo (cf.
    # _REPO_ROOT), pas depuis le dossier du scenario -- les mondes sont
    # partages entre scenarios, pas dedies a un seul comme le plan.
    # Default None uniquement pour l'ordre des champs du dataclass -- load()
    # leve une erreur si absent, jamais None en pratique.
    monde: Path | None = None


def load(path: Path) -> Scenario:
    path = Path(path)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))

    mission = []
    for step in raw.get("mission", []):
        step = dict(step)
        step_type = step.pop("type", None)
        if not step_type:
            raise ValueError(f"{path}: étape de mission sans 'type': {step}")
        mission.append(MissionStep(type=step_type, params=step))

    criteres = [Criterion(metrique=c["metrique"], attendu=c["attendu"])
               for c in raw.get("criteres", [])]

    plan_raw = raw.get("plan")
    plan = (path.parent / plan_raw).resolve() if plan_raw else None

    monde_raw = raw.get("monde")
    if not monde_raw:
        raise ValueError(f"{path}: 'monde' obligatoire (cf. scenario.py: parallelisme plan/monde/sequence)")
    monde = (_REPO_ROOT / monde_raw).resolve()

    return Scenario(
        id=raw["id"],
        description=(raw.get("description") or "").strip(),
        mission=mission,
        criteres=criteres,
        graine=raw.get("graine"),
        plan=plan,
        monde=monde,
    )
