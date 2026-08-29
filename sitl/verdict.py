#!/usr/bin/env python3
# sitl/verdict.py
# ─────────────────────────────────────────────────────────────────────────────
# Calcul du verdict pass/fail (cf. INTEGRATION_TEST.md §5/§9) à partir d'une
# RunTrace déjà collectée -- fonction pure, testable sans lancer quoi que ce
# soit (même esprit que MC_CONV::step() côté C++ : la logique de décision ne
# doit pas dépendre de comment la donnée a été récoltée).

import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from codes_common import read_csv as read_codes_csv  # noqa: E402

INDEX_CSV = Path(__file__).resolve().parent.parent / "INDEX.csv"


def _severity_by_name() -> dict[str, str]:
    return {c.name: c.severity for c in read_codes_csv(INDEX_CSV)}


@dataclass
class RunTrace:
    # États MISSION_STATE vus pendant le run, dans l'ordre d'arrivée
    # (transitions consécutives identiques déjà dédupliquées par l'appelant).
    mission_states_seen: list[str] = field(default_factory=list)
    # Noms de code (cf. Entry::name côté C++, ex: "GPS_lost") vus au moins
    # une fois, tous composants confondus.
    codes_seen: set[str] = field(default_factory=set)
    # nom de process (cf. procman.ProcessManager) -> vivant à la fin du run.
    processes_alive_at_end: dict[str, bool] = field(default_factory=dict)
    # Distances brutes (mètres) des points lidar non-nuls observés pendant
    # le run (cf. PROD_LIDAR, `pt.N.range_mm` — dédupliquées au dixième de
    # mètre près, sinon liste non bornée sur un run long).
    lidar_hit_distances_m: set[float] = field(default_factory=set)


@dataclass
class CriterionResult:
    metrique: str
    attendu: object
    obtenu: object
    ok: bool


@dataclass
class Verdict:
    scenario_id: str
    results: list[CriterionResult]

    @property
    def passed(self) -> bool:
        return all(r.ok for r in self.results)


def _is_subsequence(needle: list, haystack: list) -> bool:
    it = iter(haystack)
    return all(x in it for x in needle)


def evaluate(scenario, trace: RunTrace) -> Verdict:
    results = []
    severity_by_name = None  # chargée à la demande, un seul critère en a besoin ici

    for crit in scenario.criteres:
        if crit.metrique == "mission_state_sequence":
            obtenu = _is_subsequence(crit.attendu, trace.mission_states_seen)
            results.append(CriterionResult(crit.metrique, crit.attendu,
                                           trace.mission_states_seen, obtenu))

        elif crit.metrique == "aucun_code_emergency":
            if severity_by_name is None:
                severity_by_name = _severity_by_name()
            emergency_seen = [c for c in trace.codes_seen
                              if severity_by_name.get(c) == "Emergency"]
            ok = (len(emergency_seen) == 0) == bool(crit.attendu)
            results.append(CriterionResult(crit.metrique, crit.attendu,
                                           emergency_seen, ok))

        elif crit.metrique == "processus_vivants_a_la_fin":
            manquants = [name for name in crit.attendu
                        if not trace.processes_alive_at_end.get(name, False)]
            results.append(CriterionResult(crit.metrique, crit.attendu,
                                           manquants, len(manquants) == 0))

        elif crit.metrique == "lidar_hit_range_m":
            lo, hi = crit.attendu
            hits = sorted(n for n in trace.lidar_hit_distances_m if lo <= n <= hi)
            results.append(CriterionResult(crit.metrique, crit.attendu,
                                           hits, len(hits) > 0))

        else:
            results.append(CriterionResult(crit.metrique, crit.attendu,
                                           None, False))  # métrique inconnue = échec explicite, pas ignoré

    return Verdict(scenario_id=scenario.id, results=results)


def print_report(verdict: Verdict):
    print(f"=== Verdict : {verdict.scenario_id} — "
         f"{'PASS' if verdict.passed else 'FAIL'} ===")
    for r in verdict.results:
        status = "OK  " if r.ok else "FAIL"
        print(f"  [{status}] {r.metrique}: attendu={r.attendu!r} obtenu={r.obtenu!r}")
