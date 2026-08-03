#!/usr/bin/env python3
# sitl/run_scenario.py
# ─────────────────────────────────────────────────────────────────────────────
# Point d'entrée CLI : charge un scénario, lance les process (procman),
# joue la mission (mission_driver) en observant l'état système
# (shm_observer), calcule un verdict (verdict.py).
#
# Types d'étape de mission supportés pour l'instant (étape 2/3, "SITL nu") :
#   arm, takeoff (alt), wait (duration), land
# Pas encore : hold/resume/answer (attendent un vrai scénario qui les
# exerce), segments intermédiaires (attendent Nav validé sur SITL nu,
# roadmap étape 3).
#
# Usage : sitl/.venv/bin/python3 sitl/run_scenario.py sitl/scenarios/001_arm_takeoff_wait_land.yaml
#         --sim-vehicle /chemin/vers/sim_vehicle.py (ou variable d'env
#         ARDUPILOT_SIM_VEHICLE, pour ne pas le retaper à chaque fois)
#
# Sans argument de scénario : démarre juste les process (dashboard/drone/
# udp_bridge/sim_vehicle/gps_bridge) et attend Ctrl+C, sans jouer de mission
# ni calculer de verdict -- mode "interactif", pour piloter à la main via le
# dashboard (cf. sitl/run.sh, sans nom de scénario en argument).

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mission_driver  # noqa: E402
import procman  # noqa: E402
import scenario as scenario_mod  # noqa: E402
import shm_observer  # noqa: E402
import verdict as verdict_mod  # noqa: E402

READY_TIMEOUT_S = 10.0
STEP_TIMEOUT_S = 15.0
POLL_S = 0.2


def run_step(step: scenario_mod.MissionStep, driver: mission_driver.MissionDriver,
            observer: shm_observer.ShmObserver, trace: verdict_mod.RunTrace):
    print(f"-- étape: {step.type} {step.params}")

    if step.type == "arm":
        driver.arm()
        ok = observer.wait_for_mission_state("Armed", STEP_TIMEOUT_S)
    elif step.type == "takeoff":
        driver.takeoff(float(step.params.get("alt", 10)))
        ok = observer.wait_for_mission_state("Mission", STEP_TIMEOUT_S)
    elif step.type == "wait":
        duration = float(step.params.get("duration", 1))
        deadline = time.monotonic() + duration
        ok = True
        while time.monotonic() < deadline:
            _record_observations(observer, trace)
            time.sleep(POLL_S)
    elif step.type == "land":
        driver.land()
        ok = observer.wait_for_mission_state("Landing", STEP_TIMEOUT_S)
    else:
        raise ValueError(f"type d'étape inconnu: {step.type!r}")

    if not ok:
        print(f"   ATTENTION: étape '{step.type}' n'a pas atteint l'état attendu "
             f"dans les {STEP_TIMEOUT_S}s (mission_state={observer.mission_state()!r})")


def _record_observations(observer: shm_observer.ShmObserver, trace: verdict_mod.RunTrace):
    state = observer.mission_state()
    if state and (not trace.mission_states_seen or trace.mission_states_seen[-1] != state):
        trace.mission_states_seen.append(state)
    for codes in observer.active_codes().values():
        trace.codes_seen.update(codes)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario_path", type=Path, nargs="?", default=None,
                        help="absent = mode interactif (démarre les process, attend Ctrl+C, pas de verdict)")
    parser.add_argument("--sim-vehicle", default=os.environ.get("ARDUPILOT_SIM_VEHICLE"),
                        help="chemin vers sim_vehicle.py (défaut: $ARDUPILOT_SIM_VEHICLE, "
                            "absent des deux = pas de vrai ArduPilot SITL)")
    parser.add_argument("--build-dir", type=Path, default=procman.REPO_ROOT / "build_sitl")
    args = parser.parse_args()

    cfg = procman.ProcessConfig(build_dir=args.build_dir, ardupilot_sim_vehicle=args.sim_vehicle)

    if args.scenario_path is None:
        _run_interactive(cfg)
        return

    scn = scenario_mod.load(args.scenario_path)
    print(f"Scénario '{scn.id}': {scn.description}")
    trace = verdict_mod.RunTrace()

    with procman.ProcessManager(cfg) as pm:
        observer = shm_observer.ShmObserver().start()
        driver = mission_driver.MissionDriver()
        driver.start_heartbeat()

        try:
            print("Attente du système (PROD_MC)...")
            if not observer.get("MC") and not _wait_ready(observer):
                print("ERREUR: système jamais vu sur la shm (drone/udp_bridge démarrés ?)")
                sys.exit(1)

            for step in scn.mission:
                run_step(step, driver, observer, trace)
                _record_observations(observer, trace)

        finally:
            driver.stop_heartbeat()
            observer.stop()
            for name in ("drone", "udp_bridge", "sim_vehicle", "gps_bridge"):
                trace.processes_alive_at_end[name] = pm.is_alive(name)

    result = verdict_mod.evaluate(scn, trace)
    verdict_mod.print_report(result)
    sys.exit(0 if result.passed else 1)


def _run_interactive(cfg: procman.ProcessConfig):
    with procman.ProcessManager(cfg) as pm:
        observer = shm_observer.ShmObserver().start()
        driver = mission_driver.MissionDriver()
        driver.start_heartbeat()
        print(f"Mode interactif : process démarrés, dashboard sur "
             f"http://localhost:{cfg.dashboard_port}/sitl/prod")
        print("Ctrl+C pour arrêter proprement.")
        try:
            while True:
                if not observer.get("MC") and not pm.is_alive("drone"):
                    print("ATTENTION: drone ne répond plus (crashé au démarrage ?)")
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            driver.stop_heartbeat()
            observer.stop()


def _wait_ready(observer: shm_observer.ShmObserver) -> bool:
    deadline = time.monotonic() + READY_TIMEOUT_S
    while time.monotonic() < deadline:
        if observer.get("MC"):
            return True
        time.sleep(POLL_S)
    return False


if __name__ == "__main__":
    main()
