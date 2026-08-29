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
# udp_bridge/sim_vehicle/lidar_server/sensor_bridge) et attend Ctrl+C, sans
# jouer de mission ni calculer de verdict -- mode "interactif", pour piloter
# à la main via le dashboard (cf. sitl/run.sh, sans nom de scénario en
# argument).

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import mission_compiler  # noqa: E402
import mission_driver  # noqa: E402
import procman  # noqa: E402
import scenario as scenario_mod  # noqa: E402
import shm_observer  # noqa: E402
import verdict as verdict_mod  # noqa: E402

# ArduCopter SITL a froid (calibration, init EKF3, premier heartbeat) peut
# prendre 15-30s -- 10s etait trop court, mesure sur run reel (echec avant
# meme reception du premier heartbeat MAVProxy).
READY_TIMEOUT_S = 45.0
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

    lidar = observer.get("LIDAR") or {}
    count = int(lidar.get("count", 0))
    for i in range(count):
        try:
            range_mm = float(lidar[f"pt.{i}.range_mm"])
        except (KeyError, ValueError):
            continue
        trace.lidar_hit_distances_m.add(round(range_mm / 1000.0, 1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario_path", type=Path, nargs="?", default=None,
                        help="absent = mode interactif (démarre les process, attend Ctrl+C, pas de verdict)")
    parser.add_argument("--sim-vehicle", default=os.environ.get("ARDUPILOT_SIM_VEHICLE"),
                        help="chemin vers sim_vehicle.py (défaut: $ARDUPILOT_SIM_VEHICLE, "
                            "absent des deux = pas de vrai ArduPilot SITL)")
    parser.add_argument("--build-dir", type=Path, default=procman.REPO_ROOT / "build_sitl")
    parser.add_argument("--gcs-windows", action="store_true",
                        help="ouvre les fenetres MAVProxy (console + carte) -- "
                            "besoin d'un DISPLAY fonctionnel (WSLg, poste de dev)")
    parser.add_argument("--sensor-fake-fixed", action="store_true",
                        help="diagnostic : sensor_bridge bypass MAVLink/SIM_STATE/LidarServer, "
                            "envoie une trame GPS+LD06 fixe en continu (cf. sensor_bridge.py --fake-fixed)")
    parser.add_argument("--godot-headless", default=os.environ.get("GODOT_HEADLESS_BIN"),
                        help="chemin vers le binaire Godot headless (défaut: $GODOT_HEADLESS_BIN, "
                            "absent des deux = pas de moteur 3D/LidarServer)")
    parser.add_argument("--viewer", default=None,
                        help="host:port de world_engine/Viewer.gd déjà lancé à la main dans un "
                            "autre terminal (optionnel, debug -- cf. sitl/SITL.md §9)")
    parser.add_argument("--godot-editor", default=os.environ.get("GODOT_EDITOR_BIN"),
                        help="chemin vers le binaire Godot x11 (éditeur, avec fenêtre) -- lance "
                            "automatiquement le viewer avec ce run si fourni (défaut: "
                            "$GODOT_EDITOR_BIN). Ignoré si --viewer est déjà fourni (viewer "
                            "supposé déjà lancé à la main)")
    parser.add_argument("--pose-port", type=int, default=15103,
                        help="port UDP du viewer lancé via --godot-editor (LidarServer utilise 15102)")
    args = parser.parse_args()

    viewer_proc = None
    viewer_addr = args.viewer

    try:
        if args.scenario_path is None:
            if args.godot_editor and not viewer_addr:
                world_for_viewer = procman.REPO_ROOT / "world_engine" / "worlds" / "empty.json"
                viewer_proc = _launch_viewer(args.godot_editor, world_for_viewer, args.pose_port)
                viewer_addr = f"127.0.0.1:{args.pose_port}"

            cfg = procman.ProcessConfig(build_dir=args.build_dir, ardupilot_sim_vehicle=args.sim_vehicle,
                                        ardupilot_gcs_windows=args.gcs_windows,
                                        sensor_fake_fixed=args.sensor_fake_fixed,
                                        godot_headless_bin=args.godot_headless,
                                        viewer_addr=viewer_addr)
            _run_interactive(cfg)
            return

        scn = scenario_mod.load(args.scenario_path)
        print(f"Scénario '{scn.id}': {scn.description}")
        trace = verdict_mod.RunTrace()

        # Plan de mission propre au scenario (cf. sitl/scenario.py, champ `plan`)
        # -- distinct du scenario lui-meme (ARM/TAKEOFF/pannes, jamais la
        # trajectoire). Pas de plan associe -> procman.py ecrit un plan par
        # defaut sans segment (cf. ProcessConfig.mission_plan_bin) -- Mission
        # se termine instantanement (TMiSequencer, plus rien a executer), utile
        # seulement pour tester jusqu'a l'arm, pas au-dela.
        mission_plan_bin = None
        if scn.plan:
            mission_plan_bin = procman.REPO_ROOT / "sitl" / "logs" / "missionplan.bin"
            mission_plan_bin.parent.mkdir(parents=True, exist_ok=True)
            mission_plan_bin.write_bytes(mission_compiler.compile_plan(scn.plan))
            print(f"Plan de mission '{scn.plan}' compile -> {mission_plan_bin}")

        if args.godot_editor and not viewer_addr:
            viewer_proc = _launch_viewer(args.godot_editor, scn.monde, args.pose_port)
            viewer_addr = f"127.0.0.1:{args.pose_port}"

        cfg = procman.ProcessConfig(build_dir=args.build_dir, ardupilot_sim_vehicle=args.sim_vehicle,
                                    ardupilot_gcs_windows=args.gcs_windows,
                                    sensor_fake_fixed=args.sensor_fake_fixed,
                                    godot_headless_bin=args.godot_headless,
                                    mission_plan_bin=mission_plan_bin,
                                    world_json=scn.monde,
                                    viewer_addr=viewer_addr)

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

                # cf. run_step() : le land du scenario n'attend que la
                # TRANSITION vers mission_state=="Landing" (pas la fin reelle
                # de l'atterrissage) -- sortir d'ici tout de suite coupait la
                # simulation en pleine descente (bug constate en session).
                # Le verdict et le compte des process vivants sont calcules
                # maintenant (etat juste apres la sequence scriptee), mais
                # plus aucune coupure automatique ensuite -- SEUL un Ctrl+C
                # explicite arrete la simulation desormais.
                for name in ("drone", "udp_bridge", "sim_vehicle", "sensor_bridge", "lidar_server"):
                    trace.processes_alive_at_end[name] = pm.is_alive(name)
                result = verdict_mod.evaluate(scn, trace)
                verdict_mod.print_report(result)

                print("Scénario terminé -- la simulation continue de tourner (ex: observer "
                     "l'atterrissage jusqu'au bout) -- Ctrl+C pour arrêter proprement.")
                try:
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    print("\nArrêt demandé, nettoyage...")

            finally:
                driver.stop_heartbeat()
                observer.stop()

        sys.exit(0 if result.passed else 1)

    finally:
        # Viewer jamais gere par procman (decision actee, cf. _launch_viewer) --
        # on le termine nous-memes ici, symetrique du try/finally de ProcessManager.
        if viewer_proc is not None and viewer_proc.poll() is None:
            viewer_proc.terminate()
            try:
                viewer_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                viewer_proc.kill()
                viewer_proc.wait(timeout=5)


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


def _launch_viewer(godot_editor: str, world_path, pose_port: int) -> subprocess.Popen:
    """Lance world_engine/Viewer.gd (bloc [3], optionnel/debug) avec fenetre --
    process a part, PAS gere par procman.ProcessManager (decision actee :
    jamais de rendu sur le chemin critique du raycast/du reste du systeme,
    cf. sitl/SITL.md §9). Termine explicitement par l'appelant (finally)."""
    return subprocess.Popen([
        godot_editor, "--path", str(procman.REPO_ROOT / "world_engine"),
        "res://Viewer.tscn", "--",
        f"--world={world_path}", f"--pose-port={pose_port}",
    ])


def _wait_ready(observer: shm_observer.ShmObserver) -> bool:
    deadline = time.monotonic() + READY_TIMEOUT_S
    while time.monotonic() < deadline:
        if observer.get("MC"):
            return True
        time.sleep(POLL_S)
    return False


if __name__ == "__main__":
    main()
