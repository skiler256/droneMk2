#!/usr/bin/env python3
# Démarre/arrête les process d'un run SITL : tools/dashboard/dashboard_server.py
# (SEUL lecteur du flux UDP de udp_bridge, cf. shm_observer.py — deux
# lecteurs concurrents se feraient la course sur le même port), ArduPilot
# SITL, `drone` (le vrai binaire, -DDRONE_USE_SITL_DRIVERS -DDRONE_USE_SIM_CLOCK),
# udp_bridge, world_engine/LidarServer.gd (bloc [3], géométrie) et
# sensor_bridge.py (bloc [4], GPS+LD06+TF Luna fusionnés, cf. roadmap étape 4).
#
# Usage typique : `with ProcessManager(cfg) as pm: ...` — l'arrêt (SIGTERM
# puis SIGKILL si besoin) est garanti même si le scénario lève une exception.

import os
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

sys.path.insert(0, str(REPO_ROOT / "tools"))
import missionplan_wire  # noqa: E402


def _x11_available() -> bool:
    """DISPLAY peut etre positionne sans serveur X joignable (sandbox headless,
    WSL sans WSLg) -- run_in_terminal_window.sh (utilise par sim_vehicle.py
    pour lancer arducopter) tente alors d'ouvrir un xterm et echoue
    silencieusement (exit 0), arducopter n'est jamais demarre. On verifie
    reellement plutot que de se fier a la seule presence de DISPLAY --
    connexion socket directe, pas de dependance a xdpyinfo (souvent absent,
    paquet x11-utils pas installe par defaut)."""
    display = os.environ.get("DISPLAY", "")
    if not display:
        return False
    # Formes courantes : ":0", ":0.0", "unix:0" -> socket unix locale.
    # "host:0" (X distant) -> TCP port 6000+N, pas teste ici (rare sous WSL).
    host, _, rest = display.partition(":")
    if host not in ("", "unix"):
        return False
    screen_str = rest.split(".")[0]
    if not screen_str.isdigit():
        return False
    socket_path = f"/tmp/.X11-unix/X{screen_str}"
    if not os.path.exists(socket_path):
        return False
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(1)
            s.connect(socket_path)
        return True
    except OSError:
        return False


@dataclass
class ProcessConfig:
    build_dir: Path = REPO_ROOT / "build_sitl"
    log_dir: Path = REPO_ROOT / "sitl" / "logs"
    # sim_vehicle.py : introuvable dans cet environnement de dev (pas
    # d'ArduPilot installe ici) -- laisser None desactive son lancement,
    # utile pour tester le reste de la chaine sans SITL reel.
    ardupilot_sim_vehicle: str | None = None
    ardupilot_frame: str = "ArduCopter"
    launch_sensor_bridge: bool = True
    sim_udp_endpoint: str = "127.0.0.1:14551"
    launch_dashboard: bool = True
    dashboard_port: int = 8766
    # Fenetres MAVProxy (console + carte) -- utile en mode interactif sur
    # poste de dev avec DISPLAY fonctionnel, sans objet pour un run scenario
    # automatise/headless (defaut off).
    ardupilot_gcs_windows: bool = False
    # Diagnostic : bypass MAVLink/SIM_STATE/LidarServer, sensor_bridge envoie
    # une trame GPS+LD06 fixe en continu -- isole UdpGpsDriver/UdpLidarDriver/
    # SensorFusion/shm de la reception SIM_STATE (cf. sensor_bridge.py
    # --fake-fixed).
    sensor_fake_fixed: bool = False
    # Plan de mission binaire deja compile (cf. tools/mission_compiler.py) --
    # None = pas de plan associe au run (mode interactif sans scenario),
    # procman genere alors un plan par defaut minimal (header par defaut,
    # aucun segment) lui-meme, en Python, sans sous-process.
    mission_plan_bin: Path | None = None
    # Binaire Godot headless (cf. world_engine/LidarServer.gd) -- meme
    # raisonnement que ardupilot_sim_vehicle : introuvable par defaut dans
    # cet environnement de dev, None desactive son lancement (permet de
    # tester le reste de la chaine sans moteur 3D reel).
    godot_headless_bin: str | None = None
    # Monde 3D (JSON, cf. world_engine/World.gd) -- None = pas de scenario
    # associe (mode interactif), procman utilise alors world_engine/worlds/
    # empty.json (meme raisonnement que mission_plan_bin par defaut).
    world_json: Path | None = None
    lidar_server_port: int = 15102
    # Viewer temps reel (world_engine/Viewer.gd), lance A LA MAIN par
    # l'utilisateur dans un autre terminal -- procman ne le gere jamais
    # (decouple du chemin critique, cf. discussion roadmap etape 4). None =
    # sensor_bridge.py ne diffuse pas de pose (aucun cout si personne n'ecoute
    # de toute facon, mais autant ne pas emettre pour rien par defaut).
    viewer_addr: str | None = None


class ProcessManager:
    def __init__(self, cfg: ProcessConfig):
        self.cfg = cfg
        self._procs: dict[str, subprocess.Popen] = {}
        cfg.log_dir.mkdir(parents=True, exist_ok=True)

    def __enter__(self):
        self.start_all()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop_all()
        return False

    def _spawn(self, name: str, cmd: list[str], cwd: Path | None = None, env: dict | None = None):
        log_path = self.cfg.log_dir / f"{name}.log"
        log_file = open(log_path, "wb")
        proc = subprocess.Popen(
            cmd, cwd=str(cwd) if cwd else None, env=env,
            stdout=log_file, stderr=subprocess.STDOUT,
        )
        self._procs[name] = proc
        print(f"procman: {name} demarre (pid={proc.pid}, log={log_path})")
        return proc

    def start_all(self):
        if self.cfg.launch_dashboard:
            self._spawn("dashboard", [
                "python3", str(REPO_ROOT / "tools" / "dashboard" / "dashboard_server.py"),
                "--no-browser", "--port", str(self.cfg.dashboard_port),
            ])
            time.sleep(0.3)  # doit lier son socket UDP avant que udp_bridge n'envoie
            print(f"procman: dashboard sur http://localhost:{self.cfg.dashboard_port}/sitl/prod")

        # LidarServer.gd (bloc [3]) n'a aucune dependance a ArduPilot (pure
        # geometrie, pas de MAVLink) -- lance en premier, avant sim_vehicle.
        # sensor_bridge.py (RaycastClient) retente ses requetes independamment
        # si le serveur n'est pas encore pret (pas besoin d'attente stricte
        # ici, contrairement au heartbeat MAVLink).
        if self.cfg.world_json:
            world_json = self.cfg.world_json
        else:
            world_json = REPO_ROOT / "world_engine" / "worlds" / "empty.json"
            print(f"procman: monde par defaut (vide) -> {world_json}")
        if self.cfg.godot_headless_bin:
            self._spawn("lidar_server", [
                self.cfg.godot_headless_bin, "--headless",
                "--path", str(REPO_ROOT / "world_engine"),
                "--script", "LidarServer.gd", "--",
                f"--world={world_json}", f"--port={self.cfg.lidar_server_port}",
            ])
            time.sleep(0.3)  # doit lier son socket UDP avant que sensor_bridge n'interroge

        if self.cfg.ardupilot_sim_vehicle:
            # PAS --no-mavproxy : sans MAVProxy, rien ne relaie le port serie
            # interne d'arducopter (--serial0, tcp:5760 par defaut) vers nos
            # ports UDP -- aucun trafic n'arrive jamais. --no-extra-ports
            # desactive le fan-out automatique de sim_vehicle.py, qui sous
            # WSL2 cible wsl2_host_ip() (la gateway, ex: 192.168.1.1) et pas
            # 127.0.0.1 -- ici tout est en loopback, on cible explicitement.
            # Si un serveur X est reellement joignable (WSLg, poste de dev),
            # on laisse DISPLAY tel quel -- sim_vehicle.py ouvre alors ses
            # fenetres normalement (xterm ArduCopter, console/map si demandes).
            # Sinon (sandbox headless, DISPLAY pointant dans le vide) on le
            # coupe pour forcer le repli headless de run_in_terminal_window.sh
            # (log dans /tmp/ArduCopter.log) -- sans ca, arducopter n'est
            # jamais demarre (xterm echoue silencieusement, exit 0).
            sim_env = dict(os.environ)
            if not _x11_available():
                sim_env.pop("DISPLAY", None)
            sim_cmd = [
                self.cfg.ardupilot_sim_vehicle, "-v", self.cfg.ardupilot_frame,
                "--no-extra-ports",
                "--out", "127.0.0.1:14550",
                "--out", self.cfg.sim_udp_endpoint,
                # RNGFND1_TYPE=10 (MAVLink) -- sans ca, ArduPilot ignore en
                # silence les DISTANCE_SENSOR envoyes par sensor_bridge.py
                # (TF Luna), aucun effet sur l'EKF/altitude simulee.
                "--add-param-file", str(REPO_ROOT / "sitl" / "rangefinder.parm"),
            ]
            if self.cfg.ardupilot_gcs_windows:
                sim_cmd += ["--console", "--map"]
            self._spawn("sim_vehicle", sim_cmd, env=sim_env)
            time.sleep(2)  # laisse SITL demarrer avant de brancher le reste

        # MissionControl refuse l'arm sans plan de mission lisible
        # (runPreflightChecks() -> readFile(DRONE_MISSION_PLAN_PATH), sinon
        # /etc/drone/missionplan.bin qui n'existe pas en dev). Le scenario
        # fournit le sien (cfg.mission_plan_bin, compile par run_scenario.py
        # via tools/mission_compiler.py) -- sans scenario (mode interactif),
        # on ecrit ici un plan minimal par defaut (aucun segment) directement
        # en Python via tools/missionplan_wire.py, meme format binaire que
        # MISSIONPLAN_CONV::serialize() cote C++ (valide par round-trip).
        if self.cfg.mission_plan_bin:
            missionplan_bin = self.cfg.mission_plan_bin
        else:
            missionplan_bin = self.cfg.log_dir / "missionplan.bin"
            missionplan_bin.write_bytes(missionplan_wire.encode({}, []))
            print(f"procman: plan de mission par defaut (vide) -> {missionplan_bin}")
        drone_env = dict(os.environ)
        drone_env["DRONE_MISSION_PLAN_PATH"] = str(missionplan_bin)
        # Le modele de frame ArduPilot SITL simule un pack 3S generique
        # (maxVoltage code en dur a 12.6V dans SIM_Frame.h -- SIM_BATT_VOLTAGE
        # y est clippe, --add-param-file/--defaults ne peut pas le depasser),
        # incompatible avec kBattLowVoltage/kBattCriticalVoltage calibres sur
        # un vrai pack 4S (14.0V/12.5V). Seuils SITL abaisses sous 12.6V pour
        # ne pas declencher ces codes artificiellement (cf. MavlinkInterface.cpp).
        drone_env["DRONE_BATT_LOW_V"] = "11.0"
        drone_env["DRONE_BATT_CRITICAL_V"] = "9.5"
        # ArduPilot SITL standard ne simule pas de telemetrie ESC/DShot --
        # ESC_STATUS n'arrive jamais, rpmFault se declenchait immediatement
        # a l'arm et desarmait le drone sans demande GCS (cf. MavlinkInterface.cpp
        # kSkipRpmCheck). Comportement de vol reel inchange.
        drone_env["DRONE_SKIP_RPM_CHECK"] = "1"
        self._spawn("drone", [str(self.cfg.build_dir / "drone")], env=drone_env)
        time.sleep(0.5)

        self._spawn("udp_bridge", [str(self.cfg.build_dir / "udp_bridge")])
        time.sleep(0.3)

        if self.cfg.launch_sensor_bridge:
            venv_python = REPO_ROOT / "sitl" / ".venv" / "bin" / "python3"
            sensor_bridge_cmd = [
                str(venv_python), "-u", str(REPO_ROOT / "sitl" / "sensor_bridge.py"),
                "--sim-udp", self.cfg.sim_udp_endpoint,
                "--lidar-server", f"127.0.0.1:{self.cfg.lidar_server_port}",
            ]
            if self.cfg.sensor_fake_fixed:
                sensor_bridge_cmd.append("--fake-fixed")
            if self.cfg.viewer_addr:
                sensor_bridge_cmd += ["--viewer", self.cfg.viewer_addr]
            self._spawn("sensor_bridge", sensor_bridge_cmd)

    def stop_all(self):
        # Ordre inverse du demarrage -- sensor_bridge/udp_bridge (observateurs)
        # avant drone/sim_vehicle/lidar_server (le systeme lui-meme et le
        # moteur 3D).
        for name in reversed(list(self._procs)):
            proc = self._procs[name]
            if proc.poll() is not None:
                continue
            proc.send_signal(signal.SIGTERM)

        for name, proc in self._procs.items():
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print(f"procman: {name} ne repond pas a SIGTERM, SIGKILL")
                proc.kill()
                proc.wait(timeout=5)
            print(f"procman: {name} arrete (code={proc.returncode})")

    def is_alive(self, name: str) -> bool:
        proc = self._procs.get(name)
        return proc is not None and proc.poll() is None

    def kill(self, name: str, sig=signal.SIGKILL):
        """Panne systeme deliberee (cf. INTEGRATION_TEST.md §7.2) -- pas un
        arret propre, ne retire pas le process du suivi (stop_all() verra
        qu'il est deja mort et l'ignorera)."""
        proc = self._procs.get(name)
        if proc and proc.poll() is None:
            proc.send_signal(sig)
