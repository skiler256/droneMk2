#!/usr/bin/env python3
# sitl/procman.py
# ─────────────────────────────────────────────────────────────────────────────
# Démarre/arrête les process d'un run SITL : tools/dashboard/dashboard_server.py
# (SEUL lecteur du flux UDP de udp_bridge, cf. shm_observer.py — deux
# lecteurs concurrents se feraient la course sur le même port), ArduPilot
# SITL, `drone` (le vrai binaire, -DDRONE_USE_SITL_DRIVERS -DDRONE_USE_SIM_CLOCK),
# udp_bridge, gps_bridge.py (Phase B). Pas de moteur 3D ici (roadmap étape 4,
# pas construit).
#
# Usage typique : `with ProcessManager(cfg) as pm: ...` — l'arrêt (SIGTERM
# puis SIGKILL si besoin) est garanti même si le scénario lève une exception.

import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


@dataclass
class ProcessConfig:
    build_dir: Path = REPO_ROOT / "build_sitl"
    log_dir: Path = REPO_ROOT / "sitl" / "logs"
    # sim_vehicle.py : introuvable dans cet environnement de dev (pas
    # d'ArduPilot installe ici) -- laisser None desactive son lancement,
    # utile pour tester le reste de la chaine sans SITL reel.
    ardupilot_sim_vehicle: str | None = None
    ardupilot_frame: str = "ArduCopter"
    launch_gps_bridge: bool = True
    sim_udp_endpoint: str = "127.0.0.1:14551"
    launch_dashboard: bool = True
    dashboard_port: int = 8766


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

    def _spawn(self, name: str, cmd: list[str], cwd: Path | None = None):
        log_path = self.cfg.log_dir / f"{name}.log"
        log_file = open(log_path, "wb")
        proc = subprocess.Popen(
            cmd, cwd=str(cwd) if cwd else None,
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

        if self.cfg.ardupilot_sim_vehicle:
            self._spawn("sim_vehicle", [
                self.cfg.ardupilot_sim_vehicle, "-v", self.cfg.ardupilot_frame,
                "--no-mavproxy",
            ])
            time.sleep(2)  # laisse SITL demarrer avant de brancher le reste

        self._spawn("drone", [str(self.cfg.build_dir / "drone")])
        time.sleep(0.5)

        self._spawn("udp_bridge", [str(self.cfg.build_dir / "udp_bridge")])
        time.sleep(0.3)

        if self.cfg.launch_gps_bridge:
            venv_python = REPO_ROOT / "sitl" / ".venv" / "bin" / "python3"
            self._spawn("gps_bridge", [
                str(venv_python), "-u", str(REPO_ROOT / "sitl" / "gps_bridge.py"),
                "--sim-udp", self.cfg.sim_udp_endpoint,
            ])

    def stop_all(self):
        # Ordre inverse du demarrage -- gps_bridge/udp_bridge (observateurs)
        # avant drone/sim_vehicle (le systeme lui-meme).
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
