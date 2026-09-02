#!/usr/bin/env python3
# Observabilité de la shm de prod pour l'orchestrateur — PAS de mmap/struct
# direct (cf. discussion : 7 structs C++ imbriquées, risque réel de
# désalignement silencieux). PAS non plus d'écoute UDP directe du flux de
# udp_bridge (kStatusPort=15001) : ce port n'a qu'UN seul lecteur possible
# (sendto() vers une cible fixe, pas de broadcast/multicast) — deux binds
# concurrents (dashboard_server.py + un observer maison) se font la course
# et le second échoue (`Address already in use`, bug trouvé en testant
# sitl/run.sh --dashboard).
#
# À la place : dashboard_server.py est le SEUL lecteur UDP du pont (démarré
# par procman.py, systématiquement), et cet observer interroge son API HTTP
# déjà existante (/api/bridge/status, cf. tools/dashboard/dashboard_server.py)
# — un seul point d'observation, partagé par l'humain (navigateur) et
# l'orchestrateur (ce module).

import json
import threading
import time
import urllib.error
import urllib.request

DASHBOARD_PORT = 8766
POLL_PERIOD_S = 0.1


class ShmObserver:
    """Interroge en tâche de fond dashboard_server.py, garde le dernier
    snapshot par segment PROD_*."""

    def __init__(self, dashboard_port: int = DASHBOARD_PORT):
        self._url = f"http://127.0.0.1:{dashboard_port}/api/bridge/status"
        self._lock = threading.Lock()
        self._latest = {}  # segment (ex: "PROD_MC") -> {fields, ts}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)

    def _run(self):
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self._url, timeout=1.0) as resp:
                    payload = json.loads(resp.read())
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError):
                # dashboard_server.py pas encore démarré / requête ratée --
                # un poll raté ne doit pas planter l'orchestrateur.
                time.sleep(POLL_PERIOD_S)
                continue

            now = time.monotonic()
            with self._lock:
                for segment, entry in payload.items():
                    if not segment.startswith("PROD_"):
                        continue
                    self._latest[segment] = {"fields": entry.get("fields", {}), "ts": now}

            time.sleep(POLL_PERIOD_S)

    def get(self, segment: str, max_age_s: float | None = None):
        """segment sans le prefixe PROD_ (ex: 'MC', 'SYS') -> dict de champs,
        ou None si jamais recu / perime (si max_age_s fourni)."""
        with self._lock:
            entry = self._latest.get(f"PROD_{segment}")
        if not entry:
            return None
        if max_age_s is not None and (time.monotonic() - entry["ts"]) > max_age_s:
            return None
        return entry["fields"]

    def mission_state(self):
        seg = self.get("MC")
        return seg.get("state") if seg else None

    def active_codes(self):
        """-> {composant: [nom_code, ...]} a partir de PROD_SYS."""
        seg = self.get("SYS") or {}
        out = {}
        for key, value in seg.items():
            if ".code." not in key:
                continue
            comp = key.split(".code.")[0]
            out.setdefault(comp, []).append(value)
        return out

    def wait_for_mission_state(self, target: str, timeout_s: float, poll_s: float = 0.1):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.mission_state() == target:
                return True
            time.sleep(poll_s)
        return False
