#!/usr/bin/env bash
# sitl/run.sh
# ─────────────────────────────────────────────────────────────────────────────
# Un seul point d'entrée pour lancer le SITL : construit si besoin (build
# SITL séparé, build_sitl/ — jamais le build de prod), crée le venv Python
# si besoin, puis délègue à run_scenario.py (mode scénario ou interactif).
# Le dashboard web (localhost:8766/sitl/prod) est démarré automatiquement
# par procman.py — rien à faire de spécial ici pour l'avoir.
#
# Usage :
#   sitl/run.sh                                   # interactif, sans ArduPilot
#   sitl/run.sh scenarios/001_arm_takeoff_wait_land.yaml
#   sitl/run.sh --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py scenarios/...
#   sitl/run.sh --rebuild ...                     # force la reconfiguration/recompilation
#
# ARDUPILOT_SIM_VEHICLE (variable d'env) : évite de retaper --sim-vehicle à
# chaque fois — export-la dans ton .bashrc si tu veux.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$REPO_ROOT/build_sitl"
VENV="$REPO_ROOT/sitl/.venv"

usage() {
  sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

REBUILD=0
SIM_VEHICLE="${ARDUPILOT_SIM_VEHICLE:-}"
SCENARIO=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rebuild) REBUILD=1; shift ;;
    --sim-vehicle) SIM_VEHICLE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) SCENARIO="$1"; shift ;;
  esac
done

# ─── Build (séparé du build de prod — jamais les mêmes drivers) ────────────
if [[ "$REBUILD" == "1" || ! -x "$BUILD_DIR/drone" || ! -x "$BUILD_DIR/udp_bridge" ]]; then
  echo "== build SITL ($BUILD_DIR) =="
  cmake -S "$REPO_ROOT" -B "$BUILD_DIR" -DBUILD_ITESTS=ON \
    -DDRONE_USE_SITL_DRIVERS=ON -DDRONE_USE_SIM_CLOCK=ON
  cmake --build "$BUILD_DIR" -j"$(nproc)" --target drone udp_bridge
fi

# ─── venv Python (pymavlink/pyyaml, cf. sitl/requirements.txt) ─────────────
if [[ ! -x "$VENV/bin/python3" ]]; then
  echo "== creation du venv sitl/.venv =="
  python3 -m venv "$VENV"
  "$VENV/bin/pip" install --quiet -r "$REPO_ROOT/sitl/requirements.txt"
fi

ARGS=("$REPO_ROOT/sitl/run_scenario.py" --build-dir "$BUILD_DIR")
[[ -n "$SIM_VEHICLE" ]] && ARGS+=(--sim-vehicle "$SIM_VEHICLE")
[[ -n "$SCENARIO" ]] && ARGS+=("$SCENARIO")

"$VENV/bin/python3" "${ARGS[@]}"
