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
#   sitl/run.sh --gcs-windows --sim-vehicle ...   # ouvre console+carte MAVProxy (besoin DISPLAY)
#   sitl/run.sh --sensor-fake-fixed --sim-vehicle ... # diagnostic : bypass SIM_STATE, GPS+LD06 fixe en continu
#   sitl/run.sh --godot-headless ~/godot3/Godot_v3.6-stable_linux_headless.64 ... # moteur 3D (LidarServer)
#   sitl/run.sh --viewer 127.0.0.1:15103 ...  # diffuse la pose vers un Viewer.gd
#                                              # deja lance a la main (cf. SITL.md §9)
#   sitl/run.sh --godot-editor ~/godot3-editor/Godot_v3.6-stable_x11.64 ... # lance le viewer
#                                              # automatiquement (fenetre), pas besoin d'un
#                                              # deuxieme terminal -- ignore si --viewer deja donne
#
# ARDUPILOT_SIM_VEHICLE / GODOT_HEADLESS_BIN / GODOT_EDITOR_BIN (variables
# d'env) : évitent de retaper --sim-vehicle/--godot-headless/--godot-editor
# à chaque fois — exporte-les dans ton .bashrc si tu veux.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$REPO_ROOT/build_sitl"
VENV="$REPO_ROOT/sitl/.venv"

usage() {
  sed -n '2,26p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

REBUILD=0
SIM_VEHICLE="${ARDUPILOT_SIM_VEHICLE:-}"
GODOT_HEADLESS="${GODOT_HEADLESS_BIN:-}"
GODOT_EDITOR="${GODOT_EDITOR_BIN:-}"
GCS_WINDOWS=0
SENSOR_FAKE_FIXED=0
VIEWER=""
SCENARIO=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rebuild) REBUILD=1; shift ;;
    --sim-vehicle) SIM_VEHICLE="$2"; shift 2 ;;
    --godot-headless) GODOT_HEADLESS="$2"; shift 2 ;;
    --godot-editor) GODOT_EDITOR="$2"; shift 2 ;;
    --gcs-windows) GCS_WINDOWS=1; shift ;;
    --sensor-fake-fixed) SENSOR_FAKE_FIXED=1; shift ;;
    --viewer) VIEWER="$2"; shift 2 ;;
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
[[ -n "$GODOT_HEADLESS" ]] && ARGS+=(--godot-headless "$GODOT_HEADLESS")
[[ -n "$GODOT_EDITOR" ]] && ARGS+=(--godot-editor "$GODOT_EDITOR")
[[ "$GCS_WINDOWS" == "1" ]] && ARGS+=(--gcs-windows)
[[ "$SENSOR_FAKE_FIXED" == "1" ]] && ARGS+=(--sensor-fake-fixed)
[[ -n "$VIEWER" ]] && ARGS+=(--viewer "$VIEWER")
[[ -n "$SCENARIO" ]] && ARGS+=("$SCENARIO")

"$VENV/bin/python3" "${ARGS[@]}"
