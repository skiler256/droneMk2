#!/usr/bin/env python3
# tools/mission_compiler.py
# ─────────────────────────────────────────────────────────────────────────────
# Compile un plan de mission YAML (lisible/editable a la main) vers le binaire
# que MissionControl lit (DRONE_MISSION_PLAN_PATH, cf. MissionControl.cpp
# runPreflightChecks()) -- utilise missionplan_wire.py (miroir Python du
# format C++, cf. son en-tete). Vise a devenir l'outil de compilation cote
# GCS une fois le pipeline d'upload construit, pas seulement un outil SITL.
#
# Format YAML :
#   header:                    # optionnel, champs manquants = defauts C++
#     battWarningV: 14.0
#     battCriticalV: 12.5
#     rtbTransitAlt: 30.0
#     gcsLossAction: 0
#     navVolumeRadius: 200.0
#     navVolumeCeiling: 100.0
#   segments:                  # optionnel, liste vide = plan sans segment
#     - type: Hold             # Waypoint|Corridor|Curve|Landing|Hold|PayloadAction
#       duration: 15.0         # champs specifiques au type, cf. missionplan_wire.py
#
# Usage : tools/mission_compiler.py --yaml plan.yaml --out plan.bin

import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import missionplan_wire  # noqa: E402


def compile_plan(yaml_path: Path) -> bytes:
    doc = yaml.safe_load(yaml_path.read_text()) or {}
    header = doc.get("header", {}) or {}
    segments_doc = doc.get("segments", []) or []

    segments = []
    for i, seg in enumerate(segments_doc):
        if "type" not in seg:
            raise ValueError(f"segment #{i}: champ 'type' manquant")
        seg_type = seg["type"]
        fields = {k: v for k, v in seg.items() if k != "type"}
        segments.append((seg_type, fields))

    return missionplan_wire.encode(header, segments)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    bytes_out = compile_plan(args.yaml)
    args.out.write_bytes(bytes_out)
    print(f"mission_compiler: {args.yaml} -> {args.out} ({len(bytes_out)} octets)")


if __name__ == "__main__":
    main()
