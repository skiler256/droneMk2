#!/usr/bin/env python3
# Compile un plan de mission YAML vers le binaire que MissionControl lit
# (DRONE_MISSION_PLAN_PATH) -- utilise missionplan_wire.py (miroir Python du
# format C++). Format YAML et procédure : tools/README.md §4.
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
