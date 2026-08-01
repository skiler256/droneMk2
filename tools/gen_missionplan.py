#!/usr/bin/env python3
# tools/gen_missionplan.py
# ─────────────────────────────────────────────────────────────────────────────
# Génère include/drone/generated/missionplan.hpp à partir de MISSIONPLAN.csv
# (source de vérité unique du header + des types de segment d'un plan de
# mission). Ne PAS éditer missionplan.hpp à la main.
#
# Usage : python3 tools/gen_missionplan.py [--csv MISSIONPLAN.csv] [--out include/drone/generated/missionplan.hpp]
#
# Ce fichier généré ne contient QUE le header et les segments (schéma
# répétitif). Le framing du plan complet (magic/version/CRC32, boucle TLV
# sur la liste de segments) est de la logique, pas du répétitif — voir
# MISSIONPLAN_CONV dans MissionControl/MissionPlan.hpp (écrit à la main).

import argparse
import sys
from pathlib import Path

from missionplan_common import read_csv


def struct_name(name: str) -> str:
    return "".join(p.capitalize() for p in name.split("_"))


def emit_pack_unpack(lines, fields, size_expr):
    lines.append(f"  [[nodiscard]] std::array<std::byte, {size_expr}> pack() const noexcept {{")
    lines.append(f"    std::array<std::byte, {size_expr}> buf{{}};")
    offset = 0
    for f in fields:
        lines.append(f"    std::memcpy(buf.data() + {offset}, &{f.name}, sizeof({f.name}));")
        offset += f.size
    lines.append("    return buf;")
    lines.append("  }")
    lines.append("")


def emit_unpack(lines, type_name, fields, size_expr):
    lines.append(f"  [[nodiscard]] static {type_name} unpack(std::span<const std::byte> bytes) noexcept {{")
    lines.append(f"    {type_name} out{{}};")
    lines.append(f"    if (bytes.size() < {size_expr}) return out;")
    offset = 0
    for f in fields:
        lines.append(f"    std::memcpy(&out.{f.name}, bytes.data() + {offset}, sizeof(out.{f.name}));")
        offset += f.size
    lines.append("    return out;")
    lines.append("  }")


def generate(entities, csv_path: Path) -> str:
    header = next(e for e in entities if e.kind == "HEADER")
    segments = [e for e in entities if e.kind == "SEGMENT"]

    lines = []
    lines.append("// include/drone/generated/missionplan.hpp")
    lines.append("// ────────────────────────────────────────────────────────")
    lines.append(f"// GÉNÉRÉ depuis {csv_path.name} par tools/gen_missionplan.py — NE PAS ÉDITER À LA MAIN.")
    lines.append("// Régénérer : python3 tools/gen_missionplan.py (ou : cmake --build .)")
    lines.append("#pragma once")
    lines.append("#include <array>")
    lines.append("#include <cstddef>")
    lines.append("#include <cstdint>")
    lines.append("#include <cstring>")
    lines.append("#include <optional>")
    lines.append("#include <span>")
    lines.append("#include <variant>")
    lines.append("#include <vector>")
    lines.append("")
    lines.append("namespace MISSIONPLAN {")
    lines.append("")

    # ─── MissionHeader ──────────────────────────────────────────────────
    lines.append("struct MissionHeader {")
    lines.append(f"  static constexpr std::size_t kSize = {header.size};")
    for f in header.fields:
        lines.append(f"  {f.cpp_type} {f.name}{{{f.cpp_default}}};")
    lines.append("")
    emit_pack_unpack(lines, header.fields, "kSize")
    emit_unpack(lines, "MissionHeader", header.fields, "kSize")
    lines.append("};")
    lines.append("")

    # ─── SegmentKind ────────────────────────────────────────────────────
    lines.append("// Identifiant TLV stable (ordre du CSV) — NE PAS réordonner des entrées")
    lines.append("// existantes, ajouter seulement en fin (compat plans déjà écrits sur disque).")
    lines.append("enum class SegmentKind : uint8_t {")
    for i, s in enumerate(segments):
        lines.append(f"  {struct_name(s.name)} = {i},")
    lines.append("};")
    lines.append("")

    # ─── Une struct par type de segment ─────────────────────────────────
    struct_names = []
    for s in segments:
        name = struct_name(s.name) + "Segment"
        struct_names.append(name)
        lines.append(f"struct {name} {{")
        lines.append(f"  static constexpr SegmentKind kKind = SegmentKind::{struct_name(s.name)};")
        lines.append(f"  static constexpr std::size_t kPayloadSize = {s.size};")
        lines.append("  static_assert(kPayloadSize <= 255, \"payload doit tenir dans le champ len (u8) du TLV\");")
        for f in s.fields:
            lines.append(f"  {f.cpp_type} {f.name}{{{f.cpp_default}}};")
        lines.append("")
        emit_pack_unpack(lines, s.fields, "kPayloadSize")
        emit_unpack(lines, name, s.fields, "kPayloadSize")
        lines.append("};")
        lines.append("")

    # ─── Variant + encode/decode TLV ────────────────────────────────────
    variant_list = ", ".join(struct_names)
    lines.append(f"using SegmentVariant = std::variant<{variant_list}>;")
    lines.append("")
    lines.append("// [kind:u8][len:u8][payload...] — len permet à un lecteur de sauter un")
    lines.append("// segment de type inconnu (plan écrit par un firmware plus récent) sans")
    lines.append("// faire échouer tout le parse (cf. decodeSegment ci-dessous).")
    lines.append("")
    lines.append("[[nodiscard]] inline std::vector<std::byte> encodeSegment(const SegmentVariant &seg) noexcept {")
    lines.append("  return std::visit([](const auto &s) {")
    lines.append("    std::vector<std::byte> out;")
    lines.append("    out.reserve(2 + s.kPayloadSize);")
    lines.append("    out.push_back(static_cast<std::byte>(s.kKind));")
    lines.append("    out.push_back(static_cast<std::byte>(s.kPayloadSize));")
    lines.append("    auto payload = s.pack();")
    lines.append("    out.insert(out.end(), payload.begin(), payload.end());")
    lines.append("    return out;")
    lines.append("  }, seg);")
    lines.append("}")
    lines.append("")
    lines.append("struct DecodedSegment {")
    lines.append("  // nullopt si kind inconnu ou len incohérente avec le type attendu —")
    lines.append("  // consumed reste correct dans tous les cas pour permettre de sauter")
    lines.append("  // l'entrée et continuer le parse du reste du plan.")
    lines.append("  std::optional<SegmentVariant> segment;")
    lines.append("  std::size_t consumed{0};")
    lines.append("};")
    lines.append("")
    lines.append("[[nodiscard]] inline DecodedSegment decodeSegment(std::span<const std::byte> bytes) noexcept {")
    lines.append("  if (bytes.size() < 2) return {};")
    lines.append("  auto kind = static_cast<uint8_t>(bytes[0]);")
    lines.append("  auto len = static_cast<uint8_t>(bytes[1]);")
    lines.append("  if (bytes.size() < static_cast<std::size_t>(2 + len)) return {};")
    lines.append("  auto payload = bytes.subspan(2, len);")
    lines.append("  DecodedSegment out{.segment = std::nullopt, .consumed = static_cast<std::size_t>(2 + len)};")
    lines.append("  switch (static_cast<SegmentKind>(kind)) {")
    for s, name in zip(segments, struct_names):
        lines.append(f"  case SegmentKind::{struct_name(s.name)}:")
        lines.append(f"    if (len != {name}::kPayloadSize) return out;")
        lines.append(f"    out.segment = {name}::unpack(payload);")
        lines.append("    return out;")
    lines.append("  }")
    lines.append("  return out; // kind inconnu -> segment=nullopt, consumed correct (skip)")
    lines.append("}")
    lines.append("")
    lines.append("} // namespace MISSIONPLAN")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="MISSIONPLAN.csv", type=Path)
    parser.add_argument("--out", default="include/drone/generated/missionplan.hpp", type=Path)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    csv_path = args.csv if args.csv.is_absolute() else repo_root / args.csv
    out_path = args.out if args.out.is_absolute() else repo_root / args.out

    entities = read_csv(csv_path)
    if not entities:
        sys.exit(f"erreur: aucune entité valide trouvée dans {csv_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(generate(entities, csv_path), encoding="utf-8")
    n_segments = sum(1 for e in entities if e.kind == "SEGMENT")
    print(f"1 header + {n_segments} types de segment -> {out_path}")


if __name__ == "__main__":
    main()
