#!/usr/bin/env python3
# tools/gen_codes.py
# ─────────────────────────────────────────────────────────────────────────────
# Génère include/drone/generated/codes.hpp à partir de INDEX.csv (source de
# vérité unique du dictionnaire de codes, lisible aussi tel quel par le GCS
# Python, et éditable via tools/codes_editor.py). Ne PAS éditer codes.hpp à
# la main — il est régénéré (par ce script, ou automatiquement par CMake).
#
# Usage : python3 tools/gen_codes.py [--csv INDEX.csv] [--out include/drone/generated/codes.hpp]
#
# Format attendu de INDEX.csv (séparateur ';', en-tête sur la première ligne) :
#   Code;Component;Category;Name;Description;Criticity
#   FCP001;Flight Controller;Power;batt_low;tension batterie faible;Critical
#
# Tables composant/catégorie/sévérité et logique de validation : voir
# tools/codes_common.py (partagé avec codes_editor.py).

import argparse
import sys
from pathlib import Path

from codes_common import COMPONENTS, CATEGORIES, read_csv


def cpp_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"')


def generate(codes, csv_path: Path) -> str:
    lines = []
    lines.append("// include/drone/generated/codes.hpp")
    lines.append("// ────────────────────────────────────────────────────────")
    lines.append(f"// GÉNÉRÉ depuis {csv_path.name} par tools/gen_codes.py — NE PAS ÉDITER À LA MAIN.")
    lines.append(f"// Régénérer : python3 tools/gen_codes.py (ou : cmake --build .)")
    lines.append("#pragma once")
    lines.append('#include "drone/types.hpp"')
    lines.append("#include <array>")
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append("namespace CODES {")
    lines.append("")
    lines.append("enum class Component : uint8_t {")
    for label, (id_, full) in sorted(COMPONENTS.items(), key=lambda kv: kv[1][0]):
        lines.append(f"  {label} = {id_}, // {full}")
    lines.append("};")
    lines.append("")
    lines.append("enum class Category : uint8_t {")
    for label, (id_, full) in sorted(CATEGORIES.items(), key=lambda kv: kv[1][0]):
        lines.append(f"  {label} = {id_}, // {full}")
    lines.append("};")
    lines.append("")
    lines.append("enum class Severity : uint8_t { Info = 0, Warning = 1, Critical = 2, Emergency = 3 };")
    lines.append("")
    lines.append("// true si Severity doit être stockée dans l'ensemble actif \"majeur\"")
    lines.append("// (jamais écrasé silencieusement) plutôt que le ring buffer \"mineur\".")
    lines.append("[[nodiscard]] constexpr bool isMajor(Severity s) noexcept {")
    lines.append("  return s == Severity::Critical || s == Severity::Emergency;")
    lines.append("}")
    lines.append("")
    lines.append("struct Code {")
    lines.append("  Component component;")
    lines.append("  Category category;")
    lines.append("  uint16_t number;")
    lines.append("")
    lines.append("  constexpr bool operator==(const Code &) const noexcept = default;")
    lines.append("")
    lines.append("  // [component:8][category:8][number:16] — 4 octets.")
    lines.append("  [[nodiscard]] constexpr uint32_t pack() const noexcept {")
    lines.append("    return (static_cast<uint32_t>(component) << 24) |")
    lines.append("           (static_cast<uint32_t>(category) << 16) |")
    lines.append("           static_cast<uint32_t>(number);")
    lines.append("  }")
    lines.append("")
    lines.append("  [[nodiscard]] static constexpr Code unpack(uint32_t packed) noexcept {")
    lines.append("    return Code{")
    lines.append("        static_cast<Component>((packed >> 24) & 0xFF),")
    lines.append("        static_cast<Category>((packed >> 16) & 0xFF),")
    lines.append("        static_cast<uint16_t>(packed & 0xFFFF),")
    lines.append("    };")
    lines.append("  }")
    lines.append("};")
    lines.append("")
    lines.append("// Une entrée par code : sert au lookup runtime de severity()/name()/description().")
    lines.append("struct Entry {")
    lines.append("  Code code;")
    lines.append("  Severity severity;")
    lines.append("  const char *name;")
    lines.append("  const char *description;")
    lines.append("};")
    lines.append("")
    for c in codes:
        lines.append(f"inline constexpr Code {c.code}{{Component::{c.component_label}, "
                     f"Category::{c.category_label}, {c.number}}};")
    lines.append("")
    lines.append(f"inline constexpr std::array<Entry, {len(codes)}> kTable{{{{")
    for c in codes:
        lines.append(f'    Entry{{{c.code}, Severity::{c.severity}, '
                     f'"{cpp_escape(c.name)}", "{cpp_escape(c.description)}"}},')
    lines.append("}};")
    lines.append("")
    lines.append("// Recherche linéaire : table petite, appelée hors boucle chaude uniquement")
    lines.append("// (raiseCode/clearCode, pas depuis loop()).")
    lines.append("[[nodiscard]] constexpr const Entry *find(Code code) noexcept {")
    lines.append("  for (const auto &e : kTable)")
    lines.append("    if (e.code == code)")
    lines.append("      return &e;")
    lines.append("  return nullptr;")
    lines.append("}")
    lines.append("")
    lines.append("} // namespace CODES")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="INDEX.csv", type=Path)
    parser.add_argument("--out", default="include/drone/generated/codes.hpp", type=Path)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    csv_path = args.csv if args.csv.is_absolute() else repo_root / args.csv
    out_path = args.out if args.out.is_absolute() else repo_root / args.out

    codes = read_csv(csv_path)
    if not codes:
        sys.exit(f"erreur: aucun code valide trouvé dans {csv_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(generate(codes, csv_path), encoding="utf-8")
    print(f"{len(codes)} codes -> {out_path}")


if __name__ == "__main__":
    main()
