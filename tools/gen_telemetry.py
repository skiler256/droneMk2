#!/usr/bin/env python3
# tools/gen_telemetry.py
# ─────────────────────────────────────────────────────────────────────────────
# Génère include/drone/generated/telemetry.hpp à partir de TELEMETRY.csv
# (source de vérité unique du format des paquets TelMain/TelSec, partagée
# entre les deux liens). Ne PAS éditer telemetry.hpp à la main.
#
# Usage : python3 tools/gen_telemetry.py [--csv TELEMETRY.csv] [--out include/drone/generated/telemetry.hpp]
#
# Sérialisation : little-endian implicite (memcpy champ par champ) — les
# deux extrémités (Pi5 ARM64 et PC GCS x86_64) sont toutes deux
# little-endian, pas besoin de gérer l'endianness pour l'instant.

import argparse
import sys
from pathlib import Path

from telemetry_common import read_csv


def struct_name(packet_id: str) -> str:
    return "".join(p.capitalize() for p in packet_id.split("_")) + "Pkt"


def generate(packets, csv_path: Path) -> str:
    lines = []
    lines.append("// include/drone/generated/telemetry.hpp")
    lines.append("// ────────────────────────────────────────────────────────")
    lines.append(f"// GÉNÉRÉ depuis {csv_path.name} par tools/gen_telemetry.py — NE PAS ÉDITER À LA MAIN.")
    lines.append("// Régénérer : python3 tools/gen_telemetry.py (ou : cmake --build .)")
    lines.append("#pragma once")
    lines.append("#include <array>")
    lines.append("#include <cstddef>")
    lines.append("#include <cstdint>")
    lines.append("#include <cstring>")
    lines.append("#include <span>")
    lines.append("")
    lines.append("namespace TELEM {")
    lines.append("")
    lines.append("enum class Direction : uint8_t { Down = 0, Up = 1 };")
    lines.append("enum class Priority : uint8_t { Routine = 0, Critical = 1 };")
    lines.append("")
    lines.append("// Bitmask : quel(s) lien(s) portent ce paquet. TelSec (bas débit, secours)")
    lines.append("// ne porte qu'un sous-ensemble léger — TelMain porte toujours tout.")
    lines.append("enum class Link : uint8_t { Main = 0b01, Sec = 0b10 };")
    lines.append("")
    lines.append("// Identifiants stables (ordre du CSV) — NE PAS réordonner, ajouter en fin.")
    lines.append("enum class PacketID : uint8_t {")
    for i, p in enumerate(packets):
        lines.append(f"  {p.packet_id} = {i},")
    lines.append("};")
    lines.append("")

    for p in packets:
        name = struct_name(p.packet_id)
        lines.append(f"struct {name} {{")
        lines.append(f"  static constexpr PacketID kId = PacketID::{p.packet_id};")
        lines.append(f"  static constexpr size_t kSize = {p.size};")
        for f in p.fields:
            lines.append(f"  {f.cpp_type} {f.name}{{}};")
        lines.append("")
        lines.append("  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {")
        lines.append("    std::array<std::byte, kSize> buf{};")
        offset = 0
        for f in p.fields:
            lines.append(f"    std::memcpy(buf.data() + {offset}, &{f.name}, sizeof({f.name}));")
            offset += f.size
        lines.append("    return buf;")
        lines.append("  }")
        lines.append("")
        lines.append(f"  [[nodiscard]] static {name} unpack(std::span<const std::byte> bytes) noexcept {{")
        lines.append(f"    {name} out{{}};")
        lines.append("    if (bytes.size() < kSize) return out;")
        offset = 0
        for f in p.fields:
            lines.append(f"    std::memcpy(&out.{f.name}, bytes.data() + {offset}, sizeof(out.{f.name}));")
            offset += f.size
        lines.append("    return out;")
        lines.append("  }")
        lines.append("};")
        lines.append("")

    lines.append("// Une entrée par paquet : priorité/taux/taille, utilisée par le scheduler")
    lines.append("// d'émission (choix du prochain paquet dû) sans dupliquer ces constantes.")
    lines.append("struct Meta {")
    lines.append("  PacketID id;")
    lines.append("  Direction direction;")
    lines.append("  Priority priority;")
    lines.append("  uint16_t rateHz;       // 0 = événementiel")
    lines.append("  uint16_t redundancyHz; // pour événementiel : renvoi périodique même sans changement")
    lines.append("  uint8_t linkMask;      // combinaison de Link (Main | Sec)")
    lines.append("  uint8_t size;")
    lines.append("  const char *name;")
    lines.append("};")
    lines.append("")
    lines.append(f"inline constexpr std::array<Meta, {len(packets)}> kTable{{{{")
    for p in packets:
        mask = "static_cast<uint8_t>(Link::Main)" if p.links == "MAIN" \
            else "static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec)"
        lines.append(
            f'    Meta{{PacketID::{p.packet_id}, Direction::{p.direction.capitalize()}, '
            f"Priority::{p.priority}, {p.rate_hz}, {p.redundancy_hz}, {mask}, {p.size}, "
            f'"{p.packet_id}"}},')
    lines.append("}};")
    lines.append("")
    lines.append("[[nodiscard]] constexpr const Meta *find(PacketID id) noexcept {")
    lines.append("  for (const auto &m : kTable)")
    lines.append("    if (m.id == id)")
    lines.append("      return &m;")
    lines.append("  return nullptr;")
    lines.append("}")
    lines.append("")
    lines.append("} // namespace TELEM")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="TELEMETRY.csv", type=Path)
    parser.add_argument("--out", default="include/drone/generated/telemetry.hpp", type=Path)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    csv_path = args.csv if args.csv.is_absolute() else repo_root / args.csv
    out_path = args.out if args.out.is_absolute() else repo_root / args.out

    packets = read_csv(csv_path)
    if not packets:
        sys.exit(f"erreur: aucun paquet valide trouvé dans {csv_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(generate(packets, csv_path), encoding="utf-8")
    print(f"{len(packets)} paquets -> {out_path}")


if __name__ == "__main__":
    main()
