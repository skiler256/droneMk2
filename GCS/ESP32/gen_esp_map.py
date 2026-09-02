#!/usr/bin/env python3
# Génère firmware/include/esp_map_generated.h à partir de mapingESP.csv —
# source de vérité unique du câblage (voyants MCP23017/GPIO direct + boutons),
# même principe que tools/gen_codes.py. Ne PAS éditer esp_map_generated.h à
# la main.
#
# Usage : python3 GCS/ESP32/gen_esp_map.py
#
# Format attendu de mapingESP.csv (séparateur ';', en-tête sur la 1re ligne) :
#   Label;Kind;Index;Chip;Port;Bit;Gpio
#   ENG;MCP;0;0x20;A;0;            <- voyant sur MCP23017 0x20, port A, bit 0
#   DISARMED;DIRECT;32;;;;26        <- voyant sur GPIO26 direct
#   MC_ACK;BTN;40;;;;25             <- bouton sur GPIO25

import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CSV_PATH = HERE / "mapingESP.csv"
OUT_PATH = HERE / "firmware" / "include" / "esp_map_generated.h"

KNOWN_KINDS = {"MCP", "DIRECT", "BTN"}


def read_rows(path: Path):
    text = path.read_text(encoding="utf-8")
    reader = csv.DictReader(text.splitlines(), delimiter=";")
    rows = []
    for lineno, row in enumerate(reader, start=2):
        label = (row.get("Label") or "").strip()
        if not label:
            continue
        kind = (row.get("Kind") or "").strip().upper()
        if kind not in KNOWN_KINDS:
            sys.exit(f"erreur: {path}:{lineno}: Kind '{kind}' inconnu (attendu {KNOWN_KINDS})")
        rows.append({
            "label": label,
            "kind": kind,
            "chip": (row.get("Chip") or "").strip(),
            "port": (row.get("Port") or "").strip().upper(),
            "bit": (row.get("Bit") or "").strip(),
            "gpio": (row.get("Gpio") or "").strip(),
        })
    return rows


def mcp_pin(port: str, bit: str) -> int:
    # Adafruit_MCP23017 : broches 0-7 = port A, 8-15 = port B.
    return int(bit) + (8 if port == "B" else 0)


def generate(rows) -> str:
    leds = [r for r in rows if r["kind"] in ("MCP", "DIRECT")]
    buttons = [r for r in rows if r["kind"] == "BTN"]
    chips = sorted({r["chip"] for r in rows if r["kind"] == "MCP"})

    lines = []
    lines.append("// GCS/ESP32/firmware/include/esp_map_generated.h")
    lines.append("// ────────────────────────────────────────────────────────")
    lines.append("// GÉNÉRÉ depuis mapingESP.csv par GCS/ESP32/gen_esp_map.py — NE PAS ÉDITER À LA MAIN.")
    lines.append("#pragma once")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("enum class LedKind : uint8_t { MCP, DIRECT };")
    lines.append("")
    lines.append("struct LedEntry {")
    lines.append("  const char *label;")
    lines.append("  LedKind kind;")
    lines.append("  uint8_t mcpChipIndex; // index dans kMcpAddresses (ignoré si DIRECT)")
    lines.append("  uint8_t mcpPin;       // 0-15 (ignoré si DIRECT)")
    lines.append("  uint8_t gpio;         // ignoré si MCP")
    lines.append("};")
    lines.append("")
    lines.append("struct ButtonEntry {")
    lines.append("  const char *label;")
    lines.append("  uint8_t gpio;")
    lines.append("};")
    lines.append("")
    lines.append(f"inline constexpr uint8_t kMcpAddresses[] = {{{', '.join(chips)}}};")
    lines.append("")
    lines.append(f"inline constexpr LedEntry kLeds[] = {{")
    for r in leds:
        if r["kind"] == "MCP":
            chip_idx = chips.index(r["chip"])
            pin = mcp_pin(r["port"], r["bit"])
            lines.append(f'    {{"{r["label"]}", LedKind::MCP, {chip_idx}, {pin}, 0}},')
        else:
            lines.append(f'    {{"{r["label"]}", LedKind::DIRECT, 0, 0, {r["gpio"]}}},')
    lines.append("};")
    lines.append("")
    lines.append(f"inline constexpr ButtonEntry kButtons[] = {{")
    for r in buttons:
        lines.append(f'    {{"{r["label"]}", {r["gpio"]}}},')
    lines.append("};")
    lines.append("")
    lines.append(f"inline constexpr size_t kNumLeds = {len(leds)};")
    lines.append(f"inline constexpr size_t kNumButtons = {len(buttons)};")
    lines.append("")
    return "\n".join(lines)


def main():
    if not CSV_PATH.exists():
        sys.exit(f"erreur: {CSV_PATH} introuvable")
    rows = read_rows(CSV_PATH)
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(generate(rows), encoding="utf-8")
    leds = sum(1 for r in rows if r["kind"] in ("MCP", "DIRECT"))
    buttons = sum(1 for r in rows if r["kind"] == "BTN")
    print(f"{leds} voyants, {buttons} boutons -> {OUT_PATH}")


if __name__ == "__main__":
    main()
