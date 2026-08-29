// GCS/ESP32/firmware/include/esp_map_generated.h
// ────────────────────────────────────────────────────────
// GÉNÉRÉ depuis mapingESP.csv par GCS/ESP32/gen_esp_map.py — NE PAS ÉDITER À LA MAIN.
#pragma once
#include <stdint.h>

enum class LedKind : uint8_t { MCP, DIRECT };

struct LedEntry {
  const char *label;
  LedKind kind;
  uint8_t mcpChipIndex; // index dans kMcpAddresses (ignoré si DIRECT)
  uint8_t mcpPin;       // 0-15 (ignoré si DIRECT)
  uint8_t gpio;         // ignoré si MCP
};

struct ButtonEntry {
  const char *label;
  uint8_t gpio;
};

inline constexpr uint8_t kMcpAddresses[] = {0x20, 0x21};

inline constexpr LedEntry kLeds[] = {
    {"ENG", LedKind::MCP, 0, 0, 0},
    {"FC", LedKind::MCP, 0, 1, 0},
    {"WFB", LedKind::MCP, 0, 2, 0},
    {"GPS", LedKind::MCP, 0, 3, 0},
    {"MAG", LedKind::MCP, 0, 4, 0},
    {"UHF", LedKind::MCP, 0, 5, 0},
    {"MW", LedKind::MCP, 0, 6, 0},
    {"BAT", LedKind::MCP, 0, 7, 0},
    {"PWR", LedKind::MCP, 0, 8, 0},
    {"NAV", LedKind::MCP, 0, 9, 0},
    {"EKF", LedKind::MCP, 0, 10, 0},
    {"VOL", LedKind::MCP, 0, 11, 0},
    {"OBS", LedKind::MCP, 0, 12, 0},
    {"LDR", LedKind::MCP, 0, 13, 0},
    {"CMD", LedKind::MCP, 0, 14, 0},
    {"RPM", LedKind::MCP, 0, 15, 0},
    {"MC", LedKind::MCP, 1, 0, 0},
    {"EMG", LedKind::MCP, 1, 1, 0},
    {"PRE", LedKind::MCP, 1, 2, 0},
    {"STP", LedKind::MCP, 1, 7, 0},
    {"HLD", LedKind::MCP, 1, 8, 0},
    {"LND", LedKind::MCP, 1, 9, 0},
    {"DISARMED", LedKind::DIRECT, 0, 0, 26},
    {"ARMED", LedKind::DIRECT, 0, 0, 18},
    {"HB_Main", LedKind::DIRECT, 0, 0, 19},
    {"HB_Sec", LedKind::DIRECT, 0, 0, 23},
};

inline constexpr ButtonEntry kButtons[] = {
    {"MC_ACK", 25},
    {"MW_ACK", 33},
};

inline constexpr size_t kNumLeds = 26;
inline constexpr size_t kNumButtons = 2;
