#pragma once
#include "drone/Components/MavlinkInterface/PendingAck.hpp"
#include "drone/types.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mavlink.h>

// Conversions pures entre les unités MAVLink et TYPES:: — extraites de
// TTx/TRx::loop() pour être testables sans passer par les Tasks (loop()
// est privée, appelée uniquement par le thread RT de la Task).
namespace MAVLINK_CONV {

// GPS_INPUT.lat/lon : degE7 (degrés * 1e7, int32).
[[nodiscard]] inline int32_t degToE7(double deg) noexcept {
  return static_cast<int32_t>(std::lround(deg * 1e7));
}

// BATTERY_STATUS.voltages[0] : mV -> V. UINT16_MAX = "non mesuré" (spec
// MAVLink) -> 0V, même convention que caToAmps(-1)/clampBatteryPct(-1)
// ci-dessous — sans ce garde, 65535mV afficherait 65.5V au lieu de "inconnu".
[[nodiscard]] inline TYPES::Volts mvToVolts(uint16_t mv) noexcept {
  return TYPES::Volts{mv == UINT16_MAX ? 0.0f : static_cast<float>(mv) / 1000.0f};
}

// BATTERY_STATUS.current_battery : cA (0.01A), -1 si non mesuré -> 0A.
[[nodiscard]] inline TYPES::Amps caToAmps(int16_t ca) noexcept {
  return TYPES::Amps{ca >= 0 ? static_cast<float>(ca) * 0.01f : 0.0f};
}

// BATTERY_STATUS.battery_remaining : int8 %, -1 si inconnu -> 0%.
[[nodiscard]] inline uint8_t clampBatteryPct(int8_t pct) noexcept {
  return static_cast<uint8_t>(pct >= 0 ? pct : 0);
}

// SCALED_PRESSURE.temperature : cdegC -> degC.
[[nodiscard]] inline float cdegcToC(int16_t cdegc) noexcept {
  return static_cast<float>(cdegc) / 100.0f;
}

// ATTITUDE.roll/pitch/yaw (rad, Euler ZYX) -> quaternion. ArduPilot SITL
// n'émet jamais ATTITUDE_QUATERNION (constaté en session, quelle que soit
// la demande SET_MESSAGE_INTERVAL) — seulement ATTITUDE en Euler, d'où
// cette conversion locale plutôt que de dépendre d'un message que le FC ne
// produit pas.
[[nodiscard]] inline TYPES::Quat eulerToQuat(float roll, float pitch, float yaw) noexcept {
  float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
  float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
  float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);

  return TYPES::Quat{cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

// ESC_TELEMETRY_1_TO_4.current[i] : cA (0.01A) -> A. Distinct de caToAmps()
// (int16_t signé, -1="non mesuré") : ici uint16_t non signé, jamais négatif.
[[nodiscard]] inline float escCaToAmps(uint16_t ca) noexcept {
  return static_cast<float>(ca) * 0.01f;
}

// Un ACK (COMMAND_ACK ou PARAM_VALUE) correspond-il à la commande en
// attente ? Faux si le type ou l'identifiant (command/param_id) diffère.
[[nodiscard]] inline bool ackMatchesPending(const mavlink_message_t &msg,
                                            const PendingAck &pending) noexcept {
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK && pending.kind == PendingKind::Command) {
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&msg, &ack);
    return ack.command == pending.command;
  }
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_VALUE && pending.kind == PendingKind::Param) {
    mavlink_param_value_t val{};
    mavlink_msg_param_value_decode(&msg, &val);
    return std::strncmp(val.param_id, pending.paramId.data(), 16) == 0;
  }
  return false;
}

} // namespace MAVLINK_CONV
