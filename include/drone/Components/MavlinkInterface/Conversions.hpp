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

// GPS_INPUT.yaw : cdeg, 0 = "non disponible" selon la spec MAVLink (PAS
// "plein nord") — la valeur 0 doit donc toujours être remappée sur 36000.
[[nodiscard]] inline uint16_t headingToYawCdeg(TYPES::Radians heading) noexcept {
  float deg = TYPES::toDegrees(heading).v;
  deg = std::fmod(deg, 360.0f);
  if (deg < 0.0f)
    deg += 360.0f;
  auto cdeg = static_cast<uint16_t>(std::lround(deg * 100.0));
  return cdeg == 0 ? 36000 : cdeg;
}

// GPS_INPUT.lat/lon : degE7 (degrés * 1e7, int32).
[[nodiscard]] inline int32_t degToE7(double deg) noexcept {
  return static_cast<int32_t>(std::lround(deg * 1e7));
}

// BATTERY_STATUS.voltages[0] : mV -> V.
[[nodiscard]] inline TYPES::Volts mvToVolts(uint16_t mv) noexcept {
  return TYPES::Volts{static_cast<float>(mv) / 1000.0f};
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

// Un ACK (COMMAND_ACK ou PARAM_VALUE) correspond-il à la commande
// actuellement en attente ? Faux si le type d'ACK ne correspond pas au
// type de commande en attente (ex: PARAM_VALUE reçu alors qu'on attend un
// COMMAND_ACK), ou si l'identifiant (command / param_id) diffère.
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
