#pragma once

#include "drone/Components/SensorFusions/Driver.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

// Driver factice de dev/SITL : reçoit un GPSData simulé en UDP (envoyé par
// le SITL_MASTER, cf. sitl/gps_bridge.py — Phase B) à la place du vrai
// NEO_M8N I2C/UART. Sélectionné dans main.cpp via l'option CMake
// DRONE_USE_SITL_DRIVERS, même principe que UdpMavlinkLink pour
// MavlinkInterface.
//
// Trame : [magic:4]["GPS1"][version:1][champs GPSData en LE] — magic+version
// explicites pour éviter le piège connu de désalignement de struct entre
// deux bouts d'un protocole (cf. INTEGRATION_TEST.md §8.4), même esprit que
// TELEM::*/MISSIONPLAN_CONV côté télémétrie.
class UdpGpsDriver : public IDriver<GPSData> {
public:
  explicit UdpGpsDriver(uint16_t localPort);
  ~UdpGpsDriver() override;

  UdpGpsDriver(const UdpGpsDriver &) = delete;
  UdpGpsDriver &operator=(const UdpGpsDriver &) = delete;

  [[nodiscard]] std::optional<TYPES::DriverError> init() override;
  [[nodiscard]] std::expected<GPSData, TYPES::DriverError> update() override;

private:
  uint16_t localPort_;
  std::array<std::byte, 256> rxBuf_{};
};
