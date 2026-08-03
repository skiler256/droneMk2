#pragma once

#include "drone/Components/SensorFusions/Driver.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

// Driver factice de dev/SITL : reçoit un LidarData simulé en UDP (envoyé
// par le SITL_MASTER — "rien détecté" constant tant que le moteur 3D
// n'existe pas, cf. INTEGRATION_TEST.md roadmap étape 4) à la place du
// vrai LD06 UART. Sélectionné via DRONE_USE_SITL_DRIVERS, cf. UdpGpsDriver.
//
// Trame : [magic:4]["LID1"][version:1][distance_mm:360*u16][intensity:360*u8]
// — magic+version explicites, cf. INTEGRATION_TEST.md §8.4.
class UdpLidarDriver : public IDriver<LidarData> {
public:
  explicit UdpLidarDriver(uint16_t localPort);
  ~UdpLidarDriver() override;

  UdpLidarDriver(const UdpLidarDriver &) = delete;
  UdpLidarDriver &operator=(const UdpLidarDriver &) = delete;

  [[nodiscard]] std::optional<TYPES::DriverError> init() override;
  [[nodiscard]] std::expected<LidarData, TYPES::DriverError> update() override;

private:
  uint16_t localPort_;
  // 360*(2+1) + en-tête = 1085 octets — marge jusqu'au MTU Ethernet standard,
  // même raisonnement que UdpTelemetryDriver::rxBuf_ (cf. LIDAR_SCAN).
  std::array<std::byte, 1536> rxBuf_{};
};
