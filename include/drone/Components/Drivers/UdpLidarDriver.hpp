#pragma once

#include "drone/Components/SensorFusions/Driver.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

// Driver de dev/SITL : LidarData simulé en UDP à la place du LD06 réel —
// "rien détecté" constant tant que le moteur 3D n'existe pas.
//
// Trame : [magic:4]["LID1"][version:1][distance_mm:360*u16][intensity:360*u8]
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
  // 360*(2+1) + en-tête = 1085 octets, marge jusqu'au MTU Ethernet.
  std::array<std::byte, 1536> rxBuf_{};
};
