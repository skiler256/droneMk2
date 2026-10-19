#pragma once

#include "drone/Components/SensorFusions/Driver.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

// Driver de dev/SITL : GPSData simulé en UDP à la place du NEO_M8N réel,
// sélectionné par l'option CMake DRONE_USE_SITL_DRIVERS.
//
// Trame : [magic:4]["GPS1"][version:1][champs GPSData en LE] — magic et
// version explicites contre le désalignement de struct entre les deux bouts
// (cf. INTEGRATION_TEST.md §8.4).
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
