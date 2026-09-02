#include "drone/Components/SystemMonitoring/Driver/LinuxThermalZoneDriver.hpp"

#include <charconv>
#include <cstdio>

LinuxThermalZoneDriver::LinuxThermalZoneDriver(const char *zonePath) : zonePath_(zonePath) {}

std::expected<float, TYPES::DriverError> LinuxThermalZoneDriver::readCelsius() {
  std::FILE *f = std::fopen(zonePath_, "r");
  if (f == nullptr)
    return std::unexpected(TYPES::DriverError::NotInitialized);

  char buf[32]{};
  std::size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
  std::fclose(f);
  if (n == 0)
    return std::unexpected(TYPES::DriverError::InvalidData);

  // thermal_zone*/temp expose des millidegrés Celsius, sans saut de ligne
  // significatif pour from_chars (il s'arrête au premier caractère non
  // numérique de toute façon).
  int milliC{};
  auto [ptr, ec] = std::from_chars(buf, buf + n, milliC);
  if (ec != std::errc{})
    return std::unexpected(TYPES::DriverError::InvalidData);

  return static_cast<float>(milliC) / 1000.0f;
}
