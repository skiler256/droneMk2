#pragma once

#include "drone/Components/SensorFusions/Driver.hpp"

class LIS3MDL : public IDriver<MagData> {
public:
  explicit LIS3MDL(std::uint8_t address = 0x1C, const char *bus = "/dev/i2c-1");

  [[nodiscard]] std::optional<TYPES::DriverError> init() override;
  [[nodiscard]] std::expected<MagData, TYPES::DriverError> update() override;

private:
  [[nodiscard]] std::optional<TYPES::DriverError> writeReg(std::uint8_t reg,
                                                           std::uint8_t value);

  std::uint8_t address_;
  const char *bus_;

  static constexpr double kScale8G = 0.29; // mG / LSB, plage ±8 gauss
};