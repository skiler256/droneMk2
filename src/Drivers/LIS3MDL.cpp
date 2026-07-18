#include "drone/Components/Drivers/LIS3MDL.hpp"
#include "drone/types.hpp"

#include <unistd.h>

LIS3MDL::LIS3MDL(std::uint8_t address, const char *bus)
    : address_(address), bus_(bus) {}

std::optional<TYPES::DriverError> LIS3MDL::init() {
  if (auto err = i2cConnect(address_, bus_)) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return err;
  }

  // Registres de configuration — NE PAS MODIFIER, valeurs reprises à
  // l'identique de l'ancien driver (CTRL_REG1..4).
  if (auto err = writeReg(0x20, 0b01111000))
    return err; // CTRL_REG1
  if (auto err = writeReg(0x21, 0b00100000))
    return err; // CTRL_REG2
  if (auto err = writeReg(0x22, 0b00000000))
    return err; // CTRL_REG3
  if (auto err = writeReg(0x23, 0b00001100))
    return err; // CTRL_REG4

  setHealth(TYPES::DriverHealth::Connected);
  return TYPES::DriverError::None;
}

std::optional<TYPES::DriverError> LIS3MDL::writeReg(std::uint8_t reg,
                                                    std::uint8_t value) {
  const std::uint8_t buf[2] = {reg, value};
  if (::write(fd_, buf, 2) != 2) {
    return TYPES::DriverError::I2CWriteFailed;
  }
  return std::nullopt;
}

std::expected<MagData, TYPES::DriverError> LIS3MDL::update() {
  // 1. Lecture du registre de statut (STATUS_REG = 0x27)
  std::uint8_t statusReg = 0x27;
  if (::write(fd_, &statusReg, 1) != 1) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return std::unexpected(TYPES::DriverError::I2CWriteFailed);
  }

  std::uint8_t status{0};
  if (::read(fd_, &status, 1) != 1) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return std::unexpected(TYPES::DriverError::I2CReadFailed);
  }

  // Bit ZYXDA (0x08) : nouvelle mesure disponible sur les 3 axes.
  if ((status & 0x08) == 0) {
    return std::unexpected(TYPES::DriverError::NoNewData);
  }

  // 2. Lecture des 6 octets de données (OUT_X_L = 0x28, bit 0x80 =
  // auto-increment)
  std::uint8_t reg = 0x28 | 0x80;
  if (::write(fd_, &reg, 1) != 1) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return std::unexpected(TYPES::DriverError::I2CWriteFailed);
  }

  std::uint8_t data[6];
  if (::read(fd_, data, 6) != 6) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return std::unexpected(TYPES::DriverError::I2CReadFailed);
  }

  const auto rawX = static_cast<std::int16_t>(data[1] << 8 | data[0]);
  const auto rawY = static_cast<std::int16_t>(data[3] << 8 | data[2]);
  const auto rawZ = static_cast<std::int16_t>(data[5] << 8 | data[4]);

  MagData out{};
  out.x = rawX * kScale8G;
  out.y = rawY * kScale8G;
  out.z = rawZ * kScale8G;

  return out;
}