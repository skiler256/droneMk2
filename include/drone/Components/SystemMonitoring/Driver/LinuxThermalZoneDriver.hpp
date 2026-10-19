#pragma once
#include "drone/Components/SystemMonitoring/Driver.hpp"

// Lit /sys/class/thermal/<zone>/temp (API kernel standard, pas spécifique
// Pi5) : même driver en dev et en prod, aucune variante factice.
class LinuxThermalZoneDriver : public ISystemTempSource {
public:
  explicit LinuxThermalZoneDriver(const char *zonePath = "/sys/class/thermal/thermal_zone0/temp");

  [[nodiscard]] std::expected<float, TYPES::DriverError> readCelsius() override;

private:
  const char *zonePath_;
};
