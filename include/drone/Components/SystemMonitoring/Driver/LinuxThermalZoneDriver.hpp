#pragma once
#include "drone/Components/SystemMonitoring/Driver.hpp"

// Lit la température CPU depuis /sys/class/thermal/<zone>/temp (API kernel
// Linux standard, pas spécifique Pi5) — même driver en dev et en prod,
// contrairement à ITelemetryLink/IVideoSource : aucune raison de le
// factice-ifier, un fichier thermal_zone existe sur toute machine Linux
// avec capteur (dev x86 compris).
class LinuxThermalZoneDriver : public ISystemTempSource {
public:
  explicit LinuxThermalZoneDriver(const char *zonePath = "/sys/class/thermal/thermal_zone0/temp");

  [[nodiscard]] std::expected<float, TYPES::DriverError> readCelsius() override;

private:
  const char *zonePath_;
};
