#pragma once
#include "drone/Components/System Monitoring/Driver.hpp"

// Lien secondaire de secours (CC1101, sub-GHz SPI, bas débit) : ne porte
// que le sous-ensemble léger de TELEMETRY.csv (Links=BOTH).
// PAS ENCORE IMPLÉMENTÉ — squelette d'interface seulement, cf. discussion
// archi (accès hardware traité séparément). Ne pas ajouter au build tant
// que le .cpp n'existe pas.
class CC1101_Driver : public ITelemetryLink {
public:
  CC1101_Driver() = default;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  send(std::span<const std::byte> payload) override;
};
