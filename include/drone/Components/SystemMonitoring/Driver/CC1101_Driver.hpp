#pragma once
#include "drone/Components/SystemMonitoring/Driver.hpp"

// Lien secondaire SDL (CC1101, sub-GHz SPI) : sous-ensemble Links=BOTH.
// PAS ENCORE IMPLÉMENTÉ — squelette seul, ne pas ajouter au build tant que
// le .cpp n'existe pas.
class CC1101_Driver : public ITelemetryLink {
public:
  CC1101_Driver() = default;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  send(std::span<const std::byte> payload) override;
};
