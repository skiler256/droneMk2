#pragma once
#include "drone/Components/System Monitoring/Driver.hpp"

// Lien principal (WFB-NG, wifi broadcast) : télémétrie/commandes + vidéo.
// PAS ENCORE IMPLÉMENTÉ — squelette d'interface seulement, cf. discussion
// archi (accès hardware traité séparément). Ne pas ajouter au build tant
// que le .cpp n'existe pas.
class WFB_NG_Driver : public ITelemetryLink, public IVideoSource {
public:
  WFB_NG_Driver() = default;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  send(std::span<const std::byte> payload) override;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  pollFrame() override;
};
