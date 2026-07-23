#pragma once
#include "drone/types.hpp"
#include <cstddef>
#include <expected>
#include <optional>
#include <span>

// Interface du lien MAVLink (UART vers le FC) — driver purement I/O, pas
// de thread (même principe que ITelemetryLink de SysMonitoring), appelé
// depuis les Tasks (TTx/TTxACK/TRx/FSTask) qui possèdent leur propre
// boucle RT. Injectée par référence dans MavlinkInterface, jamais un type
// concret : permet de brancher un driver simulé (SITL) sans toucher au
// composant.
class IMavlinkLink {
public:
  IMavlinkLink() = default;
  virtual ~IMavlinkLink() = default;

  IMavlinkLink(const IMavlinkLink &) = delete;
  IMavlinkLink &operator=(const IMavlinkLink &) = delete;
  IMavlinkLink(IMavlinkLink &&) = delete;
  IMavlinkLink &operator=(IMavlinkLink &&) = delete;

  // Non bloquant : renvoie DriverError::NoNewData si rien n'est disponible.
  [[nodiscard]] virtual std::expected<std::span<const std::byte>,
                                      TYPES::DriverError>
  poll() = 0;

  // Envoi normal — passe par le scheduler de TTx/TTxACK côté appelant.
  [[nodiscard]] virtual std::optional<TYPES::DriverError>
  send(std::span<const std::byte> frame) = 0;

  // Envoi prioritaire : contourne volontairement tout ordonnancement,
  // réservé à FSTask (cf. MAVLINKINTERFACE.md §8). Protégé par le même
  // verrou interne que send() — thread-safe, mais aucune mise en attente.
  [[nodiscard]] virtual std::optional<TYPES::DriverError>
  sendUrgent(std::span<const std::byte> frame) = 0;

  [[nodiscard]] TYPES::DriverHealth getHealth() const { return health_; }

protected:
  void setHealth(TYPES::DriverHealth h) { health_ = h; }

private:
  TYPES::DriverHealth health_{TYPES::DriverHealth::Unconnected};
};
