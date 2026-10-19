#pragma once
#include "drone/types.hpp"
#include <cstddef>
#include <expected>
#include <optional>
#include <span>

// Lien MAVLink vers le FC : purement I/O, sans thread propre. Injecté par
// interface pour pouvoir brancher un driver SITL sans toucher au composant.
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

  // Réservé à FSTask : contourne la file d'attente, pas le verrou d'écriture
  // — priorité procédurale, cf. MAVLINKINTERFACE.md §9.
  [[nodiscard]] virtual std::optional<TYPES::DriverError>
  sendUrgent(std::span<const std::byte> frame) = 0;

  [[nodiscard]] TYPES::DriverHealth getHealth() const { return health_; }

protected:
  void setHealth(TYPES::DriverHealth h) { health_ = h; }

private:
  TYPES::DriverHealth health_{TYPES::DriverHealth::Unconnected};
};
