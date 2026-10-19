#pragma once
#include "drone/types.hpp"
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

// Un rôle = une interface ; drivers purement I/O, sans thread propre —
// cf. SYSMONITORING.md §7.

// Lien bidirectionnel, format TELEM::* packé (generated/telemetry.hpp).
class ITelemetryLink {
public:
  ITelemetryLink() = default;
  virtual ~ITelemetryLink() = default;

  ITelemetryLink(const ITelemetryLink &) = delete;
  ITelemetryLink &operator=(const ITelemetryLink &) = delete;
  ITelemetryLink(ITelemetryLink &&) = delete;
  ITelemetryLink &operator=(ITelemetryLink &&) = delete;

  // Non bloquant : renvoie DriverError::NoNewData si rien n'est disponible.
  [[nodiscard]] virtual std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() = 0;

  [[nodiscard]] virtual std::optional<TYPES::DriverError>
  send(std::span<const std::byte> payload) = 0;

  [[nodiscard]] TYPES::DriverHealth getHealth() const { return health_; }

protected:
  void setHealth(TYPES::DriverHealth h) { health_ = h; }

private:
  TYPES::DriverHealth health_{TYPES::DriverHealth::Unconnected};
};

// Descendant seul, frame déjà décapsulée (JPEG brut) : le transport (RTP
// ou UDP de dev) reste caché derrière cette méthode.
class IVideoSource {
public:
  IVideoSource() = default;
  virtual ~IVideoSource() = default;

  IVideoSource(const IVideoSource &) = delete;
  IVideoSource &operator=(const IVideoSource &) = delete;
  IVideoSource(IVideoSource &&) = delete;
  IVideoSource &operator=(IVideoSource &&) = delete;

  [[nodiscard]] virtual std::expected<std::span<const std::byte>, TYPES::DriverError>
  pollFrame() = 0;

  [[nodiscard]] TYPES::DriverHealth getHealth() const { return health_; }

protected:
  void setHealth(TYPES::DriverHealth h) { health_ = h; }

private:
  TYPES::DriverHealth health_{TYPES::DriverHealth::Unconnected};
};

// Pas de getHealth() : lecture fichier ponctuelle, aucun lien à suivre —
// un échec est un DriverError, pas un état. Cf. SYSMONITORING.md §12.
class ISystemTempSource {
public:
  ISystemTempSource() = default;
  virtual ~ISystemTempSource() = default;

  ISystemTempSource(const ISystemTempSource &) = delete;
  ISystemTempSource &operator=(const ISystemTempSource &) = delete;
  ISystemTempSource(ISystemTempSource &&) = delete;
  ISystemTempSource &operator=(ISystemTempSource &&) = delete;

  [[nodiscard]] virtual std::expected<float, TYPES::DriverError> readCelsius() = 0;
};
