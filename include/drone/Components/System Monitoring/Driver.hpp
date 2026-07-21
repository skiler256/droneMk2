#pragma once
#include "drone/types.hpp"
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

// Interfaces des liens de communication de SysMonitoring — un rôle = une
// interface, pas d'interface unique pour tout (vidéo/télémétrie/commandes
// ont des besoins trop différents). Comme IDriver (SensorFusions), aucun
// driver concret ne possède de thread : il est purement I/O passif, appelé
// depuis la boucle RT de la Task qui le possède (TelMainTask/TelSecTask/
// VideoTask/GCSRxTask).
//
// Pensées pour être portables d'un véhicule à l'autre : ce qui change entre
// projets, c'est l'implémentation concrète (WFB_NG_Driver, CC1101_Driver,
// ...), jamais ces interfaces.

// Lien télémétrie bidirectionnel générique (metric + commandes, format
// TELEM::* packé — voir include/drone/generated/telemetry.hpp).
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

// Source vidéo : uniquement descendant, une frame déjà décapsulée (JPEG
// brut) — le driver concret cache le protocole de transport (RTP, ou UDP
// brut pour le driver factice de dev) derrière cette seule méthode.
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
