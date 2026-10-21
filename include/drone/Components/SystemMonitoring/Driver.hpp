#pragma once
#include "drone/types.hpp"
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

// Un rôle = une interface ; drivers purement I/O, sans thread propre —
// cf. SYSMONITORING.md §7.

// Santé observable d'un lien, pour le paquet LINK_QUALITY.
//
// Les champs sont optionnels parce qu'ils ne sont pas tous mesurables sur
// tous les transports : un socket UDP n'a pas de RSSI, et wfb-ng ne compte
// pas de CRC faux. Un optional vide signifie « non mesurable ici », ce qui
// n'est pas la même chose que zéro.
struct LinkQuality {
  std::optional<int8_t> rssiDbm;
  std::optional<uint8_t> lqi;
  std::optional<float> lossRatio; // 0..1
  uint32_t crcErrors{0};
  uint32_t lengthErrors{0};
  uint32_t overflows{0};
  uint32_t ccaRefusals{0};
};

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

  // Émet sans s'effacer devant une occupation du canal, pour les paquets
  // marqués AckMode::ForceNoCca — ceux où rester muet est pire qu'une
  // collision (arrêt d'urgence).
  //
  // L'implémentation par défaut appelle send() : sur un transport sans
  // contrainte d'accès au média — UDP, wfb-ng — « forcer » n'a aucun sens
  // différent d'« émettre », et il n'y a rien à contourner. Seuls les liens
  // qui écoutent avant d'émettre ont à redéfinir cette méthode.
  [[nodiscard]] virtual std::optional<TYPES::DriverError>
  sendForced(std::span<const std::byte> payload) {
    return send(payload);
  }

  // Santé mesurable du lien. Défaut : rien de mesurable, ce qui est la
  // réponse honnête pour un transport qui n'expose aucune métrique. Permet à
  // SysMonitoring d'assembler LINK_QUALITY sans connaître le type concret du
  // lien — même raison que sendForced().
  [[nodiscard]] virtual LinkQuality quality() const { return {}; }

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
