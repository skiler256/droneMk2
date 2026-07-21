#pragma once
#include "drone/Components/System Monitoring/Driver.hpp"
#include <array>
#include <cstdint>

// Driver factice de dev : transporte les paquets TELEM::* en UDP brut sur
// loopback, à la place d'un vrai lien radio (WFB-NG/CC1101). Sert à valider
// tout le pipeline SysMonitoring <-> GlobalWatchdog <-> shm sans attendre le
// hardware RF — pas de garantie de livraison (comme un vrai lien lossy),
// pas de dépendance externe (sockets POSIX seulement).
//
// Réutilisable tel quel pour TelMain et TelSec : seuls les ports diffèrent.
class UdpTelemetryDriver : public ITelemetryLink {
public:
  UdpTelemetryDriver(uint16_t localPort, uint16_t peerPort);
  ~UdpTelemetryDriver() override;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  send(std::span<const std::byte> payload) override;

private:
  int fd_{-1};
  uint16_t peerPort_;
  std::array<std::byte, 512> rxBuf_{}; // large marge : le plus gros paquet TELEM fait 20 octets
};
