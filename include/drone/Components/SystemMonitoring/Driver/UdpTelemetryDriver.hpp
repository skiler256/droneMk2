#pragma once
#include "drone/Components/SystemMonitoring/Driver.hpp"
#include <array>
#include <cstdint>

// Driver de dev : TELEM::* en UDP brut sur loopback à la place d'un lien
// radio réel, pour valider le pipeline sans hardware RF. Sans garantie de
// livraison, comme un vrai lien lossy. Sert à TelMain et TelSec, seuls les
// ports diffèrent.
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
  // Couvre le plus gros paquet TELEM (LIDAR_SCAN, ~1085o) + le header,
  // marge jusqu'au MTU Ethernet.
  std::array<std::byte, 1536> rxBuf_{};
};
