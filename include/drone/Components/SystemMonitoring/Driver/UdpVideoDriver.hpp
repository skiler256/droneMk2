#pragma once
#include "drone/Components/SystemMonitoring/Driver.hpp"
#include <array>
#include <cstdint>

// Driver de dev : une frame MJPEG = un datagramme UDP loopback, là où
// WFB-NG encapsulera en RTP. pollFrame() renvoie du JPEG brut des deux
// côtés, VideoTask ne voit pas la différence.
class UdpVideoDriver : public IVideoSource {
public:
  explicit UdpVideoDriver(uint16_t localPort);
  ~UdpVideoDriver() override;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  pollFrame() override;

private:
  int fd_{-1};
  // Suffisant pour du JPEG basse résolution ; pas de découpage multi-paquets.
  std::array<std::byte, 65000> rxBuf_{};
};
