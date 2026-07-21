#pragma once
#include "drone/Components/System Monitoring/Driver.hpp"
#include <array>
#include <cstdint>

// Driver factice de dev pour la vidéo : reçoit des frames MJPEG brutes en
// UDP sur loopback (une frame = un datagramme), à la place du vrai lien
// WFB-NG qui encapsule en RTP. VideoTask ne voit jamais la différence :
// IVideoSource::pollFrame() renvoie du JPEG brut dans les deux cas.
class UdpVideoDriver : public IVideoSource {
public:
  explicit UdpVideoDriver(uint16_t localPort);
  ~UdpVideoDriver() override;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  pollFrame() override;

private:
  int fd_{-1};
  // Marge large pour une frame JPEG basse résolution ; un vrai flux plus
  // gros nécessiterait un découpage en plusieurs paquets, pas géré ici.
  std::array<std::byte, 65000> rxBuf_{};
};
