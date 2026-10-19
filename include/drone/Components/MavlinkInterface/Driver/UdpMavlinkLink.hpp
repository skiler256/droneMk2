#pragma once
#include "drone/Components/MavlinkInterface/Driver.hpp"
#include <array>
#include <mutex>
#include <netinet/in.h>

// Lien MAVLink vers ArduPilot SITL en UDP, à la place de l'UART réel. Le
// port source de SITL étant éphémère, le pair est appris au premier paquet
// reçu plutôt que configuré — cf. MAVLINKINTERFACE.md, section SITL.
class UdpMavlinkLink : public IMavlinkLink {
public:
  explicit UdpMavlinkLink(uint16_t localPort);
  ~UdpMavlinkLink() override;

  UdpMavlinkLink(const UdpMavlinkLink &) = delete;
  UdpMavlinkLink &operator=(const UdpMavlinkLink &) = delete;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  send(std::span<const std::byte> frame) override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  sendUrgent(std::span<const std::byte> frame) override;

private:
  [[nodiscard]] std::optional<TYPES::DriverError>
  sendToPeer(std::span<const std::byte> frame);

  int fd_{-1};
  std::mutex writeMutex_;

  sockaddr_in peerAddr_{};
  bool havePeer_{false};

  std::array<std::byte, 512> rxBuf_{};
};
