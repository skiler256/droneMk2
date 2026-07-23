#include "drone/Components/MavlinkInterface/Driver/UdpMavlinkLink.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

UdpMavlinkLink::UdpMavlinkLink(uint16_t localPort) {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY); // SITL arrive depuis une autre machine (WSL2)
  addr.sin_port = htons(localPort);

  if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    fd_ = -1;
    setHealth(TYPES::DriverHealth::Unconnected);
    return;
  }

  int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

  setHealth(TYPES::DriverHealth::Connected);
}

UdpMavlinkLink::~UdpMavlinkLink() {
  if (fd_ >= 0)
    ::close(fd_);
}

std::expected<std::span<const std::byte>, TYPES::DriverError> UdpMavlinkLink::poll() {
  if (fd_ < 0)
    return std::unexpected(TYPES::DriverError::NotInitialized);

  sockaddr_in from{};
  socklen_t fromLen = sizeof(from);
  ssize_t n = ::recvfrom(fd_, rxBuf_.data(), rxBuf_.size(), 0,
                        reinterpret_cast<sockaddr *>(&from), &fromLen);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return std::unexpected(TYPES::DriverError::NoNewData);
    return std::unexpected(TYPES::DriverError::SocketReadFailed);
  }

  // Pair appris/mis à jour à chaque paquet reçu — SITL peut changer de
  // port source entre deux lancements (voire en cours de session).
  {
    std::lock_guard<std::mutex> lock(writeMutex_);
    peerAddr_ = from;
    havePeer_ = true;
  }

  return std::span<const std::byte>(rxBuf_.data(), static_cast<size_t>(n));
}

std::optional<TYPES::DriverError>
UdpMavlinkLink::sendToPeer(std::span<const std::byte> frame) {
  if (fd_ < 0)
    return TYPES::DriverError::NotInitialized;

  std::lock_guard<std::mutex> lock(writeMutex_);
  if (!havePeer_)
    return TYPES::DriverError::NotInitialized; // rien reçu de SITL encore, pair inconnu

  ssize_t n = ::sendto(fd_, frame.data(), frame.size(), 0,
                       reinterpret_cast<sockaddr *>(&peerAddr_), sizeof(peerAddr_));
  if (n < 0 || static_cast<size_t>(n) != frame.size())
    return TYPES::DriverError::SocketWriteFailed;

  return std::nullopt;
}

std::optional<TYPES::DriverError>
UdpMavlinkLink::send(std::span<const std::byte> frame) {
  return sendToPeer(frame);
}

std::optional<TYPES::DriverError>
UdpMavlinkLink::sendUrgent(std::span<const std::byte> frame) {
  return sendToPeer(frame);
}
