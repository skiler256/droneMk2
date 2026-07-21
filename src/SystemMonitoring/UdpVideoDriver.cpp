#include "drone/Components/System Monitoring/Driver/UdpVideoDriver.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

UdpVideoDriver::UdpVideoDriver(uint16_t localPort) {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
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

UdpVideoDriver::~UdpVideoDriver() {
  if (fd_ >= 0)
    ::close(fd_);
}

std::expected<std::span<const std::byte>, TYPES::DriverError>
UdpVideoDriver::pollFrame() {
  if (fd_ < 0)
    return std::unexpected(TYPES::DriverError::NotInitialized);

  ssize_t n = ::recv(fd_, rxBuf_.data(), rxBuf_.size(), 0);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return std::unexpected(TYPES::DriverError::NoNewData);
    return std::unexpected(TYPES::DriverError::SocketReadFailed);
  }

  return std::span<const std::byte>(rxBuf_.data(), static_cast<size_t>(n));
}
