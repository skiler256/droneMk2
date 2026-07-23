#include "drone/Components/MavlinkInterface/Driver/UartMavlinkLink.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

UartMavlinkLink::UartMavlinkLink(const char *portName, speed_t baudRate) {
  int fd = ::open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return;
  }

  termios tty{};
  if (::tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    setHealth(TYPES::DriverHealth::Unconnected);
    return;
  }

  cfsetispeed(&tty, baudRate);
  cfsetospeed(&tty, baudRate);
  cfmakeraw(&tty); // 8N1, pas de contrôle de flux, mode brut

  // Non bloquant : poll() est appelé depuis la boucle RT de TRx, elle ne
  // doit jamais attendre un octet qui n'arrive pas (O_NONBLOCK à l'open
  // suffit déjà, VMIN/VTIME à 0 pour ne pas dépendre du canonical mode).
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    setHealth(TYPES::DriverHealth::Unconnected);
    return;
  }

  fd_ = fd;
  setHealth(TYPES::DriverHealth::Connected);
}

UartMavlinkLink::~UartMavlinkLink() {
  if (fd_ >= 0)
    ::close(fd_);
}

std::expected<std::span<const std::byte>, TYPES::DriverError>
UartMavlinkLink::poll() {
  if (fd_ < 0)
    return std::unexpected(TYPES::DriverError::NotInitialized);

  ssize_t n = ::read(fd_, rxBuf_.data(), rxBuf_.size());
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return std::unexpected(TYPES::DriverError::NoNewData);
    return std::unexpected(TYPES::DriverError::UARTReadFailed);
  }
  if (n == 0)
    return std::unexpected(TYPES::DriverError::NoNewData);

  return std::span<const std::byte>(rxBuf_.data(), static_cast<size_t>(n));
}

std::optional<TYPES::DriverError>
UartMavlinkLink::writeLocked(std::span<const std::byte> frame) {
  if (fd_ < 0)
    return TYPES::DriverError::NotInitialized;

  std::lock_guard<std::mutex> lock(writeMutex_);

  ssize_t n = ::write(fd_, frame.data(), frame.size());
  if (n < 0 || static_cast<size_t>(n) != frame.size())
    return TYPES::DriverError::UARTWriteFailed;

  return std::nullopt;
}

std::optional<TYPES::DriverError>
UartMavlinkLink::send(std::span<const std::byte> frame) {
  return writeLocked(frame);
}

std::optional<TYPES::DriverError>
UartMavlinkLink::sendUrgent(std::span<const std::byte> frame) {
  return writeLocked(frame);
}
