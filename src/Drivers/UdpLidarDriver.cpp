#include "drone/Components/Drivers/UdpLidarDriver.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
constexpr std::array<char, 4> kMagic{'L', 'I', 'D', '1'};
constexpr uint8_t kVersion = 1;
constexpr std::size_t kHeaderSize = kMagic.size() + sizeof(kVersion);
constexpr std::size_t kPayloadSize = 360 * sizeof(uint16_t) + 360 * sizeof(uint8_t);
} // namespace

UdpLidarDriver::UdpLidarDriver(uint16_t localPort) : localPort_(localPort) {}

UdpLidarDriver::~UdpLidarDriver() = default; // fd_ fermé par ~IDriver()

std::optional<TYPES::DriverError> UdpLidarDriver::init() {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return TYPES::DriverError::SocketOpenFailed;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(localPort_);

  if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    fd_ = -1;
    setHealth(TYPES::DriverHealth::Unconnected);
    return TYPES::DriverError::SocketBindFailed;
  }

  int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

  setHealth(TYPES::DriverHealth::Connected);
  return std::nullopt;
}

std::expected<LidarData, TYPES::DriverError> UdpLidarDriver::update() {
  if (fd_ < 0)
    return std::unexpected(TYPES::DriverError::NotInitialized);

  ssize_t n = ::recv(fd_, rxBuf_.data(), rxBuf_.size(), 0);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return std::unexpected(TYPES::DriverError::NoNewData);
    return std::unexpected(TYPES::DriverError::SocketReadFailed);
  }

  auto received = static_cast<std::size_t>(n);
  if (received < kHeaderSize + kPayloadSize)
    return std::unexpected(TYPES::DriverError::InvalidData);

  if (std::memcmp(rxBuf_.data(), kMagic.data(), kMagic.size()) != 0)
    return std::unexpected(TYPES::DriverError::InvalidData);
  if (static_cast<uint8_t>(rxBuf_[kMagic.size()]) != kVersion)
    return std::unexpected(TYPES::DriverError::InvalidData);

  LidarData data{};
  std::size_t off = kHeaderSize;
  std::memcpy(data.distance_mm.data(), rxBuf_.data() + off, data.distance_mm.size() * sizeof(uint16_t));
  off += data.distance_mm.size() * sizeof(uint16_t);
  std::memcpy(data.intensity.data(), rxBuf_.data() + off, data.intensity.size() * sizeof(uint8_t));

  return data;
}
