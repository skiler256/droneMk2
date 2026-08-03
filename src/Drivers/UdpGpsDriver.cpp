#include "drone/Components/Drivers/UdpGpsDriver.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
constexpr std::array<char, 4> kMagic{'G', 'P', 'S', '1'};
constexpr uint8_t kVersion = 1;
constexpr std::size_t kHeaderSize = kMagic.size() + sizeof(kVersion);

// Ordre de champs figé — cf. sitl/gps_bridge.py (Phase B) pour le
// pendant Python. Pas de gap : GPSData::sats (vector) exclu du fil,
// cf. commentaire du header (pas nécessaire pour "SITL nu").
constexpr std::size_t kPayloadSize =
    sizeof(double) * 2 +               // latitude, longitude
    sizeof(int) * 6 +                  // timeArray
    sizeof(uint8_t) * 2 +              // gpsFixOk, fixType
    sizeof(uint32_t) * 3 +             // pAcc, sAcc, vAcc
    sizeof(float) * 2 +                // hdop, vdop
    sizeof(int32_t) +                  // altMslMm
    sizeof(uint32_t) +                 // iTowMs
    sizeof(int) * 3 +                  // velNED
    sizeof(uint32_t) * 2 +             // speed, groundSpeed
    sizeof(double);                    // heading
} // namespace

UdpGpsDriver::UdpGpsDriver(uint16_t localPort) : localPort_(localPort) {}

UdpGpsDriver::~UdpGpsDriver() = default; // fd_ fermé par ~IDriver()

std::optional<TYPES::DriverError> UdpGpsDriver::init() {
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

std::expected<GPSData, TYPES::DriverError> UdpGpsDriver::update() {
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

  GPSData data{};
  std::size_t off = kHeaderSize;
  auto take = [&](auto &field) {
    std::memcpy(&field, rxBuf_.data() + off, sizeof(field));
    off += sizeof(field);
  };

  take(data.coord.latitude);
  take(data.coord.longitude);
  take(data.timeArray);
  uint8_t gpsFixOk8{};
  take(gpsFixOk8);
  data.gpsFixOk = gpsFixOk8 != 0;
  take(data.fixType);
  take(data.pAcc);
  take(data.sAcc);
  take(data.vAcc);
  take(data.hdop);
  take(data.vdop);
  take(data.altMslMm);
  take(data.iTowMs);
  take(data.velNED);
  take(data.speed);
  take(data.groundSpeed);
  take(data.heading);

  return data;
}
