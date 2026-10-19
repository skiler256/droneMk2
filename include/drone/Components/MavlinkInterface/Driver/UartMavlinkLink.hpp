#pragma once
#include "drone/Components/MavlinkInterface/Driver.hpp"
#include <array>
#include <mutex>
#include <termios.h>

// Driver UART réel. send() et sendUrgent() partagent le même chemin
// d'écriture sous mutex — cf. MAVLINKINTERFACE.md §9.
class UartMavlinkLink : public IMavlinkLink {
public:
  explicit UartMavlinkLink(const char *portName, speed_t baudRate = B921600);
  ~UartMavlinkLink() override;

  UartMavlinkLink(const UartMavlinkLink &) = delete;
  UartMavlinkLink &operator=(const UartMavlinkLink &) = delete;

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  send(std::span<const std::byte> frame) override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  sendUrgent(std::span<const std::byte> frame) override;

private:
  [[nodiscard]] std::optional<TYPES::DriverError>
  writeLocked(std::span<const std::byte> frame);

  int fd_{-1};
  std::mutex writeMutex_;
  std::array<std::byte, 512> rxBuf_{}; // large marge : le plus gros message MAVLink v2 fait 280 octets
};
