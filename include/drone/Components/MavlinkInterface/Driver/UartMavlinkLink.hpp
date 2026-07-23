#pragma once
#include "drone/Components/MavlinkInterface/Driver.hpp"
#include <array>
#include <mutex>
#include <termios.h>

// Driver UART réel vers le FC. send()/sendUrgent() passent tous les deux
// par le même chemin d'écriture protégé par mutex — la "priorité" de
// sendUrgent() est purement procédurale (FSTask l'appelle directement,
// sans passer par le scheduler de TTxACK), pas un mécanisme OS particulier
// : un seul fil UART ne transporte de toute façon qu'un frame à la fois.
// Cf. MAVLINKINTERFACE.md §8.
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
