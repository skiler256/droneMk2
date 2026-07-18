#pragma once

#include "drone/Components/SensorFusions/Driver.hpp"
#include <cstdint>

class NEO_M8N : public IDriver<GPSData> {
public:
  explicit NEO_M8N(const char *portName = "/dev/ttyAMA1");

  [[nodiscard]] std::optional<TYPES::DriverError> init() override;
  [[nodiscard]] std::expected<GPSData, TYPES::DriverError> update() override;

private:
  // --- Configuration (exécutée une seule fois dans init()) ---
  [[nodiscard]] std::optional<TYPES::DriverError>
  sendConfigFrame(const std::uint8_t *frame, std::size_t len);
  [[nodiscard]] std::optional<TYPES::DriverError> configureGNSS();

  // --- Parsing UBX (avance d'un octet à chaque appel, état persistant) ---
  void stepParser(std::uint8_t byte);
  void handleUBX(std::uint8_t cls, std::uint8_t id, std::uint16_t payloadSize);
  [[nodiscard]] int makeI4(int cursor) const;
  [[nodiscard]] std::uint32_t makeU4(int cursor) const;

  const char *portName_;

  enum class ParseState : std::uint8_t {
    SyncA = 1,
    SyncB,
    Class,
    Id,
    LenL,
    LenM,
    Payload,
    Checksum
  };

  ParseState parseState_{ParseState::SyncA};
  std::uint8_t ubxClass_{0};
  std::uint8_t ubxId_{0};
  std::uint8_t lenL_{0};
  std::uint8_t lenM_{0};
  std::uint16_t payloadSize_{0};
  int payloadCursor_{0};
  std::uint8_t payloadBuffer_[1024]{};

  GPSData data_{};
  bool newPacketDecoded_{
      false}; // reset à chaque update(), mis à true si un paquet exploité a été
              // décodé pendant cet appel
};