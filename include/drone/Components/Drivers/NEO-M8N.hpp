#pragma once

#include "drone/Components/SensorFusions/Driver.hpp"
#include <cstdint>

class NEO_M8N : public IDriver<GPSData> {
public:
  explicit NEO_M8N(const char *portName = "/dev/ttyAMA1");

  [[nodiscard]] std::optional<TYPES::DriverError> init() override;
  [[nodiscard]] std::expected<GPSData, TYPES::DriverError> update() override;

  // Trames rejetées sur checksum depuis le démarrage — une valeur qui grimpe
  // signale un câblage/blindage douteux plutôt qu'un bug logiciel.
  [[nodiscard]] std::uint32_t checksumErrors() const noexcept {
    return checksumErrors_;
  }

private:
  // Configuration, une seule fois dans init().
  [[nodiscard]] std::optional<TYPES::DriverError>
  sendConfigFrame(const std::uint8_t *frame, std::size_t len);
  [[nodiscard]] std::optional<TYPES::DriverError> configureGNSS();

  // Parsing UBX : avance d'un octet par appel, état persistant.
  void stepParser(std::uint8_t byte);
  void handleUBX(std::uint8_t cls, std::uint8_t id, std::uint16_t payloadSize);
  [[nodiscard]] int makeI4(int cursor) const;
  [[nodiscard]] std::uint32_t makeU4(int cursor) const;
  [[nodiscard]] std::uint16_t makeU2(int cursor) const;

  const char *portName_;

  enum class ParseState : std::uint8_t {
    SyncA = 1,
    SyncB,
    Class,
    Id,
    LenL,
    LenM,
    Payload,
    ChecksumA,
    ChecksumB
  };

  ParseState parseState_{ParseState::SyncA};
  std::uint8_t ubxClass_{0};
  std::uint8_t ubxId_{0};
  std::uint8_t lenL_{0};
  std::uint8_t lenM_{0};
  std::uint16_t payloadSize_{0};
  int payloadCursor_{0};

  // Fletcher 8 bits cumulé sur class/id/len/payload, comparé aux deux octets
  // de fin de trame.
  std::uint8_t ckA_{0};
  std::uint8_t ckB_{0};
  bool checksumOk_{false};
  std::uint32_t checksumErrors_{0}; // cumulé, exposé par checksumErrors()
  std::uint8_t payloadBuffer_[1024]{};

  GPSData data_{};
  bool newPacketDecoded_{
      false}; // reset à chaque update(), mis à true si un paquet exploité a été
              // décodé pendant cet appel
};