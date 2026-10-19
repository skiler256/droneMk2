#include "drone/Components/Drivers/NEO-M8N.hpp"

#include <algorithm>
#include <cerrno>
#include <unistd.h>

NEO_M8N::NEO_M8N(const char *portName) : portName_(portName) {}

std::optional<TYPES::DriverError> NEO_M8N::init() {
  // Démarre TOUJOURS à 9600 à froid : la config n'est pas persistée en EEPROM.
  if (auto err = uartConnect(portName_, B9600)) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return err;
  }

  // Trame UBX-CFG-PRT : demande au module de basculer lui-même sur 115200
  // bit/s. NE PAS MODIFIER — trame binaire figée (payload + checksum Fletcher).
  static constexpr std::uint8_t CFG_BAUDRATE[] = {
      0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
      0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00,
      0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E};

  if (auto err = sendConfigFrame(CFG_BAUDRATE, sizeof(CFG_BAUDRATE))) {
    return err;
  }

  // Le module a changé de vitesse : notre port série doit être fermé et
  // rouvert à 115200 pour rester synchronisé avec lui.
  ::close(fd_);
  fd_ = -1;

  if (auto err = uartConnect(portName_, B115200)) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return err;
  }

  if (auto err = configureGNSS()) {
    return err;
  }

  // update() est appelé périodiquement par la Task du composant : il ne doit
  // JAMAIS bloquer sur read(). On force donc un read() non bloquant.
  tty_.c_cc[VMIN] = 0;
  tty_.c_cc[VTIME] = 0;
  if (::tcsetattr(fd_, TCSANOW, &tty_) != 0) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return TYPES::DriverError::UARTAttrSetFailed;
  }

  setHealth(TYPES::DriverHealth::Connected);
  return std::nullopt;
}

std::optional<TYPES::DriverError>
NEO_M8N::sendConfigFrame(const std::uint8_t *frame, std::size_t len) {
  if (::write(fd_, frame, len) != static_cast<ssize_t>(len)) {
    return TYPES::DriverError::UARTWriteFailed;
  }
  if (::tcdrain(fd_) != 0) {
    return TYPES::DriverError::UARTWriteFailed;
  }
  return std::nullopt;
}

std::optional<TYPES::DriverError> NEO_M8N::configureGNSS() {
  // Toutes les trames ci-dessous sont recopiées à l'identique de l'ancien
  // driver — NE PAS MODIFIER les octets (payload + checksum UBX).

  static constexpr std::uint8_t CFG_GNSS[] = {
      0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08,
      0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01,
      0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10,
      0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01,
      0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E,
      0x00, 0x01, 0x00, 0x01, 0x01, 0x30, 0xAD};

  static constexpr std::uint8_t CFG_NAV5[] = {
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,
      0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
      0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x02, 0x3C, 0x00, 0x00, 0x00,
      0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0xCE};

  static constexpr std::uint8_t CFG_MSG_GGA_OFF[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
  static constexpr std::uint8_t CFG_MSG_GLL_OFF[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
  static constexpr std::uint8_t CFG_MSG_GSA_OFF[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
  static constexpr std::uint8_t CFG_MSG_GSV_OFF[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
  static constexpr std::uint8_t CFG_MSG_RMC_OFF[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
  static constexpr std::uint8_t CFG_MSG_VTG_OFF[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

  static constexpr std::uint8_t CFG_MSG_POSLLH_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE};
  static constexpr std::uint8_t CFG_MSG_STATUS_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5};
  static constexpr std::uint8_t CFG_MSG_VELNED_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x23, 0x2E};
  static constexpr std::uint8_t CFG_MSG_SOL_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x06,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDA};
  static constexpr std::uint8_t CFG_MSG_SVINFO_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x30,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x41, 0x00};
  static constexpr std::uint8_t CFG_MSG_TIMEUTC_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x21,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x32, 0x97};
  static constexpr std::uint8_t CFG_MSG_PVT_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};
  // Seule source de hdop/vdop ; checksum calculé, pas repris de l'ancien driver.
  static constexpr std::uint8_t CFG_MSG_DOP_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x04,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x15, 0xCC};

  static constexpr std::uint8_t CFG_RATE[] = {0xB5, 0x62, 0x06, 0x08, 0x06,
                                              0x00, 0xC8, 0x00, 0x01, 0x00,
                                              0x01, 0x00, 0xDE, 0x6A};

  const std::uint8_t *frames[] = {
      CFG_GNSS,          CFG_NAV5,           CFG_MSG_GGA_OFF,   CFG_MSG_GLL_OFF,
      CFG_MSG_GSA_OFF,   CFG_MSG_GSV_OFF,    CFG_MSG_RMC_OFF,   CFG_MSG_VTG_OFF,
      CFG_MSG_POSLLH_ON, CFG_MSG_STATUS_ON,  CFG_MSG_VELNED_ON, CFG_MSG_SOL_ON,
      CFG_MSG_SVINFO_ON, CFG_MSG_TIMEUTC_ON, CFG_MSG_PVT_ON,    CFG_MSG_DOP_ON,
      CFG_RATE};
  const std::size_t sizes[] = {
      sizeof(CFG_GNSS),          sizeof(CFG_NAV5),
      sizeof(CFG_MSG_GGA_OFF),   sizeof(CFG_MSG_GLL_OFF),
      sizeof(CFG_MSG_GSA_OFF),   sizeof(CFG_MSG_GSV_OFF),
      sizeof(CFG_MSG_RMC_OFF),   sizeof(CFG_MSG_VTG_OFF),
      sizeof(CFG_MSG_POSLLH_ON), sizeof(CFG_MSG_STATUS_ON),
      sizeof(CFG_MSG_VELNED_ON), sizeof(CFG_MSG_SOL_ON),
      sizeof(CFG_MSG_SVINFO_ON), sizeof(CFG_MSG_TIMEUTC_ON),
      sizeof(CFG_MSG_PVT_ON),    sizeof(CFG_MSG_DOP_ON),
      sizeof(CFG_RATE)};

  for (std::size_t i = 0; i < std::size(frames); ++i) {
    if (auto err = sendConfigFrame(frames[i], sizes[i])) {
      return err;
    }
  }

  return std::nullopt;
}

// --- Parsing (aucune boucle, aucun thread : un pas de state machine par octet
// lu) ---

std::expected<GPSData, TYPES::DriverError> NEO_M8N::update() {
  newPacketDecoded_ = false;

  std::uint8_t byte;
  ssize_t n;
  while ((n = ::read(fd_, &byte, 1)) > 0) {
    stepParser(byte);
  }

  if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    setHealth(TYPES::DriverHealth::Unconnected);
    return std::unexpected(TYPES::DriverError::UARTReadFailed);
  }

  if (!newPacketDecoded_) {
    return std::unexpected(TYPES::DriverError::NoNewData);
  }

  return data_;
}

void NEO_M8N::stepParser(std::uint8_t byte) {
  // Couvre class/id/len/payload : tout sauf sync et checksum.
  auto accumulate = [this](std::uint8_t b) {
    ckA_ = static_cast<std::uint8_t>(ckA_ + b);
    ckB_ = static_cast<std::uint8_t>(ckB_ + ckA_);
  };

  switch (parseState_) {
  case ParseState::SyncA:
    parseState_ = (byte == 0xB5) ? ParseState::SyncB : ParseState::SyncA;
    break;

  case ParseState::SyncB:
    parseState_ = (byte == 0x62) ? ParseState::Class : ParseState::SyncA;
    break;

  case ParseState::Class:
    ckA_ = 0;
    ckB_ = 0;
    accumulate(byte);
    ubxClass_ = byte;
    parseState_ = ParseState::Id;
    break;

  case ParseState::Id:
    accumulate(byte);
    ubxId_ = byte;
    parseState_ = ParseState::LenL;
    break;

  case ParseState::LenL:
    accumulate(byte);
    lenL_ = byte;
    parseState_ = ParseState::LenM;
    break;

  case ParseState::LenM:
    accumulate(byte);
    lenM_ = byte;
    payloadSize_ = static_cast<std::uint16_t>(lenL_ | (lenM_ << 8));
    if (payloadSize_ > sizeof(payloadBuffer_)) {
      parseState_ = ParseState::SyncA; // trame corrompue, on resynchronise
    } else {
      payloadCursor_ = 0;
      parseState_ = ParseState::Payload;
    }
    break;

  case ParseState::Payload:
    accumulate(byte);
    payloadBuffer_[payloadCursor_++] = byte;
    if (payloadCursor_ == payloadSize_) {
      parseState_ = ParseState::ChecksumA;
    }
    break;

  case ParseState::ChecksumA:
    // Les DEUX octets sont consommés : un CK_B valant 0xB5 ouvrait sinon une
    // trame fantôme et faisait perdre la suivante.
    checksumOk_ = (byte == ckA_);
    parseState_ = ParseState::ChecksumB;
    break;

  case ParseState::ChecksumB:
    // Trame corrompue ignorée mais comptée (cf. checksumErrors()).
    if (checksumOk_ && byte == ckB_) {
      handleUBX(ubxClass_, ubxId_, payloadSize_);
    } else {
      ++checksumErrors_;
    }
    parseState_ = ParseState::SyncA;
    break;
  }
}

void NEO_M8N::handleUBX(std::uint8_t cls, std::uint8_t id,
                        std::uint16_t payloadSize) {
  if (cls != 0x01) {
    return;
  }

  switch (id) {
  case 0x21: // NAV-TIMEUTC
    // bit2 du champ `valid` (offset 19) = validUTC. Sans lui, la date n'est
    // pas acquise et alimenterait une semaine GPS fausse (SENSORFUSION.md §3).
    if (payloadSize == 20 && (payloadBuffer_[19] & 0x04) != 0) {
      data_.timeArray[0] = (payloadBuffer_[13] << 8) | payloadBuffer_[12];
      data_.timeArray[1] = payloadBuffer_[14];
      data_.timeArray[2] = payloadBuffer_[15];
      data_.timeArray[3] = payloadBuffer_[16];
      data_.timeArray[4] = payloadBuffer_[17];
      data_.timeArray[5] = payloadBuffer_[18];
      newPacketDecoded_ = true;
    }
    break;

  case 0x03: // NAV-STATUS
    if (payloadSize == 16) {
      data_.fixType = payloadBuffer_[4];
      // 0x03 = fix 3D, 0x04 = GPS + dead reckoning. 0x05 ("time only") ne
      // porte AUCUNE position.
      data_.gpsFixOk =
          (payloadBuffer_[4] == 0x03 || payloadBuffer_[4] == 0x04);
      newPacketDecoded_ = true;
    }
    break;

  case 0x12: // NAV-VELNED
    if (payloadSize == 36) {
      data_.velNED[0] = makeI4(4);
      data_.velNED[1] = makeI4(8);
      data_.velNED[2] = makeI4(12);
      data_.speed = makeU4(16);
      data_.groundSpeed = makeU4(20);
      data_.heading = makeI4(24) * 1e-5;
      newPacketDecoded_ = true;
    }
    break;

  case 0x02: // NAV-POSLLH
    if (payloadSize == 28) {
      data_.coord.longitude = makeI4(4) * 1e-7;
      data_.coord.latitude = makeI4(8) * 1e-7;
      data_.altMslMm = makeI4(16); // hMSL, mm (offset 12 = height ellipsoïde, ignoré)
      data_.vAcc = makeU4(24);     // mm (offset 20 = hAcc, déjà repris depuis NAV-PVT.pAcc)
      newPacketDecoded_ = true;
    }
    break;

  case 0x04: // NAV-DOP (gDOP/pDOP/tDOP non utilisés)
    if (payloadSize == 18) {
      data_.vdop = static_cast<float>(makeU2(10)) * 0.01f;
      data_.hdop = static_cast<float>(makeU2(12)) * 0.01f;
      newPacketDecoded_ = true;
    }
    break;

  case 0x30: { // NAV-SVINFO
    // numCh vient du payload : valider la taille annoncée évite une lecture
    // très au-delà du tampon. Offsets par canal : SENSORFUSION.md §3.
    const std::uint8_t numCh = payloadBuffer_[4];
    if (payloadSize != 8u + 12u * numCh) {
      break; // longueur annoncée incompatible avec numCh
    }

    // Le compte suivi n'est PAS plafonné, seul le détail l'est.
    data_.satellitesTracked = numCh;
    const std::size_t kept =
        std::min<std::size_t>(numCh, GPSData::kMaxSatDetail);
    data_.sats = {};
    for (std::size_t i = 0; i < kept; ++i) {
      data_.sats[i] = {payloadBuffer_[9 + 12 * i],   // svid
                       payloadBuffer_[12 + 12 * i],  // cno (force du signal)
                       payloadBuffer_[11 + 12 * i]}; // quality
    }
    data_.satDetailCount = static_cast<std::uint8_t>(kept);
    newPacketDecoded_ = true;
    break;
  }

  case 0x07: // NAV-PVT
    if (payloadSize == 92) {
      data_.iTowMs = makeU4(0);
      data_.pAcc = makeU4(40);
      data_.sAcc = makeU4(68);
      newPacketDecoded_ = true;
    }
    break;

  default:
    break; // ex: 0x06 (NAV-SOL) activé sur le module mais non parsé —
           // comportement d'origine préservé
  }
}

int NEO_M8N::makeI4(int cursor) const {
  return static_cast<int32_t>(
      (static_cast<std::uint32_t>(payloadBuffer_[cursor])) |
      (static_cast<std::uint32_t>(payloadBuffer_[cursor + 1]) << 8) |
      (static_cast<std::uint32_t>(payloadBuffer_[cursor + 2]) << 16) |
      (static_cast<std::uint32_t>(payloadBuffer_[cursor + 3]) << 24));
}

std::uint32_t NEO_M8N::makeU4(int cursor) const {
  return static_cast<std::uint32_t>(
      (static_cast<std::uint32_t>(payloadBuffer_[cursor])) |
      (static_cast<std::uint32_t>(payloadBuffer_[cursor + 1]) << 8) |
      (static_cast<std::uint32_t>(payloadBuffer_[cursor + 2]) << 16) |
      (static_cast<std::uint32_t>(payloadBuffer_[cursor + 3]) << 24));
}

std::uint16_t NEO_M8N::makeU2(int cursor) const {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(payloadBuffer_[cursor])) |
      (static_cast<std::uint16_t>(payloadBuffer_[cursor + 1]) << 8));
}