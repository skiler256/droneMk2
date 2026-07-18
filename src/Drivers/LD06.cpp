#include "drone/Components/Drivers/LD06.hpp"

#include <cmath>
#include <unistd.h>

namespace {

// Table CRC8 standard du protocole LD06/LD19 (polynôme 0x4D) — constante
// publique du protocole capteur, publiée par le fabricant (LDROBOT).
constexpr std::uint8_t kCrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};

} // namespace

LD06::LD06(const char* portName) : portName_(portName) {}

std::optional<TYPES::DriverError> LD06::init() {
    if (auto err = uartConnect(portName_, B230400)) {
        setHealth(TYPES::DriverHealth::Unconnected);
        return err;
    }

    // Lecture non bloquante : update() est appelé périodiquement par la Task,
    // il ne doit jamais rester en attente sur read().
    tty_.c_cc[VMIN] = 0;
    tty_.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &tty_) != 0) {
        setHealth(TYPES::DriverHealth::Unconnected);
        return TYPES::DriverError::UARTAttrSetFailed;
    }

    setHealth(TYPES::DriverHealth::Connected);
    return std::nullopt;
}

std::uint8_t LD06::crc8(const std::uint8_t* data, std::size_t len) {
    std::uint8_t crc = 0;
    for (std::size_t i = 0; i < len; ++i) {
        crc = kCrcTable[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

std::expected<LidarData, TYPES::DriverError> LD06::update() {
    crcErrorSeen_ = false;

    std::uint8_t byte;
    ssize_t n;
    while ((n = ::read(fd_, &byte, 1)) > 0) {
        switch (parseState_) {
            case ParseState::WaitHeader:
                if (byte == 0x54) {
                    packetBuffer_[0] = byte;
                    packetCursor_ = 1;
                    parseState_ = ParseState::WaitVerLen;
                }
                break;

            case ParseState::WaitVerLen:
                if (byte == 0x2C) {
                    packetBuffer_[1] = byte;
                    packetCursor_ = 2;
                    parseState_ = ParseState::Filling;
                } else if (byte == 0x54) {
                    packetCursor_ = 1;
                } else {
                    parseState_ = ParseState::WaitHeader;
                }
                break;

            case ParseState::Filling:
                packetBuffer_[packetCursor_++] = byte;
                if (packetCursor_ == kPacketSize) {
                    if (!processPacket()) {
                        crcErrorSeen_ = true;
                    }
                    packetCursor_ = 0;
                    parseState_ = ParseState::WaitHeader;
                }
                break;
        }
    }

    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        setHealth(TYPES::DriverHealth::Unconnected);
        return std::unexpected(TYPES::DriverError::UARTReadFailed);
    }

    if (rotationJustCompleted_) {
        rotationJustCompleted_ = false;
        return lastCompletedScan_;
    }

    if (crcErrorSeen_) {
        return std::unexpected(TYPES::DriverError::InvalidData);
    }

    return std::unexpected(TYPES::DriverError::NoNewData);
}

bool LD06::processPacket() {
    const std::uint8_t receivedCrc = packetBuffer_[kPacketSize - 1];
    const std::uint8_t computedCrc = crc8(packetBuffer_, kPacketSize - 1);
    if (receivedCrc != computedCrc) {
        return false;
    }

    const std::uint16_t startRaw = static_cast<std::uint16_t>(packetBuffer_[4] | (packetBuffer_[5] << 8));
    const std::uint16_t endRaw   = static_cast<std::uint16_t>(packetBuffer_[42] | (packetBuffer_[43] << 8));

    double startDeg = startRaw / 100.0;
    double endDeg = endRaw / 100.0;
    if (endDeg < startDeg) {
        endDeg += 360.0; // le paquet traverse 0°
    }
    const double stepDeg = (endDeg - startDeg) / (kPointsPerPacket - 1);

    // Détection de fin de tour : le nouvel angle de départ "revient en arrière"
    // par rapport au précédent (ex: 358° -> 3°).
    if (lastStartAngleDeg_ >= 0.0 && startDeg < lastStartAngleDeg_) {
        lastCompletedScan_ = accumulator_;
        rotationJustCompleted_ = true;
        accumulator_ = LidarData{}; // remise à zéro pour le tour suivant
    }
    lastStartAngleDeg_ = startDeg;

    for (int i = 0; i < kPointsPerPacket; ++i) {
        const std::size_t offset = 6 + static_cast<std::size_t>(i) * 3;
        const auto distanceMm = static_cast<std::uint16_t>(packetBuffer_[offset] | (packetBuffer_[offset + 1] << 8));
        const auto intensity = packetBuffer_[offset + 2];

        double angleDeg = startDeg + stepDeg * i;
        if (angleDeg >= 360.0) angleDeg -= 360.0;

        const int idx = static_cast<int>(std::lround(angleDeg)) % 360;

        if (distanceMm > 0) {
            accumulator_.distance_mm[static_cast<std::size_t>(idx)] = distanceMm;
            accumulator_.intensity[static_cast<std::size_t>(idx)] = intensity;
        }
    }

    return true;
}