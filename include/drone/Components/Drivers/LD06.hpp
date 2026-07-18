#pragma once

#include "drone/Components/SensorFusions/Driver.hpp"
#include <cstdint>

class LD06 : public IDriver<LidarData> {
public:
    explicit LD06(const char* portName = "/dev/ttyAMA2");

    [[nodiscard]] std::optional<TYPES::DriverError> init() override;
    [[nodiscard]] std::expected<LidarData, TYPES::DriverError> update() override;

private:
    static constexpr int kPointsPerPacket = 12;
    static constexpr std::size_t kPacketSize = 47; // header+verlen+speed+start+12*(dist+intens)+end+ts+crc

    [[nodiscard]] bool processPacket(); // décode packetBuffer_, retourne false si CRC invalide
    [[nodiscard]] static std::uint8_t crc8(const std::uint8_t* data, std::size_t len);

    const char* portName_;

    enum class ParseState : std::uint8_t { WaitHeader, WaitVerLen, Filling };
    ParseState parseState_{ParseState::WaitHeader};

    std::uint8_t packetBuffer_[kPacketSize]{};
    std::size_t packetCursor_{0};

    LidarData accumulator_{};      // tour en cours de construction
    double lastStartAngleDeg_{-1.0}; // -1 = pas encore de référence (premier paquet)

    bool crcErrorSeen_{false};     // au moins un paquet invalide jeté depuis le dernier update() réussi
    LidarData lastCompletedScan_{};
    bool rotationJustCompleted_{false};
};