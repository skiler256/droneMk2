#include "drone/Components/MissionControl/MissionPlan.hpp"
#include "drone/utilities.hpp"
#include <array>
#include <cstring>

namespace MISSIONPLAN_CONV {

namespace {
constexpr std::array<uint8_t, 4> kMagic{'M', 'P', 'L', 'N'};
constexpr uint8_t kVersion = 1;
constexpr std::size_t kMagicSize = kMagic.size();
constexpr std::size_t kVersionSize = 1;
constexpr std::size_t kCrcSize = 4;
constexpr std::size_t kMinSize =
    kMagicSize + kVersionSize + MISSIONPLAN::MissionHeader::kSize + kCrcSize;
} // namespace

std::vector<std::byte> serialize(const MissionPlan &plan) {
  std::vector<std::byte> out;
  out.reserve(kMinSize);

  for (auto b : kMagic)
    out.push_back(static_cast<std::byte>(b));
  out.push_back(static_cast<std::byte>(kVersion));

  auto headerBytes = plan.header.pack();
  out.insert(out.end(), headerBytes.begin(), headerBytes.end());

  for (const auto &seg : plan.segments) {
    auto segBytes = MISSIONPLAN::encodeSegment(seg);
    out.insert(out.end(), segBytes.begin(), segBytes.end());
  }

  uint32_t crc = UTILITIES::crc32(out.data(), out.size());
  std::array<std::byte, kCrcSize> crcBytes{};
  std::memcpy(crcBytes.data(), &crc, kCrcSize);
  out.insert(out.end(), crcBytes.begin(), crcBytes.end());

  return out;
}

std::expected<MissionPlan, ParseError> parse(std::span<const std::byte> bytes) {
  if (bytes.size() < kMinSize)
    return std::unexpected(ParseError::Truncated);

  for (std::size_t i = 0; i < kMagicSize; ++i) {
    if (static_cast<uint8_t>(bytes[i]) != kMagic[i])
      return std::unexpected(ParseError::BadMagic);
  }
  if (static_cast<uint8_t>(bytes[kMagicSize]) != kVersion)
    return std::unexpected(ParseError::BadVersion);

  std::size_t crcOffset = bytes.size() - kCrcSize;
  uint32_t storedCrc{};
  std::memcpy(&storedCrc, bytes.data() + crcOffset, kCrcSize);
  uint32_t computedCrc = UTILITIES::crc32(bytes.data(), crcOffset);
  if (storedCrc != computedCrc)
    return std::unexpected(ParseError::BadChecksum);

  MissionPlan plan{};
  std::size_t offset = kMagicSize + kVersionSize;
  plan.header = MISSIONPLAN::MissionHeader::unpack(bytes.subspan(offset));
  offset += MISSIONPLAN::MissionHeader::kSize;

  // consumed=0 : longueur incohérente, aucune progression possible — on
  // abandonne au lieu de boucler. Un kind inconnu est juste ignoré, cf.
  // MISSIONCONTROL.md §5.
  while (offset < crcOffset) {
    auto decoded =
        MISSIONPLAN::decodeSegment(bytes.subspan(offset, crcOffset - offset));
    if (decoded.consumed == 0)
      return std::unexpected(ParseError::MalformedSegment);
    if (decoded.segment)
      plan.segments.push_back(*decoded.segment);
    offset += decoded.consumed;
  }

  return plan;
}

} // namespace MISSIONPLAN_CONV
