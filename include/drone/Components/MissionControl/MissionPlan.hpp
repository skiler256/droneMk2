#pragma once
#include "drone/generated/missionplan.hpp"
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <vector>

// Framing du fichier plan de mission (partie non générée) :
//   [magic "MPLN":4o][version:u8][MissionHeader:kSize o]
//   [segments TLV...][crc32:4o]
// Le CRC32 couvre tout ce qui précède — détecte une corruption disque au
// lieu de parser du garbage en silence.
namespace MISSIONPLAN_CONV {

struct MissionPlan {
  MISSIONPLAN::MissionHeader header{};
  std::vector<MISSIONPLAN::SegmentVariant> segments{};
};

enum class ParseError : uint8_t {
  Truncated,    // taille insuffisante pour magic+version+header+crc
  BadMagic,     // 4 premiers octets != "MPLN"
  BadVersion,   // version inconnue de ce binaire
  BadChecksum,  // CRC32 ne correspond pas au contenu (corruption)
  MalformedSegment, // TLV illisible (consumed=0) avant d'atteindre le CRC
};

[[nodiscard]] std::vector<std::byte> serialize(const MissionPlan &plan);
[[nodiscard]] std::expected<MissionPlan, ParseError>
parse(std::span<const std::byte> bytes);

} // namespace MISSIONPLAN_CONV
