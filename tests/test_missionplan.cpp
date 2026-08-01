// tests/test_missionplan.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Framing du plan de mission (MISSIONPLAN_CONV::serialize/parse) — logique
// pure, testable sans MissionControl. Lance : cd build && ctest -v

#include "drone/Components/MissionControl/MissionPlan.hpp"
#include "drone/utilities.hpp"
#include <array>
#include <cstring>
#include <gtest/gtest.h>

using namespace MISSIONPLAN;
using namespace MISSIONPLAN_CONV;

namespace {

// Recalcule et ré-attache un CRC valide sur un buffer patché à la main
// (magic+version+header+segments déjà en place, sans son crc d'origine).
std::vector<std::byte> reattachCrc(std::vector<std::byte> body) {
  uint32_t crc = UTILITIES::crc32(body.data(), body.size());
  std::array<std::byte, 4> crcBytes{};
  std::memcpy(crcBytes.data(), &crc, 4);
  body.insert(body.end(), crcBytes.begin(), crcBytes.end());
  return body;
}

} // namespace

TEST(MissionPlanTest, RoundTripHeaderAndSegments) {
  MissionPlan plan{};
  plan.header.battWarningV = 14.8f;
  plan.header.rtbTransitAlt = 25.0f;
  plan.segments.push_back(WaypointSegment{
      .north = 10.0f, .east = -5.0f, .alt = -12.0f, .speed = 3.0f, .acceptRadius = 1.5f});
  plan.segments.push_back(HoldSegment{.duration = 4.0f});
  PayloadActionSegment action{};
  action.actionId = 7;
  action.payload[0] = 0xAB;
  action.payload[15] = 0xCD;
  plan.segments.push_back(action);

  auto bytes = serialize(plan);
  auto parsed = parse(bytes);
  ASSERT_TRUE(parsed.has_value());

  EXPECT_FLOAT_EQ(parsed->header.battWarningV, 14.8f);
  EXPECT_FLOAT_EQ(parsed->header.rtbTransitAlt, 25.0f);
  ASSERT_EQ(parsed->segments.size(), 3u);

  auto *wp = std::get_if<WaypointSegment>(&parsed->segments[0]);
  ASSERT_NE(wp, nullptr);
  EXPECT_FLOAT_EQ(wp->north, 10.0f);
  EXPECT_FLOAT_EQ(wp->east, -5.0f);

  auto *hold = std::get_if<HoldSegment>(&parsed->segments[1]);
  ASSERT_NE(hold, nullptr);
  EXPECT_FLOAT_EQ(hold->duration, 4.0f);

  auto *pa = std::get_if<PayloadActionSegment>(&parsed->segments[2]);
  ASSERT_NE(pa, nullptr);
  EXPECT_EQ(pa->actionId, 7);
  EXPECT_EQ(pa->payload[0], 0xAB);
  EXPECT_EQ(pa->payload[15], 0xCD);
}

TEST(MissionPlanTest, EmptyPlanRoundTrips) {
  MissionPlan plan{}; // header aux valeurs par défaut du CSV, aucun segment
  auto parsed = parse(serialize(plan));
  ASSERT_TRUE(parsed.has_value());
  EXPECT_TRUE(parsed->segments.empty());
  EXPECT_FLOAT_EQ(parsed->header.battWarningV, 14.0f);
}

TEST(MissionPlanTest, TruncatedBufferRejected) {
  auto bytes = serialize(MissionPlan{});
  bytes.resize(bytes.size() - 10);
  auto parsed = parse(bytes);
  ASSERT_FALSE(parsed.has_value());
  EXPECT_EQ(parsed.error(), ParseError::Truncated);
}

TEST(MissionPlanTest, BadMagicRejected) {
  auto bytes = serialize(MissionPlan{});
  bytes[0] = std::byte{0x00};
  auto parsed = parse(bytes);
  ASSERT_FALSE(parsed.has_value());
  EXPECT_EQ(parsed.error(), ParseError::BadMagic);
}

TEST(MissionPlanTest, BadVersionRejected) {
  auto bytes = serialize(MissionPlan{});
  bytes[4] = std::byte{0xFF}; // offset du champ version, juste après le magic (4o)
  auto parsed = parse(bytes);
  ASSERT_FALSE(parsed.has_value());
  EXPECT_EQ(parsed.error(), ParseError::BadVersion);
}

TEST(MissionPlanTest, CorruptedByteFailsChecksum) {
  MissionPlan plan{};
  plan.segments.push_back(WaypointSegment{});
  auto bytes = serialize(plan);
  bytes[6] = std::byte{static_cast<uint8_t>(static_cast<uint8_t>(bytes[6]) ^ 0xFFu)}; // un octet du header
  auto parsed = parse(bytes);
  ASSERT_FALSE(parsed.has_value());
  EXPECT_EQ(parsed.error(), ParseError::BadChecksum);
}

TEST(MissionPlanTest, UnknownSegmentKindSkippedForwardCompat) {
  MissionPlan plan{};
  plan.segments.push_back(WaypointSegment{.north = 1.0f});
  auto bytes = serialize(plan);

  // Insère un TLV de type inconnu (99) juste après le header, avant le
  // waypoint déjà encodé, puis recalcule le CRC — simule un plan écrit par
  // un firmware futur avec un type de segment que ce binaire ne connaît pas.
  std::vector<std::byte> body(bytes.begin(), bytes.end() - 4); // retire l'ancien crc
  auto insertAt = static_cast<std::ptrdiff_t>(4 + 1 + MissionHeader::kSize);
  std::vector<std::byte> unknownTlv{std::byte{99}, std::byte{3}, std::byte{0}, std::byte{0},
                                    std::byte{0}};
  body.insert(body.begin() + insertAt, unknownTlv.begin(), unknownTlv.end());

  auto parsed = parse(reattachCrc(body));
  ASSERT_TRUE(parsed.has_value());
  ASSERT_EQ(parsed->segments.size(), 1u); // le segment inconnu est sauté, pas ajouté
  auto *wp = std::get_if<WaypointSegment>(&parsed->segments[0]);
  ASSERT_NE(wp, nullptr);
  EXPECT_FLOAT_EQ(wp->north, 1.0f);
}

TEST(MissionPlanTest, MalformedSegmentLengthRejected) {
  auto bytes = serialize(MissionPlan{}); // pas de segment, juste header
  std::vector<std::byte> body(bytes.begin(), bytes.end() - 4);
  // TLV qui annonce une longueur bien plus grande que ce qui suit réellement.
  body.push_back(std::byte{static_cast<uint8_t>(SegmentKind::Waypoint)});
  body.push_back(std::byte{200});

  auto parsed = parse(reattachCrc(body));
  ASSERT_FALSE(parsed.has_value());
  EXPECT_EQ(parsed.error(), ParseError::MalformedSegment);
}
