// include/drone/generated/missionplan.hpp
// ────────────────────────────────────────────────────────
// GÉNÉRÉ depuis MISSIONPLAN.csv par tools/gen_missionplan.py — NE PAS ÉDITER À LA MAIN.
// Régénérer : python3 tools/gen_missionplan.py (ou : cmake --build .)
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <variant>
#include <vector>

namespace MISSIONPLAN {

struct MissionHeader {
  static constexpr std::size_t kSize = 21;
  float battWarningV{14.0f};
  float battCriticalV{12.5f};
  float rtbTransitAlt{30.0f};
  uint8_t gcsLossAction{0};
  float navVolumeRadius{200.0f};
  float navVolumeCeiling{100.0f};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &battWarningV, sizeof(battWarningV));
    std::memcpy(buf.data() + 4, &battCriticalV, sizeof(battCriticalV));
    std::memcpy(buf.data() + 8, &rtbTransitAlt, sizeof(rtbTransitAlt));
    std::memcpy(buf.data() + 12, &gcsLossAction, sizeof(gcsLossAction));
    std::memcpy(buf.data() + 13, &navVolumeRadius, sizeof(navVolumeRadius));
    std::memcpy(buf.data() + 17, &navVolumeCeiling, sizeof(navVolumeCeiling));
    return buf;
  }

  [[nodiscard]] static MissionHeader unpack(std::span<const std::byte> bytes) noexcept {
    MissionHeader out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.battWarningV, bytes.data() + 0, sizeof(out.battWarningV));
    std::memcpy(&out.battCriticalV, bytes.data() + 4, sizeof(out.battCriticalV));
    std::memcpy(&out.rtbTransitAlt, bytes.data() + 8, sizeof(out.rtbTransitAlt));
    std::memcpy(&out.gcsLossAction, bytes.data() + 12, sizeof(out.gcsLossAction));
    std::memcpy(&out.navVolumeRadius, bytes.data() + 13, sizeof(out.navVolumeRadius));
    std::memcpy(&out.navVolumeCeiling, bytes.data() + 17, sizeof(out.navVolumeCeiling));
    return out;
  }
};

// Identifiant TLV stable (ordre du CSV) — NE PAS réordonner des entrées
// existantes, ajouter seulement en fin (compat plans déjà écrits sur disque).
enum class SegmentKind : uint8_t {
  Waypoint = 0,
  Corridor = 1,
  Curve = 2,
  Landing = 3,
  Hold = 4,
  PayloadAction = 5,
};

struct WaypointSegment {
  static constexpr SegmentKind kKind = SegmentKind::Waypoint;
  static constexpr std::size_t kPayloadSize = 20;
  static_assert(kPayloadSize <= 255, "payload doit tenir dans le champ len (u8) du TLV");
  float north{0.0f};
  float east{0.0f};
  float alt{-10.0f};
  float speed{2.0f};
  float acceptRadius{2.0f};

  [[nodiscard]] std::array<std::byte, kPayloadSize> pack() const noexcept {
    std::array<std::byte, kPayloadSize> buf{};
    std::memcpy(buf.data() + 0, &north, sizeof(north));
    std::memcpy(buf.data() + 4, &east, sizeof(east));
    std::memcpy(buf.data() + 8, &alt, sizeof(alt));
    std::memcpy(buf.data() + 12, &speed, sizeof(speed));
    std::memcpy(buf.data() + 16, &acceptRadius, sizeof(acceptRadius));
    return buf;
  }

  [[nodiscard]] static WaypointSegment unpack(std::span<const std::byte> bytes) noexcept {
    WaypointSegment out{};
    if (bytes.size() < kPayloadSize) return out;
    std::memcpy(&out.north, bytes.data() + 0, sizeof(out.north));
    std::memcpy(&out.east, bytes.data() + 4, sizeof(out.east));
    std::memcpy(&out.alt, bytes.data() + 8, sizeof(out.alt));
    std::memcpy(&out.speed, bytes.data() + 12, sizeof(out.speed));
    std::memcpy(&out.acceptRadius, bytes.data() + 16, sizeof(out.acceptRadius));
    return out;
  }
};

struct CorridorSegment {
  static constexpr SegmentKind kKind = SegmentKind::Corridor;
  static constexpr std::size_t kPayloadSize = 28;
  static_assert(kPayloadSize <= 255, "payload doit tenir dans le champ len (u8) du TLV");
  float northStart{0.0f};
  float eastStart{0.0f};
  float northEnd{0.0f};
  float eastEnd{0.0f};
  float alt{-10.0f};
  float speed{2.0f};
  float width{5.0f};

  [[nodiscard]] std::array<std::byte, kPayloadSize> pack() const noexcept {
    std::array<std::byte, kPayloadSize> buf{};
    std::memcpy(buf.data() + 0, &northStart, sizeof(northStart));
    std::memcpy(buf.data() + 4, &eastStart, sizeof(eastStart));
    std::memcpy(buf.data() + 8, &northEnd, sizeof(northEnd));
    std::memcpy(buf.data() + 12, &eastEnd, sizeof(eastEnd));
    std::memcpy(buf.data() + 16, &alt, sizeof(alt));
    std::memcpy(buf.data() + 20, &speed, sizeof(speed));
    std::memcpy(buf.data() + 24, &width, sizeof(width));
    return buf;
  }

  [[nodiscard]] static CorridorSegment unpack(std::span<const std::byte> bytes) noexcept {
    CorridorSegment out{};
    if (bytes.size() < kPayloadSize) return out;
    std::memcpy(&out.northStart, bytes.data() + 0, sizeof(out.northStart));
    std::memcpy(&out.eastStart, bytes.data() + 4, sizeof(out.eastStart));
    std::memcpy(&out.northEnd, bytes.data() + 8, sizeof(out.northEnd));
    std::memcpy(&out.eastEnd, bytes.data() + 12, sizeof(out.eastEnd));
    std::memcpy(&out.alt, bytes.data() + 16, sizeof(out.alt));
    std::memcpy(&out.speed, bytes.data() + 20, sizeof(out.speed));
    std::memcpy(&out.width, bytes.data() + 24, sizeof(out.width));
    return out;
  }
};

struct CurveSegment {
  static constexpr SegmentKind kKind = SegmentKind::Curve;
  static constexpr std::size_t kPayloadSize = 28;
  static_assert(kPayloadSize <= 255, "payload doit tenir dans le champ len (u8) du TLV");
  float centerNorth{0.0f};
  float centerEast{0.0f};
  float radius{10.0f};
  float alt{-10.0f};
  float speed{2.0f};
  float startAngle{0.0f};
  float endAngle{1.5708f};

  [[nodiscard]] std::array<std::byte, kPayloadSize> pack() const noexcept {
    std::array<std::byte, kPayloadSize> buf{};
    std::memcpy(buf.data() + 0, &centerNorth, sizeof(centerNorth));
    std::memcpy(buf.data() + 4, &centerEast, sizeof(centerEast));
    std::memcpy(buf.data() + 8, &radius, sizeof(radius));
    std::memcpy(buf.data() + 12, &alt, sizeof(alt));
    std::memcpy(buf.data() + 16, &speed, sizeof(speed));
    std::memcpy(buf.data() + 20, &startAngle, sizeof(startAngle));
    std::memcpy(buf.data() + 24, &endAngle, sizeof(endAngle));
    return buf;
  }

  [[nodiscard]] static CurveSegment unpack(std::span<const std::byte> bytes) noexcept {
    CurveSegment out{};
    if (bytes.size() < kPayloadSize) return out;
    std::memcpy(&out.centerNorth, bytes.data() + 0, sizeof(out.centerNorth));
    std::memcpy(&out.centerEast, bytes.data() + 4, sizeof(out.centerEast));
    std::memcpy(&out.radius, bytes.data() + 8, sizeof(out.radius));
    std::memcpy(&out.alt, bytes.data() + 12, sizeof(out.alt));
    std::memcpy(&out.speed, bytes.data() + 16, sizeof(out.speed));
    std::memcpy(&out.startAngle, bytes.data() + 20, sizeof(out.startAngle));
    std::memcpy(&out.endAngle, bytes.data() + 24, sizeof(out.endAngle));
    return out;
  }
};

struct LandingSegment {
  static constexpr SegmentKind kKind = SegmentKind::Landing;
  static constexpr std::size_t kPayloadSize = 9;
  static_assert(kPayloadSize <= 255, "payload doit tenir dans le champ len (u8) du TLV");
  float north{0.0f};
  float east{0.0f};
  uint8_t category{2};

  [[nodiscard]] std::array<std::byte, kPayloadSize> pack() const noexcept {
    std::array<std::byte, kPayloadSize> buf{};
    std::memcpy(buf.data() + 0, &north, sizeof(north));
    std::memcpy(buf.data() + 4, &east, sizeof(east));
    std::memcpy(buf.data() + 8, &category, sizeof(category));
    return buf;
  }

  [[nodiscard]] static LandingSegment unpack(std::span<const std::byte> bytes) noexcept {
    LandingSegment out{};
    if (bytes.size() < kPayloadSize) return out;
    std::memcpy(&out.north, bytes.data() + 0, sizeof(out.north));
    std::memcpy(&out.east, bytes.data() + 4, sizeof(out.east));
    std::memcpy(&out.category, bytes.data() + 8, sizeof(out.category));
    return out;
  }
};

struct HoldSegment {
  static constexpr SegmentKind kKind = SegmentKind::Hold;
  static constexpr std::size_t kPayloadSize = 4;
  static_assert(kPayloadSize <= 255, "payload doit tenir dans le champ len (u8) du TLV");
  float duration{0.0f};

  [[nodiscard]] std::array<std::byte, kPayloadSize> pack() const noexcept {
    std::array<std::byte, kPayloadSize> buf{};
    std::memcpy(buf.data() + 0, &duration, sizeof(duration));
    return buf;
  }

  [[nodiscard]] static HoldSegment unpack(std::span<const std::byte> bytes) noexcept {
    HoldSegment out{};
    if (bytes.size() < kPayloadSize) return out;
    std::memcpy(&out.duration, bytes.data() + 0, sizeof(out.duration));
    return out;
  }
};

struct PayloadActionSegment {
  static constexpr SegmentKind kKind = SegmentKind::PayloadAction;
  static constexpr std::size_t kPayloadSize = 17;
  static_assert(kPayloadSize <= 255, "payload doit tenir dans le champ len (u8) du TLV");
  uint8_t actionId{0};
  std::array<uint8_t, 16> payload{{}};

  [[nodiscard]] std::array<std::byte, kPayloadSize> pack() const noexcept {
    std::array<std::byte, kPayloadSize> buf{};
    std::memcpy(buf.data() + 0, &actionId, sizeof(actionId));
    std::memcpy(buf.data() + 1, &payload, sizeof(payload));
    return buf;
  }

  [[nodiscard]] static PayloadActionSegment unpack(std::span<const std::byte> bytes) noexcept {
    PayloadActionSegment out{};
    if (bytes.size() < kPayloadSize) return out;
    std::memcpy(&out.actionId, bytes.data() + 0, sizeof(out.actionId));
    std::memcpy(&out.payload, bytes.data() + 1, sizeof(out.payload));
    return out;
  }
};

using SegmentVariant = std::variant<WaypointSegment, CorridorSegment, CurveSegment, LandingSegment, HoldSegment, PayloadActionSegment>;

// [kind:u8][len:u8][payload...] — len permet à un lecteur de sauter un
// segment de type inconnu (plan écrit par un firmware plus récent) sans
// faire échouer tout le parse (cf. decodeSegment ci-dessous).

[[nodiscard]] inline std::vector<std::byte> encodeSegment(const SegmentVariant &seg) noexcept {
  return std::visit([](const auto &s) {
    std::vector<std::byte> out;
    out.reserve(2 + s.kPayloadSize);
    out.push_back(static_cast<std::byte>(s.kKind));
    out.push_back(static_cast<std::byte>(s.kPayloadSize));
    auto payload = s.pack();
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
  }, seg);
}

struct DecodedSegment {
  // nullopt si kind inconnu ou len incohérente avec le type attendu —
  // consumed reste correct dans tous les cas pour permettre de sauter
  // l'entrée et continuer le parse du reste du plan.
  std::optional<SegmentVariant> segment;
  std::size_t consumed{0};
};

[[nodiscard]] inline DecodedSegment decodeSegment(std::span<const std::byte> bytes) noexcept {
  if (bytes.size() < 2) return {};
  auto kind = static_cast<uint8_t>(bytes[0]);
  auto len = static_cast<uint8_t>(bytes[1]);
  if (bytes.size() < static_cast<std::size_t>(2 + len)) return {};
  auto payload = bytes.subspan(2, len);
  DecodedSegment out{.segment = std::nullopt, .consumed = static_cast<std::size_t>(2 + len)};
  switch (static_cast<SegmentKind>(kind)) {
  case SegmentKind::Waypoint:
    if (len != WaypointSegment::kPayloadSize) return out;
    out.segment = WaypointSegment::unpack(payload);
    return out;
  case SegmentKind::Corridor:
    if (len != CorridorSegment::kPayloadSize) return out;
    out.segment = CorridorSegment::unpack(payload);
    return out;
  case SegmentKind::Curve:
    if (len != CurveSegment::kPayloadSize) return out;
    out.segment = CurveSegment::unpack(payload);
    return out;
  case SegmentKind::Landing:
    if (len != LandingSegment::kPayloadSize) return out;
    out.segment = LandingSegment::unpack(payload);
    return out;
  case SegmentKind::Hold:
    if (len != HoldSegment::kPayloadSize) return out;
    out.segment = HoldSegment::unpack(payload);
    return out;
  case SegmentKind::PayloadAction:
    if (len != PayloadActionSegment::kPayloadSize) return out;
    out.segment = PayloadActionSegment::unpack(payload);
    return out;
  }
  return out; // kind inconnu -> segment=nullopt, consumed correct (skip)
}

} // namespace MISSIONPLAN
