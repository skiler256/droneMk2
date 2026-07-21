#pragma once
#include "drone/generated/telemetry.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <span>

// [PacketID:1][payload applicatif:N]. Le lien (WFB-NG/CC1101) gère déjà sa
// propre trame bas niveau (longueur, CRC radio) — cet en-tête d'un octet
// sert uniquement à distinguer le type de paquet TELEM une fois le payload
// applicatif extrait par le driver.
namespace TELEM {

template <typename Pkt>
[[nodiscard]] std::array<std::byte, 1 + Pkt::kSize> encode(const Pkt &p) noexcept {
  std::array<std::byte, 1 + Pkt::kSize> buf{};
  buf[0] = static_cast<std::byte>(Pkt::kId);
  auto payload = p.pack();
  std::copy(payload.begin(), payload.end(), buf.begin() + 1);
  return buf;
}

struct Frame {
  PacketID id;
  std::span<const std::byte> payload;
};

[[nodiscard]] inline std::optional<Frame> decode(std::span<const std::byte> bytes) noexcept {
  if (bytes.empty())
    return std::nullopt;
  return Frame{static_cast<PacketID>(bytes[0]), bytes.subspan(1)};
}

} // namespace TELEM
