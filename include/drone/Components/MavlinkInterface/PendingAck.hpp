#pragma once
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include <array>
#include <cstddef>
#include <mavlink.h>

// État de la commande ACK'd actuellement en vol côté TTxACK — une seule à
// la fois (cf. ackQueue_ pour la file d'attente). Garde la trame déjà
// packée pour la retransmettre telle quelle sur retry.
enum class PendingKind : uint8_t { None, Command, Param };

struct PendingAck {
  PendingKind kind{PendingKind::None};

  uint16_t command{0};            // valide si kind == Command (MAV_CMD_*)
  std::array<char, 16> paramId{}; // valide si kind == Param

  TYPES::TimePoint sentAt{};
  uint8_t retries{0};

  std::array<std::byte, 64> frame{}; // plus gros message ACK'd (CMD_LONG) < 64 octets
  size_t frameLen{0};

  // Corrélation vers SharedCmdMem::CommandReport — vrai seulement pour un
  // ordre MissionControl (via TCmd), pas pour une commande interne.
  bool fromMissionControl{false};
  uint32_t cmdSeq{0};
  CODES::Code cmdCode{};
};
