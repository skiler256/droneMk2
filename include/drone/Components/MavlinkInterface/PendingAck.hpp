#pragma once
#include "drone/types.hpp"
#include <array>
#include <cstddef>
#include <mavlink.h>

// État de la commande ACK'd actuellement en vol côté TTxACK — un seul
// pending à la fois (pas de file multi-commandes pour l'instant, cf.
// MAVLINKINTERFACE.md §10 : la queue interne alimentée par TCmd reste à
// faire tant que MissionControl n'existe pas). Garde la trame déjà
// packée pour pouvoir la retransmettre telle quelle sur retry, sans
// reconstruire le message.
enum class PendingKind : uint8_t { None, Command, Param };

struct PendingAck {
  PendingKind kind{PendingKind::None};

  uint16_t command{0};            // valide si kind == Command (MAV_CMD_*)
  std::array<char, 16> paramId{}; // valide si kind == Param

  TYPES::TimePoint sentAt{};
  uint8_t retries{0};

  std::array<std::byte, 64> frame{}; // plus gros message ACK'd (CMD_LONG) < 64 octets
  size_t frameLen{0};
};
