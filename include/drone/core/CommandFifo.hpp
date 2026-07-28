#pragma once
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include <cstdint>

// Un ordre ponctuel de MissionControl vers un composant subordonné (ex:
// takeoff, set mode) — code+payload identifient l'ordre (payload = un seul
// scalaire libre, ex: altitude en cm). `seq` est assigné par
// SharedCmdMemHandler::pushMavlinkCommand() (pas l'appelant), sert à
// corréler le CommandReport de retour.
//
// Élément d'une RingBuffer<PendingCommand, N> dans SharedCmdMem — poussée/
// dépilée via pushFifo()/popFifo(), pas de mutex propre (POD, safe en shm
// sous le CAS-lock de SharedMemoryHandler).
struct PendingCommand {
  CODES::Code code{};
  uint32_t payload{0};
  uint32_t seq{0};
  TYPES::TimePoint ts{};
};

// Résultat d'un PendingCommand — juste succès/échec, pas de détail
// (MAV_RESULT etc.) : un diagnostic plus fin passe par les CODES:: du
// subordonné, pas par ce canal de corrélation.
struct CommandReport {
  uint32_t seq{0};
  CODES::Code code{}; // redondant avec seq, utile pour l'inspection/debug
  bool success{false};
  TYPES::TimePoint ts{};
};
