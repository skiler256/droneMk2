#pragma once
#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"

// Commandes GCS routées vers MissionControl (CMD_ARM/DISARM/MODE/WAYPOINT/
// RTL/LAND/TAKEOFF/HOLD/RESUME/ANSWER, cf. TELEMETRY.csv). CMD_EMERGENCY
// n'y est PAS : elle est signalée via un CODES::Code (SMO001, cf.
// INDEX.csv) plutôt que ce FIFO — un ordre d'urgence ne doit jamais
// attendre son tour derrière d'autres commandes en file. TSafetyMonitor
// (MC) scanne les codes actifs, pas ce FIFO, donc aucune latence de
// dépilement pour l'E-stop.
enum class GCSCommandType : uint8_t {
  Arm,
  Disarm,
  Mode,
  Waypoint,
  Rtl,
  Land,
  Takeoff,
  Hold,
  Resume,
  Answer // réponse à un MC_QUERY (cf. SharedCmdMem::PendingQuery)
};

// Struct à plat (pas d'union) : plus simple à stocker en shm — le léger
// gaspillage d'espace (les champs hors-sujet pour `type` restent à zéro)
// est négligeable pour une commande ponctuelle GCS, pas un flux haute
// fréquence.
struct GCSCommand {
  uint32_t seq{0};
  GCSCommandType type{GCSCommandType::Arm};
  TYPES::TimePoint ts{};
  uint8_t mode{0};                         // Mode
  TYPES::Position waypoint{};              // Waypoint : north/east/down (ned)
  TYPES::MetersPerSec approachSpeed{0.0f}; // Waypoint
  TYPES::Meters acceptanceRadius{0.0f};    // Waypoint
  TYPES::Meters altitude{0.0f};            // Takeoff
  uint32_t answerSeq{0};                   // Answer : corrèle à PendingQuery::seq
  uint8_t answerChoice{0};                 // Answer : cf. QueryType pour la sémantique
};

struct SharedComMem {
SharedCompMem compMem;
struct payload {
  RingBuffer<GCSCommand, 8> gcsCmds{};
  uint32_t nextGcsSeq{0};

  // Dernière réception HEARTBEAT_GCS par lien — sert de proxy de
  // "qualité" faute de RSSI matériel (WFB_NG_Driver/CC1101_Driver pas
  // encore implémentés, cf. SYSMONITORING.md §7) : un consommateur (MC,
  // dashboard) déduit fraîcheur/perte de lien via msBetween(now, ...).
  TYPES::TimePoint lastHeartbeatMain{};
  TYPES::TimePoint lastHeartbeatSec{};
};
payload data;
};

class SharedComMemHandler : public SharedCompMemHandler {
public:
  explicit SharedComMemHandler(TYPES::ComponentID id, TYPES::Us timeout,
                              SharedComMem &Com)
      : SharedCompMemHandler(Com.compMem, id, timeout), Com_(Com) {};

  // seq assigné ici, jamais par l'appelant — même raison que
  // SharedCmdMemHandler::pushMavlinkCommand.
  [[nodiscard]] bool pushGCSCommand(GCSCommand c) {
    auto seq = getData(id_, timeout_, Com_.data.nextGcsSeq).value_or(0);
    auto next = seq + 1;
    setData(id_, timeout_, Com_.data.nextGcsSeq, next);
    c.seq = seq;
    auto now = TYPES::Clock::now();
    c.ts = now;
    return pushFifo(id_, timeout_, Com_.data.gcsCmds, c);
  }

  [[nodiscard]] std::optional<GCSCommand> popGCSCommand() {
    return popFifo(id_, timeout_, Com_.data.gcsCmds);
  }

  void markHeartbeatMain() {
    auto now = TYPES::Clock::now();
    setData(id_, timeout_, Com_.data.lastHeartbeatMain, now);
  }
  void markHeartbeatSec() {
    auto now = TYPES::Clock::now();
    setData(id_, timeout_, Com_.data.lastHeartbeatSec, now);
  }

  [[nodiscard]] std::optional<TYPES::TimePoint> getLastHeartbeatMain() {
    return getData(id_, timeout_, Com_.data.lastHeartbeatMain);
  }
  [[nodiscard]] std::optional<TYPES::TimePoint> getLastHeartbeatSec() {
    return getData(id_, timeout_, Com_.data.lastHeartbeatSec);
  }

private:
    SharedComMem &Com_;

    uint32_t computeChecksum(TYPES::ComponentID) {
      return UTILITIES::crc32(Com_.data) ^ historyChecksum();
    };

    void reset(TYPES::ComponentID id) {
      if (id == TYPES::ComponentID::SysMonitoring) {
        Com_.data = SharedComMem::payload{};
        sanitizeHistory(comp_.HotStartHistory);
        sanitizeHistory(comp_.ColdStartHistory);
        updateChecksum(id);
      }
    };


  };
