#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/core/CommandFifo.hpp"
#include "drone/core/SharedMemory.hpp"
#include "drone/types.hpp"

// Segments Line/Curve ajoutés seulement quand ils existeront réellement.
enum class SegmentType : uint8_t { Waypoint };

// Défini ici et non dans MissionControl.hpp : SysMonitoring lit l'état via
// cmd_ sans dépendre du header du composant. Pas de RTB en V1.
enum class MissionState : uint8_t {
  Idle,
  PreFlight,
  Armed,
  Mission,
  Manual,
  Hold,
  Landing,
  Emergency
};

// Un type + un seq de corrélation avec CMD_ANSWER, pas un paquet par
// question — cf. MC_QUERY dans TELEMETRY.csv.
enum class QueryType : uint8_t {
  None,                     // aucune question active
  ObstacleNonContournable,  // choice: 0=RTB, 1=MANUAL
  EmergencyRecoverable,     // choice: 0=resume, 1=land
  LidarLost                 // choice: 0=reprendre sans évitement d'obstacle, 1=MANUAL
};

struct PendingQuery {
  uint32_t seq{0};
  QueryType type{QueryType::None};
};

// Cible continue, pas un ordre ponctuel : valeur unique, pas de FIFO. La
// promotion armé->engagé appartient à TSeq, jamais à MC — cf.
// NAVIGATION.md §3-4. `seq` (et non un timestamp) sert à détecter un
// changement de segment, qui ne survient pas à chaque tick.
struct NavSegment {
  // Assigné par setArmedSegment() ; seq=0 = sentinel "rien d'armé".
  uint32_t seq{0};
  SegmentType type{SegmentType::Waypoint};
  // Vérifié par TSeq avant promotion, tous types — NAV_CONV::canEngage.
  TYPES::Position entryWp{};
  TYPES::Position exitWp{}; // Waypoint : le point cible
  TYPES::MetersPerSec speed{0.0f};
  TYPES::Meters targetAltitude{0.0f};
};

enum class SegmentStatus : uint8_t { EnCours, Atteint, Hold };

// Écrit par TAP à chaque tick ; `seq` reprend celui du NavSegment concerné,
// pour rester interprétable si MC en pousse un nouveau entre-temps.
struct SegmentProgress {
  uint32_t seq{0};
  SegmentStatus status{SegmentStatus::EnCours};
  TYPES::Meters distanceRestante{0.0f};
  TYPES::TimePoint ts{};
};

// Une paire (ordres + reports) par subordonné : MavlinkInterface et Nav.
// SF sera ajoutée le jour où elle en aura besoin, pas par anticipation.
struct SharedCmdMem {
  SharedCompMem compMem;

  struct payload {
    RingBuffer<PendingCommand, 8> mavlinkCmds{};
    RingBuffer<CommandReport, 8> mavlinkReports{};
    uint32_t nextMavlinkSeq{0};

    // Ordres ponctuels, distincts de NavSegment/SegmentProgress ci-dessous.
    RingBuffer<PendingCommand, 8> navCmds{};
    RingBuffer<CommandReport, 8> navReports{};
    uint32_t nextNavSeq{0};

    NavSegment armedSegment{};   // écrit par MC
    NavSegment engagedSegment{}; // écrit par TSeq (Nav)
    SegmentProgress segmentProgress{};
    uint32_t nextSegmentSeq{1}; // commence à 1, 0 = sentinel "rien d'armé"

    // Écrit par TStateMachine, lu par SysMonitoring (MISSION_STATE).
    // Reste à Idle tant que TStateMachine n'a pas de logique.
    MissionState state{MissionState::Idle};

    // Écrit par MC, lu par SysMonitoring (MC_QUERY) ; la réponse revient
    // via SharedComMem::gcsCmds.
    PendingQuery query{};
    uint32_t nextQuerySeq{1}; // commence à 1, 0 = sentinel côté seq initial

    // Preuve de vie lue par FSTask, seul filet puisque MC ne peut pas être
    // cold-starté — cf. MAVLINKINTERFACE.md §9.
    TYPES::TimePoint lastHeartbeat{};
  };
  payload data;
};

class SharedCmdMemHandler : public SharedCompMemHandler {
public:
  explicit SharedCmdMemHandler(TYPES::ComponentID id, TYPES::Us timeout,
                               SharedCmdMem &cmd)
      : SharedCompMemHandler(cmd.compMem, id, timeout), cmd_(cmd) {};

  // seq assigné ici : pas de collision/oubli possible côté appelant.
  [[nodiscard]] bool pushMavlinkCommand(CODES::Code code, uint32_t payload) {
    auto seq = getData(id_, timeout_, cmd_.data.nextMavlinkSeq).value_or(0);
    auto next = seq + 1;
    setData(id_, timeout_, cmd_.data.nextMavlinkSeq, next);
    PendingCommand c{.code = code, .payload = payload, .seq = seq, .ts = TYPES::Clock::now()};
    return pushFifo(id_, timeout_, cmd_.data.mavlinkCmds, c);
  }

  [[nodiscard]] std::optional<PendingCommand> popMavlinkCommand() {
    return popFifo(id_, timeout_, cmd_.data.mavlinkCmds);
  }

  // Appelé par le subordonné une fois le PendingCommand résolu.
  [[nodiscard]] bool pushMavlinkReport(uint32_t seq, CODES::Code code, bool success) {
    CommandReport r{.seq = seq, .code = code, .success = success, .ts = TYPES::Clock::now()};
    return pushFifo(id_, timeout_, cmd_.data.mavlinkReports, r);
  }

  [[nodiscard]] std::optional<CommandReport> popMavlinkReport() {
    return popFifo(id_, timeout_, cmd_.data.mavlinkReports);
  }

  // Même mécanisme que pushMavlinkCommand/popMavlinkCommand, pour Nav.
  [[nodiscard]] bool pushNavCommand(CODES::Code code, uint32_t payload) {
    auto seq = getData(id_, timeout_, cmd_.data.nextNavSeq).value_or(0);
    auto next = seq + 1;
    setData(id_, timeout_, cmd_.data.nextNavSeq, next);
    PendingCommand c{.code = code, .payload = payload, .seq = seq, .ts = TYPES::Clock::now()};
    return pushFifo(id_, timeout_, cmd_.data.navCmds, c);
  }

  [[nodiscard]] std::optional<PendingCommand> popNavCommand() {
    return popFifo(id_, timeout_, cmd_.data.navCmds);
  }

  [[nodiscard]] bool pushNavReport(uint32_t seq, CODES::Code code, bool success) {
    CommandReport r{.seq = seq, .code = code, .success = success, .ts = TYPES::Clock::now()};
    return pushFifo(id_, timeout_, cmd_.data.navReports, r);
  }

  [[nodiscard]] std::optional<CommandReport> popNavReport() {
    return popFifo(id_, timeout_, cmd_.data.navReports);
  }

  // Écrit par MC, lu par TSeq ; seq assigné ici.
  void setArmedSegment(NavSegment s) {
    auto seq = getData(id_, timeout_, cmd_.data.nextSegmentSeq).value_or(1);
    auto next = seq + 1;
    setData(id_, timeout_, cmd_.data.nextSegmentSeq, next);
    s.seq = seq;
    setData(id_, timeout_, cmd_.data.armedSegment, s);
  }

  [[nodiscard]] std::optional<NavSegment> getArmedSegment() {
    return getData(id_, timeout_, cmd_.data.armedSegment);
  }

  // Écrit par TSeq (jamais MC), lu par TAP et MC.
  void setEngagedSegment(NavSegment s) {
    setData(id_, timeout_, cmd_.data.engagedSegment, s);
  }

  [[nodiscard]] std::optional<NavSegment> getEngagedSegment() {
    return getData(id_, timeout_, cmd_.data.engagedSegment);
  }

  // Avancement — écrit par TAP à chaque tick, lu par MC.
  void setSegmentProgress(SegmentProgress p) {
    setData(id_, timeout_, cmd_.data.segmentProgress, p);
  }

  [[nodiscard]] std::optional<SegmentProgress> getSegmentProgress() {
    return getData(id_, timeout_, cmd_.data.segmentProgress);
  }

  // État système — écrit par TStateMachine (MC), lu par SysMonitoring.
  void setMissionState(MissionState s) { setData(id_, timeout_, cmd_.data.state, s); }

  [[nodiscard]] std::optional<MissionState> getMissionState() {
    return getData(id_, timeout_, cmd_.data.state);
  }

  // Question au GCS ; seq assigné ici.
  void setPendingQuery(QueryType type) {
    auto seq = getData(id_, timeout_, cmd_.data.nextQuerySeq).value_or(1);
    auto next = seq + 1;
    setData(id_, timeout_, cmd_.data.nextQuerySeq, next);
    PendingQuery q{.seq = seq, .type = type};
    setData(id_, timeout_, cmd_.data.query, q);
  }

  [[nodiscard]] std::optional<PendingQuery> getPendingQuery() {
    return getData(id_, timeout_, cmd_.data.query);
  }

  // Écrit par TStateMachine à chaque cycle, lu par FSTask.
  void setHeartbeat(TYPES::TimePoint t) { setData(id_, timeout_, cmd_.data.lastHeartbeat, t); }

  [[nodiscard]] std::optional<TYPES::TimePoint> getHeartbeat() {
    return getData(id_, timeout_, cmd_.data.lastHeartbeat);
  }

private:
  SharedCmdMem &cmd_;

  uint32_t computeChecksum(TYPES::ComponentID) {
    return UTILITIES::crc32(cmd_.data) ^ historyChecksum();
  };

  void reset(TYPES::ComponentID id) {
    // Seul le propriétaire peut reconstruire sa propre shm.
    if (id == TYPES::ComponentID::MissionControl) {
      cmd_.data = SharedCmdMem::payload{};
      sanitizeHistory(comp_.HotStartHistory);
      sanitizeHistory(comp_.ColdStartHistory);
      updateChecksum(id);
    }
  };
};
