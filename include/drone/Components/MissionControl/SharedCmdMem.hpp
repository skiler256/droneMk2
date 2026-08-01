#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/core/CommandFifo.hpp"
#include "drone/core/SharedMemory.hpp"
#include "drone/types.hpp"

// Segments Line/Curve ajoutés seulement quand ils existeront réellement.
enum class SegmentType : uint8_t { Waypoint };

// États du système — cf. SDD §3.6.2 (noms des états uniquement, le graphe
// de transitions n'est pas encore implémenté). Défini ici (pas dans
// MissionControl.hpp) pour que SysMonitoring puisse lire l'état courant
// via cmd_ sans dépendre du header du composant MissionControl lui-même.
// Pas de RTB en V1 (décidé explicitement) — cf. MC_CONV::step().
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

// Question active posée au GCS (cf. MC_QUERY dans TELEMETRY.csv) — pas un
// paquet par question, un type + un seq de corrélation avec CMD_ANSWER.
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

// Cible continue que TAP poursuit jusqu'à complétion — pas un ordre
// ponctuel comme PendingCommand, donc valeur unique (pas de FIFO). MC
// écrit armedSegment (le suivant, en attente) ; TSeq promeut armedSegment
// en engagedSegment quand ce dernier est Atteint (ou sur ordre d'amorçage
// initial via navCmds), après vérification de sécurité — jamais MC
// n'écrit directement engagedSegment. `seq` permet de détecter un
// changement (reset des états internes du PID) plutôt qu'un timestamp,
// puisqu'un segment ne change pas à chaque tick comme un échantillon
// capteur.
struct NavSegment {
  // Assigné par setArmedSegment(), jamais par l'appelant (même raison que
  // PendingCommand::seq) — seq=0 sert de sentinel "rien d'armé" côté TSeq.
  uint32_t seq{0};
  SegmentType type{SegmentType::Waypoint};
  // Position attendue du drone au début du segment (pour tous les types,
  // pas seulement Line/Curve) — c'est contre ce point que TSeq vérifie la
  // sécurité avant de promouvoir armé->engagé (cf. NAV_CONV::canEngage).
  TYPES::Position entryWp{};
  TYPES::Position exitWp{}; // Waypoint : le point cible
  TYPES::MetersPerSec speed{0.0f};
  TYPES::Meters targetAltitude{0.0f};
};

enum class SegmentStatus : uint8_t { EnCours, Atteint, Hold };

// Écrit par TAP à chaque tick — `seq` reprend celui du NavSegment concerné
// pour que MC sache à quoi l'avancement se rapporte (utile si MC pousse un
// nouveau segment avant que Nav ait fini de traiter l'ancien).
struct SegmentProgress {
  uint32_t seq{0};
  SegmentStatus status{SegmentStatus::EnCours};
  TYPES::Meters distanceRestante{0.0f};
  TYPES::TimePoint ts{};
};

// Une paire de CommandFifo (ordres + reports) par composant subordonné —
// MavlinkInterface et Nav pour l'instant, SF ajoutée quand elle en aura
// besoin (cf. discussion architecture, pas de FIFO fantôme construit par
// anticipation).
struct SharedCmdMem {
  SharedCompMem compMem;

  struct payload {
    RingBuffer<PendingCommand, 8> mavlinkCmds{};
    RingBuffer<CommandReport, 8> mavlinkReports{};
    uint32_t nextMavlinkSeq{0};

    // Ordres ponctuels pour Nav (FIFO "classique", contenu à définir à
    // l'usage) — distinct de NavSegment/SegmentProgress ci-dessous.
    RingBuffer<PendingCommand, 8> navCmds{};
    RingBuffer<CommandReport, 8> navReports{};
    uint32_t nextNavSeq{0};

    NavSegment armedSegment{};   // écrit par MC
    NavSegment engagedSegment{}; // écrit par TSeq (Nav)
    SegmentProgress segmentProgress{};
    uint32_t nextSegmentSeq{1}; // commence à 1, 0 = sentinel "rien d'armé"

    // État système courant — écrit par TStateMachine (MC), lu par
    // SysMonitoring pour MISSION_STATE. Pas encore écrit (TStateMachine
    // n'a pas de logique, cf. MissionControl.hpp) : reste à Idle jusque-là.
    MissionState state{MissionState::Idle};

    // Question active posée au GCS — écrit par MC, lu par SysMonitoring
    // pour MC_QUERY. La réponse (CMD_ANSWER) revient via SharedComMem::gcsCmds.
    PendingQuery query{};
    uint32_t nextQuerySeq{1}; // commence à 1, 0 = sentinel côté seq initial

    // Preuve de vie de TStateMachine — lu par MavlinkInterface (FSTask)
    // avec son propre timeout, pour un fallback autonome (LAND) si MC ne
    // répond plus. Personne d'autre ne redémarre MC en sécurité (cf.
    // CLAUDE.md, jamais de cold-start), donc c'est le seul filet.
    TYPES::TimePoint lastHeartbeat{};
  };
  payload data;
};

class SharedCmdMemHandler : public SharedCompMemHandler {
public:
  explicit SharedCmdMemHandler(TYPES::ComponentID id, TYPES::Us timeout,
                               SharedCmdMem &cmd)
      : SharedCompMemHandler(cmd.compMem, id, timeout), cmd_(cmd) {};

  // seq assigné ici, jamais par l'appelant — élimine toute possibilité de
  // collision/oubli côté MissionControl.
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

  // Appelé par le composant subordonné (ex: MavlinkInterface) une fois un
  // PendingCommand résolu — succès ou échec, cf. CommandReport.
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

  // Segment armé — écrit par MC, lu par TSeq. seq assigné ici (jamais par
  // l'appelant), même raison que pushMavlinkCommand.
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

  // Segment engagé — écrit par TSeq (jamais MC directement, cf. NavSegment),
  // lu par TAP et par MC (pour savoir si armedSegment a été consommé).
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

  // Question au GCS — seq assigné ici (même raison que setArmedSegment),
  // jamais par l'appelant.
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

  // Heartbeat MC — écrit par TStateMachine à chaque cycle, lu par
  // MavlinkInterface (FSTask) pour détecter un MC mort.
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
