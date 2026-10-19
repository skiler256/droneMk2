#include "drone/Components/MissionControl/MissionControl.hpp"
#include "drone/Components/MissionControl/Conversions.hpp"
#include "drone/generated/codes.hpp"
#include "drone/utilities.hpp"
#include <cmath>
#include <cstdlib>
#include <fstream>

using namespace TYPES;

namespace {

// Surcharge réservée au SITL, cf. AUDIT.md §20.
[[nodiscard]] std::string missionPlanPath() {
#ifdef DRONE_USE_SITL_DRIVERS
  if (const char *override = std::getenv("DRONE_MISSION_PLAN_PATH"))
    return override;
#endif
  return "/etc/drone/missionplan.bin";
}

// cf. MISSIONCONTROL.md §4 (armError).
constexpr Ms kArmTimeout{5000};

// cf. MISSIONCONTROL.md §5 (pas de timeout sur la durée du segment).
constexpr Ms kNavStaleThreshold{500};

// cf. MISSIONCONTROL.md §4 (landed).
constexpr float kLandedVelocityThreshold = 1.0f;

// Détection de fin de montée post-arm, cf. sitl/SITL.md §3.
constexpr float kTakeoffAltToleranceM = 1.0f;
constexpr float kTakeoffVertSpeedToleranceMs = 0.3f;

[[nodiscard]] bool hasMajorCode(SharedSysStateMemHandler &sysState, ComponentID id,
                                CODES::Code code) {
  auto entries = sysState.getMajorCodes(id);
  if (!entries)
    return false;
  auto packed = code.pack();
  for (const auto &e : *entries)
    if (e.code == packed)
      return true;
  return false;
}

// S'appuie sur ComponentBase plutôt que de recompter les crashs, cf.
// MISSIONCONTROL.md §6.
[[nodiscard]] bool anyComponentHealth(SharedSysStateMemHandler &sysState,
                                      std::span<const ComponentID> ids,
                                      ComponentHealth health) {
  for (auto id : ids)
    if (sysState.getHealth(id).value_or(ComponentHealth::DEAD) == health)
      return true;
  return false;
}

[[nodiscard]] std::optional<std::vector<std::byte>> readFile(std::string_view path) {
  std::ifstream file(std::string(path), std::ios::binary | std::ios::ate);
  if (!file)
    return std::nullopt;
  auto size = file.tellg();
  if (size <= 0)
    return std::nullopt;
  file.seekg(0);
  std::vector<std::byte> buf(static_cast<std::size_t>(size));
  if (!file.read(reinterpret_cast<char *>(buf.data()), size))
    return std::nullopt;
  return buf;
}

// Charge et valide le plan ; batt_low bloque aussi, pas seulement
// batt_critical — cf. MISSIONCONTROL.md §5.
[[nodiscard]] bool runPreflightChecks(MissionControl &comp) {
  bool gpsOk = !hasMajorCode(comp.sysState_, ComponentID::SensorFusion, CODES::SFS002) &&
              !hasMajorCode(comp.sysState_, ComponentID::SensorFusion, CODES::SFS005);
  bool battOk = !hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::FCP001) &&
               !hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::FCP003);
  if (!gpsOk || !battOk) {
    comp.sysState_.raiseCode(CODES::MCA009);
    return false;
  }

  auto bytes = readFile(missionPlanPath());
  if (!bytes) {
    comp.sysState_.raiseCode(CODES::MCA004);
    return false;
  }
  auto plan = MISSIONPLAN_CONV::parse(*bytes);
  if (!plan) {
    comp.sysState_.raiseCode(CODES::MCA004);
    return false;
  }
  {
    std::lock_guard lk(comp.planMutex_);
    comp.plan_ = *plan;
  }
  return true;
}

} // namespace

// Fréquences/priorités, cf. MISSIONCONTROL.md §2.
MissionControl::MissionControl(ComponenConfig config,
                               SharedSysStateMemHandler &sysState,
                               SharedCmdMemHandler &cmd,
                               SharedFCStatusHandler &fc,
                               SharedNavMemHandler &nav, SharedSFMemHandler &SF,
                               SharedComMemHandler &com)
    : ComponentBase<4>(config, cmd, sysState), cmd_(cmd), sysState_(sysState),
      fc_(fc), nav_(nav), SF_(SF), com_(com),
      TStateMachine(*this,
                    {.id = 0,
                     .RTpriority = 65,
                     .core = config.CompCore,
                     .loopFrequency = TYPES::Hz{20},
                     .timeout = TYPES::Ms{500}},
                    localWD),
      TMiSequencer(*this,
                   {.id = 1,
                    .RTpriority = 65,
                    .core = config.CompCore,
                    .loopFrequency = TYPES::Hz{10},
                    .timeout = TYPES::Ms{500}},
                   localWD),
      TSafetyMonitor(*this,
                     {.id = 2,
                      .RTpriority = 85,
                      .core = config.CompCore,
                      .loopFrequency = TYPES::Hz{20},
                      .timeout = TYPES::Ms{500}},
                     localWD),
      TGCSCmdRouter(*this,
                    {.id = 3,
                     .RTpriority = 65,
                     .core = config.CompCore,
                     .loopFrequency = TYPES::Hz{10},
                     .timeout = TYPES::Ms{500}},
                    localWD)

{
  registerTask(TStateMachine);
  registerTask(TMiSequencer);
  registerTask(TSafetyMonitor);
  registerTask(TGCSCmdRouter);
  activate();
};

void StateMachineTask::loop() {
  auto now = Clock::now();

  // Preuve de vie pour le fallback autonome MavlinkInterface, cf.
  // MISSIONCONTROL.md §9 -- publiée avant toute logique.
  comp.cmd_.setHeartbeat(now);

  MC_CONV::Inputs in{};

  // Seul TGCSCmdRouter dépile gcsCmds, cf. MISSIONCONTROL.md §7.
  if (auto gcs = comp.gcsOrders_.pop()) {
    if (gcs->type == GCSCommandType::Answer)
      in.queryAnswer = std::pair<uint32_t, uint8_t>{gcs->answerSeq, gcs->answerChoice};
    else
      in.gcsCmd = gcs;
  }

  if (comp.state_.load() == MissionState::PreFlight) {
    if (!preflightChecked_) {
      // Code precis leve par runPreflightChecks() elle-meme, cf. MISSIONCONTROL.md §5.
      preflightPassed_ = runPreflightChecks(comp);
      preflightChecked_ = true;
    }
    in.preflightResult = preflightPassed_;
  } else {
    preflightChecked_ = false; // prêt pour la prochaine tentative d'armement
  }

  in.armError = false;
  if (comp.state_.load() == MissionState::Armed) {
    // Armé en attente de confirmation FC depuis plus de kArmTimeout ->
    // erreur. armed absent (pas encore de HEARTBEAT) traité comme non-armé.
    bool armConfirmTimeout = !comp.fc_.getArmed().value_or(false) &&
                            UTILITIES::msBetween(armRequestedAt_, now) > kArmTimeout;

    bool rpmFault = hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::MVP001);

    in.armError = armConfirmTimeout || rpmFault;
  }

  in.recoverableFailureActive = comp.recoverableFailureActive_.load();
  in.unrecoverableFailureActive = comp.unrecoverableFailureActive_.load();
  in.obstacleActive = comp.obstacleActive_.load();
  in.volumeBreached = comp.volumeBreached_.load();
  in.lidarLost = comp.lidarLost_.load();
  in.missionComplete = comp.missionComplete_.load();

  // cf. MISSIONCONTROL.md §4 (landed).
  in.landed = false;
  if (comp.state_.load() == MissionState::Landing && !comp.fc_.getArmed().value_or(true)) {
    auto state = comp.SF_.getDynamicState();
    float speed = state ? state->velNed.norm().v : 0.0f;
    bool stationary = speed < kLandedVelocityThreshold;
    in.landed = stationary;

    if (!stationary && !crashSuspectedActive_) {
      comp.sysState_.raiseCode(CODES::MCA008);
      crashSuspectedActive_ = true;
    } else if (stationary && crashSuspectedActive_) {
      comp.sysState_.clearCode(CODES::MCA008);
      crashSuspectedActive_ = false;
    }
  }

  if (comp.state_.load() == MissionState::Emergency)
    in.emergencyElapsed = UTILITIES::msBetween(emergencyEnteredAt_, now);
  if (holdQueryAt_ != TYPES::TimePoint{})
    in.holdQueryElapsed = UTILITIES::msBetween(holdQueryAt_, now);

  // Rapports des subordonnés : les vider à chaque cycle, sinon les FIFO se
  // remplissent et les push échouent. Cf. MISSIONCONTROL.md §8.
  bool commandRejected = false;
  while (auto report = comp.cmd_.popMavlinkReport())
    commandRejected |= !report->success;
  while (auto report = comp.cmd_.popNavReport())
    commandRejected |= !report->success;

  if (commandRejected && !commandRejectedActive_) {
    comp.sysState_.raiseCode(CODES::MCA011);
    commandRejectedActive_ = true;
  }

  uint32_t pendingSeq = 0;
  if (auto q = comp.cmd_.getPendingQuery())
    pendingSeq = q->seq;

  auto outcome = MC_CONV::step(comp.state_.load(), previousFlightMode_, pendingSeq, in);

  // Refus de push = commande perdue. MCA010 est levé dans tous les cas ; pour
  // les commandes SANS rattrapage (LAND, disarm, set-mode) la transition est
  // en plus bloquée, sinon le système annoncerait un état que le FC n'a jamais
  // reçu. cf. MISSIONCONTROL.md §4.
  bool sensitiveCommandLost = false;
  auto sendFC = [&](CODES::Code code, uint32_t payload, bool sensitive) {
    if (comp.cmd_.pushMavlinkCommand(code, payload))
      return true;
    comp.sysState_.raiseCode(CODES::MCA010);
    if (sensitive)
      sensitiveCommandLost = true;
    return false;
  };

  // cf. MISSIONCONTROL.md §4, "Commande effective du FC".
  if (outcome.next == MissionState::Armed && comp.state_.load() != MissionState::Armed) {
    armRequestedAt_ = now;
    sendFC(CODES::MCO002, 0, /*sensitive=*/true); // set mode
    sendFC(CODES::MCO003, 0, /*sensitive=*/false); // arm : armError rattrape
  }
  if (outcome.next == MissionState::Idle && comp.state_.load() == MissionState::Armed) {
    sendFC(CODES::MCO004, 0, /*sensitive=*/true); // disarm
  }
  if (outcome.next == MissionState::Mission && comp.state_.load() == MissionState::Armed) {
    // Altitude source et garde takeoffComplete_ : MISSIONCONTROL.md §4, sitl/SITL.md §3.
    float takeoffAlt;
    {
      std::lock_guard lk(comp.planMutex_);
      takeoffAlt = comp.plan_.header.takeoffAlt;
    }
    comp.takeoffTargetAlt_.store(takeoffAlt);
    comp.takeoffComplete_.store(false);
    auto altitudeCm = static_cast<uint32_t>(takeoffAlt * 100.0f);
    // takeoffComplete_ reste false si la commande est perdue : le plan ne
    // démarrera pas, donc pas besoin de bloquer la transition.
    sendFC(CODES::MCO001, altitudeCm, /*sensitive=*/false);
  }
  if (outcome.next == MissionState::Landing && comp.state_.load() != MissionState::Landing) {
    sendFC(CODES::MCO006, 0, /*sensitive=*/true); // LAND
  }

  // L'état n'avance pas : MC réessaiera au cycle suivant (20Hz), et le GCS voit
  // que l'ordre n'a pas pris au lieu d'un état mensonger.
  if (sensitiveCommandLost)
    return;

  if ((comp.state_.load() == MissionState::Mission || comp.state_.load() == MissionState::Manual) &&
      outcome.next != comp.state_.load())
    previousFlightMode_ = comp.state_.load();

  if (outcome.next == MissionState::Emergency && comp.state_.load() != MissionState::Emergency)
    emergencyEnteredAt_ = now;

  // cf. MISSIONCONTROL.md §8.
  auto currentQuery = comp.cmd_.getPendingQuery();
  QueryType currentType = currentQuery ? currentQuery->type : QueryType::None;
  QueryType desiredType = outcome.query.value_or(QueryType::None);
  if (desiredType != currentType) {
    comp.cmd_.setPendingQuery(desiredType);
    holdQueryAt_ =
        (desiredType == QueryType::None) ? TYPES::TimePoint{} : now;
  }

  comp.state_.store(outcome.next);
  comp.cmd_.setMissionState(comp.state_.load());
}

void MiSequencerTask::loop() {
  if (comp.state_.load() != MissionState::Mission)
    return; // inactif hors MISSION — pas de pilotage manuel Nav-aware pour l'instant (TODO)

  auto now = Clock::now();

  // cf. sitl/SITL.md §3.
  if (!comp.takeoffComplete_.load()) {
    auto state = comp.SF_.getDynamicState();
    if (!state)
      return; // pas d'état SF dispo, on retente au prochain cycle
    float altUpM = -state->posNed.ned.z; // NED : z négatif = vers le haut
    float altErrorM = std::abs(altUpM - comp.takeoffTargetAlt_.load());
    float vertSpeedMs = std::abs(state->velNed.ned.z);
    if (altErrorM > kTakeoffAltToleranceM || vertSpeedMs > kTakeoffVertSpeedToleranceMs)
      return; // montée toujours en cours
    comp.takeoffComplete_.store(true);
  }

  if (inHold_) {
    if (now < holdUntil_)
      return;
    inHold_ = false;
    ++nextIndex_;
  }

  if (armedSeqPending_ != 0) {
    // cf. MISSIONCONTROL.md §5, sitl/SITL.md §9 bug #2 (navConfirmedEngaged
    // vérifié AVANT l'âge de navCmd, jamais l'inverse).
    bool navHealthy = comp.sysState_.getHealth(ComponentID::Navigation)
                          .value_or(ComponentHealth::DEAD) == ComponentHealth::NOMINAL;
    auto navCmd = comp.nav_.getCommand();
    auto progress = comp.cmd_.getSegmentProgress();
    bool navConfirmedEngaged = progress && progress->seq == armedSeqPending_;
    bool navCmdFresh;
    if (!navConfirmedEngaged) {
      navCmdFresh = true; // pas encore confirme -- latence normale de promotion, pas une panne
    } else {
      navCmdFresh = navCmd && UTILITIES::msBetween(navCmd->ts, now) < kNavStaleThreshold;
    }
    bool navOk = navHealthy && navCmdFresh;

    if (!navOk && !navStaleActive_) {
      comp.sysState_.raiseCode(CODES::MCA005);
      navStaleActive_ = true;
    } else if (navOk && navStaleActive_) {
      comp.sysState_.clearCode(CODES::MCA005);
      navStaleActive_ = false;
    }

    if (!progress || progress->seq != armedSeqPending_ ||
        progress->status != SegmentStatus::Atteint)
      return; // toujours en route vers la cible armée
    ++nextIndex_;
    armedSeqPending_ = 0;
  }

  std::size_t segmentCount;
  {
    std::lock_guard lk(comp.planMutex_);
    segmentCount = comp.plan_.segments.size();
  }
  if (nextIndex_ >= segmentCount) {
    comp.missionComplete_.store(true);
    return;
  }

  MISSIONPLAN::SegmentVariant seg;
  {
    std::lock_guard lk(comp.planMutex_);
    seg = comp.plan_.segments[nextIndex_];
  }

  if (const auto *wp = std::get_if<MISSIONPLAN::WaypointSegment>(&seg)) {
    Position entryWp;
    if (lastTargetWp_) {
      entryWp = *lastTargetWp_;
    } else {
      auto state = comp.SF_.getDynamicState();
      if (!state)
        return; // pas d'état SF dispo, on retente au prochain cycle
      entryWp = state->posNed;
    }

    Position exitWp{Vec3{wp->north, wp->east, wp->alt}};
    NavSegment navSeg{.type = SegmentType::Waypoint,
                      .entryWp = entryWp,
                      .exitWp = exitWp,
                      .speed = MetersPerSec{wp->speed},
                      .targetAltitude = Meters{wp->alt}};

    // Rien n'a jamais été engagé -> Nav n'auto-promeut pas seul (rien dont
    // la complétion pourrait déclencher tryPromote()), amorçage explicite
    // requis. Pour les segments suivants, Nav promeut seul sur Atteint.
    bool firstSegmentEver = comp.cmd_.getEngagedSegment().value_or(NavSegment{}).seq == 0;

    comp.cmd_.setArmedSegment(navSeg);
    auto armed = comp.cmd_.getArmedSegment();
    if (!armed)
      return; // écriture/lecture échouée (timeout shm) — on retente au prochain cycle

    armedSeqPending_ = armed->seq;
    lastTargetWp_ = exitWp;

    if (firstSegmentEver)
      [[maybe_unused]] auto pushed = comp.cmd_.pushNavCommand(CODES::MCO005, 0);
    return;
  }

  if (const auto *hold = std::get_if<MISSIONPLAN::HoldSegment>(&seg)) {
    inHold_ = true;
    holdUntil_ = now + Ms{static_cast<int64_t>(hold->duration * 1000.0f)};
    return;
  }

  // Corridor/Curve (pas de SegmentType côté Nav pour l'instant) et
  // PayloadAction (pas encore routé, cf. discussion archi) : segment sauté,
  // signalé plutôt qu'ignoré en silence.
  comp.sysState_.raiseCode(CODES::MCA001);
  ++nextIndex_;
}

// Surveillance seule, décision dans TStateMachine -- cf. MISSIONCONTROL.md §6.
void SafetyMonitorTask::loop() {
  bool bothGcsLinksLost =
      hasMajorCode(comp.sysState_, ComponentID::SysMonitoring, CODES::SMC002) &&
      hasMajorCode(comp.sysState_, ComponentID::SysMonitoring, CODES::SMC003);

  // MissionControl absent de cette liste par construction, cf. MISSIONCONTROL.md §6/§9.
  static constexpr std::array<ComponentID, 4> kMonitoredComponents{
      ComponentID::Navigation, ComponentID::SensorFusion,
      ComponentID::MavlinkInterface, ComponentID::SysMonitoring};

  bool anyDead = anyComponentHealth(comp.sysState_, kMonitoredComponents, ComponentHealth::DEAD);
  bool anySick = anyComponentHealth(comp.sysState_, kMonitoredComponents, ComponentHealth::SICK);

  comp.recoverableFailureActive_.store(
      hasMajorCode(comp.sysState_, ComponentID::SensorFusion, CODES::SFS002) ||
      hasMajorCode(comp.sysState_, ComponentID::SensorFusion, CODES::SFS005) ||
      hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::MVC004) ||
      hasMajorCode(comp.sysState_, ComponentID::MissionControl, CODES::MCA005) ||
      bothGcsLinksLost || anySick);

  comp.unrecoverableFailureActive_.store(
      hasMajorCode(comp.sysState_, ComponentID::SysMonitoring, CODES::SMO001) ||
      hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::MVP001) ||
      hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::FCP003) ||
      anyDead);

  comp.obstacleActive_.store(hasMajorCode(comp.sysState_, ComponentID::Navigation, CODES::NVA004));
  comp.lidarLost_.store(hasMajorCode(comp.sysState_, ComponentID::SensorFusion, CODES::SFS004));

  // Volume de vol autorisé, cf. MISSIONCONTROL.md §6.
  bool breached = false;
  if (auto state = comp.SF_.getDynamicState()) {
    MISSIONPLAN::MissionHeader hdr;
    {
      std::lock_guard lk(comp.planMutex_);
      hdr = comp.plan_.header;
    }
    float horizDist = std::sqrt(state->posNed.ned.x * state->posNed.ned.x +
                                state->posNed.ned.y * state->posNed.ned.y);
    float altitude = -state->posNed.ned.z; // NED : z négatif en vol -> altitude positive

    if (hdr.navVolumeRadius > 0.0f && horizDist > hdr.navVolumeRadius)
      breached = true;
    if (hdr.navVolumeCeiling > 0.0f && altitude > hdr.navVolumeCeiling)
      breached = true;
  }

  if (breached && !volumeBreachActive_) {
    comp.sysState_.raiseCode(CODES::MCA007);
    volumeBreachActive_ = true;
  } else if (!breached && volumeBreachActive_) {
    comp.sysState_.clearCode(CODES::MCA007);
    volumeBreachActive_ = false;
  }
  comp.volumeBreached_.store(breached);
}

// Seul consommateur de gcsCmds, cf. MISSIONCONTROL.md §7.
void GCSCmdRouterTask::loop() {
  auto gcs = comp.com_.popGCSCommand();
  if (!gcs)
    return;

  switch (gcs->type) {
  case GCSCommandType::Arm:
  case GCSCommandType::Disarm:
  case GCSCommandType::Takeoff:
  case GCSCommandType::Mode:
  case GCSCommandType::Hold:
  case GCSCommandType::Resume:
  case GCSCommandType::Land:
  case GCSCommandType::Answer:
    if (!comp.gcsOrders_.push(*gcs))
      comp.sysState_.raiseCode(CODES::SMC001); // même code que la FIFO GCS pleine, même nature d'échec
    break;
  case GCSCommandType::Waypoint:
    // TODO: pilotage manuel non construit, cf. MISSIONCONTROL.md §9.
    comp.sysState_.raiseCode(CODES::MCA002);
    break;
  case GCSCommandType::Rtl:
    // Pas de RTB en V1, cf. MISSIONCONTROL.md §9.
    comp.sysState_.raiseCode(CODES::MCA003);
    break;
  }
}
