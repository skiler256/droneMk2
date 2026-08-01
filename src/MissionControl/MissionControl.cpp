#include "drone/Components/MissionControl/MissionControl.hpp"
#include "drone/Components/MissionControl/Conversions.hpp"
#include "drone/generated/codes.hpp"
#include "drone/utilities.hpp"
#include <cstdlib>
#include <fstream>

using namespace TYPES;

namespace {

// Chemin fixe pour l'instant — pas d'upload GCS -> disque, pas de hot-swap
// (décidé explicitement, "on fait simple" en attendant). À revoir quand le
// pipeline d'upload sera construit. Surchargeable par variable
// d'environnement (DRONE_MISSION_PLAN_PATH) pour les tests (mc_sitl) —
// écrire dans /etc/drone en dev/CI demande souvent root, comportement de
// prod inchangé si la variable n'est pas définie.
[[nodiscard]] std::string missionPlanPath() {
  if (const char *override = std::getenv("DRONE_MISSION_PLAN_PATH"))
    return override;
  return "/etc/drone/missionplan.bin";
}

// Poignée de main matérielle bornée (le FC met un temps fini à confirmer
// l'armement) — PAS le genre de timeout écarté par l'utilisateur pour les
// segments de mission (ceux-là jugés sur la santé de Nav, pas la durée).
constexpr Ms kArmTimeout{5000};

// Nav publie un NavCommand frais à chaque cycle APTask (50Hz) quand il a
// quelque chose à piloter — largement au-dessus de ce seuil si Nav est
// bloqué/mort plutôt que juste "un segment un peu long".
constexpr Ms kNavStaleThreshold{500};

// En dessous de cette vitesse pendant LANDING+désarmé : posé normalement.
// Au-dessus : désarmé en plein mouvement, suspicion de crash plutôt qu'un
// atterrissage — valeur de départ arbitraire, à affiner en vol réel.
constexpr float kLandedVelocityThreshold = 1.0f;

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

// S'appuie sur le mécanisme déjà en place (ComponentBase, cf.
// ComponenConfig::max_hot_start/max_cold_start) plutôt que de recompter
// les crashs soi-même : DEAD = ComponentBase a renoncé à démarrer les Task
// de ce composant (trop de cold-starts), ne reviendra pas sans
// intervention. SICK = a dû cold-start mais a fini par s'initialiser —
// tourne, juste secoué.
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

// Charge + valide le plan de mission (MISSIONPLAN_CONV::parse fait déjà
// toutes les vérifications de format : magic/version/CRC/segments, cf.
// test_missionplan.cpp) — écrit comp.plan_ si succès.
[[nodiscard]] bool runPreflightChecks(MissionControl &comp) {
  auto bytes = readFile(missionPlanPath());
  if (!bytes)
    return false;
  auto plan = MISSIONPLAN_CONV::parse(*bytes);
  if (!plan)
    return false;
  comp.plan_ = *plan;
  return true;
}

} // namespace

// Fréquences/priorités : valeurs de départ arbitraires (même démarche que
// Navigation cette session), à ajuster une fois chaque Task réellement
// implémentée. TSafetyMonitor la plus prioritaire : c'est la Task dont un
// retard a le plus grand impact sécurité (cf. CLAUDE.md, MC "le plus critique").
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

  // Preuve de vie — lue par MavlinkInterface (FSTask) pour un fallback
  // autonome (LAND) si TStateMachine s'arrête, cf. MAVLINKINTERFACE.md.
  // Publiée inconditionnellement, avant toute logique : un blocage plus
  // bas dans ce cycle ne doit pas empêcher ce heartbeat d'être déjà parti.
  comp.cmd_.setHeartbeat(now);

  MC_CONV::Inputs in{};

  // Seul TGCSCmdRouter dépile SharedComMem::gcsCmds (cf. MissionControl.hpp,
  // StateMachineOrderQueue) — ici on ne lit que ce qu'il a déjà classé
  // comme concernant la state machine.
  if (auto gcs = comp.gcsOrders_.pop()) {
    if (gcs->type == GCSCommandType::Answer)
      in.queryAnswer = std::pair<uint32_t, uint8_t>{gcs->answerSeq, gcs->answerChoice};
    else
      in.gcsCmd = gcs;
  }

  if (comp.state_ == MissionState::PreFlight) {
    if (!preflightChecked_) {
      preflightPassed_ = runPreflightChecks(comp);
      preflightChecked_ = true;
      if (!preflightPassed_)
        comp.sysState_.raiseCode(CODES::MCA004);
    }
    in.preflightResult = preflightPassed_;
  } else {
    preflightChecked_ = false; // prêt pour la prochaine tentative d'armement
  }

  in.armError = false;
  if (comp.state_ == MissionState::Armed) {
    // Armé en attente de confirmation FC depuis plus de kArmTimeout ->
    // erreur. armed absent (pas encore de HEARTBEAT) traité comme non-armé.
    bool armConfirmTimeout = !comp.fc_.getArmed().value_or(false) &&
                            UTILITIES::msBetween(armRequestedAt_, now) > kArmTimeout;

    // RPM bas/donnée ESC absente : détecté et publié par TFSTask (MV, cf.
    // MavlinkInterface.cpp) — ici on ne fait que lire le code MVP001.
    bool rpmFault = hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::MVP001);

    in.armError = armConfirmTimeout || rpmFault;
  }

  in.recoverableFailureActive = comp.recoverableFailureActive_;
  in.unrecoverableFailureActive = comp.unrecoverableFailureActive_;
  in.obstacleActive = comp.obstacleActive_;
  in.volumeBreached = comp.volumeBreached_;
  in.lidarLost = comp.lidarLost_;
  in.missionComplete = comp.missionComplete_;

  // Le FC désarme seul en fin d'atterrissage (LAND_DISARMDELAY côté
  // ArduPilot) — armed=false pendant LANDING est un premier signal, mais
  // pas suffisant seul : un désarmement en plein vol (crash) le
  // déclencherait aussi. Vérification supplémentaire sur la vitesse —
  // cf. kLandedVelocityThreshold.
  in.landed = false;
  if (comp.state_ == MissionState::Landing && !comp.fc_.getArmed().value_or(true)) {
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

  if (comp.state_ == MissionState::Emergency)
    in.emergencyElapsed = UTILITIES::msBetween(emergencyEnteredAt_, now);

  uint32_t pendingSeq = 0;
  if (auto q = comp.cmd_.getPendingQuery())
    pendingSeq = q->seq;

  auto outcome = MC_CONV::step(comp.state_, previousFlightMode_, pendingSeq, in);

  // Commande effective au FC sur les transitions qui l'exigent — MC_CONV
  // décide l'état, c'est ici qu'on parle vraiment à MavlinkInterface.
  if (outcome.next == MissionState::Armed && comp.state_ != MissionState::Armed) {
    armRequestedAt_ = now;
    [[maybe_unused]] auto pushed = comp.cmd_.pushMavlinkCommand(CODES::MCO003, 0);
  }
  if (outcome.next == MissionState::Idle && comp.state_ == MissionState::Armed) {
    [[maybe_unused]] auto pushed = comp.cmd_.pushMavlinkCommand(CODES::MCO004, 0);
  }
  if (outcome.next == MissionState::Mission && comp.state_ == MissionState::Armed) {
    auto altitudeCm = in.gcsCmd ? static_cast<uint32_t>(in.gcsCmd->altitude.v * 100.0f) : 0u;
    [[maybe_unused]] auto pushed = comp.cmd_.pushMavlinkCommand(CODES::MCO001, altitudeCm);
  }
  if (outcome.next == MissionState::Landing && comp.state_ != MissionState::Landing) {
    [[maybe_unused]] auto pushed = comp.cmd_.pushMavlinkCommand(CODES::MCO006, 0);
  }

  if ((comp.state_ == MissionState::Mission || comp.state_ == MissionState::Manual) &&
      outcome.next != comp.state_)
    previousFlightMode_ = comp.state_;

  if (outcome.next == MissionState::Emergency && comp.state_ != MissionState::Emergency)
    emergencyEnteredAt_ = now;

  // Publication de la question au GCS — générique sur outcome.query (type
  // de question qui DEVRAIT être active ce cycle, cf. Conversions.hpp),
  // pas câblé sur un état particulier : couvre EMERGENCY (recoverable) et
  // HOLD (LidarLost) avec la même logique. Ne republie que sur changement
  // de type, sinon chaque cycle romprait la corrélation de seq avec
  // CMD_ANSWER (setPendingQuery incrémente toujours le seq).
  auto currentQuery = comp.cmd_.getPendingQuery();
  QueryType currentType = currentQuery ? currentQuery->type : QueryType::None;
  QueryType desiredType = outcome.query.value_or(QueryType::None);
  if (desiredType != currentType)
    comp.cmd_.setPendingQuery(desiredType);

  comp.state_ = outcome.next;
  comp.cmd_.setMissionState(comp.state_);
}

void MiSequencerTask::loop() {
  if (comp.state_ != MissionState::Mission)
    return; // inactif hors MISSION — pas de pilotage manuel Nav-aware pour l'instant (TODO)

  auto now = Clock::now();

  if (inHold_) {
    if (now < holdUntil_)
      return;
    inHold_ = false;
    ++nextIndex_;
  }

  if (armedSeqPending_ != 0) {
    // Pas de timeout sur la durée du segment (décidé explicitement : tant
    // que Nav est sain, un segment un peu long ne doit rien déclencher) —
    // on juge Nav sur sa santé et la fraîcheur de son dernier NavCommand.
    bool navHealthy = comp.sysState_.getHealth(ComponentID::Navigation)
                          .value_or(ComponentHealth::DEAD) == ComponentHealth::NOMINAL;
    auto navCmd = comp.nav_.getCommand();
    bool navCmdFresh = navCmd && UTILITIES::msBetween(navCmd->ts, now) < kNavStaleThreshold;
    bool navOk = navHealthy && navCmdFresh;

    if (!navOk && !navStaleActive_) {
      comp.sysState_.raiseCode(CODES::MCA005);
      navStaleActive_ = true;
    } else if (navOk && navStaleActive_) {
      comp.sysState_.clearCode(CODES::MCA005);
      navStaleActive_ = false;
    }

    auto progress = comp.cmd_.getSegmentProgress();
    if (!progress || progress->seq != armedSeqPending_ ||
        progress->status != SegmentStatus::Atteint)
      return; // toujours en route vers la cible armée
    ++nextIndex_;
    armedSeqPending_ = 0;
  }

  if (nextIndex_ >= comp.plan_.segments.size()) {
    comp.missionComplete_ = true;
    return;
  }

  const auto &seg = comp.plan_.segments[nextIndex_];

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

// Surveillance seule — la décision (bascule d'état) reste dans
// TStateMachine, cf. comp.recoverableFailureActive_/unrecoverableFailureActive_/
// obstacleActive_ (même séparation que TFSTask chez Nav/MavlinkInterface).
void SafetyMonitorTask::loop() {
  // Codes "emergency-worthy" seulement — cf. Conversions.hpp : NVA004
  // (obstacle) et NVA005 (ekf stale côté Nav) sont volontairement exclus,
  // ils déclenchent un HOLD (Nav se freine déjà seul via holdActive_), pas
  // une bascule système en EMERGENCY. SFS003 (Mag_lost) volontairement
  // absent : le magnétomètre est sur le FC depuis la bascule d'archi, plus
  // surveillé par SensorFusion — code orphelin, cf. SENSORFUSION.md §7.
  //
  // Perte GCS : un seul lien perdu (l'autre tient) n'est qu'informatif —
  // le lien restant fait déjà office de secours (cf. SYSMONITORING.md).
  // Seule la perte SIMULTANÉE des deux devient un échec système : si le
  // GCS revient dans les 30s, EMERGENCY lui demande "reprendre ou
  // atterrir ?" ; sinon, timeout -> LANDING automatique (cf. §3/§8) — ça
  // couvre naturellement le cas où le lien est down et personne ne peut
  // répondre à la question.
  bool bothGcsLinksLost =
      hasMajorCode(comp.sysState_, ComponentID::SysMonitoring, CODES::SMC002) &&
      hasMajorCode(comp.sysState_, ComponentID::SysMonitoring, CODES::SMC003);

  // Composants surveillés pour l'escalade sur santé (pas MissionControl
  // lui-même : s'il est mort, il n'est pas là pour se surveiller — c'est
  // à MavlinkInterface de commander LAND de façon autonome dans ce cas,
  // cf. MISSIONCONTROL.md §9, pas encore construit).
  static constexpr std::array<ComponentID, 4> kMonitoredComponents{
      ComponentID::Navigation, ComponentID::SensorFusion,
      ComponentID::MavlinkInterface, ComponentID::SysMonitoring};

  bool anyDead = anyComponentHealth(comp.sysState_, kMonitoredComponents, ComponentHealth::DEAD);
  bool anySick = anyComponentHealth(comp.sysState_, kMonitoredComponents, ComponentHealth::SICK);

  // Recoverable : peut revenir tout seul (GPS/EKF/lien FC/GCS), ou via un
  // hot-start (Nav, SICK = a dû cold-start mais tourne) — vaut la peine
  // d'attendre/demander au GCS.
  comp.recoverableFailureActive_ =
      hasMajorCode(comp.sysState_, ComponentID::SensorFusion, CODES::SFS002) ||
      hasMajorCode(comp.sysState_, ComponentID::SensorFusion, CODES::SFS005) ||
      hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::MVC004) ||
      hasMajorCode(comp.sysState_, ComponentID::MissionControl, CODES::MCA005) ||
      bothGcsLinksLost || anySick;

  // Unrecoverable : aucune attente possible, atterrissage direct.
  // - E-stop GCS : l'opérateur a demandé l'arrêt, pas la peine de lui
  //   redemander "reprendre ou atterrir ?" juste après (cf. SDD §4).
  // - RPM bas / batterie critique : panne matérielle qui ne se résout pas
  //   en vol (ex: l'ESC coupe les moteurs sous un seuil de tension).
  // - anyDead : ComponentBase a renoncé à démarrer ce composant, ne
  //   reviendra pas tout seul.
  comp.unrecoverableFailureActive_ =
      hasMajorCode(comp.sysState_, ComponentID::SysMonitoring, CODES::SMO001) ||
      hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::MVP001) ||
      hasMajorCode(comp.sysState_, ComponentID::MavlinkInterface, CODES::FCP003) ||
      anyDead;

  comp.obstacleActive_ = hasMajorCode(comp.sysState_, ComponentID::Navigation, CODES::NVA004);
  // LIDAR perdu : ni recoverable ni unrecoverable — HOLD + question au
  // GCS (cf. Conversions.hpp, discussion : pas une panne qui menace le
  // vol lui-même, juste la capacité d'évitement d'obstacle).
  comp.lidarLost_ = hasMajorCode(comp.sysState_, ComponentID::SensorFusion, CODES::SFS004);

  // Volume de vol autorisé — cahier des charges SDD §1.2 ("maintenir le
  // drone à l'intérieur de la zone d'opération"). Rayon/plafond à 0 (plan
  // pas encore chargé) désactive le check plutôt que de déclencher sur un
  // volume par défaut nul.
  bool breached = false;
  if (auto state = comp.SF_.getDynamicState()) {
    const auto &hdr = comp.plan_.header;
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
  comp.volumeBreached_ = breached;
}

// Seul consommateur de SharedComMem::gcsCmds — cf. StateMachineOrderQueue
// (MissionControl.hpp) : un deuxième point de dépilement volerait des
// commandes qui ne lui étaient pas destinées.
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
    // TODO: pilotage manuel — décision prise de le construire comme une
    // consigne velNED envoyée en direct par le GCS (pas un enchaînement de
    // waypoints), reporté après validation des premiers tests
    // d'intégration. CMD_WAYPOINT n'a donc pour l'instant aucune
    // destination (ni ce format, ni Nav côté réception).
    comp.sysState_.raiseCode(CODES::MCA002);
    break;
  case GCSCommandType::Rtl:
    // Pas de RTB en V1 (décidé explicitement) — CMD_RTL reste un paquet
    // valide au niveau fil, mais rien ne le consomme côté state machine.
    comp.sysState_.raiseCode(CODES::MCA003);
    break;
  }
}
