#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/Components/Navigation/Conversions.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"

using namespace TYPES;

namespace {
// cf. NAVIGATION.md §5.
constexpr Meters kArrivalTolerance{0.5f};

// Redondant au seuil d'EKFTask côté SF, volontairement plus large -- pas
// une simple relecture du même check, cf. NAVIGATION.md §6.
constexpr Ms kEkfStaleThreshold{400};
} // namespace

Navigation::Navigation(ComponenConfig config, SharedSysStateMemHandler &sysState,
                       SharedNavMemHandler &nav, SharedSFMemHandler &SF,
                       SharedCmdMemHandler &cmd)
    : ComponentBase<3>(config, nav, sysState), nav_(nav), SF_(SF),
      sysState_(sysState), cmd_(cmd),
      TSeq(*this,
          {.id = 0,
           .RTpriority = 65,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{20}, // séquencement, pas de contrainte de latence serrée
           .timeout = TYPES::Ms{500}},
          localWD),
      TAP(*this,
          {.id = 1,
           .RTpriority = 70,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{50},
           .timeout = TYPES::Ms{200}},
          localWD),
      TFS(*this,
          {.id = 2,
           .RTpriority = 80,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{20},
           .timeout = TYPES::Ms{500}},
          localWD)

{
  registerTask(TSeq);
  registerTask(TAP);
  registerTask(TFS);
  activate();
};

// cf. NAVIGATION.md §4.
bool TSeqTask::tryPromote() {
  if (comp.armedSegment_.seq == 0 || comp.armedSegment_.seq == comp.engagedSegment_.seq)
    return false;

  auto state = comp.SF_.getDynamicState();
  if (!state)
    return false;

  if (!NAV_CONV::canEngage(comp.armedSegment_, *state))
    return false;

  comp.cmd_.setEngagedSegment(comp.armedSegment_);
  return true;
}

void TSeqTask::loop() {
  if (auto seg = comp.cmd_.getEngagedSegment()) {
    if (seg->seq != comp.lastSegmentSeq_) {
      comp.engagedSegment_ = *seg;
      comp.lastSegmentSeq_ = seg->seq;
      comp.newSegment_ = true;
    }
  }

  if (auto armed = comp.cmd_.getArmedSegment()) {
    comp.armedSegment_ = *armed;
  }

  if (auto order = comp.cmd_.popNavCommand()) {
    if (order->code == CODES::MCO005) {
      // Amorçage — même chemin que la promotion automatique sur Atteint.
      bool ok = tryPromote();
      [[maybe_unused]] auto pushed =
          comp.cmd_.pushNavReport(order->seq, order->code, ok);
    } else {
      // Ordres pas encore définis, cf. NAVIGATION.md §4.
      [[maybe_unused]] auto pushed =
          comp.cmd_.pushNavReport(order->seq, order->code, false);
    }
  }

  // Auto-séquencement : le segment engagé vient-il d'être atteint ?
  if (auto progress = comp.cmd_.getSegmentProgress()) {
    if (progress->status == SegmentStatus::Atteint)
      tryPromote();
  }

  comp.nav_.setLateralArmed(comp.armedSegment_.seq != 0
                                ? NAV_CONV::lateralModeCode(comp.armedSegment_.type)
                                : CODES::Code{});
  comp.nav_.setLateralEngaged(comp.engagedSegment_.seq != 0
                                  ? NAV_CONV::lateralModeCode(comp.engagedSegment_.type)
                                  : CODES::Code{});
}

void APTask::loop() {
  auto now = Clock::now();

  if (comp.holdActive_) {
    // TFS a demandé un hold : consigne vitesse nulle, cap/alt figés à
    // l'état courant plutôt que de continuer la loi de correction.
    auto state = comp.SF_.getDynamicState();
    NavCommand hold{.ts = now};
    if (state) {
      // Pas de négation : même convention NED que NavCommand::altitudeCmd.
      hold.altitudeCmd = Meters{state->posNed.ned.z};
    }
    comp.nav_.setCommand(hold);
    comp.nav_.setVerticalEngaged(CODES::NVA007); // Hold
    return;
  }

  if (comp.engagedSegment_.seq == 0)
    return; // rien engagé -- cf. NAVIGATION.md §5

  auto state = comp.SF_.getDynamicState();
  if (!state)
    return; // pas d'état SF disponible ce cycle

  if (comp.newSegment_) {
    // Reset des états internes au changement de segment — rien à reset
    // pour l'instant, le P-controller de computeWaypointGuidance() n'a pas
    // de mémoire (pas d'intégrateur).
    comp.newSegment_ = false;
  }

  auto vertical = NAV_CONV::computeVerticalGuidance(
      comp.engagedSegment_.targetAltitude, Meters{state->posNed.ned.z});

  switch (comp.engagedSegment_.type) {
  case SegmentType::Waypoint: {
    auto guidance = NAV_CONV::computeWaypointGuidance(
        comp.engagedSegment_, *state, now, kArrivalTolerance);
    guidance.cmd.velNed.ned.z = vertical.vz.v;
    comp.nav_.setCommand(guidance.cmd);

    SegmentProgress progress{
        .seq = comp.engagedSegment_.seq,
        .status = guidance.arrived ? SegmentStatus::Atteint
                                   : SegmentStatus::EnCours,
        .distanceRestante = guidance.distanceRemaining,
        .ts = now};
    comp.cmd_.setSegmentProgress(progress);
    break;
  }
  }

  // Vitesse toujours proportionnelle (seule loi implémentée, cf. NAVIGATION.md §10).
  comp.nav_.setVerticalEngaged(vertical.modeCode);
  comp.nav_.setSpeedEngaged(CODES::NVA010);
}

void TFSTask::loop() {
  auto now = Clock::now();

  auto state = comp.SF_.getDynamicState();
  bool ekfStale =
      !state || UTILITIES::msBetween(state->ts, now) > kEkfStaleThreshold;

  if (ekfStale && !ekfStaleActive_) {
    comp.sysState_.raiseCode(CODES::NVA005);
    ekfStaleActive_ = true;
  } else if (!ekfStale && ekfStaleActive_) {
    comp.sysState_.clearCode(CODES::NVA005);
    ekfStaleActive_ = false;
  }

  bool obstacle = false;
  if (state) {
    if (auto map = comp.SF_.getLidarMap()) {
      Vec3 forward = NAV_CONV::travelDirection(comp.engagedSegment_, *state);
      obstacle = NAV_CONV::obstacleInBox(*map, forward);
    }
  }

  if (obstacle && !obstacleActive_) {
    comp.sysState_.raiseCode(CODES::NVA004);
    obstacleActive_ = true;
  } else if (!obstacle && obstacleActive_) {
    comp.sysState_.clearCode(CODES::NVA004);
    obstacleActive_ = false;
  }

  comp.holdActive_ = ekfStaleActive_ || obstacleActive_;
}
