#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"

// Consigne de sortie de la loi de correction (cf. Document de Conception
// Logicielle §3.4.3, "Loi de correction" -> Vel_NED, Throttle(Alt),
// Heading) — c'est ce que MavlinkInterface lit pour construire
// SET_POSITION_TARGET_LOCAL_NED.
struct NavCommand {
  TYPES::TimePoint ts{};
  // vx, vy, vz — les 3 composantes sont envoyées telles quelles à
  // MavlinkInterface (SET_POSITION_TARGET_LOCAL_NED). vz vient du
  // P-controller altitude de Nav (NAV_CONV::computeVerticalGuidance).
  TYPES::Velocity velNed{};
  TYPES::Radians headingCmd{0.0f}; // cap absolu désiré
  // Altitude absolue visée, repère NED (négative en vol, cf. Position) —
  // indicatif seulement (affichage GCS via UDP_bridge) : la commande
  // réelle envoyée au FC passe par velNed.ned.z, pas ce champ.
  TYPES::Meters altitudeCmd{0.0f};
};

struct SharedNavMem {
  SharedCompMem compMem;

  // Affichage FMA (armé/engagé par axe), indicatif seulement — pas via
  // sysState_.raiseCode/clearCode. À plat pour que TSeq/TAP écrivent
  // chacun le leur sans se marcher dessus. Cf. NAVIGATION.md §7.
  struct payload {
    NavCommand cmd{};
    CODES::Code lateralArmed{};
    CODES::Code lateralEngaged{};
    CODES::Code verticalArmed{};
    CODES::Code verticalEngaged{};
    CODES::Code speedEngaged{};
  };
  payload data;
};

class SharedNavMemHandler : public SharedCompMemHandler {
public:
  explicit SharedNavMemHandler(TYPES::ComponentID id, TYPES::Us timeout,
                               SharedNavMem &nav)
      : SharedCompMemHandler(nav.compMem, id, timeout), nav_(nav) {};

  void setCommand(NavCommand c) { setData(id_, timeout_, nav_.data.cmd, c); }

  [[nodiscard]] std::optional<NavCommand> getCommand() {
    return getData(id_, timeout_, nav_.data.cmd);
  }

  // FMA — écrit par TSeq (latéral) et TAP (vertical/vitesse), lu par le GCS.
  void setLateralArmed(CODES::Code c) { setData(id_, timeout_, nav_.data.lateralArmed, c); }
  void setLateralEngaged(CODES::Code c) { setData(id_, timeout_, nav_.data.lateralEngaged, c); }
  void setVerticalArmed(CODES::Code c) { setData(id_, timeout_, nav_.data.verticalArmed, c); }
  void setVerticalEngaged(CODES::Code c) { setData(id_, timeout_, nav_.data.verticalEngaged, c); }
  void setSpeedEngaged(CODES::Code c) { setData(id_, timeout_, nav_.data.speedEngaged, c); }

  [[nodiscard]] std::optional<CODES::Code> getLateralArmed() {
    return getData(id_, timeout_, nav_.data.lateralArmed);
  }
  [[nodiscard]] std::optional<CODES::Code> getLateralEngaged() {
    return getData(id_, timeout_, nav_.data.lateralEngaged);
  }
  [[nodiscard]] std::optional<CODES::Code> getVerticalArmed() {
    return getData(id_, timeout_, nav_.data.verticalArmed);
  }
  [[nodiscard]] std::optional<CODES::Code> getVerticalEngaged() {
    return getData(id_, timeout_, nav_.data.verticalEngaged);
  }
  [[nodiscard]] std::optional<CODES::Code> getSpeedEngaged() {
    return getData(id_, timeout_, nav_.data.speedEngaged);
  }

private:
  SharedNavMem &nav_;

  uint32_t computeChecksum(TYPES::ComponentID) {
    return UTILITIES::crc32(nav_.data) ^ historyChecksum();
  };

  void reset(TYPES::ComponentID id) {
    // Seul le propriétaire peut reconstruire sa propre shm.
    if (id == TYPES::ComponentID::Navigation) {
      nav_.data = SharedNavMem::payload{};
      sanitizeHistory(comp_.HotStartHistory);
      sanitizeHistory(comp_.ColdStartHistory);
      updateChecksum(id);
    }
  };
};
