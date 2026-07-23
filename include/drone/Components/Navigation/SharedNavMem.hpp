#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"

// Consigne de sortie de la loi de correction (cf. Document de Conception
// Logicielle §3.4.3, "Loi de correction" -> Vel_NED, Throttle(Alt),
// Heading) — c'est ce que MavlinkInterface lit pour construire
// SET_POSITION_TARGET_LOCAL_NED.
struct NavCommand {
  TYPES::TimePoint ts{};
  TYPES::Velocity velNed{};        // vx, vy désirés — vz ignoré, cf. altitudeCmd
  TYPES::Radians headingCmd{0.0f}; // cap absolu désiré
  // Altitude absolue désirée, repère NED (négative en vol, cf. Position) —
  // envoyée comme position z, pas vz : plus stable qu'un taux de montée
  // intégré (cf. MAVLINKINTERFACE.md, SET_POSITION_TARGET_LOCAL_NED).
  TYPES::Meters altitudeCmd{0.0f};
};

// Info segment/mode autopilote (engaged/armed, façon FMA) : pas encore
// détaillé, cf. discussion à venir quand on s'attaquera à cette partie.

struct SharedNavMem {
  SharedCompMem compMem;

  struct payload {
    NavCommand cmd{};
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
