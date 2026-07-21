#pragma once

#include "drone/Components/SharedCompMem.hpp"

// Payload pas encore détaillé (cf. MAVLINKINTERFACE.md §10) — sera rempli
// quand TRx sera implémentée (ATTITUDE, LOCAL_POSITION_NED,
// BATTERY_STATUS, ESC_STATUS, SCALED_PRESSURE, santé EKF...).
struct SharedFCStatus {
  SharedCompMem compMem;
  struct payload {};
  payload data;
};

class SharedFCStatusHandler : public SharedCompMemHandler {
public:
  explicit SharedFCStatusHandler(TYPES::ComponentID id, TYPES::Us timeout,
                                 SharedFCStatus &fc)
      : SharedCompMemHandler(fc.compMem, id, timeout), fc_(fc) {};

private:
  SharedFCStatus &fc_;

  uint32_t computeChecksum(TYPES::ComponentID) {
    return UTILITIES::crc32(fc_.data) ^ historyChecksum();
  };

  void reset(TYPES::ComponentID id) {
    if (id == TYPES::ComponentID::MavlinkInterface) {
      fc_.data = SharedFCStatus::payload{};
      sanitizeHistory(comp_.HotStartHistory);
      sanitizeHistory(comp_.ColdStartHistory);
      updateChecksum(id);
    }
  };
};
