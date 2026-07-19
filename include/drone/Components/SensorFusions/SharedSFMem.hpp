#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"
struct SharedSFMem {
  SharedCompMem compMem;
  struct payload {
    int blip;
  };
  payload data;
};

class SharedSFMemHandler : public SharedCompMemHandler {
public:
  explicit SharedSFMemHandler(TYPES::ComponentID id, TYPES::Us timeout,
                              SharedSFMem &SF)
      : SharedCompMemHandler(SF.compMem, id, timeout), SF_(SF) {};

  int getBlip() { // pour test
    auto fetch = getData(id_, timeout_, SF_.data.blip);
    if (!fetch)
      return -1;
    return fetch.value();
  }

  void setBlip(int a) { setData(id_, timeout_, SF_.data.blip, a); };

private:
  SharedSFMem &SF_;

  uint32_t computeChecksum(TYPES::ComponentID) {
    return UTILITIES::crc32(SF_.data) ^ historyChecksum();
  };

  // Seul le propriétaire peut reconstruire sa propre shm.
  void reset(TYPES::ComponentID id) {
    if (id == TYPES::ComponentID::SensorFusion) {
      SF_.data = SharedSFMem::payload{};
      sanitizeHistory(comp_.HotStartHistory);
      sanitizeHistory(comp_.ColdStartHistory);
      updateChecksum(id);
    }
  };
};