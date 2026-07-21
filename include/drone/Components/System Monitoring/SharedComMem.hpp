#pragma once
#include "drone/Components/SharedCompMem.hpp"

struct SharedComMem {
SharedCompMem compMem;
struct payload {
  // commande GCS ...
};
payload data;
};

class SharedComMemHandler : public SharedCompMemHandler {
public:
  explicit SharedComMemHandler(TYPES::ComponentID id, TYPES::Us timeout,
                              SharedComMem &Com)
      : SharedCompMemHandler(Com.compMem, id, timeout), Com_(Com) {};

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