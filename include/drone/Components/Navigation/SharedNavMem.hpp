#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"
struct SharedNavMem {
    SharedCompMem compMem;

    struct payload {
    int bloup;
    };

    payload data;

};

class SharedNavMemHandler : public SharedCompMemHandler {
    public:
    explicit SharedNavMemHandler(TYPES::ComponentID id,TYPES::Us timeout,SharedNavMem& nav):
    SharedCompMemHandler(nav.compMem, id, timeout),
    nav_(nav){};
    

    int getBloup(){ // pour test
        return nav_.data.bloup;
    }

    private:
    SharedNavMem& nav_;

      uint32_t computeChecksum(TYPES::ComponentID) {
return UTILITIES::crc32(nav_.data) ^ historyChecksum();
};

void reset(TYPES::ComponentID id){
    // Seul le propriétaire peut reconstruire sa propre shm.
    if(id == TYPES::ComponentID::Navigation){
        nav_.data = SharedNavMem::payload{};
        sanitizeHistory(comp_.HotStartHistory);
        sanitizeHistory(comp_.ColdStartHistory);
        updateChecksum(id);
    }
};
};