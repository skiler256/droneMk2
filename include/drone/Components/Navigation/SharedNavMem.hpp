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

      uint32_t computeChecksum() {
return UTILITIES::crc32(nav_.data);
};

void reset(){};
};