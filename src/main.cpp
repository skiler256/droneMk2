#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/types.hpp"
#include <iostream>
#include <unistd.h>

using namespace TYPES;
using namespace std;

int main(){

    SharedNavMem Nav{};
    SharedSysStateMem stateMem{};

    ComponenConfig config{
        .id = ComponentID::Navigation,
    };

    SharedNavMemHandler navHandler(config.id, Us(2000),Nav);
    SharedSysStateMemHandler stateHandler(stateMem, Us(2000));

    
    config.id = ComponentID::Navigation;

    Navigation composant(config,stateHandler,navHandler);

    cout<<"salam les roya \n";

    while (true) {
    sleep(2);
    }

    return 0;
};
