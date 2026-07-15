#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/Components/SensorFusions/SensorsFusion.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/types.hpp"
#include <iostream>
#include <unistd.h>

using namespace TYPES;
using namespace std;

int main() {

  SharedNavMem Nav{};
  SharedSFMem SF{};
  SharedSysStateMem stateMem{};

  ComponenConfig config{.id = ComponentID::Navigation, .CompCore = 0};
  ComponenConfig config_{.id = ComponentID::SensorFusion, .CompCore=0};

  SharedNavMemHandler navHandler(config.id, Us(2000), Nav);
  SharedSFMemHandler sfHandler(config_.id, Us(2000),SF);
  SharedSysStateMemHandler stateHandler(stateMem, Us(2000));

SensorFusions composant_(config_, stateHandler, sfHandler);
  Navigation composant(config, stateHandler, navHandler, sfHandler);
  

  cout << "salam les roya \n";

  while (true) {
    sleep(2);
  }

  return 0;
};
