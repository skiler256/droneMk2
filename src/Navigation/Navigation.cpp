#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <iostream>

using namespace TYPES;

Navigation::Navigation(ComponenConfig config,
                       SharedSysStateMemHandler &sysState,
                       SharedNavMemHandler &nav, SharedSFMemHandler& SF)
    : ComponentBase<1>(config, nav, sysState), nav_(nav),SF_(SF), temps(Clock::now()),
      TAP(*this,
          {.id = 0,
           .RTpriority = 70,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{100},
           .timeout = TYPES::Ms{200}},
          localWD)

{
  registerTask(TAP);
  activate();
};

void APTask::loop() {
  // Ms dure = UTILITIES::msBetween(comp.temps, Clock::now());
  std::cout <<"Je suis nav\n";
};