#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <iostream>

using namespace TYPES;

Navigation::Navigation(ComponenConfig config,
                       SharedSysStateMemHandler &sysState,
                       SharedNavMemHandler &nav, SharedSFMemHandler& SF)
    : ComponentBase(config, nav, sysState), nav_(nav),SF_(SF), temps(Clock::now()),
      TAP(*this,
          {.id = 0,
           .RTpriority = 70,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{100},
           .timeout = TYPES::Ms{200}},
          localWD)

{
  if (hotstart) {
    restore();
    init();
  } else if (coldstart) {
    init();
  }
};

void Navigation::restore() {

};

void Navigation::init() { TAP.start(); };

void APTask::loop() {
  // Ms dure = UTILITIES::msBetween(comp.temps, Clock::now());
  auto a = comp.SF_.getBlip();
  std::cout <<"Je suis nav :" <<a << "\n";
};