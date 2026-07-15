#include "drone/Components/SensorFusions/SensorsFusion.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <iostream>

using namespace TYPES;

SensorFusions::SensorFusions(ComponenConfig config,
                       SharedSysStateMemHandler &sysState,
                       SharedSFMemHandler &sf)
    : ComponentBase(config, sf, sysState), SF_(sf), temps(Clock::now()),
      TGPS(*this,
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

void SensorFusions::restore() {

};

void SensorFusions::init() { TGPS.start(); };

void GPSTask::loop() {
  Ms dure = UTILITIES::msBetween(comp.temps, Clock::now());
  comp.SF_.setBlip(static_cast<int>(dure.count())*2);
  std::cout <<"Je suis SensorFusion " <<dure.count() << "\n";
};