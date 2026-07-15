#include "drone/Components/Navigation/Navigation.hpp"
#include "drone/types.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <iostream>

using namespace TYPES;

Navigation::Navigation(ComponenConfig config,SharedSysStateMemHandler& sysState, SharedNavMemHandler& nav ):
ComponentBase(config, nav, sysState),
nav_(nav),
     temps(Clock::now()),
TAP(*this, {.id           = 0,
             .RTpriority   = 70,
             .loopFrequency = TYPES::Hz{103},
             .timeout       = TYPES::Ms{200}}, localWD)
        
{
    if(hotstart) {
        restore();
        init();
    }else if (coldstart) {
        init();
    }
};

void Navigation::restore(){

};

void Navigation::init(){
    TAP.start();

};

void APTask::loop(){
    Ms dure =  UTILITIES::msBetween(comp.temps, Clock::now());
    std::cout<<dure.count()<<"\n";
};