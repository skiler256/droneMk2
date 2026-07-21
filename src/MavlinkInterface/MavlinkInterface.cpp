#include "drone/Components/MavlinkInterface/MavlinkInterface.hpp"
#include "drone/types.hpp"

using namespace TYPES;

// RT priorities et fréquences : placeholders raisonnables, à ajuster
// quand les loop() seront implémentées (étape suivante) — rien de figé ici.
MavlinkInterface::MavlinkInterface(ComponenConfig config,
                                   SharedSysStateMemHandler &sysState,
                                   SharedFCStatusHandler &fc, SharedNavMemHandler &nav,
                                   IMavlinkLink &link)
    : ComponentBase<5>(config, fc, sysState), fc_(fc), sysState_(sysState),
      nav_(nav), link_(link),
      TTx(*this,
          {.id = 0,
           .RTpriority = 75,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{50}, // couvre SET_POSITION_TARGET_LOCAL_NED (50Hz)
           .timeout = TYPES::Ms{200}},
          localWD),
      TTxACK(*this,
             {.id = 1,
              .RTpriority = 70,
              .core = config.CompCore,
              .loopFrequency = TYPES::Hz{20},
              .timeout = TYPES::Ms{1000}}, // laisse la place au cycle retry 500ms
             localWD),
      TRx(*this,
          {.id = 2,
           .RTpriority = 75,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{100}, // marge sur le plus rapide des flux RX (50Hz)
           .timeout = TYPES::Ms{200}},
          localWD),
      TCmd(*this,
           {.id = 3,
            .RTpriority = 65,
            .core = config.CompCore,
            .loopFrequency = TYPES::Hz{50},
            .timeout = TYPES::Ms{200}},
           localWD),
      TFailSafe(*this,
                {.id = 4,
                 .RTpriority = 80, // supervision failsafe : priorité la plus haute des 5
                 .core = config.CompCore,
                 .loopFrequency = TYPES::Hz{20},
                 .timeout = TYPES::Ms{500}},
                localWD)

{
  registerTask(TTx);
  registerTask(TTxACK);
  registerTask(TRx);
  registerTask(TCmd);
  registerTask(TFailSafe);
  activate();
};

// Toutes les loop() sont des stubs vides pour l'instant — étape suivante.
// FSTask::loop() restera vide plus longtemps que les autres (décision
// explicite, cf. MAVLINKINTERFACE.md §2/§8).

void TTxTask::loop() {}
void TTxAckTask::loop() {}
void TRxTask::loop() {}
void TCmdTask::loop() {}
void FSTask::loop() {}
