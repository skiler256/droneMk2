#pragma once
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/watchdog.hpp"
#include "drone/types.hpp"

class Navigation;

class APTask : public Task {
public:
  explicit APTask(Navigation &component, TaskConfig config,
                  ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  Navigation &comp;
  void loop() override;
};

class Navigation : public ComponentBase<1> {
public:
  explicit Navigation(ComponenConfig config, SharedSysStateMemHandler &sysState,
                      SharedNavMemHandler &nav, SharedSFMemHandler& SF);

  SharedNavMemHandler &nav_;
  SharedSFMemHandler &SF_;

  int a;
  TYPES::TimePoint temps;

private:
  APTask TAP;
};
