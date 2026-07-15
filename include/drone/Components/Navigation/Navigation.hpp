#pragma once
#include "drone/Components/Navigation/SharedNavMem.hpp"
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

class Navigation : public ComponentBase {
public:
  explicit Navigation(ComponenConfig config, SharedSysStateMemHandler &sysState,
                      SharedNavMemHandler &nav);

  SharedNavMemHandler &nav_;

  int a;
  TYPES::TimePoint temps;

private:
  APTask TAP;

  void restore();
  void init();
};
