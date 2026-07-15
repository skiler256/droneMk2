#pragma once
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/watchdog.hpp"
#include "drone/types.hpp"

class SensorFusions;

class GPSTask : public Task {
public:
  explicit GPSTask(SensorFusions &component, TaskConfig config,
                  ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SensorFusions &comp;
  void loop() override;
};

class SensorFusions : public ComponentBase {
public:
  explicit SensorFusions(ComponenConfig config, SharedSysStateMemHandler &sysState,
                      SharedSFMemHandler &SF);

  SharedSFMemHandler &SF_;

  int a;
  TYPES::TimePoint temps;

private:
  GPSTask TGPS;

  void restore();
  void init();
};
