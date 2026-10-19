#pragma once
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/SensorFusions/Driver.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/watchdog.hpp"
#include "drone/types.hpp"

class SensorFusions;

// GPS seul : le magnétomètre est câblé sur le FC (§3.5 du .md).
class GPSTask : public Task {
public:
  explicit GPSTask(SensorFusions &component, TaskConfig config,
                  ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SensorFusions &comp;
  void loop() override;

  // Édge-triggered : raiseCode() seulement sur transition, pas à chaque cycle.
  bool gpsFaultActive_{false};
  bool gpsLockActive_{false};

  // Détecte un driver muet qui ne renvoie jamais d'erreur explicite. Initialisé
  // à la construction : partir de l'epoch levait la panne au premier cycle.
  TYPES::TimePoint lastGoodGpsTs_{TYPES::Clock::now()};
};

// Relais pur, aucune validation de plausibilité — cf. SENSORFUSION.md §4.
class EKFTask : public Task {
public:
  explicit EKFTask(SensorFusions &component, TaskConfig config,
                   ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SensorFusions &comp;
  void loop() override;
  bool EKFFaultActive_{false};
  bool wasArmed_{false}; // front d'armement, pour capturer l'origine NED
};

// Projette le LIDAR en NED via l'attitude : dépend donc de TEKF (§5).
class LidarTask : public Task {
public:
  explicit LidarTask(SensorFusions &component, TaskConfig config,
                     ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SensorFusions &comp;
  void loop() override;

  // Édge-triggered, même logique que GPSTask.
  bool lidarFaultActive_{false};
  TYPES::TimePoint lastGoodLidarTs_{TYPES::Clock::now()};
};

class SensorFusions : public ComponentBase<3> {
public:
  explicit SensorFusions(ComponenConfig config, SharedSysStateMemHandler &sysState,
                      SharedSFMemHandler &SF, IDriver<GPSData> &gps,
                      SharedFCStatusHandler &fc, IDriver<LidarData> &lidar);

  SharedSFMemHandler &SF_;
  SharedSysStateMemHandler &sysState_; // pour raiseCode/clearCode depuis les Task
  IDriver<GPSData> &gps_;
  SharedFCStatusHandler &fc_;
  IDriver<LidarData> &lidar_;

  // HomeCalibrator, appliqué à la position seule — nul pour l'instant (§4).
  TYPES::Vec3 homeOffsetNed{};

private:
  GPSTask TGPS;
  EKFTask TEKF;
  LidarTask TLidar;
};
