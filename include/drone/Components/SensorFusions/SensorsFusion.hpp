#pragma once
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/SensorFusions/Driver.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/watchdog.hpp"
#include "drone/types.hpp"

class SensorFusions;

// GPS — le magnétomètre est câblé directement sur le FC (§3.5 du .md),
// plus fusionné ici.
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

  // Dernier échantillon exploitable — détecte un driver muet sans jamais
  // d'erreur explicite (gpsHasNoNewData seul ne couvre pas ce cas).
  TYPES::TimePoint lastGoodGpsTs_{};
};

// Relaie attitude + position/vitesse EKF du FC vers SharedSFMem.state (+
// HomeCalibrator) — aucune validation, si l'EKF FC déconne le drone crash.
class EKFTask : public Task {
public:
  explicit EKFTask(SensorFusions &component, TaskConfig config,
                   ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SensorFusions &comp;
  void loop() override;
  bool EKFFaultActive_{false};
};

// Convertit chaque mesure (angle, distance) LIDAR en point NED via
// l'attitude courante de SharedSFMem.state (dépend donc de TEKF).
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
  TYPES::TimePoint lastGoodLidarTs_{};
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

  // Offset NED (HomeCalibrator) appliqué à la position EKF du FC — nul pour l'instant.
  TYPES::Vec3 homeOffsetNed{};

private:
  GPSTask TGPS;
  EKFTask TEKF;
  LidarTask TLidar;
};
