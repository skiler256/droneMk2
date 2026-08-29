#include "drone/Components/SensorFusions/SensorsFusion.hpp"
#include "drone/Components/SensorFusions/Conversions.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <algorithm>

using namespace TYPES;
using namespace CODES;

namespace {

// Seuils de plausibilité GPS — à ajuster à l'usage.
constexpr float kMaxPlausibleSpeedMs = 50.0f; // ~180 km/h, marge généreuse
constexpr std::uint32_t kMaxHAccMm = 5000;    // 5 m, hAcc NAV-PVT
constexpr std::uint32_t kMaxSAccMmS = 1000;   // 1 m/s, sAcc NAV-PVT

constexpr Ms kAgeThrEKF{1000}; // EKF FC : ATTITUDE/LOCAL_POSITION_NED arrivent à 10-50Hz

// Staleness : capteur considéré en panne au-delà de ce délai, même sans
// erreur driver explicite (ex: câble débranché mais port toujours réactif).
constexpr Ms kMaxGpsAge{1500};   // GPS ~10Hz nominal, marge large (fixes UBX plus lents)
constexpr Ms kMaxLidarAge{500};  // LD06, un tour complet ~10Hz nominal

} // namespace

SensorFusions::SensorFusions(ComponenConfig config,
                       SharedSysStateMemHandler &sysState,
                       SharedSFMemHandler &sf, IDriver<GPSData> &gps,
                       SharedFCStatusHandler &fc, IDriver<LidarData> &lidar)
    : ComponentBase<3>(config, sf, sysState), SF_(sf), sysState_(sysState),
      gps_(gps), fc_(fc), lidar_(lidar),
      TGPS(*this,
          {.id = 0,
           .RTpriority = 70,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{100},
           .timeout = TYPES::Ms{200}},
          localWD),
      TEKF(*this,
          {.id = 1,
           .RTpriority = 70,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{50},
           .timeout = TYPES::Ms{200}},
          localWD),
      TLidar(*this,
          {.id = 2,
           .RTpriority = 60,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{10},
           .timeout = TYPES::Ms{300}},
          localWD)

{
  registerTask(TGPS);
  registerTask(TEKF);
  registerTask(TLidar);
  activate();
};

void GPSTask::loop() {
  auto now = Clock::now();
  auto gpsResult = comp.gps_.update();

  bool gpsHasNoNewData =
      !gpsResult && gpsResult.error() == DriverError::NoNewData;

  // Capteur muet en continu (pas d'EIO du port) -> détecté ici, pas par gpsResult.
  if (UTILITIES::msBetween(lastGoodGpsTs_, now) > kMaxGpsAge && !gpsFaultActive_) {
    comp.sysState_.raiseCode(SFS002);
    gpsFaultActive_ = true;
  }

  if (gpsHasNoNewData)
    return; // rien de neuf ce cycle, inutile de toucher la shm

  // Read-modify-write : préserve les champs non rafraîchis ce cycle.
  auto sample = comp.SF_.getGpsMag().value_or(GpsMagSample{});

  if (gpsResult) {
    const GPSData &d = *gpsResult;

    bool coordPlausible = SF_CONV::gpsCoordPlausible(d.coord.latitude, d.coord.longitude);
    Vec3 velNed = SF_CONV::gpsVelNedMs(d.velNED[0], d.velNED[1], d.velNED[2]);
    bool speedPlausible = velNed.norm() <= kMaxPlausibleSpeedMs;
    bool accuracyOk = SF_CONV::gpsAccuracyOk(d.pAcc, d.sAcc, kMaxHAccMm, kMaxSAccMmS);

    if (d.gpsFixOk && coordPlausible && speedPlausible && accuracyOk) {
      sample.latitude = d.coord.latitude;
      sample.longitude = d.coord.longitude;
      sample.altMsl = static_cast<float>(d.altMslMm) / 1000.0f;
      sample.velNed = Velocity{velNed};
      sample.horizAccuracy = static_cast<float>(d.pAcc) / 1000.0f; // mm -> m
      sample.speedAccuracy = static_cast<float>(d.sAcc) / 1000.0f; // mm/s -> m/s
      sample.vertAccuracy = static_cast<float>(d.vAcc) / 1000.0f;  // mm -> m
      sample.hdop = d.hdop;
      sample.vdop = d.vdop;
      sample.fixType = SF_CONV::mapUbxFixType(d.fixType);
      sample.satellitesVisible = static_cast<std::uint8_t>(d.sats.size());

      sample.satId.fill(0);
      sample.satStrength.fill(0);
      sample.satQuality.fill(0);
      std::size_t n = std::min(d.sats.size(), GpsMagSample::kMaxSatellites);
      for (std::size_t i = 0; i < n; ++i) {
        sample.satId[i] = d.sats[i].id;
        sample.satStrength[i] = d.sats[i].strength;
        sample.satQuality[i] = d.sats[i].quality;
      }

      sample.gpsWeek = SF_CONV::gpsWeekFromUbxDate(d.timeArray);
      sample.gpsTowMs = d.iTowMs;
      sample.ts = now;
      lastGoodGpsTs_ = now;

      if (gpsFaultActive_) {
        comp.sysState_.clearCode(SFS002);
        gpsFaultActive_ = false;
      }
      comp.sysState_.raiseCode(SFS001); // Info, mineur : pas de spam à craindre
    } else if (!gpsFaultActive_) {
      comp.sysState_.raiseCode(SFS002);
      gpsFaultActive_ = true;
    }
  } else if (!gpsHasNoNewData && !gpsFaultActive_) {
    // Erreur driver réelle (pas juste NoNewData) -> traitée comme perte GPS.
    comp.sysState_.raiseCode(SFS002);
    gpsFaultActive_ = true;
  }

  comp.SF_.setGpsMag(sample);
};

void EKFTask::loop() {
  auto posVel = comp.fc_.getLocalPosVel();
  auto attitude = comp.fc_.getAttitude();
  auto posVelTs = comp.fc_.getLocalPosVelTs();
  auto attitudeTs = comp.fc_.getAttitudeTs();
  if (!posVel || !attitude || !posVelTs || !attitudeTs)
    return; // FC pas encore prêt / shm indisponible ce cycle

  // Staleness sur ATTITUDE/LOCAL_POSITION_NED (10-50Hz), pas HEARTBEAT (1Hz)
  // — comparer à HEARTBEAT déclenchait SFS005 en continu en nominal.
  auto now = Clock::now();
  bool stale = UTILITIES::msBetween(*posVelTs, now) > kAgeThrEKF ||
              UTILITIES::msBetween(*attitudeTs, now) > kAgeThrEKF;
  if (stale) {
    if (!EKFFaultActive_) {
      comp.sysState_.raiseCode(SFS005);
      EKFFaultActive_ = true;
    }
    return;
  }

  DynamicState state{};
  state.ts = now;
  state.posNed = Position{Vec3{posVel->pos.ned.x - comp.homeOffsetNed.x,
                               posVel->pos.ned.y - comp.homeOffsetNed.y,
                               posVel->pos.ned.z - comp.homeOffsetNed.z}};
  state.velNed = posVel->vel; // pas d'offset sur une vitesse
  state.attitude = *attitude;

  if (EKFFaultActive_) {
    comp.sysState_.clearCode(SFS005);
    EKFFaultActive_ = false;
  }

  comp.SF_.setDynamicState(state);
};

void LidarTask::loop() {
  auto now = Clock::now();
  auto scan = comp.lidar_.update();
  if (!scan) {
    // Erreur driver ou tour complet trop vieux (capteur muet) -> même code.
    bool driverError = scan.error() != DriverError::NoNewData;
    bool stale = UTILITIES::msBetween(lastGoodLidarTs_, now) > kMaxLidarAge;
    if ((driverError || stale) && !lidarFaultActive_) {
      comp.sysState_.raiseCode(SFS004);
      lidarFaultActive_ = true;
    }
    return; // pas de tour complet ce cycle (NoNewData) ou erreur driver
  }
  lastGoodLidarTs_ = now;
  if (lidarFaultActive_) {
    comp.sysState_.clearCode(SFS004);
    lidarFaultActive_ = false;
  }

  auto state = comp.SF_.getDynamicState();
  if (!state || UTILITIES::msBetween(state->ts, now) > kAgeThrEKF)
    return; // attitude absente ou périmée, impossible de compenser fiablement

  Eigen::Quaternionf q = state->attitude.q.asEigen();

  LidarMap map{};
  map.ts = now;
  for (std::size_t i = 0; i < map.points.size(); ++i) {
    std::uint16_t distMm = scan->distance_mm[i];
    if (distMm == 0)
      continue; // sentinelle : aucune mesure à cet angle ce tour-ci

    float distM = static_cast<float>(distMm) / 1000.0f;
    map.points[i] = SF_CONV::lidarPointNed(distM, static_cast<float>(i), q);
    map.intensity[i] = scan->intensity[i];
    map.rangeMm[i] = distMm;
  }

  comp.SF_.setLidarMap(map);
};
