#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"
#include <array>
#include <cstdint>

// Échantillon GPS. Nommé GpsMagSample par héritage (le cap magnétique y
// était bundlé, envoyé dans GPS_INPUT.yaw) — le magnétomètre est câblé
// directement sur le FC depuis la bascule d'archi (cf. SENSORFUSION.md),
// donc plus de champ heading ici : GPS_INPUT.yaw part à 0 ("inconnu" selon
// la spec MAVLink), l'EKF3 du FC utilise son propre compas.
struct GpsMagSample {
  TYPES::TimePoint ts{};
  double latitude{0.0};     // degrés WGS84
  double longitude{0.0};
  float altMsl{0.0f};       // m, MSL
  TYPES::Velocity velNed{}; // vn, ve, vd
  float hdop{0.0f};
  float vdop{0.0f};
  float speedAccuracy{0.0f};
  float horizAccuracy{0.0f};
  float vertAccuracy{0.0f};
  uint8_t fixType{0};
  uint8_t satellitesVisible{0};

  // Détail par satellite (id/force du signal/qualité), cf. GPSData::sats —
  // capé à 16 (largement suffisant, une constellation visible dépasse
  // rarement ça) pour rester une struct POD taille fixe en shm. Seuls les
  // `satellitesVisible` premiers slots (borné à 16) sont significatifs.
  static constexpr std::size_t kMaxSatellites = 16;
  std::array<uint8_t, kMaxSatellites> satId{};
  std::array<uint8_t, kMaxSatellites> satStrength{};
  std::array<uint8_t, kMaxSatellites> satQuality{};

  // Temps GPS (semaine + ms dans la semaine), calculé par le récepteur —
  // PAS `ts` (horloge interne, steady_clock, sans rapport avec le temps
  // GPS) : c'est ce que MavlinkInterface envoie tel quel dans
  // GPS_INPUT.time_week/time_week_ms pour l'EKF3 du FC.
  uint16_t gpsWeek{0};
  uint32_t gpsTowMs{0};
};

// Nuage de points LIDAR en NED, relatif au drone, projeté via l'attitude
// courante. points[i].isZero() : aucune mesure à cet angle ce tour-ci.
// rangeMm/intensity sont la mesure brute repère-corps (rangeMm[i] = portée
// à l'angle i en degrés, AVANT projection NED) — conservée à côté de
// `points` pour la télémétrie (LIDAR_SCAN) : reprojeter `points` en
// repère-corps demanderait de désappliquer l'attitude, inutilement fragile.
struct LidarMap {
  TYPES::TimePoint ts{};
  std::array<TYPES::Vec3, 360> points{};
  std::array<uint8_t, 360> intensity{};
  std::array<uint16_t, 360> rangeMm{};
};

// État dynamique fusionné, repère NED — vérité terrain pour Nav/MC.
// posNed a l'offset HomeCalibrator appliqué, attitude relayée du FC.
struct DynamicState {
  TYPES::TimePoint ts{};
  TYPES::Position posNed{};
  TYPES::Velocity velNed{};
  TYPES::Attitude attitude{};
};

struct SharedSFMem {
  SharedCompMem compMem;

  struct payload {
    GpsMagSample gpsMag{};
    LidarMap lidar{};
    DynamicState state{};
  };
  payload data;
};

class SharedSFMemHandler : public SharedCompMemHandler {
public:
  explicit SharedSFMemHandler(TYPES::ComponentID id, TYPES::Us timeout,
                              SharedSFMem &SF)
      : SharedCompMemHandler(SF.compMem, id, timeout), SF_(SF) {};

  // Écriture — appelée par les Task de SensorFusion (GPSTask, LidarTask, EKFTask).
  void setGpsMag(GpsMagSample s) { setData(id_, timeout_, SF_.data.gpsMag, s); }
  void setLidarMap(LidarMap m) { setData(id_, timeout_, SF_.data.lidar, m); }
  void setDynamicState(DynamicState s) { setData(id_, timeout_, SF_.data.state, s); }

  // Lecture — Nav (state), MissionControl (lidar), MavlinkInterface (gpsMag).
  [[nodiscard]] std::optional<GpsMagSample> getGpsMag() {
    return getData(id_, timeout_, SF_.data.gpsMag);
  }
  [[nodiscard]] std::optional<LidarMap> getLidarMap() {
    return getData(id_, timeout_, SF_.data.lidar);
  }
  [[nodiscard]] std::optional<DynamicState> getDynamicState() {
    return getData(id_, timeout_, SF_.data.state);
  }

private:
  SharedSFMem &SF_;

  uint32_t computeChecksum(TYPES::ComponentID) {
    return UTILITIES::crc32(SF_.data) ^ historyChecksum();
  };

  // Seul le propriétaire peut reconstruire sa propre shm.
  void reset(TYPES::ComponentID id) {
    if (id == TYPES::ComponentID::SensorFusion) {
      SF_.data = SharedSFMem::payload{};
      sanitizeHistory(comp_.HotStartHistory);
      sanitizeHistory(comp_.ColdStartHistory);
      updateChecksum(id);
    }
  };
};
