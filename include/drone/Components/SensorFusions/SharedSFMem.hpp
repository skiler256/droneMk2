#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"
#include <array>
#include <cstdint>

// Échantillon GPS+cap : le cap est celui du magnétomètre pris au moment
// de la réception GPS (pas une valeur indépendante) — permet de
// construire GPS_INPUT côté MavlinkInterface sans recorréler deux
// horodatages différents.
struct GpsMagSample {
  TYPES::TimePoint ts{};
  double latitude{0.0};     // degrés WGS84
  double longitude{0.0};
  float altMsl{0.0f};       // m, MSL
  TYPES::Velocity velNed{}; // vn, ve, vd
  TYPES::Radians heading{0.0f};
  float hdop{0.0f};
  float vdop{0.0f};
  float speedAccuracy{0.0f};
  float horizAccuracy{0.0f};
  float vertAccuracy{0.0f};
  uint8_t fixType{0};
  uint8_t satellitesVisible{0};
};

// Carte LIDAR post-TiltCompensator (déjà corrigée de l'assiette du drone).
// distanceMm[i] == 0 : aucune mesure reçue à cet angle ce tour-ci
// (convention déjà utilisée par LidarData, distance physique nulle
// impossible).
struct LidarMap {
  TYPES::TimePoint ts{};
  std::array<uint16_t, 360> distanceMm{};
  std::array<uint8_t, 360> intensity{};
};

// État dynamique fusionné, repère NED — la donnée que Nav/MissionControl
// consomment comme vérité terrain. attitude est relayée depuis
// SharedFCStatus (pas re-fusionnée ici), posNed/velNed sont calibrés
// (offset HomeCalibrator appliqué).
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

  // Écriture — appelée par les Task de SensorFusion (GPSTask/MagTask,
  // LidarTask, EKFTask — pas encore implémentées).
  void setGpsMag(GpsMagSample s) { setData(id_, timeout_, SF_.data.gpsMag, s); }
  void setLidarMap(LidarMap m) { setData(id_, timeout_, SF_.data.lidar, m); }
  void setDynamicState(DynamicState s) { setData(id_, timeout_, SF_.data.state, s); }

  // Lecture — Navigation (state), MissionControl (lidar pour évitement),
  // MavlinkInterface (gpsMag pour construire GPS_INPUT).
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
