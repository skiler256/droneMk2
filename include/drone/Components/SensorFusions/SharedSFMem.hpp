#pragma once

#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"
#include <array>
#include <cstdint>

// Nom hérité de l'époque où le cap magnétique y était bundlé ; plus de
// champ heading depuis la bascule du magnéto vers le FC (§3.5 du .md).
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

  // Capé à 16 pour rester POD taille fixe ; seuls les satellitesVisible
  // premiers slots sont significatifs — cf. SENSORFUSION.md §3.
  static constexpr std::size_t kMaxSatellites = 16;
  std::array<uint8_t, kMaxSatellites> satId{};
  std::array<uint8_t, kMaxSatellites> satStrength{};
  std::array<uint8_t, kMaxSatellites> satQuality{};

  // Temps GPS du récepteur, PAS `ts` (steady_clock interne) — envoyé tel
  // quel dans GPS_INPUT.time_week/time_week_ms.
  uint16_t gpsWeek{0};
  uint32_t gpsTowMs{0};
};

// Points en NED relatif au drone ; isZero() = aucune mesure à cet angle.
// rangeMm/intensity gardent la mesure brute repère-corps pour LIDAR_SCAN
// — cf. SENSORFUSION.md §5.
struct LidarMap {
  TYPES::TimePoint ts{};
  std::array<TYPES::Vec3, 360> points{};
  std::array<uint8_t, 360> intensity{};
  std::array<uint16_t, 360> rangeMm{};
};

// État fusionné en NED, vérité terrain pour Nav/MC. posNed porte l'offset
// HomeCalibrator, l'attitude est relayée du FC.
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

  // Écriture : les trois Task de SensorFusion.
  void setGpsMag(GpsMagSample s) { setData(id_, timeout_, SF_.data.gpsMag, s); }
  void setLidarMap(LidarMap m) { setData(id_, timeout_, SF_.data.lidar, m); }
  void setDynamicState(DynamicState s) { setData(id_, timeout_, SF_.data.state, s); }

  // Lecture : Nav (state), MC (lidar), MavlinkInterface (gpsMag).
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
