#pragma once
#include "drone/types.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>

// Conversions pures extraites de GPSTask/LidarTask::loop() pour être
// testables sans passer par les Tasks (loop() est privée, appelée
// uniquement par le thread RT de la Task) — même principe que
// MavlinkInterface/Conversions.hpp.
namespace SF_CONV {

// UBX NAV-STATUS.gpsFix -> code approximatif MAVLink GPS_FIX_TYPE (seuls
// 0/2/3 se recoupent, le reste dégradé en NO_FIX par prudence).
[[nodiscard]] constexpr std::uint8_t mapUbxFixType(std::uint8_t ubx) noexcept {
  switch (ubx) {
  case 0:
    return 0; // NO_GPS
  case 2:
    return 2; // 2D_FIX
  case 3:
    return 3; // 3D_FIX
  default:
    return 1; // NO_FIX
  }
}

// Numéro de semaine GPS (époque 1980-01-06) depuis une date calendaire UBX
// (year/month/day). Le nombre de ms DANS la semaine vient séparément de
// GPSData::iTowMs (NAV-PVT) — pas besoin de l'heure ici, iTOW l'encode déjà.
[[nodiscard]] inline std::uint16_t gpsWeekFromUbxDate(const int (&timeArray)[6]) noexcept {
  using namespace std::chrono;
  year_month_day ymd{year{timeArray[0]}, month{static_cast<unsigned>(timeArray[1])},
                     day{static_cast<unsigned>(timeArray[2])}};
  sys_days date{ymd};
  sys_days gpsEpoch{year{1980} / January / 6};
  auto daysSinceEpoch = (date - gpsEpoch).count();
  return static_cast<std::uint16_t>(daysSinceEpoch / 7);
}

[[nodiscard]] constexpr bool gpsCoordPlausible(double latitude, double longitude) noexcept {
  return std::abs(latitude) <= 90.0 && std::abs(longitude) <= 180.0;
}

// GPSData::velNED (cm/s, NED) -> m/s.
[[nodiscard]] inline TYPES::Vec3 gpsVelNedMs(int vnCms, int veCms, int vdCms) noexcept {
  return TYPES::Vec3{static_cast<float>(vnCms) / 100.0f, static_cast<float>(veCms) / 100.0f,
                     static_cast<float>(vdCms) / 100.0f};
}

[[nodiscard]] constexpr bool gpsAccuracyOk(std::uint32_t pAccMm, std::uint32_t sAccMmS,
                                          std::uint32_t maxHAccMm,
                                          std::uint32_t maxSAccMmS) noexcept {
  return pAccMm <= maxHAccMm && sAccMmS <= maxSAccMmS;
}

// Projette une mesure LIDAR (distance, angle en degrés) en point NED
// relatif au drone via l'attitude courante — repère capteur/corps : x=avant,
// y=droite, z=0 (scan 2D horizontal).
[[nodiscard]] inline TYPES::Vec3 lidarPointNed(float distanceM, float angleDeg,
                                               const Eigen::Quaternionf &attitudeQ) noexcept {
  float theta = angleDeg * (std::numbers::pi_v<float> / 180.0f);
  Eigen::Vector3f body{distanceM * std::cos(theta), distanceM * std::sin(theta), 0.0f};
  Eigen::Vector3f ned = attitudeQ * body;
  return TYPES::Vec3{ned.x(), ned.y(), ned.z()};
}

} // namespace SF_CONV
