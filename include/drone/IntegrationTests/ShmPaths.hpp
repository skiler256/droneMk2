#pragma once
#include <string_view>

// Chemins shm dédiés aux binaires de test d'intégration (SITL/HIL) —
// distincts de ceux de GlobalWatchdog (drone_nav/drone_sf/...) pour ne
// jamais entrer en collision si un vrai `drone` tournait en même temps.
namespace SITL {
inline constexpr std::string_view kNavShmPath = "/drone_sitl_nav";
inline constexpr std::string_view kSFShmPath = "/drone_sitl_sf";
inline constexpr std::string_view kSysShmPath = "/drone_sitl_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_sitl_fc";
inline constexpr std::string_view kCmdShmPath = "/drone_sitl_cmd";
} // namespace SITL

// HIL (hil_sf_mv) : SensorFusion + MavlinkInterface face au vrai matériel.
namespace HIL {
inline constexpr std::string_view kNavShmPath = "/drone_hil_nav";
inline constexpr std::string_view kSFShmPath = "/drone_hil_sf";
inline constexpr std::string_view kSysShmPath = "/drone_hil_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_hil_fc";
inline constexpr std::string_view kCmdShmPath = "/drone_hil_cmd";
} // namespace HIL

// NAV_SITL (nav_sitl) : Navigation + MavlinkInterface face à ArduPilot
// SITL, sans SensorFusion — le segment SF existe quand même (Navigation
// en dépend) mais est rempli par un relais minimal dans nav_sitl.cpp, pas
// par un vrai SensorFusions (GPS simulé laissé côté SITL, cf. NAVIGATION.md).
namespace NAV_SITL {
inline constexpr std::string_view kNavShmPath = "/drone_navsitl_nav";
inline constexpr std::string_view kSFShmPath = "/drone_navsitl_sf";
inline constexpr std::string_view kSysShmPath = "/drone_navsitl_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_navsitl_fc";
inline constexpr std::string_view kCmdShmPath = "/drone_navsitl_cmd";
} // namespace NAV_SITL
