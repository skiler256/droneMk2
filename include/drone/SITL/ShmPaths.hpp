#pragma once
#include <string_view>

// Chemins shm dédiés aux binaires SITL — distincts de ceux de GlobalWatchdog
// (drone_nav/drone_sf/drone_sysstate/drone_fc) pour ne jamais entrer en
// collision si un vrai `drone` tournait par erreur en même temps sur la
// même machine.
namespace SITL {
inline constexpr std::string_view kNavShmPath = "/drone_sitl_nav";
inline constexpr std::string_view kSFShmPath = "/drone_sitl_sf";
inline constexpr std::string_view kSysShmPath = "/drone_sitl_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_sitl_fc";
} // namespace SITL
