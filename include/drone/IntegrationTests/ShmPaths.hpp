#pragma once
#include <string_view>

// Chemins shm des binaires SITL/HIL, distincts de ceux de GlobalWatchdog
// pour ne jamais collisionner avec un vrai `drone` qui tournerait.
namespace SITL {
inline constexpr std::string_view kNavShmPath = "/drone_sitl_nav";
inline constexpr std::string_view kSFShmPath = "/drone_sitl_sf";
inline constexpr std::string_view kSysShmPath = "/drone_sitl_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_sitl_fc";
inline constexpr std::string_view kCmdShmPath = "/drone_sitl_cmd";
inline constexpr std::string_view kComShmPath = "/drone_sitl_com";
} // namespace SITL

// HIL (hil_sf_mv) : SensorFusion + MavlinkInterface face au vrai matériel.
namespace HIL {
inline constexpr std::string_view kNavShmPath = "/drone_hil_nav";
inline constexpr std::string_view kSFShmPath = "/drone_hil_sf";
inline constexpr std::string_view kSysShmPath = "/drone_hil_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_hil_fc";
inline constexpr std::string_view kCmdShmPath = "/drone_hil_cmd";
inline constexpr std::string_view kComShmPath = "/drone_hil_com";
} // namespace HIL

// NAV_SITL : Navigation + MavlinkInterface sans SensorFusion — le segment
// SF existe quand même, rempli par un relais minimal dans nav_sitl.cpp.
namespace NAV_SITL {
inline constexpr std::string_view kNavShmPath = "/drone_navsitl_nav";
inline constexpr std::string_view kSFShmPath = "/drone_navsitl_sf";
inline constexpr std::string_view kSysShmPath = "/drone_navsitl_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_navsitl_fc";
inline constexpr std::string_view kCmdShmPath = "/drone_navsitl_cmd";
inline constexpr std::string_view kComShmPath = "/drone_navsitl_com";
} // namespace NAV_SITL

// MC_SITL : MissionControl seul, plan prédéfini et faux Nav minimal.
namespace MC_SITL {
inline constexpr std::string_view kNavShmPath = "/drone_mcsitl_nav";
inline constexpr std::string_view kSFShmPath = "/drone_mcsitl_sf";
inline constexpr std::string_view kSysShmPath = "/drone_mcsitl_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_mcsitl_fc";
inline constexpr std::string_view kCmdShmPath = "/drone_mcsitl_cmd";
inline constexpr std::string_view kComShmPath = "/drone_mcsitl_com";
} // namespace MC_SITL

// SYSMON_SITL : SysMonitoring seul, données shm injectées à la main. Sur
// les MÊMES ports UDP que main.cpp, exprès : le dashboard GCS l'observe
// sans configuration différente.
namespace SYSMON_SITL {
inline constexpr std::string_view kNavShmPath = "/drone_smsitl_nav";
inline constexpr std::string_view kSFShmPath = "/drone_smsitl_sf";
inline constexpr std::string_view kSysShmPath = "/drone_smsitl_sysstate";
inline constexpr std::string_view kFCShmPath = "/drone_smsitl_fc";
inline constexpr std::string_view kCmdShmPath = "/drone_smsitl_cmd";
inline constexpr std::string_view kComShmPath = "/drone_smsitl_com";
} // namespace SYSMON_SITL
