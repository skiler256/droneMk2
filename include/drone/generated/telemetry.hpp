// include/drone/generated/telemetry.hpp
// ────────────────────────────────────────────────────────
// GÉNÉRÉ depuis TELEMETRY.csv par tools/gen_telemetry.py — NE PAS ÉDITER À LA MAIN.
// Régénérer : python3 tools/gen_telemetry.py (ou : cmake --build .)
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

namespace TELEM {

enum class Direction : uint8_t { Down = 0, Up = 1 };
enum class Priority : uint8_t { Routine = 0, Critical = 1 };

// Bitmask : quel(s) lien(s) portent ce paquet. TelSec (bas débit, secours)
// ne porte qu'un sous-ensemble léger — TelMain porte toujours tout.
enum class Link : uint8_t { Main = 0b01, Sec = 0b10 };

// Identifiants stables (ordre du CSV) — NE PAS réordonner, ajouter en fin.
enum class PacketID : uint8_t {
  POS = 0,
  VELNED = 1,
  ATTITUDE = 2,
  HEALTH = 3,
  GPSFIX = 4,
  BATTERY = 5,
  CODE = 6,
  ARM_STATE = 7,
  MISSION_STATE = 8,
  AUTOPILOT_MODE = 9,
  CMD_ARM = 10,
  CMD_DISARM = 11,
  CMD_MODE = 12,
  CMD_WAYPOINT = 13,
  CMD_RTL = 14,
  CMD_LAND = 15,
  HEARTBEAT_GCS = 16,
  CMD_EMERGENCY = 17,
  CMD_TAKEOFF = 18,
  CMD_HOLD = 19,
  CMD_RESUME = 20,
  CMD_ANSWER = 21,
  MC_QUERY = 22,
  GPSPOS = 23,
  SAT_INFO = 24,
  LIDAR_SCAN = 25,
  CPU_TEMP = 26,
};

struct PosPkt {
  static constexpr PacketID kId = PacketID::POS;
  static constexpr size_t kSize = 12;
  float north{};
  float east{};
  float down{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &north, sizeof(north));
    std::memcpy(buf.data() + 4, &east, sizeof(east));
    std::memcpy(buf.data() + 8, &down, sizeof(down));
    return buf;
  }

  [[nodiscard]] static PosPkt unpack(std::span<const std::byte> bytes) noexcept {
    PosPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.north, bytes.data() + 0, sizeof(out.north));
    std::memcpy(&out.east, bytes.data() + 4, sizeof(out.east));
    std::memcpy(&out.down, bytes.data() + 8, sizeof(out.down));
    return out;
  }
};

struct VelnedPkt {
  static constexpr PacketID kId = PacketID::VELNED;
  static constexpr size_t kSize = 12;
  float vn{};
  float ve{};
  float vd{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &vn, sizeof(vn));
    std::memcpy(buf.data() + 4, &ve, sizeof(ve));
    std::memcpy(buf.data() + 8, &vd, sizeof(vd));
    return buf;
  }

  [[nodiscard]] static VelnedPkt unpack(std::span<const std::byte> bytes) noexcept {
    VelnedPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.vn, bytes.data() + 0, sizeof(out.vn));
    std::memcpy(&out.ve, bytes.data() + 4, sizeof(out.ve));
    std::memcpy(&out.vd, bytes.data() + 8, sizeof(out.vd));
    return out;
  }
};

struct AttitudePkt {
  static constexpr PacketID kId = PacketID::ATTITUDE;
  static constexpr size_t kSize = 16;
  float qw{};
  float qx{};
  float qy{};
  float qz{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &qw, sizeof(qw));
    std::memcpy(buf.data() + 4, &qx, sizeof(qx));
    std::memcpy(buf.data() + 8, &qy, sizeof(qy));
    std::memcpy(buf.data() + 12, &qz, sizeof(qz));
    return buf;
  }

  [[nodiscard]] static AttitudePkt unpack(std::span<const std::byte> bytes) noexcept {
    AttitudePkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.qw, bytes.data() + 0, sizeof(out.qw));
    std::memcpy(&out.qx, bytes.data() + 4, sizeof(out.qx));
    std::memcpy(&out.qy, bytes.data() + 8, sizeof(out.qy));
    std::memcpy(&out.qz, bytes.data() + 12, sizeof(out.qz));
    return out;
  }
};

struct HealthPkt {
  static constexpr PacketID kId = PacketID::HEALTH;
  static constexpr size_t kSize = 2;
  uint8_t component{};
  uint8_t health{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &component, sizeof(component));
    std::memcpy(buf.data() + 1, &health, sizeof(health));
    return buf;
  }

  [[nodiscard]] static HealthPkt unpack(std::span<const std::byte> bytes) noexcept {
    HealthPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.component, bytes.data() + 0, sizeof(out.component));
    std::memcpy(&out.health, bytes.data() + 1, sizeof(out.health));
    return out;
  }
};

struct GpsfixPkt {
  static constexpr PacketID kId = PacketID::GPSFIX;
  static constexpr size_t kSize = 6;
  uint8_t fix_type{};
  uint8_t num_sats{};
  float hdop{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &fix_type, sizeof(fix_type));
    std::memcpy(buf.data() + 1, &num_sats, sizeof(num_sats));
    std::memcpy(buf.data() + 2, &hdop, sizeof(hdop));
    return buf;
  }

  [[nodiscard]] static GpsfixPkt unpack(std::span<const std::byte> bytes) noexcept {
    GpsfixPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.fix_type, bytes.data() + 0, sizeof(out.fix_type));
    std::memcpy(&out.num_sats, bytes.data() + 1, sizeof(out.num_sats));
    std::memcpy(&out.hdop, bytes.data() + 2, sizeof(out.hdop));
    return out;
  }
};

struct BatteryPkt {
  static constexpr PacketID kId = PacketID::BATTERY;
  static constexpr size_t kSize = 9;
  float voltage{};
  float current{};
  uint8_t remaining_pct{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &voltage, sizeof(voltage));
    std::memcpy(buf.data() + 4, &current, sizeof(current));
    std::memcpy(buf.data() + 8, &remaining_pct, sizeof(remaining_pct));
    return buf;
  }

  [[nodiscard]] static BatteryPkt unpack(std::span<const std::byte> bytes) noexcept {
    BatteryPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.voltage, bytes.data() + 0, sizeof(out.voltage));
    std::memcpy(&out.current, bytes.data() + 4, sizeof(out.current));
    std::memcpy(&out.remaining_pct, bytes.data() + 8, sizeof(out.remaining_pct));
    return out;
  }
};

struct CodePkt {
  static constexpr PacketID kId = PacketID::CODE;
  static constexpr size_t kSize = 9;
  uint32_t code{};
  uint8_t active{};
  uint32_t ts_ms{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &code, sizeof(code));
    std::memcpy(buf.data() + 4, &active, sizeof(active));
    std::memcpy(buf.data() + 5, &ts_ms, sizeof(ts_ms));
    return buf;
  }

  [[nodiscard]] static CodePkt unpack(std::span<const std::byte> bytes) noexcept {
    CodePkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.code, bytes.data() + 0, sizeof(out.code));
    std::memcpy(&out.active, bytes.data() + 4, sizeof(out.active));
    std::memcpy(&out.ts_ms, bytes.data() + 5, sizeof(out.ts_ms));
    return out;
  }
};

struct ArmStatePkt {
  static constexpr PacketID kId = PacketID::ARM_STATE;
  static constexpr size_t kSize = 2;
  uint8_t armed{};
  uint8_t reason{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &armed, sizeof(armed));
    std::memcpy(buf.data() + 1, &reason, sizeof(reason));
    return buf;
  }

  [[nodiscard]] static ArmStatePkt unpack(std::span<const std::byte> bytes) noexcept {
    ArmStatePkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.armed, bytes.data() + 0, sizeof(out.armed));
    std::memcpy(&out.reason, bytes.data() + 1, sizeof(out.reason));
    return out;
  }
};

struct MissionStatePkt {
  static constexpr PacketID kId = PacketID::MISSION_STATE;
  static constexpr size_t kSize = 1;
  uint8_t state{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &state, sizeof(state));
    return buf;
  }

  [[nodiscard]] static MissionStatePkt unpack(std::span<const std::byte> bytes) noexcept {
    MissionStatePkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.state, bytes.data() + 0, sizeof(out.state));
    return out;
  }
};

struct AutopilotModePkt {
  static constexpr PacketID kId = PacketID::AUTOPILOT_MODE;
  static constexpr size_t kSize = 24;
  uint32_t lateral_engaged{};
  uint32_t lateral_armed{};
  uint32_t vertical_engaged{};
  uint32_t vertical_armed{};
  float target_altitude{};
  float target_speed{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &lateral_engaged, sizeof(lateral_engaged));
    std::memcpy(buf.data() + 4, &lateral_armed, sizeof(lateral_armed));
    std::memcpy(buf.data() + 8, &vertical_engaged, sizeof(vertical_engaged));
    std::memcpy(buf.data() + 12, &vertical_armed, sizeof(vertical_armed));
    std::memcpy(buf.data() + 16, &target_altitude, sizeof(target_altitude));
    std::memcpy(buf.data() + 20, &target_speed, sizeof(target_speed));
    return buf;
  }

  [[nodiscard]] static AutopilotModePkt unpack(std::span<const std::byte> bytes) noexcept {
    AutopilotModePkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.lateral_engaged, bytes.data() + 0, sizeof(out.lateral_engaged));
    std::memcpy(&out.lateral_armed, bytes.data() + 4, sizeof(out.lateral_armed));
    std::memcpy(&out.vertical_engaged, bytes.data() + 8, sizeof(out.vertical_engaged));
    std::memcpy(&out.vertical_armed, bytes.data() + 12, sizeof(out.vertical_armed));
    std::memcpy(&out.target_altitude, bytes.data() + 16, sizeof(out.target_altitude));
    std::memcpy(&out.target_speed, bytes.data() + 20, sizeof(out.target_speed));
    return out;
  }
};

struct CmdArmPkt {
  static constexpr PacketID kId = PacketID::CMD_ARM;
  static constexpr size_t kSize = 0;

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    return buf;
  }

  [[nodiscard]] static CmdArmPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdArmPkt out{};
    if (bytes.size() < kSize) return out;
    return out;
  }
};

struct CmdDisarmPkt {
  static constexpr PacketID kId = PacketID::CMD_DISARM;
  static constexpr size_t kSize = 0;

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    return buf;
  }

  [[nodiscard]] static CmdDisarmPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdDisarmPkt out{};
    if (bytes.size() < kSize) return out;
    return out;
  }
};

struct CmdModePkt {
  static constexpr PacketID kId = PacketID::CMD_MODE;
  static constexpr size_t kSize = 1;
  uint8_t mode{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &mode, sizeof(mode));
    return buf;
  }

  [[nodiscard]] static CmdModePkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdModePkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.mode, bytes.data() + 0, sizeof(out.mode));
    return out;
  }
};

struct CmdWaypointPkt {
  static constexpr PacketID kId = PacketID::CMD_WAYPOINT;
  static constexpr size_t kSize = 20;
  float north{};
  float east{};
  float down{};
  float approach_speed{};
  float acceptance_radius{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &north, sizeof(north));
    std::memcpy(buf.data() + 4, &east, sizeof(east));
    std::memcpy(buf.data() + 8, &down, sizeof(down));
    std::memcpy(buf.data() + 12, &approach_speed, sizeof(approach_speed));
    std::memcpy(buf.data() + 16, &acceptance_radius, sizeof(acceptance_radius));
    return buf;
  }

  [[nodiscard]] static CmdWaypointPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdWaypointPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.north, bytes.data() + 0, sizeof(out.north));
    std::memcpy(&out.east, bytes.data() + 4, sizeof(out.east));
    std::memcpy(&out.down, bytes.data() + 8, sizeof(out.down));
    std::memcpy(&out.approach_speed, bytes.data() + 12, sizeof(out.approach_speed));
    std::memcpy(&out.acceptance_radius, bytes.data() + 16, sizeof(out.acceptance_radius));
    return out;
  }
};

struct CmdRtlPkt {
  static constexpr PacketID kId = PacketID::CMD_RTL;
  static constexpr size_t kSize = 0;

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    return buf;
  }

  [[nodiscard]] static CmdRtlPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdRtlPkt out{};
    if (bytes.size() < kSize) return out;
    return out;
  }
};

struct CmdLandPkt {
  static constexpr PacketID kId = PacketID::CMD_LAND;
  static constexpr size_t kSize = 0;

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    return buf;
  }

  [[nodiscard]] static CmdLandPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdLandPkt out{};
    if (bytes.size() < kSize) return out;
    return out;
  }
};

struct HeartbeatGcsPkt {
  static constexpr PacketID kId = PacketID::HEARTBEAT_GCS;
  static constexpr size_t kSize = 0;

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    return buf;
  }

  [[nodiscard]] static HeartbeatGcsPkt unpack(std::span<const std::byte> bytes) noexcept {
    HeartbeatGcsPkt out{};
    if (bytes.size() < kSize) return out;
    return out;
  }
};

struct CmdEmergencyPkt {
  static constexpr PacketID kId = PacketID::CMD_EMERGENCY;
  static constexpr size_t kSize = 0;

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    return buf;
  }

  [[nodiscard]] static CmdEmergencyPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdEmergencyPkt out{};
    if (bytes.size() < kSize) return out;
    return out;
  }
};

struct CmdTakeoffPkt {
  static constexpr PacketID kId = PacketID::CMD_TAKEOFF;
  static constexpr size_t kSize = 4;
  float altitude{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &altitude, sizeof(altitude));
    return buf;
  }

  [[nodiscard]] static CmdTakeoffPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdTakeoffPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.altitude, bytes.data() + 0, sizeof(out.altitude));
    return out;
  }
};

struct CmdHoldPkt {
  static constexpr PacketID kId = PacketID::CMD_HOLD;
  static constexpr size_t kSize = 0;

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    return buf;
  }

  [[nodiscard]] static CmdHoldPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdHoldPkt out{};
    if (bytes.size() < kSize) return out;
    return out;
  }
};

struct CmdResumePkt {
  static constexpr PacketID kId = PacketID::CMD_RESUME;
  static constexpr size_t kSize = 0;

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    return buf;
  }

  [[nodiscard]] static CmdResumePkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdResumePkt out{};
    if (bytes.size() < kSize) return out;
    return out;
  }
};

struct CmdAnswerPkt {
  static constexpr PacketID kId = PacketID::CMD_ANSWER;
  static constexpr size_t kSize = 5;
  uint32_t seq{};
  uint8_t choice{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &seq, sizeof(seq));
    std::memcpy(buf.data() + 4, &choice, sizeof(choice));
    return buf;
  }

  [[nodiscard]] static CmdAnswerPkt unpack(std::span<const std::byte> bytes) noexcept {
    CmdAnswerPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.seq, bytes.data() + 0, sizeof(out.seq));
    std::memcpy(&out.choice, bytes.data() + 4, sizeof(out.choice));
    return out;
  }
};

struct McQueryPkt {
  static constexpr PacketID kId = PacketID::MC_QUERY;
  static constexpr size_t kSize = 5;
  uint32_t seq{};
  uint8_t query_type{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &seq, sizeof(seq));
    std::memcpy(buf.data() + 4, &query_type, sizeof(query_type));
    return buf;
  }

  [[nodiscard]] static McQueryPkt unpack(std::span<const std::byte> bytes) noexcept {
    McQueryPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.seq, bytes.data() + 0, sizeof(out.seq));
    std::memcpy(&out.query_type, bytes.data() + 4, sizeof(out.query_type));
    return out;
  }
};

struct GpsposPkt {
  static constexpr PacketID kId = PacketID::GPSPOS;
  static constexpr size_t kSize = 16;
  float lat{};
  float lon{};
  float alt_msl{};
  float vdop{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &lat, sizeof(lat));
    std::memcpy(buf.data() + 4, &lon, sizeof(lon));
    std::memcpy(buf.data() + 8, &alt_msl, sizeof(alt_msl));
    std::memcpy(buf.data() + 12, &vdop, sizeof(vdop));
    return buf;
  }

  [[nodiscard]] static GpsposPkt unpack(std::span<const std::byte> bytes) noexcept {
    GpsposPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.lat, bytes.data() + 0, sizeof(out.lat));
    std::memcpy(&out.lon, bytes.data() + 4, sizeof(out.lon));
    std::memcpy(&out.alt_msl, bytes.data() + 8, sizeof(out.alt_msl));
    std::memcpy(&out.vdop, bytes.data() + 12, sizeof(out.vdop));
    return out;
  }
};

struct SatInfoPkt {
  static constexpr PacketID kId = PacketID::SAT_INFO;
  static constexpr size_t kSize = 49;
  uint8_t count{};
  std::array<uint8_t, 16> id{};
  std::array<uint8_t, 16> strength{};
  std::array<uint8_t, 16> quality{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &count, sizeof(count));
    std::memcpy(buf.data() + 1, &id, sizeof(id));
    std::memcpy(buf.data() + 17, &strength, sizeof(strength));
    std::memcpy(buf.data() + 33, &quality, sizeof(quality));
    return buf;
  }

  [[nodiscard]] static SatInfoPkt unpack(std::span<const std::byte> bytes) noexcept {
    SatInfoPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.count, bytes.data() + 0, sizeof(out.count));
    std::memcpy(&out.id, bytes.data() + 1, sizeof(out.id));
    std::memcpy(&out.strength, bytes.data() + 17, sizeof(out.strength));
    std::memcpy(&out.quality, bytes.data() + 33, sizeof(out.quality));
    return out;
  }
};

struct LidarScanPkt {
  static constexpr PacketID kId = PacketID::LIDAR_SCAN;
  static constexpr size_t kSize = 1084;
  uint16_t min_dist_mm{};
  int16_t min_bearing_deg{};
  std::array<uint16_t, 360> range_mm{};
  std::array<uint8_t, 360> intensity{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &min_dist_mm, sizeof(min_dist_mm));
    std::memcpy(buf.data() + 2, &min_bearing_deg, sizeof(min_bearing_deg));
    std::memcpy(buf.data() + 4, &range_mm, sizeof(range_mm));
    std::memcpy(buf.data() + 724, &intensity, sizeof(intensity));
    return buf;
  }

  [[nodiscard]] static LidarScanPkt unpack(std::span<const std::byte> bytes) noexcept {
    LidarScanPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.min_dist_mm, bytes.data() + 0, sizeof(out.min_dist_mm));
    std::memcpy(&out.min_bearing_deg, bytes.data() + 2, sizeof(out.min_bearing_deg));
    std::memcpy(&out.range_mm, bytes.data() + 4, sizeof(out.range_mm));
    std::memcpy(&out.intensity, bytes.data() + 724, sizeof(out.intensity));
    return out;
  }
};

struct CpuTempPkt {
  static constexpr PacketID kId = PacketID::CPU_TEMP;
  static constexpr size_t kSize = 4;
  float temp_c{};

  [[nodiscard]] std::array<std::byte, kSize> pack() const noexcept {
    std::array<std::byte, kSize> buf{};
    std::memcpy(buf.data() + 0, &temp_c, sizeof(temp_c));
    return buf;
  }

  [[nodiscard]] static CpuTempPkt unpack(std::span<const std::byte> bytes) noexcept {
    CpuTempPkt out{};
    if (bytes.size() < kSize) return out;
    std::memcpy(&out.temp_c, bytes.data() + 0, sizeof(out.temp_c));
    return out;
  }
};

// Une entrée par paquet : priorité/taux/taille, utilisée par le scheduler
// d'émission (choix du prochain paquet dû) sans dupliquer ces constantes.
struct Meta {
  PacketID id;
  Direction direction;
  Priority priority;
  uint16_t rateHz;       // 0 = événementiel
  uint16_t redundancyHz; // pour événementiel : renvoi périodique même sans changement
  uint8_t linkMask;      // combinaison de Link (Main | Sec)
  uint16_t size;         // u16 : un paquet à tableau (ex: LIDAR_SCAN) dépasse 255 octets
  const char *name;
};

inline constexpr std::array<Meta, 27> kTable{{
    Meta{PacketID::POS, Direction::Down, Priority::Routine, 10, 0, static_cast<uint8_t>(Link::Main), 12, "POS"},
    Meta{PacketID::VELNED, Direction::Down, Priority::Routine, 10, 0, static_cast<uint8_t>(Link::Main), 12, "VELNED"},
    Meta{PacketID::ATTITUDE, Direction::Down, Priority::Routine, 10, 0, static_cast<uint8_t>(Link::Main), 16, "ATTITUDE"},
    Meta{PacketID::HEALTH, Direction::Down, Priority::Routine, 5, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 2, "HEALTH"},
    Meta{PacketID::GPSFIX, Direction::Down, Priority::Routine, 1, 0, static_cast<uint8_t>(Link::Main), 6, "GPSFIX"},
    Meta{PacketID::BATTERY, Direction::Down, Priority::Routine, 1, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 9, "BATTERY"},
    Meta{PacketID::CODE, Direction::Down, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 9, "CODE"},
    Meta{PacketID::ARM_STATE, Direction::Down, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 2, "ARM_STATE"},
    Meta{PacketID::MISSION_STATE, Direction::Down, Priority::Critical, 0, 1, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 1, "MISSION_STATE"},
    Meta{PacketID::AUTOPILOT_MODE, Direction::Down, Priority::Critical, 0, 1, static_cast<uint8_t>(Link::Main), 24, "AUTOPILOT_MODE"},
    Meta{PacketID::CMD_ARM, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 0, "CMD_ARM"},
    Meta{PacketID::CMD_DISARM, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 0, "CMD_DISARM"},
    Meta{PacketID::CMD_MODE, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 1, "CMD_MODE"},
    Meta{PacketID::CMD_WAYPOINT, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 20, "CMD_WAYPOINT"},
    Meta{PacketID::CMD_RTL, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 0, "CMD_RTL"},
    Meta{PacketID::CMD_LAND, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 0, "CMD_LAND"},
    Meta{PacketID::HEARTBEAT_GCS, Direction::Up, Priority::Routine, 1, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 0, "HEARTBEAT_GCS"},
    Meta{PacketID::CMD_EMERGENCY, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 0, "CMD_EMERGENCY"},
    Meta{PacketID::CMD_TAKEOFF, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 4, "CMD_TAKEOFF"},
    Meta{PacketID::CMD_HOLD, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 0, "CMD_HOLD"},
    Meta{PacketID::CMD_RESUME, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 0, "CMD_RESUME"},
    Meta{PacketID::CMD_ANSWER, Direction::Up, Priority::Critical, 0, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 5, "CMD_ANSWER"},
    Meta{PacketID::MC_QUERY, Direction::Down, Priority::Critical, 0, 1, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 5, "MC_QUERY"},
    Meta{PacketID::GPSPOS, Direction::Down, Priority::Routine, 1, 0, static_cast<uint8_t>(Link::Main), 16, "GPSPOS"},
    Meta{PacketID::SAT_INFO, Direction::Down, Priority::Routine, 1, 0, static_cast<uint8_t>(Link::Main), 49, "SAT_INFO"},
    Meta{PacketID::LIDAR_SCAN, Direction::Down, Priority::Routine, 2, 0, static_cast<uint8_t>(Link::Main), 1084, "LIDAR_SCAN"},
    Meta{PacketID::CPU_TEMP, Direction::Down, Priority::Routine, 1, 0, static_cast<uint8_t>(Link::Main) | static_cast<uint8_t>(Link::Sec), 4, "CPU_TEMP"},
}};

[[nodiscard]] constexpr const Meta *find(PacketID id) noexcept {
  for (const auto &m : kTable)
    if (m.id == id)
      return &m;
  return nullptr;
}

} // namespace TELEM
