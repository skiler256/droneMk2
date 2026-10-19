#pragma once
#include "drone/types.hpp"
#include <cerrno>
#include <array>
#include <cstdint>
#include <cstring>
#include <expected>
#include <optional>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

template <typename T> class IDriver {
public:
  IDriver() = default;
  virtual ~IDriver() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // Un descripteur de fichier ne se copie pas ; le move reste possible.
  IDriver(const IDriver &) = delete;
  IDriver &operator=(const IDriver &) = delete;
  IDriver(IDriver &&) = delete;
  IDriver &operator=(IDriver &&) = delete;

  [[nodiscard]] virtual std::optional<TYPES::DriverError> init() = 0;
  [[nodiscard]] virtual std::expected<T, TYPES::DriverError> update() = 0;

  [[nodiscard]] TYPES::DriverHealth getHealth() const { return health_; }

protected:
  // protected et non private : les dérivées les appellent depuis init().

  [[nodiscard]] std::optional<TYPES::DriverError>
  i2cConnect(std::uint8_t i2cAddress, const char *busPath = "/dev/i2c-1") {
    int fd = ::open(busPath, O_RDWR);
    if (fd < 0) {
      health_ = TYPES::DriverHealth::Unconnected;
      return TYPES::DriverError::I2COpenFailed;
    }

    if (::ioctl(fd, I2C_SLAVE, i2cAddress) < 0) {
      ::close(fd);
      health_ = TYPES::DriverHealth::Unconnected;
      return TYPES::DriverError::I2CAddressFailed;
    }

    fd_ = fd;
    health_ = TYPES::DriverHealth::Connected;
    return std::nullopt;
  }

  [[nodiscard]] std::optional<TYPES::DriverError>
  uartConnect(const char *portName, speed_t baudRate = B115200) {
    int fd = ::open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
      health_ = TYPES::DriverHealth::Unconnected;
      return TYPES::DriverError::UARTOpenFailed;
    }

    std::memset(&tty_, 0, sizeof(tty_));
    if (::tcgetattr(fd, &tty_) != 0) {
      ::close(fd);
      health_ = TYPES::DriverHealth::Unconnected;
      return TYPES::DriverError::UARTAttrGetFailed;
    }

    cfsetispeed(&tty_, baudRate);
    cfsetospeed(&tty_, baudRate);
    cfmakeraw(&tty_); // 8N1, pas de contrôle de flux, mode brut

    tty_.c_cc[VMIN] = 0;  // read() non bloquant...
    tty_.c_cc[VTIME] = 5; // ...avec un timeout de 0.5s

    if (::tcsetattr(fd, TCSANOW, &tty_) != 0) {
      ::close(fd);
      health_ = TYPES::DriverHealth::Unconnected;
      return TYPES::DriverError::UARTAttrSetFailed;
    }

    fd_ = fd;
    health_ = TYPES::DriverHealth::Connected;
    return std::nullopt;
  }

  void setHealth(TYPES::DriverHealth h) { health_ = h; }

  int fd_{-1};
  termios tty_{};

private:
  TYPES::DriverHealth health_{TYPES::DriverHealth::Unconnected};
};

struct GPSData {
  struct SatelliteInfo {
    std::uint8_t id{0};
    std::uint8_t strength{0};
    std::uint8_t quality{0};
  };

  struct Coord {
    double latitude{44.0};
    double longitude{0.7};
  };

  Coord coord{};

  // Tableau FIXE, jamais un vector : ce driver tourne en SCHED_FIFO.
  static constexpr std::size_t kMaxSatDetail = 16;
  std::array<SatelliteInfo, kMaxSatDetail> sats{};
  std::uint8_t satDetailCount{0}; // entrées valides de `sats`

  // Canaux suivis (numCh), NON plafonné — distinct du détail ci-dessus,
  // cf. SENSORFUSION.md §3.
  std::uint8_t satellitesTracked{0};

  // year, month, day, hour, min, sec (UBX-NAV-TIMEUTC)
  int timeArray[6]{0, 0, 0, 0, 0, 0};

  bool gpsFixOk{false}; // fix 3D uniquement

  // Brut UBX, pas la numérotation MAVLink (conversion côté SensorFusion).
  std::uint8_t fixType{0};

  std::uint32_t pAcc{0}; // mm, horizontal (NAV-PVT hAcc)
  std::uint32_t sAcc{0}; // mm/s (NAV-PVT sAcc)
  std::uint32_t vAcc{0}; // mm, vertical (NAV-POSLLH vAcc)

  float hdop{0.0f}; // sans dimension, NAV-DOP hDOP (scale 0.01)
  float vdop{0.0f}; // sans dimension, NAV-DOP vDOP (scale 0.01)

  std::int32_t altMslMm{0}; // mm, altitude au-dessus du niveau de la mer (NAV-POSLLH hMSL)

  // iTOW brut : déjà le format de GPS_INPUT.time_week_ms, calculé par le
  // récepteur et non par l'horloge du Pi.
  std::uint32_t iTowMs{0};

  int velNED[3]{0, 0, 0}; // vNORTH, vEAST, vDOWN (cm/s)
  std::uint32_t speed{0};
  std::uint32_t groundSpeed{0};
  double heading{0.0};
};

struct MagData {
  TYPES::MagneticField field{}; // Gauss, repère capteur
};

struct LidarData {
    // Indexé par angle (0-359°) ; 0 = aucune mesure (sentinelle).
    std::array<std::uint16_t, 360> distance_mm{};
    std::array<std::uint8_t, 360> intensity{};
};
