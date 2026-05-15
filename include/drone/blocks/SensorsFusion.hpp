#pragma once
#include "drone/core/BlockBase.hpp"
#include "drone/shared_state.hpp"
#include "drone/types.hpp"
#include <string_view>
#include <termios.h>
#include <unistd.h>


#include <queue>
#include <mutex>
#include <optional>

namespace CORE {
class SensorsFusion; // Forward declaration
}

namespace Sensors {

class DriverBase : public CORE::BlockBase {

  DriverBase(SensorsT name, CORE::SensorsFusion &sens, CORE::Watchdog &watchdog)
  : BlockBase(toString(name),50 , Ms{100},
                                watchdog),name(name),name_(toString(name)),
                                sens(sens){}
  

protected:
  SensorsT name;
  std::string_view name_;
  CORE::SensorsFusion &sens;
  int file = -1;
  termios tty;

  bool openI2C(const char *bus, uint8_t address);
  bool openUART(const char *portName, speed_t baudrate);
};

template<typename T>
struct Measurement {
    TimePoint stamp;
    T         data;
};

template<typename T>
class Queue {
public:
    // Appelé depuis le thread Driver (producteur)
    void push(TimePoint stamp, T data) noexcept {
        std::unique_lock lock(mutex_);
        // Limite la taille — évite la croissance infinie si Kalman est lent
        if (queue_.size() < MAX_SIZE) {
            queue_.push({stamp, std::move(data)});
        }
        // Si plein : on drop silencieusement (la mesure est trop vieille)
    }

    // Appelé depuis le thread SensorsFusion (consommateur)
    // Retourne nullopt si la queue est vide
    [[nodiscard]] std::optional<Measurement<T>> pop() noexcept {
        std::unique_lock lock(mutex_);
        if (queue_.empty()) return std::nullopt;
        auto m = queue_.front();
        queue_.pop();
        return m;
    }

    [[nodiscard]] bool empty() const noexcept {
        std::unique_lock lock(mutex_);
        return queue_.empty();
    }

private:
    static constexpr size_t MAX_SIZE = 16;  // au-delà = mesure trop vieille
    mutable std::mutex      mutex_;
    std::queue<Measurement<T>> queue_;
};


} // namespace Sensors

namespace CORE {
class SensorsFusion : public BlockBase {
public:
  SensorsFusion(CORE::SharedStateProvider &state, IFCStatus &fc, Watchdog &wd)
      : BlockBase("SensorsFusion", /*prio=*/70, Ms{500}, wd), state(state),
        fc(fc) {}

  // Entrées données depuis drivers
  void push_Tel(TimePoint stamp, Meters distance) noexcept {
        tel_queue_.push(stamp, distance);
    }
    void push_Mag(TimePoint stamp, magnetic_field B) noexcept {
        mag_queue_.push(stamp, B);
    }
    void push_Acc(TimePoint stamp, Acceleration acc) noexcept {
        acc_queue_.push(stamp, acc);
    }
    void push_Att(TimePoint stamp, Attitude att) noexcept {
        att_queue_.push(stamp, att);
    }
  void push_GPS(TimePoint date); // a finir

  // rapport de défaillance des capteurs (depuis les drivers)
  void report(SensorsT name, SensorError error, HealthStatus status);
  void erease_report(SensorsT name, SensorError error);

protected:
private:
    Sensors::Queue<Meters>       tel_queue_;
    Sensors::Queue<magnetic_field> mag_queue_;
    Sensors::Queue<Acceleration> acc_queue_;
    Sensors::Queue<Attitude>     att_queue_;

  SharedStateProvider &state;
  IFCStatus &fc;
};
} // namespace CORE
