#pragma once
#include <array>
#include <cstddef>
#include <mavlink.h>
#include <mutex>
#include <optional>

// Unique canal par lequel TTxACK apprend qu'un ACK est arrivé, TRx étant
// seule à lire le fd UART — cf. MAVLINKINTERFACE.md §1-2. Écrase le plus
// ancien si plein (à l'inverse de PendingAckQueue, cf. son en-tête).
class AckFifo {
public:
  static constexpr size_t kCapacity = 8;

  void push(const mavlink_message_t &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    buf_[head_] = msg;
    head_ = (head_ + 1) % kCapacity;
    if (count_ < kCapacity)
      ++count_;
    else
      tail_ = (tail_ + 1) % kCapacity; // plein : le plus ancien est écrasé
  }

  [[nodiscard]] std::optional<mavlink_message_t> pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ == 0)
      return std::nullopt;
    mavlink_message_t msg = buf_[tail_];
    tail_ = (tail_ + 1) % kCapacity;
    --count_;
    return msg;
  }

private:
  std::mutex mutex_;
  std::array<mavlink_message_t, kCapacity> buf_{};
  size_t head_{0};
  size_t tail_{0};
  size_t count_{0};
};
