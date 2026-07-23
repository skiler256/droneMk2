#pragma once
#include <array>
#include <cstddef>
#include <mavlink.h>
#include <mutex>
#include <optional>

// FIFO borné, thread-safe, pour transmettre les ACK (COMMAND_ACK/
// PARAM_VALUE) de TRx vers TTxACK. TRx est la SEULE Task à lire le fd
// UART (deux lecteurs concurrents mélangeraient le flux d'octets) — ce
// FIFO est donc l'unique canal par lequel TTxACK apprend qu'un ACK est
// arrivé. Écrase le plus ancien si plein plutôt que d'allouer
// dynamiquement (cohérent avec le reste du code de vol, -fno-exceptions).
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
