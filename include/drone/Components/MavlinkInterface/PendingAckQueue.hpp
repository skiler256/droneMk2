#pragma once
#include "drone/Components/MavlinkInterface/PendingAck.hpp"
#include <array>
#include <cstddef>
#include <mutex>
#include <optional>

// Commandes ACK'd en attente d'ENVOI (PendingAck déjà construits, frame
// packée incluse), à l'inverse d'AckFifo qui route des messages reçus.
// Dépilée par TTxACK seule, un élément à la fois.
class PendingAckQueue {
public:
  static constexpr size_t kCapacity = 8;

  // Échoue si pleine, jamais d'écrasement : perdre une commande de
  // configuration est plus grave que perdre un ACK redondant.
  [[nodiscard]] bool push(const PendingAck &req) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ >= kCapacity)
      return false;
    buf_[head_] = req;
    head_ = (head_ + 1) % kCapacity;
    ++count_;
    return true;
  }

  [[nodiscard]] std::optional<PendingAck> pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ == 0)
      return std::nullopt;
    PendingAck req = buf_[tail_];
    tail_ = (tail_ + 1) % kCapacity;
    --count_;
    return req;
  }

  // Abandonne la séquence en cours sur échec, au lieu d'enchaîner
  // aveuglément (ex: guided échoué -> arm -> takeoff quand même).
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = tail_ = count_ = 0;
  }

private:
  std::mutex mutex_;
  std::array<PendingAck, kCapacity> buf_{};
  size_t head_{0};
  size_t tail_{0};
  size_t count_{0};
};
