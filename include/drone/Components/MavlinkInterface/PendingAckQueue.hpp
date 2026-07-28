#pragma once
#include "drone/Components/MavlinkInterface/PendingAck.hpp"
#include <array>
#include <cstddef>
#include <mutex>
#include <optional>

// FIFO borné de commandes ACK'd EN ATTENTE D'ENVOI — alimentée par TCmd
// (ordres MissionControl) ou par des méthodes ponctuelles comme
// requestInitStreams(). Dépilée uniquement par TTxACK, un élément à la
// fois (jamais deux commandes ACK'd en vol simultanément).
//
// Contrairement à AckFifo (qui route des messages REÇUS), ici chaque
// élément est déjà un PendingAck entièrement construit (frame packée
// incluse) : TTxACK n'a qu'à le recopier dans pendingAck_ et l'envoyer.
class PendingAckQueue {
public:
  static constexpr size_t kCapacity = 8;

  // Échoue (renvoie false) si la queue est pleine plutôt que d'écraser une
  // commande déjà en attente d'envoi — contrairement à AckFifo, perdre une
  // commande de configuration silencieusement serait plus grave que perdre
  // un ACK redondant.
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

  // Vide la queue sans envoyer ce qui reste — utilisé par TTxACK quand une
  // commande échoue (DENIED/FAILED/timeout) pour abandonner la séquence en
  // cours plutôt que d'enchaîner aveuglément (ex: guided échoué -> arm
  // -> takeoff quand même).
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
