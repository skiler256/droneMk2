#pragma once
#include "drone/Components/MavlinkInterface/PendingAck.hpp"
#include <array>
#include <cstddef>
#include <mutex>
#include <optional>

// FIFO borné de commandes ACK'd EN ATTENTE D'ENVOI — alimentée par TCmd
// (routage MissionControl, pas encore implémenté) ou par des méthodes
// publiques ponctuelles comme MavlinkInterface::requestInitStreams().
// Dépilée uniquement par TTxACK (cf. MavlinkInterface.cpp, étape 3 de
// TTxAckTask::loop()), un élément à la fois — jamais deux commandes ACK'd
// en vol simultanément, cf. MAVLINKINTERFACE.md.
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

private:
  std::mutex mutex_;
  std::array<PendingAck, kCapacity> buf_{};
  size_t head_{0};
  size_t tail_{0};
  size_t count_{0};
};
