#pragma once
#include "drone/generated/telemetry.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <array>
#include <cstddef>

// Choisit, à chaque cycle radio, le paquet TELEM::* à émettre en priorité
// sur un lien donné (TelMain ou TelSec) — un seul paquet par cycle, le
// lien ne permettant qu'un envoi à la fois.
//
// Règles (cf. discussion archi) :
//  - seuls les paquets descendants (Direction::Down) sont schedulés ici ;
//    les montants (commandes GCS) sont reçus via ITelemetryLink::poll(),
//    pas envoyés depuis ce scheduler.
//  - Critical passe toujours avant Routine.
//  - à priorité égale, le paquet le plus en retard sur sa période gagne
//    (pas un round-robin naïf) — évite qu'un paquet haute fréquence
//    affame silencieusement un paquet plus lent.
//  - un paquet événementiel (rateHz == 0) est dû dès que markDirty() a été
//    appelé pour lui, ou — s'il a une redondance configurée — à la
//    fréquence de redondance même sans changement (résistance à la perte
//    du paquet événementiel sur un lien lossy, sans ACK).
class PacketScheduler {
public:
  explicit PacketScheduler(TELEM::Link link) : link_(link) {
    for (const auto &m : TELEM::kTable) {
      if (m.direction != TELEM::Direction::Down)
        continue;
      if ((m.linkMask & static_cast<uint8_t>(link_)) == 0)
        continue;
      slots_[count_++] = Slot{&m, TYPES::Clock::now(), false};
    }
  }

  // À appeler par la Task quand elle détecte que l'état source d'un paquet
  // événementiel a changé (ex: MISSION_STATE) — force son envoi au
  // prochain cycle dû, en plus de sa redondance périodique.
  void markDirty(TELEM::PacketID id) {
    if (Slot *s = find(id))
      s->pending = true;
  }

  // Le paquet le plus urgent dû à cet instant, ou nullptr si rien n'est dû.
  // Ne modifie pas l'état interne : c'est markSent() qui fait foi, une fois
  // l'envoi réellement effectué (permet à l'appelant de gérer un échec de
  // send() sans perdre le paquet).
  [[nodiscard]] const TELEM::Meta *nextDue(TYPES::TimePoint now) const {
    const Slot *best = nullptr;
    TYPES::Ms bestOverdue{0};

    for (size_t i = 0; i < count_; ++i) {
      const Slot &s = slots_[i];
      TYPES::Ms overdue{};
      if (!isDue(s, now, overdue))
        continue;

      if (best == nullptr) {
        best = &s;
        bestOverdue = overdue;
        continue;
      }

      auto prio = static_cast<uint8_t>(s.meta->priority);
      auto bestPrio = static_cast<uint8_t>(best->meta->priority);
      if (prio > bestPrio || (prio == bestPrio && overdue > bestOverdue)) {
        best = &s;
        bestOverdue = overdue;
      }
    }

    return best ? best->meta : nullptr;
  }

  void markSent(TELEM::PacketID id, TYPES::TimePoint now) {
    if (Slot *s = find(id)) {
      s->lastSent = now;
      s->pending = false;
    }
  }

private:
  struct Slot {
    const TELEM::Meta *meta{nullptr};
    TYPES::TimePoint lastSent{};
    bool pending{false};
  };

  // true si s est dû à `now` ; renvoie dans `overdue` de combien (utilisé
  // pour départager deux paquets dus à la même priorité). Pour un
  // événementiel en attente (pending), overdue est forcé au maximum : un
  // changement d'état prime toujours sur la redondance périodique d'un
  // autre paquet de même priorité.
  [[nodiscard]] static bool isDue(const Slot &s, TYPES::TimePoint now,
                                  TYPES::Ms &overdue) {
    if (s.meta->rateHz > 0) {
      auto period = TYPES::Ms(1000 / s.meta->rateHz);
      overdue = UTILITIES::msBetween(s.lastSent, now) - period;
      return overdue.count() >= 0;
    }

    if (s.pending) {
      overdue = TYPES::Ms::max();
      return true;
    }

    if (s.meta->redundancyHz > 0) {
      auto period = TYPES::Ms(1000 / s.meta->redundancyHz);
      overdue = UTILITIES::msBetween(s.lastSent, now) - period;
      return overdue.count() >= 0;
    }

    return false;
  }

  [[nodiscard]] Slot *find(TELEM::PacketID id) {
    for (size_t i = 0; i < count_; ++i)
      if (slots_[i].meta->id == id)
        return &slots_[i];
    return nullptr;
  }

  TELEM::Link link_;
  std::array<Slot, TELEM::kTable.size()> slots_{};
  size_t count_{0};
};
