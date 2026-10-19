#pragma once
#include "drone/generated/telemetry.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <array>
#include <cstddef>

// Choisit le paquet TELEM::* à émettre, un seul par cycle radio — règles
// de départage : SYSMONITORING.md §5.
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

  void markDirty(TELEM::PacketID id) {
    if (Slot *s = find(id))
      s->pending = true;
  }

  // Ne modifie pas l'état : markSent() fait foi, cf. SYSMONITORING.md §5.
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

  // `overdue` = de combien le paquet est en retard, sert au départage.
  // Forcé au maximum pour un événementiel en attente (cf. §5, règle 4).
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
