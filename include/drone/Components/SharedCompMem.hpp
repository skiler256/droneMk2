#pragma once
#include "drone/core/SharedMemory.hpp"
#include "drone/types.hpp"
#include <cstddef>
#include <expected>
#include <optional>
#include <unistd.h>

#define MAX_COMPONENT_RESTART 10

struct SharedCompMem {

  SharedMemory mem;

  std::array<TYPES::TimePoint, MAX_COMPONENT_RESTART> HotStartHistory;
  std::array<TYPES::TimePoint, MAX_COMPONENT_RESTART> ColdStartHistory;
};

class SharedCompMemHandler : public SharedMemoryHandler {
public:
  explicit SharedCompMemHandler(SharedCompMem &comp, TYPES::ComponentID id,
                                TYPES::Us timeout)
      : SharedMemoryHandler(comp.mem), comp_(comp), id_(id), timeout_(timeout) {

  }

  void recordHotStart(TYPES::TimePoint ts) {
    recordStart(comp_.HotStartHistory, ts);
  }

  void recordColdStart(TYPES::TimePoint ts) {
    recordStart(comp_.ColdStartHistory, ts);
  };

  [[nodiscard]] std::optional<TYPES::TimePoint> getHotStartTs(size_t i) {
    return getStart(comp_.HotStartHistory, i);
  }

  [[nodiscard]] std::optional<TYPES::TimePoint> getColdtStartTs(size_t i) {
    return getStart(comp_.ColdStartHistory, i);
  };

protected:
  SharedCompMem &comp_;
  TYPES::ComponentID id_;
  TYPES::Us timeout_;

  // A combiner avec le checksum du payload de la sous-classe, sinon une
  // corruption de l'historique de redémarrage passe inaperçue.
  uint32_t historyChecksum() {
    return UTILITIES::crc32(comp_.HotStartHistory) ^
           UTILITIES::crc32(comp_.ColdStartHistory);
  };

  // Filet partiel : ne détecte que les horodatages dans le futur
  // (impossibles). TimePoint{} (slot jamais utilisé) n'est pas corrompu.
  void sanitizeHistory(
      std::array<TYPES::TimePoint, MAX_COMPONENT_RESTART> &history) {
    auto now = TYPES::Clock::now();
    for (auto &ts : history) {
      if (ts > now)
        ts = TYPES::TimePoint{};
    }
  };

private:
  void recordStart(std::array<TYPES::TimePoint, MAX_COMPONENT_RESTART> &history,
                   TYPES::TimePoint ts) {
    // repairOnCorrupt=true : intention d'écriture malgré le getData interne.
    auto fetch = getData(id_, timeout_, history, /*repairOnCorrupt=*/true);
    if (!fetch)
      return;
    std::array<TYPES::TimePoint, MAX_COMPONENT_RESTART> &history_ =
        fetch.value();
    for (size_t i = MAX_COMPONENT_RESTART - 1; i > 0; --i) {
      history_[i] = history_[i - 1];
    }
    history_[0] = ts;

    setData(id_, timeout_, history, history_);
  };

  std::optional<TYPES::TimePoint>
  getStart(std::array<TYPES::TimePoint, MAX_COMPONENT_RESTART> &history, size_t i) {
    auto data = getData(id_, timeout_, history);
    if (data)
      return data->at(i);
    return {};
  };
};

// rajouter retour erreur