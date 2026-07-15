#pragma once
#include "drone/core/SharedMemory.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <cstdint>
#include <optional>

struct SharedSysStateMem {
  SharedMemory mem;
  std::array<TYPES::ComponentHealth,
             static_cast<uint8_t>(TYPES::ComponentID::Count)>
      compHealth;
};

class SharedSysStateMemHandler : SharedMemoryHandler {
public:
  explicit SharedSysStateMemHandler(SharedSysStateMem &state, TYPES::Us timeout)
      : SharedMemoryHandler(state.mem), state_(state), timeout_(timeout) {};

  std::optional<TYPES::ComponentHealth> getHealth(TYPES::ComponentID id) {
    return getArrayElement<TYPES::ComponentHealth>(
        id, timeout_, static_cast<uint8_t>(id), state_.compHealth);
  };

  void setHealth(TYPES::ComponentID id, TYPES::ComponentHealth health) {
    setArrayElement(id, timeout_, static_cast<uint8_t>(id), health,
                    state_.compHealth);
  };

private:
  SharedSysStateMem &state_;
  TYPES::Us timeout_;

  uint32_t computeChecksum() { return UTILITIES::crc32(state_); };

  void reset() { // a implémenter
  };
};