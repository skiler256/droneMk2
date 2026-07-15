#pragma once
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <fcntl.h>
#include <optional>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

class WriterGuard {
public:
  explicit WriterGuard(std::atomic<bool> &flag) : flag_(flag) {
    flag_.store(true, std::memory_order_acquire);
  }

  ~WriterGuard() { flag_.store(false, std::memory_order_release); }

private:
  std::atomic<bool> &flag_;
};

struct SharedMemory {
  std::atomic<bool> initialized {false};
  std::array<std::atomic<bool>, static_cast<uint8_t>(TYPES::ComponentID::Count)>
      writersFlags;
  uint32_t checksum;
};

class SharedMemoryHandler {
public:
  explicit SharedMemoryHandler(SharedMemory &mem) : mem_(mem) {};

protected:
  SharedMemory &mem_;
  std::optional<TYPES::shmError>
      error; // le handler enregistre les problèmes d'acces shm

  virtual uint32_t computeChecksum() = 0;
  virtual void reset() = 0;

  void resetWriterFlag(TYPES::ComponentID id) {
    mem_.writersFlags[static_cast<uint8_t>(id)].store(
        false, std::memory_order_release);
  };

  template <typename T>
  [[nodiscard]] std::optional<T> getData(TYPES::ComponentID id,
                                         TYPES::Us timeout, T &t) {
    T data;
    auto fetch = access(id, timeout, [&data, &t] { data = t; });
    if (!fetch) {
      error.emplace(fetch.error());
      return {};
    } else {
      error.reset();
      return data;
    }
  };

  template <typename T>
  void setData(TYPES::ComponentID id, TYPES::Us timeout, T &t, T &value) {

    auto fetch = access(id, timeout, [&value, &t] { t = value; });
    if (!fetch) {
      error.emplace(fetch.error());
    } else {
      error.reset();
    }
  };

  template <typename T, typename A>
  std::optional<T> getArrayElement(TYPES::ComponentID id, TYPES::Us timeout,
                                   size_t i, A &array) {
    std::optional<T> data;
    auto fetch = getData(id, timeout, array);
    if (!fetch)
      return data;
    return data.emplace(fetch->at(i));
  };

  template <typename T, typename A>
  void setArrayElement(TYPES::ComponentID id, TYPES::Us timeout, size_t i, T &t,
                       A &array) {

    auto fetch = getData(id, timeout, array);
    if (!fetch)
      return;
    A &array_ = fetch.value();
    array_[i] = t;
    setData(id, timeout, array, array_);
  };

private:
  bool isAvailable() {
    for (const auto &flag : mem_.writersFlags) {
      if (flag)
        return false;
    }
    return true;
  };

  void init(){
    mem_.checksum = computeChecksum();
  }

  template <typename F>
  std::expected<void, TYPES::shmError> access(TYPES::ComponentID id,
                                              TYPES::Us timeout, F &&f) {
    if (UTILITIES::waitUntil([&]() { return isAvailable(); }, timeout))

    {

      WriterGuard guard(mem_.writersFlags[static_cast<uint8_t>(id)]);

      if(!mem_.initialized.load(std::memory_order_acquire)){
        init();
        mem_.initialized.store(true, std::memory_order_release);
      }

      if (mem_.checksum != computeChecksum()) {
        return std::unexpected(TYPES::shmError::corrupt);
      }

      std::forward<F>(f)();
      mem_.checksum = computeChecksum();
      return {};
    } else
      return std::unexpected(TYPES::shmError::timeout);
  };
};