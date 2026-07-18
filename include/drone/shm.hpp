#pragma once

#include <expected>
#include <new>
#include <string_view>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

enum class ShmError {
  OpenFailed,
  TruncateFailed,
  MmapFailed,
  UnmapFailed,
  CloseFailed,
  UnlinkFailed
};

template <typename T>
struct ShmPublisher {
  int fd = -1;
  T *ptr = nullptr;
};

// ─────────────────────────────────────────────────────────────────────────
// publishSharedMemory : crée le segment + construit l'objet T dedans.
// Appelé UNIQUEMENT par le process propriétaire (GlobalWatchdog), une
// seule fois par segment, au démarrage.
// ─────────────────────────────────────────────────────────────────────────
template <typename T>
std::expected<ShmPublisher<T>, ShmError>
publishSharedMemory(std::string_view path) {
  ShmPublisher<T> shm;

  shm.fd = shm_open(path.data(), O_CREAT | O_RDWR, 0666);
  if (shm.fd < 0)
    return std::unexpected(ShmError::OpenFailed);

  if (ftruncate(shm.fd, sizeof(T)) != 0) {
    close(shm.fd);
    shm_unlink(path.data());
    return std::unexpected(ShmError::TruncateFailed);
  }

  void *mem = mmap(nullptr, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED,
                   shm.fd, 0);
  if (mem == MAP_FAILED) {
    close(shm.fd);
    shm_unlink(path.data());
    return std::unexpected(ShmError::MmapFailed);
  }

  shm.ptr = new (mem) T{}; // construction : le segment n'existait pas avant

  return shm;
}

// ─────────────────────────────────────────────────────────────────────────
// attachSharedMemory : se connecte à un segment déjà publié par le
// propriétaire. PAS de O_CREAT, PAS de placement-new (l'objet existe déjà
// et est géré ailleurs). Utilisé par les process enfants (Navigation,
// SensorFusion, etc).
// ─────────────────────────────────────────────────────────────────────────
template <typename T>
std::expected<ShmPublisher<T>, ShmError>
attachSharedMemory(std::string_view path) {
  ShmPublisher<T> shm;

  shm.fd = shm_open(path.data(), O_RDWR, 0666);
  if (shm.fd < 0)
    return std::unexpected(ShmError::OpenFailed);

  void *mem = mmap(nullptr, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED,
                   shm.fd, 0);
  if (mem == MAP_FAILED) {
    close(shm.fd);
    return std::unexpected(ShmError::MmapFailed);
  }

  shm.ptr = static_cast<T *>(mem); // pas de construction : deja fait par le publisher

  return shm;
}

// ─────────────────────────────────────────────────────────────────────────
// destroySharedMemory : détruit l'objet, unmap, close, ET unlink le nom.
// Appelé UNIQUEMENT par le propriétaire (GlobalWatchdog) à l'arrêt propre.
// ─────────────────────────────────────────────────────────────────────────
template <typename T>
std::expected<void, ShmError>
destroySharedMemory(std::string_view path, ShmPublisher<T> &shm) {
  if (shm.ptr) {
    shm.ptr->~T();

    if (munmap(shm.ptr, sizeof(T)) != 0)
      return std::unexpected(ShmError::UnmapFailed);

    shm.ptr = nullptr;
  }

  if (shm.fd >= 0) {
    if (close(shm.fd) != 0)
      return std::unexpected(ShmError::CloseFailed);

    shm.fd = -1;
  }

  if (shm_unlink(path.data()) != 0)
    return std::unexpected(ShmError::UnlinkFailed);

  return {};
}

// ─────────────────────────────────────────────────────────────────────────
// detachSharedMemory : unmap + close SANS unlink. Appelé par un process
// non-propriétaire (enfant) qui veut se détacher proprement sans supprimer
// le segment pour les autres.
// ─────────────────────────────────────────────────────────────────────────
template <typename T>
std::expected<void, ShmError> detachSharedMemory(ShmPublisher<T> &shm) {
  if (shm.ptr) {
    if (munmap(shm.ptr, sizeof(T)) != 0)
      return std::unexpected(ShmError::UnmapFailed);

    shm.ptr = nullptr;
  }

  if (shm.fd >= 0) {
    if (close(shm.fd) != 0)
      return std::unexpected(ShmError::CloseFailed);

    shm.fd = -1;
  }

  return {};
}