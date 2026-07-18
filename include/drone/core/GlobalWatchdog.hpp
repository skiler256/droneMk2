#pragma once
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/core/SharedSysStateMem.hpp"
#include "drone/shm.hpp"
#include "drone/types.hpp"

#include <array>
#include <atomic>
#include <csignal>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include <sys/wait.h>
#include <unistd.h>

// Noms des segments shm — un seul endroit pour les faire evoluer
inline constexpr std::string_view kNavShmPath = "/drone_nav";
inline constexpr std::string_view kSFShmPath = "/drone_sf";
inline constexpr std::string_view kSysShmPath = "/drone_sysstate";

class GlobalWatchdog {
public:
  explicit GlobalWatchdog(std::string binaryPath)
      : binaryPath_(std::move(binaryPath)) {}
    
  void run() {
    if (!publishAll()) {
      std::cerr << "GlobalWatchdog: echec publication shm, abandon\n";
      return;
    }
    installSignalHandler();
    forkChildren();
    monitorLoop();
    shutdown();
  }

private:
  struct ChildProc {
    pid_t pid = -1;
    TYPES::ComponentID id{};
    const char *role = nullptr;
  };

  std::string binaryPath_;

  ShmPublisher<SharedNavMem> navShm_;
  ShmPublisher<SharedSFMem> sfShm_;
  ShmPublisher<SharedSysStateMem> sysShm_;

  std::optional<SharedNavMemHandler> navHandler_;
  std::optional<SharedSFMemHandler> sfHandler_;
  std::optional<SharedSysStateMemHandler> sysHandler_;

  std::array<ChildProc, 2> children_{};

  static inline std::atomic<bool> keepRunning_{true};

  static void signalHandler(int) {
    keepRunning_.store(false, std::memory_order_release);
  }

  void installSignalHandler() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
  }

  [[nodiscard]] bool publishAll() {
    auto nav = publishSharedMemory<SharedNavMem>(kNavShmPath);
    if (!nav) return false;
    navShm_ = nav.value();

    auto sf = publishSharedMemory<SharedSFMem>(kSFShmPath);
    if (!sf) return false;
    sfShm_ = sf.value();

    auto sys = publishSharedMemory<SharedSysStateMem>(kSysShmPath);
    if (!sys) return false;
    sysShm_ = sys.value();

    // GlobalWatchdog s'enregistre lui meme comme "writer" ID pour ces
    // handlers : il ne fait qu'appeler resetWriterFlag() dessus, jamais
    // getData/setData, donc l'id passe ici ne sert qu'a satisfaire les
    // constructeurs de SharedCompMemHandler.
    navHandler_.emplace(TYPES::ComponentID::GlobalWatchdog, TYPES::Us(2000),
                        *navShm_.ptr);
    sfHandler_.emplace(TYPES::ComponentID::GlobalWatchdog, TYPES::Us(2000),
                       *sfShm_.ptr);
    sysHandler_.emplace(*sysShm_.ptr, TYPES::Us(2000));

    return true;
  }

  void forkChildren() {
    children_[0] = spawn(TYPES::ComponentID::Navigation, "navigation");
    children_[1] = spawn(TYPES::ComponentID::SensorFusion, "sensorfusion");
  }

  ChildProc spawn(TYPES::ComponentID id, const char *role) {
    pid_t pid = fork();
    if (pid == 0) {
      execl(binaryPath_.c_str(), binaryPath_.c_str(), role, nullptr);
      std::cerr << "GlobalWatchdog: execl a echoue pour " << role << "\n";
      _exit(127);
    }
    return ChildProc{pid, id, role};
  }

  void monitorLoop() {
    while (keepRunning_.load(std::memory_order_acquire)) {
      for (auto &child : children_) {
        int status = 0;
        pid_t res = waitpid(child.pid, &status, WNOHANG);
        if (res == child.pid) {
          std::cerr << "GlobalWatchdog: " << child.role
                    << " est mort, reset writer flags + respawn\n";
          resetAll(child.id);
          child = spawn(child.id, child.role);
        }
      }
      sleep(1); // provisoire : a remplacer par un vrai thread RT plus tard
    }
  }

  void resetAll(TYPES::ComponentID id) {
    navHandler_->resetWriterFlag(id);
    sfHandler_->resetWriterFlag(id);
    sysHandler_->resetWriterFlag(id);
  }

  void shutdown() {
    std::cout << "GlobalWatchdog: arret, nettoyage\n";

    for (auto &child : children_) {
      if (child.pid > 0) {
        kill(child.pid, SIGTERM);
        waitpid(child.pid, nullptr, 0);
      }
    }

    // GlobalWatchdog est le seul proprietaire : destroy = unmap + close + unlink
    destroySharedMemory(kNavShmPath, navShm_);
    destroySharedMemory(kSFShmPath, sfShm_);
    destroySharedMemory(kSysShmPath, sysShm_);
  }
};