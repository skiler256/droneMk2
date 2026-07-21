#pragma once
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/Components/System Monitoring/SharedComMem.hpp"
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
inline constexpr std::string_view kComShmPath = "/drone_com";

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
  ShmPublisher<SharedComMem> comShm_;

  std::optional<SharedNavMemHandler> navHandler_;
  std::optional<SharedSFMemHandler> sfHandler_;
  std::optional<SharedSysStateMemHandler> sysHandler_;
  std::optional<SharedComMemHandler> comHandler_;

  std::array<ChildProc, 3> children_{};

  static inline std::atomic<bool> keepRunning_{true};

  static void signalHandler(int) {
    keepRunning_.store(false, std::memory_order_release);
  }

  // sigaction() (pas std::signal()) : sur Linux/glibc signal() active
  // SA_RESTART par défaut, ce qui relance waitpid() au lieu de le faire
  // échouer avec EINTR — monitorLoop() resterait bloqué indéfiniment.
  void installSignalHandler() {
    struct sigaction sa {};
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; // pas de SA_RESTART

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
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

    auto com = publishSharedMemory<SharedComMem>(kComShmPath);
    if (!com) return false;
    comShm_ = com.value();

    // GlobalWatchdog s'enregistre lui meme comme "writer" ID pour ces
    // handlers : il ne fait qu'appeler resetWriterFlag() dessus, jamais
    // getData/setData, donc l'id passe ici ne sert qu'a satisfaire les
    // constructeurs de SharedCompMemHandler.
    navHandler_.emplace(TYPES::ComponentID::GlobalWatchdog, TYPES::Us(2000),
                        *navShm_.ptr);
    sfHandler_.emplace(TYPES::ComponentID::GlobalWatchdog, TYPES::Us(2000),
                       *sfShm_.ptr);
    sysHandler_.emplace(*sysShm_.ptr, TYPES::ComponentID::GlobalWatchdog,
                        TYPES::Us(2000));
    comHandler_.emplace(TYPES::ComponentID::GlobalWatchdog, TYPES::Us(2000),
                        *comShm_.ptr);

    return true;
  }

  void forkChildren() {
    children_[0] = spawn(TYPES::ComponentID::Navigation, "navigation");
    children_[1] = spawn(TYPES::ComponentID::SensorFusion, "sensorfusion");
    children_[2] = spawn(TYPES::ComponentID::SysMonitoring, "sysmonitoring");
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
    // waitpid(-1, ..., 0) bloque jusqu'à ce qu'UN enfant change d'état —
    // pas de poll, réveil immédiat par le kernel.
    while (keepRunning_.load(std::memory_order_acquire)) {
      int status = 0;
      pid_t pid = waitpid(-1, &status, 0);

      if (pid == -1)
        continue; // EINTR (signal reçu) ou ECHILD : on reboucle

      for (auto &child : children_) {
        if (child.pid == pid) {
          if (!keepRunning_.load(std::memory_order_acquire)) {
            // Ctrl+C envoie le signal à tout le groupe, donc à cet enfant
            // aussi : ne pas le respawn, juste noter qu'il est déjà mort.
            child.pid = -1;
          } else {
            std::cerr << "GlobalWatchdog: " << child.role
                      << " est mort, reset writer flags + respawn\n";
            resetAll(child.id);
            child = spawn(child.id, child.role);
          }
          break;
        }
      }
    }
  }

  void resetAll(TYPES::ComponentID id) {
    navHandler_->resetWriterFlag(id);
    sfHandler_->resetWriterFlag(id);
    sysHandler_->resetWriterFlag(id);
    comHandler_->resetWriterFlag(id);
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
    destroySharedMemory(kComShmPath, comShm_);
  }
};