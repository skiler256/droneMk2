// tests/test_global_watchdog.cpp
// ─────────────────────────────────────────────────────────────────────────────
// GlobalWatchdog en boîte noire : run() bloque indéfiniment (publish shm +
// fork + monitorLoop + shutdown), donc chaque scénario est lancé dans un
// process forké dédié, contrôlé depuis le process de test via signaux, et
// vérifié via effets de bord externes (segments shm, fichier de log écrit
// par le binaire compagnon tests/helpers/stub_child.cpp — voir ce fichier
// pour le contrat DRONE_TEST_LOG / DRONE_TEST_BEHAVIOR).
//
// Pourquoi un fork par test et pas des GlobalWatchdog construits
// directement dans le process gtest : `keepRunning_` est un
// `static inline std::atomic<bool>` — partagé entre TOUTES les instances
// de GlobalWatchdog du même process, jamais remis à true après un premier
// arrêt. Deux tests qui construiraient chacun leur propre GlobalWatchdog
// dans le même process gtest se marcheraient dessus : le second verrait
// keepRunning_ déjà à false et monitorLoop() ne s'exécuterait jamais. En
// production ça ne se voit pas (une seule instance pour toute la vie du
// process), mais c'est un vrai smell de conception (devrait être un membre
// d'instance, pas un static de classe) — révélé uniquement par le test.
// Le fork isole chaque test dans un process frais où ce static repart de
// zéro, contournant le problème sans y toucher.

#include "drone/core/GlobalWatchdog.hpp"
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#ifndef STUB_CHILD_PATH
#error "STUB_CHILD_PATH doit etre defini par CMake (voir CMakeLists.txt)"
#endif

namespace {

// Lance GlobalWatchdog(STUB_CHILD_PATH) dans un process forké dédié, avec
// son propre groupe de process (setpgid) — nécessaire pour le test qui
// simule un vrai Ctrl+C (signal envoyé au groupe entier, pas juste au
// watchdog).
pid_t spawnGlobalWatchdog(std::string_view behavior, const std::string &logPath) {
  pid_t pid = fork();
  if (pid == 0) {
    setpgid(0, 0);
    if (!behavior.empty())
      setenv("DRONE_TEST_BEHAVIOR", behavior.data(), 1);
    if (!logPath.empty())
      setenv("DRONE_TEST_LOG", logPath.c_str(), 1);

    GlobalWatchdog gw(STUB_CHILD_PATH);
    gw.run(); // bloque jusqu'à SIGINT/SIGTERM, revient après shutdown()
    _exit(0);
  }
  return pid;
}

// Attente bornée : un hang devient un échec de CE test, pas un blocage de
// toute la suite (pas de waitpid bloquant indéfiniment).
bool waitForExitWithTimeout(pid_t pid, std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    pid_t res = waitpid(pid, &status, WNOHANG);
    if (res == pid)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool shmExists(std::string_view path) {
  int fd = shm_open(path.data(), O_RDONLY, 0);
  if (fd >= 0) {
    close(fd);
    return true;
  }
  return false;
}

std::vector<std::string> readLogRoles(const std::string &logPath) {
  std::vector<std::string> roles;
  std::ifstream log(logPath);
  std::string line;
  while (std::getline(log, line)) {
    auto spacePos = line.find(' ');
    if (spacePos != std::string::npos)
      roles.push_back(line.substr(spacePos + 1));
  }
  return roles;
}

std::string uniqueLogPath(const char *suffix) {
  return "/tmp/drone_test_gwd_" + std::to_string(getpid()) + "_" + suffix +
        ".log";
}

} // namespace

TEST(GlobalWatchdogTest, SpawnsChildrenWithCorrectRolesAndCleansUpShm) {
  auto logPath = uniqueLogPath("roles");
  std::remove(logPath.c_str());

  pid_t gwPid = spawnGlobalWatchdog("stay_alive", logPath);

  // Laisse le temps à publishAll() + fork des deux enfants.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  EXPECT_TRUE(shmExists(kNavShmPath));
  EXPECT_TRUE(shmExists(kSFShmPath));
  EXPECT_TRUE(shmExists(kSysShmPath));

  auto roles = readLogRoles(logPath);
  EXPECT_NE(std::find(roles.begin(), roles.end(), "navigation"), roles.end());
  EXPECT_NE(std::find(roles.begin(), roles.end(), "sensorfusion"),
           roles.end());

  kill(gwPid, SIGTERM);
  ASSERT_TRUE(waitForExitWithTimeout(gwPid, std::chrono::milliseconds(2000)));

  EXPECT_FALSE(shmExists(kNavShmPath));
  EXPECT_FALSE(shmExists(kSFShmPath));
  EXPECT_FALSE(shmExists(kSysShmPath));

  std::remove(logPath.c_str());
}

TEST(GlobalWatchdogTest, RespawnsChildrenThatDie) {
  auto logPath = uniqueLogPath("respawn");
  std::remove(logPath.c_str());

  pid_t gwPid = spawnGlobalWatchdog("exit_after_short_delay", logPath);

  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  kill(gwPid, SIGTERM);
  waitForExitWithTimeout(gwPid, std::chrono::milliseconds(2000));

  auto roles = readLogRoles(logPath);
  std::remove(logPath.c_str());

  int navCount = static_cast<int>(
      std::count(roles.begin(), roles.end(), "navigation"));
  int sfCount = static_cast<int>(
      std::count(roles.begin(), roles.end(), "sensorfusion"));

  // Plus d'une occurrence de chaque rôle = preuve de respawn après mort.
  EXPECT_GT(navCount, 1);
  EXPECT_GT(sfCount, 1);
}

TEST(GlobalWatchdogTest, GroupWideSigtermDoesNotResurrectChildren) {
  // Régression du bug Ctrl+C : un vrai Ctrl+C envoie SIGTERM/SIGINT à tout
  // le groupe de process, pas seulement à GlobalWatchdog — donc les
  // enfants meurent en même temps que la décision d'arrêt est prise. Avant
  // le fix (monitorLoop.hpp), une mort d'enfant détectée après le SIGTERM
  // mais avant la relecture de keepRunning_ déclenchait un respawn
  // immédiatement suivi d'un re-kill par shutdown(). Ici : un seul spawn
  // par rôle doit apparaître dans le log, jamais un second après le
  // SIGTERM de groupe.
  auto logPath = uniqueLogPath("groupsig");
  std::remove(logPath.c_str());

  pid_t gwPid = spawnGlobalWatchdog("stay_alive", logPath);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  kill(-gwPid, SIGTERM); // -pid : tout le groupe (setpgid fait dans le fork)

  ASSERT_TRUE(waitForExitWithTimeout(gwPid, std::chrono::milliseconds(2000)));

  auto roles = readLogRoles(logPath);
  std::remove(logPath.c_str());

  int navCount = static_cast<int>(
      std::count(roles.begin(), roles.end(), "navigation"));
  int sfCount = static_cast<int>(
      std::count(roles.begin(), roles.end(), "sensorfusion"));

  EXPECT_EQ(navCount, 1);
  EXPECT_EQ(sfCount, 1);
}
