// tests/helpers/stub_child.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Binaire compagnon minimal pour tester GlobalWatchdog en boîte noire, sans
// dépendre du vrai `drone` (le dispatch de composants réel est trop lourd,
// et pas encore branché, pour servir de cible de test ici).
//
// Comportement piloté par variables d'environnement (héritées à travers
// execl par GlobalWatchdog::spawn(), qui ne touche pas à l'environnement) :
//   DRONE_TEST_LOG      : fichier où logger "<pid> <role>\n" au démarrage
//   DRONE_TEST_BEHAVIOR : "exit_after_short_delay" -> quitte après ~30ms
//                         (n'importe quelle autre valeur, ou absente) ->
//                         reste vivant jusqu'à un signal, comme un vrai
//                         composant qui tourne normalement.

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <thread>
#include <unistd.h>

int main(int argc, char **argv) {
  const char *role = argc > 1 ? argv[1] : "unknown";

  if (const char *logPath = std::getenv("DRONE_TEST_LOG")) {
    // write() unique sur un fd O_APPEND : atomique pour une petite ligne,
    // même si plusieurs enfants stub écrivent "en même temps".
    int fd = open(logPath, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd >= 0) {
      std::string line = std::to_string(getpid()) + " " + role + "\n";
      ssize_t written = write(fd, line.data(), line.size());
      (void)written;
      close(fd);
    }
  }

  const char *behavior = std::getenv("DRONE_TEST_BEHAVIOR");
  if (behavior && std::strcmp(behavior, "exit_after_short_delay") == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    return 0;
  }

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
