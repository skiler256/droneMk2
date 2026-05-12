// main.cpp
//
// Point d'entrée du programme drone.
//
// Responsabilités :
//   1. Vérifier les prérequis système (RT, permissions)
//   2. Verrouiller la mémoire RAM (pas de page fault en vol)
//   3. Créer les états partagés (AppState)
//   4. Créer et démarrer les 4 blocs + watchdog
//   5. Attendre SIGTERM/SIGINT
//   6. Arrêt ordonné : nav → HOLD, puis arrêt des threads
// ─────────────────────────────────────────────────────────────────────────────

#include "drone/shared_state.hpp"

#include "monitoring/monitoring_block.hpp"
#include "drivers/sensor_fusion_block.hpp"
#include "navigation/navigation_block.hpp"
#include "mavlink/mavlink_block.hpp"
#include "core/watchdog.hpp"

#include <sys/mman.h>      // mlockall
#include <sys/resource.h>  // setrlimit
#include <csignal>
#include <atomic>
#include <iostream>
#include <thread>

// ─── Signal handling ─────────────────────────────────────────────────────────
// std::atomic garantit l'accès thread-safe depuis le handler de signal

namespace {
    std::atomic_bool g_shutdown{false};

    void signalHandler(int sig) {
        std::cout << "\n[main] Signal " << sig << " reçu — arrêt en cours...\n";
        g_shutdown = true;
    }
}

// ─── Prérequis système ───────────────────────────────────────────────────────

static bool checkRTPermissions() {
    // Vérifier qu'on peut utiliser SCHED_FIFO
    // Nécessite : sudo setcap cap_sys_nice+ep ./drone  ou  sudo ./drone
    struct sched_param p{ .sched_priority = 1 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &p) != 0) {
        std::cerr << "[main] AVERTISSEMENT : impossible d'activer SCHED_FIFO\n"
                  << "       Lancer avec : sudo setcap cap_sys_nice+ep ./drone\n"
                  << "       Les performances RT ne sont pas garanties.\n";
        return false;
    }
    // Remettre en SCHED_OTHER pour le main (qui n'est pas critique)
    p.sched_priority = 0;
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &p);
    return true;
}

static void lockMemory() {
    // Verrouille toute la RAM du process en mémoire physique.
    // Sans ça, le kernel peut swapper des pages → latence imprévisible en vol.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[main] AVERTISSEMENT : mlockall échoué — "
                  << "page faults possibles en vol\n";
    }
}

// ─── main ────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "[main] Drone Pi — démarrage\n";

    // ── 1. Prérequis ──────────────────────────────────────────────────────
    checkRTPermissions();
    lockMemory();

    // ── 2. Handlers de signaux ────────────────────────────────────────────
    std::signal(SIGTERM, signalHandler);   // kill / systemd stop
    std::signal(SIGINT,  signalHandler);   // Ctrl+C

    // ── 3. États partagés ─────────────────────────────────────────────────
    // Unique instance, durée de vie = celle du programme.
    // Chaque bloc reçoit uniquement l'interface dont il a besoin.
    drone::AppState app;

    // ── 4. Watchdog ───────────────────────────────────────────────────────
    drone::Watchdog watchdog;

    // ── 5. Création des blocs ─────────────────────────────────────────────
    //
    // Ordre de création = ordre de dépendance :
    //   ① SensorFusion   écrit app.state
    //   ② Navigation     lit app.state,  écrit app.nav
    //   ③ MavlinkBlock   lit app.nav,    écrit app.fc
    //   ④ Monitoring     lit tout,       écrit dans les logs
    //
    drone::SensorFusionBlock sensor_fusion{app.state, watchdog};
    drone::NavigationBlock   navigation   {app.state, app.nav, app.fc, watchdog};
    drone::MavlinkBlock      mavlink      {app.nav,   app.fc,  watchdog};
    drone::MonitoringBlock   monitoring   {app.state, app.nav, app.fc, watchdog};

    // ── 6. Démarrage dans l'ordre ─────────────────────────────────────────
    //
    // On démarre le monitoring en premier pour capturer les événements
    // des autres blocs dès leur démarrage.
    std::cout << "[main] Démarrage des blocs...\n";
    monitoring.start();
    watchdog.start();
    sensor_fusion.start();
    mavlink.start();          // démarre avant nav : le heartbeat FC doit
                              // être établi avant d'envoyer des consignes
    navigation.start();

    std::cout << "[main] Tous les blocs démarrés — en attente de signal\n";

    // ── 7. Boucle principale : watchdog Linux /dev/watchdog ───────────────
    //
    // Si cette boucle se bloque, le kernel reboot le Pi.
    // ArduPilot détecte la perte de heartbeat → atterrissage.
    int wd_fd = ::open("/dev/watchdog", O_WRONLY);
    if (wd_fd < 0) {
        std::cerr << "[main] /dev/watchdog non disponible (normal en dev)\n";
    }

    while (!g_shutdown) {
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Rafraîchir le watchdog kernel
        if (wd_fd >= 0) {
            ::write(wd_fd, "1", 1);
        }
    }

    // ── 8. Arrêt ordonné ──────────────────────────────────────────────────
    //
    // Ordre inverse du démarrage.
    // La nav passe en HOLD avant de s'arrêter → le FC reçoit une dernière
    // consigne nulle avant que le heartbeat disparaisse.
    std::cout << "[main] Arrêt ordonné...\n";

    navigation.stop();        // envoie HOLD puis s'arrête
    mavlink.stop();           // continue le heartbeat jusqu'à l'arrêt nav
    sensor_fusion.stop();
    watchdog.stop();
    monitoring.stop();        // en dernier : capture les logs d'arrêt

    // Désarmer le watchdog kernel proprement
    if (wd_fd >= 0) {
        ::write(wd_fd, "V", 1);   // magic close : désarme le watchdog
        ::close(wd_fd);
    }

    std::cout << "[main] Arrêt complet.\n";
    return 0;
}