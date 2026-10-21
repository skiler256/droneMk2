#pragma once
#include <atomic>
#include "drone/Components/MavlinkInterface/SharedFCStatus.hpp"
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/Components/SystemMonitoring/Driver.hpp"
#include "drone/Components/SystemMonitoring/PacketScheduler.hpp"
#include "drone/Components/SystemMonitoring/SharedComMem.hpp"
#include "drone/core/ComponentBase.hpp"

class SysMonitoring;

// Composants pertinents pour le GCS, GlobalWatchdog exclu.
inline constexpr size_t kMonitoredComponentCount = 5;

// File locale au process, sans verrou : même thread des deux côtés —
// cf. SYSMONITORING.md §6.
struct PendingCodeEvent {
  uint32_t code{0};
  bool active{false}; // true = code levé, false = code effacé
  TYPES::TimePoint ts{};
};
inline constexpr size_t kPendingCodeCap = 16;

// Lien principal (WFB-NG) : porte tous les paquets descendants + vidéo.
class TelMainTask : public Task {
public:
  explicit TelMainTask(SysMonitoring &component, TaskConfig config,
                       ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
  // Un composant par envoi ; propre à ce lien (§6).
  size_t healthCycleIndex_{0};
  // Un couple (composant, Task) par envoi (§13). TASK_RATE est Links=MAIN,
  // donc ce curseur n'existe pas côté TelSec.
  size_t taskRateCycleIndex_{0};

  // Diff des codes majeurs ; snapshot propre à ce lien (§6).
  std::array<std::array<uint32_t, kMajorCodeSlots>, kMonitoredComponentCount> lastMajorCodes_{};
  bool codePrimed_{false};
  std::array<PendingCodeEvent, kPendingCodeCap> pendingCodes_{};
  size_t pendingCodeCount_{0};
};

// Un acquittement à émettre : de quoi remplir un CMD_ACK.
struct AckEvent {
  uint32_t seq{};
  uint8_t packetId{};
  uint8_t result{};
};

// Résultats portés par CMD_ACK.result.
inline constexpr uint8_t kAckAccepted = 0;  // reçue et appliquée
inline constexpr uint8_t kAckDuplicate = 1; // déjà appliquée, non rejouée
inline constexpr uint8_t kAckRejected = 2;  // reçue mais non applicable

// File d'acquittements entre la tâche de réception et une tâche d'émission.
//
// Un seul producteur (GCSRxTask) et un seul consommateur (la tâche du lien
// concerné) : un anneau avec deux indices atomiques suffit, sans verrou. Ce
// choix n'est pas de l'élégance mal placée — les tâches d'émission sont des
// threads SCHED_FIFO, et un mutex pris par la tâche de réception les
// bloquerait le temps d'une transaction SPI, soit plusieurs millisecondes.
//
// L'invariant « un PacketScheduler appartient à une seule tâche » est
// préservé : la tâche d'émission constate elle-même que sa file n'est pas
// vide et se marque dirty. La tâche de réception ne touche aucun scheduler.
class AckQueue {
public:
  static constexpr size_t kCapacity = 8;

  // Appelé par la tâche de réception uniquement. Renvoie false si la file est
  // pleine : on préfère perdre un acquittement — le sol retransmettra — que
  // d'écraser un acquittement plus ancien, donc plus urgent.
  [[nodiscard]] bool push(const AckEvent &ev) {
    const size_t t = tail_.load(std::memory_order_relaxed);
    const size_t suivant = (t + 1) % kCapacity;
    if (suivant == head_.load(std::memory_order_acquire))
      return false;
    buf_[t] = ev;
    tail_.store(suivant, std::memory_order_release);
    return true;
  }

  // Appelés par la tâche d'émission uniquement.
  [[nodiscard]] bool empty() const {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
  }

  [[nodiscard]] bool pop(AckEvent &out) {
    const size_t h = head_.load(std::memory_order_relaxed);
    if (h == tail_.load(std::memory_order_acquire))
      return false;
    out = buf_[h];
    head_.store((h + 1) % kCapacity, std::memory_order_release);
    return true;
  }

private:
  std::array<AckEvent, kCapacity> buf_{};
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
};

// Dédoublonnage des commandes acquittées, propre à la tâche de réception donc
// sans partage ni synchronisation.
//
// Un seul seq retenu par type de commande : le sol attend l'acquittement
// avant d'envoyer la commande suivante du même type, il n'y en a donc qu'une
// en vol à la fois. Le dédoublonnage est GLOBAL et non par lien : PDL et SDL
// étant indépendants, la même commande peut arriver par les deux, et armer
// deux fois n'est pas armer une fois.
class UplinkDedup {
public:
  // Renvoie true si cette trame est un doublon déjà appliqué.
  [[nodiscard]] bool isDuplicate(TELEM::PacketID id, uint32_t seq) const {
    const size_t i = static_cast<size_t>(id);
    if (i >= kSlots)
      return false;
    return vu_[i] && dernier_[i] == seq;
  }

  void remember(TELEM::PacketID id, uint32_t seq) {
    const size_t i = static_cast<size_t>(id);
    if (i >= kSlots)
      return;
    dernier_[i] = seq;
    vu_[i] = true;
  }

private:
  static constexpr size_t kSlots = TELEM::kTable.size();
  std::array<uint32_t, kSlots> dernier_{};
  std::array<bool, kSlots> vu_{};
};

// Lien de secours (CC1101) : sous-ensemble léger (Links=BOTH uniquement).
class TelSecTask : public Task {
public:
  explicit TelSecTask(SysMonitoring &component, TaskConfig config,
                      ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
  size_t healthCycleIndex_{0};
  std::array<std::array<uint32_t, kMajorCodeSlots>, kMonitoredComponentCount> lastMajorCodes_{};
  bool codePrimed_{false};
  std::array<PendingCodeEvent, kPendingCodeCap> pendingCodes_{};
  size_t pendingCodeCount_{0};
};

// Réception des commandes montantes (CMD_*/HEARTBEAT_GCS), sur les deux liens.
class GCSRxTask : public Task {
public:
  explicit GCSRxTask(SysMonitoring &component, TaskConfig config,
                     ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
  bool mainLostActive_{false};
  bool secLostActive_{false};
  UplinkDedup dedup_;
};

// Relai vidéo (WFB-NG uniquement), tolérant à la perte.
class VideoTask : public Task {
public:
  explicit VideoTask(SysMonitoring &component, TaskConfig config,
                     ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  SysMonitoring &comp;
  void loop() override;
};

class SysMonitoring : public ComponentBase<4> {
public:
  // Drivers injectés par interface, cf. SYSMONITORING.md §7.
  explicit SysMonitoring(ComponenConfig config, SharedSysStateMemHandler &sysState,
                         SharedComMemHandler &com, SharedNavMemHandler &nav,
                         SharedSFMemHandler &sf, SharedFCStatusHandler &fc,
                         SharedCmdMemHandler &cmd, ITelemetryLink &mainLink,
                         ITelemetryLink &secLink, IVideoSource &videoSource,
                         ISystemTempSource &tempSource);

  SharedComMemHandler &com_;
  SharedSysStateMemHandler &sysState_; // pour raiseCode/clearCode depuis les Task
  SharedNavMemHandler &nav_;           // source POS/VELNED/ATTITUDE/AUTOPILOT_MODE (FMA)
  SharedSFMemHandler &sf_;             // source GPSFIX/GPSPOS/SAT_INFO/LIDAR_SCAN
  SharedFCStatusHandler &fc_;          // source BATTERY
  SharedCmdMemHandler &cmd_;           // source MISSION_STATE, AUTOPILOT_MODE (cible)

  ITelemetryLink &mainLink_;
  ITelemetryLink &secLink_;
  IVideoSource &videoSource_;
  ISystemTempSource &tempSource_;

  PacketScheduler mainSched_{TELEM::Link::Main};
  PacketScheduler secSched_{TELEM::Link::Sec};

  // Une file par lien : l'acquittement repart sur le lien qui a porté la
  // requête, PDL et SDL étant indépendants.
  AckQueue ackMain_;
  AckQueue ackSec_;

private:
  TelMainTask TTelMain;
  TelSecTask TTelSec;
  GCSRxTask TGCSRx;
  VideoTask TVideo;
};
