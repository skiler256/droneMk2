#pragma once
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/watchdog.hpp"
#include <atomic>
#include <mutex>

class Navigation;

// Séquencement événementiel, séparé de TAP (calcul pur) — NAVIGATION.md §2.
class TSeqTask : public Task {
public:
  explicit TSeqTask(Navigation &component, TaskConfig config,
                    ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  Navigation &comp;
  void loop() override;

  // false si rien d'armé, déjà engagé, ou sécurité non respectée.
  bool tryPromote();
};

// Autopilote : lit holdActive_ (posé par TFS) et calcule la consigne via
// la loi de correction du segment courant (cf. NAV_CONV).
class APTask : public Task {
public:
  explicit APTask(Navigation &component, TaskConfig config,
                  ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  Navigation &comp;
  void loop() override;
};

// Sécurité redondante : pose holdActive_, n'écrit jamais NavCommand (TAP
// en reste seul écrivain) — cf. NAVIGATION.md §6.
class TFSTask : public Task {
public:
  explicit TFSTask(Navigation &component, TaskConfig config,
                   ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  Navigation &comp;
  void loop() override;

  // Edge-triggered par cause ; holdActive_ en est l'OR, pas edge-triggered.
  bool ekfStaleActive_{false};
  bool obstacleActive_{false};
  bool lidarStaleActive_{false};
};

class Navigation : public ComponentBase<3> {
public:
  explicit Navigation(ComponenConfig config, SharedSysStateMemHandler &sysState,
                      SharedNavMemHandler &nav, SharedSFMemHandler &SF,
                      SharedCmdMemHandler &cmd);

  SharedNavMemHandler &nav_;
  SharedSFMemHandler &SF_;
  SharedSysStateMemHandler
      &sysState_;            // pour raiseCode/clearCode depuis les Task
  SharedCmdMemHandler &cmd_; // segment courant + ordres ponctuels de MC

  // Partagés entre les 3 Tasks — cf. ARCHITECTURE_IPC.md §3. Les structs
  // multi-champs passent par le mutex, les scalaires par un atomique.
  std::mutex segmentsMutex_;
  NavSegment engagedSegment_{};
  NavSegment armedSegment_{};

  std::atomic<uint32_t> lastSegmentSeq_{0};
  std::atomic<bool> newSegment_{false}; // consommé par TAP via exchange()
  std::atomic<bool> holdActive_{false}; // posé par TFS, lu par TAP

private:
  TSeqTask TSeq;
  APTask TAP;
  TFSTask TFS;
};
