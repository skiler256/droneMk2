#pragma once
#include "drone/Components/MissionControl/SharedCmdMem.hpp"
#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/Components/SensorFusions/SharedSFMem.hpp"
#include "drone/core/ComponentBase.hpp"
#include "drone/core/watchdog.hpp"

class Navigation;

// Séquencement (façon FMA Airbus) : dépile navCmds, promeut armedSegment
// en engagedSegment (sur amorçage ou sur Atteint, cf. canEngage), tient
// le FMA latéral. Séparé de TAP (calcul pur) car événementiel — cf. NAVIGATION.md §2.
class TSeqTask : public Task {
public:
  explicit TSeqTask(Navigation &component, TaskConfig config,
                    ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  Navigation &comp;
  void loop() override;

  // Tente de promouvoir armedSegment en engagedSegment — false si rien
  // d'armé, déjà engagé, ou sécurité non respectée (cf. canEngage).
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

// Sécurité redondante, indépendante de TAP : santé EKF, présence
// d'obstacle — déclenche un hold via holdActive_ (jamais d'écriture
// directe sur NavCommand, TAP reste le seul écrivain).
class TFSTask : public Task {
public:
  explicit TFSTask(Navigation &component, TaskConfig config,
                   ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  Navigation &comp;
  void loop() override;

  // Édge-triggered, un flag par cause (même pattern que gpsFaultActive_
  // dans SensorFusion) — holdActive_ est l'OR des deux, pas lui-même edge-triggered.
  bool ekfStaleActive_{false};
  bool obstacleActive_{false};
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

  // Rafraîchis par TSeq, lus par TAP.
  NavSegment engagedSegment_{};
  NavSegment armedSegment_{};
  uint32_t lastSegmentSeq_{0};
  bool newSegment_{
      false}; // consommé (remis à false) par TAP au reset de ses états internes

  // Posé par TFS, lu par TAP — un seul écrivain de NavCommand (TAP), cf. NAVIGATION.md §1.
  bool holdActive_{false};

private:
  TSeqTask TSeq;
  APTask TAP;
  TFSTask TFS;
};
