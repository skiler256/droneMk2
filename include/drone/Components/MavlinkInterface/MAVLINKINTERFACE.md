# MavlinkInterface — architecture, Tasks, piping

Documentation de référence du composant `MavlinkInterface` : rôle de
chaque Task, comment un message MAVLink circule, et les décisions d'archi
actées avant implémentation. Complète `GUIDE_COMPOSANTS.md` (générique) et
le SDD (`Document de Conception Logicielle.pdf`, §3.5 et §4). **Ce
document décrit une architecture décidée mais pas encore implémentée** —
contrairement à `SYSMONITORING.md`, aucun fichier `.hpp`/`.cpp` de ce
composant n'existe encore au moment de l'écriture.

## 1. Vue d'ensemble

```
FC (ArduPilot, UART) ◄──────────────► MavlinkInterface (process isolé)
                                        │
                                        ├─ TTx     : flux routine (GPS_INPUT,
                                        │            SET_POSITION_TARGET_LOCAL_NED,
                                        │            HEARTBEAT) — son propre scheduler
                                        ├─ TTxACK  : flux ponctuel ACK'd (PARAM_SET,
                                        │            CMD_DO_SET_MODE, init MAV_CMD_SET_
                                        │            MESSAGE_INTERVAL) — son propre
                                        │            scheduler + retry/timeout
                                        ├─ TRx     : parsing des messages entrants,
                                        │            écrit dans SharedFCStatus
                                        ├─ TCmd    : reçoit les décisions de
                                        │            MissionControl, les traduit en
                                        │            paquets pour TTx/TTxACK
                                        └─ FSTask  : surveille les conditions de
                                                     failsafe propres au lien FC,
                                                     peut envoyer CMD_DO_SET_MODE(LAND)
                                                     directement (bypass scheduler)

SensorFusion  ◄── lit SharedFCStatus (ATTITUDE, LOCAL_POSITION_NED)
MissionControl ◄── lit SharedFCStatus (BATTERY_STATUS, ESC_STATUS, santé EKF)
```

Comme `SysMonitoring`, `MavlinkInterface` ne connaît le lien physique
(UART vers le FC) qu'au travers d'une interface driver injectée — jamais
un type concret — pour permettre le SITL (driver simulé en face) sans
toucher au composant.

## 2. Les 5 Tasks

| Task | Rôle | Notes |
|---|---|---|
| `TTx` | Émet les messages TX routine dus (`GPS_INPUT`, `SET_POSITION_TARGET_LOCAL_NED` événementiels ; `HEARTBEAT` périodique pur) | Son propre `MavScheduler` |
| `TTxACK` | Émet les commandes ponctuelles (`PARAM_SET`, `CMD_DO_SET_MODE`, séquence d'init `MAV_CMD_SET_MESSAGE_INTERVAL`), gère timeout/retry | Son propre `MavScheduler` — indépendant de celui de `TTx`, pour qu'une commande en attente d'ACK ne bloque jamais le flux routine |
| `TRx` | Lit/parse les messages entrants, écrit dans `SharedFCStatus` | `ATTITUDE`, `LOCAL_POSITION_NED`, `BATTERY_STATUS`, `ESC_STATUS`, `SCALED_PRESSURE`, `HEARTBEAT`, `COMMAND_ACK`/`PARAM_VALUE` (consommés par le tracker retry de `TTxACK`) |
| `TCmd` | Reçoit les décisions de MissionControl (pattern générique déjà présent dans les autres composants), les traduit en messages à passer à `TTx`/`TTxACK` | Pas de MAVLink direct ici — pur routage MC → interne |
| `FSTask` | Surveille les conditions de failsafe propres à MavlinkInterface (perte double-lien avec timeout géré indépendamment de MC, MC mort qui ne revient pas, 3 crashs SysMonitoring/60s, E-stop reçu) | **`loop()` volontairement vide pour l'instant** — implémentée en dernier, une fois le reste validé |

## 3. Pourquoi deux schedulers séparés (`TTx` / `TTxACK`)

Un seul canal UART physique = un seul message en vol à la fois, donc
mutuelle exclusion nécessaire au niveau du driver — mais **pas** au niveau
de la logique de choix. Si `TTxACK` attend un ACK (jusqu'à 3×500ms = 1.5s
dans le pire cas) et bloque son propre scheduler tant qu'il ne l'a pas
reçu, ça n'empêche pas `TTx` de continuer à faire gagner ses propres
messages sur ses propres cycles — deux instances de scheduler, une classe
partagée, comme `mainSched_`/`secSched_` dans SysMonitoring.

## 4. Score de priorité — différent de SysMonitoring

Le SDD spécifie un score continu `temps écoulé × coefficient` (ex:
`HEARTBEAT` coef 1000, `GPS_INPUT` coef 80, `SET_POSITION_TARGET_LOCAL_NED`
coef 50 ; côté `TTxACK` : `CMD_DO_SET_MODE` coef 900, `PARAM_SET` coef 200,
init coef 150), pas les deux tiers discrets (Critical/Routine) +
most-overdue-first utilisés par `PacketScheduler` de SysMonitoring. Le
`MavScheduler` de `MavlinkInterface` réutilise le même **pattern**
(`nextDue()`/`markSent()`, deux instances filtrées) mais un algorithme de
score différent — pas la même classe.

## 5. GPS_INPUT / SET_POSITION_TARGET_LOCAL_NED : événementiel, jamais périodique

Contrairement à `HEARTBEAT` (périodique pur, doit toujours partir pour ne
jamais déclencher le failsafe GCS du FC), les messages qui transportent des
données issues de SensorFusion/Navigation sont envoyés **uniquement quand
une nouvelle donnée existe**, sans redondance périodique de secours (à
l'inverse du pattern `MISSION_STATE` de SysMonitoring). Raison de sécurité
explicite : si la source (SF/Nav) meurt silencieusement, on ne veut
**surtout pas** que le FC continue de recevoir une position/vitesse
périmée — l'absence de message doit se traduire par un timeout EKF côté
FC (failsafe autonome ArduPilot), pas être masquée par un renvoi
mécanique de la dernière valeur connue.

## 6. Retry / échec sur commande ACK'd (`TTxACK`)

`PARAM_SET`/`CMD_DO_SET_MODE` : timeout 500ms, 3 tentatives. Cascade sur
échec des 3 : déclenche un code (`CODES::`) → tentative de reconnexion
UART → si toujours en échec, passage en état dégradé → `FSTask` déclenche
la procédure de failsafe correspondante.

## 7. Perte de heartbeat FC — pas de "bonne" issue, retry quand même

Deux cas possibles à la perte du heartbeat FC, aucun n'a d'issue
vraiment positive côté logiciel :
- FC mort → crash → fin.
- Liaison physique coupée → FC lui-même ne reçoit plus rien → GCS
  failsafe ArduPilot (autonome, indépendant du Pi) → atterrissage → fin.

`MavlinkInterface` tente quand même une reconnexion UART (coûte rien,
couvre le cas d'un connecteur qui reprend contact), plutôt qu'un
kill/respawn systématique du process entier par `ComponentWatchdog`.

## 8. Commande d'urgence de `FSTask` — bypass volontaire du scheduler

Quand `FSTask` détecte une condition qui justifie un `LAND` immédiat
(double perte de lien avec timeout, MC mort qui ne revient pas, E-stop),
elle **n'utilise pas** le `MavScheduler` de `TTxACK` — elle appelle une
méthode dédiée sur l'interface driver (`sendUrgent()`), qui écrit
directement sur l'UART (protégée par le même mutex interne que les envois
normaux, donc thread-safe, mais sans passer par la logique de choix/score).
Le failsafe ne doit jamais dépendre de la justesse d'un algorithme
d'ordonnancement pour partir. `sendUrgent()` plutôt que réutiliser `send()`
: ça rend chaque contournement du flux normal grep-able/auditable dans le
code.

## 9. Dépendance `mavlink/c_library_v2`

Intégré en **git submodule** (pas vendored, pas FetchContent) — c'est ce
que fait ArduPilot lui-même et l'écosystème companion-computer autour.
Pointé sur un commit précis, `target_include_directories` vers le
sous-dossier du dialecte `ardupilotmega/` (hérite de `common.xml`). Headers
C purs déjà générés — rien à générer nous-mêmes.

## 10. Ce qui n'est PAS encore fait / tranché

- Aucun fichier source n'existe encore pour ce composant.
- `FSTask::loop()` restera vide jusqu'à ce que le reste (TX/RX/ACK) soit
  validé — décision explicite de séquencement, pas un oubli.
- Table des messages MAVLink → pas de générateur façon `TELEMETRY.csv`
  prévu : on s'appuie directement sur les structs générées par
  `c_library_v2`, pas de format maison à dupliquer.
- `TCmd` dépend de MissionControl (n'existe pas encore dans le roadmap) —
  comme `GCSRxTask` de SysMonitoring, `TCmd` n'aura personne en face pour
  l'instant.
- Le format exact de `SharedFCStatus` (quels champs, quelle Task écrit
  quoi) n'a pas encore été détaillé.
