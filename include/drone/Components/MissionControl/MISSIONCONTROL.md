# MissionControl — architecture, Tasks, state machine, failsafe

Documentation de référence du composant `MissionControl` : la state
machine (états/transitions décidés par l'utilisateur, pas le diagramme
d'origine du SDD §3.6.2 qui restait ambigu), le rôle de chaque Task, le
séquenceur de plan de mission, et la chaîne de failsafe. Complète
`GUIDE_COMPOSANTS.md` (générique) et le SDD (§3.6). `MissionControl` ne
fait jamais de cold-start (cf. `CLAUDE.md`) — c'est le composant le plus
critique du système.

**État** : state machine + séquenceur + failsafe + routage GCS
opérationnels et testés (33 cas GTest entre `MC_CONV::step()` et
`MISSIONPLAN_CONV`). Pilotage manuel (`MANUAL`) et pipeline d'upload du
plan de mission volontairement pas construits — cf. §9.

## 1. Vue d'ensemble

```
SharedComMem.gcsCmds ──► TGCSCmdRouter ──► StateMachineOrderQueue ──► TStateMachine
   (GCSRxTask, SysMonitoring)  (seul consommateur)   (intra-process,       │
                                                       std::mutex)          │
                                                                            ▼
sysState_ (codes majeurs) ──► TSafetyMonitor ──► criticalFailureActive_/obstacleActive_
                                                                            │
                                                                            ▼
                                                          MC_CONV::step() ──► MissionState
                                                                            │
                                                    ┌───────────────────────┼───────────────────────┐
                                                    ▼                       ▼                        ▼
                                       cmd_.pushMavlinkCommand      cmd_.setMissionState      cmd_.setPendingQuery
                                       (arm/disarm/takeoff/land)    (→ SysMonitoring          (→ MC_QUERY, réponse
                                       → MavlinkInterface           → MISSION_STATE)          via CMD_ANSWER)

comp.plan_ (chargé au PRE_FLIGHT) ──► TMiSequencer ──setArmedSegment──► SharedCmdMem (Nav)
```

Quatre Tasks, séparées par nature du travail (même logique que Nav/
MavlinkInterface) : `TStateMachine` (décision), `TMiSequencer`
(exécution du plan), `TSafetyMonitor` (surveillance), `TGCSCmdRouter`
(routage). Cf. §2.

## 2. Les 4 Tasks

| Task | Rôle | Fréquence | RT priority |
|---|---|---|---|
| `TStateMachine` | Construit les `Inputs`, appelle `MC_CONV::step()`, publie `MissionState`, commande le FC (arm/disarm/takeoff/land) sur les transitions qui l'exigent | 20Hz | 65 |
| `TMiSequencer` | Parcourt `plan_`, arme les segments Nav dans l'ordre, gère les segments HOLD, détecte la fin de mission | 10Hz | 65 |
| `TSafetyMonitor` | Scanne les codes majeurs des autres composants, publie `criticalFailureActive_`/`obstacleActive_` | 20Hz | 85 (la plus prioritaire — MC "le plus critique", cf. `CLAUDE.md`) |
| `TGCSCmdRouter` | Seul consommateur de `SharedComMem::gcsCmds` ; classe et relaie vers `TStateMachine` ou signale l'absence de destination | 10Hz | 65 |

`TSafetyMonitor` tourne à la priorité RT la plus haute des quatre — c'est
la Task dont un retard a le plus grand impact sécurité (surveillance,
pas calcul de contrôle, donc pas besoin d'être la plus *fréquente*, cf.
`TStateMachine`/`TFS` chez Nav pour la même logique).

## 3. State machine (`MC_CONV::step()`, `Conversions.hpp`)

Fonction pure `step(current, previous, pendingQuerySeq, Inputs) -> Outcome`,
extraite pour être testable sans passer par `TStateMachine::loop()`
(privée) — même principe que `NAV_CONV`/`MAVLINK_CONV`.

### États (`SharedCmdMem::MissionState`)

`Idle`, `PreFlight`, `Armed`, `Mission`, `Manual`, `Hold`, `Landing`,
`Emergency` — **pas de `Rtb`** (retiré, pas de RTB en V1, décidé
explicitement).

### Transitions

```
IDLE ──ARM(GCS)──► PREFLIGHT
PREFLIGHT ──preflightResult=true──► ARMED
PREFLIGHT ──preflightResult=false──► IDLE
ARMED ──armError──► IDLE
ARMED ──DISARM(GCS)──► IDLE
ARMED ──TAKEOFF(GCS)──► MISSION
MISSION ──criticalFailureActive──► EMERGENCY
MISSION ──LAND(GCS) ou missionComplete──► LANDING
MISSION ──obstacleActive ou HOLD(GCS)──► HOLD
MISSION ──MODE(GCS, mode=1)──► MANUAL
MANUAL ──criticalFailureActive──► EMERGENCY
MANUAL ──LAND(GCS)──► LANDING
MANUAL ──obstacleActive ou HOLD(GCS)──► HOLD
MANUAL ──MODE(GCS, mode=0)──► MISSION
HOLD ──criticalFailureActive──► EMERGENCY
HOLD ──LAND(GCS)──► LANDING
HOLD ──RESUME(GCS)──► previous (MISSION ou MANUAL)
LANDING ──landed──► IDLE
EMERGENCY ──réponse "resume" (CMD_ANSWER)──► previous
EMERGENCY ──réponse "land" (CMD_ANSWER) ou timeout 30s──► LANDING
```

`previous` : dernier `MissionState` stable (`Mission` ou `Manual`) avant
la bascule courante — maintenu par `TStateMachine`, mis à jour
**uniquement** en quittant `Mission`/`Manual` eux-mêmes (jamais en
quittant `Hold`/`Emergency`), pour qu'une reprise depuis l'un de ces
états retombe toujours sur le vrai mode de vol d'origine, pas sur un
état transitoire. C'est ce qui permet à `EMERGENCY` déclenché depuis
`HOLD` de reprendre directement en `MISSION`/`MANUAL`, pas en `HOLD`.

### `PreFlight` — pas un simple bool

`Inputs::preflightResult` est un `std::optional<bool>` (pas un `bool`) :
`nullopt` = vérifications en cours (reste en `PREFLIGHT`), `true`/`false`
= résultat définitif. Sans cette distinction, un échec de préflight
restait bloqué en `PREFLIGHT` indéfiniment (bug trouvé en écrivant la
vraie vérification, cf. §5).

### `HOLD` — pas de sortie automatique

Contrairement à une lecture naïve du SDD, la sortie de `HOLD` (obstacle
dégagé) n'est **pas** automatique — seule une commande `RESUME` explicite
du GCS en sort. Décision prise par prudence : un obstacle qui vient de
disparaître du champ LIDAR ne garantit pas que la voie est sûre.

### `EMERGENCY` — question au GCS, pas de reprise silencieuse

Cf. §6 pour le détail du mécanisme `PendingQuery`/`CMD_ANSWER`.
`Outcome::shouldQuery` est un signal de **niveau**, pas de front :
`TStateMachine` ne (re)publie la question que sur front montant/descendant
(sinon chaque cycle romprait la corrélation de `seq` avec `CMD_ANSWER`,
cf. `TFSTask::ekfStaleActive_` pour le même pattern ailleurs dans le code).

## 4. `TStateMachine` — construire les `Inputs`, commander le FC

### `armError`

Deux causes indépendantes, valides seulement pendant `ARMED` :
- **Timeout de confirmation** (`kArmTimeout=5000ms`) : le FC n'a jamais
  confirmé l'armement (`SharedFCStatus::armed`, extrait du bit
  `MAV_MODE_FLAG_SAFETY_ARMED` de `HEARTBEAT.base_mode` par
  `MavlinkInterface`) — poignée de main matérielle bornée, **pas** le
  genre de timeout écarté pour les segments de mission (cf. §5).
- **RPM bas** (`CODES::MVP001`) : détecté et publié par `MavlinkInterface`
  (`FSTask`, cf. `MAVLINKINTERFACE.md`), pas par MC — `TStateMachine` ne
  fait que lire le code. Décision explicite : le check vit là où la
  donnée ESC arrive, et sert aussi de voyant GCS indépendamment de MC.

### `landed` — vérifié, pas juste `armed==false`

Pas de nouvelle souscription MAVLink (`EXTENDED_SYS_STATE`) pour
l'instant, mais `armed==false` seul ne suffit pas : un désarmement en
plein vol (crash, coupure moteur) le déclencherait aussi, faisant
passer LANDING→IDLE sur un crash plutôt qu'un atterrissage. **Vérification
ajoutée** : `armed==false` **ET** vitesse (`SF_.getDynamicState()->velNed`)
sous `kLandedVelocityThreshold` (1.0 m/s, valeur de départ arbitraire).
Si désarmé mais encore en mouvement significatif pendant `LANDING` :
`landed` reste faux, `CODES::MCA008` (suspicion de crash) levé au lieu de
transitionner vers `IDLE` — édge-triggered (`crashSuspectedActive_`).

### Commande effective du FC

`MC_CONV::step()` décide l'état, mais ne parle jamais au FC lui-même —
c'est `TStateMachine::loop()` qui pousse les ordres `mavlinkCmds` sur les
transitions qui l'exigent :

| Transition | Ordre poussé |
|---|---|
| `* → ARMED` | `MCO003` (arm) |
| `ARMED → IDLE` | `MCO004` (disarm) |
| `ARMED → MISSION` | `MCO001` (takeoff, altitude = `CMD_TAKEOFF.altitude` du GCS) |
| `* → LANDING` | `MCO006` (land, `MAV_CMD_DO_SET_MODE`/`COPTER_MODE_LAND`) |

Aucun de ces 4 ordres n'existait avant — un bug trouvé en écrivant cette
doc (la state machine tournait "à vide", sans jamais réellement
commander le FC). `mavlinkReports` (résultat ACK de chaque ordre) reste
non consommé (cf. §9) : MC agit sur `SharedFCStatus::armed`/les codes,
pas sur l'ACK précis.

## 5. `TMiSequencer` — exécution du plan de mission

### Préflight réel

`runPreflightChecks()` (anonyme, `MissionControl.cpp`) lit
`/etc/drone/missionplan.bin` (**chemin fixe** — pas d'upload GCS→disque,
pas de hot-swap, décidé explicitement "on fait simple" en attendant ce
pipeline) et appelle `MISSIONPLAN_CONV::parse()` (magic/version/CRC/
segments déjà validés, cf. `MISSIONPLAN.md`/`test_missionplan.cpp`).
Exécuté **une seule fois par entrée en `PRE_FLIGHT`** (pas à 20Hz — c'est
de l'I/O disque), résultat mis en cache (`preflightChecked_`/
`preflightPassed_`) jusqu'à sortie de l'état. Échec → `CODES::MCA004`.

### Séquencement — même pattern armé/engagé que Nav

Pour chaque `WaypointSegment` : `entryWp` = cible du segment précédent
(`lastTargetWp_`), ou position réelle via `SF_.getDynamicState()` pour le
tout premier. Amorçage explicite (`pushNavCommand(MCO005, ...)`)
**seulement** pour le tout premier segment de la Mission — Nav promeut
ensuite les suivants automatiquement sur `SegmentStatus::Atteint` (déjà
acté côté Nav, cf. `NAVIGATION.md` §3-4).

`HOLD` (segment du plan, pause temporisée) ≠ `MissionState::Hold`
(système) : ne touche pas Nav, le drone reste sur la dernière cible
atteinte pendant `duration` secondes.

`CORRIDOR`/`CURVE` (pas de `SegmentType` équivalent côté Nav) et
`PAYLOAD_ACTION` (pas encore routé) : segment sauté, `CODES::MCA001`
levé — jamais ignoré en silence.

### Pas de timeout sur la durée du segment — décision explicite

**Point à retenir** : contrairement à un choix naïf ("le segment prend
trop longtemps → erreur"), tant que Nav reste sain, un segment long ne
déclenche rien. `TMiSequencer` juge Nav sur :
- sa santé publiée (`sysState_.getHealth(Navigation) == NOMINAL`) ;
- la fraîcheur de son dernier `NavCommand` (`kNavStaleThreshold=500ms` —
  `APTask` publie à 50Hz dès qu'il pilote quelque chose, donc largement
  au-dessus de ce seuil si Nav est bloqué ou mort).

Édge-triggered (`navStaleActive_`, même pattern que
`TFSTask::ekfStaleActive_`) : `CODES::MCA005` levé/effacé sur changement
d'état, pas à chaque cycle. `MCA005` est aussi scanné par
`TSafetyMonitor` (cf. §6) — un Nav mort en plein vol est traité comme un
échec critique système, pas juste un problème de séquencement local.

## 6. `TSafetyMonitor` — surveillance, pas décision

Sépare surveillance (`TSafetyMonitor`) et décision (`TStateMachine`),
même principe que `TFSTask` chez Nav/MavlinkInterface : `TSafetyMonitor`
ne fait *que* publier `comp.recoverableFailureActive_`/
`comp.unrecoverableFailureActive_`/`comp.obstacleActive_`, jamais de
bascule d'état lui-même.

### Deux scénarios distincts, pas un seul "failure" plat

**Point à retenir** : toutes les pannes ne méritent pas la même réaction.
`criticalFailureActive` (un seul bool) a été scindé en deux, sur retour
d'expérience terrain (l'ESC coupe les moteurs sous un certain seuil de
tension — gravissime, aucune attente possible dans ce cas) :

- **`recoverableFailureActive_`** → `EMERGENCY` (hold, question au GCS si
  la panne se résorbe, timeout 30s sinon, cf. §3/§8). Réservé aux pannes
  qui peuvent revenir toutes seules ou via un hot-start.
- **`unrecoverableFailureActive_`** → `LANDING` **immédiat**, sans
  attendre de réponse GCS ni de timeout — même si une `EMERGENCY` était
  déjà en cours pour une autre raison (cf. `MC_CONV::step()`, cas
  `Emergency`, vérifié en premier).

Ni l'un ni l'autre n'est un scan générique "n'importe quel code majeur
actif" — ça ferait basculer le système en `EMERGENCY`/`LANDING` sur un
simple obstacle (`NVA004`, Critical) ou un hold EKF redondant côté Nav
(`NVA005`, Critical), qui doivent rester des `HOLD` locaux gérés par Nav
lui-même (`holdActive_`), pas une urgence système. `SFS003` (Mag_lost)
absent délibérément : le magnétomètre est sur le FC depuis la bascule
d'archi, plus surveillé par SensorFusion — code orphelin (cf.
`SENSORFUSION.md` §7).

#### `recoverableFailureActive_`

| Code | Composant qui le lève | Signification |
|---|---|---|
| `SFS002` | SensorFusion | GPS perdu ou aberrant |
| `SFS005` | SensorFusion | EKF mort |
| `MVC004` | MavlinkInterface | Lien FC perdu (plus de HEARTBEAT) |
| `MCA005` | MissionControl | Nav périmé/mort pendant l'exécution d'un segment |
| `SMC002` **+** `SMC003` (simultanés) | SysMonitoring | Perte des deux liens GCS (wfb-ng ET 433MHz) — un seul lien perdu n'est qu'informatif, l'autre fait déjà office de secours |
| `ComponentHealth::SICK` | Navigation/SensorFusion/MavlinkInterface/SysMonitoring | A dû cold-start mais a fini par s'initialiser (cf. ci-dessous) |

#### `unrecoverableFailureActive_`

| Code | Composant qui le lève | Signification |
|---|---|---|
| `SMO001` | SysMonitoring | E-stop GCS (`CMD_EMERGENCY`) — l'opérateur a demandé l'arrêt, pas la peine de lui redemander "reprendre ou atterrir ?" |
| `MVP001` | MavlinkInterface | RPM moteur bas/ESC_STATUS absente pendant que le FC est armé |
| `FCP003` | MavlinkInterface | Tension batterie proche du seuil de coupure moteur ESC |
| `ComponentHealth::DEAD` | Navigation/SensorFusion/MavlinkInterface/SysMonitoring | `ComponentBase` a renoncé à démarrer les Task de ce composant (trop de cold-starts) — ne reviendra pas sans intervention |

`FCP001` (`batt_low`, seuil moins critique que `FCP003`) est levé pour le
tableau de voyants GCS mais **n'entre dans aucun des deux scans** — pas
encore assez grave pour changer l'état système, juste informatif.

### Escalade sur crash — s'appuie sur `ComponentHealth`, pas un compteur maison

**Point à retenir** : pas de nouvelle logique de comptage de crashs —
`ComponentBase` calcule déjà `ComponentHealth` (`NOMINAL`/`DEGRADED`/
`SICK`/`DEAD`) à partir de `ComponenConfig::max_hot_start`/
`max_cold_start` à chaque (re)démarrage d'un composant (cf.
`ComponentBase.hpp`). `TSafetyMonitor` ne fait que lire
`sysState_.getHealth(id)` pour Navigation/SensorFusion/MavlinkInterface/
SysMonitoring — `DEGRADED` (un seul hot-start) est ignoré, trop mineur.

**MissionControl lui-même n'est pas dans cette liste** : s'il est mort,
il n'est pas là pour se surveiller. C'est à `MavlinkInterface` de
commander `LAND` de façon autonome dans ce cas précis (cf. SDD §4,
"Crash MissionControl") — **pas encore construit**, cf. §9.

### `obstacleActive_`

Un seul code : `NVA004` (Navigation) — obstacle détecté par `TFS` côté
Nav dans la zone de sécurité devant le drone.

### `lidarLost_` — ni recoverable ni unrecoverable

**Point à retenir** : `SFS004` (Lidar_lost) ne rentre dans **aucun** des
deux scans ci-dessus — perdre le LIDAR retire la capacité d'évitement
d'obstacle, ce n'est pas une panne qui menace le vol lui-même. Traitement
dédié : `HOLD` (depuis Mission/Manual, même liste que `obstacleActive`/
`volumeBreached`) + publication de `QueryType::LidarLost` (`choice`:
0=reprendre sans évitement d'obstacle, 1=passer en `MANUAL`) tant que
`lidarLost` reste vrai. Répond au même mécanisme `PendingQuery`/
`CMD_ANSWER` que `EMERGENCY` (cf. §8), généralisé pour l'occasion :
`Outcome::query` porte maintenant un `QueryType` (pas juste un bool), et
`TStateMachine` republie/efface uniquement sur changement de type — pas
de couplage entre le mécanisme de question et un état particulier.

### `volumeBreached_` — volume de vol autorisé

Cahier des charges SDD §1.2 ("maintenir le drone à l'intérieur de la
zone d'opération définie par l'opérateur") — pas un code scanné mais un
calcul direct dans `TSafetyMonitor` : compare `SF_.getDynamicState()` à
`plan_.header.navVolumeRadius` (distance horizontale à l'origine) et
`navVolumeCeiling` (altitude). Désactivé (`> 0.0f` requis) tant qu'aucun
plan n'est chargé, pour ne pas déclencher sur un volume par défaut nul.

Traité **comme un obstacle** (`HOLD` immédiat, pas `EMERGENCY` ni
`LANDING`) — décision explicite : le drone reste pilotable en dehors de
la zone, pas de RTB en V1 pour l'y ramener automatiquement, donc pas de
raison de forcer un atterrissage immédiat comme pour une panne matérielle
irrécupérable. Édge-triggered (`volumeBreachActive_`), lève `CODES::MCA007`.

## 7. `TGCSCmdRouter` et `StateMachineOrderQueue`

**Bug trouvé et corrigé en session** : `TStateMachine` dépilait
initialement `SharedComMem::gcsCmds` directement. Un deuxième
consommateur (ex: un futur routage `CMD_WAYPOINT` vers Nav) aurait volé
des commandes qui ne lui étaient pas destinées — FIFO à un seul
consommateur possible. Corrigé : **seul** `TGCSCmdRouter` dépile
`gcsCmds`, il classe chaque commande et relaie vers
`StateMachineOrderQueue` (FIFO **intra-process**, `std::mutex` — pas de
la shm, les deux Tasks tournent dans le même process, cf. `CLAUDE.md`
"std::mutex local pour éviter les races intra-process").

| Type de commande | Destination |
|---|---|
| `Arm`/`Disarm`/`Takeoff`/`Mode`/`Hold`/`Resume`/`Land`/`Answer` | `StateMachineOrderQueue` → `TStateMachine` |
| `Waypoint` | Aucune (`CODES::MCA002`) — pilotage manuel pas construit, cf. §9 |
| `Rtl` | Aucune (`CODES::MCA003`) — pas de RTB en V1, décidé explicitement |

`CMD_EMERGENCY` ne passe **jamais** par ce FIFO : signalée directement
via `CODES::SMO001` (`SysMonitoring`, `GCSRxTask`), lue par
`TSafetyMonitor` indépendamment du rythme de dépilement — un ordre
d'urgence n'attend jamais son tour derrière une commande normale en file.

## 8. `PendingQuery`/`CMD_ANSWER` — MC interroge le GCS

Mécanisme générique (pas un paquet par question) pour les décisions qui
nécessitent une validation opérateur (SDD §4, "Reprise après EMERGENCY") :

- `SharedCmdMem::PendingQuery{seq, QueryType}` — publié par MC
  (`setPendingQuery`), lu par `SysMonitoring` pour le paquet descendant
  `MC_QUERY` (`seq`, `query_type`).
- `QueryType::EmergencyRecoverable` — seul type utilisé aujourd'hui
  (`choice`: 0=resume, 1=land). `QueryType::ObstacleNonContournable`
  existe dans l'enum (SDD §4) mais rien ne le publie encore — pas de
  logique de contournement d'obstacle non franchissable implémentée.
- Réponse : paquet montant `CMD_ANSWER(seq, choice)`, routé comme les
  autres commandes GCS (`GCSCommandType::Answer`) via `TGCSCmdRouter` →
  `StateMachineOrderQueue`. `MC_CONV::step()` vérifie que `seq` correspond
  au `pendingQuerySeq` courant avant d'agir — une réponse à une question
  déjà périmée est ignorée.

## 9. Ce qui n'est PAS encore fait / tranché

- **Pilotage manuel** (`MANUAL`) : décision prise (consigne vélNED
  envoyée en direct par le GCS, pas un enchaînement de waypoints) mais
  pas implémentée — reportée après les premiers tests d'intégration.
  `CMD_WAYPOINT` n'a donc aujourd'hui aucune destination.
- **Pipeline d'upload du plan de mission** : chemin fixe sur disque
  seulement, pas d'upload GCS→disque, pas de re-parsing à chaud/hot-swap
  (SDD §3.6.4) — délibérément simplifié en attendant.
- **`mavlinkReports` non consommé** : `MavlinkInterface` écrit le
  résultat (succès/échec) de `MCO001/003/004/006` dans ce FIFO, mais
  aucun composant ne le lit. MC réagit au bit `armed`/aux codes, pas à
  l'ACK précis — un rejet fin (ex: takeoff refusé pour raison X) reste
  invisible.
- **RTB retiré en V1** (décidé explicitement) : `CMD_RTL` reste un
  paquet valide au niveau fil (signalé `MCA003`), plus d'état `Rtb` dans
  `MissionState`.
- **`kMinArmedRpm`/`kBattLowVoltage`/`kBattCriticalVoltage`**
  (`MavlinkInterface.cpp`, 1000 tr/min / 14.0V / 12.5V) : valeurs de
  départ arbitraires, à calibrer sur banc avec les vrais moteurs/ESC/
  hélices/pack — `kBattCriticalVoltage` doit rester confortablement
  au-dessus du seuil de coupure moteur réel de l'ESC.
- **`QueryType::ObstacleNonContournable`** : existe dans l'enum, jamais
  publié — pas de logique de contournement d'obstacle implémentée.
- **Crash de `MissionControl` lui-même — RÉSOLU** : `TSafetyMonitor`
  scanne la santé des 4 *autres* composants, mais MC ne peut pas se
  surveiller lui-même s'il est mort. Corrigé via un heartbeat MC
  (`SharedCmdMem::lastHeartbeat`, publié par `TStateMachine` à chaque
  cycle) surveillé indépendamment par `MavlinkInterface::FSTask`, qui
  commande `LAND` de façon autonome (`CODES::MVC005`) si MC ne répond
  plus — cf. `MAVLINKINTERFACE.md`. Même mécanisme pour la perte
  simultanée des deux liens GCS, vue indépendamment de SysMonitoring/MC
  (`CODES::MVC006`).
- **`kLandedVelocityThreshold`** (`MissionControl.cpp`, 1.0 m/s) : valeur
  de départ arbitraire, à affiner en vol réel.
- **Pas encore testé en SITL/intégration** — contrairement à Nav
  (`nav_sitl`, dashboard dédié), MC n'a que les tests unitaires de
  `MC_CONV`/`MISSIONPLAN_CONV`. Prochaine étape logique une fois la base
  du GCS avancée.
