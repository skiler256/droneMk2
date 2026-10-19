# Audit — travaux restants

> Document de travail. Les **anomalies** relevées en revue de code sont toutes
> traitées (table « Déjà réglé » en fin de document). Ce qui suit est le reste
> à faire côté logiciel, hors fonctionnalités non écrites.
>
> Dernière mise à jour : 2026-10-18. Toutes les références de fichier et de
> ligne ci-dessous ont été vérifiées contre le code à cette date.

## Récapitulatif

| # | À faire | Nature | Où regarder |
|---|---|---|---|
| **R1** | La règle « `MissionControl` ne fait jamais de cold-start » n'existe que dans la doc | Règle non codée | `core/ComponentBase.hpp:150-175` (décision uniforme pour les 5) · règle énoncée `CLAUDE.md:80` |
| **R2** | L'absence d'un composant n'est pas observable dans le système | Code manquant | `core/GlobalWatchdog.hpp` `spawnWithRetry()` · `data/INDEX.csv` (aucune ligne `GW`) |
| **R3** | `QueryType::ObstacleNonContournable` jamais levé | Branche morte | `MissionControl/SharedCmdMem.hpp:28` · producteur attendu dans `MC_CONV::step()` |
| **R4** | Pas de `CODES::` pour un obstacle vu par `TLidar` | Code manquant | `src/SensorsFusion/SensorFusion.cpp` `LidarTask::loop()` |
| **R5** | Pas de `CODES::` pour un ordre `TCmd` inconnu | Code manquant | `src/MavlinkInterface/MavlinkInterface.cpp` `TCmdTask::loop()` |
| **R6** | **`mavlinkReports` écrit mais lu par personne** | Câblage inachevé | écrit `MavlinkInterface.cpp:89,344` · `popMavlinkReport()` défini `SharedCmdMem.hpp:125`, **zéro appelant** |
| **R7** | `ARM_STATE` reste aux valeurs par défaut | Câblage inachevé | `SystemMonitoring.cpp:227` (encodage) · note `:124` · source attendue `SharedFCStatus` |
| **R8** | FIFO `navCmds`/`navReports` déclarées, inutilisées | Câblage inachevé | `SharedCmdMem.hpp:75-76` · `TCmd` côté SF inexistant |
| **R9** | `HomeCalibrator` toujours `{0,0,0}` | Câblage inachevé | `SensorsFusion.hpp:71` · appliqué `SensorFusion.cpp:160` |
| **R10** | `FSTask` lève `MVC004` sur perte FC mais ne déclenche rien ; rien ne vérifie que le FC a pris la dernière commande | Câblage inachevé | `MavlinkInterface.cpp:211-214` |
| **R11** | `verticalArmed` jamais écrit | Câblage inachevé | setter `SharedNavMem.hpp:50` · **zéro appelant** dans `src/` |
| R12 *(hors périmètre : matériel)* | Seuils RPM arbitraires (1000/10000 tr/min) | Calibration | `MavlinkInterface.cpp:33-34` · utilisés `:229` |
| ~~R13~~ | **Requalifié en N7** : ce n'est pas de la calibration mais une incohérence code/matériel | — | voir N7 |
| R14 *(hors périmètre : matériel)* | Paramètres FC à re-régler après la bascule du magnéto | Réglage FC | `SENSORFUSION.md` §3.5 (`EK3_SRC1_YAW`, `COMPASS_ENABLE`) |
| R15 *(hors périmètre : matériel)* | Port UART du FC = placeholder non vérifié | Réglage | `src/main.cpp:41` (`kFcUartPort`), utilisé `:195` |
| **R16** | Deux arêtes `MISSION`↔`MANUAL` ambiguës dans le SDD | **Décision** | `MissionControl/Conversions.hpp:52` (`MC_CONV::step`) |
| **R17** | 3 canaux TX MAVLink pour une seule identité `(sysid, compid)` | **Décision** | `MAVLINKINTERFACE.md` §3 |
| **R18** | `SegmentType` n'a que `Waypoint` ; pas de PID latéral ni de loi de vitesse constante | Décision + code | `SharedCmdMem.hpp:9` · `NAVIGATION.md` §10 |
| **R19** | Reprise après panne (MC → Nav, après un hold) différée | Décision | `NAVIGATION.md` §10 · `MISSIONCONTROL.md` §3 |
| **N1** | **État partagé entre Tasks d'un même composant, sans aucune synchronisation** | **Élevée** | `Navigation.hpp:67-75` · `MissionControl.hpp:136-157` · zéro `std::atomic` dans les deux |
| **N2** | `TFS` fait de l'évitement d'obstacle sur une `LidarMap` dont il ne vérifie jamais l'âge | Moyenne | `Navigation.cpp:181-184` · `ts` écrit `SensorFusion.cpp:200`, jamais lu |
| **N3** | `SFS001` inonde le journal des codes mineurs | Moyenne | `SensorFusion.cpp:123` · ring de 16 slots, `SharedSysStateMem.hpp` |
| **N4** | Les compteurs de staleness démarrent à l'epoch → alarme Emergency à chaque démarrage | Moyenne | `SensorsFusion.hpp:26,55` · levée `SensorFusion.cpp:70,180` |
| **N5** | §17 corrigé seulement dans MissionControl : 5 sites jettent encore le refus | Faible | `Navigation.cpp:84,88` · `MavlinkInterface.cpp:89,344` |
| **N6** | Condition morte : `!gpsHasNoNewData` toujours vrai à ce point | Cosmétique | `SensorFusion.cpp:128` (retour l.76) |
| **N7** | **Seuils batterie incompatibles avec le pack 6S : protection inopérante** | **Critique** | `MavlinkInterface.cpp:59-60` · comparés `:255-256` · pack documenté `HARDWARE.md:170` |
| **N8** | `value_or()` optimiste sur deux checks de sécurité de `FSTask` | Moyenne | `MavlinkInterface.cpp:222` (`getArmed`) · `:254` (`getBattery`) |
| **N9** | `std::cout` bloquant sur le chemin RT de réception FC | Moyenne | `MavlinkInterface.cpp:649,667,675` (TRx, prio 75, 100Hz) |
| Q1 | ~~§17 sans test~~ — **couvert** par deux tests d'intégration | — | `tests/test_audit.cpp` `Audit17_*` — exigent `setcap`, sinon **skippés** |
| **Q2** | `ENABLE_TSAN` non ajouté | Qualité | `CMakeLists.txt:44` (bloc sanitizers) |
| **Q3** | Corps de composants relus par motifs seulement | Qualité | `SensorFusion.cpp` (211 l.) et `Navigation.cpp` (196 l.) **jamais vraiment relus** |
| **Q4** | Pas validé en vol complet | Qualité | HIL bring-up — `hil_sf_mv`, cf. `sitl/SITL.md` |

---

## Groupe 1 — règles énoncées, pas codées

Ce ne sont pas des manques de fonctionnalité : ce sont des décisions déjà
prises dont on peut croire à tort qu'elles sont actives.

**R1 — cold-start de `MissionControl`.** `CLAUDE.md:80` en fait une règle dure
(« sa state machine ne peut pas être resetée en sécurité »). Or
`ComponentBase.hpp:150-175` applique la même décision hot/cold/DEAD aux cinq
composants, sans traitement particulier par `ComponentID`. Un `MissionControl`
qui accumule assez de hot-starts se verra donc cold-starté comme les autres.
C'est le point le plus trompeur de la liste.

**R2 — absence d'un composant invisible.** `spawnWithRetry()` réessaie trois
fois puis trace sur `stderr`. Rien de plus : `setHealth()` étant self-report
only (`ARCHITECTURE_IPC.md` §2.3), GW ne peut pas marquer `DEAD` le slot d'un
autre composant. Il faudrait un code `GW` dans `data/INDEX.csv`, qui n'en
contient aucun aujourd'hui — décision à prendre sur le libellé et la sévérité.

**R3/R4/R5 — branches et codes manquants.** `ObstacleNonContournable` existe
dans l'enum `QueryType` et aucun producteur ne le lève. Même schéma pour un
obstacle détecté par `TLidar` et pour un ordre `TCmd` inconnu : le chemin de
détection existe, le code de diagnostic non.

## Groupe 2 — câblage inachevé entre composants

**R6 est le plus important.** `MavlinkInterface` pousse consciencieusement un
`CommandReport` à chaque commande résolue, corrélé par `seq`
(`MavlinkInterface.cpp:89` et `:344`). `popMavlinkReport()` est défini
(`SharedCmdMem.hpp:125`) et **personne ne l'appelle**. MC ne sait donc jamais
si ses ordres ont abouti : toute la boucle de retour est construite à moitié,
ce qui est pire qu'absente puisque le code donne l'impression du contraire.

Les autres suivent le même motif — un côté écrit, l'autre n'existe pas
(`ARM_STATE`), ou les deux côtés manquent (`navCmds`/`navReports`,
`verticalArmed` dont le setter n'a aucun appelant), ou la valeur est neutre en
permanence (`homeOffsetNed`).

**R10** mérite une mention : `FSTask` détecte la perte de HEARTBEAT FC et lève
`MVC004`, mais ne déclenche aucune action — et symétriquement, rien ne vérifie
que le FC a bien pris en compte la dernière commande envoyée.

## Groupe 3 — calibration et réglages, à faire avec le matériel

Ces points ne peuvent pas avancer sans le drone sous la main.

**R12** — `kMinArmedRpm=1000`/`kMaxArmedRpm=10000` sont des valeurs de départ
explicitement arbitraires. Un iFlight XING-E Pro 2207 1800KV en 6S peut
dépasser 10000 tr/min selon l'hélice : le seuil haut risque de se déclencher en
vol nominal. `bench_lidar`/`bench_gps` existent désormais pour ce genre de
relevé ; un équivalent moteur reste à écrire si tu veux instrumenter le banc.

**R14** — conséquence directe de la bascule du magnétomètre vers le FC :
`EK3_SRC1_YAW` doit pointer le compas (et non `2`=GPS, valeur utilisée pour les
essais SITL), `COMPASS_ENABLE` réactivé.

**R15** — `kFcUartPort = "/dev/ttyAMA0"` est un placeholder choisi par
cohérence avec le GPS (`ttyAMA1`) et le LD06 (`ttyAMA2`), jamais vérifié sur le
matériel réel. À confirmer au premier bring-up.

## Groupe 4 — décisions d'archi encore ouvertes

**R16** bloque l'écriture de la logique de `TStateMachine` : deux arêtes
`MISSION`↔`MANUAL` restent ambiguës dans le SDD. **R17** est plus technique :
trois canaux TX (`COMM_0/1/2`) partagent une identité `(sysid, compid)` unique.
**R18** et **R19** touchent Navigation — types de segment au-delà de
`Waypoint`, loi de guidage latéral, et reprise après hold.

## Qualité — ce que je sais ne pas être couvert

**Q1** — le blocage sélectif sur refus de `push` (`MCA010`) est implémenté et
**non testé** : la logique vit dans `TStateMachineTask::loop()`, privée et
pilotée par un thread RT à 20Hz. Un test d'intégration est désormais faisable,
`setcap cap_sys_nice+ep` fonctionnant dans l'environnement de dev.

**Q3** — honnêteté sur le périmètre de la revue : les **corps** de
`SensorFusion.cpp` et `Navigation.cpp` n'ont jamais été relus ligne à ligne,
seulement balayés par motifs (`.value()` non gardés, divisions, indexations,
`[[maybe_unused]]`). Ce sont les deux fichiers où une relecture a le meilleur
rendement.

---

## Relecture complète du code de production

Faite le 2026-10-04 sur les corps jamais relus (`SensorFusion.cpp`,
`Navigation.cpp`) puis sur les membres partagés des cinq composants. Les
emplacements ci-dessous sont vérifiés.

### N1 — état partagé entre Tasks, sans synchronisation — **Élevée**

Chaque composant fait tourner plusieurs `Task` en threads `SCHED_FIFO`
distincts, et plusieurs membres sont écrits par l'un, lus par l'autre. Les
commentaires le documentent explicitement (« Rafraîchis par TSeq, lus par
TAP », « Écrits par TSafetyMonitor, lus par TStateMachine ») — seule la
synchronisation manque. **Zéro `std::atomic` dans `Navigation.hpp` comme dans
`MissionControl.hpp`.**

| Membre | Écrit par | Lu par | Risque |
|---|---|---|---|
| `engagedSegment_`, `armedSegment_` (`NavSegment`, multi-champs) | TSeq (20Hz) | TAP (50Hz) | **lecture déchirée** : nouveau `seq` avec ancienne cible |
| `newSegment_` | TSeq **et** TAP | les deux | écriture concurrente sur le même drapeau |
| `holdActive_` | TFS (20Hz) | TAP (50Hz) | course formelle (1 octet, bénin en pratique) |
| `plan_` (contient un `std::vector`) | TStateMachine au PRE_FLIGHT | TMiSequencer (10Hz, `plan_.segments[i]`) | **réallocation sous les pieds du lecteur** |
| `state_` (`MissionState`) | TStateMachine | TMiSequencer, TGCSCmdRouter | lecture d'un état périmé |
| 5 drapeaux de panne | TSafetyMonitor (20Hz) | TStateMachine (20Hz) | course formelle |

Les deux sérieux sont `engagedSegment_` et `plan_`. Le premier : `TAP` peut
calculer un guidage vers un waypoint à moitié mis à jour. Le second : si
`TStateMachine` recharge le plan pendant que `TMiSequencer` l'indexe, le tampon
du vecteur est libéré sous le lecteur — même classe que la course sur
`tasks_` du watchdog, mais sur le plan de mission.

**La règle est connue et appliquée ailleurs.** `AckFifo` et `PendingAckQueue`
(MavlinkInterface) ainsi que `StateMachineOrderQueue` (MissionControl) ont tous
leur `std::mutex`, conformément à `CLAUDE.md`. Elle n'a simplement pas été
étendue aux membres simples.

À noter l'ironie : la couche IPC se protège des lectures déchirées **entre
process** par un verrou CAS et un checksum, et chaque composant déchire son
propre état **entre ses propres threads**.

### N2 — `TFS` ignore l'âge de la `LidarMap` — Moyenne

`TFSTask::loop()` revérifie indépendamment la fraîcheur de l'EKF
(`kEkfStaleThreshold=400ms`, distinct du seuil de SF) parce que c'est une
sécurité redondante qui « ne fait confiance à aucun calcul déjà fait
ailleurs ». Puis il lit `getLidarMap()` et l'utilise **sans regarder
`map->ts`** — champ pourtant écrit à chaque publication.

Or `LidarTask` cesse de publier sans rien signaler quand l'attitude est périmée
(`SensorFusion.cpp:194`), et si le LIDAR meurt la carte garde sa dernière
valeur indéfiniment. Deux issues :

- carte figée **avec** un obstacle → `NVA004` permanent, hold permanent. Gênant
  mais sûr.
- carte figée **vide** → `TFS` conclut « pas d'obstacle », pour toujours.
  L'évitement est inopérant en silence.

Atténué un étage plus haut : `MissionControl` réagit à `SFS004` via
`lidarLost_` (HOLD + question GCS). Mais l'incohérence reste, et elle porte
précisément sur la couche dont le rôle est de ne rien supposer.

### N3 — `SFS001` détruit le journal des codes mineurs — Moyenne

`SensorFusion.cpp:123` lève `SFS001` (`GPS_lock`, Info donc mineur) à **chaque
échantillon GPS valide**, soit ~10Hz. `raiseMinor()` décale les 16 slots du ring
à chaque appel : le journal est intégralement rempli de `SFS001` en ~1,6
seconde, évinçant tout autre code mineur du composant.

Le commentaire en place dit « Info, mineur : pas de spam à craindre ». C'est
vrai du GCS — mais le ring est un journal de diagnostic, et il devient
inutilisable. À lever sur **front** (acquisition du fix) plutôt qu'en continu.

### N4 — staleness initialisée à l'epoch — Moyenne

`lastGoodGpsTs_{}` et `lastGoodLidarTs_{}` valent `TimePoint{}`.
`msBetween(epoch, now)` est donc énorme dès le **premier** cycle, bien avant
que le capteur ait pu produire quoi que ce soit.

Conséquence : `SFS002` est levé au premier tick de `GPSTask`. Or `SFS002` est
**Emergency**, avec alarme sonore `airbus_caution` et panneaux GCS
`GPS,EKF,EMG`. Une alarme Emergency se déclenche donc à chaque mise sous
tension et reste active jusqu'au premier fix — plusieurs minutes à froid. Même
schéma pour `SFS004` (Warning) côté lidar.

Techniquement « pas de GPS au démarrage » est exact. Mais une alarme critique
systématique au boot est une alarme que l'opérateur apprend à ignorer.
Initialiser ces horodatages à la construction donnerait une fenêtre de grâce
égale au seuil (1500ms GPS, 500ms lidar).

### N5 — correction partielle du §17, de ma part — Faible

J'ai traité les 6 sites de `MissionControl` et laissé les autres : 2 dans
`Navigation.cpp` (`:84`, `:88`, `pushNavReport`) et 2 dans
`MavlinkInterface.cpp` (`:89`, `:344`, `pushMavlinkReport`). Sans effet
aujourd'hui puisque ni `navReports` ni `mavlinkReports` ne sont consommés
(cf. R6/R8) — mais c'est la même classe, et je l'ai annoncé réglé.

### N6 — condition morte — Cosmétique

`SensorFusion.cpp:128` teste `!gpsHasNoNewData` alors que la fonction a déjà
retourné ligne 76 dans ce cas. Toujours vrai.

Également relevé sans gravité : `setGpsMag(sample)` est appelé même quand
l'échantillon a été rejeté, réécrivant des données identiques sous verrou.

---

### N7 — seuils batterie incompatibles avec le pack réel — **Critique**

```cpp
constexpr float kBattLowVoltage = 14.0f;       // MavlinkInterface.cpp:59
constexpr float kBattCriticalVoltage = 12.5f;  // :60
...
bool battLow = voltage > 0.0f && voltage < kBattLowVoltage;        // :255
bool battCritical = voltage > 0.0f && voltage < kBattCriticalVoltage;
```

`voltage` est la tension **du pack** (ArduPilot place le total dans
`BATTERY_STATUS.voltages[0]`, converti par `mvToVolts`), comparée directement
aux seuils — aucune division par le nombre de cellules.

Or `HARDWARE.md:170` documente un pack **LiPo 6S**. Pour 6S :

| Niveau | Tension pack | Seuil codé |
|---|---|---|
| Pleine charge (4.2 V/cell) | 25.2 V | |
| Nominal (3.7 V/cell) | 22.2 V | |
| Bas (3.5 V/cell) | **21.0 V** | 14.0 V |
| Critique (3.3 V/cell) | **19.8 V** | 12.5 V |
| Cellule détruite (2.3 V/cell) | 14.0 V | ← `kBattLowVoltage` |

**14.0 V correspond à 2.33 V/cellule et 12.5 V à 2.08 V/cellule** — des valeurs
qu'un pack 6S n'atteint jamais en vol, l'ESC coupant les moteurs bien avant
(ce que la description de `FCP003` dans `INDEX.csv` mentionne d'ailleurs
explicitement).

En revanche, 14.0 V = 3.50 V/cell et 12.5 V = 3.13 V/cell sur un pack **4S** :
ce sont des seuils 4S parfaitement sensés, vraisemblablement jamais mis à jour
après le passage en 6S.

**Conséquence** : `FCP001` (Critical) et `FCP003` (Emergency) ne peuvent pas se
lever en vol nominal. Donc :

- aucune alerte batterie basse au GCS ;
- `runPreflightChecks()` (`MissionControl.cpp:77-78`) ne bloque jamais
  l'armement sur batterie faible ;
- `unrecoverableFailureActive_` (`MissionControl.cpp:415`), qui déclenche le
  `LANDING` immédiat sans attente de récupération, ne se déclenche jamais pour
  cause de batterie.

Le drone volerait donc jusqu'à la coupure ESC, sans avertissement ni
atterrissage automatique.

**Ce point ne nécessite pas le matériel** — contrairement à R13 où je l'avais
classé. Il s'agit d'une incohérence entre le code et `HARDWARE.md`, détectable
et corrigeable sur table. Seule la valeur exacte retenue (3.5/3.3 V par cellule
ou autre) relève de ton choix.

### N8 — `value_or()` optimiste sur des checks de sécurité — Moyenne

```cpp
if (!kSkipRpmCheck && comp.fc_.getArmed().value_or(false)) {   // :222
float voltage = comp.fc_.getBattery().value_or(FCBatteryStatus{}).voltage.v;  // :254
```

Un échec de lecture shm (timeout sur le verrou CAS, corruption) produit donc
« désarmé » et « 0 V » — c'est-à-dire *tout va bien* :

- `getArmed()` en échec saute **l'intégralité du check RPM** ;
- `getBattery()` en échec donne `voltage = 0`, donc `battLow` et `battCritical`
  passent à `false`. Les tests étant edge-triggered, un timeout transitoire
  **efface** une alarme batterie légitime déjà active.

`MissionControl` fait l'inverse au même genre d'endroit :
`.value_or(ComponentHealth::DEAD)` (`MissionControl.cpp:52,286`), c'est-à-dire
pessimiste. Les deux composants traitent l'échec de lecture de façon opposée.

### N9 — `std::cout` sur le chemin temps réel de réception — Moyenne

`TRx` (priorité RT 75, 100Hz) écrit sur `std::cout` pour trois messages :
`STATUSTEXT` (`:649`), `GPS_RAW_INT` (`:667`) et `EKF_STATUS_REPORT` (`:675`),
les deux derniers demandés à 1Hz par `requestInitStreams()`.

`std::cout` prend un verrou interne et peut **bloquer** si la sortie est lente
ou si un tampon de pipe est plein. Or `TRx` est le **seul lecteur** du lien FC :
le bloquer arrête la lecture des HEARTBEAT, ce que `FSTask` interprétera en
perte de lien (`MVC004`) — une panne provoquée par la journalisation.

La doc qualifie ces trois logs d'« observabilité bring-up, log console
seulement », donc temporaires. Ils sont néanmoins sur le chemin de vol.

### Q1 — §17 couvert par deux tests d'intégration

`Audit17_FullFifoBlocksLandingTransition` et
`Audit17_FreeFifoAllowsLandingTransition` font tourner un **vrai**
`MissionControl` avec ses quatre threads RT et tout son décor shm sur la pile.

Le scénario évite d'injecter une commande GCS : avec un plan vide,
`TMiSequencer` pose `missionComplete_`, ce qui demande `Mission → Landing`. Le
FC est armé explicitement, sinon `landed` est vrai d'emblée et l'état file vers
`Idle`, masquant le point testé.

- FIFO pleine → l'état **n'annonce pas** `Landing`, et `MCA010` est levé ;
- FIFO libre → la transition a bien lieu (contrôle positif, sans lequel le
  premier test passerait même si la state machine était cassée).

**Attention : ces deux tests se skippent en silence** sans
`cap_sys_nice`. `RLIMIT_RTPRIO` valant 0 par défaut (WSL2, conteneurs), les
threads `SCHED_FIFO` ne démarrent pas et `GTEST_SKIP` s'applique. Pour les
exécuter réellement :

```bash
sudo setcap cap_sys_nice+ep build/test_audit
./build/test_audit --gtest_filter='*Audit17*'
```

C'est le même piège qui avait laissé trois tests de `test_watchdog` cassés sans
que la suite ne rougisse : **un `ctest` vert ne prouve pas que ces tests ont
tourné.** Vérifier le compte de `SKIPPED` fait partie de la relecture.

---

## Fonctionnalités non écrites

Pas des anomalies. **Attention** : trois entrées de cette section étaient
périmées — recopiées des sections « ce qui n'est pas encore fait » des docs,
écrites à un moment où elles étaient vraies, sans revérification contre le
code. Elles sont listées en fin de section avec leur correctif.

### Côté matériel — écrire un `.cpp` derrière une interface existante

Aucune ne touche au cœur : `SysMonitoring` ne connaît que `ITelemetryLink` et
`IVideoSource`, `SensorFusion` que `IDriver<T>`.

- **`WFB_NG_Driver.cpp`** (lien PDL) et **`CC1101_Driver.cpp`** (lien SDL) :
  `.hpp` seuls. **C'est le vrai blocage COM** — tout le pipeline télémétrie
  tourne aujourd'hui sur `UdpTelemetryDriver` en loopback, donc validé de bout
  en bout, il ne manque que le transport réel.
- **Driver `IVideoSource` V4L2** : rien ne lit la caméra USB.
  `UdpVideoDriver` est un bouchon qui reçoit du MJPEG déjà encodé. Pré-requis
  du HUD vidéo.

### Côté protocole — transfert du plan de vol

Le plan **existe et fonctionne** : format `MPLN` (magic/version/header/TLV/
CRC32), `parse()`/`serialize()` testés, `tools/mission_compiler.py` compile du
YAML, MC le charge au `PREFLIGHT` et `TMiSequencer` l'exécute. Ce qui manque est
uniquement son **transport** : aucun paquet de `TELEMETRY.csv` ne porte un plan,
il arrive donc par un fichier sur disque. Manquent l'upload GCS → drone et le
tracé côté GCS.

### Côté fonctionnel

- **Pilotage manuel** : `CMD_WAYPOINT` est bien routé, lève `MCA002`
  (`gcs_command_unrouted`) et s'arrête — « pas de destination implémentée ».
  Suppose de router vers Nav via `navCmds` et une loi de consigne.
- **Consommateur vidéo** : `VideoTask::loop()` appelle `pollFrame()` puis jette
  la frame. Aucun destinataire défini (buffer partagé ? export réseau ?) —
  décision d'archi.
- **Mesure de la fréquence réelle des `Task`** : voir ci-dessous, c'est la seule
  entrée de cette section qui soit activement trompeuse aujourd'hui.
- **`ARM_STATE.reason`** : reste à 0, pas de table de raisons définie (IHM).
- **Qualité de lien** : pas de RSSI matériel, `lastHeartbeatMain/Sec` sert de
  proxy de fraîcheur. Suffisant pour détecter une perte, pas pour afficher une
  qualité.
- **Port UART du FC** : `kFcUartPort` est un placeholder non vérifié.
- **Seuils RPM** : `kMinArmedRpm=1000`/`kMaxArmedRpm=10000`, valeurs de départ
  arbitraires à calibrer au banc.

### Entrées périmées, retirées

| Entrée | Ce qui était écrit | Réalité vérifiée |
|---|---|---|
| Logique de `TStateMachine` | « non écrite » | **Écrite et testée** : 8 états dans `MC_CONV::step()`, 34 tests GTest, exercée par le test d'intégration du §17 |
| Règle de cold-start MC | « absente du code » | **Implémentée** (QF), cf. `ARCHITECTURE_IPC.md` §4.3 |
| Plan de vol | « pipeline d'upload pas construit » | Exact, mais le **plan lui-même** fonctionne — seul le transport manque, cf. ci-dessus |

Trois autres entrées du corps de l'audit ont été retirées pour la même raison :
`CompCore` (lu par les 5 composants, 29 sites), `R8` (`navCmds` **est** utilisé :
MC pousse `MCO005`, Nav le dépile), `R10` (non-action sur `MVC004`
**explicitement voulue et documentée**, le failsafe GCS d'ArduPilot s'en charge)
et `R11` (`verticalArmed` différé avec les lois de guidage, cf. `NAVIGATION.md`
§7).

**Règle tirée de ces cinq cas** : une entrée d'audit issue d'un balayage par
motifs ou recopiée d'une doc se revérifie contre le code avant d'être actionnée.

---

## Déjà réglé

Conservé en une ligne pour ne pas re-chercher ce qui l'a été. Le détail est dans
l'historique git et dans les docs de référence concernées.

| # | Anomalie | Traitement |
|---|---|---|
| 1b | Confusion « slot vide » / « lecture shm échouée » | `getStartTs()` renvoie `expected<optional<TimePoint>, shmError>` ; escalade pessimiste sur échec de lecture ; `peekError()` non destructif pour ne pas priver le watchdog |
| 3 | Pas de reprise après un `fork()` échoué | 3 tentatives espacées de 50 ms, puis trace explicite |
| 4 | Lost update sur les codes de diagnostic | `modifyData()` tient le verrou shm sur la séquence lire-modifier-écrire — **0 perte / 3000** |
| 8 | `codes.hpp` tracké désynchronisé | `include/drone/generated/` dé-tracké + `.gitignore` |
| 12 | `LD06`/`LIS3MDL` sans test | `tests/test_drivers.cpp` (15 tests) + bancs `bench_gps`/`bench_lidar`/`bench_mag` |
| F1 | Le GCS affichait un `hz` fabriqué (sinus) | Mesure par compteur d'itérations dans `Task`, fenêtre d'1 s calculée par `ComponentWatchdog`, `taskHz`/`taskTargetHz` dans `SharedSysStateMem`, paquet `TASK_RATE` (ID 27, `MAIN`, 5 Hz, défilement des 19 `Task`), carte GCS branchée sur la mesure réelle — cf. `SYSMONITORING.md` §13 |
| 17 | Commandes de vol perdues en silence | `MCA010` sur refus + transition bloquée pour LAND/disarm/set-mode |
| 20 | Protections désactivables par l'environnement | les trois lectures sous `#ifdef DRONE_USE_SITL_DRIVERS` ; `=1` exigé au lieu d'une simple présence |
| R6+R8 | Rapports de commande jamais lus | `TStateMachine` draine les deux FIFO, `MCA011` sur échec |
| R7 | `ARM_STATE` aux valeurs par défaut | alimenté par `fc_.getArmed()` sur **les deux** liens |
| R9 | `HomeCalibrator` toujours `{0,0,0}` | origine NED capturée au front d'armement du FC |
| 2 | Course thread watchdog / `registerTask` + `heartbeat` | thread lancé par `start()` hors du ctor ; `lastHeartbeat` via `std::atomic_ref` |
| 5 | Deux erreurs `launchRTThread` avalées | `start()` renvoie `expected` ; `startupError()` fait sortir le rôle, GW escalade |
| 6 | ~~`CompCore` jamais lu~~ | **constat erroné** : lu par les 5 composants sur 29 sites, rien à corriger |
| 19 | `keepRunning_` static de classe | laissé en l'état, raison documentée (handler POSIX sans `this`) — `ARCHITECTURE_IPC.md` §2.5 |
| — | Priorité du watchdog 80 → 90 | au-dessus de `TSafetyMonitor` (85) : le placement cesse d'être une condition de correction |
| 1 | **Échelle hot/cold/DEAD inatteignable** — tout redémarrage rapporté `NOMINAL` | `firstBoot` transmis par `GlobalWatchdog` via `argv`, plus aucun sondage de la shm |
| 7 | Coquille `getColdtStartTs` | renommé `getColdStartTs` |
| 9 | `sendUrgent()` non documenté, renvois `§8` faux | documenté en `MAVLINKINTERFACE.md` §9, renvois corrigés |
| 10 | **Lecture hors bornes du parseur UBX** (NAV-SVINFO) | `payloadSize == 8 + 12×numCh` exigé, détail plafonné à 16 |
| 11 | **Checksum UBX jamais vérifié**, 1 octet sur 2 consommé | Fletcher vérifié, trames rejetées et comptées (`checksumErrors()`) |
| 13 | `gpsFixOk` acceptait un fix « time only » | `0x03` ou `0x04` uniquement |
| 14 | `NAV-TIMEUTC` : drapeaux de validité ignorés | bit `validUTC` exigé |
| 15 | Allocation `std::vector` dans un thread RT | `std::array` de 16 + compteurs |
| 16 | 354 lignes de code mort dans `main.cpp` | supprimées (601 → 246 lignes) |
| 18 | `kPayloadSize` non lié aux `take()` | contrôle de l'offset final dans les deux drivers UDP |
| — | `shm_timeout` jamais lu | supprimé de `ComponenConfig` |

**Régression évitée pendant la correction du §10**, à retenir : plafonner `sats`
à 16 tronquait `satellitesVisible`, que `SensorFusion` dérivait de
`sats.size()`. Le compte envoyé au FC et au GCS passait donc à 16 au plus, alors
qu'un récepteur multi-constellation suit couramment 20 à 30 canaux. D'où la
séparation `satellitesTracked` (non plafonné) / `satDetailCount` (plafonné),
verrouillée par un test. Référence permanente : `SENSORFUSION.md` §3.
