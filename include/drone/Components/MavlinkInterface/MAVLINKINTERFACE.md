# MavlinkInterface — architecture, Tasks, piping, SITL

Documentation de référence du composant `MavlinkInterface` : rôle de
chaque Task, comment un message MAVLink circule, les décisions d'archi
actées, et les pièges concrets rencontrés face à ArduPilot (réels et
SITL). Complète `GUIDE_COMPOSANTS.md` (générique) et le SDD (`Document de
Conception Logicielle.pdf`, §3.5 et §4).

**État** : implémenté et validé en vol SITL (arm/guided/takeoff/vitesse
horizontale/altitude/GPS_INPUT). `TCmd`↔`MissionControl` fonctionne déjà
via `SharedCmdMem`, bien que `MissionControl` lui-même n'existe pas
encore. Revue de code complémentaire passée en session (résultats ACK,
staleness, failsafe FC, observabilité) — cf. §6/§7/§9/§10.

## 1. Vue d'ensemble

```
FC (ArduPilot, UART en prod / UDP en SITL) ◄──► MavlinkInterface (process isolé)
                                        │
                                        ├─ TTx     : flux routine (GPS_INPUT,
                                        │            SET_POSITION_TARGET_LOCAL_NED,
                                        │            HEARTBEAT)
                                        ├─ TTxACK  : flux ponctuel ACK'd (COMMAND_LONG :
                                        │            SET_MESSAGE_INTERVAL, NAV_TAKEOFF,
                                        │            DO_SET_MODE, COMPONENT_ARM_DISARM),
                                        │            retry/timeout
                                        ├─ TRx     : parsing des messages entrants,
                                        │            écrit dans SharedFCStatus
                                        ├─ TCmd    : dépile SharedCmdMem (ordres de
                                        │            MissionControl), traduit en
                                        │            COMMAND_LONG poussés vers TTxACK
                                        └─ FSTask  : surveille le HEARTBEAT FC (§9)

SensorFusion   ── écrit SharedSFMem.gpsMag  → TTx construit GPS_INPUT
Navigation     ── écrit SharedNavMem.cmd    → TTx construit SET_POSITION_TARGET_LOCAL_NED
MissionControl ── écrit SharedCmdMem        → TCmd construit COMMAND_LONG
SharedFCStatus ── lu par SensorFusion (attitude/position), MissionControl (batterie/ESC)
```

Le lien physique (UART en prod, UDP en SITL) n'est connu qu'au travers de
l'interface `IMavlinkLink` (`UartMavlinkLink`/`UdpMavlinkLink`), injectée
au constructeur — jamais un type concret dans le composant lui-même.

## 2. Les 5 Tasks

| Task | Rôle | Fréquence |
|---|---|---|
| `TTx` | HEARTBEAT périodique (1Hz) + GPS_INPUT/SET_POSITION_TARGET_LOCAL_NED événementiels | 50Hz |
| `TTxACK` | Dépile `ackQueue_` (une commande ACK'd à la fois), gère retry/timeout | 20Hz, retry 500ms×3 |
| `TRx` | Drain UDP/UART, parse, dispatch vers `SharedFCStatus`/`ackFifo_` | 100Hz, jusqu'à 64 datagrammes/tick |
| `TCmd` | Dépile `SharedCmdMem`, traduit en `COMMAND_LONG` poussé dans `ackQueue_` | 50Hz |
| `FSTask` | Surveille la fraîcheur de `SharedFCStatus.lastHeartbeat`, lève `MVC004` (§9) | 20Hz |

Contrairement à ce qui était envisagé avant implémentation, il n'y a
**pas** de classe `MavScheduler` à score continu — `TTx` fait un simple
test « nouveau + pas périmé » par flux (§4), et `TTxACK` une FIFO bornée
+ un seul pending à la fois (§6). Plus simple que prévu, suffisant en
pratique.

## 3. Identité MAVLink et canaux — pièges réels rencontrés

**Une seule identité pour tout.** `MavlinkInterface` usurpe l'identité
GCS (`kGcsSystemId=255`, `MAV_COMP_ID_MISSIONPLANNER`) pour **tous** les
messages émis — HEARTBEAT, COMMAND_LONG, GPS_INPUT,
SET_POSITION_TARGET_LOCAL_NED. Décidé en session après un vrai bug :
utiliser une identité "onboard computer" séparée pour les commandes (une
seule pour le HEARTBEAT, une autre pour le reste) cassait le routage
MAVProxy/le filtrage `SYSID_MYGCS` côté ArduPilot — `SET_MESSAGE_INTERVAL`
n'était jamais ACK'd tant que la commande ne provenait pas de la même
identité que le HEARTBEAT.

**Un canal MAVLink par thread émetteur.** Les fonctions `_pack()` (sans
suffixe `_chan`) routent implicitement vers `MAVLINK_COMM_0`, dont le
compteur de séquence (`mavlink_status_t::current_tx_seq`) est une
structure globale **partagée, non protégée**. `TTx` et `TTxACK` tournent
dans des threads RT séparés ; les faire empaqueter tous les deux sur ce
canal implicite créait une race de données réelle (corruption de
séquence → MAVProxy gonflait artificiellement son compteur "lost"
pendant les salves `TTxACK`, jusqu'à plusieurs milliers). Fix : chaque
thread émetteur a son propre canal (`kTxChanTTx=MAVLINK_COMM_1`,
`kTxChanTTxAck=MAVLINK_COMM_2`), via les variantes `_pack_chan()`. `TRx`
garde `MAVLINK_COMM_0` pour `mavlink_parse_char()` (RX, pas de conflit
avec les canaux TX). Aucun mutex nécessaire — isolation native de la lib.

## 4. GPS_INPUT / SET_POSITION_TARGET_LOCAL_NED : événementiel, jamais périodique

Contrairement à `HEARTBEAT` (périodique pur, doit toujours partir pour ne
jamais déclencher le failsafe GCS du FC), les messages qui transportent
des données issues de SensorFusion/Navigation sont envoyés **uniquement
quand une nouvelle donnée existe** (`ts` différent du dernier envoyé) et
**pas périmée** (`kMaxSampleAge = 300ms`) — pas de redondance périodique
de secours. Raison de sécurité explicite : si la source (SF/Nav) meurt
silencieusement, on ne veut **surtout pas** que le FC continue de
recevoir une position/vitesse périmée — l'absence de message doit se
traduire par un timeout EKF côté FC (failsafe autonome ArduPilot), pas
être masquée par un renvoi mécanique de la dernière valeur connue.

**Piège pour les tests manuels (dashboard/pymavlink)** : un clic unique
dans un formulaire écrit `NavCommand`/`GpsMagSample` **une seule fois** —
`ts` ne change plus jamais après, donc `TTx` n'envoie qu'un seul message
puis se tait. Côté ArduPilot, GUIDED attend un flux **continu** de
`SET_POSITION_TARGET_LOCAL_NED` (timeout `GUID_TIMEOUT`, 3s par défaut) ;
un GPS "figé" au-delà de quelques centaines de ms est vu comme périmé.
`UDP_bridge.cpp` (SITL) compense en réécrivant `ts = now()` toutes les
150ms tant qu'aucune nouvelle valeur n'est arrivée — cf. §11. En
production, ce n'est pas un problème : `Navigation`/`SensorFusion`
tournent en continu et produisent un nouveau `ts` à chaque cycle.

## 5. Altitude : vitesse pure + P-controller — piège ArduCopter non documenté

**Ce qui a été essayé et a échoué.** L'intention initiale était de
piloter `vx`/`vy` en vitesse et `z` en position absolue dans le même
`SET_POSITION_TARGET_LOCAL_NED` (mélange position/vitesse). Constaté en
session, confirmé indépendamment via un script pymavlink isolé (donc pas
un bug côté `droneMk2`) :

- ArduCopter accepte ce mélange au niveau protocole (`COMMAND_ACK`
  OK, pas d'erreur) mais **n'a strictement aucun effet** sur le
  véhicule — dès qu'une seule composante de vitesse est active (non
  ignorée dans `type_mask`), ArduCopter bascule **tout** le message en
  mode vitesse et ignore purement et simplement le champ position, même
  si son bit "ignore" n'est pas posé.
- `MAV_CMD_GUIDED_CHANGE_ALTITUDE` (alternative testée : une commande
  séparée, ACK'd, dédiée au changement d'altitude en GUIDED) n'est pas
  supporté par le firmware SITL testé — `COMMAND_ACK` renvoie
  `MAV_RESULT_UNSUPPORTED`.

**Solution retenue.** `SET_POSITION_TARGET_LOCAL_NED` part en **vitesse
pure sur les 3 axes** (x/y/z tous ignorés, `vx`/`vy`/`vz` tous actifs).
`NavCommand.altitudeCmd` reste une altitude **absolue** côté
`SharedNavMem` (rien ne change côté Navigation/dashboard) — c'est
`TTxTask::loop()` qui la convertit en `vz` juste avant l'envoi, via un
simple P-controller (`altitudeToVz()`, `Kp=0.6`, plafond `±2 m/s`) :

```cpp
vz = clamp(Kp * (altitudeCmd - pos.z_courant), -2.0f, 2.0f)
```

`pos.z_courant` vient de `SharedFCStatus::getLocalPosVel()` (donc de la
dernière `LOCAL_POSITION_NED` reçue de la FC elle-même — boucle fermée
sur la vraie position, pas sur une estimation locale). Sans échantillon
de position disponible, `vz = 0` (tient l'altitude en l'état plutôt que
d'extrapoler à l'aveugle). Validé en session : convergence propre sans
oscillation sur ~3-5m d'écart, cohabite normalement avec `vx`/`vy`.

## 6. Retry / échec sur commande ACK'd (`TTxACK`)

Toute commande `COMMAND_LONG` (depuis `requestInitStreams()` ou `TCmd`)
passe par `ackQueue_` (`PendingAckQueue`, capacité 8, `push()` échoue
plutôt que d'écraser). `TTxACK::loop()` :

1. Dépile `ackFifo_` (ACK reçus routés par `TRx`) — si aucune commande
   n'est en attente ou si l'ACK ne correspond pas (`ackMatchesPending()`,
   comparaison sur `command`/`param_id`), lève `CODES::MVC001`.
   `ackMatchesPending()` ne fait que l'identité — le **résultat** est
   décodé séparément par `TTxACK` une fois le match confirmé :
   - `MAV_RESULT_ACCEPTED` → résolu, slot libéré normalement.
   - `MAV_RESULT_IN_PROGRESS` → pas final (ex: premier ACK d'un
     `NAV_TAKEOFF`), on continue d'attendre le vrai résultat.
   - tout le reste (`DENIED`/`FAILED`/`UNSUPPORTED`/
     `TEMPORARILY_REJECTED`...) → échec réel, **vide `ackQueue_`**
     (`PendingAckQueue::clear()`) pour ne pas enchaîner aveuglément la
     commande suivante (ex: `guided` refusé suivi d'un `arm`+`takeoff`
     quand même — bug identifié en revue, corrigé).
2. Si une commande est en vol : timeout 500ms, jusqu'à 3 tentatives.
   Échec définitif après 3 tentatives → même traitement qu'un résultat
   négatif explicite (`ackQueue_.clear()`), pas juste la libération du
   slot.
3. Sinon, dépile `ackQueue_` et amorce le suivi ACK/retry sur la
   commande suivante — jamais deux commandes ACK'd en vol simultanément.

**Notification du résultat, `resolvePending()`.** Que la commande
réussisse ou échoue, `TTxACK` doit prévenir quelqu'un — la destination
dépend de l'origine (`PendingAck::fromMissionControl`, posé par `TCmd`
au moment de traduire un ordre `SharedCmdMem`, cf. §8) :
- **Ordre MissionControl** (`fromMissionControl=true`) → un
  `SharedCmdMem::CommandReport` est poussé (`pushMavlinkReport()`),
  corrélé par `seq` au `PendingCommand` d'origine. **Remplace**
  l'ancien `CODES::MVC002` générique pour ce cas — un code de diagnostic
  ne dit pas *quelle* commande a échoué si plusieurs sont en vol, un
  report si.
- **Commande interne** (`fromMissionControl=false`, ex: les 8
  `SET_MESSAGE_INTERVAL` de `requestInitStreams()`) → pas d'origine
  `SharedCmdMem` à qui reporter, donc `CODES::MVC002` reste levé sur
  échec (rien sur succès, comme avant).
- Un succès sans origine MissionControl ne lève rien (comportement
  silencieux inchangé) ; un report `mavlinkReports` plein lève
  `CODES::MVC003` (même code que pour `ackQueue_`/`ackFifo_` pleines).

**Commandes actuellement routées par ici** : `MAV_CMD_SET_MESSAGE_INTERVAL`
(x5, séquence d'init, cf. §7), `MAV_CMD_NAV_TAKEOFF`,
`MAV_CMD_DO_SET_MODE`, `MAV_CMD_COMPONENT_ARM_DISARM` (via `TCmd`, cf. §8).

`SET_POSITION_TARGET_LOCAL_NED` et `GPS_INPUT` (envoyés par `TTx`) ne
sont **pas** des commandes ACK'd — ce sont des messages de consigne/
télémétrie, pas des `MAV_CMD`. ArduPilot renvoie périodiquement
`POSITION_TARGET_LOCAL_NED` (sans le `SET_`) en écho de la consigne
actuellement retenue, mais `TRx` ne l'écoute pas aujourd'hui (pas de
vérification "le FC a bien pris en compte ma dernière consigne").

## 7. Séquence d'init (`requestInitStreams()`)

ArduPilot n'envoie par défaut que HEARTBEAT (1Hz). Pour tout le reste,
`MavlinkInterface::requestInitStreams()` empile 8 `MAV_CMD_SET_MESSAGE_
INTERVAL` dans `ackQueue_` :

| Message demandé | Fréquence | Note |
|---|---|---|
| `ATTITUDE` | 10Hz | **Pas** `ATTITUDE_QUATERNION` — jamais émis par ce firmware SITL, quelle que soit la demande. `TRx` décode l'Euler et convertit via `eulerToQuat()` (§10). |
| `LOCAL_POSITION_NED` | 10Hz | Conforme au standard. |
| `BATTERY_STATUS` | 1Hz | Conforme. |
| `ESC_TELEMETRY_1_TO_4` | 2Hz | **Pas** `ESC_STATUS` générique — extension `ardupilotmega`, seule que ce firmware émet réellement (§10). |
| `SCALED_PRESSURE` | 2Hz | Conforme. |
| `SYS_STATUS` | 1Hz | Santé capteurs vue par le FC (present/enabled/health) — stockée dans `SharedFCStatus::sensorHealth`. |
| `GPS_RAW_INT` | 1Hz | Observabilité (fix_type/satellites vus par le FC lui-même, à comparer à ce qu'on lui envoie via `GPS_INPUT`) — idem, log seulement. |
| `EKF_STATUS_REPORT` | 1Hz | Observabilité (flags EKF3, variances) — idem, log seulement. |

Taux réduits par rapport aux cibles de prod (50Hz attitude/position à
terme) — validés en session sur un lien SITL PC↔Pi peu fiable (wifi) qui
perdait presque tout au-delà du HEARTBEAT à ces débits. À revoir une
fois sur liaison filaire ou en conditions réelles UART.

**Déclenchement automatique, plus de bootstrap manuel.**
`requestInitStreams()` était auparavant appelée directement par
`MV_SITL.cpp`/`HIL_SF_MV.cpp` juste après construction du composant —
problème identifié en revue : à la construction, `UdpMavlinkLink` n'a
pas encore de pair (`havePeer_=false`, personne n'a encore envoyé de
paquet), donc les 5-8 requêtes échouaient silencieusement et `TTxACK`
les abandonnait après 3 tentatives (~1,5s), sans garantie qu'un retry
tombe après l'apprentissage du pair. Fix : `TRx` déclenche
`requestInitStreams()` lui-même, une seule fois (`initStreamsRequested_`),
à la réception du **premier vrai HEARTBEAT du FC** — signal fiable que
le lien est vivant, aussi bien pour `UdpMavlinkLink` (pair appris) que
pour `UartMavlinkLink` (FC démarré et qui parle). Les binaires SITL/HIL
n'ont plus besoin d'appeler cette méthode.

## 8. `TCmd` / `SharedCmdMem` — ordres MissionControl et leur résultat

`SharedCmdMem` est le `SharedCompMem` de `MissionControl` (`/drone_cmd`
en prod, `/drone_sitl_cmd` en SITL) : une **paire** de `RingBuffer<_, 8>`
par composant subordonné — un pour les ordres, un pour les reports de
résultat — aujourd'hui seule celle de `MavlinkInterface` existe
(`mavlinkCmds`/`mavlinkReports`), les FIFO pour Nav/SF viendront quand
ces composants en auront besoin.

`PendingCommand{CODES::Code code; uint32_t payload; uint32_t seq;
TimePoint ts}` — le `code` identifie l'ordre (catégorie
`CODES::Category::Command`, lettre `O` pour Ordre), `payload` est un
champ libre uint32_t (ex: altitude en cm pour un takeoff), `seq` est
assigné **par `SharedCmdMemHandler::pushMavlinkCommand()` elle-même**
(compteur monotone stocké en shm, jamais par l'appelant — élimine toute
collision/oubli côté MissionControl). Poussé/dépilé via
`pushFifo()`/`popFifo()`, génériques sur `SharedMemoryHandler` (§13) —
pas de mutex, safe en shm inter-process via le même CAS-lock que
`getData`/`setData`.

`CommandReport{uint32_t seq; CODES::Code code; bool success; TimePoint
ts}` — poussé par `TTxACK` (`resolvePending()`, cf. §6) une fois le
`PendingCommand` correspondant résolu (ACK positif, négatif, ou
timeout). Corrélation par `seq`, pas par contenu — indispensable si
deux ordres identiques (ex: deux `arm`) sont en vol. Volontairement
minimal (juste succès/échec, pas le `MAV_RESULT` détaillé) — un
diagnostic plus fin passerait par les `CODES::` propres au subordonné,
pas par ce canal de corrélation.

Ordres actuellement traduits par `TCmdTask::loop()` :

| Code | MAV_CMD | Notes |
|---|---|---|
| `MCO001` (takeoff) | `MAV_CMD_NAV_TAKEOFF` | `payload` = altitude cible en **cm**, converti en mètres (param7) |
| `MCO002` (set_mode_guided) | `MAV_CMD_DO_SET_MODE` | `base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`, `custom_mode=COPTER_MODE_GUIDED` |
| `MCO003` (arm) | `MAV_CMD_COMPONENT_ARM_DISARM` | param1=1 |
| `MCO004` (disarm) | `MAV_CMD_COMPONENT_ARM_DISARM` | param1=0 |

Un code inconnu de `TCmd` ne part jamais sur le lien MAVLink (pas encore
de `CODES::` dédié à ce cas — à ajouter si de nouveaux ordres arrivent
sans traduction ici), mais pousse quand même un `CommandReport{success=
false}` — sinon MissionControl attendrait indéfiniment un report qui ne
viendrait jamais pour un ordre qu'il a pourtant bien émis.

## 9. `FSTask` — détection de perte de lien FC (observabilité, pas d'action)

`FSTask::loop()` compare `SharedFCStatus.lastHeartbeat` à `Clock::now()`
(`kFcLinkTimeout=3000ms`, aligné sur le failsafe GCS natif d'ArduPilot,
2-3s) et lève/efface `CODES::MVC004` (edge-triggered). **Volontairement
pas d'action autonome côté Pi** : le failsafe GCS d'ArduPilot bascule
déjà en LAND de façon indépendante du Pi en cas de perte de lien (SDD
§4, "Dernier filet de sécurité") — dupliquer cette décision côté
`MavlinkInterface` sans coordination avec `MissionControl` serait un
choix d'archi à part, pas pris ici. `FSTask` rend juste l'info
observable (logs, futur dashboard/MissionControl) là où elle était
avant totalement invisible côté Pi.

## 10. Messages qu'ArduPilot SITL émet réellement — pas toujours les "standards"

Constaté en session, à retenir pour toute extension future de `TRx` :
avant de câbler le décodage d'un nouveau message, **vérifier ce
qu'ArduPilot envoie vraiment** plutôt que de supposer que la variante
"standard" MAVLink est utilisée.

- `ATTITUDE`(30, Euler) au lieu de `ATTITUDE_QUATERNION`(31) → conversion
  locale via `eulerToQuat()` (Conversions.hpp), formule Tait-Bryan ZYX
  standard aéro.
- `ESC_TELEMETRY_1_TO_4`(11030, extension `ardupilotmega`) au lieu de
  `ESC_STATUS`(291, générique commun) — champs différents (`voltage[4]`
  en cV, `current[4]` en cA non signé, `temperature[4]` fourni,
  contrairement à `ESC_STATUS` qui n'a pas de température).
- `GLOBAL_POSITION_INT`(33) arrive aussi en flux par défaut mais n'est
  pas décodé (on utilise `LOCAL_POSITION_NED`(32), qui lui est conforme
  au standard et arrive bien).
- Trafic parasite observé sur le lien : un HEARTBEAT `system=255,
  component=230` à haute fréquence (~8Hz), ni nous ni le FC — probable
  sonde interne de MAVProxy pour calculer ses propres statistiques de
  lien. `TRx` filtre déjà `setLastHeartbeat()` sur l'identité exacte du
  FC (`kFcSystemId`/`kFcComponentId`) pour ne pas être trompé par ce
  genre de trafic (cf. commentaire dans `TRxTask::loop()`).
- `STATUSTEXT` (id 253) : longtemps tombé dans `default: break` — c'est
  pourtant par ce canal qu'ArduPilot annonce les échecs de prearm check,
  la source EKF3 effectivement utilisée, les erreurs capteur. Décodé
  depuis la revue de code (§ "État" en tête de doc) : `TRx` logge
  `severity` + `text` sur `std::cout`. Pas de champ `SharedFCStatus`
  dédié pour l'instant — juste de la visibilité console pendant
  l'intégration/bring-up, à revoir si un consommateur programmatique
  (MissionControl, dashboard) en a besoin.

## 11. `GPS_INPUT` : temps GPS (`time_week`/`time_week_ms`), yaw, précision

**`time_week`/`time_week_ms` viennent du GPS lui-même, pas de l'horloge du
Pi.** Piège identifié en revue de code : une première version calculait
ces champs depuis `GpsMagSample::ts` — mais `ts` est `TYPES::Clock` =
`std::chrono::steady_clock`, dont l'époque est arbitraire (proche du
boot système sous Linux), **pas** 1970. Le calcul produisait donc un
numéro de semaine GPS sans aucun rapport avec la réalité (juste non-nul,
donc il passait le garde-fou "time_week≠0" d'ArduPilot sans être
correct). Fix : `GpsMagSample::gpsWeek`/`gpsTowMs`, remplis côté
`SensorFusion`/`GPSTask` depuis la vraie source — date calendaire UBX
NAV-TIMEUTC (`SF_CONV::gpsWeekFromUbxDate()`) + `iTowMs` (ms dans la
semaine, NAV-PVT, précision native du récepteur) — cf. `SENSORFUSION.md`
§3. `MavlinkInterface` ne fait plus aucun calcul, juste un passe-plat de
ces deux champs.

`time_usec` reste calculé depuis `ts` (`steady_clock`) : la spec
GPS_INPUT accepte explicitement "epoch Unix OU temps depuis le boot
système" pour ce champ précis (contrairement à `time_week`), donc pas de
bug là.

`GPS_INPUT.yaw` est envoyé à **`0` en permanence** ("cap inconnu" selon
la spec MAVLink) — le magnétomètre est câblé directement sur le FC
(décision actée en session, cf. `SENSORFUSION.md` §3.5), `EK3_SRC1_YAW`
pointe vers le compas du FC, plus vers `GPS_INPUT.yaw`. `SensorFusion`
ne fournit plus de cap sur ce lien du tout.

`GPS_INPUT` transporte aussi `hdop`/`vdop`/`vert_accuracy` — remplis
depuis UBX NAV-DOP (hDOP/vDOP) et NAV-POSLLH.vAcc côté driver GPS
(`GPSTask`, cf. `SENSORFUSION.md` §3) ; laissés à 0 avant la revue de
code, ce qui annonçait à l'EKF une précision parfaite alors qu'ils
n'étaient simplement jamais mesurés (`ignore_flags=0` affirme que tous
les champs sont valides — un 0 par défaut dans un champ de précision
est trompeur, pas neutre).

## 12. Logs de debug — retirés

Les `std::fprintf(stderr, ...)` marqués DEBUG TEMPORAIRE (un par
message reçu dans `TRx`, un par `SET_POSITION_TARGET_LOCAL_NED` envoyé
dans `TTx`) ont été retirés après la phase d'investigation SITL — sur
des threads RT `SCHED_FIFO`, une I/O bloquante en boucle (50-100Hz)
pollue le jitter et viole le principe "pas d'I/O bloquante en boucle
RT". Les événements réellement utiles à observer (`STATUSTEXT`,
`SYS_STATUS`, `GPS_RAW_INT`, `EKF_STATUS_REPORT`, cf. §10) restent
loggés via `std::cout`, mais uniquement sur réception d'un message —
pas à chaque tick de boucle, donc pas la même classe de risque
(occasionnel vs. garanti à chaque cycle).

## 13. Dépendance `mavlink/c_library_v2`

Intégré en **git submodule** (pas vendored, pas FetchContent) — c'est ce
que fait ArduPilot lui-même et l'écosystème companion-computer autour.
`target_include_directories` vers `ardupilotmega/` (hérite de
`common.xml`, dialecte utilisé partout dans ce composant). Headers C
purs déjà générés — rien à générer nous-mêmes.

---

## SITL — mise en place, réseau, config ArduPilot

### Binaires dédiés

- **`mv_sitl`** (`src/IntegrationTests/MV_SITL.cpp`) : fait tourner `MavlinkInterface`
  seule face à ArduPilot SITL, sans `GlobalWatchdog`. **Publie** ses
  propres segments shm dédiés (`SITL::k*ShmPath`, `/drone_sitl_*` —
  distincts de ceux de `GlobalWatchdog` pour ne jamais entrer en
  collision si un vrai `drone` tournait par erreur en même temps). Lien
  UDP (`UdpMavlinkLink`, port local 14550). `requestInitStreams()` se
  déclenche seule au premier HEARTBEAT reçu du FC (§7), rien à appeler ici.
- **`UDP_bridge`** (`src/IntegrationTests/UDP_bridge.cpp`) : pont shm↔UDP pour piloter
  les tests depuis le dashboard web. S'**attache** (pas publish) aux
  mêmes segments que `mv_sitl` — doit donc être (re)lancé **après**
  `mv_sitl`, et relancé à chaque fois que `mv_sitl` redémarre (nouveaux
  segments = ancien attachement invalide). Réutilise les vrais handlers
  (`SharedNavMemHandler`, etc.) — aucune réimplémentation du verrou CAS.
  Deux ports localhost : `15001` (le pont pousse l'état FC/Nav/SF vers le
  dashboard, ~10Hz), `15000` (le pont écoute les injections de test).
- **Ordre de lancement** : `mv_sitl` → `UDP_bridge` → `dashboard_server.py`
  (`tools/dashboard/`, page `/sitl/mv`).

Ces binaires (+ `hil_sf_mv`, cf. `src/IntegrationTests/HIL_SF_MV.cpp` — test
HIL SensorFusion+MavlinkInterface face au vrai matériel) sont construits
seulement si `-DBUILD_ITESTS=ON` (option CMake) :
`cmake --build build --target mv_sitl udp_bridge hil_sf_mv`.

### Cycle de vie shm et pièges de process

- `mv_sitl` doit être arrêté par **`SIGTERM`** (pas `kill -9`) pour que
  `destroySharedMemory()` s'exécute proprement. `MavlinkInterface`
  (Tasks incluses) est construit dans un bloc `{}` dédié **avant** le
  nettoyage shm dans `main()` — sinon les threads `TTx`/`TTxACK`/`TRx`
  continuent d'accéder à la shm pendant son démontage (SEGV constaté en
  session, use-after-unmap, cf. `Task::~Task()`/`ComponentBase`).
- Un `kill -9` laisse des segments shm orphelins dans `/dev/shm/` —
  nettoyer manuellement (`rm /dev/shm/drone_sitl_*`) avant de relancer,
  sinon le prochain `mv_sitl` peut segfaulter sur une shm dans un état
  incohérent.
- `UDP_bridge` attaché à une shm désormais détruite (parce que `mv_sitl`
  a redémarré entre-temps) continue de tourner sans crasher mais
  n'affiche plus rien de frais côté dashboard — toujours le relancer en
  même temps que `mv_sitl`.

### Réseau : PC SITL (Windows/WSL2) ↔ Pi

**Ce qui marche : `--out` au lancement, pas `link add` en console.**

```
sim_vehicle.py -v ArduCopter --out=udp:<IP_PI>:14550 --console --map
```

`--out` fait pousser MAVProxy en continu vers le Pi, flux initié côté
WSL2 → traverse proprement le NAT WSL2 (qui gère bien le retour d'un
flux qu'il a lui-même initié). `UdpMavlinkLink` apprend le pair à la
réception (`peerAddr_`/`havePeer_`, cf. `poll()`) — conçu exactement
pour ce schéma, aucun changement de code nécessaire.

**Ce qui ne marche PAS, malgré des apparences trompeuses : `link add`
en console MAVProxy après coup.** Testé en session, `link add
udpout:<IP_PI>:14550` affiche un lien "UP" avec un % de qualité
correct — mais ne relaie **jamais** le vrai trafic du véhicule (confirmé
via `[TRx]` : seul un HEARTBEAT parasite `system=255/component=230`
arrivait, jamais `system=1`, cf. §10). Cause exacte non identifiée côté
MAVProxy — ne pas perdre de temps dessus, `--out` fonctionne et est plus
simple.

**Mode réseau WSL2** : `mirrored` (Windows 11 22H2+,
`networkingMode=mirrored` dans `.wslconfig`) simplifie le sujet en
partageant la pile réseau de l'hôte, mais `--out` fonctionne aussi bien
en NAT classique — pas un prérequis strict.

### Paramètres ArduPilot à régler pour un test SITL réaliste

Aucun de ces réglages n'est un défaut de `mv_sitl`/`MavlinkInterface` —
tous nécessaires côté SITL pour refléter l'archi réelle (pas de GPS ni
de compas physiquement câblés sur le FC ; GPS et cap arrivent tous les
deux via `GPS_INPUT` depuis SensorFusion). **Chaque changement ci-dessous
nécessite un redémarrage complet du véhicule SITL** (pas juste `param
set` — ces sous-systèmes sont instanciés au boot) :

```
param set GPS_TYPE 14        # GPS "MAVLink" (accepte GPS_INPUT) au lieu du GPS simulé interne
param set SIM_GPS1_ENABLE 0  # désactive le GPS simulé, évite le conflit avec GPS_INPUT
param set COMPASS_ENABLE 0   # pas de compas sur le FC réel — cap uniquement via GPS_INPUT.yaw
param set EK3_SRC1_YAW 2     # dit à l'EKF3 d'utiliser le GPS (pas le compas) comme source de cap
```

Sans `EK3_SRC1_YAW 2` après avoir désactivé le compas, l'EKF reste
bloqué en attente d'un cap compas qui n'existe plus →
pre-arm check `Need Position Estimate`.

### Séquence de test type (console MAVProxy ou dashboard)

```
mode guided
arm throttle
takeoff 5        # ou bouton "Décollage" du dashboard (MCO001)
```

Puis consigne Nav (vitesse horizontale + altitude absolue) depuis
`/sitl/mv` — formulaire "Injecter — consigne Nav" (`vel.x`, `vel.y`,
`heading`, `altitude` en NED ; **pas** de champ `vel.z`, calculé en
interne, cf. §5). Formulaire GPS/mag : lat/lon/alt/vel/heading/fix_type/
satellites, converti en `GPS_INPUT` par `TTx`.

### Diagnostic bas niveau (quand MAVProxy ne suffit pas)

- **`/proc/net/udp`** (`rx_queue` d'un socket) pour prouver si des
  octets arrivent réellement au niveau OS, indépendamment de toute
  interprétation applicative — utile face à un symptôme "rien ne se
  passe" pour trancher entre perte réseau réelle et bug de code.
- **Scripts pymavlink isolés** (scratchpad, pas dans le repo) pour
  tester une hypothèse MAVLink sans dépendre du code `droneMk2` — a
  permis de confirmer/infirmer plusieurs comportements ArduCopter (§5)
  de façon indépendante. Pattern : `mavutil.mavlink_connection("udpin:
  0.0.0.0:<port>")`, envoyer son propre HEARTBEAT en tâche de fond
  (sinon MAVProxy ne considère jamais le lien "up"), filtrer le premier
  HEARTBEAT reçu sur `autopilot != MAV_AUTOPILOT_INVALID` (sinon on
  capte l'écho de son propre HEARTBEAT ou celui d'un autre GCS).

## 14. Ce qui n'est PAS encore fait / tranché

- `FSTask` détecte la perte de HEARTBEAT (§9) mais ne déclenche aucune
  action côté Pi — décision explicite (l'action existe déjà côté FC via
  son propre failsafe GCS), mais si `MissionControl` doit un jour réagir
  à `MVC004` (RTB, log, notif GCS...), le câblage reste à faire.
- Pas de vérification que le FC a bien pris en compte la dernière
  consigne `SET_POSITION_TARGET_LOCAL_NED` (écho `POSITION_TARGET_
  LOCAL_NED` non écouté par `TRx`, cf. §6).
- `GPS_RAW_INT`/`EKF_STATUS_REPORT` décodés mais seulement loggés
  (`std::cout`), pas de champ `SharedFCStatus` dédié (§7/§10). `SYS_STATUS`
  lui est stocké dans `SharedFCStatus::sensorHealth`.
- Pas de `CODES::` dédié pour un ordre `TCmd` inconnu (§8).
- Table des messages MAVLink → pas de générateur façon `TELEMETRY.csv` :
  on s'appuie directement sur les structs générées par `c_library_v2`.
- `payload` de `PendingCommand` limité à un `uint32_t` — à revoir si un
  futur ordre a besoin de plus qu'un scalaire.
- `RingBuffer`/FIFO de `SharedCmdMem` pour Nav/SF pas encore créées (que
  celle de `MavlinkInterface` existe, cf. §8).
- Trois canaux TX (`COMM_0/1/2`) mais une seule identité `(sysid,
  compid)` — ArduPilot suit la séquence par identité, pas par canal, donc
  il verra des sauts de séquence entre `TTx`/`TTxACK` et rapportera de la
  perte de paquets fantôme dans ses propres stats de lien. Fonctionnellement
  bénin (aucun octet n'est réellement perdu), mais fausse le diagnostic
  MAVProxy — un `compid` distinct par tâche réglerait ça proprement, pas
  fait pour l'instant (identifié en revue de code, priorité basse).
- Migration du cap magnétique vers un magnétomètre câblé directement sur
  le FC : faite côté code (`GPS_INPUT.yaw=0` en permanence, cf. §11 ;
  `SensorFusion` ne fournit plus de cap, cf. `SENSORFUSION.md` §3.5).
  Reste à faire côté FC (hors code Pi) : `EK3_SRC1_YAW`/`COMPASS_ENABLE`
  à re-régler pour la nouvelle archi.
