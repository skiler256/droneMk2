# SITL — plateforme de test système

Documentation de référence de tout ce qui vit sous `sitl/` : ce qui est
**construit et qui tourne**, l'architecture, comment écrire un scénario, et
les binaires de test intermédiaires (SITL et HIL).

Trois blocs, comme discuté :

```
binaire (drone) <--UDP--> SITL_MASTER (sitl/, Python) <--MAVLink--> SITL_ardupilot
```

- **binaire** : le vrai `drone` (`GlobalWatchdog` + 5 process composants),
  compilé avec des drivers UDP factices à la place du vrai matériel I2C/UART.
  Aucune logique métier modifiée pour les besoins du test — principe
  directeur : **validation boîte noire**, les tests parlent au système par
  ses interfaces externes (MAVLink, UDP capteur, shm en lecture), aucun
  binding sur le code C++, le binaire testé est le binaire de production.
- **SITL_MASTER** : les scripts Python sous `sitl/` — orchestre le run,
  relaie le GPS/lidar simulés, pilote la mission via de vraies commandes
  GCS, observe l'état système, calcule un verdict.
- **SITL_ardupilot** : `sim_vehicle.py` d'ArduPilot — fournit la dynamique
  de vol, l'EKF3, le compas simulé. Nécessite le venv `~/venv-ardupilot`
  (dépendances pyserial/empy/etc.) — l'activer (`source
  ~/venv-ardupilot/bin/activate`) avant de lancer quoi que ce soit à la
  main en dehors de `sitl/run.sh`.

### Ce que la plateforme valide

- Boucle de guidage fermée : `Nav` → setpoints → contrôleur guided ArduPilot
  → dynamique.
- Chaîne capteur complète : monde 3D → mesures → `SensorFusion` →
  `MavlinkInterface` → EKF.
- Comportement système émergent : mission complète pilotée par
  `MissionControl`.
- Gestion de panne : processus mort, code de diagnostic levé, lien coupé.
- Évitement d'obstacle géométrique (freinage, cf. §9).

### Ce que la plateforme ne valide pas

- Le comportement temps réel fin sur cible Pi 5 (jitter, latence RT) — banc
  matériel séparé.
- Les drivers matériels réels (I2C, UART) — remplacés par des drivers UDP
  (validés séparément en HIL, cf. §12).
- La dynamique de vol elle-même — déléguée à ArduPilot SITL, prise comme
  référence, jamais réimplémentée.
- Le bruit/biais/latence capteur réaliste — `sensor_bridge.py` reste un
  passthrough idéal aujourd'hui (cf. §10).

---

## 1. Démarrage rapide

```bash
# Mode interactif (pas de scénario, juste les process + le dashboard,
# pratique pour piloter à la main et regarder http://localhost:8766/sitl/prod) :
sitl/run.sh

# Jouer un scénario, sans ArduPilot (pour vérifier le plombage) :
sitl/run.sh sitl/scenarios/001_arm_takeoff_wait_land.yaml

# Avec un vrai ArduPilot SITL :
sitl/run.sh --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py \
            sitl/scenarios/001_arm_takeoff_wait_land.yaml

# Une fois pour ne plus retaper --sim-vehicle :
export ARDUPILOT_SIM_VEHICLE=~/ardupilot/Tools/autotest/sim_vehicle.py
sitl/run.sh sitl/scenarios/001_arm_takeoff_wait_land.yaml

# Avec les fenetres MAVProxy (console + carte) -- poste de dev, DISPLAY
# fonctionnel requis (WSLg). Sans effet en environnement headless.
sitl/run.sh --gcs-windows --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py \
            sitl/scenarios/001_arm_takeoff_wait_land.yaml

# Diagnostic : sensor_bridge bypass MAVLink/SIM_STATE/LidarServer, envoie une
# trame GPS+LD06 fixe en continu -- isole UdpGpsDriver/UdpLidarDriver/
# SensorFusion/shm d'un probleme de reception SIM_STATE cote ArduPilot/MAVLink.
sitl/run.sh --sensor-fake-fixed --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py

# Avec le moteur 3D (LidarServer.gd, geometrie reelle du LD06/TF Luna) --
# binaire Godot headless, telecharge depuis les releases officielles (cf. §9) :
sitl/run.sh --godot-headless ~/godot3/Godot_v3.6-stable_linux_headless.64 \
            --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py \
            sitl/scenarios/001_arm_takeoff_wait_land.yaml

# Avec le viewer temps reel lance automatiquement (cf. §9, un seul terminal) :
sitl/run.sh --godot-editor ~/godot3-editor/Godot_v3.6-stable_x11.64 \
            --godot-headless ~/godot3/Godot_v3.6-stable_linux_headless.64 \
            --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py \
            sitl/scenarios/004_lidar_course.yaml
```

**⚠️ Piège rencontré en lançant depuis un environnement sans vrai accès X11**
(ex: agent/CI, session sans WSLg fonctionnel) : `_x11_available()`
(`procman.py`) ne verifie que la presence d'un socket X11 joignable, pas
une authentification (`XAUTHORITY`) valide dessus — si le socket existe
mais que l'auth echoue, `run_in_terminal_window.sh` (ArduPilot) tente
d'ouvrir un xterm et **echoue en silence** (exit 0), `arducopter` n'est
jamais demarre, `sim_vehicle.py`/MAVProxy tourne en boucle
demarrage/echec ("MAVProxy exited" repete) sans jamais se stabiliser.
Contournement : `env -u DISPLAY sitl/run.sh ...` force le repli headless
(log dans `/tmp/ArduCopter.log`), meme si un `DISPLAY` techniquement
"joignable" est present.

`sitl/run.sh` construit (`build_sitl/`, séparé du build de prod — jamais
les mêmes drivers), crée le venv Python (`sitl/.venv/`) si besoin, démarre
tout, et nettoie proprement à la sortie (Ctrl+C ou fin de scénario).

**⚠️ Étape à refaire après CHAQUE rebuild de `drone`** (le `.gitignore`-style
oubli le plus fréquent en session) :

```bash
sudo setcap cap_sys_nice+ep build_sitl/drone
```

Sans ça, les Tasks RT (`SCHED_FIFO`) ne démarrent jamais — silencieusement,
aucune erreur visible. Le process tourne, la shm existe, mais rien ne
s'exécute réellement (heartbeats jamais mis à jour, aucune commande
traitée). `setcap` est un attribut par inode : un rebuild relinke le
binaire et le perd. Cause racine : `RLIMIT_RTPRIO=0` par défaut sous
WSL2/sandbox (pas besoin d'un noyau PREEMPT_RT pour `SCHED_FIFO`, juste de
la permission — `CAP_SYS_NICE` la contourne entièrement).

**Prérequis à poser toi-même** (pas construits dans ce fil) : cloner et
compiler ArduPilot (`ArduPilot/ardupilot`, `Tools/autotest/sim_vehicle.py`).
Tout doit tourner sur la même machine (WSL), tout communique en UDP
loopback — RTF 1 obligatoire (cf. §11), un seul domaine de temps cohérent.

### Déboguer en observant en direct, sans copier-coller

`http://127.0.0.1:8766/api/bridge/status` (même API que le dashboard web)
renvoie tout l'état shm en JSON — utile pour surveiller `mission_state`,
`nav.age_ms` (segment `PROD_NAV`), `lidar.age_ms`/`count` (`PROD_LIDAR`) et
les codes actifs (`PROD_SYS`) pendant qu'un run tourne, sans attendre un
copier-coller manuel. Combiné à la lecture directe de `sitl/logs/*.log`,
ça a suffi à diagnostiquer plusieurs bugs réels en session (cf. §9) —
utile en particulier quand `run_scenario.py` a été modifié pour **ne plus
jamais couper automatiquement à la fin d'un scénario** (cf. §3.1
ci-dessous) : le run reste observable aussi longtemps que nécessaire,
Ctrl+C manuel obligatoire pour arrêter.

---

## 2. Flux de données et ports

```
world_engine/LidarServer.gd (bloc [3], géométrie pure)
   ▲  requête/réponse UDP privée, port 15102 (cf. §9, raycast_wire.py)
   │
sim_vehicle.py (ArduPilot SITL)
   │  MAVLink UDP, port 14550 (vers drone) / 14551 (SIM_STATE, vers sensor_bridge.py)
   │                                        ▲
   │           DISTANCE_SENSOR (TF Luna) ───┘ sensor_bridge.py interroge LidarServer
   ▼            injecté par sensor_bridge.py     (pose+attitude), relaie GPS/LD06/TF Luna
drone (MavlinkInterface, UdpMavlinkLink)
   │  shm interne (mêmes chemins qu'en vol : /drone_nav, /drone_sf, ...)
   │
   ├── SensorFusion  ◄── UdpGpsDriver (port 15100) ◄── sensor_bridge.py ◄── SIM_STATE (ArduPilot)
   │                 ◄── UdpLidarDriver (port 15101) ◄── sensor_bridge.py ◄── LidarServer.gd (LD06)
   │
   └── SysMonitoring ◄── TELEM::* UDP (port 5601) ◄── mission_driver.py (CMD_ARM, CMD_TAKEOFF, ...)
                     ──► TELEM::* UDP (port 5602) ──► (personne ne lit ce flux pour l'instant,
                                                        c'est le "vrai" canal GCS)

udp_bridge (C++, attache la shm de prod en LECTURE SEULE)
   ──► UDP texte "SEGMENT=PROD_<NOM>\ncle=valeur..." (port 15001, ~10Hz)
        │
        ▼
tools/dashboard/dashboard_server.py (SEUL lecteur du port 15001)
   ├─► page web http://localhost:8766/sitl/prod (humain)
   └─► API JSON http://localhost:8766/api/bridge/status (sitl/shm_observer.py, orchestrateur)
```

**Point important** : `udp_bridge` envoie son flux de statut vers une
cible UDP fixe (pas de broadcast/multicast) — un seul process peut le
lire à la fois. `dashboard_server.py` est ce lecteur unique (démarré
systématiquement par `procman.py`), et `shm_observer.py` (côté
orchestrateur) l'interroge ensuite via son API HTTP plutôt que d'écouter
le port UDP lui-même. **Bug déjà rencontré et corrigé** : les deux
essayant de binder le même port UDP 15001 → `OSError: Address already in
use` au démarrage de l'orchestrateur.

| Port | Qui écoute | Rôle |
|---|---|---|
| 14550 | `drone` (MavlinkInterface) | Lien MAVLink vers ArduPilot SITL |
| 14551 | `sensor_bridge.py` (cible, pas écoute) | Flux MAVLink SIM_STATE depuis SITL (`--sim-udp`), et cible du `DISTANCE_SENSOR` (TF Luna) injecté vers SITL |
| 15100 | `drone` (SensorFusion, `UdpGpsDriver`) | GPS simulé |
| 15101 | `drone` (SensorFusion, `UdpLidarDriver`) | LD06 simulé (géométrie réelle depuis `LidarServer.gd`) |
| 15102 | `world_engine/LidarServer.gd` | Requête/réponse raycast privée (`raycast_wire.py`) — jamais vue par le système sous test |
| 5601 | `drone` (SysMonitoring, TelMain) | Commandes GCS montantes (`CMD_ARM` etc.) |
| 5602 | (rien pour l'instant) | Télémétrie descendante — pas consommée par l'orchestrateur |
| 15001 | `dashboard_server.py` | Statut shm depuis `udp_bridge` |
| 8766 | `dashboard_server.py` | Page web + API JSON |

### Protocole texte du pont (`udp_bridge`)

Pas de bibliothèque JSON : un datagramme = un segment, débogable à la main
avec `nc -ul <port>`.

- **Statut** (le pont émet, ~10Hz, port 15001) : ligne 1
  `SEGMENT=<NS>_<NOM>` où `<NS>` vaut `SITL`, `HIL`, `NAV_SITL`, `MC_SITL` ou
  `PROD`, puis une ligne `cle=valeur` par champ.
- **Injection** (le pont écoute, port de commande) :
  `CMD=<NOM>\nNS=<SITL|HIL|NAV_SITL|MC_SITL|PROD>\ncle=valeur...`.

Le pont s'attache **de façon opportuniste et indépendante** aux segments des
binaires de test (`mv_sitl`, `hil_sf_mv`, `nav_sitl`, `mc_sitl`) comme aux
vrais chemins de prod, réessayant périodiquement : les binaires peuvent
démarrer avant ou après lui, dans n'importe quel ordre, et plusieurs
namespaces peuvent être actifs simultanément. Il réutilise les vrais handlers,
sans réimplémenter le verrou CAS.

**`PROD` est en lecture seule** (`allowGpsMagInject=false`) : les vrais
writers — Navigation, SensorFusion et MissionControl réels — sont déjà en
place.

**`CMD=GPSMAG` n'est routable que vers `SITL`.** Côté `HIL`, le vrai
`SensorFusion` tourne et est l'unique writer légitime de `gpsMag` : laisser le
pont y écrire créerait une situation à deux writers, contraire au principe
un-seul-writer-par-segment (cf. `ARCHITECTURE_IPC.md` §2.1). Côté `NAV_SITL`,
aucun `SensorFusion` ne tourne, mais `gpsMag` n'y sert à rien non plus (GPS
simulé laissé côté SITL). Rejeté dans les deux cas, avec trace sur `stderr`.

**Le routage des capteurs n'est pas symétrique** — il reproduit
l'architecture réelle plutôt qu'un chemin uniforme :

| Capteur | Réel | En simulation |
|---|---|---|
| GPS (NEO-M8N) | UART → Pi | `sensor_bridge.py` → `UdpGpsDriver` (port 15100) → `SensorFusion` |
| Magnétomètre (LIS3MDL) | I2C → FC | simulé directement par ArduPilot SITL, rien à injecter |
| TF Luna (rangefinder) | UART → FC | `sensor_bridge.py` → `DISTANCE_SENSOR` MAVLink → ArduPilot SITL (port 14551), **pas** `UdpLidarDriver` |
| LD06 (360°) | UART → Pi | `sensor_bridge.py` → `UdpLidarDriver` (port 15101) → `SensorFusion` |
| IMU / Baro | FC | simulés par ArduPilot SITL |

Le TF Luna, en particulier, ne passe **pas** par `UDP_bridge` : il est
routé comme sur le vrai FC, traité par ArduPilot SITL lui-même
(EKF/rangefinder) avant de redescendre éventuellement vers `drone` par les
canaux MAVLink habituels.

---

## 3. Créer un scénario

Un fichier déclaratif par scénario, versionné dans le repo — pas de
scénario en code (sinon impossible de dire « ce commit a cassé le
scénario 3 »). Format YAML, plus simple que ce qui était envisagé au départ
(pas encore de `modele_capteur`/graine de bruit dédiée par scénario, cf. §10
"pas encore fait") :

```yaml
id: 001_arm_takeoff_wait_land
description: >
  Texte libre.

plan: 001_arm_takeoff_wait_land.plan.yaml
monde: world_engine/worlds/empty.json

mission:
  # EKF3 met ~30s a converger a froid -- un arm envoye trop tot est
  # rejete et n'est jamais retente (mission_driver.arm() = envoi unique).
  - {type: wait, duration: 30}
  - {type: arm}
  - {type: takeoff, alt: 10}
  - {type: wait, duration: 5}
  - {type: land}

criteres:
  - {metrique: mission_state_sequence, attendu: [Armed, Mission, Landing]}
  - {metrique: aucun_code_emergency, attendu: true}
  - {metrique: processus_vivants_a_la_fin, attendu: [drone, udp_bridge]}
```

### Plan de mission (`plan:`)

Le scénario pilote **SITL_MASTER** (ARM/TAKEOFF/LAND côté GCS, injection de
pannes, timing) — **jamais** la trajectoire de vol elle-même. Ça, c'est le
rôle du plan de mission, un fichier YAML séparé référencé par `plan:`
(chemin relatif au dossier du scénario), compilé vers le binaire que
`MissionControl` lit réellement (`tools/mission_compiler.py`, codec dans
`tools/missionplan_wire.py` — miroir Python de `MISSIONPLAN_CONV`
côté C++, validé par round-trip contre le vrai décodeur).

```yaml
header:                      # optionnel, champs manquants = défauts C++
  battWarningV: 14.0
  battCriticalV: 12.5
  rtbTransitAlt: 30.0
  gcsLossAction: 0
  navVolumeRadius: 200.0
  navVolumeCeiling: 100.0
  takeoffAlt: 10.0            # metres, positif = vers le haut -- altitude REELLE
                              # de decollage (le "alt" de l'etape takeoff du
                              # scenario est cosmetique, cf. §3 ci-dessous)

segments:                    # optionnel, liste vide = mission qui se
                              # termine instantanément (cf. piège ci-dessous)
  - type: Hold                # seuls Waypoint et Hold sont executes, cf. §3
    duration: 30.0             # champs propres au type, cf. missionplan_wire.py
```

**Piège déjà rencontré** : `TMiSequencer` considère la mission terminée dès
que `nextIndex_ >= plan.segments.size()` — un plan **sans segment** déclenche
`missionComplete_` au tout premier tick une fois en `Mission`, avant même
qu'un `wait`/`land` du scénario n'ait eu le temps de s'exécuter. Toujours
mettre au moins un segment (`Hold` ou `Waypoint`) tant que le plan ne
couvre pas toute la mission voulue — sinon c'est le plan qui la termine,
pas le scénario.

Pas de `plan:` dans le scénario → `procman.py` écrit un plan par défaut sans
segment (suffisant pour tester jusqu'à l'arm, pas au-delà).

**Seuls `Waypoint` et `Hold` sont réellement exécutés** par `TMiSequencer`
(`MissionControl.cpp`) — `Corridor`/`Curve`/`Landing`/`PayloadAction` existent
dans le schéma binaire mais sont **sautés** (code `MCA001`, pas encore
routés côté Nav — `SegmentType` n'a que `Waypoint` pour l'instant).

**Altitude de takeoff = `plan.header.takeoffAlt`, pas le `alt` du scénario.**
Le paramètre `alt` de l'étape `takeoff` (`{type: takeoff, alt: 10}`) est
envoyé dans `CMD_TAKEOFF` mais **ignoré** par `MissionControl` — c'est le
plan de mission qui fixe l'altitude réelle (`MCO001` lit `plan_.header.takeoffAlt`).
Garde les deux valeurs cohérentes si tu les changes, rien ne les
synchronise automatiquement.

**`TMiSequencer` attend la fin de la montée avant d'armer le premier
segment** (altitude à ±1m de `takeoffAlt` **et** vitesse verticale <0.3m/s,
cf. `kTakeoffAltToleranceM`/`kTakeoffVertSpeedToleranceMs`) — sans ça, les
commandes `velNED` du premier `Waypoint` entraient en collision avec le
`NAV_TAKEOFF` d'ArduPilot encore en cours (constaté en session : le drone
ne décollait jamais vraiment). ArduPilot ne signale pas explicitement
"takeoff terminé" sur `NAV_TAKEOFF` (juste `ACCEPTED`), donc c'est nous qui
le détectons sur l'état réel (`DynamicState`, `SensorFusion`).

**`STABILIZE` (mode par défaut au sol) ne supporte pas le takeoff
programmatique** — `MissionControl` passe en `GUIDED` (`MCO002`) juste
avant d'armer (`MCO003`), sinon `NAV_TAKEOFF` est rejeté (`FAILED`) au
moment du décollage (cf. `ArduCopter/takeoff.cpp::has_user_takeoff()`).

### Monde 3D (`monde:`)

**Obligatoire**, même parallélisme que `plan:` — un scénario, c'est la
combinaison des trois : séquence SITL_MASTER + plan de vol + monde. Résolu
depuis la **racine du repo** (pas depuis le dossier du scénario comme
`plan:`) : les mondes vivent dans `world_engine/worlds/` et sont partagés
entre scénarios (`res://worlds/` côté Godot). Absent → erreur explicite au
chargement (`scenario.load()`), pas de valeur par défaut côté scénario —
seul `procman.py` retombe sur `world_engine/worlds/empty.json` en mode
interactif (pas de scénario du tout). Détails du moteur qui le charge : §9.

### Exemple avec vrais segments Waypoint (`002_arm_waypoints_land.yaml`)

Roadmap étape 3 ("Nav validé sur SITL nu") — un aller-retour de deux
`Waypoint` réels, `Nav` pilote dessus pour de vrai (pas juste un `Hold`) :

```yaml
# 002_arm_waypoints_land.plan.yaml
header:
  takeoffAlt: 10.0
segments:
  - {type: Waypoint, north: 15.0, east: 0.0, alt: -10.0, speed: 2.0, acceptRadius: 2.0}
  - {type: Waypoint, north: 0.0, east: 0.0, alt: -10.0, speed: 2.0, acceptRadius: 2.0}
  - {type: Hold, duration: 5.0}
```

Note `alt` du segment `Waypoint` : convention NED du plan de mission (négatif
= vers le haut), différente de `takeoffAlt` du header (positif = vers le
haut) — ne pas confondre les deux conventions dans le même fichier.

### Étapes de mission supportées (`sitl/run_scenario.py::run_step`)

| type | params | effet |
|---|---|---|
| `arm` | — | `CMD_ARM`, attend `mission_state == Armed` |
| `takeoff` | `alt` (m) | `CMD_TAKEOFF`, attend `mission_state == Mission` — `alt` cosmétique, cf. ci-dessus |
| `wait` | `duration` (s) | n'envoie rien, observe pendant N secondes |
| `land` | — | `CMD_LAND`, attend `mission_state == Landing` |

Pas encore câblés : `hold`/`resume` (`CMD_HOLD`/`CMD_RESUME` existent déjà
côté `mission_driver.MissionDriver`, juste pas exposés comme type d'étape),
`answer` (répondre à une `MC_QUERY`). Ajouter un type d'étape = un `elif`
dans `run_step()`, rien d'autre.

**Plus de coupure automatique à la fin du scénario** (bug corrigé en
session : `land` du scénario n'attend que la *transition* vers
`mission_state=="Landing"`, pas la fin réelle de l'atterrissage — couper
tout de suite après coupait le SITL en pleine descente). Une fois toutes
les étapes de `mission:` jouées, le verdict est calculé et affiché
immédiatement, mais **la simulation continue de tourner** — seul un
Ctrl+C explicite arrête tout proprement (ArduPilot, `drone`, viewer
compris). Pratique pour observer un atterrissage jusqu'au bout, ou pour
laisser le run tourner pendant qu'on surveille en direct (cf. section
précédente).

### Critères disponibles (`sitl/verdict.py::evaluate`)

| métrique | `attendu` | vérifie |
|---|---|---|
| `mission_state_sequence` | liste d'états | vue comme sous-séquence (pas forcément consécutive) dans les états observés |
| `aucun_code_emergency` | `true`/`false` | aucun code de sévérité `Emergency` (`INDEX.csv`) vu pendant le run |
| `processus_vivants_a_la_fin` | liste de noms process | chacun encore vivant juste avant l'arrêt volontaire |
| `lidar_hit_range_m` | `[min, max]` (mètres) | au moins un point lidar (`PROD_LIDAR`, `pt.N.range_mm`) observé dans cette plage pendant le run — preuve qu'un obstacle réel a été détecté, pas juste "rien à moins de 30m" |

Métrique inconnue = échec explicite (pas ignorée silencieusement).

---

## 4. Les modules

| Fichier | Rôle |
|---|---|
| `scenario.py` | Parsing YAML → `Scenario`/`MissionStep`/`Criterion` (+ `plan` et `monde`, obligatoires tous les deux, chemins résolus différemment — cf. §3) |
| `tools/missionplan_wire.py` | Codec du plan de mission binaire (miroir Python de `MISSIONPLAN_CONV`, magic/version/header/segments TLV/CRC32-C) |
| `tools/mission_compiler.py` | CLI : compile un plan de mission YAML vers le binaire lu par `MissionControl` — vise à devenir l'outil de compilation côté GCS |
| `procman.py` | Cycle de vie des process (`dashboard`, `sim_vehicle`, `drone`, `udp_bridge`, `lidar_server`, `sensor_bridge`) — `with ProcessManager(cfg) as pm:`, arrêt garanti même sur exception |
| `rangefinder.parm` | Params ArduPilot pour le TF Luna en rangefinder MAVLink (`RNGFND1_TYPE=10 RNGFND1_ORIENT=25 RNGFND1_MIN_CM=20 RNGFND1_MAX_CM=800 EK3_RNG_USE_HGT=-1` — lidar hors fusion EKF de hauteur) — passé via `--add-param-file` dans `procman.py`. Sans lui, `DISTANCE_SENSOR` envoyé par `sensor_bridge.py` est ignoré en silence (aucun effet sur l'EKF/altitude simulée) |
| `shm_observer.py` | Interroge `dashboard_server.py` (HTTP) en tâche de fond, expose `mission_state()`/`active_codes()`/`wait_for_mission_state()` |
| `mission_driver.py` | Envoie les commandes GCS via le vrai canal `TELEM::*` (réutilise `tools/dashboard/telemetry_codec.py`) + heartbeat GCS 1Hz automatique |
| `verdict.py` | Évaluation pure des critères à partir d'une `RunTrace` (même esprit que `MC_CONV::step()` côté C++ — logique testable sans rien lancer) |
| `gps_wire.py` / `lidar_wire.py` | Codec du format UDP GPS/lidar simulé (miroir Python de `UdpGpsDriver`/`UdpLidarDriver` C++) |
| `raycast_wire.py` | Codec du protocole privé requête/réponse entre `sensor_bridge.py` et `world_engine/LidarServer.gd` (jamais vu par le système sous test) |
| `sensor_bridge.py` | Lit `SIM_STATE` d'ArduPilot SITL (une seule fois, fusionné) : relaie GPS (~10Hz), interroge `LidarServer.gd` pour le LD06 (~10Hz, `UdpLidarDriver`) et le TF Luna (~100Hz, `DISTANCE_SENSOR` vers ArduPilot). `--fake-fixed` : bypass MAVLink/LidarServer, trame GPS+LD06 fixe en continu (diagnostic, pas de TF Luna dans ce mode) |
| `run_scenario.py` | Point d'entrée : charge le scénario, orchestre tout, imprime le verdict |
| `run.sh` | Wrapper shell : build + venv + délègue à `run_scenario.py` |

**Seuils batterie SITL** : le modèle de frame ArduPilot SITL simule un pack
3S générique (`maxVoltage` codé en dur à 12.6V dans `SIM_Frame.h` — `SIM_BATT_VOLTAGE`
y est *clippé*, impossible de le dépasser via `--defaults`/`--add-param-file`),
incompatible avec `kBattLowVoltage`/`kBattCriticalVoltage` calibrés sur un
vrai pack 4S (`MavlinkInterface.cpp`, 14.0V/12.5V). `procman.py` positionne
`DRONE_BATT_LOW_V=11.0`/`DRONE_BATT_CRITICAL_V=9.5` (variables d'env, même
mécanisme que `DRONE_MISSION_PLAN_PATH`) pour `drone` en SITL uniquement —
comportement de prod inchangé si ces variables ne sont pas définies.

**Check RPM/ESC désactivé en SITL** : ArduPilot SITL standard ne simule pas
de télémétrie ESC/DShot (`ESC_TELEMETRY_1_TO_4` rejeté, code `command_rejected`/
`MVC002` — bénin, sans rapport avec l'arm). Sans données ESC, la staleness
se déclenchait immédiatement dès l'armement (`FSTask::loop()`) et désarmait
le drone sans aucune demande GCS. `procman.py` positionne
`DRONE_SKIP_RPM_CHECK=1` pour `drone` en SITL.

**Ces surcharges n'existent que dans un binaire SITL.** `DRONE_SKIP_RPM_CHECK`,
`DRONE_BATT_LOW_V`, `DRONE_BATT_CRITICAL_V` et `DRONE_MISSION_PLAN_PATH` sont
compilées sous `#ifdef DRONE_USE_SITL_DRIVERS` : un binaire de production les
ignore totalement, le code de lecture n'y est même pas présent. Une variable
restée dans un service systemd ou un environnement hérité ne peut donc pas
désactiver une protection de vol.

À noter aussi que `DRONE_SKIP_RPM_CHECK` est désormais comparée à `"1"` et non
testée par simple présence : `=0` désactivait le check, l'inverse de ce que la
variable laisse lire.

---

## 5. Injection de pannes

Deux domaines distincts, jamais mélangés — l'injection de panne capteur ne
doit jamais se faire dans le moteur 3D (§9), qui reste géométrie pure :

### Pannes système (`procman.ProcessManager.kill()`)

```python
pm.kill("drone")  # SIGKILL par défaut — teste GlobalWatchdog + le reste du système
```

Pour tester "kill + le composant reste mort" (pas juste "kill + respawn
avant que quiconque réagisse") : voir `SharedWatchdogControl`
(`include/drone/core/WatchdogControl.hpp`) — positionner
`suppressRespawn[id]=true` AVANT le kill. Pas encore de wrapper Python
dédié pour ça (le mécanisme C++ existe et est testé côté GTest
`RespawnSuppressedWhenFlagSet`, cf. `tests/test_global_watchdog.cpp`) — à
écrire quand un scénario en aura besoin.

### Pannes capteur / codes de diagnostic

Depuis le dashboard (`http://localhost:8766/sitl/prod`) : lever/effacer un
code sur un composant tiers, ou changer sa santé — mêmes formulaires que
les autres dashboards SITL de ce projet. Via l'API directement :

```bash
curl -X POST http://localhost:8766/api/bridge/inject -H "Content-Type: application/json" \
  -d '{"ns":"PROD","cmd":"CODE","fields":{"component":"SensorFusion","code":"SFS004","action":"raise"}}'
```

Pas encore construit : dropout/bruit/latence réalistes sur le GPS/LD06/TF
Luna simulés eux-mêmes (bloc [4], "modèle capteur", roadmap étape 4) — pour
l'instant `sensor_bridge.py` relaie un GPS fixe idéal et des distances
géométriques brutes sans bruit (cf. §9).

---

## 6. Le dashboard (`tools/dashboard/sitl_prod.html`)

Vue générique des 9 segments shm de prod (namespace `PROD` de
`UDP_bridge.cpp`) : FC, Navigation, SensorFusion, Lidar, Santé+codes,
Segment, MissionControl, Liens GCS, `SharedWatchdogControl`. Démarré
automatiquement par `procman.py` — jamais besoin de le lancer à la main.

"État de la simulation" ici = état du **système** observé via shm (santé,
codes, mission state) — **pas** le suivi de scénario de l'orchestrateur
(id du scénario en cours, quel critère passe/échoue, temps écoulé). Ce
dernier n'existe que dans la sortie console de `run_scenario.py` pour
l'instant ; rien n'empêche de l'ajouter au dashboard plus tard si le
besoin se fait sentir.

---

## 7. Options CMake

```bash
cmake -B build_sitl -DBUILD_ITESTS=ON -DDRONE_USE_SITL_DRIVERS=ON -DDRONE_USE_SIM_CLOCK=ON
```

- `DRONE_USE_SITL_DRIVERS` : bascule `SensorFusion`/`MavlinkInterface` vers
  des drivers UDP factices (`UdpGpsDriver`/`UdpLidarDriver`/`UdpMavlinkLink`)
  au lieu du vrai matériel I2C/UART. `SysMonitoring` a toujours ses drivers
  UDP par défaut (pas besoin de cette option pour lui).
- `DRONE_USE_SIM_CLOCK` : `TYPES::Clock` devient `SimClock` au lieu de
  `std::chrono::steady_clock` — pour l'instant un simple relais (pas de
  pilotage explicite du temps), posé en prévision du rejeu accéléré
  (§10 ci-dessous). Sans effet observable aujourd'hui, mais nécessaire pour
  ne pas avoir à re-router `Clock::now()` partout plus tard.

Ces deux options sont **globales** (pas par cible) : jamais de mélange de
politique horloge/drivers entre unités de traduction d'un même binaire.

---

## 8. Roadmap — où on en est

| # | Étape | Statut |
|---|---|---|
| 0 | Abstraction d'horloge | ✅ fait (`SimClock`, `DRONE_USE_SIM_CLOCK`) |
| 1 | Nav en tests unitaires purs | ✅ fait |
| 2 | Format scénario + squelette orchestrateur, sans 3D | ✅ fait (ce document) |
| 3 | Nav validé sur SITL nu | ✅ fait — scénario `002_arm_waypoints_land` (vrais `Waypoint`, `Nav` pilote dessus, verdict `PASS`) |
| 4 | Moteur 3D + modèle capteur | 🚧 en cours — boucle complète validée en SITL réel (scénario `004_lidar_course`, obstacles réellement détectés en vol), viewer temps réel + 3 caméras construits, modèle capteur bruité pas commencé, cf. §9 |
| 5 | Enregistreur / rejeu accéléré | pas commencé |
| 6 | Élargissement | pas commencé |

---

## 9. Moteur 3D (`world_engine/`, roadmap étape 4)

Godot 3.6 choisi (léger, déjà connu, le lockstep de Gazebo importe
peu en mode "serveur de capteurs passif" — RTF1 imposé de toute façon ;
Gazebo a des soucis OpenGL rapportés sous WSL2 — alternative écartée).
Binaire officiel (pas de
paquet système ici), headless pour les tests automatisés/le run SITL réel,
éditeur complet (x11) pour un visuel manuel — lancé depuis un terminal WSL
normal, **WSLg** forward la fenêtre vers le bureau Windows automatiquement
(pas de config réseau, pas de PowerShell).

### Bloc [3] — géométrie (`World.gd`)

Charge un monde JSON (**pas** de scène modelée à la main — format éditable
en donnée, pour pouvoir fabriquer facilement des mondes adversariaux dans
une V2 future : impasses, passages étroits, obstacles en U/concavités — les
minima locaux qui cassent un évitement réactif). Raycast 360° (`raycast_360()`,
LD06) et un rayon vertical seul (`raycast_down()`, TF Luna) depuis une pose
**et une attitude complète** (roll/pitch/yaw, pas juste le cap) : les deux
capteurs sont montés rigides sur le châssis, leur mesure suit toute
l'attitude du drone (`World.gd::body_to_ned()`, rotation aéronautique
standard `Rz(yaw)·Ry(pitch)·Rx(roll)`). Convention de repère : NED → Godot
via `World.ned_to_godot()`/`ned_dir_to_godot()` (est=X, haut=Y, nord=-Z).
Angle 0 du raycast 360° = axe "nez" en repère **corps**, sens horaire — même
convention que `distance_mm[360]` côté `UdpLidarDriver`/`lidar_wire.py`.
Résultats vérifiés contre la géométrie attendue à la main (yaw, roll, pose
au-dessus d'un obstacle connu), pas juste "ça tourne sans planter".

Distances retournées en **mètres bruts** (pas mm/quantifié, pas de bruit) —
la quantification/le bruit sont la responsabilité du bloc [4] (`sensor_
bridge.py`), explicitement interdits dans `World.gd` : ce bloc a une
responsabilité unique, la géométrie, pour rester remplaçable (changement de
moteur 3D sans impact aval).

`world_engine/LidarServer.gd` : serveur headless persistant, une instance
par run (lancée par `procman.py`, tuée à l'arrêt comme ArduPilot). Charge le
monde passé en `--world=<chemin absolu>` au démarrage, répond en UDP
(port 15102 par défaut) aux requêtes de raycast — protocole privé
requête/réponse synchrone documenté dans l'en-tête du fichier et mirroré
côté Python dans `raycast_wire.py`. `--headless` **volontaire** : pas de
rendu, pour rester déterministe sur le chemin critique du raycast — un
stutter de frame ressemblerait à une panne capteur si le rendu partageait
le même chemin ; le rendu reste découplé (cf. viewer plus bas).

### Bloc [4] — passthrough "SITL nu" (`sensor_bridge.py`)

Une seule lecture `SIM_STATE` (fusionnée GPS+LD06+TF Luna, remplace
l'ancien `gps_bridge.py`), interroge `LidarServer.gd` avec la pose/attitude
courante :

- **LD06** : requête `LIDAR360` à 10Hz (cadence native du capteur réel) →
  `lidar_wire.py` → `UdpLidarDriver`.
- **TF Luna** : requête `TFLUNA` à 100Hz (cadence usine par défaut du
  capteur réel, pas une limite artificielle) → message MAVLink
  `DISTANCE_SENSOR` → ArduPilot SITL lui-même (**pas** `UdpLidarDriver` —
  routage non symétrique, cf. tableau de routage capteurs et tableau des
  ports §2 ci-dessus).

**⚠️ Position NED locale — deux limites connues et documentées en dur dans
`sensor_bridge.py`** (`sim_state_pos_ned()`) :
1. `SIM_STATE` ne donne que lat/lon/alt absolus, jamais de position locale
   → conversion vers une **origine fixe et codée en dur** (le home par
   défaut d'ArduPilot SITL/Canberra). Faux si un scénario démarre le SITL
   ailleurs — pas généralisé (capture dynamique de `GPS_GLOBAL_ORIGIN`)
   tant qu'aucun scénario n'en a besoin.
2. Conversion lat/lon → NED par **approximation terre-plate** (pas de vraie
   géodésie ellipsoïdale WGS84) — négligeable à l'échelle d'un monde de
   test (`MAX_RANGE_M = 30`), **pas valable au-delà de quelques km**. Même
   simplification qu'ArduPilot en interne (`LOCATION_SCALING_FACTOR`), donc
   cohérente avec le modèle de référence — mais à remplacer par une vraie
   transformation ellipsoïdale (pyproj/GeographicLib) avant de réutiliser
   cette base pour un projet longue portée (UAV longue distance, fusée).

Distances converties en mm/intensité constante (`128` si touché, `0` sinon
— pas de notion de matériau/réflectivité dans un raycast géométrique pur)
juste avant l'encodage `lidar_wire`, jamais dans `World.gd`.

`RNGFND1_TYPE=10` (et le reste des params §5.3) sont maintenant passés à
ArduPilot via `sitl/rangefinder.parm`/`--add-param-file` (cf. §4) — avant
ce correctif, `DISTANCE_SENSOR` était envoyé mais **ignoré en silence**
par ArduPilot (aucun backend rangefinder configuré pour l'écouter, aucun
effet sur l'EKF/altitude simulée). Pas encore revérifié bout en bout après
ce correctif précis (à confirmer sur un prochain run).

### Bugs réels trouvés et corrigés en session (boucle lidar → SITL réel)

Trouvés en observant de vrais runs (dashboard API + logs), pas par
inspection statique seule — utiles à connaître si un symptôme similaire
revient :

1. **Faux `hold_obstacle` (`NVA004`) en pleine montée** — `NAV_CONV::
   travelDirection()` (`Navigation/Conversions.hpp`) utilisait la norme 3D
   complète de la vitesse (`state.velNed.norm()`, dominée par la vitesse
   *verticale* en montée) pour décider si la direction "avant" était
   fiable. Résultat : un vecteur "avant" quasi nul mais traité comme
   unitaire, qui faisait paraître des obstacles réels à plusieurs mètres
   comme s'ils étaient à quelques centimètres dans la boîte de sécurité
   2m×1m (`obstacleInBox()`). Corrigé : norme **horizontale seule**.
2. **`nav_stale_while_sequencing` (`MCA005`) au premier segment engagé,
   deux correctifs successifs** — `APTask` (Nav) ne publie de
   `NavCommand` que si un segment a été engagé au moins une fois (exprès,
   pour ne pas gêner le `NAV_TAKEOFF` d'ArduPilot). Premier correctif
   (délai de grâce basé sur l'instant d'armement) : marchait "des fois
   oui des fois non" (course contre un seuil fixe). Deuxième correctif
   (confirmation positive via `SegmentProgress` avant de juger la
   fraîcheur) : toujours faux, car `NavCommand` **n'est jamais `nullopt`
   en pratique** — `APTask` en publie déjà un pendant la phase `Hold`
   (EKF pas encore convergé, avant même l'armement), et ce vieux
   timestamp reste en shm sans être rafraîchi entre la fin du Hold et le
   premier engagement réel. Correctif final : vérifier la confirmation
   d'engagement **avant** de regarder l'âge de `NavCommand`, peu importe
   qu'il existe déjà. Diagnostiqué via des prints temporaires directement
   dans `MiSequencerTask::loop()` (retirés après coup).
3. **`MissionState::Emergency` ne sort jamais tout seul** (pas un bug —
   comportement voulu, mais source de confusion) : même une fois la
   cause disparue (`!recoverableFailureActive`), il reste en `Emergency`
   et repose juste la question au GCS — jamais de retour auto. Un
   `NVA004`/`MCA005` transitoire suffit donc à bloquer toute la mission
   jusqu'au timeout d'urgence (atterrissage auto), même si la cause
   initiale s'est résolue en une fraction de seconde. Aucun répondant
   GCS automatique dans les scénarios actuels (cf. "pas encore fait").
4. **`build_lidar_frame()` (`sensor_bridge.py`) plantait sur NaN/inf** —
   `round(d * 1000.0)` lève `ValueError`/`OverflowError` sur une distance
   non finie (géométrie dégénérée plausible tout près d'une surface
   d'obstacle), plantant tout `sensor_bridge.py` sans message clair —
   symptôme observé : le lidar (`PROD_LIDAR`) cesse de se mettre à jour
   **définitivement**, à peu près quand le drone s'approche d'un
   obstacle. Corrigé (valeurs non finies traitées comme "rien détecté").
   **Pas encore confirmé comme LA cause exacte du gel observé** — prints
   `DEBUG-LIDAR` laissés en place dans `sensor_bridge.py` en attendant
   une reproduction propre.
5. **`HoldSegment` n'engage jamais Nav** (trou d'architecture identifié,
   pas corrigé) — `MiSequencerTask` démarre juste un minuteur local
   (`inHold_`/`holdUntil_`) pour un segment `Hold`, sans jamais appeler
   `setArmedSegment()` comme pour un `Waypoint`. Pendant tout le Hold,
   Nav continue de recalculer une consigne sur le **dernier Waypoint
   atteint** — fonctionne "par coïncidence" (le drone est déjà sur
   place), pas un vrai maintien de position explicite.

### Viewer temps réel (`world_engine/Viewer.gd`, optionnel/debug)

**Process séparé de `LidarServer.gd`, jamais lancé par `procman.py`** — même
raison que plus haut : jamais de rendu sur le chemin critique du raycast.
Charge le
même format de monde, affiche un marqueur (petit cube + "nez" rouge en
avant) qui suit la pose/attitude complète du drone en direct, reçue en
UDP "fire-and-forget" (pas de requête/réponse, contrairement à
`LidarServer.gd` — cf. `raycast_wire.py::encode_pose()`).

**Le plus simple : lancement automatique par `run.sh`** (un seul terminal,
un seul process Godot en plus, tué proprement à l'arrêt comme les autres) :

```bash
sitl/run.sh --godot-editor ~/godot3-editor/Godot_v3.6-stable_x11.64 \
            --godot-headless ~/godot3/Godot_v3.6-stable_linux_headless.64 \
            --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py \
            sitl/scenarios/004_lidar_course.yaml
```

`--godot-editor <chemin>` (ou `$GODOT_EDITOR_BIN`) lance `Viewer.gd` avec
le monde du scénario et branche `--viewer` automatiquement — ignoré si
`--viewer host:port` est déjà fourni explicitement (viewer supposé déjà
lancé à la main, cas ci-dessous). `--pose-port` pour changer le port
(15103 par défaut, distinct du 15102 de `LidarServer.gd`).

**Ou à la main, dans un terminal séparé** (utile pour redémarrer le viewer
sans relancer tout le SITL) :

```bash
# ⚠️ --world DOIT être un chemin ABSOLU : --path world_engine fait chdir
# Godot dans ce dossier, un chemin relatif s'y résout et ne trouve rien
# (err=7, ERR_FILE_NOT_FOUND). Scène explicite obligatoire aussi --
# project.godot pointe par défaut sur WorldViewer.tscn (visu statique).
~/godot3-editor/Godot_v3.6-stable_x11.64 --path world_engine res://Viewer.tscn -- \
    --world="$HOME/droneMk2/world_engine/worlds/course.json" --pose-port=15103

# puis, dans un autre terminal :
sitl/run.sh --viewer 127.0.0.1:15103 --godot-headless ~/godot3/Godot_v3.6-stable_linux_headless.64 \
            --sim-vehicle ~/ardupilot/Tools/autotest/sim_vehicle.py \
            sitl/scenarios/004_lidar_course.yaml
```

Si le viewer n'est pas lancé (`--viewer`/`--godot-editor` absents),
`sensor_bridge.py` n'émet rien vers ce canal — aucun coût, aucune
dépendance.

**⚠️ Piège `look_at()` trouvé en session** (écran gris malgré une scène
correcte) : `Camera.look_at()` de Godot **hérite** de la base existante du
nœud au lieu de toujours repartir propre — si la transform statique d'une
caméra dans un `.tscn` a un déterminant négatif (miroir, ex: manipulation
accidentelle du gizmo dans l'éditeur), `look_at()` garde ce miroir, et
`cull_mode Back` (défaut de `SpatialMaterial`) élimine alors toutes les
faces avant → écran gris, quel que soit l'angle visé. `Viewer.gd::_ready()`
fait `camera.transform = Transform.IDENTITY` avant chaque `look_at()` pour
ne jamais en dépendre — même précaution prise dans
`ThirdPersonCamera.gd::_process()` (reconstruction complète chaque frame).

**Navigation caméra libre** (`world_engine/FreeCamera.gd`) : clic droit
maintenu + souris pour regarder autour, ZQSD pour se déplacer, espace/ctrl
pour monter/descendre, shift pour accélérer — pas de capture souris
permanente, utilisable au pavé tactile.

**Touche C : cycle entre 3 vues** — libre → embarquée (`DroneCam`) →
suivi 3ᵉ personne (`ThirdPersonCam`) → libre... `make_current()` gère
l'exclusivité automatiquement.

**Vue embarquée** (`DroneCam`, enfant de `DroneMarker`, position/angle de
la vraie caméra physique du drone) : constantes `DRONE_CAM_OFFSET`
(mètres, repère local `DroneMarker` : x=avant, y=haut, z=troisième axe) et
`DRONE_CAM_PITCH_DEG` en tête de `Viewer.gd`.

**Vue suivi 3ᵉ personne** (`world_engine/ThirdPersonCamera.gd`, nouveau) :
suit la position du drone en continu, **seule la rotation autour du drone
est gérée à la souris** (clic droit + glisser — sens orbite standard,
inversé par rapport à `FreeCamera`), molette pour zoomer (2-40m,
`DEFAULT_DISTANCE=8m`). `third_person_height_offset` (export sur `Viewer`,
0.3m par défaut) règle la hauteur du point visé au-dessus du centre du
drone — à ajuster à l'œil selon l'échelle du modèle 3D utilisé.

**Modèle 3D du drone** (remplace le marqueur cube/nez par défaut) : dépose
un `.glb` dans `world_engine/models/` (`.gitignore`, pas versionné — cf.
`README.md` du dossier pour la convention d'axes Blender/glTF à respecter
avant export), renseigne `drone_model_path` dans l'Inspecteur du nœud
`Viewer` (`Viewer.tscn`), sauvegarde la scène. Si le nez ne pointe pas
dans le bon sens une fois importé, ajuste `drone_model_yaw_offset_deg` par
pas de 90° — correctif d'axe normal, pas un signe d'échec d'import. Si le
modèle flotte au-dessus du sol : `drone_model_ground_offset_m` (mètres,
hauteur réelle du train d'atterrissage — `DroneMarker` représente le
repère IMU/CG rapporté par la simulation, pas le contact au sol, d'où
l'écart si l'origine du modèle est au niveau des pieds).

### Orchestration (`procman.py`)

`LidarServer.gd` démarré en premier (aucune dépendance MAVLink, pure
géométrie) — nécessite `--godot-headless <chemin binaire>`
(`$GODOT_HEADLESS_BIN`), sinon désactivé comme `ardupilot_sim_vehicle`.
Monde résolu depuis `scenario.monde` (`ProcessConfig.world_json`), avec
repli sur `world_engine/worlds/empty.json` en mode interactif (pas de
scénario).

### Fichiers

| Fichier | Rôle |
|---|---|
| `world_engine/project.godot` | Projet Godot minimal |
| `world_engine/World.gd` | Géométrie pure (bloc [3]) — raycast 360°/vertical, attitude complète |
| `world_engine/LidarServer.gd` | Serveur headless persistant, requête/réponse UDP (port 15102) |
| `world_engine/worlds/trivial.json` | Monde de test (une box, un cylindre) |
| `world_engine/worlds/empty.json` | Monde vide (scénarios 001/002, repli interactif) |
| `world_engine/test_raycast.gd` | Test géométrie en isolation, headless — `godot --headless --path world_engine --script test_raycast.gd` |
| `world_engine/WorldViewer.tscn` / `test_scene_load.gd` | Visu manuelle STATIQUE dans l'éditeur (sol/lumière/caméra, charge `trivial.json`) — **pas** ce qu'utilise `LidarServer.gd` en run réel |
| `world_engine/Viewer.gd` / `Viewer.tscn` / `test_viewer.gd` | Viewer temps réel optionnel (non-headless, découplé) — marqueur animé par la pose live, modèle 3D, 3 caméras, cf. ci-dessus |
| `world_engine/FreeCamera.gd` | Caméra libre (souris + ZQSD), attachée à la caméra principale de `Viewer.tscn` |
| `world_engine/ThirdPersonCamera.gd` | Vue suivi 3ᵉ personne, orbite souris + zoom molette, cf. ci-dessus |
| `world_engine/models/` | Modèles 3D du drone (`.glb`, `.gitignore` — jamais commités, cf. `README.md` du dossier) |
| `world_engine/worlds/obstacle_north.json` | Monde de test, un obstacle isolé (scénario `003_lidar_obstacle_check`) |
| `world_engine/worlds/course.json` | Monde de test, parcours rectangulaire avec 3 obstacles (scénario `004_lidar_course`, cf. plus bas) |
| `sitl/raycast_wire.py` | Codec des deux protocoles privés : `sensor_bridge.py` ↔ `LidarServer.gd` (requête/réponse) ET `sensor_bridge.py` → `Viewer.gd` (pose, fire-and-forget) |
| `sitl/sensor_bridge.py` | Bloc [4] passthrough — GPS+LD06+TF Luna fusionnés (remplace `gps_bridge.py`) |

### Scénario `004_lidar_course` (validation "en conditions")

Parcours rectangulaire réel (4 `Waypoint`, un virage à chaque coin, 20m×15m
autour du point de décollage) avec 3 obstacles répartis le long du trajet
(`world_engine/worlds/course.json` — 2 "immeubles" + 1 "poteau", dégagement
5,5-6,5m vérifié à la main). Sert de scénario de référence pour tester la
boucle complète en conditions réelles (virages, détection lidar à
plusieurs caps, viewer/3 caméras) — c'est en le faisant tourner
plusieurs fois que les bugs listés plus haut ont été trouvés.

### Pas encore fait

- Modèle capteur bruité (bloc [4] réel : bruit gaussien paramétrable par
  capteur, biais constant + dérive lente, latence configurable par flux,
  dropout, quantification à la résolution réelle du capteur, graine de
  bruit fixée et versionnée par scénario) — `sensor_bridge.py` reste un
  passthrough "SITL nu" (GPS idéal fixe, distances géométriques brutes sans
  bruit).
- Détection de collision géométrique (le drone traverse les obstacles sans
  qu'aucune couche ne le détecte comme un échec de test) — hors périmètre
  de la boucle lidar, pas commencé.
- Formes de monde au-delà de `box`/`cylinder` (`World.gd::load_from_json()`).
- **Vrai maintien de position pendant un `Hold`** — cf. bug #5 ci-dessus
  (`HoldSegment` n'engage jamais Nav).
- **Réponse GCS automatique pendant un scénario** — aucun scénario ne
  répond aux questions `Hold`/`Emergency` (`queryAnswer`), donc tout
  `hold_obstacle`/`nav_stale_while_sequencing`, même transitoire et
  légitime, bloque la mission jusqu'au timeout d'urgence. Vraie décision
  d'archi à trancher (pas encore fait) : soit garder les mondes de test
  assez dégagés pour ne jamais déclencher ces codes, soit apprendre à
  `mission_driver.py`/`run_scenario.py` à répondre automatiquement.
- **Gel définitif du lidar près d'un obstacle** — symptôme observé
  plusieurs fois, cause probable identifiée (NaN/inf non protégé dans
  `build_lidar_frame()`, cf. bug #4 ci-dessus) et corrigée, mais **pas
  encore confirmée** par une reproduction propre avec les prints
  `DEBUG-LIDAR` en place. Si ça revient : chercher ces lignes dans
  `sitl/logs/sensor_bridge.log`.

---

## 10. Ce qui n'est PAS encore fait (hors moteur 3D, cf. §9)

- **Rejeu accéléré** (bloc [6], étape 5) — `SimClock` pose l'infrastructure
  mais aucun outil ne pilote encore le temps explicitement.
- **Suivi de scénario dans le dashboard** — cf. §6.
- **Étapes `hold`/`resume`/`answer`** dans `run_scenario.py` — cf. §3.
- **Injection de panne système différée** (`SharedWatchdogControl`) — pas
  encore de wrapper Python, cf. §5.

---

## 11. Contraintes structurantes

**RTF 1 obligatoire.** Les composants sont des process RT qui lisent
`Clock::now()` en temps mur — un `--speedup` de SITL casserait la
cohérence (Nav tournerait à sa fréquence réelle pendant que le monde
avancerait N fois plus vite). Conséquence acceptée : le circuit complet
tourne en temps réel, une suite de scénarios prend le temps qu'elle prend.

**Abstraction d'horloge posée, pas encore exploitée.** `TYPES::Clock`
devient `SimClock` avec `DRONE_USE_SIM_CLOCK` (cf. §7) — pour l'instant un
simple relais vers `steady_clock`, sans pilotage explicite du temps. Pose
l'infrastructure pour un futur rejeu accéléré (§10) sans avoir à re-router
`Clock::now()` partout plus tard.

**Reproductibilité du verdict, pas du run bit-exact.** Plusieurs process RT
avec ordonnancement variable interdisent un rejeu bit-exact. Les critères de
`verdict.py` (§3, "critères disponibles") portent donc sur des enveloppes
(séquence d'états, plages, présence/absence), jamais sur des trajectoires
exactes — une assertion sur des flottants comparés à l'identique serait
rouge une fois sur trois et finirait ignorée.

**Bindings Python interdits sur le code C++.** L'orchestrateur ne parle au
système que par ses interfaces externes (MAVLink, UDP capteur, lecture shm
directe via `mmap`+`struct`, jamais de couplage au code C++ lui-même) — cf.
§0, principe de validation boîte noire.

## 12. Binaires de test intermédiaires (SITL et HIL)

Entre les tests unitaires GTest et le SITL complet (`drone` + `sim_vehicle.py`
+ moteur 3D), plusieurs binaires isolent un sous-ensemble de composants pour
valider un maillon sans monter tout l'appareillage. Tous compilés uniquement
avec `-DBUILD_ITESTS=ON` (cf. §7), tous dans `src/IntegrationTests/`, tous
avec leurs propres chemins shm dédiés (`include/drone/IntegrationTests/
ShmPaths.hpp`, namespaces `SITL::`/`HIL::`/`NAV_SITL::`/`MC_SITL::`) pour ne
jamais entrer en collision avec un vrai `drone` (namespace `PROD`, chemins
publiés par `GlobalWatchdog`) qui tournerait en même temps sur la même
machine.

| Binaire | Composants réels | Face à | Remplace |
|---|---|---|---|
| `mv_sitl` | `MavlinkInterface` seule | ArduPilot SITL (UDP) | `GlobalWatchdog` + les 4 autres composants |
| `nav_sitl` | `Navigation` + `MavlinkInterface` | ArduPilot SITL (UDP) | `SensorFusion` (relais minimal `DynamicState`, pas de vrai GPS/lidar) |
| `mc_sitl` | `MissionControl` seul | un plan de mission prédéfini + faux `Nav` (progression par délai fixe) | tout le reste — teste la state machine/séquenceur/failsafe, pas le pilotage |
| `sysmon_sitl` | `SysMonitoring` seul | données shm injectées à la main (port UDP texte 15020) | tous les autres composants — sert à observer le flux télémétrie réel sur le GCS sans faire tourner le système complet |
| `hil_sf_mv` | `SensorFusion` + `MavlinkInterface` | **vrai matériel** : GPS/LIDAR sur `/dev/ttyAMA1`/`/dev/ttyAMA2`, FC réel sur `/dev/ttyAMA0` | `GlobalWatchdog` + les 3 autres composants — seul binaire de ce tableau qui touche du matériel physique, pas de SITL ArduPilot |
| `udp_bridge` | (aucun — outil de pont) | s'attache en lecture seule aux segments shm de n'importe lequel des binaires ci-dessus, ou aux vrais chemins `PROD` | — relaie tout en UDP texte vers `dashboard_server.py`, réessaie périodiquement si un binaire démarre après lui, plusieurs sources actives en même temps possible |

`nav_sitl` est le palier "Nav sur SITL nu" (roadmap étape 3, §8) : valide le
guidage réel sans monter le moteur 3D ni le modèle capteur. `hil_sf_mv` est
le seul binaire de ce tableau en **hardware-in-the-loop** au sens strict —
capteurs et FC réels, aucune simulation — utilisé pour valider la chaîne
capteur/MAVLink indépendamment du reste du système avant intégration
complète. Pas de magnétomètre dans `hil_sf_mv` : câblé directement sur le
FC, hors du chemin `SensorFusion` (cf. `SENSORFUSION.md`).

---

## 13. Références

- `tests/test_global_watchdog.cpp::RespawnSuppressedWhenFlagSet` — preuve
  que le mécanisme de panne système différée fonctionne.
