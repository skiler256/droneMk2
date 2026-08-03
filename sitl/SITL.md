# SITL — plateforme de test système

Documentation de référence de tout ce qui vit sous `sitl/`. Complète
`INTEGRATION_TEST.md` (la spécification/roadmap) — ce document-ci explique
ce qui est **construit et qui tourne**, pas ce qui est prévu.

Trois blocs, comme discuté :

```
binaire (drone) <--UDP--> SITL_MASTER (sitl/, Python) <--MAVLink--> SITL_ardupilot
```

- **binaire** : le vrai `drone` (8 process, `GlobalWatchdog` + 5 composants),
  compilé avec des drivers UDP factices à la place du vrai matériel I2C/UART.
  Aucune logique métier modifiée pour les besoins du test (cf.
  `INTEGRATION_TEST.md` §1.3, "validation boîte noire").
- **SITL_MASTER** : les scripts Python sous `sitl/` — orchestre le run,
  relaie le GPS/lidar simulés, pilote la mission via de vraies commandes
  GCS, observe l'état système, calcule un verdict.
- **SITL_ardupilot** : `sim_vehicle.py` d'ArduPilot — fournit la dynamique
  de vol, l'EKF3, le compas simulé. Pas installé dans cet environnement de
  dev — à faire tourner sur ta machine (WSL).

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
```

`sitl/run.sh` construit (`build_sitl/`, séparé du build de prod — jamais
les mêmes drivers), crée le venv Python (`sitl/.venv/`) si besoin, démarre
tout, et nettoie proprement à la sortie (Ctrl+C ou fin de scénario).

**Prérequis à poser toi-même** (pas construits dans ce fil) : cloner et
compiler ArduPilot (`ArduPilot/ardupilot`, `Tools/autotest/sim_vehicle.py`)
— cf. `INTEGRATION_TEST.md` §13. Tout doit tourner sur la même machine
(WSL), tout communique en UDP loopback (cf. §8.1 — RTF 1 obligatoire, un
seul domaine de temps cohérent).

---

## 2. Flux de données et ports

```
sim_vehicle.py (ArduPilot SITL)
   │  MAVLink UDP, port 14550
   ▼
drone (MavlinkInterface, UdpMavlinkLink)
   │  shm interne (mêmes chemins qu'en vol : /drone_nav, /drone_sf, ...)
   │
   ├── SensorFusion  ◄── UdpGpsDriver (port 15100) ◄── gps_bridge.py ◄── SIM_STATE (ArduPilot)
   │                 ◄── UdpLidarDriver (port 15101) ◄── gps_bridge.py (flux "rien détecté" constant)
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
| 14551 | `gps_bridge.py` (cible, pas écoute) | Flux MAVLink SIM_STATE depuis SITL (`--sim-udp`) |
| 15100 | `drone` (SensorFusion, `UdpGpsDriver`) | GPS simulé |
| 15101 | `drone` (SensorFusion, `UdpLidarDriver`) | Lidar simulé |
| 5601 | `drone` (SysMonitoring, TelMain) | Commandes GCS montantes (`CMD_ARM` etc.) |
| 5602 | (rien pour l'instant) | Télémétrie descendante — pas consommée par l'orchestrateur |
| 15001 | `dashboard_server.py` | Statut shm depuis `udp_bridge` |
| 8766 | `dashboard_server.py` | Page web + API JSON |

---

## 3. Créer un scénario

Format YAML (`INTEGRATION_TEST.md` §6, allégé — pas encore de `monde`/
`modele_capteur`, cf. §7 plus bas) :

```yaml
id: 001_arm_takeoff_wait_land
description: >
  Texte libre.

mission:
  - {type: arm}
  - {type: takeoff, alt: 10}
  - {type: wait, duration: 5}
  - {type: land}

criteres:
  - {metrique: mission_state_sequence, attendu: [Armed, Mission, Landing]}
  - {metrique: aucun_code_emergency, attendu: true}
  - {metrique: processus_vivants_a_la_fin, attendu: [drone, udp_bridge]}
```

### Étapes de mission supportées (`sitl/run_scenario.py::run_step`)

| type | params | effet |
|---|---|---|
| `arm` | — | `CMD_ARM`, attend `mission_state == Armed` |
| `takeoff` | `alt` (m) | `CMD_TAKEOFF`, attend `mission_state == Mission` |
| `wait` | `duration` (s) | n'envoie rien, observe pendant N secondes |
| `land` | — | `CMD_LAND`, attend `mission_state == Landing` |

Pas encore câblés : `hold`/`resume` (`CMD_HOLD`/`CMD_RESUME` existent déjà
côté `mission_driver.MissionDriver`, juste pas exposés comme type d'étape),
`answer` (répondre à une `MC_QUERY`), segments de vol intermédiaires
(attendent Nav validé sur SITL nu, roadmap étape 3). Ajouter un type
d'étape = un `elif` dans `run_step()`, rien d'autre.

### Critères disponibles (`sitl/verdict.py::evaluate`)

| métrique | `attendu` | vérifie |
|---|---|---|
| `mission_state_sequence` | liste d'états | vue comme sous-séquence (pas forcément consécutive) dans les états observés |
| `aucun_code_emergency` | `true`/`false` | aucun code de sévérité `Emergency` (`INDEX.csv`) vu pendant le run |
| `processus_vivants_a_la_fin` | liste de noms process | chacun encore vivant juste avant l'arrêt volontaire |

Métrique inconnue = échec explicite (pas ignorée silencieusement).

---

## 4. Les modules

| Fichier | Rôle |
|---|---|
| `scenario.py` | Parsing YAML → `Scenario`/`MissionStep`/`Criterion` |
| `procman.py` | Cycle de vie des process (`dashboard`, `sim_vehicle`, `drone`, `udp_bridge`, `gps_bridge`) — `with ProcessManager(cfg) as pm:`, arrêt garanti même sur exception |
| `shm_observer.py` | Interroge `dashboard_server.py` (HTTP) en tâche de fond, expose `mission_state()`/`active_codes()`/`wait_for_mission_state()` |
| `mission_driver.py` | Envoie les commandes GCS via le vrai canal `TELEM::*` (réutilise `tools/dashboard/telemetry_codec.py`) + heartbeat GCS 1Hz automatique |
| `verdict.py` | Évaluation pure des critères à partir d'une `RunTrace` (même esprit que `MC_CONV::step()` côté C++ — logique testable sans rien lancer) |
| `gps_wire.py` / `lidar_wire.py` | Codec du format UDP GPS/lidar simulé (miroir Python de `UdpGpsDriver`/`UdpLidarDriver` C++) |
| `gps_bridge.py` | Lit `SIM_STATE` d'ArduPilot SITL, relaie en GPS UDP (~10Hz) + lidar "rien détecté" constant |
| `run_scenario.py` | Point d'entrée : charge le scénario, orchestre tout, imprime le verdict |
| `run.sh` | Wrapper shell : build + venv + délègue à `run_scenario.py` |

---

## 5. Injection de pannes

Deux domaines distincts (`INTEGRATION_TEST.md` §7 — ne pas mélanger) :

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

Pas encore construit : dropout/bruit/latence réalistes sur le GPS/lidar
simulés eux-mêmes (bloc [4], "modèle capteur", roadmap étape 4) — pour
l'instant `gps_bridge.py` relaie un fix parfait fixe (cf. §7).

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
  (§8 ci-dessous). Sans effet observable aujourd'hui, mais nécessaire pour
  ne pas avoir à re-router `Clock::now()` partout plus tard.

Ces deux options sont **globales** (pas par cible) : jamais de mélange de
politique horloge/drivers entre unités de traduction d'un même binaire.

---

## 8. Ce qui n'est PAS encore fait

- **Moteur 3D** (`INTEGRATION_TEST.md` bloc [3]) — roadmap étape 4,
  volontairement après l'orchestrateur ("l'orchestrateur ne se rétrofitte
  pas"). Le lidar simulé envoie "rien détecté" en continu jusque-là.
- **Modèle capteur bruité** (bloc [4]) — même étape 4. `gps_bridge.py`
  envoie un fix idéal fixe (`fixType=3`, `hdop=1.0`, pas de bruit/dropout).
- **Rejeu accéléré** (bloc [6], étape 5) — `SimClock` pose l'infrastructure
  mais aucun outil ne pilote encore le temps explicitement.
- **`gps_bridge.py` jamais testé contre un vrai `sim_vehicle.py`** —
  écrit contre les champs MAVLink `SIM_STATE` documentés, testé avec un
  faux message Python, mais pas de vraie instance ArduPilot disponible
  dans cet environnement de dev. À vérifier sur ta machine en premier.
- **Suivi de scénario dans le dashboard** — cf. §6.
- **Étapes `hold`/`resume`/`answer`** dans `run_scenario.py` — cf. §3.

---

## 9. Références

- `INTEGRATION_TEST.md` — spécification/roadmap complète (blocs, formats,
  contraintes RTF1, pièges connus).
- `tests/test_global_watchdog.cpp::RespawnSuppressedWhenFlagSet` — preuve
  que le mécanisme de panne système différée fonctionne.
