# droneMk2

Drone autonome à voilure tournante, développement solo. Compute embarqué
Raspberry Pi 5 (kernel PREEMPT_RT) piloté par un logiciel de vol C++23 en
isolation de process, flight controller ArduPilot en aval (stabilisation
d'attitude, contrôle moteurs), station sol (GCS) dédiée en Node.js avec
boîtier physique d'alarmes.

## État du projet

- **Matériel** : châssis et connexions électriques terminés.
- **Logiciel embarqué** : composants implémentés jusqu'à `MavlinkInterface`
  (cf. [`ARCHITECTURE_IPC.md`](ARCHITECTURE_IPC.md) et les docs par
  composant sous `include/drone/Components/*/`) ; `MissionControl`,
  `Navigation`, `SensorFusion`, `SysMonitoring` existent et sont testés en
  SITL (cf. [`sitl/SITL.md`](sitl/SITL.md)).
- **GCS** : dashboard web + boîtier ESP32 fonctionnels ; image Debian live
  bootable en cours (bloquée sur un bug de démarrage non résolu, cf.
  [`GCS/README.md`](GCS/README.md)).
- **Développement en pause** — reprise prévue ultérieurement. Cette
  documentation a été mise à jour spécifiquement pour permettre une reprise
  autonome.

## Vue d'ensemble

```
Capteurs (IMU, magnéto, GPS, lidars) ─┐
                                       ├─► drone (Pi 5, C++, process isolés)
FC ArduPilot (SpeedyBee H743) ◄───────┘        │
        ▲ MAVLink v2 / UART                    │ wfb-ng 433MHz (télémétrie)
        │                                       ▼
     moteurs                                   GCS (Node.js + boîtier ESP32)
```

`drone` est un unique binaire, un rôle par process (`fork()+exec()`, un
domaine fonctionnel = un process isolé), chaque process organisant ses
propres tâches en threads temps réel — détails dans
[`ARCHITECTURE_IPC.md`](ARCHITECTURE_IPC.md).

## Carte du dépôt

| Dossier | Contenu |
|---|---|
| `src/`, `include/drone/` | Logiciel de vol C++23 (composants, IPC, drivers) |
| `tests/` | Tests unitaires GTest |
| `sitl/` | Harnais de test SITL/HIL (Python), scénarios, orchestration |
| `world_engine/` | Moteur 3D Godot (géométrie lidar pour la simulation) |
| `GCS/` | Station sol : serveur Node.js, dashboard web, firmware ESP32, image Debian live |
| `tools/` | Générateurs/éditeurs pour les formats répétitifs (codes, alarmes, télémétrie, plan de mission), scripts de debug matériel (`tools/hw_debug/`) |
| `third_party/` | Dépendances externes (submodule MAVLink `c_library_v2`) |
| `terrain/` | Données de référence (tuile d'élévation SRTM) |
| `docs/img/` | Illustrations (vues CAO, schémas) référencées par la documentation |
| `data/` | Tables source de vérité (`ALARMS.csv`, `INDEX.csv`, `MISSIONPLAN.csv`, `MISSION_ALARMS.csv`, `TELEMETRY.csv`), éditées via `tools/codes_editor.py` |

## Documentation

| Document | Contenu |
|---|---|
| [`ARCHITECTURE_IPC.md`](ARCHITECTURE_IPC.md) | Couche IPC (shared memory), répartition process/thread, watchdogs |
| [`GUIDE_COMPOSANTS.md`](GUIDE_COMPOSANTS.md) | Comment ajouter un composant (patron de code) |
| `include/drone/Components/*/*.md` | Doc par composant (MissionControl, Navigation, SensorFusion, MavlinkInterface, SysMonitoring) |
| [`sitl/SITL.md`](sitl/SITL.md) | Plateforme de test SITL et HIL : lancement, architecture, scénarios, binaires intermédiaires, limites connues (référence à jour) |
| [`INTEGRATION_TEST.md`](INTEGRATION_TEST.md) | Spécification d'origine de la plateforme SITL/HIL — conservée pour le raisonnement et les numéros de section cités dans le code, pas la référence de ce qui tourne aujourd'hui |
| [`GCS/README.md`](GCS/README.md) | Station sol : dashboard, alarmes, firmware ESP32, image Debian live |
| [`tools/README.md`](tools/README.md) | Procédures : ajouter un code/une alarme, régénérer la télémétrie, lancer le dashboard |

## Démarrage rapide

```bash
# Build production
cmake -B build && cmake --build build -j"$(nproc)"

# Build + tests SITL (voir sitl/SITL.md pour le détail)
sitl/run.sh
```
