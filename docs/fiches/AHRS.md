# AHRS — Attitude and Heading Reference System

Rôle : détermination de l'attitude (roulis/tangage) et du cap, seule
source d'attitude utilisée par le logiciel de vol.

| | |
|---|---|
| **Composition matérielle** | **MPU6050** (IMU accéléro+gyro), puce montée directement sur le PCB du **FC** — jamais exposée au Pi. **LIS3MDL** (magnétomètre), I2C, adresse `0x1C` — driver Pi existant mais **plus câblé sur le Pi**. |
| **Interface logicielle** | L'attitude vient exclusivement du **FC via MAVLink** (`SharedFCStatus`, `include/drone/Components/MavlinkInterface/`) — le Pi ne calcule pas de fusion à partir de l'IMU brute. `SensorFusion` (`src/SensorsFusion/SensorFusion.cpp`) consomme cette attitude déjà fusionnée côté FC (EKF3 ArduPilot), republie dans `SharedSFMem`, et vérifie sa fraîcheur (`kAgeThrEKF=1000ms`). |
| **État actuel** | Chaîne FC → MAVLink → `SensorFusion` fonctionnelle. `LIS3MDL.hpp` (`include/drone/Components/Drivers/`) existe mais n'est **plus câblé** : décision actée de câbler le magnétomètre directement sur le FC pour la santé de l'EKF3 plutôt que sur le Pi — **`GPS_INPUT.yaw` reste à 0 en permanence côté logiciel**, le FC utilise son propre compas. Fichier driver conservé (registres figés, "NE PAS MODIFIER") au cas où un magnétomètre externe serait un jour remonté côté Pi. |
| **Redondance / sécurité** | `TFS` (Navigation, cf. fiche OCAS) vérifie indépendamment que `SensorFusion` publie toujours un état frais (`kEkfStaleThreshold=400ms`) — check distinct de celui de `SensorFusion` sur les messages MAVLink du FC, pensé pour couvrir un `SensorFusion` gelé/planté que son propre check ne peut pas détecter sur lui-même. |

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../img/imu-magneto-dark.svg">
  <img alt="Emplacement IMU et magnétomètre sur le FC" src="../img/imu-magneto-light.svg">
</picture>

Doc détaillée : `HARDWARE.md` §4, `include/drone/Components/Navigation/NAVIGATION.md` §6 (fraîcheur EKF côté `TFS`).
