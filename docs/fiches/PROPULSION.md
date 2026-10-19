# Motorisation / propulsion

Rôle : sustentation et contrôle de poussée. Pas d'équivalent aviation
naturel pour un quadricoptère (pas de "moteur" unique façon aile fixe) —
nom brut, pas d'acronyme inventé.

| | |
|---|---|
| **Composition matérielle** | Quadricoptère, châssis 8 pouces (style FPV freestyle). Hélices tripales 6045 (6" diamètre, pas 4.5"). Moteurs **iFlight XING-E Pro 2207**, 1800KV, 2-6S. ESC **Flywoo GOKU G45M**, 45A, firmware AM32, footprint 20×20mm. Batterie **LiPo 6S, 4300mAh**. |
| **Interface logicielle** | Télémétrie lue via MAVLink `ESC_STATUS`/`ESC_TELEMETRY_1_TO_4`, parsée dans `src/MavlinkInterface/MavlinkInterface.cpp` (RPM, courant, température par moteur). |
| **État actuel** | Matériel monté et câblé (châssis + élec terminés). Seuils RPM `kMinArmedRpm=1000`/`kMaxArmedRpm=10000` (`MavlinkInterface.cpp`) explicitement marqués "valeurs de départ arbitraires, à calibrer sur banc" — **pas encore calées sur ce moteur précis** (1800KV en 6S peut largement dépasser 10000 tr/min à pleine charge selon l'hélice, à vérifier avant calibration). |
| **Redondance / sécurité** | Aucune redondance moteur (quadricoptère standard) — la sécurité repose sur la détection de RPM hors plage plutôt que sur un moteur de secours. |

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../img/propulsion-chassis-dark.svg">
  <img alt="Motorisation et disposition châssis" src="../img/propulsion-chassis-light.svg">
</picture>

Doc détaillée : `HARDWARE.md` §8.
