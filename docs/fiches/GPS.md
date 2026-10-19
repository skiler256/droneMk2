# GPS — positionnement

Rôle : position, vitesse et temps de référence. Terme déjà standard en
aviation, pas d'acronyme dédié inventé.

| | |
|---|---|
| **Composition matérielle** | Module **NEO-M8N**, antenne patch intégrée, montée sur l'arrière-haut de la coque. |
| **Interface logicielle** | Driver `src/Drivers/NEO-M8N.cpp`, UART `/dev/ttyAMA1` par défaut. Démarre toujours à 9600 bauds (défaut du module, pas persisté en EEPROM) — le driver envoie une trame UBX-CFG-PRT figée pour basculer à 115200 puis rouvre le port. Parseur binaire UBX : `NAV-STATUS`, `NAV-PVT`, `NAV-DOP`, `NAV-POSLLH`, `NAV-TIMEUTC`, `NAV-SAT`. |
| **État actuel** | Implémenté (`src/Drivers/NEO-M8N.cpp` présent, pas un squelette). |
| **Redondance / sécurité** | Capteur unique, pas de secours GPS identifié à ce stade. |

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../img/gps-antenne-dark.svg">
  <img alt="Position GPS et antenne" src="../img/gps-antenne-light.svg">
</picture>

Doc détaillée : `HARDWARE.md` §5.
