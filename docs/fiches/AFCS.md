# AFCS — Automatic Flight Control System

Rôle : stabilisation d'attitude et contrôle moteur bas niveau. Le Pi 5 ne
fait jamais de contrôle bas niveau directement — il commande l'AFCS via
MAVLink, qui exécute.

| | |
|---|---|
| **Composition matérielle** | Flight controller **Sequre**, sous firmware **ArduPilot**. |
| **Interface logicielle** | `MavlinkInterface` (`src/MavlinkInterface/MavlinkInterface.cpp`), communication **MAVLink v2** sur UART. Liaison abstraite derrière `IMavlinkLink` : `UartMavlinkLink` en prod, `UdpMavlinkLink` en SITL — permet de tester tout le composant sans matériel FC réel. Baudrate ~920000 (valeur standard la plus proche : 921600, **à confirmer précisément**). |
| **État actuel** | Implémenté et testé en SITL (cf. `README.md`, `sitl/SITL.md`). **Le port UART exact côté Pi (`/dev/ttyAMAx`) n'est pas documenté dans le code** — contrairement au GPS/LD06 qui ont un port par défaut explicite dans leur driver, écart à combler dans la doc. |
| **Redondance / sécurité** | Perte de heartbeat MAVLink détectée par `MavlinkInterface`, remonte au `ComponentWatchdog`/`GlobalWatchdog` (cf. fiche CBIT). `MissionControl` ne fait jamais de cold-start — sa state machine ne peut pas être resetée en sécurité. |

Pas d'illustration disponible pour l'instant (seules des photos existent, pas raccord avec le reste de la DA en tracé/rendu).

Doc détaillée : `HARDWARE.md` §7, `include/drone/Components/MavlinkInterface/MAVLINKINTERFACE.md`.
