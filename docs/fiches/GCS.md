# GCS — Ground Control Station

Rôle : poste sol — dashboard opérateur, alarmes, relais télémétrie. Terme
déjà standard en aviation/UAV, pas d'acronyme à inventer.

| | |
|---|---|
| **Composition matérielle** | PC (dashboard web en Chromium) + boîtier **ESP32** dédié aux voyants d'alarme physiques : 2× **MCP23017** (I2C, adresses `0x20`/`0x21`, 16 voyants chacune) + 4 GPIO directs + 2 boutons physiques d'acquittement. |
| **Interface logicielle** | `GCS/server.js` : relais UDP↔WebSocket, table de codes, pilotage des LEDs ESP32, volume PulseAudio. `GCS/public/` : dashboard web (`alarm.js`/`link.js`/`volume.js`, système d'alarme sonore priorisé). `GCS/debug-injector.js` (port 3001) : injection de faux paquets CODE pour tester sans drone. Firmware ESP32 (`GCS/ESP32/`) : protocole série `LED:.../BTN:...`, généré par `gen_esp_map.py`/`mapingESP.csv`. |
| **État actuel** | Dashboard + boîtier ESP32 fonctionnels. Télémétrie toujours **émulée en UDP loopback** en attendant les liens radio réels (cf. fiche PDL/SDL). Image Debian live bootable en cours (`GCS/live-image/`) — bloquée sur un bug "BOOT FAIL" SYSLINUX non résolu, reproduit sur 3 clés USB différentes, cause racine non identifiée. |
| **Redondance / sécurité** | Alarmes classées **MW**/**MC** (Master Warning/Master Caution, façon Airbus — cf. `data/ALARMS.csv`), acquittées séparément via les boutons physiques `MW_ACK`/`MC_ACK` du boîtier ESP32. |

Pas d'illustration disponible pour l'instant.

Doc détaillée : `GCS/README.md`, `HARDWARE.md` §10.
