# tools/

Générateurs, éditeurs et scripts pour les formats répétitifs du projet.
Principe commun : un CSV est la source de vérité (éditable à la main ou via
un éditeur web dédié), un générateur produit le header C++ correspondant —
jamais l'inverse, ne jamais éditer un fichier sous `include/drone/
generated/` à la main.

## 1. Ajouter un code de diagnostic

1. `python3 tools/codes_editor.py [--port 8765]` — serveur web local
   (stdlib uniquement, pas de dépendance), édite `INDEX.csv` directement
   (composant, catégorie, nom, description, criticité, colonne `Alarm`,
   colonne `Panel` — labels de voyants ESP32, cf. `GCS/README.md` §3).
   Ou éditer `INDEX.csv` à la main (`;` comme séparateur, format documenté
   en tête de `gen_codes.py`).
2. `cmake --build build/` (ou `build_sitl/`) régénère automatiquement
   `include/drone/generated/codes.hpp` (dépendance CMake sur le CSV) — pas
   besoin de lancer `gen_codes.py` à la main sauf hors build.

Le même `INDEX.csv` est lisible tel quel côté GCS Python et servi par
`GCS/server.js` (`/codes.csv`) pour le dashboard web.

## 2. Ajouter une alarme sonore

1. Même éditeur, onglet "Alarmes" (`tools/codes_editor.py`) : CRUD sur
   `ALARMS.csv` (`AlarmId;Label;SoundFile;Type;Priority;PeriodMs;Class`) et
   sur le mapping `MISSION_ALARMS.csv` (`State;AlarmId`). Un garde-fou
   empêche de supprimer une alarme encore référencée par un code ou un état
   de mission.
2. Déposer le fichier son dans `GCS/public/sounds/<SoundFile>.mp3` (ou
   `.wav`/`.mp4` — un seul format suffit).
3. Référencer l'alarme depuis un code (colonne `Alarm` d'`INDEX.csv`, étape
   1) ou laisser le repli automatique (`master_warning`/`master_caution`).

Détail du système d'alarme (priorité, préemption, acquittement) :
`GCS/README.md` §3.

## 3. Ajouter/modifier un paquet de télémétrie

Même éditeur (`tools/codes_editor.py`), édite aussi `TELEMETRY.csv`.
**Attention** : contrairement aux codes (identifiant = la chaîne elle-même,
ex. `FCP001`), le `PacketID` d'un paquet télémétrie est son **index** dans
`TELEMETRY.csv` — l'ordre des lignes fait partie du format binaire.
L'éditeur préserve toujours l'ordre existant (ajout en fin, édition en
place) ; supprimer une ligne décale les ID de tous les paquets suivants et
casse la compatibilité avec un binaire déjà déployé — à éviter après un
premier déploiement réel. `cmake --build` régénère
`include/drone/generated/telemetry.hpp` automatiquement.

## 4. Compiler un plan de mission

`MISSIONPLAN.csv` définit le header et les types de segment (répétitif,
régénère `include/drone/generated/missionplan.hpp` via `gen_missionplan.py`,
même mécanisme CMake que les deux précédents). Le **framing** complet
(magic/version/CRC32, TLV) est écrit à la main côté C++
(`MISSIONPLAN_CONV`, `MissionControl/MissionPlan.hpp`) et mirroré côté
Python dans `tools/missionplan_wire.py`.

Pour compiler un plan de mission concret (YAML → binaire lu par
`MissionControl`) :

```bash
python3 tools/mission_compiler.py plan.yaml --out plan.bin
```

Format YAML : `header:` (champs optionnels, défauts C++ sinon) +
`segments:` (liste, types `Waypoint`/`Corridor`/`Curve`/`Landing`/`Hold`/
`PayloadAction` — voir le détail des champs par type en tête de
`mission_compiler.py`/`missionplan_wire.py`). Utilisé aujourd'hui par le
harnais SITL (`sitl/scenarios/*.plan.yaml`), vise à devenir l'outil de
compilation côté GCS une fois un pipeline d'upload construit.

## 5. Lancer un dashboard de dev

`tools/dashboard/dashboard_server.py [--port 8766]` — un seul serveur pour
tous les cas de test dev (page menu sur `/`) :

| Route | Contenu |
|---|---|
| `/sysmonitoring` | Télémétrie TelMain/TelSec (UDP loopback) |
| `/sitl/mv` | État shm de `MavlinkInterface` face à ArduPilot SITL (`mv_sitl`) |
| `/hil/sf_mv` | État shm de `SensorFusion`+`MavlinkInterface` face au vrai matériel (`hil_sf_mv`) |
| `/nav/sitl` | État shm de `Navigation`+`MavlinkInterface` face au SITL, sans `SensorFusion` (`nav_sitl`) |
| `/sitl/prod` | État shm des 5 composants réels (`drone` en mode SITL) |

Démarré automatiquement par `sitl/run.sh` (cf. `sitl/SITL.md`) — pas besoin
de le lancer à la main pour un run SITL normal, seulement pour observer un
binaire de test isolé lancé manuellement (`mv_sitl`, `hil_sf_mv`, etc.).

**⚠️ Ne jamais lancer en même temps que `GCS/server.js`** : les deux sont
des implémentations indépendantes du même concept de dashboard télémétrie,
sur les **mêmes ports UDP** (5601/5602/5611/5612) — duplication
intentionnelle et temporaire tant que le matériel H562 réel n'existe pas
(cf. `GCS/README.md`), mais les lancer ensemble fait échouer le bind du
second démarré.

## 6. Scripts de debug matériel (`tools/hw_debug/`)

Voir `tools/hw_debug/README.md` — scripts autonomes de test d'un module
isolé (radio, lidar) sur banc ou en réparation, indépendants du reste du
projet.
