# SensorFusion — architecture, Tasks, pièges

Documentation de référence du composant `SensorFusion` : rôle de chaque
Task, comment une mesure capteur devient une donnée exploitable par le
reste du système, les décisions d'archi actées, et ce qui reste
délibérément différé. Complète `GUIDE_COMPOSANTS.md` (générique) et le
SDD (`Document de Conception Logicielle.pdf`, §3.3).

**État** : implémenté (GPS, EKF/HomeCalibrator, LIDAR), staleness et
codes de faute couverts pour les 3 capteurs/sources. Testé sur le vrai
Pi (I2C activé, `hil_sf_mv`), pas encore validé en vol complet.
`HomeCalibrator` réel laissé en constante neutre, à brancher plus tard.
**Pas de magnétomètre sur le Pi** : câblé directement sur le FC (§3.5),
bascule d'archi actée en session — l'EKF3 du FC utilise son propre
compas, `GPS_INPUT.yaw` part systématiquement à 0 ("inconnu").

## 1. Vue d'ensemble

```
NEO_M8N (UART) ────► TGPS   ──► SharedSFMem.gpsMag  ──► MavlinkInterface (GPS_INPUT, yaw=0)

SharedFCStatus ───► TEKF   ──► SharedSFMem.state    ──► Navigation, TLidar (attitude)
(écrit par MavlinkInterface,
 attitude + LOCAL_POSITION_NED du FC, magnétomètre y est câblé)

LD06 (UART) ──────► TLidar ──► SharedSFMem.lidar     ──► MissionControl (évitement, futur)
                    (utilise SharedSFMem.state.attitude, donc dépend de TEKF)
```

Les drivers (`NEO_M8N`, `LD06`) sont injectés au composant par référence
sur l'interface générique `IDriver<T>` (`Driver.hpp`), jamais un type
concret dans `SensorFusions` lui-même — même principe que `IMavlinkLink`
côté `MavlinkInterface`, substituable en SITL/HIL sans changement de
code métier.

**Écart avec le SDD** : le diagramme de classe (§3.3.2) prévoit une
interface nommée par capteur (`IGPSDriver`, `IMagDriver`, `ILIDARDriver`)
au-dessus des classes concrètes. Le code retient `IDriver<T>` générique à
la place — un seul patron, pas de prolifération d'interfaces quasi
identiques, même garantie de substituabilité. Acté en session, le SDD
reste à corriger sur ce point (et sur le magnétomètre lui-même, cf. §3.5).

## 2. Les 3 Tasks

| Task | Rôle | Fréquence |
|---|---|---|
| `TGPS` (`GPSTask`) | GPS — lecture, filtrage, écrit `SharedSFMem.gpsMag` | 100Hz |
| `TEKF` (`EKFTask`) | Relaie attitude + position/vitesse EKF du FC (`SharedFCStatus`) vers `SharedSFMem.state`, applique `HomeCalibrator` | 50Hz |
| `TLidar` (`LidarTask`) | Convertit chaque mesure (angle, distance) LIDAR en point NED via l'attitude courante, écrit `SharedSFMem.lidar` | 10Hz |

Pas de `TCmd` pour l'instant (§7) — contrairement à `MavlinkInterface`,
qui en a un opérationnel.

## 3. `TGPS`

Le nom (`GPSTask`, membre `TGPS`) date d'une époque où GPS et
magnétomètre étaient fusionnés dans la même Task (un seul writer sur
`GpsMagSample`, pour éviter que deux Task séparées ne s'écrasent
mutuellement leurs champs). Depuis la migration du cap vers le FC (§3.5),
`TGPS` ne fait plus que du GPS — le nom reste tel quel, pas de valeur à
renommer pour ça seul.

`GPSTask::loop()` fait un **read-modify-write** sur `GpsMagSample` (lit
l'état courant via `getGpsMag()`, ne modifie que les champs rafraîchis ce
cycle, réécrit) — utile même à writer unique : préserve les champs déjà
en shm quand le driver renvoie `DriverError::NoNewData` (pas une panne,
juste "rien de neuf ce cycle").

### Staleness et filtres de plausibilité GPS

Deux mécanismes distincts, tous les deux avant d'accepter/garder un
échantillon :

- **Staleness** : si `lastGoodGpsTs_` date de plus de `kMaxGpsAge`
  (1500ms) — y compris quand le driver ne renvoie jamais d'erreur
  explicite (capteur débranché sans `EIO`) — `CODES::SFS002` est levé.
  Vérifié à **chaque** cycle, même ceux où `NoNewData` ferait sortir tôt
  de la fonction.
- **Plausibilité** de chaque nouvelle lecture (`GPSData`) :
  - `gpsFixOk` (UBX NAV-STATUS ≥ 3D)
  - `|latitude| ≤ 90°`, `|longitude| ≤ 180°`
  - vitesse plausible (`≤ 50 m/s`, marge généreuse arbitraire — à ajuster
    selon les perfs réelles du drone)
  - `hAcc ≤ 5000mm`, `sAcc ≤ 1000mm/s` (NAV-PVT)

Tout échec (staleness ou plausibilité) déclenche `CODES::SFS002`
(Critical, edge-triggered — ne relève qu'à la transition bon→mauvais, ne
spamme pas à 100Hz) et **ne met pas à jour** les champs GPS de
`GpsMagSample` (garde la dernière valeur connue plutôt que d'injecter une
lecture aberrante).

### Précision GPS réellement renseignée

`hdop`/`vdop` viennent de UBX NAV-DOP (message ajouté en revue de code —
ni NAV-PVT ni NAV-POSLLH ne les fournissent), `vertAccuracy` de
NAV-POSLLH.vAcc. Avant cette revue, ces trois champs restaient à 0 en
permanence — `GPS_INPUT.ignore_flags=0` affirmant alors à l'EKF3 du FC
que la précision était parfaite, alors qu'elle n'était simplement jamais
mesurée. `gpsWeek`/`gpsTowMs` (semaine GPS + ms dans la semaine) viennent
de la date UBX NAV-TIMEUTC + `iTowMs` (NAV-PVT) — **jamais** de `ts`
(horloge interne `steady_clock`, sans rapport avec le temps GPS réel).

## 3.5. Cap magnétique — migré vers le FC

**Décision actée en session** : le magnétomètre est câblé directement
sur le FC plutôt que sur le Pi. Raison : santé de l'EKF3 d'ArduPilot —
lui donner son propre compas plutôt que de dépendre de `GPS_INPUT.yaw`
(source unique, latence IPC+MAVLink en plus, perdue si `MavlinkInterface`
ou `SensorFusion` crash) simplifie l'archi et suit le chemin le mieux
supporté par ArduPilot.

Ce qui a été retiré côté SF (et ce qui ne reviendra pas sans nouvelle
décision explicite) :
- `IDriver<MagData> &mag_` sur `SensorFusions`, plus de driver `LIS3MDL`
  instancié dans `main.cpp`/`hil_sf_mv` (le fichier driver lui-même
  n'est pas supprimé, juste plus câblé — pourrait resservir).
- `GpsMagSample::heading`, `SF_CONV::calibrateMag`/`magHeading`/
  `magPhysicallyPlausible`, `CODES::SFS003` (Mag_lost) ne sont plus
  levés — l'entrée reste dans `INDEX.csv` par cohérence historique du
  registre de codes, mais est orpheline désormais.
- Côté `MavlinkInterface` : `GPS_INPUT.yaw` est envoyé à `0` de façon
  permanente ("cap inconnu" selon la spec MAVLink), plus calculé depuis
  un cap GPS/mag.

À faire côté FC (hors code Pi) : `EK3_SRC1_YAW` doit pointer vers le
compas (pas `2`=GPS comme configuré pour les tests SITL précédents),
`COMPASS_ENABLE` réactivé si désactivé pendant la phase GPS_INPUT-only.

## 4. `TEKF` — relais + `HomeCalibrator`

Décision actée : **si l'EKF du FC déconne, le drone se crash de toute
façon** (dépendance structurelle : Nav/MissionControl n'ont aucune autre
source de position/attitude). `TEKF` ne fait donc **aucune validation de
plausibilité** — juste un relais + recalage d'origine + une vérification
de **fraîcheur**.

`SharedFCStatus` porte désormais un timestamp par champ
(`attitudeTs`/`localPosVelTs`, stampés en interne par
`setAttitude()`/`setLocalPosVel()`) — **pas** `lastHeartbeat`, qui arrive
à 1Hz alors qu'`ATTITUDE`/`LOCAL_POSITION_NED` arrivent à 10-50Hz
(comparer à `lastHeartbeat` avec un seuil de 200ms déclenchait
`SFS005` en continu, même en fonctionnement nominal — bug identifié et
corrigé en revue de code). `EKFTask::loop()` compare ces deux timestamps
à `kAgeThrEKF` (1000ms — élargi depuis un seuil initial de 200ms, qui
laissait trop peu de marge au débit réduit du lien SITL, cf.
`MAVLINKINTERFACE.md` §7) ; au-delà, `CODES::SFS005` (Emergency,
edge-triggered) et `SharedSFMem.state` n'est pas mis à jour.

`homeOffsetNed` (`Vec3`, membre de `SensorFusions`) est appliqué
uniquement à la **position** (`posNed`), jamais à la vitesse (un offset
constant n'a pas de sens sur une vitesse). **Actuellement toujours
`{0,0,0}`** — pas de calibration réelle branchée (décision explicite :
différé, probablement via `TCmd` plus tard quand ce mécanisme existera
côté SF).

## 5. `TLidar` — nuage de points NED, pas une carte angle/distance

**Écart avec la première version de `SharedSFMem::LidarMap`.** La
structure initiale (`distanceMm[360]`/`intensity[360]`, indexée par angle
brut) mirrorait exactement `LidarData` — c'est-à-dire aucune vraie
compensation, juste un passe-plat. Remplacée par `points[360]` (`Vec3`
NED, relatif au drone) — chaque mesure (angle, distance) est vraiment
projetée en repère NED via l'attitude courante, pas seulement recopiée.

```cpp
// repère capteur/corps : x=avant, y=droite, z=0 (scan 2D horizontal)
Vec3 point = SF_CONV::lidarPointNed(distanceM, angleDeg, attitudeQuat);
```

Sentinelle identique à `LidarData` : `points[i].isZero()` (via
`Vec3::isZero()`) signifie "aucune mesure reçue à cet angle ce tour-ci"
(une distance physique nulle est impossible, donc pas d'ambiguïté avec un
vrai point à l'origine).

`TLidar` lit l'attitude depuis `SharedSFMem.state` (donc dépend
indirectement de `TEKF`, jamais directement de `MavlinkInterface`/
`SharedFCStatus`) — décision actée en session, cohérente avec le principe
"un seul point d'entrée par donnée" (l'attitude ne doit être lue qu'une
fois côté FC, recopiée par TEKF, puis réutilisée en interne à SF).

**Staleness** : `lastGoodLidarTs_` comparé à `kMaxLidarAge` (500ms), même
logique que `TGPS` — couvre le cas d'un capteur muet en continu sans
jamais renvoyer d'erreur driver explicite.

**Panne LIDAR** (`CODES::SFS004`, **Warning**, pas Critical) : sévérité
volontairement plus faible que GPS/EKF — perte du LIDAR dégrade
l'évitement d'obstacle mais ne met pas en péril le vol lui-même. Choix
par défaut, à corriger si tu juges que ça mérite Critical une fois
l'évitement d'obstacle câblé côté `Navigation`/`MissionControl`.

## 6. Tests unitaires (`SF_CONV`, `tests/test_sensorfusion.cpp`)

Même principe que `MavlinkInterface` : les `loop()` des 3 Task sont
privées, appelées uniquement par le thread RT — pas testées directement.
La logique pure en a été extraite dans `Conversions.hpp`
(`mapUbxFixType`, `gpsWeekFromUbxDate`, `gpsCoordPlausible`,
`gpsVelNedMs`, `gpsAccuracyOk`, `lidarPointNed`), testée indépendamment.

## 7. `TCmd` — pas encore câblé

Contrairement à `MavlinkInterface`, `SensorFusion` n'a pas de Task `TCmd`
ni de FIFO dédiée dans `SharedCmdMem` (seule celle de `MavlinkInterface`,
`mavlinkCmds`, existe aujourd'hui). Décision explicite : à part
déclencher le futur vrai `HomeCalibrator`, aucun ordre MissionControl→SF
n'est identifié comme utile pour l'instant — différé jusqu'à en avoir
réellement besoin, même pattern que pour `MavlinkInterface` avant que le
takeoff/arm/disarm en aient eu besoin.

## 8. Table des codes (`CODES::Category::Sensor`, préfixe `SFS`)

| Code | Nom | Sévérité | Déclencheur |
|---|---|---|---|
| `SFS001` | GPS_lock | Info | Fix GPS valide (édge-triggered à chaque lecture bonne — pas de "clear" pour un code mineur, cf. ring buffer) |
| `SFS002` | GPS_lost | Critical | Fix invalide, coordonnées/vitesse implausibles, hAcc/sAcc dégradés, staleness, ou erreur driver GPS réelle |
| `SFS003` | Mag_lost | *(orphelin)* | Plus levé depuis la migration du Mag vers le FC (§3.5) — code conservé dans `INDEX.csv` par cohérence historique |
| `SFS004` | Lidar_lost | Warning | Staleness ou erreur driver LIDAR réelle (hors `NoNewData`) |
| `SFS005` | EKF_dead | Emergency | `attitudeTs`/`localPosVelTs` (FC) périmés de plus de `kAgeThrEKF` |

## 9. Ce qui n'est PAS encore fait / tranché

- `HomeCalibrator` réel : `homeOffsetNed` toujours `{0,0,0}`, pas de
  mécanisme de capture (au premier fix EKF valide ? sur ordre explicite
  via un futur `TCmd` ? — différé, cf. §4/§7).
- `TCmd`/FIFO `SharedCmdMem` dédiée à SF : pas encore créées (§7).
- Persistance de calibration hors-process (SDD §3.3.3, "les données de
  calibration doivent être stockées à l'extérieur du process") : pas
  abordée, format/emplacement pas choisis.
- Pas encore validé en vol complet (HIL bring-up en cours) — le driver
  `LIS3MDL` reste dans le repo mais n'est plus instancié nulle part.
- Pas de code `CODES::` pour un obstacle détecté par `TLidar` — la
  détection/decision d'évitement reste entièrement côté `Navigation`/
  `MissionControl` (SDD §4), `TLidar` ne fait que produire la donnée brute
  compensée.
- Paramètres FC (`EK3_SRC1_YAW`, `COMPASS_ENABLE`) à re-régler pour la
  nouvelle archi Mag-sur-FC (§3.5) — pas fait depuis ce côté du code.
