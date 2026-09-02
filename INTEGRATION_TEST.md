# Plateforme de tests d'intégration système

> Statut : spécification d'origine, conservée pour la numérotation de
> section — largement citée par numéro dans les commentaires du code (C++
> et Python) et dans les tests, donc gardée stable plutôt que renumérotée
> ou supprimée. **L'état réellement construit et à jour vit dans
> `sitl/SITL.md`** — ce document-ci reste la trace du plan d'origine et de
> son raisonnement, pas la référence de ce qui tourne aujourd'hui.
> Portée : validation système bout-en-bout du soft drone (5 composants)
> contre un environnement simulé complet.

---

## 1. Objectif et périmètre

### 1.1 Ce que la plateforme valide

- **Boucle de guidage fermée** : `Nav` → setpoints → contrôleur guided ArduPilot → dynamique.
- **Chaîne capteur complète** : monde → mesures bruitées → `SensorFusion` → `MavlinkInterface` → EKF.
- **Comportement système émergent** : mission complète A→Z pilotée par `MissionControl`.
- **Gestion de panne** : capteur dégradé, processus mort, lien coupé, EKF divergent.
- **Évitement d'obstacle** : V1 freinage, V2 contournement.

### 1.2 Ce que la plateforme ne valide pas

- Le comportement temps réel fin sur cible Pi 5 (jitter, latence RT) → banc séparé.
- Les drivers matériels réels (I2C, UART) → remplacés par des drivers UDP.
- La dynamique de vol elle-même → déléguée à ArduPilot SITL, considérée comme référence.

### 1.3 Principe directeur

> **Validation boîte noire.** Les tests parlent au système par ses interfaces
> externes (MAVLink, UDP capteur, shm en lecture). Aucun binding sur le code C++.
> Le binaire testé est le binaire de production.

---

## 2. Architecture en blocs

```
┌──────────────────────────────────────────────────────────────────┐
│ [5] ORCHESTRATEUR (Python)                                       │
│     cycle de vie · scénarios · pannes système · verdict          │
└───┬──────────────────────────────────────────────────────────┬───┘
    │                                                          │
    │  ┌────────────────────────────────────────────────────┐  │
    │  │ [1] SYSTÈME SOUS TEST                              │  │
    │  │     8 composants · binaire prod · drivers UDP      │  │
    │  └───┬──────────────────────────────────┬─────────────┘  │
    │      │ MAVLink                          │ UDP capteur    │
    │      │                                  │                │
    │  ┌───▼──────────────────────┐           │                │
    │  │ [2] ArduPilot SITL       │           │                │
    │  │     dynamique · EKF      │           │                │
    │  │     guided · compas sim  │           │                │
    │  └───┬──────────────────▲───┘           │                │
    │      │ SIM_STATE        │ DISTANCE_     │                │
    │      │ (vérité)         │ SENSOR        │                │
    │  ┌───▼──────────────────┴───────────────▼─────────────┐  │
    │  │ [4] MODÈLE CAPTEUR                                 │  │
    │  │     bruit · biais · latence · dropout · pannes     │  │
    │  └───▲────────────────────────────────────────────────┘  │
    │      │ mesures géométriques brutes                       │
    │  ┌───┴────────────────────────────────────────────────┐  │
    │  │ [3] MOTEUR 3D — géométrie pure                     │  │
    │  │     monde · raycasting · rien d'autre              │  │
    │  └────────────────────────────────────────────────────┘  │
    │                                                          │
    └──────────────────┬───────────────────────────────────────┘
                       │
              ┌────────▼─────────────────────────────────┐
              │ [6] ENREGISTREUR                         │
              │     traces horodatées · rejeu accéléré   │
              └──────────────────────────────────────────┘
```

---

## 3. Description des blocs

### 3.1 [1] Système sous test

Les 5 composants, lancés comme en production (processus indépendants, binaire
unique, dispatch argv). Seule différence : les drivers matériels sont remplacés
par des drivers UDP via l'interface `IDriver`.

**Contrainte** : aucune modification du code de prod pour les besoins du test,
à l'exception du point ouvert §10.1 (watchdog).

### 3.2 [2] ArduPilot SITL

Fournit la dynamique multirotor, l'EKF3, le contrôleur guided, le baro, l'IMU
et le compas simulés.

- Lancement : `sim_vehicle.py -v ArduCopter --console --map`
- Reste le **modèle de référence** : on ne réimplémente pas de dynamique.
- Exporte l'état vrai (non bruité, hors EKF) via `SIM_STATE` / `SIMSTATE`.
- Tourne obligatoirement à **RTF 1** (cf. §8.1).

### 3.3 [3] Moteur 3D

**Responsabilité unique : la géométrie.**

- Charge un monde (obstacles : arbres, rochers, bâtiments, véhicules).
- Place un acteur cinématique à la pose reçue de `SIM_STATE`.
- Lance les rayons, retourne distances + identifiant/matériau de la cible.
- **Ne simule aucune physique** : le drone traverse les obstacles. Une collision
  est détectée géométriquement et compte comme échec de test.

**Interdit dans ce bloc** : bruit, modèle capteur, injection de panne, logique
de test. Ce bloc doit rester remplaçable (changement de moteur sans impact aval).

### 3.4 [4] Modèle capteur

Livrable de première classe, testable unitairement, piloté par fichier.

| Fonction | Détail |
|---|---|
| Bruit | gaussien paramétrable par capteur |
| Biais | constant + dérive lente |
| Latence | retard configurable par flux |
| Dropout | perte de trames, taux paramétrable |
| Quantification | résolution réelle du capteur |
| Étalement LD06 | pose par point, pas de snapshot instantané (cf. §9.3) |
| Pannes capteur | perte fix GPS, HDOP dégradé, lidar aveuglé, dérive de biais |

**Graine de bruit fixée et versionnée par scénario.**

Fichier de modèle capteur **séparé** du scénario : permet de rejouer le même vol
avec un GPS propre puis dégradé sans toucher au plan de vol.

### 3.5 [5] Orchestrateur

Écrit en Python. Parle au système uniquement par ses interfaces externes.

Responsabilités :
- charger un scénario (mission + monde + modèle capteur + pannes + critères) ;
- démarrer et arrêter SITL, le moteur, les 8 processus ;
- injecter les **pannes système** (distinctes des pannes capteur, cf. §7) ;
- collecter les traces ;
- calculer un **verdict pass/fail automatique** et produire un rapport.

Outils :
- `pymavlink` — commandes GCS, plan de vol, lecture télémétrie
- `UDP_bridge` existant — injection capteur
- `mmap` + `struct` — **lecture directe de la shm** : observabilité totale de
  l'état interne des composants, sans couplage au code C++

> **Prior art** : étudier le framework d'autotest Python d'ArduPilot avant de
> définir le format de scénario.

### 3.6 [6] Enregistreur

Capture sur une base de temps commune : trafic UDP capteur, tlog MAVLink,
snapshots shm, logs SITL, sorties des composants.

Deux usages :
1. **Post-mortem** — analyser un échec sans le reproduire.
2. **Rejeu accéléré** — rejouer le flux capteur enregistré dans le système, sans
   moteur ni SITL, à la vitesse permise par l'abstraction d'horloge.

> ⚠️ Le rejeu est en **boucle ouverte**. Valide pour `SensorFusion`, détection de
> panne, décodage `MavlinkInterface`, transitions `MissionControl`.
> **Invalide pour Nav en boucle fermée** — les données rejouées ne réagissent pas
> aux nouvelles décisions.

---

## 4. Routage des capteurs

Le routage n'est pas symétrique : il reproduit l'architecture réelle.

| Capteur | Réel | En simulation |
|---|---|---|
| GPS (NEO-M8N) | UART → Pi | [4] → `GPS_INPUT` → **[1]** (via `MavlinkInterface`) |
| Magnétomètre (LIS3MDL) | I2C → **FC** | simulé par SITL, rien à injecter |
| TF Luna (rangefinder) | UART → **FC** | [4] → `DISTANCE_SENSOR` → **[2]** |
| LD06 (360°) | UART → Pi | [4] → `UDP_bridge` → **[1]** (`SensorFusion`) |
| IMU / Baro | FC | simulés par SITL |

---

## 5. Configuration ArduPilot requise

### 5.1 Paramètres SITL

```
SIM_GPS_DISABLE = 1      # ⚠️ CRITIQUE — nom variable selon version
                         # (SIM_GPS1_ENABLE = 0 sur firmware récent)
```

> Sans ça, le GPS simulé interne alimente l'EKF et le chemin `GPS_INPUT` du
> système n'est **pas testé**. Faux positif garanti.

### 5.2 Paramètres EKF / sources

```
AHRS_EKF_TYPE   = 3
EK3_ENABLE      = 1
EK2_ENABLE      = 0
GPS1_TYPE       = 14      # MAVLink ("GPS_TYPE" si firmware < 4.5)
GPS_AUTO_CONFIG = 0
GPS_AUTO_SWITCH = 0
EK3_SRC1_POSXY  = 3       # GPS
EK3_SRC1_VELXY  = 3       # GPS
EK3_SRC1_POSZ   = 1       # Baro
EK3_SRC1_VELZ   = 3       # GPS
EK3_SRC1_YAW    = 1       # Compass
```

### 5.3 Rangefinder

```
RNGFND1_TYPE    = 10      # MAVLink
RNGFND1_ORIENT  = 25      # vers le bas
RNGFND1_MIN_CM  = 20      # TF Luna : 0,2–8 m
RNGFND1_MAX_CM  = 800
EK3_RNG_USE_HGT = -1      # lidar HORS de la fusion EKF
```

> Noms `RNGFND1_MIN` / `RNGFND1_MAX` **en mètres** sur firmware récent — vérifier.

### 5.4 Failsafes

```
FS_GCS_ENABLE   = ?       # choisir "Land" dans le menu MP, pas un chiffre au hasard
FS_GCS_TIMEOUT  = 5
FS_EKF_ACTION   = 1       # Land
FS_EKF_THRESH   = 0.8
SYSID_MYGCS     = 255     # doit matcher kGcsSystemId de MavlinkInterface
```

### 5.5 Limites alignées

Les limites FC doivent matcher les constantes logicielles, sinon deux écrêteurs
divergents rendent le diagnostic impossible.

```
WPNAV_SPEED     = ?       # = MAX_HORIZONTAL_MS × 100
WPNAV_SPEED_UP  = ?       # = MAX_CLIMB_MS × 100
WPNAV_SPEED_DN  = ?       # = MAX_DESCENT_MS × 100
```

### 5.6 Rappel : masque de setpoint

`SET_POSITION_TARGET_LOCAL_NED`, mask **`0x09C7`** — vitesses actives (vx, vy, vz),
yaw actif, positions et accélérations ignorées, yaw_rate ignoré.
Frame `MAV_FRAME_LOCAL_NED`. `vz < 0` = montée. `yaw` en **radians**.

---

## 6. Format de scénario

Un fichier déclaratif par scénario, versionné dans le repo. **Pas de scénario en
code** — sinon impossible d'en avoir trente et de dire « ce commit a cassé le 12 ».

Contenu minimal :

```yaml
id: 012_evitement_frontal
description: >
  Segment droit, obstacle statique à 40 m, vérifier le freinage.

monde: worlds/plaine_arbres.json
modele_capteur: sensors/nominal.yaml
graine: 42

mission:
  - {type: takeoff, alt: 10}
  - {type: segment, from: [0,0,-10], to: [100,0,-10], vitesse: 5.0}
  - {type: land}

pannes: []          # cf. §7

criteres:
  - {metrique: distance_min_obstacle, min: 5.0}
  - {metrique: collision, attendu: false}
  - {metrique: erreur_laterale_rms, max: 0.5}
```

---

## 7. Injection de pannes — deux domaines distincts

Ne pas mélanger. Deux mécanismes, deux endroits.

### 7.1 Pannes capteur → bloc [4]

- perte de fix GPS / dégradation HDOP
- dérive de biais magnétomètre (côté SITL)
- lidar aveuglé, secteur mort, dropout
- latence anormale

### 7.2 Pannes système → bloc [5]

- `kill -9` d'un composant
- corruption d'un segment shm
- gel de thread (SIGSTOP)
- saturation CPU
- coupure du lien MAVLink Pi ↔ FC

> **Ne jamais mettre l'injection de panne dans le moteur 3D.**

---

## 8. Contraintes structurantes

### 8.1 RTF 1 obligatoire

Les composants sont des processus RT qui lisent `Clock::now()` en temps mur.
`--speedup` de SITL casserait la cohérence : Nav tournerait à 50 Hz réels
pendant que le monde avance N fois plus vite.

**Conséquence** : le circuit complet est bloqué au temps réel. Une suite de
30 scénarios × 5 min = 2h30.

### 8.2 Abstraction d'horloge — prérequis

Pour lever partiellement §8.1 et permettre le rejeu accéléré (§3.6) et les tests
de timeout (`FS_GCS_TIMEOUT` × N scénarios).

- À faire **avant Nav**, tant que 5 composants sur 8 sont écrits.
- Coût maintenant : 5 composants. Dans 6 mois : 8 + tous les tests.
- Implémentation : **paramètre de template / politique résolue à la compilation**.
  Pas d'interface virtuelle appelée à 50 Hz (règles JPL, `-fno-rtti`).

### 8.3 Reproductibilité

| Niveau | Atteignable ? |
|---|---|
| Rejeu bit-exact | ❌ Non — 8 processus RT, ordonnancement variable |
| Reproductibilité du **verdict** | ✅ Oui — objectif retenu |

**Corollaire** : les assertions portent sur des **enveloppes** (RMS, max, seuils),
jamais sur des trajectoires exactes. Un test comparant des flottants à l'identique
sera rouge une fois sur trois et finira ignoré.

### 8.4 Versionnage des formats de fil

Magic + numéro de version sur chaque protocole inter-bloc, comme dans la shm.
Sans ça : soirées perdues sur des désalignements de struct entre le moteur et le C++.

---

## 9. Métriques et critères pass/fail

### 9.1 Suivi de trajectoire

| Métrique | Critère indicatif |
|---|---|
| Erreur latérale max (segment droit) | à définir |
| Erreur latérale RMS | à définir |
| Dépassement à la jonction de segments | à définir |
| Temps d'établissement après échelon de vitesse | à définir |
| Erreur d'altitude en régime établi | à définir |
| Pompage sur boucle P altitude | absence requise |

### 9.2 Évitement

| Métrique | Critère |
|---|---|
| Distance minimale à l'obstacle | > marge de sécurité |
| Distance de freinage | < distance de détection |
| Collision | jamais |
| Temps de détection → début de freinage | à définir |

### 9.3 Fidélité LD06 — point critique

Un vrai LD06 livre ses points **au fil de la rotation**, en paquets, pas en
snapshot instantané. À 10 Hz de rotation et 5 m/s, le drone parcourt 50 cm par
tour, et il a tourné pendant ce temps : le scan est **étalé**.

Le bloc [4] doit horodater chaque point et lui associer la pose du drone à cet
instant. Sans ça, un planificateur de contournement accumule une erreur
systématique invisible en simulation et fatale en vol.

### 9.4 Panne

| Métrique | Critère |
|---|---|
| Temps de détection de perte composant | à définir |
| Temps de détection perte heartbeat FC | < seuil FSTask |
| Passage en LAND après mort du Pi | < `FS_GCS_TIMEOUT` + marge |
| Comportement au retour du composant | conforme au scénario |

---

## 10. Décisions ouvertes à l'origine — état actuel

### 10.1 Watchdog et pannes volontaires — résolu

Question d'origine : `GlobalWatchdog` relance-t-il un composant tué
volontairement par l'orchestrateur ? Si oui sans contrôle, l'injection de
panne ne teste rien (le processus revient avant que le système ait réagi).

**Résolu dans le code du watchdog, pas dans l'orchestrateur** (comme prévu
ci-dessus) : `SharedWatchdogControl` (`include/drone/core/
WatchdogControl.hpp`) expose `suppressRespawn[id]`, à positionner avant le
kill pour empêcher `GlobalWatchdog` de relancer ce composant précis. Preuve
de fonctionnement : `tests/test_global_watchdog.cpp::
RespawnSuppressedWhenFlagSet`. Pas encore de wrapper Python dédié côté
`sitl/procman.py` — à écrire quand un scénario en aura besoin (cf.
`sitl/SITL.md` §5).

### 10.2 Choix du moteur 3D — tranché

Godot 3.6 retenu. En mode « serveur de capteurs passif », le lockstep de
Gazebo importe peu (RTF 1 imposé de toute façon) ; Gazebo a en plus des
soucis OpenGL rapportés sous WSL2. Détails d'implémentation : `sitl/SITL.md`
§9.

### 10.3 Format de monde

Doit permettre de fabriquer facilement des mondes **adversariaux** pour la V2
(impasses, passages étroits, obstacles en U, concavités) — les minima locaux qui
cassent un évitement réactif. Donc format éditable en données, pas scène modelée
à la main.

---

## 11. Roadmap d'implémentation

> Plan d'origine ci-dessous, conservé tel quel pour le raisonnement (ordre
> choisi et pourquoi). **État réel d'avancement à jour : `sitl/SITL.md` §8.**

Ordre choisi pour minimiser le coût de rétrofit : **l'orchestrateur avant le 3D**.
Le moteur est un module remplaçable derrière une interface ; l'orchestrateur ne
se rétrofitte pas.

| # | Étape | Livrable | Critère de sortie |
|---|---|---|---|
| 0 | Abstraction d'horloge | framework `Task` | 5 composants migrés, tests verts |
| 1 | Nav en tests unitaires purs | GTest | segment → setpoint déterministe |
| 2 | Format scénario + squelette orchestrateur | Python, sans 3D | lance SITL nu, joue une mission, rend un verdict |
| 3 | Nav validé sur SITL nu | — | métriques §9.1 dans l'enveloppe |
| 4 | Moteur 3D + modèle capteur | blocs [3] et [4] | tranche verticale : 1 scénario trivial traversant **toutes** les couches |
| 5 | Enregistreur + rejeu | bloc [6] | rejeu accéléré d'un scénario capturé |
| 6 | Élargissement | 30 scénarios | suite complète exécutable |

### 11.1 Garde-fous

- **Tranche verticale d'abord** (étape 4) : un scénario trivial mais complet, avec
  verdict automatique. Élargir ensuite est linéaire ; rétrofitter ne l'est pas.
- **Garder l'étape 3 vivante** : le chemin « Nav + SITL nu, sans 3D » doit rester
  exécutable en permanence. C'est le filet quand la plateforme est en travaux.
- **Timebox explicite** sur l'étape 4. Dépassement franc = signal qu'il faut
  reconsidérer Gazebo (lockstep + capteurs déjà faits contre coût de setup).

---

## 12. Pièges connus

| # | Piège | Parade |
|---|---|---|
| 1 | GPS SITL laissé actif → chemin `GPS_INPUT` non testé | §5.1 |
| 2 | Moteur 3D dans le chemin critique : un stutter de frame ressemble à une panne capteur | découpler raycasting du rendu ; [4] horodate et **détecte les trous** |
| 3 | Latence ajoutée par la boucle SITL → moteur → SITL sur le rangefinder | mesurer et budgétiser |
| 4 | LD06 simulé en snapshot instantané | §9.3 |
| 5 | Assertions sur trajectoires exactes | §8.3 — enveloppes uniquement |
| 6 | Désalignement de struct moteur ↔ C++ | §8.4 — magic + version |
| 7 | Bindings Python sur le C++ | interfaces externes uniquement (§1.3) |
| 8 | `--speedup` | interdit (§8.1) |

---

## 13. Références

- ArduPilot SITL — interface JSON : `sim_vehicle.py -f JSON:127.0.0.1`
  (exemples Python, MATLAB/Simulink, C++)
- `ardupilot_gazebo` — plugin officiel, lockstep, capteurs additionnels
- Framework d'autotest ArduPilot (Python) — prior art pour le format de scénario
- `SIM_STATE` / `SIMSTATE` — état vrai de simulation, hors EKF
- Docs internes : `MAVLINKINTERFACE.md`, `SENSORFUSION.md`, `sitl/SITL.md`
  (état à jour de tout ce qui est construit)