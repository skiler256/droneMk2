# CLAUDE.md

Contexte du projet pour Claude Code. Lire ce fichier avant toute intervention.

## Vue d'ensemble

Drone autonome développé en solo, ~1 an de projet, hardware terminé (châssis,
connexions élec faites). Phase actuelle : implémentation logicielle, tests
unitaires, puis intégration SITL.

- **Compute embarqué** : Raspberry Pi 5, kernel PREEMPT_RT
- **Flight controller** : Sequre sous ArduPilot (stabilisation
  d'attitude, contrôle moteurs) — le Pi 5 ne fait PAS de contrôle bas niveau
- **Communication FC** : MAVLink v2 sur UART
- **Capteurs** : MPU6050 (IMU), LIS3MDL (magnéto), LD06 + TF Luna (lidars),
  GPS (UART)
- **Télémétrie** : deux liens indépendants
  - **PDL** (principal) : wfb-ng sur RTL8812AU, 2,4 / 5 GHz — haut débit
  - **SDL** (secours) : CC1101 en SPI, 433 MHz, ASK/OOK 9,6 kbit/s — porte
    toutes les commandes montantes critiques
- **Collaborateur** ("mon pote") : développe le GCS dashboard en Python
  (PySide6/Qt, pyqtgraph, Node.js/WebSocket côté serveur). Il travaille contre
  un simulateur que j'ai fourni. Je ne touche pas à son code sauf demande
  explicite.

## RÈGLE DURE — matériel réel : demander avant d'exécuter

S'applique dès qu'une session a un accès (SSH ou local) au Raspberry Pi du
drone. Le Pi est relié au FC par UART et aux périphériques par SPI/I2C/UART :
une commande n'y a pas seulement des effets logiciels, elle peut faire
tourner des moteurs.

**Demander l'accord explicite avant chaque exécution** de :

- `build/drone` et tout binaire du logiciel de vol — démarrer le lien MAVLink
  suffit à lancer la machine à états de `MissionControl` ;
- les bancs `bench_gps` / `bench_lidar` / `bench_mag` et tout ce qui ouvre un
  port série, SPI ou I2C vers un périphérique ;
- toute commande MAVLink vers le FC, armement et changement de mode en
  particulier ;
- toute manipulation des liens radio (wfb-ng, CC1101) : une coupure de lien
  déclenche les procédures d'urgence ;
- tout ce qui touche aux paramètres du FC.

Ne nécessitent pas de demande : lecture de fichiers, `git`, `cmake`, `ctest`,
inspection système (`ss`, `systemctl status`, `dmesg`, `lsmod`), et
l'édition de fichiers du dépôt.

En cas de doute sur la catégorie, demander. Une question de trop coûte dix
secondes ; une hélice qui part en coûte beaucoup plus.

Indiquer aussi, avant une séance de travail sur le matériel, l'état attendu
du drone (hélices démontées, batterie débranchée des ESC) plutôt que de le
supposer.

## Philosophie d'ingénierie

Standards inspirés NASA JPL / Airbus autopilot design patterns :

- **Simplicité over complexité** — je rejette systématiquement les solutions
  sur-ingénierées. Exemple déjà tranché : seqlock remplacé par un simple
  atomic bool + checksum, `flock` rejeté au profit du `waitpid()` déjà connu
  par GlobalWatchdog. Si tu proposes une solution, la version la plus simple
  qui marche gagne, pas la plus élégante en apparence.
- **Types forts partout** — pas de float nu pour des grandeurs physiques
  (voir `types.hpp`, pattern `Scalar<Tag>`). Ne jamais introduire un type
  faible là où un type fort existe déjà ou pourrait exister.
- **Process isolation** — chaque composant tourne en process Linux isolé
  (fork/execl), pas en thread partagé. Ne jamais proposer de revenir à du
  multi-thread intra-process pour l'isolation des composants.
- **Erreurs typées, jamais silencieuses** — `std::expected`, pas
  d'exceptions dans le code de vol (`-fno-exceptions` en prod, exceptions
  seulement activées pour GTest). Ne jamais laisser une fonction avaler une
  erreur silencieusement (cf. bug connu sur `access()` qui retourne `void`).

## Architecture IPC (couche en cours de finalisation — PRIORITAIRE)

Deux types de segments de mémoire partagée POSIX (`shm_open`/`mmap`) :

- **`SharedCompMem`** : un seul writer exclusif par segment (mémoire propre
  à un composant, ex: `SharedNavMem`, `SharedSFMem`)
- **`SharedSysStateMem`** : plusieurs writers, un slot par composant indexé
  par `ComponentID`

Décisions déjà actées, ne pas remettre en question sans raison forte :
- Pas de seqlock — flag atomic bool par writer + checksum
- Le lifecycle des segments est géré uniquement par le process d'init
  système, jamais par un composant individuel
- `GlobalWatchdog` reset les flags de writer bloqués en mappant uniquement
  le tableau de flags (pas tout le segment)
- `std::mutex` local pour éviter les races intra-process entre threads d'un
  même composant
- Les types Eigen sont exclus des payloads shared memory — passer par des
  structs POD intermédiaires + `Eigen::Map` pour la reconstruction
- Pattern template `access(id, lambda)` pour unifier lock/operate/unlock

### État de la couche IPC

Considérée comme figée — les points ouverts qui subsistaient (indexation de
tableau par `ComponentID`, granularité du verrou, propagation d'erreur sur
accès shm, isolation de namespace, contrat `reset()`/`setHealth()`, taille de
l'historique de restart) sont réglés. Détail du design et des raisons
derrière chaque décision : `ARCHITECTURE_IPC.md`.

## Hiérarchie de watchdog (deux niveaux)

- **`GlobalWatchdog`** : surveille les PID des process composants
- **`ComponentWatchdog`** : surveille les heartbeats des Tasks à l'intérieur
  de chaque process

Hiérarchie de restart : crash de Task → hot-start du Component → cold-start
du Component → DEAD + procédures d'urgence. **`MissionControl` ne fait
jamais de cold-start** — sa state machine ne peut pas être resetée en
sécurité, ne jamais proposer de le lui faire faire.

Le tracking des restarts utilise des ring buffers de timestamps ("X
restarts en Y secondes"), pas de compteurs absolus.

## Composants du système (ordre de dépendance pour l'implémentation)

1. IPC layer (en cours, prioritaire)
2. GlobalWatchdog
3. SysMonitoring (composant le plus simple, valide le pattern process
   isolation + IPC de bout en bout — "golden path")
4. ComponentWatchdog (validé d'abord via SysMonitoring)
5. MavlinkInterface (implémenté : 905 lignes, 34 tests, banc HIL_SF_MV)
6. SensorFusion
7. Nav
8. MissionControl (le plus critique, jamais de cold-start)

## Stack technique

- **Langage** : C++23, kernel PREEMPT_RT
- **Build** : CMake (voir `CMakeLists.txt` — flags stricts :
  `-Wall -Wextra -Werror -Wshadow -Wconversion -Wnull-dereference
  -fno-exceptions -fno-rtti`)
- **Tests** : GTest (`cd build && ctest -v`), `-fexceptions` activé
  uniquement pour les binaires de test
- **Cross-compilation** : toolchain ARM64 pour la cible Pi 5
  (`cmake -B build/arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/arm64.cmake`)
- **Lib MAVLink** : `mavlink/c_library_v2` (headers C purs), dialecte
  `ardupilotmega.xml`
- **SITL** : ArduPilot SITL (`ArduPilot/ardupilot`, `sim_vehicle.py`)
- **Outils debug MAVLink** : MAVProxy, pymavlink (fuzzing/scripts de test),
  Wireshark + plugin MAVLink si besoin d'inspection bytes
- **Éditeur/tooling** : VS Code en Remote-SSH, clangd

## Comment je veux travailler avec toi (Claude Code)

- **L'architecture et la philosophie du système, c'est mon terrain.** Je
  veux faire le travail de software engineer moi-même — pas une archi
  préfaite par toi qui "ressemble à tout et rien". Sur une question de
  design, présente les options avec leurs tradeoffs, je tranche, tu
  implémentes ce qui a été décidé. Ne comble jamais un vide d'archi
  toi-même parce que c'est plus simple ou plus rapide — pose la question.
- **Le répétitif/mécanique, c'est ton terrain.** Dictionnaires de codes,
  tables de paquets, boilerplate de sérialisation : plutôt que de me faire
  éditer des gros tableaux à la main, construis-moi un outil (générateur,
  éditeur web) pour que je puisse le faire moi-même en continu, avec ou
  sans toi. `tools/gen_codes.py`+`codes_editor.py` et
  `tools/gen_telemetry.py`+même éditeur sont le modèle à suivre pour tout
  nouveau format répétitif.
- **Dis-moi quand tu vois un bug, un manque de feature, un trou de sécu —
  toujours.** Je veux apprendre, donc signale plutôt que de rester
  silencieux ou de corriger sans le dire. Je valide, tu implémentes ta
  remarque ensuite.
- **Diffs petits et digestes.** Je relis chaque modification pour chasser
  les bugs/manques — un gros bloc de 8 fichiers d'un coup est dur à
  auditer sérieusement. Découpe le travail en morceaux reviewables, même
  après un feu vert global sur la direction générale, et marque une pause
  entre chaque morceau significatif plutôt que d'enchaîner.
- **Pas de vibe coding.** Pas d'exécution autonome de tâches en chaîne
  pendant que je suis absent. Mode interactif : je pose une question ou
  une tâche précise, tu réponds ou proposes un diff, je valide.
- Toujours passer par le confirm avant d'éditer un fichier ou d'exécuter une
  commande (build, test, SITL) — sauf instruction contraire explicite dans
  le message.
- Si je demande une fonction "rébarbative" (ex: stringifier un enum class),
  tu peux l'écrire directement dans le fichier concerné après lecture du
  contexte, en suivant le pattern déjà utilisé pour `toString(NavMode)` /
  `toString(SensorsT)` dans `types.hpp`.
- Signale-moi si une de mes demandes contredit une décision déjà prise plus
  haut dans ce fichier, plutôt que de l'appliquer silencieusement.
- Workflow perso : je fonctionne par pics de productivité, j'ai besoin de
  petites tâches actives avec critères d'acceptation clairs plutôt que de
  gros blocs vagues.
- **Contexte perso** : je prépare une prépa pour des études d'ingénieur
  (pas forcément soft) — je touche à la méca/élec/RF/soft depuis ~5 ans,
  je veux rester polyvalent. Ce projet me sert aussi à apprendre le dev en
  milieu critique/aéro, donc les explications sur le "pourquoi" d'un choix
  d'archi sont bienvenues, pas juste le "quoi". J'ai déjà de bonnes bases
  en C++ (petits projets perso avant celui-ci).
- J'ai un SDD qui couvre l'archi globale (découpage composant/Task, double
  watchdog, IPC, interfaces pour faciliter le SITL...) mais il n'est pas
  exhaustif — normal qu'il manque des détails qu'on tranche au fil de
  l'implémentation.

## État actuel

Le travail en cours porte sur la **couche liaison de SysMonitoring**, pas sur
MavlinkInterface. Détail complet et mesures dans `BRINGUP_PDL.md`.

**Acquis, mesuré sur matériel :**

- liaison SDL bidirectionnelle, 8 Hz descendant à ~2 % de perte (16 Hz tenus) ;
- liaison PDL opérationnelle, 250 trames sur 250 descendantes, 5,1 kbit/s ;
- banc HIL multi-liens (`./tools/link_bench.sh`), les deux liens simultanément ;
- acquittement des commandes critiques de bout en bout, avec dédoublonnage ;
- pont SDL dans le firmware ESP32 (panneau d'alarme + radio, même carte).

**Ouvert :**

- rien n'est persistant : mode moniteur, canal et processus wfb-ng sont à
  refaire à la main après chaque redémarrage (unités systemd à écrire) ;
- `main.cpp` construit encore des `UdpTelemetryDriver` : les vrais pilotes
  ne sont pas branchés dans le logiciel de vol ;
- `GlobalWatchdog` ne sait spawner que lui-même avec un rôle ; superviser
  wfb-ng demande une seconde catégorie d'enfant (binaire externe) ;
- le sens montant du SDL n'est pas caractérisé — les mesures se sont
  révélées non reproductibles, cf. la mise en garde dans `BRINGUP_PDL.md` ;
- latence PDL dominée par le blocage FEC (`k=8`), `-T fec_timeout` à régler.

## MavlinkInterface — tests restants

Le composant est implémenté (905 lignes, 34 tests unitaires) et tourne dans
`src/IntegrationTests/HIL_SF_MV.cpp` face au vrai FC. Ce qui suit est la liste
de couverture d'origine, à confronter à ce qui est déjà couvert :

**Unit tests (GTest, sans SITL)**
- Parsing/sérialisation : décodage v2 valide, rejet CRC invalide, rejet
  magic byte incorrect, message tronqué, messages concaténés, sérialisation
  de commandes sortantes
- Connexion : détection perte/reprise de heartbeat, gestion `SYS_STATUS`
  avec flag d'erreur critique
- Interface IPC : écriture correcte dans `SharedCompMem`/`SharedSysState`
  sur message valide, aucune écriture sur message invalide, pas de deadlock
  sur accès concurrent simulé via `access(id, lambda)`
- Robustesse : erreur UART (device disparu, EIO) sans crash, rate limiting
  si le FC spam

**Tests SITL**
- Connexion complète + heartbeat, réception télémétrie réelle, envoi de
  commande (arm/disarm) + ACK, changement de mode de vol, coupure/reprise
  de lien avec réaction correcte du watchdog, fuzzing léger de messages
  malformés

**Point d'archi à trancher avant de coder** : est-ce que MavlinkInterface
gère un reconnect automatique en interne, ou est-ce que c'est le rôle du
ComponentWatchdog de le redémarrer entièrement en cas de perte de lien ?
