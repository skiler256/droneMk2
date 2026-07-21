# CLAUDE.md

Contexte du projet pour Claude Code. Lire ce fichier avant toute intervention.

## Vue d'ensemble

Drone autonome développé en solo, ~1 an de projet, hardware terminé (châssis,
connexions élec faites). Phase actuelle : implémentation logicielle, tests
unitaires, puis intégration SITL.

- **Compute embarqué** : Raspberry Pi 5, kernel PREEMPT_RT
- **Flight controller** : SpeedyBee H743 sous ArduPilot (stabilisation
  d'attitude, contrôle moteurs) — le Pi 5 ne fait PAS de contrôle bas niveau
- **Communication FC** : MAVLink v2 sur UART
- **Capteurs** : MPU6050 (IMU), LIS3MDL (magnéto), LD06 + TF Luna (lidars),
  GPS (UART)
- **Télémétrie** : wfb-ng 433MHz
- **Collaborateur** ("mon pote") : développe le GCS dashboard en Python
  (PySide6/Qt, pyqtgraph, Node.js/WebSocket côté serveur). Il travaille contre
  un simulateur que j'ai fourni. Je ne touche pas à son code sauf demande
  explicite.

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

### Bugs connus à corriger dans la couche IPC (avant de la figer)

1. `ComponentID::Count` doit être `static_cast<std::size_t>()` pour les
   paramètres de template de tableau
2. `access()` bloque actuellement sur TOUS les writer flags au lieu du seul
   slot concerné — doit attendre uniquement le flag pertinent
3. `access()` retourne `void`, ce qui rend les échecs silencieux → garbage
   dans des appelants comme `getHealth()`. Doit propager une erreur/optional
4. Le champ `checksum` unique cause des races d'écriture entre writers
   concurrents — doit devenir un tableau (un checksum par writer/slot)
5. `using namespace std` / `using namespace TYPES` dans les headers polluent
   le namespace global — à supprimer, qualifier explicitement
6. `virtual void reset()` et `setHealth()` déclarés sans corps ni `= 0` —
   trancher le contrat d'interface et implémenter
7. `MAX_COMPONENT_RESTART` dupliqué en magic number au lieu d'utiliser la
   constante définie

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
5. MavlinkInterface (composant en cours d'implémentation actuellement)
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

## État actuel — prochaine tâche

Implémentation de **MavlinkInterface**. Tests à couvrir (déjà définis) :

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
