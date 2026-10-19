# Architecture : IPC, Task, ComponentBase, Watchdogs

> Document de référence — explique le *pourquoi* et le *comment ça marche
> ensemble* de la couche fondation du système. Pour le *comment j'en écris
> un nouveau*, voir `GUIDE_COMPOSANTS.md`. Pour la vue d'ensemble
> fonctionnelle (flux de données, diagrammes), voir `Design.md`.

## 1. Vue d'ensemble : qui parle à qui

```
                         ┌─────────────────┐
                         │  GlobalWatchdog  │  process racine
                         │  (main.cpp par   │
                         │   défaut)        │
                         └───┬─────────┬────┘
                    fork+exec│         │fork+exec
              ┌────────▼──┐ ... ┌──▼─────────────┐
              │ Navigation │     │ MissionControl │  5 process, un par domaine
              │  (process) │     │   (process)    │  fonctionnel (§1.1)
              └─────┬──────┘     └──────┬─────────┘
                    │                   │
                    │  ComponentBase    │
                    │  ├─ ComponentWatchdog (thread RT local)
                    │  └─ N × Task (thread RT chacune, §3)
                    │                   │
                    └─────────┬─────────┘
                              │ lit/écrit via handlers
                     ┌────────▼────────┐
                     │  Mémoire partagée │  SharedNavMem, SharedSFMem,
                     │  (POSIX shm_open) │  SharedSysStateMem...
                     └───────────────────┘
```

### 1.1 Process : un binaire, un rôle par argv

Un seul exécutable (`drone`, `add_executable(drone src/main.cpp)`) sert pour
le process racine et les 5 composants — pas un binaire par composant. Le rôle
est choisi à l'exécution :

```cpp
// src/main.cpp
std::string_view role = (argc > 1) ? argv[1] : "globalwatchdog";
if (role == "navigation")       return runNavigation();
if (role == "sensorfusion")     return runSensorFusion();
if (role == "sysmonitoring")    return runSysMonitoring();
if (role == "mavlinkinterface") return runMavlinkInterface();
if (role == "missioncontrol")   return runMissionControl();
// sinon : rôle par défaut, GlobalWatchdog
```

`GlobalWatchdog::forkChildren()` lance les 5 en passant le rôle en `argv[1]`
du `fork()+exec(argv[0], role)` :

```cpp
children_[0] = spawn(ComponentID::Navigation, "navigation");
children_[1] = spawn(ComponentID::SensorFusion, "sensorfusion");
children_[2] = spawn(ComponentID::SysMonitoring, "sysmonitoring");
children_[3] = spawn(ComponentID::MavlinkInterface, "mavlinkinterface");
children_[4] = spawn(ComponentID::MissionControl, "missioncontrol");
```

Chaque domaine fonctionnel (Navigation, SensorFusion, SysMonitoring,
MavlinkInterface, MissionControl) est donc un **process Linux isolé** —
crash d'un composant = crash d'un process, sans effet mémoire direct sur les
autres (cf. §2.5 pour la récupération). Un `add_executable` séparé n'existe
que pour les binaires SITL/HIL (`src/IntegrationTests/*.cpp`, ex. `mv_sitl`,
`hil_sf_mv`), jamais pour les composants de production.

**À l'intérieur d'un process**, la granularité descend au thread : chaque
tâche indépendante du composant (ex. séquencement de mission, loi de
pilotage, fail-safe pour `Navigation`) est une `Task` = un thread `SCHED_FIFO`
dédié avec sa propre fréquence et son propre hearteat — voir §3. Le process
ne contient donc jamais de logique métier directement dans son `main()` de
rôle : il construit un `ComponentBase<N>` dérivé, qui possède ses `Task` et
son `ComponentWatchdog` local.

Trois niveaux de supervision, chacun avec une responsabilité précise et
**aucun recouvrement** — l'ensemble joue le rôle d'un **CBIT** (Continuous
Built-In Test), le terme avionique pour une auto-surveillance qui tourne en
continu pendant le vol plutôt qu'au sol avant démarrage (par opposition à
un PBIT/Power-on BIT) :

| Niveau | Surveille | Détecte | Action |
|---|---|---|---|
| `ComponentWatchdog` (thread, dans le process) | Heartbeat des `Task` du composant | Une `Task` bloquée/plantée | Redémarre la `Task` seule, ou tue le process si ça se répète trop |
| `GlobalWatchdog` (process racine) | PID des process composants | Un process mort (crash, tué par son propre `ComponentWatchdog`) | `fork`+`exec` un nouveau process, libère les verrous shm qu'il tenait |
| `ComponentBase` (à la (re)construction d'un composant) | Historique de restart persisté en shm | Trop de hot/cold-starts récents | Décide `NOMINAL`/`DEGRADED`/`SICK`/`DEAD`, appelle (ou non) `init()`/`restore()` |

## 2. La mémoire partagée

### 2.1 Anatomie d'un segment

```cpp
struct SharedMemory {           // embarqué dans TOUT segment shm
  std::atomic<bool> initialized;
  std::atomic<ComponentID> owner;  // Free (== ComponentID::Count) ou l'id du détenteur
  uint32_t checksum;
};

struct SharedXMem {
  SharedMemory mem;             // toujours en premier
  /* ... payload propre au composant ... */
};
```

Un `SharedMemoryHandler` (classe de base, `SharedMemory.hpp`) enveloppe
l'accès. Deux familles de segments concrets :

- **`SharedCompMem`** (via `SharedCompMemHandler`) : un seul propriétaire
  toute sa vie (`SharedNavMem`, `SharedSFMem`). Porte aussi
  `HotStartHistory`/`ColdStartHistory` (ring buffers de 10 timestamps) —
  c'est ce que `ComponentBase` lit pour décider hot/cold/DEAD.
- **`SharedSysStateMem`** (via `SharedSysStateMemHandler`) : tous les
  composants y écrivent, un slot par `ComponentID` (santé + codes de
  diagnostic — voir §5).

**Contrainte sur les payloads : POD uniquement.** Un segment est partagé
entre process via `mmap`, donc le payload traverse la frontière par copie
mémoire brute et doit être hashé par `crc32<T>()` — deux opérations qui
exigent `trivially_copyable` et `standard_layout`. **Eigen ne garantit ni
l'un ni l'autre** (alignement/vectorisation, et `Eigen::Quaternionf` est
même non-initialisé par défaut), d'où l'interdiction d'un type Eigen dans
un payload. Les types de `types.hpp` (`Vec3`, `Quat`) sont donc des floats
bruts, et exposent `asEigen()` qui rend une vue `Eigen::Map` *à la demande*
pour le calcul — sans jamais stocker le type Eigen lui-même, ce qui laisse
la struct POD.

**`RingBuffer<T, N>` : refuse plutôt qu'écrase.** Les files en shm
(`SharedMemory.hpp`) sont des POD sans mutex propre, manipulées uniquement
par `pushFifo()`/`popFifo()` sous le verrou CAS. Quand la file est pleine,
`pushFifo()` **échoue et retourne `false`** au lieu d'écraser le plus ancien
élément : sur un canal qui porte des ordres (un `PendingCommand` de
`MissionControl`, par exemple), perdre un élément silencieusement est pire
qu'un refus explicite que l'appelant peut constater et remonter. C'est le
même principe que « erreurs typées, jamais silencieuses » appliqué à l'IPC.
À noter que le ring des codes **mineurs** suit la règle inverse (écrasement
FIFO assumé, voir §5) — c'est un journal, pas un canal d'ordres.

### 2.2 Le verrou : un seul atomic, pas un mutex

Pas de `std::mutex` (ne fonctionne pas entre process), pas de futex custom.
Un unique `std::atomic<ComponentID> owner` fait office de spinlock CAS :

```cpp
auto tryAcquire = [&] {
  ComponentID expected = ComponentID::Count; // Free
  return mem_.owner.compare_exchange_strong(expected, id, memory_order_acq_rel);
};
UTILITIES::waitUntil(tryAcquire, timeout); // poll toutes les 10µs jusqu'au timeout
```

**Toute lecture ET toute écriture passent par le même verrou** — pas de
RW-lock séparé. Un design antérieur (`nonLockedAccess`) laissait les
lectures sans verrou pour permettre des lectures concurrentes ; ça
autorisait un lecteur à lire pendant qu'un writer écrivait (torn read).
Le coût de la sérialisation totale est négligeable aux fréquences du
projet (20-100Hz, sections critiques de quelques µs) — la sécurité prime
sur le débit ici.

`owner` a une double casquette : verrou (CAS pour acquérir) **et** registre
d'identité (qui le détient). C'est ce deuxième rôle qui permet à
`GlobalWatchdog` de libérer le verrou d'un process mort sans deviner qui le
détenait — voir §2.4.

### 2.3 `id` : jeton du verrou, pas forcément le sujet

Distinction qui revient partout dans le code, à bien comprendre :

- **`id` passé à `getData`/`setData`/`lockedAccess`** = l'identité **réelle
  de l'appelant**, utilisée uniquement comme jeton de verrouillage.
- **Le "sujet"** (le composant lu/modifié) peut être complètement
  différent — ex. `getHealth(ComponentID sujet)` : n'importe quel composant
  peut consulter la santé d'un autre.

Pourquoi cette distinction est structurelle et pas cosmétique : si un
lecteur verrouillait avec l'id du **sujet** plutôt que le sien, et
crashait pendant la lecture (en tenant le verrou), `GlobalWatchdog`
chercherait à libérer *son propre* id (le process mort) — qui ne
correspondrait jamais à ce qui est stocké dans `owner` (l'id du sujet).
Deadlock permanent du segment. D'où : write = toujours self-report (pas de
paramètre `id` du tout sur `setHealth()`/`raiseCode()`/`clearCode()` —
codé en dur sur `ownId_`), read = `id` explicite = le sujet, verrouillé en
interne avec `ownId_` du handler.

### 2.4 Checksum : intégrité, et sa granularité

Chaque accès verrouillé vérifie `checksumValid(id)` avant, et appelle
`updateChecksum(id)` après. Par défaut (`SharedMemoryHandler`), ça
compare/écrit le seul `mem_.checksum` du segment — correct pour un segment
**mono-écrivain** puisqu'il n'y a jamais qu'un seul flux de vérité.

**Pour un segment multi-écrivains (`SharedSysStateMem`), ce n'est PAS
suffisant.** Un seul checksum global veut dire que `reset(id)` — appelé
sur corruption détectée pendant une écriture — ne répare que le slot de
`id`, mais recalcule le checksum sur **tout** le payload, y compris les
slots d'autres composants jamais vérifiés. Résultat : la corruption d'un
composant B peut se faire "blanchir" silencieusement par une écriture
totalement indépendante d'un composant A. `SharedSysStateMemHandler`
override donc `checksumValid`/`updateChecksum` pour maintenir un tableau
`state_.checksums[id]` — un checksum **par composant**, indépendant. Un
writer ne peut jamais revalider que son propre slot.

**Choix du CRC32 : CRC-32C (Castagnoli), pas le CRC-32 classique.** Polynôme
`0x1EDC6F41` (réfléchi `0x82F63B78`) — celui qu'implémente nativement
l'extension CRC32 d'ARMv8. Conséquences :

- Sur la cible (Pi 5, Cortex-A76) : instruction machine dédiée
  (`__crc32d`/`__crc32b`), quasi gratuite en cycles.
- En dev x86_64 : repli logiciel par table de lookup, portable, ~1 cycle/octet.
  Le choix est fait **à la compilation**, sans branche à l'exécution.
- Les deux chemins produisent le **même** résultat, ce qui permet de comparer
  un checksum calculé en dev à un checksum calculé sur cible (logs, tests
  golden).

À réserver à l'intégrité d'un payload shm : un CRC32 **ne résiste pas** à une
falsification volontaire, ce n'est pas une primitive cryptographique.

**`repairOnCorrupt` : qui a le droit de réparer.** `lockedAccess()` prend un
flag qui décide si une corruption détectée déclenche `reset(id)` ou non. Il
est à `true` uniquement pour les accès qui sont déjà un read-modify-write
dans le cadre d'une **écriture** (`setData`, `pushFifo`, `popFifo`,
`setArrayElement`) et à `false` pour une **lecture pure** (`getData`). La
raison : une lecture qui réparerait le segment effacerait la preuve de la
corruption avant que quiconque l'ait constatée, et ferait d'un lecteur
passif un mutateur silencieux. Un lecteur remonte donc `shmError::corrupt`
et laisse le prochain writer réparer.

### 2.5 Récupération sur crash

`GlobalWatchdog::monitorLoop()` bloque sur `waitpid(-1, ..., 0)` — pas de
poll, réveil immédiat par le kernel dès qu'un enfant meurt (Note technique
: `sigaction()` est utilisé explicitement, pas `std::signal()`, parce que
`signal()` active `SA_RESTART` par défaut sur Linux/glibc, ce qui aurait
fait relancer `waitpid()` au lieu de le faire échouer avec `EINTR` — le
process serait resté bloqué indéfiniment même après `SIGTERM`). À la mort
d'un composant, `GlobalWatchdog::resetAll(id)` appelle `resetWriterFlag(id)`
sur chaque segment :

```cpp
void resetWriterFlag(ComponentID id) {
  ComponentID expected = id;
  mem_.owner.compare_exchange_strong(expected, ComponentID::Count, acq_rel);
}
```

CAS conditionnel : ne libère que si `id` détenait *vraiment* le verrou. Si
un autre writer (vivant) le détient au même instant, le CAS échoue et ne
touche rien — impossible de voler le verrou d'un composant qui tourne.

**Pourquoi `keepRunning_` est un `static`.** Le drapeau d'arrêt de
`monitorLoop()` est un `static inline std::atomic<bool>`, ce qui ressemble à un
défaut mais découle de l'API POSIX : un handler de signal a la signature
`void(int)` et ne reçoit aucun `this`. Le drapeau qu'il touche doit donc être
atteignable statiquement, et un `std::atomic<bool>` lock-free est l'une des
rares écritures autorisées depuis un handler.

Il sert à deux choses : sortir de la boucle (le signal interrompt `waitpid()`
avec `EINTR`, la boucle revérifie le drapeau et part vers `shutdown()`), et
distinguer un enfant mort *parce qu'on s'arrête* — qu'on ne respawne pas — d'un
enfant qui a crashé.

Conséquence acceptée : une seconde instance de `GlobalWatchdog` dans le même
process verrait le drapeau déjà à `false` et sa boucle ne tournerait jamais.
Invisible en production (une instance pour toute la vie du process), et le
contourner demanderait un `static GlobalWatchdog*` — on déplacerait le `static`
sans le supprimer. Décision : laisser en l'état. `tests/test_global_watchdog.cpp`
forke chaque scénario pour obtenir un drapeau frais.

## 3. Task : la boucle temps réel

Une `Task` encapsule un thread `SCHED_FIFO` dédié qui appelle `loop()` à
fréquence fixe et heartbeat après chaque itération :

```cpp
void run() {
  auto period = Us(1'000'000.0f / Tconfig.loopFrequency.v);
  auto next_time = Clock::now();
  while (running_.load(acquire)) {
    loop();
    TlocalWD.heartbeat(Tconfig.id);
    next_time += period;
    sleep_until(next_time);
  }
}
```

Points structurels :
- `RTpriority` doit être **≥ 1** — `SCHED_FIFO` avec priorité 0 est
  rejeté par le kernel (`EINVAL`), et `pthread_create` échoue
  *silencieusement* (pas d'exception, `-fno-exceptions`) : la Task ne
  tourne juste jamais, sans erreur visible si le retour n'est pas vérifié.
- `start()`/`stop()` sont idempotents (`threadValid_` gardé en interne) —
  nécessaire puisque `ComponentWatchdog::handleDeadTask` peut les rappeler
  à chaque cycle de restart, et le destructeur de `Task` appelle `stop()`
  aussi (RAII : sans ça, un thread RT peut survivre à la destruction de
  l'objet qui l'a lancé, référençant un `this` pendouillant).
- `generation()` : compteur incrémenté à chaque `start()`, sert de preuve
  externe qu'un restart a eu lieu. Ne PAS utiliser `pthread_self()` pour
  ça — glibc recycle fréquemment la même adresse mémoire pour un nouveau
  thread juste après un `join()`, deux incarnations successives peuvent
  avoir le même id.

### État partagé entre les `Task` d'un même composant

La couche shm protège les échanges **entre process** (verrou CAS + checksum,
§2). À l'intérieur d'un process, les `Task` sont des threads distincts et
plusieurs membres du composant sont écrits par l'une, lus par l'autre — il faut
donc aussi synchroniser là, conformément à la règle « `std::mutex` local pour
les races intra-process » de `CLAUDE.md`.

Deux outils selon la nature de la donnée :

| Donnée | Mécanisme | Pourquoi |
|---|---|---|
| Scalaire (bool, enum, `uint32_t`, float) | `std::atomic` | lecture/écriture indivisible, sans verrou sur le chemin RT |
| Struct multi-champs, conteneur | `std::mutex` + copie locale | un atomique ne couvre pas plusieurs champs ; sans verrou le lecteur voit un état mi-ancien mi-neuf |

Appliqué à :

- **`Navigation`** — `engagedSegment_`/`armedSegment_` (`NavSegment`) sous
  `segmentsMutex_`, chaque lecteur en prenant une copie ; `lastSegmentSeq_`,
  `newSegment_` et `holdActive_` en atomiques.
- **`MissionControl`** — `plan_` sous `planMutex_` (il contient un
  `std::vector` : une réallocation pendant que `TMiSequencer` l'indexe libère le
  tampon sous le lecteur) ; `state_`, `missionComplete_`, `takeoffComplete_`,
  `takeoffTargetAlt_` et les cinq drapeaux de panne en atomiques.
- **`MavlinkInterface`** — déjà conforme avant : `AckFifo` et `PendingAckQueue`
  portent leur propre mutex. Idem `StateMachineOrderQueue` côté
  `MissionControl`.
- **`SysMonitoring`** — rien à faire : son état de défilement et ses snapshots
  de codes sont des membres **par `Task`**, jamais partagés (cf.
  `SYSMONITORING.md` §6).

**`newSegment_` se consomme par `exchange()`**, pas par un `store(false)` :
`TSeq` le pose et `TAP` le consomme, donc un simple store écraserait un drapeau
posé entre la lecture et l'écriture, et le changement de segment serait perdu.

Les lectures déchirées que §2 interdit entre process étaient donc possibles
entre threads du même composant — `TAP` pouvait calculer un guidage vers un
waypoint à moitié mis à jour (nouveau `seq`, ancienne cible).

## 4. Les trois niveaux de watchdog, en détail

### 4.1 `ComponentWatchdog` — heartbeat des Task

Thread unique par composant, poll à 100Hz max (`sleep_until`, pas de
busy-wait).

**Lancé par `start()`, jamais par le constructeur.** Deux raisons, qui étaient
deux anomalies :

1. Les `Task` s'enregistrent (`tasks_.push_back`) *après* la construction du
   watchdog. Un thread démarré dans le constructeur itérait donc `tasks_`
   pendant que le vecteur se remplissait — réallocation possible sous les pieds
   d'un autre thread, comportement indéfini.
2. Un constructeur ne peut pas échouer proprement. `launchRTThread` qui échoue
   laissait `started_` à `false` et **plus aucune `Task` n'était surveillée**,
   sans trace. `start()` renvoie un `std::expected<void, int>` porteur de
   l'`errno`.

`ComponentBase::activate()` appelle donc `localWD.start()` **avant** de démarrer
les `Task` — personne n'émet de heartbeat avant qu'il y ait quelqu'un pour
l'observer. En cas d'échec, `startupError()` porte l'`errno` ; le rôle appelant
(`src/main.cpp`) sort en code 1, `GlobalWatchdog` constate la mort de l'enfant
et applique l'escalade normale (§4.3). Aucun code de diagnostic dédié n'est
nécessaire : un composant dont le thread temps réel ne démarre pas est
inutilisable, et mourir est l'information la plus honnête. C'est aussi ce qui
rend visible l'oubli de `setcap cap_sys_nice+ep` après un rebuild
(cf. `sitl/SITL.md` §1) — auparavant le process tournait sans rien exécuter.

**Priorité 90, cœur dédié.** `RT_PRIORITY_COMPONENT_WATCHDOG` est fixé
strictement au-dessus de la `Task` la plus prioritaire du système
(`TSafetyMonitor`, 85). Sans cette marge, un watchdog partageant un cœur avec la
`Task` qu'il surveille pourrait en être affamé : en `SCHED_FIFO`, une priorité
simplement *égale* ne préempte pas. Le cas s'est présenté — `Navigation::TFS`
est à 80, soit l'ancienne priorité du watchdog.

`CORE_COMPONENT_WATCHDOG` (cœur 3) est volontairement **distinct** de
`ComponenConfig::CompCore`, qui épingle les `Task` du composant : les watchdogs
vivent groupés hors des cœurs de calcul. Avec la priorité 90, ce placement
redevient un choix de performance et non plus une condition de correction.

**`lastHeartbeat` : accès par `std::atomic_ref`.** Écrit par les threads de
`Task`, lu par celui du watchdog. Un membre `std::atomic` rendrait `TaskHandle`
non copiable et donc inutilisable dans un `std::vector` ; `std::atomic_ref`
(C++20) donne l'accès atomique sans toucher au type.

Pour chaque `Task` enregistrée :

```cpp
if (msBetween(t.lastHeartbeat, now) > t.timeout)
  handleDeadTask(t);
```

`handleDeadTask` maintient un **ring buffer d'horodatages de restart**
(pas un compteur absolu — convention du projet, "X restarts en Y
secondes") :

```cpp
while (!t.restartHistory.empty() &&
       msBetween(t.restartHistory.front(), now) > t.restartWindow)
  t.restartHistory.pop_front();     // purge la fenêtre glissante
t.restartHistory.push_back(now);

if (t.restartHistory.size() >= t.maxRestart)
  shutdown();          // _exit(1) — tue le process ENTIER
else
  { t.stop(); t.start(); t.lastHeartbeat = now; }  // restart simple
```

Escalade volontaire : au-delà de `maxRestart` redémarrages dans
`restartWindow`, la `Task` n'est plus relancée seule — c'est le process
entier qui est tué. C'est `GlobalWatchdog` qui le respawn, et `ComponentBase` (niveau
au-dessus) qui décide l'escalade finale (hot-start / cold-start / DEAD) à
partir de l'historique *persisté en shm*, qui survit au crash du process
contrairement au ring buffer en RAM du `ComponentWatchdog`.

`ComponentWatchdog` poll aussi les erreurs shm de tous les segments
enregistrés (`registerShmSource`), pour ne pas en rater une écrasée entre
deux accès du composant lui-même (`lastError_` côté `SharedMemoryHandler`
est *sticky* — un succès ne l'efface pas, seule la consommation le fait).

Troisième rôle, en plus de la surveillance et du poll d'erreurs : il mesure la
fréquence réelle de chaque `Task`. Les `Task` ne font qu'incrémenter un
compteur ; le watchdog tient le compteur précédent et l'instant du dernier
échantillon, calcule la cadence sur une fenêtre d'une seconde et la publie dans
`SharedSysStateMem`. Le choix de mettre le calcul ici — et non dans la `Task` —
garde la boucle de vol à un `fetch_add` relaxed, et rend l'arithmétique
mono-thread donc sans synchronisation. Détail et transport :
`SYSMONITORING.md` §13.

### 4.2 `GlobalWatchdog` — PID des process

Voir §2.5 pour le mécanisme de détection/récupération. Un détail de
robustesse à connaître avant toute modification de `monitorLoop()` : un vrai Ctrl+C
envoie `SIGINT`/`SIGTERM` à **tout le groupe de process**, pas seulement à
`GlobalWatchdog` — donc les enfants meurent en même temps que la décision
d'arrêt est prise. Sans précaution, une mort d'enfant détectée juste après
le signal (mais avant que la boucle ne revérifie `keepRunning_`)
déclencherait un respawn immédiatement suivi d'un re-kill par
`shutdown()`. Le code vérifie donc `keepRunning_` une seconde fois, juste
avant de décider de respawn, et marque l'enfant comme déjà mort sinon.

### 4.3 `ComponentBase` — décision hot/cold/DEAD

À la construction (donc à chaque (re)démarrage du process), lit
l'historique persisté dans `SharedCompMem` :

```
lastCold récent (< max_cold_start_interval) ?  → DEAD, arrêt (rien d'autre ne s'exécute)
sinon lastHot × max_hot_start récent ?          → cold-start : SICK, enregistre un cold-start
sinon pas un premier démarrage ?                → hot-start : DEGRADED, enregistre un hot-start
sinon                                            → NOMINAL, premier démarrage
```

**« Suis-je un redémarrage ? » est transmis, jamais déduit.** Un composant qui
vient de naître n'a aucun moyen de le savoir : il n'a pas de mémoire, et
l'historique shm ne peut pas servir d'indice puisque c'est lui-même qui
l'alimente. Seul `GlobalWatchdog` le sait — il vient de récolter le PID mort et
de décider le respawn. L'information descend donc par `argv` :

```cpp
// GlobalWatchdog::spawn(), chemin respawn uniquement
execl(binaryPath_, binaryPath_, role, "respawn", nullptr);

// src/main.cpp
const bool firstBoot = !(argc > 2 && std::string_view(argv[2]) == "respawn");
```

puis par `ComponenConfig::firstBoot` jusqu'au constructeur. L'absence de cet
argv vaut premier démarrage : `./drone navigation` lancé à la main pour
déboguer se comporte comme un système neuf, et les binaires SITL autonomes
(`mc_sitl`, `mv_sitl`, `sysmon_sitl`) n'ont rien à renseigner.

**Les deux sources ne peuvent pas se contredire** : `publishSharedMemory()`
ouvre avec `O_CREAT` puis fait un `new (mem) T{}`, donc chaque démarrage de
`GlobalWatchdog` re-zéroe les segments — que le précédent se soit arrêté
proprement (`shm_unlink`) ou qu'il ait été tué brutalement. Au début de chaque
vie de GW, l'historique est toujours vierge, ce qui coïncide exactement avec le
`firstBoot=true` de `forkChildren()`.

Corollaire à connaître : **l'échelle d'escalade est bornée à une vie de GW.** Un
composant qui crashe trois fois, puis un redémarrage de GW, repart de
`NOMINAL`. Défendable — un redémarrage système complet *est* un départ neuf —
mais ce n'est pas une propriété évidente à la lecture.

**`MissionControl` ne cold-start jamais.** La règle de `CLAUDE.md` est
désormais dans le code : partout où l'escalade mènerait à un cold-start, MC
passe directement `DEAD`. Sa state machine ne peut pas être resetée en
sécurité, donc un redémarrage à froid vaudrait moins qu'un arrêt franc.

Conséquence à connaître : cela vaut aussi pour le chemin « lecture d'historique
échouée », qui est pessimiste (cf. ci-dessous). Un timeout sur le verrou shm au
moment de la construction de MC le déclare donc `DEAD`.

**Sémantique de `max_hot_start`** : la branche lit
`getHotStartTs(max_hot_start - 1)`, soit le N-ième hot-start le plus récent. Il
faut donc N hot-starts **déjà enregistrés** pour escalader : avec la valeur par
défaut 3, trois respawns sont tolérés (`DEGRADED`) et le quatrième bascule en
cold-start (`SICK`). Même lecture pour `max_cold_start`.

### Deux pièges historiques de cette branche

Le test de « suis-je un redémarrage ? » a été faux deux fois de suite, dans les
deux sens — à connaître avant d'y retoucher.

1. **Toujours DEGRADED.** La condition était `if (getHotStartTs(0))`, or cet
   `optional` **réussit toujours** dès que l'index existe dans le tableau à
   taille fixe : un slot jamais écrit vaut `TimePoint{}` (epoch), pas "vide".
   Tout composant neuf était donc vu comme redémarré, en permanence.
2. **Jamais DEGRADED.** Le correctif a ajouté un test `!= TimePoint{}`
   (`isRecorded`), ce qui a surcorrigé : la branche était alors gardée par sa
   propre conséquence — seule elle écrivait l'historique qu'elle exigeait de
   trouver. Aucun redémarrage n'était plus jamais détecté, et `DEGRADED`,
   `SICK` et `DEAD` devenaient inatteignables en production. Les tests ne le
   voyaient pas parce qu'ils pré-remplissaient l'historique à la main.

D'où la forme actuelle, qui ne sonde plus rien. `isRecorded` reste utilisé pour
les fenêtres `lastCold`/`lastHot`, où le piège n°1 s'applique toujours.

Note : `getStart()` renvoie `{}` aussi bien sur slot vide que sur échec de
lecture shm, et `isRecorded()` écrase les deux en `false` — un timeout sur le
verrou CAS peut donc faire rater une escalade `SICK`/`DEAD`. Limite connue, cf.
`AUDIT.md` §1.

`ComponentBase<MaxTasks>` est templaté sur le nombre de `Task` du
composant dérivé (connu à la compilation). Contrainte C++ incontournable :
`ComponentBase` se construit *avant* les membres de la classe dérivée
(dont les `Task`), donc il ne peut jamais les démarrer depuis son propre
constructeur — elles n'existent pas encore. D'où le pattern
`registerTask(task_); activate();` explicite en fin de constructeur du
composant dérivé (voir `GUIDE_COMPOSANTS.md` §4) : c'est le minimum
irréductible, non automatisable plus loin côté base.

## 5. Système de codes de diagnostic (`CODES::`)

Complète `ComponentHealth` (état grossier, déjà consommé par le
`ComponentWatchdog`/l'escalade ci-dessus) par un diagnostic fin, façon
codes OBD2 automobile : `FCP001` = Flight Controller / Power / 001.

- **Dictionnaire** : `INDEX.csv` (source de vérité, éditable via
  `python3 tools/codes_editor.py` — recherche/filtre/CRUD complet) →
  génère `include/drone/generated/codes.hpp` (`CODES::Component`,
  `CODES::Category`, `CODES::Severity`, une constante `constexpr` par
  code). Régénéré automatiquement par CMake (`add_custom_command`,
  dépendance de `drone_lib`/tous les tests) si le CSV change.
- **Encodage binaire** : 4 octets — `[component:u8][category:u8][number:u16]`
  (`Code::pack()`/`unpack()`).
- **Stockage IPC**, par composant, dans `SharedSysStateMem` :
  - Mineur (`Info`/`Warning`) → ring buffer FIFO de 16 (même pattern shift
    que `SharedCompMem::recordStart`), écrasement normal.
  - Majeur (`Critical`/`Emergency`) → ensemble actif de 8 slots, jamais
    écrasé silencieusement — un code y reste jusqu'à `clearCode()`
    explicite. Nécessaire : un ring buffer unique aurait pu laisser un
    warning mineur éclipser une urgence encore active.
  - `raiseCode(code)`/`clearCode(code)` routent automatiquement vers le
    bon stockage en cherchant la sévérité dans `CODES::kTable` — l'appelant
    n'a jamais à choisir.

## 6. Limites connues

- `ComponentBase::restore()` reste un no-op par défaut — aucun composant
  n'a encore d'état réel à réhydrater après un hot-start.
- La règle "`MissionControl` ne fait jamais de cold-start" (`CLAUDE.md`)
  n'est pas codée dans `ComponentBase` — aucune logique spécifique par
  composant aujourd'hui, la décision hot/cold/DEAD (§4.3) est uniforme pour
  les 5 composants. `MissionControl` existe et tourne (`GlobalWatchdog::
  children_` est un `std::array<ChildProc, 5>`, les 5 domaines sont forkés
  — voir §1.1), seule la règle de non-cold-start n'a pas encore de
  traitement différencié.
- **Saturation des slots de codes majeurs** : `raiseMajor()` dispose de
  `kMajorCodeSlots` (8) emplacements par composant. Si les 8 sont déjà
  occupés par des codes distincts encore actifs, le 9e code majeur est
  **perdu** — pas de repli, pas d'écrasement (contrairement au ring des
  mineurs). Choix assumé : un composant qui accumule plus de 8 pannes
  critiques simultanées a un problème plus grave que la perte du 9e code, et
  tout repli "élégant" (écraser le plus ancien, compteur de débordement)
  revient à masquer l'une des 8 pannes déjà présentes.
- La bande passante réseau (codes courts 1 octet pour la télémétrie
  wfb-ng) n'a pas de mécanisme dédié — c'est un problème de sérialisation
  côté `Monitoring`→GCS, pas de l'IPC, volontairement pas traité ici.
