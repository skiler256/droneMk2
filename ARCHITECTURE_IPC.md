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
                    ┌────────▼──┐   ┌──▼─────────┐
                    │ Navigation │   │SensorFusion│  ... un process par composant
                    │  (process) │   │ (process)  │
                    └─────┬──────┘   └──────┬─────┘
                          │                 │
                          │  ComponentBase  │
                          │  ├─ ComponentWatchdog (thread RT local)
                          │  └─ N × Task (thread RT chacune)
                          │                 │
                          └────────┬────────┘
                                   │ lit/écrit via handlers
                          ┌────────▼────────┐
                          │  Mémoire partagée │  SharedNavMem, SharedSFMem,
                          │  (POSIX shm_open) │  SharedSysStateMem...
                          └───────────────────┘
```

Trois niveaux de supervision, chacun avec une responsabilité précise et
**aucun recouvrement** :

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
- **Le "sujet"** (quel composant on lit/modifie) peut être complètement
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
  tourne juste jamais, sans erreur visible si on ne vérifie pas le retour.
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

## 4. Les trois niveaux de watchdog, en détail

### 4.1 `ComponentWatchdog` — heartbeat des Task

Thread unique par composant (lancé par `ComponentBase`), poll à 100Hz max
(`sleep_until`, pas de busy-wait). Pour chaque `Task` enregistrée :

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
`restartWindow`, on ne relance plus la `Task` seule — on tue le process
entier. C'est `GlobalWatchdog` qui le respawn, et `ComponentBase` (niveau
au-dessus) qui décide l'escalade finale (hot-start / cold-start / DEAD) à
partir de l'historique *persisté en shm*, qui survit au crash du process
contrairement au ring buffer en RAM du `ComponentWatchdog`.

`ComponentWatchdog` poll aussi les erreurs shm de tous les segments
enregistrés (`registerShmSource`), pour ne pas en rater une écrasée entre
deux accès du composant lui-même (`lastError_` côté `SharedMemoryHandler`
est *sticky* — un succès ne l'efface pas, seule la consommation le fait).

### 4.2 `GlobalWatchdog` — PID des process

Voir §2.5 pour le mécanisme de détection/récupération. Un détail de
robustesse à connaître si tu modifies `monitorLoop()` : un vrai Ctrl+C
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
sinon au moins un hot-start déjà enregistré ?   → hot-start : DEGRADED
sinon                                            → NOMINAL, premier démarrage
```

Piège déjà rencontré : `getHotStartTs(i)`/`getColdtStartTs(i)` renvoient un
`optional` qui **réussit toujours** dès que l'index existe dans le tableau
à taille fixe — un slot jamais écrit vaut `TimePoint{}` (epoch), pas
"vide". Sans vérifier explicitement `!= TimePoint{}`, un composant tout
neuf serait vu comme DEGRADED en permanence (bug réel, corrigé).

`ComponentBase<MaxTasks>` est templaté sur le nombre de `Task` du
composant dérivé (connu à la compilation). Contrainte C++ incontournable :
`ComponentBase` se construit *avant* les membres de la classe dérivée
(dont les `Task`), donc il ne peut jamais les démarrer depuis son propre
constructeur — elles n'existent pas encore. D'où le pattern
`registerTask(task_); activate();` explicite en fin de constructeur du
composant dérivé (voir `GUIDE_COMPOSANTS.md` §4) : c'est le minimum
irréductible qu'on ne peut pas automatiser plus loin côté base.

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

## 6. Ce qui n'est PAS encore fait (au 2026, dernière mise à jour de ce doc)

- `ComponentBase::restore()` reste un no-op par défaut — aucun composant
  n'a encore d'état réel à réhydrater après un hot-start.
- La règle "`MissionControl` ne fait jamais de cold-start" (`CLAUDE.md`)
  n'est pas codée dans `ComponentBase` — pas de logique spécifique par
  composant aujourd'hui, `MissionControl` n'existe pas encore.
- `GlobalWatchdog::children_` est un `std::array<ChildProc, 2>` figé
  (Navigation + SensorFusion) — à généraliser au 3e composant.
- La bande passante réseau (codes courts 1 octet pour la télémétrie
  wfb-ng) n'a pas de mécanisme dédié — c'est un problème de sérialisation
  côté `Monitoring`→GCS, pas de l'IPC, volontairement pas traité ici.
