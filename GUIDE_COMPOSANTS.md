# Guide : IPC et construction d'un composant

> Complète `Design.md` (vue d'ensemble architecture) et `CLAUDE.md` (contexte
> projet). Ce doc explique comment *utiliser* la couche IPC et comment
> câbler un nouveau composant de A à Z, avec des exemples tirés du code
> réel (`Navigation`/`SensorFusions`).

## 1. Vue d'ensemble de l'IPC

Chaque segment de mémoire partagée est un couple **struct de données +
handler** :

- La struct (`SharedNavMem`, `SharedSFMem`, `SharedSysStateMem`) contient un
  `SharedMemory mem` (métadonnées : verrou, checksum) + le payload propre au
  composant.
- Le handler (`SharedNavMemHandler`, etc.) hérite de `SharedMemoryHandler`
  (via `SharedCompMemHandler` pour les segments mono-composant) et expose
  des méthodes d'accès typées (`getBlip()`, `setHealth()`...).

**Deux familles de segments :**

| Type | Écrivains | Exemple |
|---|---|---|
| `SharedCompMem` | un seul composant, toute sa vie | `SharedNavMem`, `SharedSFMem` |
| `SharedSysStateMem` | tous les composants (un slot par `ComponentID`) | l'état de santé global |

**Le verrou** : un seul `std::atomic<ComponentID> owner` par segment (CAS
`Free → id` pour acquérir). Toute lecture ET toute écriture passent par le
même verrou (pas de RW-lock séparé) — voir `SharedMemory.hpp`.

**Règle d'or sur `id`** : le `id` passé à un accès shm est **l'identité
réelle de l'appelant** (utilisée comme jeton de verrou et pour la
récupération sur crash par `GlobalWatchdog`), pas forcément le sujet
consulté. Exemple : `SharedSysStateMemHandler` prend un `ownId` au
constructeur (ton identité) distinct du `id` passé à `getHealth(id)` (de
qui tu veux la santé). Si tu construis un nouveau handler multi-écrivains,
respecte ce pattern — verrouiller avec l'id du sujet plutôt que le tien
peut créer un deadlock permanent si tu crashes en tenant le verrou.

## 2. Créer un nouveau segment shm pour ton composant

Copie le pattern de `SharedNavMem.hpp` :

```cpp
// include/drone/Components/MonComposant/SharedMonComposantMem.hpp
#pragma once
#include "drone/Components/SharedCompMem.hpp"
#include "drone/types.hpp"

struct SharedMonComposantMem {
  SharedCompMem compMem; // historique restart, verrou — toujours en premier

  struct payload {
    // tes données. Types forts (TYPES::Meters, etc.), pas de float nu.
  };

  payload data;
};

class SharedMonComposantMemHandler : public SharedCompMemHandler {
public:
  explicit SharedMonComposantMemHandler(TYPES::ComponentID id, TYPES::Us timeout,
                                        SharedMonComposantMem &mem)
      : SharedCompMemHandler(mem.compMem, id, timeout), mem_ref_(mem) {};

  // Accesseurs publics typés — PAS de getData/setData bruts exposés.
  // Lecture : getData(id_, timeout_, ...) — repairOnCorrupt=false (défaut).
  // Écriture : setData(id_, timeout_, ...) — répare une corruption détectée.

private:
  SharedMonComposantMem &mem_ref_;

  // Doit couvrir payload ET l'historique de restart hérité (sinon une
  // corruption de l'historique ne sera jamais détectée) :
  uint32_t computeChecksum() {
    return UTILITIES::crc32(mem_ref_.data) ^ historyChecksum();
  };

  // N'autorise que le propriétaire à se reconstruire lui-même.
  void reset(TYPES::ComponentID id) {
    if (id == id_) {
      mem_ref_.data = SharedMonComposantMem::payload{};
      sanitizeHistory(comp_.HotStartHistory);
      sanitizeHistory(comp_.ColdStartHistory);
      mem_.checksum = computeChecksum();
    }
  };
};
```

Points obligatoires (sinon ça compile mais silencieusement faux) :
- `computeChecksum()` doit hacher `historyChecksum()` en plus de ton
  payload, sinon la corruption de l'historique de restart est invisible.
- `reset(id)` doit vérifier `id == id_` (n'importe quel composant ne doit
  pas pouvoir reconstruire tes données).
- Pour un segment multi-écrivains façon `SharedSysStateMem` (pas
  mono-composant), `reset(id)` doit être **segmenté** : ne reconstruire que
  le slot de `id`, jamais tout le tableau (sinon tu effaces les données des
  autres). Voir `SharedSysStateMem.hpp` pour le pattern exact.

## 3. Écrire une Task

Une `Task` est une boucle temps réel (thread `SCHED_FIFO` dédié) supervisée
par un `ComponentWatchdog`. Pattern (`Navigation.hpp`) :

```cpp
class MonComposant; // forward decl

class MaTask : public Task {
public:
  explicit MaTask(MonComposant &component, TaskConfig config, ComponentWatchdog &WD)
      : Task(config, WD), comp(component) {};

private:
  MonComposant &comp;
  void loop() override; // définie dans le .cpp
};
```

```cpp
void MaTask::loop() {
  // Appelée à Tconfig.loopFrequency. Doit rester rapide — pas de sleep,
  // pas d'I/O bloquante longue (ça retarde le heartbeat, cf. §6).
}
```

`TaskConfig` à fournir à la construction :

```cpp
TaskConfig{
    .id = 0,                          // TaskID unique DANS ce composant
    .RTpriority = 70,                 // SCHED_FIFO, 1-99 (0 est invalide !)
    .core = config.CompCore,
    .loopFrequency = TYPES::Hz{100},  // fréquence de loop()
    .timeout = TYPES::Ms{200},        // heartbeat manqué au-delà = "morte"
    // .maxRestart = 3, .restartWindow{60000} — défauts OK dans la plupart des cas
}
```

`maxRestart`/`restartWindow` : si la Task meurt (timeout dépassé) plus de
`maxRestart` fois dans `restartWindow`, `ComponentWatchdog` tue le **process
entier** (`exit(1)`) plutôt que de continuer à la relancer seule —
`GlobalWatchdog` respawn le process, et `ComponentBase` (voir §4) décide
hot-start/cold-start/DEAD au prochain démarrage.

## 4. Écrire le composant

Hérite de `ComponentBase<MaxTasks>` (`MaxTasks` = nombre de `Task` de ton
composant, connu à la compilation — `Navigation`/`SensorFusions` en ont 1
chacun aujourd'hui). `ComponentBase` gère le cycle
hot-start/cold-start/DEAD et possède le `ComponentWatchdog` local
(`localWD`, `protected`) :

```cpp
// MonComposant.hpp
class MonComposant : public ComponentBase<1> { // 1 Task ici
public:
  explicit MonComposant(ComponenConfig config, SharedSysStateMemHandler &sysState,
                        SharedMonComposantMemHandler &mem);

  SharedMonComposantMemHandler &mem_;

private:
  MaTask task_;
};
```

```cpp
// MonComposant.cpp
MonComposant::MonComposant(ComponenConfig config, SharedSysStateMemHandler &sysState,
                           SharedMonComposantMemHandler &mem)
    : ComponentBase<1>(config, mem, sysState), mem_(mem),
      task_(*this,
            {.id = 0, .RTpriority = 70, .core = config.CompCore,
             .loopFrequency = TYPES::Hz{100}, .timeout = TYPES::Ms{200}},
            localWD)
{
  registerTask(task_); // une fois par Task membre, AVANT activate()
  activate();          // décide hot/cold/DEAD, restore()+démarre les tasks
};
```

`registerTask`/`activate()` centralisent le dispatch hot-start/cold-start
dans `ComponentBase` — plus besoin de répéter un `if (hotstart) {...} else
if (coldstart) {...}` dans chaque composant. `activate()` ne démarre rien
si le composant est DEAD. Deux hooks `protected virtual` à override si
besoin :
- `restore()` — no-op par défaut, à définir si ton composant a un état à
  réhydrater après un hot-start (persisté par toi dans ta propre shm).
- `init()` — par défaut démarre toutes les tasks enregistrées via
  `registerTask` ; à override seulement si tu as besoin de plus que ça.

`ComponenConfig` :

```cpp
ComponenConfig{
    .id = TYPES::ComponentID::MonComposant, // ajouter la valeur dans types.hpp
    .CompCore = 0,
    // .max_cold_start=1, .max_hot_start=3, .*_interval{60000} — défauts OK
}
```

## 5. Câbler dans `main.cpp` / `GlobalWatchdog`

1. Ajoute ton `ComponentID` dans `types.hpp` (avant `Count`).
2. Ajoute un chemin shm dans `GlobalWatchdog.hpp` (`kMonComposantShmPath`) et
   publie-le dans `publishAll()`.
3. Ajoute un fork dans `forkChildren()` (`spawn(ComponentID::MonComposant, "moncomposant")`)
   et étends `children_` (actuellement `std::array<ChildProc, 2>`).
4. Dans `main.cpp`, ajoute une fonction `runMonComposant()` (copie
   `runNavigation()`) : `attachSharedMemory` sur chaque segment nécessaire,
   construis les handlers puis le composant, boucle `while(true) sleep(2)`.
5. Ajoute le dispatch dans `main()` : `if (role == "moncomposant") return runMonComposant();`.

## 6. Pièges déjà rencontrés (lis avant de coder)

- **Priorité `SCHED_FIFO` 0 = invalide.** `RTpriority` doit être ≥ 1, sinon
  `pthread_create` échoue silencieusement (`EINVAL`) et ta Task ne tourne
  jamais sans que rien ne le signale bruyamment.
- **Un composant démarre toujours en écrivant sa propre santé** via
  `sysState_.setHealth(...)` — c'est `ComponentBase` qui le fait pour toi
  dans son constructeur, ne le refais pas dans le tien. `setHealth`/
  `raiseCode`/`clearCode` sont toujours des self-reports (pas de paramètre
  `id` — le handler connaît déjà sa propre identité) ; seules les lectures
  (`getHealth(id)`, `getMinorCodes(id)`...) prennent un `id` explicite,
  puisqu'on peut légitimement consulter un AUTRE composant.
- **`restore()` reste un no-op par défaut** — la réhydratation réelle après
  un hot-start n'existe pour aucun composant aujourd'hui (`Navigation`,
  `SensorFusions` n'ont pas d'état à persister pour l'instant). À toi de
  l'implémenter si ton composant en a besoin.
- **`loop()` doit rester rapide.** Un appel bloquant long (I/O, sleep)
  retarde le heartbeat suivant et peut déclencher un restart injustifié par
  le watchdog — sépare le travail lent dans un thread séparé si besoin.
- **Ne jamais faire de cold-start sur `MissionControl`** (quand il
  existera) — sa state machine ne peut pas être resetée en sécurité selon
  `CLAUDE.md`. Ce n'est pas encore appliqué dans le code (pas de logique
  spécifique par composant dans `ComponentBase` aujourd'hui), à garder en
  tête si tu touches à ce composant.

## 7. Tester

Regarde `tests/test_watchdog.cpp` pour des exemples directement
réutilisables :
- `ComponentBaseTest.*` — tester la décision hot/cold/DEAD sans process réel
  (juste des handlers + `ComponentBase<0>` construits en mémoire locale,
  sans Task à enregistrer).
- `TaskRealExecutionTest.*` — tester la cadence réelle d'une Task (nécessite
  `SCHED_FIFO`, se skip proprement sinon — voir `rtSchedulingAvailable()`).

Pour tester le cycle de vie complet en process réel (fork/exec via
`GlobalWatchdog`), regarde `tests/test_global_watchdog.cpp` et son binaire
compagnon `tests/helpers/stub_child.cpp`.

`cd build && cmake --build . -j4 && ctest -v`
