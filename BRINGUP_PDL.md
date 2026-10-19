# Bring-up du lien principal (PDL / wfb-ng) — brief pour la session sur le Pi

Document de passation, destiné à une session Claude Code ouverte **sur le
Raspberry Pi du drone**. Il porte le contexte, l'état du code et l'ordre des
opérations. Le travail d'intégration GCS et de dépôt se fait dans une autre
session, sur la machine de développement.

---

## 1. Contexte

Drone autonome, Raspberry Pi 5 sous PREEMPT_RT, logiciel de vol C++23, FC
sous ArduPilot en MAVLink. Lire `CLAUDE.md` avant toute intervention : il
porte les décisions d'architecture déjà prises et la façon de travailler
attendue.

Le logiciel de vol est complet et testé hors matériel (12 suites GTest). Ce
qui manque pour voler est la liaison radio réelle : jusqu'ici la télémétrie
passe par UDP loopback entre le drone et un GCS tournant sur la même
machine.

Deux liens sont prévus :

| | Rôle | Matériel | État logiciel |
|---|---|---|---|
| **PDL** | télémétrie complète + vidéo | wfb-ng 433 MHz, RTL8812AU côté sol | **objet de ce bring-up** |
| **SDL** | secours, sous-ensemble léger | CC1101 sub-GHz SPI | pas commencé |

---

## 2. Ce qui est déjà écrit

`WFB_NG_Driver` existe et compile (`src/SystemMonitoring/WFB_NG_Driver.cpp`,
en-tête sous `include/drone/Components/SystemMonitoring/Driver/`). Design
détaillé : `SYSMONITORING.md` §15.

Le point à comprendre avant de toucher au code : **wfb-ng est un transport
UDP**. `wfb_tx` lit sur un port UDP local et diffuse, `wfb_rx` reçoit et
réécrit sur un port UDP local. Le logiciel de vol n'ouvre donc aucun socket
RF. `WFB_NG_Driver` hérite de `UdpTelemetryDriver` et n'ajoute que la lecture
des statistiques de wfb-ng, pour que le drone connaisse la qualité de son
lien montant.

Quatre tests couvrent cette lecture contre un faux serveur TCP local
(`tests/test_drivers.cpp`, filtre `WfbNg*`). Ils valident le découpage en
lignes et la dégradation sans service — **pas le format réel**.

---

## 3. Diagnostic d'abord

Rien ne doit être modifié avant d'avoir ces réponses.

### Sur le Pi

```bash
which wfb_tx wfb_rx
systemctl status 'wfb@*' 2>/dev/null | head -20
ls /sys/class/net                      # interface wifi dediee presente ?
iw dev                                 # mode monitor possible ?
```

### Sur la station sol

```bash
lsmod | grep 88XXau                    # module DKMS charge ?
iw dev
systemctl status 'wfb@*' 2>/dev/null | head -20
```

### Résultat attendu et conduite à tenir

| Constat | Suite |
|---|---|
| wfb-ng installé et actif des deux côtés | aller au §4 |
| wfb-ng absent | l'installer avant tout : c'est un prérequis, pas du code |
| adaptateur RTL8812AU absent ou module non chargé | problème matériel/DKMS côté sol, à régler d'abord |

---

## 4. Trouver le socket de statistiques

**C'est le seul point réellement ouvert du code.** `parseStatsLine()` cherche
les champs `"rssi"`, `"lost"` et `"recv"` dans des lignes JSON, par analogie
avec ce que `wfb-cli` affiche. Ces noms **n'ont pas été vérifiés contre une
sortie réelle**.

À déterminer sur le Pi :

1. Sur quel port le service expose ses statistiques (chercher dans la
   configuration de wfb-ng et avec `ss -tlnp`).
2. À quoi ressemble une ligne réelle. Le plus simple :

   ```bash
   nc 127.0.0.1 <port> | head -5
   ```

3. Corriger les noms de champs dans `parseStatsLine()` si besoin, et adapter
   les tests avec une ligne réelle en exemple.

Si le format n'est pas du JSON par ligne, le dire plutôt que de forcer : la
fonction est isolée précisément pour être remplacée.

**Contrainte à respecter** : la lecture ne doit jamais bloquer. `poll()` est
appelé depuis une boucle temps réel en SCHED_FIFO. Le socket est non
bloquant, `connect()` compris, et la tentative est limitée à une par seconde.

---

## 5. Brancher le driver

Aujourd'hui `src/main.cpp` instancie deux `UdpTelemetryDriver` sur des ports
loopback de développement (5601/5602 pour le principal, 5611/5612 pour le
secours). Le passage au lien réel consiste à :

1. remplacer le driver du lien principal par `WFB_NG_Driver`, avec les ports
   qu'attend wfb-ng et le port de statistiques trouvé au §4 ;
2. laisser le lien de secours en `UdpTelemetryDriver` tant que le CC1101
   n'est pas écrit.

`SysMonitoring` ne connaît que les interfaces : le remplacement ne le touche
pas. C'était l'intention du découpage, le vérifier plutôt que le supposer.

---

## 6. Compilation sur le Pi

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build
```

Deux pièges connus :

- **`setcap` est perdu à chaque édition de liens.** Sans lui, les threads
  SCHED_FIFO ne démarrent pas et les tests temps réel se *skippent* en
  silence — la suite reste verte alors que les tests qui comptent n'ont pas
  tourné. Après chaque build :

  ```bash
  sudo setcap cap_sys_nice+ep build/test_watchdog
  sudo setcap cap_sys_nice+ep build/drone
  ./build/test_watchdog            # verifier : zero SKIPPED
  ```

- **La cross-compilation est documentée mais inexistante.** `CMakeLists.txt`
  mentionne `cmake/arm64.cmake`, ce fichier n'est pas dans le dépôt. Compiler
  sur le Pi, ou écrire le fichier de toolchain.

---

## 7. Validation

Dans l'ordre, chaque étape avant la suivante :

1. **Transport** — le GCS reçoit la télémétrie par la radio et non par
   loopback. Le tableau de bord se remplit, les panneaux ne passent pas en
   demi-teinte (une teinte atténuée signale un groupe périmé, c'est le
   mécanisme normal de fraîcheur).
2. **Montant** — une commande envoyée depuis le GCS arrive au drone.
   `HEARTBEAT_GCS` doit arriver régulièrement, sinon les codes
   `gcs_main_link_lost` se lèvent.
3. **Qualité** — `rssiDbm()` et `lossRatio()` renvoient des valeurs
   plausibles. Les éloigner physiquement doit faire chuter le RSSI.
4. **Perte franche** — couper l'alimentation de la radio sol et vérifier que
   le drone lève bien `gcs_main_link_lost`, puis le comportement de repli.

Sur le point 4, attention : couper le lien pendant un vol réel déclenche les
procédures d'urgence. À faire au sol, moteurs désarmés, la première fois.

---

## 8. Ce qui n'est pas dans ce périmètre

- **Le CC1101 (SDL)** : driver SPI entier à écrire, rien n'existe.
  `tools/hw_debug/CC1101_test.py` ne lit que `PARTNUM`/`VERSION` et confirme
  le bus, pas le fonctionnement radio.
- **La vidéo** : `VideoTask` interroge bien `pollFrame()` mais ne relaie
  nulle part, et wfb-ng transporte la vidéo dans un flux distinct de la
  télémétrie. Chantier séparé.
- **Le RSSI descendant** : il se mesure sur le récepteur de la station sol,
  côté `server.js`, pas sur le drone. Décision déjà prise.
- **Le plan de vol** : jamais transmis au GCS, c'est un choix d'architecture
  en attente et non un manque de ce lien.

---

## 9. Rappels de méthode

Repris de `CLAUDE.md`, parce qu'ils comptent particulièrement ici :

- L'architecture appartient à Jules. Sur une question de conception,
  présenter les options et attendre l'arbitrage — ne jamais combler un vide
  d'architecture parce que c'est plus rapide.
- Signaler tout bug, manque ou trou de sécurité plutôt que de corriger en
  silence.
- Diffs petits et relisibles, une pause entre chaque morceau.
- Commentaires courts : une à deux lignes, le raisonnement va dans le `.md`.
- Documentation au registre impersonnel, sans « tu » ni « on ».
