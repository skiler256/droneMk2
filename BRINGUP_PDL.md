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

---

## 10. Journal de bring-up — 19 octobre 2026

Première session de mise en service des liaisons. Les acquis sont réels, le
lien n'est pas encore opérationnel. Ce qui suit évite de refaire le chemin.

### Acquis

**Noyau du Pi reconstruit en 6.12.113 PREEMPT_RT.** L'arbre du noyau RT
précédent (6.6.78-rt52) avait été supprimé, rendant toute compilation de
module impossible. Depuis Linux 6.12, `PREEMPT_RT` est dans mainline :
aucun correctif externe n'est nécessaire, c'est une simple option de
configuration. L'arbre est conservé sur la machine de développement, où la
place ne manque pas — le Pi n'a que 3,4 Go libres, ce qui interdit d'y
garder un arbre de compilation.

`CONFIG_IKCONFIG_PROC` est désormais activé : `/proc/config.gz` existe, la
configuration du noyau ne peut plus être perdue.

Tous les périphériques sont opérationnels sur ce noyau, le logiciel de vol
compile sans erreur, et la suite passe à 12/12 avec **18/18 tests temps
réel exécutés, aucun ignoré**.

### Bug du pilote RTL8812AU — corrigé

`os_dep/linux/ioctl_cfg80211.c`, fonction
`cfg80211_rtw_set_monitor_channel` : la garde de version déclare le
paramètre `struct net_device *dev` à partir de 6.13, alors que le noyau
l'a introduit en **6.12**. Sur tout noyau 6.12, le pilote lit `dev` comme
s'il s'agissait de `chandef`, déréférence un pointeur aberrant et provoque
un *Oops* qui laisse le verrou `RTNL` pris — la machine répond encore au
ping mais plus aux connexions SSH.

Correctif : remplacer `KERNEL_VERSION(6, 13, 0)` par
`KERNEL_VERSION(6, 12, 0)`. **À remonter au dépôt amont**, tout noyau 6.12
est concerné.

### Puissance d'émission — paramètre indispensable

Le module expose `rtw_tx_pwr_idx_override` (0–63). **Sa valeur par défaut
de 0 laisse l'étage radio muet** : la couche logicielle compte les paquets
émis, le matériel ne transmet rien. Symptôme trompeur — les compteurs
`TX_ANT` de `wfb_tx` montent normalement.

```
modprobe 88XXau_wfb rtw_tx_pwr_idx_override=30
```

Avec ce paramètre, les trames ont été capturées côté station sol à
-36 dBm, signature `57:42:75` (« WBu ») et `link_id` encodé dans l'adresse
MAC. **L'émission radio est donc prouvée.**

Attention : `iw` affiche alors `txpower -30 dBm`, et `-63 dBm` si l'indice
vaut 63. Ce n'est pas une mesure de puissance mais l'indice brut recopié
dans le champ. Ne pas en tirer de conclusion.

### Point bloquant restant

La liaison est **intermittente**. La même configuration capture les trames
à un essai et rien au suivant, et le récepteur de la station sol a fini par
ne plus capturer même le trafic ambiant après un rechargement du module.
`wfb_rx` n'a jamais pu établir de session, ce qui est cohérent : sans flux
continu, l'échange de clés de session n'aboutit pas.

Deux causes possibles, non départagées :

- **dégradation du pilote après un temps en mode monitor** — la seule
  liaison réussie a eu lieu immédiatement après un rechargement des deux
  modules ;
- **canal non appliqué** : un rechargement détaché a laissé le Pi sur le
  canal 116 en 5 GHz alors que la commande demandait le canal 11. Toujours
  vérifier le canal des deux côtés **juste avant** le test, jamais le
  supposer.

### Pistes non explorées

- `-M 0` : indice MCS d'injection, le défaut `-M 1` peut être refusé.
- `-f rts` : type de trame, prévu par wfb-ng pour les puces récalcitrantes.
- Rendre le paramètre persistant via `/etc/modprobe.d/` plutôt que de le
  passer à la main à chaque chargement.

### Méthode

Deux pièges rencontrés, à ne pas refaire :

- **`shutdown -r +N` est un mauvais filet de sécurité** : il crée
  `/run/nologin` et interdit toute connexion non-root pendant le décompte.
  Utiliser `systemd-run --on-active=N systemctl reboot`, qui ne bloque
  rien et s'annule proprement.
- **Un module installé en deux exemplaires** (`.ko` et `.ko.xz`) fait
  charger le mauvais : le noyau préfère la version compressée. Supprimer
  l'ancienne avant `depmod`.

Le journal persistant (`Storage=persistent` dans
`/etc/systemd/journald.conf.d/`) a été activé sur les deux machines. C'est
lui qui a permis de lire la trace de l'*Oops* et d'identifier la fonction
fautive ; sans lui, le diagnostic serait resté une conjecture.
