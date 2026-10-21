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

**Résolu — c'était NetworkManager.** Il gérait les deux interfaces : il les
scannait, les remettait en mode géré et leur changeait de canal. D'où
l'intermittence, qui avait toutes les apparences d'une dégradation du
pilote : la liaison marchait juste après un rechargement de module, avant
que NetworkManager ne reprenne la main.

```bash
nmcli dev set <interface> managed no
```

C'est une étape que l'installeur officiel de wfb-ng effectue lui-même, et
que j'avais sautée en reproduisant la séquence de mémoire plutôt qu'en
lisant leur code.

Diagnostic : capturer avec `tcpdump` sur un canal où l'on **sait** qu'il y a
du trafic (celui de la box). Si rien n'est capturé là, le récepteur est en
cause et non l'émetteur d'en face.

Un second piège a brouillé la piste : un rechargement de module détaché
avait laissé le Pi sur le canal 116 en 5 GHz alors que la commande
demandait le canal 11. **Toujours vérifier le canal des deux côtés juste
avant le test**, jamais le supposer.

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

---

## 11. Lien secondaire (SDL / CC1101) — 20 octobre 2026

Première mise en service du lien de secours. Le matériel est validé des deux
côtés et les paquets passent, mais le taux d'erreur reste élevé.

### Matériel validé

| | Drone (Pi, SPI direct) | Station sol (ESP32) |
|---|---|---|
| Identité | `PARTNUM 0x00`, `VERSION 0x14` | idem |
| Bus SPI | lectures répétées cohérentes | **3000 transactions, 0 erreur**, jusqu'à 4 MHz |
| Registres relus | 0 divergence sur 34 | 0 divergence sur 47 |
| Plancher de bruit | -101 dBm | -101 dBm |
| Émission | `MARCSTATE` atteint TX, FIFO vidée | idem |

Câblage de l'ESP32, qui ne figurait nulle part : **SCK 14, MISO 35, MOSI 13,
CS 4**. MISO sur 35 plutôt que sur le 12 par défaut de HSPI, parce que le
GPIO12 est une broche de *strapping* qui empêche le démarrage si elle est
tirée au niveau haut. À reporter dans `mapingESP.csv`.

### Cause racine : décalage de quartz entre les deux modules

**C'est le point à retenir.** Les deux modules sont vendus pour un quartz de
26 MHz, mais leurs fréquences réelles diffèrent d'environ **184 ppm**.
Réglée sur 433,80 MHz, la station sol émettait physiquement à **433,72** —
80 kHz plus bas.

Avec une déviation GFSK de 5 kHz, un décalage de 80 kHz vaut seize fois la
déviation : le démodulateur ne peut rien distinguer. Élargir le filtre de
réception n'y change rien, puisque le problème n'est pas la bande passante
mais la position du signal dans le discriminateur.

**Correctif** : décaler la consigne de la station sol de 80 kHz vers le
haut. Configurée sur 433,88 MHz (`FREQ = 0x10 B0 0C`), elle émet
physiquement sur 433,80. Cela corrige les **deux sens** à la fois : son
récepteur, décalé du même biais, écoute alors lui aussi physiquement
433,80.

Un décalage de quartz est indétectable autrement : les deux bouts se parlent
entre eux si on les règle pareil, simplement hors de la fréquence voulue.
Ici ils ne se parlaient même pas, chacun ayant son propre biais.

### Méthode de mesure — le balayage de fréquence

L'outil qui a tout débloqué : `tools/hw_debug/cc1101_sweep.py` règle `FREQ`
pas à pas et lit le `RSSI`, ce qui localise une porteuse sans analyseur de
spectre.

Deux pièges rencontrés, les deux fois des erreurs de ma part :

- **La résolution est celle du filtre de réception, pas du pas.** Avec un
  filtre de 101 kHz, un signal apparaît sur plusieurs pas voisins et la
  position lue peut être fausse de 100 kHz. Pour mesurer une fréquence,
  utiliser le filtre le plus étroit possible (58 kHz, `MDMCFG4 = 0xF8`).
- **Toujours balayer émetteur éteint d'abord.** La bande 433 MHz est
  occupée — stations météo, télécommandes de portail. Un pic n'est à soi
  que s'il disparaît quand on coupe. J'ai conclu une fois sans ce contrôle
  et je me suis trompé.

### Antennes

Les antennes de 40 mm livrées avec les modules sont inadaptées : la longueur
d'onde à 433 MHz vaut 693 mm, un quart d'onde en fait **173**.

Mesuré sur le banc : un brin de 173 mm seul n'a rien apporté (2 dB, dans le
bruit). **L'ajout d'un second brin de même longueur relié à la masse,
perpendiculaire, a rapporté 17 dB** — de -57,5 à -40,5 dBm. Un quart d'onde
sans contrepoids ne rayonne pas ; il lui faut un plan de masse ou un second
brin pour former un dipôle.

La station sol a toujours son antenne d'origine : **lui appliquer le même
traitement est le prochain geste**, les pertes des deux antennes
s'additionnant dans le bilan de liaison.

### Configuration retenue

GFSK, 433,80 MHz physique, 9,6 kbit/s, déviation 10,3 kHz (indice de
modulation 2), filtre de réception 58 kHz, préambule 16 octets, synchro
`0x1234` sur 30/32 bits, CRC matériel, longueur variable, 10 dBm.

Trois réglages ont une raison précise :

- **9,6 kbit/s et non 38,4** : la sensibilité gagne environ 9 dB en
  descendant en débit et en resserrant le filtre. Le SDL ne porte que 19
  paquets courts et c'est le lien de secours : la robustesse prime.
- **Synchro sur 30/32 bits et non 16/16.** Une synchro courte fait
  déclencher le récepteur sur le bruit ; il sature sa FIFO, passe son temps
  à la purger et **ne peut plus émettre**. Testé : 210 fausses détections
  et 74 émissions refusées en 8 secondes.
- **Indice de modulation 2.** Une déviation large (indice 5) avait été
  essayée pour absorber le décalage de quartz ; une fois celui-ci corrigé
  elle dégrade la démodulation.

### Deuxième cause : lecture de la FIFO avant la fin du paquet

Défaut de mon propre code de réception, dans le pilote comme dans les
scripts de banc. Le CC1101 **remplit sa FIFO au fur et à mesure** que le
paquet arrive. Lire dès qu'un octet est disponible, puis demander
`longueur + 2` octets, revient à lire du vide à la suite des données.

À 9,6 kbit/s un paquet de 7 octets met ~6 ms à arriver, pour une boucle
d'interrogation à 5 ms : on tombe presque systématiquement en plein milieu.
Symptômes : longueurs aberrantes, CRC faux, et quelques succès quand le
hasard aligne les instants.

**Correctif** : après avoir lu l'octet de longueur, attendre que
`RXBYTES >= longueur + 2` avant de lire la charge utile, avec une échéance
pour ne pas bloquer. Sans ligne GDO câblée, c'est au logiciel de le faire —
une puce qui dispose de GDO signale la fin de trame par interruption.

Effet mesuré : **longueurs aberrantes passées de 50 à 0**, paquets valides
de 1 à 8 sur 50.

### Accord fin de fréquence

Le balayage RSSI ne résout pas mieux que la largeur du filtre. Pour
l'accord fin, `tools/hw_debug/cc1101_tune.py` mesure ce qui compte
vraiment : **le nombre de paquets qui passent le CRC** à chaque fréquence.
Le maximum de cette courbe est le vrai centre.

Résultat : optimum à **433,82 MHz** côté récepteur du drone, soit 20 kHz
au-dessus de la consigne nominale. Tenter de corriger ce reliquat côté
station sol a **dégradé** le résultat — les biais des deux oscillateurs ne
se composent pas comme un simple décalage unique. L'accord a donc été fait
empiriquement, chaque bout à sa valeur mesurée.

### Troisième cause : la modulation — l'OOK, pas le GFSK

Le montage antérieur de Jules, qui portait la télémétrie à travers son
jardin **avec ces mêmes antennes**, utilisait l'ASK/OOK. Ce détail était
déterminant et je l'ai d'abord écarté.

**L'OOK module l'amplitude, pas la fréquence.** Il tolère donc le décalage
de quartz de 80 kHz, là où le discriminateur GFSK s'effondre dès que
l'écart dépasse la déviation. Pour une paire de modules bon marché dont les
oscillateurs ne s'accordent pas, c'est la modulation à choisir.

Un premier essai en OOK avait échoué, mais il était **invalide** : mon
correctif de la table de puissance n'avait pas été appliqué au firmware
ESP32. Or en OOK, `PATABLE[0]` porte le niveau « 0 » et `PATABLE[1]` le
niveau « 1 », et `FREND0 = 0x11` pointe sur l'index 1. Celui-ci n'étant
jamais écrit, **la station sol émettait à puissance nulle**. Toujours
vérifier qu'un correctif a bien pris avant d'en tirer une conclusion.

Écriture correcte : `PATABLE` en rafale, deux octets `{0x00, 0xC0}`.

### État

Sens **station sol vers drone : opérationnel**. 34 paquets valides sur 50,
**LQI à 0** — la valeur parfaite — RSSI stable à -41,5 dBm, et des numéros
de séquence consécutifs sans aucun trou.

Progression au fil des correctifs : 0, puis 1 (fréquence), puis 8 (lecture
FIFO), puis 12 (accord fin), puis **34 (OOK)**.

Sens **drone vers station sol : non opérationnel**. Le firmware ESP32
rejette les trames sur des longueurs aberrantes qui décroissent
régulièrement (70, 69, 68, 67) — signature d'un décalage dans la lecture de
la FIFO, le premier octet lu n'étant pas l'octet de longueur. Le correctif
d'attente appliqué au récepteur du drone a été transposé au firmware, sans
résoudre ce cas. C'est le point de reprise.

Les 39 CRC invalides restants sont de **vrais paquets avec des bits faux**,
pas des fausses détections — aucune longueur n'est plus aberrante.

Pistes restantes, dans l'ordre de rentabilité :

1. **L'antenne de la station sol, toujours d'origine.** Le contrepoids a
   rapporté 17 dB côté drone ; les pertes des deux antennes s'additionnent.
   C'est le geste le plus rentable qui reste.
2. L'auto-brouillage : la station sol émet et reçoit sur le même module.
3. Le sens drone vers station sol n'a pas encore été accordé finement —
   le même `cc1101_tune.py` doit être rejoué depuis l'ESP32.

### Code produit

- `include/drone/Components/SystemMonitoring/Driver/CC1101_Driver.hpp`
  et `src/SystemMonitoring/CC1101_Driver.cpp` — pilote derrière
  `ITelemetryLink`, donc interchangeable avec `UdpTelemetryDriver`.
  `init()` est hors constructeur, pour la même raison que
  `ComponentWatchdog::start()`, et vérifie l'identité de la puce avant de
  la configurer.
- `src/HwBench/bench_cc1101.cpp` — banc `rx` / `tx` / `pingpong`.
- `GCS/ESP32/cc1101_test/` et `GCS/ESP32/cc1101_link/` — projets PlatformIO
  séparés du firmware du panneau d'alarme, pour qu'un essai radio ne prive
  pas la malette de ses voyants.
- `types.hpp` : trois erreurs SPI ajoutées, dans l'esprit des variantes I2C
  et UART existantes.

**La table de registres doit rester identique entre le pilote C++ et le
firmware ESP32.** Un seul écart et la liaison est muette, sans message.

### Persistance des réglages wfb-ng

Aucun des trois réglages ne survit seul à un redémarrage. Trois fichiers les
rendent permanents, posés sur les deux machines :

| Fichier | Rôle |
|---|---|
| `/etc/modprobe.d/88XXau_wfb.conf` | `rtw_tx_pwr_idx_override=30` |
| `/etc/NetworkManager/conf.d/99-wfb-unmanaged.conf` | interface non gérée |
| `/usr/local/sbin/wfb-monitor.sh` + service systemd | mode monitor et canal |

Le script **vérifie** que le mode et le canal ont été appliqués et échoue
bruyamment sinon : un échec silencieux ici donne exactement le symptôme
qu'on a mis une soirée à diagnostiquer.

Validé par un redémarrage réel de la station sol : service actif, interface
en monitor sur le canal 11, indice de puissance à 30, le tout sans
intervention.

**Ne pas utiliser `shutdown -r +N` comme filet de sécurité** : il crée
`/run/nologin` et interdit toute connexion non-root pendant le décompte —
on se verrouille dehors au lieu de se protéger. `systemd-run
--on-active=N systemctl reboot` ne bloque rien et s'annule proprement.

### Liaison SDL bidirectionnelle — cause racine : horloge SPI de l'ESP32

Les deux sens fonctionnent. La cause de tous les symptômes était unique et
n'avait rien de radio : **le bus SPI de l'ESP32 est non fiable aux horloges
lentes**, et le firmware tournait à 500 kHz.

Mesure isolée, sans radio en jeu (`GCS/ESP32/cc1101_test`, aller-retour sur
PATABLE en rafale, 500 essais par palier) :

| horloge SPI | aller-retours fautifs |
|---|---|
| 100 kHz | 470 / 500 |
| 500 kHz | 150 / 500 |
| 1 MHz | 0 / 500 |
| 2 MHz | 0 / 500 |
| 4 MHz | 0 / 500 |

La liaison est désormais à **2 MHz**. Ne pas redescendre sous 1 MHz. Le Pi
n'était pas concerné : son SPI matériel via `spidev` est propre à 500 kHz,
ce qui explique que seule la station sol était touchée.

Conséquences observées, toutes expliquées par cette seule cause :

- à l'écriture de la FIFO d'émission, des octets étaient perdus : 40 écrits,
  17 réellement émis, avec un CRC pourtant valide puisque la puce calculait
  le CRC sur ce qu'elle avait réellement reçu de l'ESP32 ;
- à la lecture de la FIFO de réception, le pointeur se bloquait : les
  premiers octets étaient justes, puis le dernier se répétait jusqu'au bout
  de la trame.

Résultats après correction, motif incrémental de 40 octets :

- station sol → drone : **50 / 50 trames valides**, 0 CRC invalide, LQI 0,
  −42 dBm (ce sens était à 34/50 avec corruption) ;
- drone → station sol : **21 trames valides sur ~30 émises**, 0 CRC
  invalide, 0 longueur aberrante, −41 dBm. Les trames manquantes
  correspondent aux instants où la station sol émettait elle-même : la
  liaison est semi-duplex.

#### Fréquences : ne pas aligner les deux registres

Le drone émet à `FREQ = 0x10AF76` (433,82 MHz), la station sol écoute à
`0x10B00C` (433,88 MHz). Cet écart de 60 kHz n'est pas une erreur : il
compense la différence de quartz entre les deux modules. Mesuré par
balayage de la fréquence d'émission corrélé à l'horodatage absolu de la
capture. Programmer la même valeur des deux côtés les désaccorderait.

#### Correctifs de la couche FIFO (firmware et pilote C++)

Quatre défauts réels, trouvés avant d'identifier la cause racine, et qui
restent valables :

1. **pas de vidage de FIFO après un CRC invalide** — les octets du paquet
   rejeté restaient et le premier était relu comme octet de longueur au tour
   suivant, d'où des longueurs aberrantes décroissant une par une ;
2. **`RXBYTES` lu une seule fois** — erratum CC1101, le compteur peut être
   lu corrompu pendant que la puce le met à jour ; deux lectures
   concordantes, espacées d'un temps d'octet ;
3. **aucune garde d'état** — la FIFO était lue sans vérifier `MARCSTATE`, et
   dès un seul octet disponible alors qu'un paquet complet en porte au
   minimum trois (longueur + deux octets d'état) ;
4. **lecture avant arrivée complète du paquet** — l'attente est maintenant
   fondée sur une durée calculée depuis la longueur annoncée, pas sur le
   compteur.

#### Méthode : deux erreurs à ne pas refaire

- **Afficher la puissance reçue sur chaque rejet, dès le début.** Deux tours
  complets ont été perdus à poursuivre des longueurs aberrantes de 83 qui
  étaient du bruit ambiant à −90 dBm, alors que les vraies trames du drone
  arrivaient à −31 dBm avec une longueur juste.
- **Faire inscrire par la capture son instant zéro absolu.** Un alignement
  temporel estimé, décalé de deux ou trois paliers, a désigné la mauvaise
  fréquence et envoyé le test de puissance hors bande — où il n'y avait
  évidemment rien à mesurer.
- **Un contrôle émetteur éteint départage toujours.** Alterner PA au maximum
  et PA à zéro dans une même capture a prouvé en une mesure que la source
  était bien le drone (38 événements contre 2).

#### Pièges d'outillage rencontrés

- `pkill -f <motif>` et `pgrep -f <motif>` **se détectent eux-mêmes** dans la
  ligne de commande du shell distant qui les exécute, et tuent la session.
  Utiliser un motif entre crochets : `pkill -f 'sdl_captur[e].py'`.
- `cat /dev/ttyUSB0` sans `stty -F /dev/ttyUSB0 115200 raw` lit à la mauvaise
  vitesse et ne produit que du bruit.
- `commande | tee fichier | head -n` fait remonter un SIGPIPE jusqu'à la
  commande, qui s'arrête bien avant la fin.
- L'accès SPI sur le drone **ne nécessite pas `sudo`** : l'utilisateur est
  dans le groupe `spi`.

## Banc HIL de qualification d'un lien — `bench_link`

Mesure la latence, la perte, le débit et la portée d'un lien radio réel, pour
les deux liens derrière `ITelemetryLink` :

| `--link=` | lien | charge utile max | pendant au sol |
|---|---|---|---|
| `cc1101` | SDL (SPI) | ~50 o utiles | l'écho est fait par le **firmware** ESP32 ; l'outil au sol ne lit que ses compteurs |
| `wfb` | PDL (wfb-ng) | 1024 o | l'outil au sol **est** le répondeur (UDP local) |
| `udp` | UDP loopback nu | 1024 o | le même script — sert à valider le banc **sans radio** |

Le rôle du pendant au sol diffère parce que la radio n'est pas au même
endroit. En SDL elle est sur l'ESP32 : c'est son firmware qui renvoie les
trames, un détour par le port série ajouterait des dizaines de millisecondes
à la mesure de latence. En PDL la radio est un adaptateur wifi piloté par
wfb-ng, qui livre les trames sur un port UDP local : l'outil peut donc
répondre lui-même.

### Usage : deux terminaux, une commande chacun

**Au sol**, lancer d'abord le pendant — il affiche la commande exacte à taper
sur le drone, puis rend compte en direct :

```sh
python3 tools/hw_debug/link_bench_gcs.py cc1101     # lien secondaire SDL
python3 tools/hw_debug/link_bench_gcs.py wfb        # lien principal PDL
python3 tools/hw_debug/link_bench_gcs.py udp        # sans radio, pour essayer
```

Il détecte le port série, signale qui l'occupe, et `--stop-server` arrête le
dashboard s'il le tient. `--duration=N` pour une mesure chronométrée,
`--verbose` pour voir toutes les lignes du firmware.

**Sur le drone**, le lanceur compile si nécessaire et porte les bons défauts :

```sh
./tools/link_bench.sh                     # ping sur SDL
./tools/link_bench.sh ramp                # cherche le plafond de charge utile
./tools/link_bench.sh ping --link=wfb     # lien principal
./tools/link_bench.sh range --csv=/tmp/portee.csv
```

Il recompile dès qu'une source est plus récente que le binaire : mesurer avec
un binaire périmé est la meilleure façon de tirer des conclusions fausses.

Les deux côtés affichent **une ligne par seconde** pendant la mesure, puis un
bilan. Sur le drone la sortie est en mode ligne, sinon rien n'apparaîtrait
avant la fin à travers ssh — stdout y est un tuyau, donc bufferisé par blocs.

Exemple de ce que donne la console du drone :

```
[   7s] emises 35    echos 26    perte  40.0 %  rtt  128.21 ms  ici -44 dBm  face -44 dBm
```

et celle du sol, en SDL :

```
[  26s] echos 87      (+4/s)   saturation 0       rejets 3
```

Les deux bouts se corroborent : sur une mesure de 18 s, l'ESP32 a renvoyé 87
échos et le drone en a reçu 78, les 9 manquants étant perdus au retour — ce
que le drone compte comme trames malformées.

### Pourquoi la latence est mesurée en aller-retour

Le drone émet une trame horodatée, le répondeur la renvoie telle quelle, le
drone calcule le temps écoulé. **Une seule horloge est en jeu** : aucune
synchronisation entre les deux machines, aucune dérive à corriger. Mesurer
la latence par sens exigerait NTP ou PTP entre le drone et le sol, et un
biais de synchronisation inconnu entraînerait une erreur du même ordre que
la grandeur mesurée.

Le répondeur n'interprète jamais l'horodatage : il inscrit seulement, dans
deux champs réservés, le RSSI et le LQI qu'il a mesurés. Le drone obtient
ainsi **la qualité des deux sens en une seule mesure**.

### Format de trame

20 octets d'en-tête, petit-boutien, offsets fixes avec `memcpy` — même
convention que les paquets générés de `TELEM`, et non un attribut `packed` :
c'est ce qui garantit que le C++ du drone, le Python du sol et le C++ de
l'ESP32 voient exactement les mêmes octets.

| offset | type | champ |
|---|---|---|
| 0 | u16 | `magic` = `0x4B4C` |
| 2 | u8 | `version` = 1 |
| 3 | u8 | `mode` (1 ping, 2 flood, 3 range, 4 ramp) |
| 4 | u32 | `seq` |
| 8 | u64 | `txNs` — horloge monotone du drone, réémise telle quelle |
| 16 | i8 | `farRssi` — rempli par le répondeur |
| 17 | u8 | `farLqi` — rempli par le répondeur |
| 18 | u8 | `farFlags`, bit 0 : les deux champs ci-dessus sont renseignés |
| 19 | u8 | réservé |

Le reste de la trame est un bourrage incrémental, pas des zéros : une
troncature devient ainsi visible à l'œil.

Le `magic` et la `version` ne sont pas décoratifs. Le même lien porte la
télémétrie réelle, et un paquet `TELEM` ne doit jamais être compté comme un
écho de banc. Toute trame qui ne les porte pas est comptée « hors banc ».

### Les quatre modes

- **`ping`** — une trame toutes les `--interval` ms. Donne RTT min/moyen/p95/max,
  gigue (écart-type), perte, trames désordonnées, RSSI et LQI des deux côtés.
  C'est le mode de référence.
- **`flood`** — émission saturée pendant `--duration` s. **Le répondeur
  n'écho pas dans ce mode** : sur un lien semi-duplex l'écho collisionnerait
  avec l'émission et le débit mesuré ne voudrait rien dire. Le taux de
  réception se lit donc sur l'outil au sol, pas sur le drone.
- **`range`** — émission à 1 Hz avec affichage du RSSI/LQI en direct, pour
  marcher avec le drone et relever la portée réelle. Utiliser `--csv` pour
  tracer ensuite.
- **`ramp`** — fait varier la taille des trames sur 8 paliers, de 20 octets à
  la charge utile maximale du lien, et donne le taux d'erreur par palier.
  Sert à dimensionner les paquets de télémétrie. Le nombre de paliers est
  fixe et non le pas : un pas constant donnerait 126 paliers de 4 trames sur
  le PDL, échantillon trop maigre pour conclure.

### Lire les résultats sans se tromper

- **« envois refusés » n'est pas de la perte radio.** C'est la charge offerte
  que le lien n'a pas pu absorber — file du socket pleine, ou FIFO du CC1101
  occupée. C'est son plafond. Les deux sont comptés séparément exprès.
- Le numéro de séquence n'avance **qu'après un envoi réussi**. Sans cela, le
  plus grand numéro vu au sol mesurerait la somme « perte + refus » et la
  perte annoncée serait fausse (mesuré : 80 % annoncés contre 1,9 % réels).
- **Sur SDL, le RTT inclut le coût de l'interrogation logicielle**, pas
  seulement le temps de vol. Faute de ligne GDO câblée, le récepteur attend
  la durée du paquet avant de lire la FIFO : comptez quelques dizaines de
  millisecondes de plancher. C'est la latence réelle du lien tel qu'il est
  câblé, mais ce n'est pas une limite physique de la radio.
- Le mode `flood` sur SDL **viole franchement le rapport cyclique admis en
  433 MHz**. Le banc le signale au démarrage. À réserver à des mesures
  courtes.

### Le port série du GCS est tenu par `server.js`

Le serveur Node du GCS garde `/dev/ttyUSB0` ouvert. Linux autorise plusieurs
ouvertures d'un tty, donc une lecture concurrente « marche » tout en volant
des octets au serveur — c'est ce qui rendait les premières captures série
erratiques. `esptool`, lui, demande un verrou exclusif et échoue franchement.

Avant de téléverser le firmware ou de lancer le banc SDL, arrêter le
serveur, ou vérifier qui tient le port :

```sh
for p in /proc/[0-9]*; do ls -l $p/fd 2>/dev/null | grep -q ttyUSB0 \
  && echo "$(basename $p) : $(tr '\0' ' ' < $p/cmdline)"; done
```

### Validation du banc lui-même

Le lien `udp` existe pour qualifier le banc sans matériel. Mesuré en
loopback sur le PC de dev :

- `ping` : 119 émises, 119 échos, 0 % de perte, RTT moyen 1,8 ms, RSSI
  distant correctement propagé ;
- `flood` : 426 342 émises et **426 342 attendues au sol** — les deux bouts
  s'accordent exactement — 1,9 % de perte, 43 Mbit/s ;
- `ramp` : 8 paliers de 55 trames, 0 % de perte à toutes les tailles.

### Mesures HIL du lien SDL — premières valeurs de référence

Relevées avec `./bench_link` sur le drone et le firmware ESP32 en mode écho,
les deux modules sur le même plan de travail (RSSI ~−43 dBm).

**`ping`, trames de 32 octets** — 125 émises, 113 échos :

| mesure | valeur |
|---|---|
| aller-retour | min 126,0 / moy 127,2 / p95 130,3 / max 130,5 ms |
| gigue (écart-type) | 1,25 ms |
| perte | 9,6 % |
| qualité ici / en face | −44 dBm, LQI 8 / −43 dBm, LQI 8 |

La latence est remarquablement stable. Elle se décompose en ~29 ms de temps
de vol par sens à 9,6 kbit/s, plus l'attente d'environ 31 ms que le
récepteur s'impose avant de lire la FIFO faute de ligne GDO câblée. **Câbler
GDO0 ferait tomber ce plancher d'environ 60 ms**, c'est la seule
amélioration de latence vraiment disponible sur ce lien.

**`ramp`** — la charge utile exploitable s'arrête bien avant le maximum
théorique :

| taille | perte |
|---|---|
| 20 o | 11 % |
| 30 o | 0 % |
| 40 o | 22 % |
| 50 o | 11 % |
| 55 o | 89 % |
| 60 et 61 o | 100 % |

À 61 octets utiles la FIFO contient 1 + 61 + 2 = **exactement 64 octets, sa
capacité totale**, sans aucune marge de synchronisation. Le `kMaxPayload =
61` du pilote est juste en théorie et inatteignable en pratique sans GDO.
**Retenir ~50 octets comme plafond utile.** Le plus gros paquet Links=BOTH
(`CMD_WAYPOINT`) en fait 21 : la marge reste confortable.

Le RTT croît de ~2,5 ms par octet, cohérent avec deux fois le temps de vol.

### Bug ouvert : `CC1101_Driver::send()` tronque les trames en vol

Révélé par le mode `flood` : le drone annonce 6231 trames et 265 kbit/s sur
un lien à 9,6 kbit/s — impossible — et le firmware n'en compte **qu'une
seule** réellement reçue.

Cause : la garde de `send()` refuse l'émission seulement si `MARCSTATE` vaut
`TX` (0x13) ou `TX_END` (0x14). Après un strobe `STX`, la puce traverse
d'abord CALIBRATE, SETTLING puis FSTXON. Un `send()` appelé pendant ces
états transitoires passe la garde, et le `SIDLE` + `SFTX` qui suit **tronque
la trame en cours d'émission**.

C'est une liste noire là où il faut une liste blanche. Correctif proposé,
à valider : n'émettre que depuis un état franchement calme — `MARCSTATE`
égal à IDLE (0x01) ou RX (0x0D) — et vérifier en plus que `TXBYTES` est nul,
sinon refuser avec `Timeout`.

Conséquence pour le logiciel de vol : dès que deux émissions se succèdent de
près, la première est perdue **sans erreur remontée**. Les modes `ping`,
`range` et `ramp` ne sont pas affectés, leur cadence étant assez lente ; le
mode `flood` sur SDL n'a aucune valeur de mesure tant que ce bug est là.

### Bug corrigé : `init()` ne laissait pas la puce se stabiliser

`init()` lisait `PARTNUM`/`VERSION` immédiatement après le strobe de reset et
échouait en `InvalidData` sur une puce parfaitement saine. Tous les scripts
Python de `tools/hw_debug/` attendaient 100 ms à cet endroit, ce qui masquait
le défaut. `init()` réessaie maintenant pendant 50 ms, et échoue toujours
franchement si la puce est réellement absente.

### Audit du banc — six défauts trouvés en le relisant

Corrigés après coup, parce qu'un outil de mesure qui ment est pire que pas
d'outil :

1. **Ports incohérents en `udp`.** Le banc prenait 5611/5612 (les ports de
   TelSec) et l'outil au sol 15612/15611 : la commande courte échouait à
   100 % de perte sans rien expliquer. Les deux prennent désormais
   15611/15612, choisis pour ne pas tomber sur 5602/5612 qu'occupe le
   dashboard.
2. **`link_bench.sh` affirmait une fausseté** — « sans l'outil au sol le banc
   ne mesure rien ». Vrai en PDL, **faux en SDL** où l'écho est fait par le
   firmware : le banc y mesure tout seul.
3. **Instruction impossible à suivre en `udp`.** L'outil écrivait « à lancer
   sur le drone » alors que `UdpTelemetryDriver` se lie à `INADDR_LOOPBACK` :
   les deux bouts sont forcément sur la même machine.
4. **Ports wfb inventés.** Ils étaient devinés alors que wfb-ng ne tourne pas
   et qu'aucun fichier de configuration n'existe aux emplacements standard.
   L'outil les déduit maintenant des process `wfb_rx`, et **refuse** avec un
   message explicite s'il ne les trouve pas. Les deviner donnerait 100 % de
   perte sans que rien n'indique que le problème vient des ports.
5. **Les dernières trames étaient comptées perdues.** À 130 ms d'aller-retour
   et une fin de mesure brutale, une à deux trames encore en vol passaient au
   passif. Le banc écoute maintenant 500 ms de plus après avoir cessé
   d'émettre, et les débits restent calculés sur la seule fenêtre d'émission.
6. **Un compteur qui mélangeait deux causes.** `malformees` additionnait les
   trames rejetées par le pilote (CRC faux : un défaut de réception) et les
   trames valides étrangères au banc (télémétrie réelle : pas un défaut).
   Séparés en `rejetees` et `hors banc`.

### Ce que le comptage des deux côtés révèle : la perte est asymétrique

Mesure de 16 s, trames de 32 octets :

| étape | trames |
|---|---|
| émises par le drone | 80 |
| reçues et renvoyées par l'ESP32 | **76** → 5 % de perte à l'aller |
| échos reçus par le drone | **56**, 20 rejetés → **26 % de perte au retour** |

**C'est le récepteur du drone qui est le point faible**, pas la radio ni le
trajet aller. Cohérent avec l'état du pilote : le correctif d'échéance
d'arrivée a réduit le problème sans l'éliminer, là où le firmware de l'ESP32,
qui attend la durée complète du paquet, ne perd presque rien.

Le banc signale maintenant ce cas de lui-même quand les rejets expliquent au
moins la moitié des échos manquants. C'est une information qu'aucun des deux
côtés ne donne seul.

### Correctif `send()` appliqué : le débit annoncé redevient physique

La garde liste maintenant les états **permis** (`IDLE` 0x01 et `RX` 0x0D) au
lieu des états interdits, plus un rattrapage sur `TXFIFO_UNDERFLOW` (0x16),
où la puce reste coincée jusqu'à ce qu'on vide la FIFO — sans quoi toutes les
émissions suivantes échoueraient définitivement.

Effet sur le mode saturation, trames de 32 octets, 6 s :

| | avant | après |
|---|---|---|
| trames émises | 5999 | **131** |
| débit annoncé | 256 kbit/s | **5,59 kbit/s** |
| charge refusée | 2,6 % | **99,5 %** |

5,59 kbit/s de charge utile sur 9,6 kbit/s brut, soit 58 % d'efficacité : le
reste est préambule (24 octets), synchro, octet de longueur et CRC. L'ordre
de grandeur est enfin cohérent avec la physique du lien.

Aucune régression sur `ping` : aller-retour inchangé à 129 ms, et la perte
mesurée est plutôt descendue (13,9 % contre ~30 % avant).

### Le plafond réel du lien n'est pas le débit, c'est la cadence

Mesure de la cadence soutenable, trames de 32 octets, 12 s par point :

| intervalle | offert | **utile** | perte |
|---|---|---|---|
| 300 ms | 3,3 tr/s | 2,8 tr/s | 12,8 % |
| 200 ms | 5,0 tr/s | **4,9 tr/s** | 0 % |
| 150 ms | 6,6 tr/s | **5,4 tr/s** | 17,7 % |
| 100 ms | 10,0 tr/s | **0** | **100 %** |
| 70 ms | 14,2 tr/s | 0 | 100 % |
| 50 ms | 20,0 tr/s | 0 | 100 % |

La coupure est franche, pas progressive, et le mécanisme est simple :
**l'aller-retour vaut 129 ms**. Dès que l'intervalle descend sous cette
valeur, l'écho revient pendant que le drone émet — et un CC1101 ne peut pas
recevoir en émettant. Chaque écho est donc perdu, systématiquement.

Conséquence de dimensionnement : **viser 200 ms d'intervalle, soit 5 trames
par seconde**, pour un échange aller-retour sur SDL tel qu'il est câblé. Le
banc avertit maintenant de lui-même quand l'intervalle demandé n'excède pas
l'aller-retour mesuré, pour qu'on ne conclue pas à un défaut radio là où il
n'y a qu'un rythme intenable.

En émission pure (sans attendre d'écho) le plafond serait plus haut, mais le
récepteur d'en face décroche aussi : à 21,75 trames/s offertes, l'ESP32 n'en
a reçu que 6 sur 174. Son firmware se bloque ~33 ms après chaque détection
pour attendre l'arrivée complète du paquet, faute de ligne GDO — il ne peut
pas suivre.

**C'est le troisième résultat qui converge vers le même remède : câbler
GDO0.** Il supprimerait cette attente, ferait tomber les 129 ms d'environ
60 ms, et relèverait du même coup la cadence soutenable et le plafond de
charge utile.

### Mise au point : d'où venaient les pertes

Le récepteur du pilote, **en réception seule, ne perd rien** : 50 trames
reçues, 0 erreur, 0 débordement, numéros consécutifs, LQI 0. Mesuré avec
`./bench_cc1101 rx` pendant que la station sol émettait son compteur.

Les pertes de 7 à 30 % rapportées plus haut viennent donc du **mode
aller-retour du banc**, pas du lien : il force le drone à émettre et à
recevoir sur une seule radio semi-duplex, et chaque émission le rend sourd
pendant toute sa durée (52 ms pour une trame de 32 octets à 9,6 kbit/s).

| configuration | temps passé à émettre | perte |
|---|---|---|
| 20 o / 400 ms | ~10 % | 8,1 % |
| 20 o / 300 ms | ~14 % | 8,2 % |
| 32 o / 200 ms | ~26 % | 6,8 % |
| 50 o / 200 ms | ~33 % | **29,7 %** |

La relation n'est pas propre — un plancher d'environ 8 % subsiste à faible
charge, et les échantillons sont petits (37 à 74 trames) — mais la rupture à
50 octets est nette, et surtout le cas « réception seule » est à zéro.

**Conséquence pour l'interprétation : le chiffre de perte du mode `ping` ne
mesure pas la fiabilité du lien.** Il mesure le coût de l'aller-retour sur un
lien semi-duplex. La télémétrie réelle est majoritairement descendante et à
sens unique, cas dans lequel le lien est propre.

Ce qui reste à mesurer pour qualifier l'usage réel : le sens descendant seul,
à la cadence visée de 8 Hz. Le banc n'a pas encore de mode pour ça — `flood`
émet sans limite de cadence, `ping` impose un aller-retour.

#### Correctifs du pilote appliqués dans cette passe

1. `send()` : liste blanche `IDLE`/`RX` au lieu d'une liste noire, plus
   rattrapage de `TXFIFO_UNDERFLOW` (cf. section précédente).
2. Échéance d'arrivée portée à 900 µs/octet **plus 2 ms de marge**, valeurs
   du firmware ESP32 qui ne perd rien. Sans marge, l'échéance partait déjà en
   retard puisque l'octet de longueur est lu après son arrivée.
3. **Aucun accès SPI pendant l'arrivée d'un paquet.** Le test d'échéance est
   remonté tout en tête de `poll()`, avant la moindre lecture de registre :
   appelé en boucle serrée, le pilote relisait `MARCSTATE` et `RXBYTES`
   toutes les 200 µs pendant 30 ms, soit des centaines d'accès concurrents
   aux écritures de la radio — ce que l'erratum CC1101 désigne comme la cause
   du dérèglement du pointeur de FIFO.

#### Hypothèse éliminée : le SPI du Pi

Même test d'intégrité que celui qui avait confondu l'ESP32 (aller-retour sur
PATABLE, 500 essais par vitesse) : **0 faute sur 500 à 100 kHz, 500 kHz,
1, 2 et 4 MHz**. Le bus SPI du Pi est sain à toutes les vitesses, contrairement
à celui de l'ESP32. Le pilote peut rester à 500 kHz.

### CCA activé et rejets comptés

`MCSM1` passe de `0x0F` à `0x3F` — `CCA_MODE = 11`, le plus prudent des
quatre : n'émettre que si le canal est libre, et jamais pendant une réception
en cours. **Des deux côtés**, sinon un seul s'efface et l'autre continue de
couper la parole.

#### Le piège qu'il fallait traiter dans le même geste

Avec le CCA actif, la puce **ignore `STX` en silence** quand le canal est
occupé et reste en réception. Or `send()` finissait par `return strobe(kSTX)`
et rendait succès dès le strobe écrit. Activer le CCA seul aurait donc rendu
les pertes *invisibles* au lieu de les réduire : le pilote aurait compté des
trames jamais émises.

`send()` vérifie maintenant que la machine d'état a quitté RX après le strobe
(huit lectures de `MARCSTATE`, borné ; après `STX` la puce calibre ~720 µs et
tout état hors RX prouve le départ). Si elle est toujours en réception, la
FIFO d'émission est vidée — pour que la trame ne s'émette pas plus tard à
contretemps — et `send()` rend `Timeout` pour que l'appelant puisse réessayer.

#### Compteurs ajoutés

Sur le pilote : `crcErrorCount()`, `lengthErrorCount()`, `ccaRefusalCount()`,
en plus de `overflowCount()`. Sans eux, une liaison qui se dégrade est
indiscernable d'une liaison silencieuse — on ne peut pas distinguer
« personne ne parle » de « tout arrive corrompu ». Le banc les affiche au
bilan, et le firmware ESP32 publie les siens sur sa ligne `BANC`.

#### Mesure : le CCA n'a encore jamais déclenché

| essai | perte | CRC faux | refus CCA |
|---|---|---|---|
| 15 s à 200 ms | 17,6 % | 12 | **0** |
| 15 s à 200 ms | 8,1 % | 6 | **0** |
| 15 s à 200 ms | 2,7 % | 2 | **0** |
| 13 s à 60 ms | 100 % | 0 | **0** |

**Zéro refus dans tous les cas, des deux côtés.** Ce n'est pas une panne du
CCA, c'est que je lui demandais de régler un problème qui n'est pas le sien :
en aller-retour, la perte vient de ce que l'émetteur est **sourd pendant sa
propre émission**, pas d'une collision mutuelle. Le CCA empêche d'émettre
par-dessus quelqu'un qui parle ; il ne rend pas l'oreille pendant qu'on parle.

Je n'ai pas réussi à construire un cas où il déclenche : à cadence élevée,
celui qui reçoit est affamé bien avant que celui qui émet soit bloqué — le
récepteur se bloquant ~33 ms par trame faute de GDO, il ne reçoit que 3 trames
sur 215 à 60 ms d'intervalle. **Le bénéfice du CCA reste donc non démontré par
la mesure.** Il déclenchera en exploitation réelle, au moment où le sol envoie
une commande pendant que le drone descend sa télémétrie — rare, mais c'est
précisément l'instant critique.

#### Risque à garder en tête pour un lien de commande

Le CCA peut **empêcher** d'émettre. `AGCCTRL1 = 0x00` place le seuil de
détection de porteuse au niveau cible de l'AGC ; dans un environnement 433 MHz
chargé, le drone pourrait refuser d'émettre alors qu'il doit transmettre une
commande. Les mesures ici (signal à −43 dBm, ambiant à −90 dBm) ne montrent
aucun refus abusif, mais ce n'est pas une preuve en milieu bruyant.

Conséquence de conception à trancher : pour `CMD_EMERGENCY`, vaut-il mieux
rester muet parce que le canal paraît occupé, ou passer outre le CCA après N
tentatives ? Rester muet sur une commande d'urgence est probablement pire que
risquer une collision.

## Acquittement des commandes critiques — couche format et contournement CCA

### La colonne `Ack` de `TELEMETRY.csv`

| valeur | sens |
|---|---|
| `NONE` | aucun acquittement, perte silencieuse assumée |
| `ACK` | acquitté, retransmis sur timeout, CCA respecté |
| `ACK_FORCE` | idem, et **passe outre le CCA** |

Éditable depuis `tools/codes_editor.html` (colonne « Acq. » et sélecteur),
plumbée par `telemetry_common.py` → `gen_telemetry.py` → `AckMode` dans
`Meta`. Aller-retour lecture/écriture du CSV vérifié identique à l'octet,
ordre des `PacketID` préservé.

`ACK_FORCE` regroupe volontairement deux propriétés — acquittement et
contournement du CCA — parce que le seul cas qui veut l'un veut l'autre. À
séparer en deux colonnes si un paquet venait à vouloir le contournement seul.

Les onze commandes critiques montantes sont en `ACK`, sauf `CMD_EMERGENCY` en
`ACK_FORCE`. Deux tests gardent ce contrat : tout paquet acquitté doit pouvoir
porter son `seq`, et `CMD_EMERGENCY` doit être le seul à contourner le CCA.

### Validation : pas d'acquittement sans `seq:u32`

Le générateur refuse `ACK`/`ACK_FORCE` sur un paquet sans champ `seq:u32`. Un
acquittement sans numéro de séquence est inappariable — l'émetteur ne saurait
pas quelle trame est confirmée — et surtout **un doublon de retransmission
serait indétectable, donc réexécuté**. Armer deux fois n'est pas armer une
fois.

Le `seq` est placé **en tête** des champs de chaque commande : il se lit donc
uniformément dans les quatre premiers octets de la charge utile, sans décoder
le reste ni connaître le type.

### `sendForced()` : méthode séparée, pas un paramètre

Ajouté à `ITelemetryLink` avec une **implémentation par défaut qui appelle
`send()`**. Sur un transport sans contrainte d'accès au média — UDP, wfb-ng —
« forcer » n'a aucun sens différent d'« émettre » : il n'y a rien à
contourner. Seul `CC1101_Driver` redéfinit la méthode, parce qu'il est le seul
des trois à écouter avant d'émettre.

L'implémentation éteint `CCA_MODE` dans `MCSM1`, appelle `send()` — donc toute
la garde d'état, la vérification de départ et les compteurs s'appliquent à
l'identique — puis rétablit le CCA **quel que soit le sort de l'émission**.
Laisser le CCA éteint après un échec transformerait une exception ponctuelle
en comportement permanent, sans que rien ne le signale. Un échec du
rétablissement est remonté en priorité sur un échec d'émission.

Vérifié sur le matériel (`./bench_link ping --forced`) : 60 émissions forcées
abouties, puis `MCSM1` relu à `0x3F` depuis un autre processus — `CCA_MODE = 3`,
le CCA est bien rétabli.

Compteur `forcedSendCount()` exposé : il doit rester bas. S'il monte, on passe
en force régulièrement, donc on collisionne délibérément.

### Ce qui n'est pas encore fait

- Dédoublonnage côté drone et émission de `CMD_ACK` par `SysMonitoring`.
- Assemblage de `LINK_QUALITY`.
- Retransmission côté sol dans `server.js`.
- **Le chemin d'urgence de bout en bout n'existe pas** : `CMD_EMERGENCY` est
  montant, donc émis par le sol. Le contournement doit donc agir dans le
  firmware de l'ESP32, qui n'est aujourd'hui qu'un répondeur de banc et ne
  décode aucun paquet de télémétrie. `sendForced()` côté drone est
  l'infrastructure symétrique, mais aucun paquet descendant n'est en
  `ACK_FORCE` aujourd'hui — il n'est donc pas encore emprunté en exploitation.

### Acquittement de bout en bout — implémentation

#### Côté drone : dédoublonnage et émission de `CMD_ACK`

`GCSRxTask` lit le `seq` dans les quatre premiers octets de la charge utile —
le générateur garantit sa présence et il est placé en tête, donc il se lit
sans connaître le type de la commande. Puis :

- **trame trop courte pour porter son `seq`** : la commande n'est pas
  appliquée. On ne saurait pas l'acquitter, et appliquer une commande
  critique sur une trame douteuse est pire que de la laisser retransmettre ;
- **doublon** (`seq` déjà vu pour ce type) : **réacquitté sans être rejoué**.
  Sinon la retransmission, qui existe pour la fiabilité, deviendrait un
  danger — armer deux fois n'est pas armer une fois ;
- **nouvelle commande** : appliquée, `seq` mémorisé, acquittement mis en file.

Le dédoublonnage est **global, pas par lien** : PDL et SDL étant indépendants
et les commandes en `Links=BOTH`, la même commande peut arriver par les deux.
Une seule table `UplinkDedup`, portée par la tâche de réception qui lit les
deux liens, est ce qui empêche la double exécution.

L'acquittement repart sur **le lien d'arrivée**, chaque lien ayant sa propre
boucle de retransmission au sol.

#### La contrainte de concurrence, et comment elle est respectée

Le composant n'a **aucun mutex**, et son invariant est que chaque
`PacketScheduler` appartient à une seule tâche. La tâche de réception ne
touchait aucun scheduler ; il ne fallait pas commencer.

La file `AckQueue` est donc un anneau à deux indices atomiques, un seul
producteur (réception) et un seul consommateur (la tâche du lien) : pas de
verrou. Les tâches d'émission sont des threads SCHED_FIFO, et un mutex pris
par la tâche de réception les bloquerait le temps d'une transaction SPI, soit
plusieurs millisecondes.

C'est la **tâche d'émission** qui constate que sa file n'est pas vide et se
marque `dirty`. Le scheduler reste ainsi la propriété exclusive d'un thread.

File pleine : `push()` refuse plutôt que d'écraser. On préfère perdre un
acquittement — le sol retransmettra — qu'un acquittement plus ancien, donc
plus urgent.

#### `LINK_QUALITY` sans connaître le type du lien

`ITelemetryLink::quality()` renvoie une `LinkQuality` dont les champs sont
optionnels, avec un défaut vide. Un `optional` vide signifie « non mesurable
ici », ce qui n'est pas zéro : un socket UDP n'a pas de RSSI et wfb-ng ne
compte pas de CRC faux. `CC1101_Driver` et `WFB_NG_Driver` redéfinissent la
méthode ; `UdpTelemetryDriver` garde le défaut. Même raison que
`sendForced()` : `SysMonitoring` assemble le paquet sans connaître les types
concrets.

#### Côté sol : retransmission dans `server.js`

| | délai | tentatives | fenêtre |
|---|---|---|---|
| `ACK` | 300 ms | 4 | ~1,2 s |
| `ACK_FORCE` | 200 ms | 8 | ~1,6 s |

Le délai de 300 ms est calé sur la mesure : ~101 ms d'aller-retour pour un
paquet de 21 octets, et ~5 trames/s de cadence soutenable. Plus court, on
réémettrait avant que l'acquittement ait eu le temps d'arriver, et on
saturerait le lien.

Trois comportements qui méritent d'être explicités :

- **l'échec est annoncé**, pas silencieux. Abandonner sans rien dire serait
  le pire : l'opérateur croirait sa commande passée. Un message
  `{type:"ack", state:"failed"}` part vers l'interface et le serveur le
  journalise ;
- **un nouvel ordre du même type annule celui en vol** : l'opérateur vient de
  décider autre chose, réémettre l'ancien n'aurait aucun sens ;
- **un acquittement en retard est ignoré** s'il ne correspond pas au `seq`
  courant, pour ne pas annuler l'ordre qui l'a remplacé.

Les ports UDP et HTTP de `server.js` sont devenus surchargeables par
l'environnement (`GCS_TELMAIN_RX`, etc.), ce qui permet de faire tourner une
instance de test à côté du dashboard.

#### Tests

Côté C++, six tests unitaires sur la logique pure (`AckQueue` : ordre, refus
quand plein ; `UplinkDedup` : première arrivée, nouveau `seq`, indépendance
des commandes, caractère global).

Côté sol, `GCS/test/test_ack.js` simule le drone en UDP et vérifie quatre
scénarios, tous au vert :

| scénario | émissions | états annoncés |
|---|---|---|
| acquitté du premier coup | 1 | pending → confirmed |
| deux premières ignorées | 3 | pending ×3 → confirmed |
| jamais acquitté | 4 | pending ×4 → **failed** |
| urgence jamais acquittée | 8 | pending ×8 → **failed** |

#### Ce qui n'est toujours pas en place

Le contournement CCA de `CMD_EMERGENCY` **ne traverse pas encore la chaîne**.
La commande est montante, donc émise par le sol ; sur SDL elle doit passer par
l'ESP32, qui n'est aujourd'hui qu'un répondeur de banc et ne décode aucun
paquet. `sendForced()` existe côté drone et est vérifié sur le matériel, mais
rien ne l'emprunte en exploitation. Au sol, `ACK_FORCE` ne se traduit pour
l'instant que par une retransmission plus insistante.

Faire de l'ESP32 un vrai pont SDL est le prochain gros morceau, et c'est lui
qui débloque l'arrêt d'urgence de bout en bout.

## Le pont SDL : la station sol parle enfin par radio

### Une seule carte, deux rôles

L'ESP32 porte le panneau d'alarme **et** la radio CC1101. Le firmware de
production `GCS/ESP32/firmware/` fait donc les deux : voyants, boutons, et
pont radio. Il ne décode aucun paquet de télémétrie — il transporte des
octets, toute la logique reste côté Node.js.

### Le pilote radio est partagé, et c'est voulu

`GCS/ESP32/shared/cc1101_radio.hpp` est la source de vérité unique, incluse
par les projets ESP32 via `build_flags = -I ../shared`.

La raison n'est pas l'élégance : la table de registres a déjà existé en deux
exemplaires — le banc et le pilote du drone — et elle a **divergé** sur la
fréquence et la longueur de préambule. Les deux bouts ne se parlaient plus,
sans aucun message d'erreur, et le diagnostic a coûté des heures.

`GCS/ESP32/cc1101_link/` garde pour l'instant sa propre copie, signalée comme
dette dans son en-tête. À migrer.

### Protocole série étendu

| sens | ligne | rôle |
|---|---|---|
| PC → ESP32 | `TX:<hex>` | émettre en respectant le CCA |
| PC → ESP32 | `TXF:<hex>` | émettre en **forçant**, CCA contourné |
| ESP32 → PC | `RX:<hex>:<rssi>:<lqi>` | trame reçue et sa qualité |
| ESP32 → PC | `LQ:<rssi>:<lqi>:<crc>:<len>:<ovf>:<cca>:<forced>` | santé, 1 Hz |
| ESP32 → PC | `TXERR:<raison>` | émission refusée — jamais silencieuse |

L'hexadécimal double le volume, mais le SDL ne porte que ~5 trames de 50
octets par seconde : à 115200 bauds c'est négligeable, et une trace au
moniteur série reste lisible à l'œil. Un cadrage binaire aurait gagné du
débit dont on n'a aucun besoin, au prix de la lisibilité.

### Présence de la radio : prouvée, pas annoncée

Premier essai de bout en bout : les commandes partaient en UDP dans le vide.
`radioReady` ne s'armait que sur `RADIO:ok`, émis **au seul démarrage** du
firmware — or le serveur se connecte presque toujours après le boot de
l'ESP32, manquait l'annonce, et croyait la radio absente.

Corrigé : recevoir une ligne `LQ` **est** la preuve que la radio répond,
puisque le firmware ne l'émet que dans ce cas. Et trois secondes de silence
la font repasser à absente. Une présence se prouve par une évidence
continue, pas par une annonce qu'on peut rater.

### Mesures de bout en bout

Émission par le pont, drone en réception (`./bench_cc1101 rx`) :

- `TX:` → trame reçue intacte côté drone (`PONT-SDL-1`, −50 dBm) ;
- `TXF:` → compteur `forcedSends` du firmware incrémenté, contournement du
  CCA effectivement emprunté ;
- sens drone → sol : **16 trames relayées** en `RX:` avec RSSI et LQI
  (`44524F4E452D30` = « DRONE-0 »).

Chaîne complète, tableau de bord → `server.js` → série → ESP32 → radio →
drone :

| observation | résultat |
|---|---|
| `CMD_ARM` retransmis | 4 essais puis **failed** annoncé |
| `CMD_EMERGENCY` retransmis | **8 essais** (politique `ACK_FORCE`) puis failed |
| `forcedSends` du firmware | 1 → 6, le forçage est bien emprunté |
| trames reçues par le drone | 9 trames de 5 octets = `[PacketID][seq:u32]` |
| qualité SDL au tableau de bord | `rssi=-44 lqi=42`, publiée chaque seconde |

Les antennes du schéma du drone ont donc enfin une source de données.

L'acquittement « échoue » parce que `bench_cc1101` n'implémente pas `CMD_ACK` :
c'est un banc, pas le logiciel de vol. **Boucler l'acquittement demande de
lancer `build/drone`**, ce qui relève de la règle dure — à demander.

### Dérive de déploiement — à régler

Le dashboard de la station sol tourne depuis **`/home/jules/gcs/GCS`**, pas
depuis le dépôt `~/droneMk2`. Deux copies coexistent, et la copie déployée est
un **instantané plus ancien**, avec une adaptation locale des chemins (CSV à
plat dans `~/gcs/` au lieu de `~/gcs/data/`).

Conséquence : pousser un fichier dans le dépôt de la station sol ne déploie
rien. Et il n'existe aucun script de déploiement — c'est exactement ainsi que
deux copies divergent sans qu'on s'en aperçoive.

Les tests ci-dessus ont donc été menés depuis le dépôt, avec un lien
symbolique vers le `node_modules` du déploiement, **sans toucher à la copie
déployée**. Un script de déploiement reste à écrire.

## Banc multi-liens : mesurer les deux en même temps

`--link` accepte une liste : `--link=cc1101,wfb` mesure les deux liens
simultanément, chacun avec sa propre numérotation, ses propres statistiques
et son propre rythme. L'outil au sol accepte la même syntaxe et répond sur
tous les transports demandés dans une **boucle `select` unique** — pas de
threads, donc aucune synchronisation à soigner dans un outil de mesure.

Mesurer l'un puis l'autre ne dit rien de ce qui se passe quand ils
fonctionnent ensemble : les deux radios sont à quelques centimètres l'une de
l'autre sur le drone, et c'est en vol qu'elles cohabitent. Une mesure côte à
côte est aussi la seule façon de comparer honnêtement un lien de secours à un
lien principal.

Le bilan se termine par un tableau comparatif, et le CSV gagne une colonne
`lien` — sans elle un enregistrement simultané serait inexploitable.

Validé à deux voies (SDL réel + bouclage UDP local, un répondeur de chaque
côté) : statistiques bien séparées, 72/72 sur l'UDP, tableau comparatif
correct.

### L'écho SDL se fait maintenant au sol, pas dans le firmware

Le firmware de production ne fait que relayer : il n'écho plus les trames de
banc. C'est l'outil au sol qui renvoie, via `RX:` → `TX:`.

**L'aller-retour mesuré inclut donc le détour par le port série**, quelques
millisecondes de plus. En échange, c'est le chemin de PRODUCTION qui est
mesuré et non un montage de test — ce qui vaut mieux.

### Deux régressions introduites en extrayant le code partagé

L'extraction du pilote radio dans `cc1101_radio.hpp` a cassé deux choses que
seul un essai sur matériel a révélées :

1. **La boucle de réessai supprimée.** L'attente calculée est une estimation ;
   le débit réel dérive de la tolérance du quartz et le compte peut être
   transitoirement court. J'avais remplacé la boucle bornée du firmware de
   banc par une seule vérification, qui jetait alors la trame. Restaurée.
2. **La lecture de FIFO repassée en rafale.** Le firmware de banc lisait en
   accès simple, octet par octet — c'est ce changement qui avait donné 90
   trames sur 90. Remis en accès simple.

Leçon : extraire du code éprouvé n'est pas une opération neutre. Il faut
remesurer, pas relire.

### Les deux hôtes n'ont pas le même optimum SPI

| | lecture FIFO | écriture FIFO |
|---|---|---|
| ESP32 (station sol) | **accès simple** | rafale |
| Raspberry Pi (drone) | **rafale** | rafale |

Sur le Pi, l'accès simple n'améliore rien et la rafale donne 50 trames sur 50
en réception seule. Sur l'ESP32 c'est l'inverse pour la lecture. **Ne pas
transposer un réglage d'un hôte à l'autre sans remesurer** — la fiche
technique recommande la rafale, et la mesure la contredit sur un seul des
deux.

## Bug ouvert : l'ESP32 n'émet pas au-delà de ~12 octets

Caractérisation précise, drone en réception seule, trames de taille croissante
émises par le firmware de production :

| taille | acceptées |
|---|---|
| 8 o | 2 / 4 |
| 12 o | 2 / 4 |
| 16 o | **0 / 4** |
| 24, 32, 50 o | **0 / 4** |

Falaise franche entre 12 et 16 octets, soit 13 et 17 octets écrits dans la
FIFO d'émission.

Ce qui est **écarté** par la mesure :

- la série et le décodage hexadécimal : le firmware relit et affiche les 32
  octets **exactement** conformes (diagnostic `TXDEC`, depuis retiré) ;
- la table de registres : comparée programmatiquement à celle du banc, **0
  écart** sur 35 registres ;
- la collision semi-duplex : le défaut persiste à 1 Hz comme à 400 ms ;
- la réception du drone : il **entend** les trames (il les rejette au CRC,
  il ne les ignore pas) ;
- l'écriture de la FIFO en accès simple : essayée, **pire** (aucune trame
  acceptée, contre 8 et 12 octets qui passaient). La rafale est restaurée.

Les octets rejetés côté drone montrent la forme du défaut : en-tête
**parfaitement** juste — magic `4C 4B`, version, mode, numéro de séquence,
`farRssi` et `farFlags` cohérents — puis, à partir de l'offset ~17, une
garbage **identique d'une trame à l'autre** (`81 6D 43 DF 79 28 FE ED`). Pas
du bruit : de la donnée périmée constante.

Le firmware de banc, avec le même code d'émission et la même table, passait
32 octets. La différence restante entre les deux est l'environnement du
firmware de production : scrutation des boutons et du port série dans la même
boucle, et I2C actif pour les MCP23017.

### Piste à vérifier en premier : un conflit de broches

`GCS/ESP32/mapingESP.csv` assigne le voyant `HB_Sec` au **GPIO 35 en direct**
— or c'est le **MISO de la radio**, et sur ESP32 la broche 35 est en entrée
seule, incapable d'être une sortie. `setup()` y fait pourtant
`pinMode(35, OUTPUT)` puis `digitalWrite(35, LOW)` avant d'initialiser le SPI.

Ce conflit est réel et doit être corrigé quoi qu'il arrive : il suffit qu'une
commande `LED:` arrive pour que le firmware réécrive cette broche en pleine
transaction SPI. C'est la première chose à éliminer avant de chercher plus
loin.

## Mise en garde sur les mesures du sens station sol → drone

**Les conclusions tirées dans cette passe sur ce sens ne sont pas fiables, et
sont retirées.**

Une émission interne de 32 octets, code strictement identique, a donné
**30 trames sur 31** puis, vingt minutes plus tard, **0 sur 31**. Aucun
changement entre les deux. La mesure n'est donc pas stable, et les variations
que j'ai attribuées à des modifications de code étaient du bruit.

Sont ainsi **invalidées** : le seuil prétendu à ~12 octets, l'effet d'un délai
avant émission, l'effet de l'ordre des appels dans `loop()`, et l'hypothèse
d'une corruption propre au chemin série. Les échantillons de 12 à 15 trames
sur lesquels je m'appuyais ne permettaient rien de tel.

### Ce qui reste établi

- **Le sens drone → station sol fonctionne bien** : 39 trames sur 39 à travers
  le pont de production, et 23 sur 26 dans un autre essai.
- **Deux régressions réelles** introduites en extrayant `cc1101_radio.hpp`,
  corrigées : boucle de réessai supprimée, et lecture de FIFO repassée en
  rafale. Elles méritent néanmoins d'être revérifiées avec un protocole
  correct, puisqu'elles ont été diagnostiquées sur de petits échantillons.
- **Il n'y a aucun conflit de broches.** La radio utilise 14, 35, 13, 4 ; les
  voyants directs 18, 19, 23, 26 ; les boutons 25 et 33 ; les MCP en I2C.
  J'avais affirmé le contraire en lisant la colonne `Index` de
  `mapingESP.csv` comme un GPIO — le GPIO est la **dernière** colonne.
- **La table de registres est identique** entre le banc et l'en-tête partagé :
  comparée programmatiquement, 0 écart sur 35.
- **Le panneau n'est pas en cause** : désactivé entièrement, le défaut
  persiste.

### Protocole à suivre avant de reprendre

1. Figer la configuration et ne plus rien changer.
2. Mesurer **au moins 100 trames** par point, et répéter chaque point **trois
   fois** à plusieurs minutes d'intervalle.
3. Si la dispersion entre répétitions dépasse l'effet cherché, aucune
   conclusion n'est permise : il faut d'abord stabiliser le montage
   (position des antennes, alimentation, environnement 433 MHz).
4. Seulement ensuite, changer une variable à la fois.

Le banc multi-liens sait déjà faire les répétitions ; c'est le bon outil pour
cela, à condition de lui demander des durées suffisantes.

## Mode `stream` : la mesure qui dimensionne vraiment

Sens unique, cadence réglée, sans écho — le sol compte ce qui arrive.
C'est le mode qui répond à « ce lien peut-il porter mes N paquets par
seconde ». Ni `flood` (qui émet sans limite) ni `ping` (qui impose un
aller-retour coûtant le double en temps d'air) n'y répondent.

```sh
# au sol
python3 tools/hw_debug/link_bench_gcs.py cc1101 --duration=56
# sur le drone
./tools/link_bench.sh stream --link=cc1101 --interval=125 --duration=40 --size=21
```

### Résultat : le SDL porte les 8 Hz visés, avec de la marge

Trames de 21 octets — la taille réelle du plus gros paquet `Links=BOTH`.

| cadence | émises | reçues | perte | livré |
|---|---|---|---|---|
| 2 Hz | 79 | 72 | 8,9 % | 1,8 tr/s |
| 5 Hz | 199 | 192 | 3,0 % | 4,9 tr/s |
| 8 Hz | 319 | 311 | **2,5 %** | 7,8 tr/s |
| 8 Hz (répétition) | 239 | 235 | **1,7 %** | 7,9 tr/s |
| 16 Hz | 481 | 472 | **1,9 %** | 15,8 tr/s — 2,7 kbit/s |
| 25 Hz | 745 | **1** | **99,9 %** | effondrement |

Les 8 Hz sont donc tenus à ~2 % de perte, **répétable**, et le lien tient
encore 16 Hz : il y a un facteur deux de marge sur le besoin.

Le 8,9 % à 2 Hz porte sur 79 trames seulement (±3 %) et n'est pas significatif
face aux points suivants.

### L'effondrement à 25 Hz est la capacité de l'air, pas le récepteur

À 21 octets utiles, une trame occupe 45 octets sur l'air (préambule 16,
synchro 2, longueur 1, charge 21, CRC 2), soit 37 ms à 9,6 kbit/s :

| cadence | occupation de l'air |
|---|---|
| 8 Hz | ~30 % |
| 16 Hz | ~60 % |
| 25 Hz | ~94 % |

À 94 % il n'y a plus d'intervalle entre les trames et tout s'écroule. La
limite est physique et calculable, pas logicielle.

### Le cadencement compte autant que la cadence

Comparaison frappante, à débit moyen comparable :

| mode | cadence moyenne | livré |
|---|---|---|
| `flood` (sans cadencement) | 16,3 tr/s | **1 trame sur 326** |
| `stream` (cadencé) | 15,8 tr/s | **472 sur 481** |

Même débit moyen, résultat opposé. En saturation les trames partent collées,
et le récepteur n'a pas l'intervalle dont il a besoin pour se remettre en
écoute. **Un débit moyen ne dit rien si l'on ne précise pas le cadencement.**

C'est aussi ce qui explique le pessimisme des mesures précédentes : `flood` et
`ping` mesuraient tous deux des régimes qui ne ressemblent pas à l'usage réel
de la télémétrie.

### Réserve réglementaire

À 8 Hz on occupe déjà ~30 % de l'air, bien au-delà du rapport cyclique de
10 % usuellement admis en 433 MHz. Ce n'est pas un problème de fonctionnement
mais c'en est un de conformité, à trancher selon l'usage.

## PDL activé — première mesure réelle

### Mise en service

Canal **1 (2412 MHz)**, choisi pour ne recouvrir ni l'un ni l'autre des liens
SSH : celui du drone est à 5500 MHz (canal 100), celui de la station sol à
2462 MHz (canal 11). Prendre le canal 11 aurait coupé la station sol, le
canal 100 le drone.

Interfaces : `wlan1` sur le drone, `wlx54c9ff02f628` au sol. Mon SSH passant
par `wlan0` et `wlo1`, les toucher était sans risque pour la session — à
vérifier avant, pas après.

```bash
nmcli dev set <if> managed no        # piège connu : NM change le canal
ip link set <if> down
iw dev <if> set monitor otherbss
ip link set <if> up
iw dev <if> set channel 1
iw dev <if> info                     # VÉRIFIER, jamais supposer
```

Processus, wfb-ng étant compilé dans `/home/jules/wfb-ng` sans installation :

| machine | commande |
|---|---|
| drone | `./wfb_tx -p 0 -u 5602 -K drone.key wlan1` |
| drone | `./wfb_rx -p 1 -c 127.0.0.1 -u 5601 -K drone.key wlan1` |
| sol | `./wfb_rx -p 0 -c 127.0.0.1 -u 5602 -K gs.key wlx54c9ff02f628` |
| sol | `./wfb_tx -p 1 -u 5601 -K gs.key wlx54c9ff02f628` |

Ports radio séparés par sens (0 descendant, 1 montant) pour que les deux
directions ne se mélangent pas. Ports UDP 5601/5602, ceux de TelMain.

Le `txpower -30 dBm` affiché est l'indice brut recopié
(`rtw_tx_pwr_idx_override=30`), pas une puissance. Sans ce paramètre l'étage
radio reste muet alors que les compteurs montent — cf. section 10.

### Résultat : le lien porte des données, sans perte

Trames de 64 octets, 10 par seconde, 25 s :

| mesure | valeur |
|---|---|
| drone → sol | **250 / 250 — 0 % de perte** |
| aller-retour | 242 / 250 — 3,2 % |
| débit | 5,1 kbit/s livré |

### Mais la latence est dominée par le blocage FEC

| | valeur |
|---|---|
| minimum | **2,83 ms** |
| moyenne | 80,1 ms |
| p95 | **405 ms** |
| maximum | 705 ms |
| gigue (écart-type) | **155 ms** |

Le minimum à 2,83 ms montre ce dont le lien est capable. La moyenne à 80 ms et
le p95 à 405 ms viennent du **codage correcteur par blocs** : par défaut
`k=8, n=12`, donc `wfb_tx` attend **huit paquets** avant d'émettre un bloc. À
10 paquets par seconde, remplir un bloc prend 800 ms — ce qui apparaît tel
quel dans l'inter-arrivée maximale mesurée au sol : **801,3 ms**.

Ce n'est pas une limite du lien, c'est un réglage inadapté à un faible débit
de paquets. `wfb_tx` expose `-T fec_timeout` exactement pour ça : vider un
bloc incomplet au bout d'un délai. La valeur par défaut est 0, donc désactivé.

**À régler avant toute conclusion sur la latence du PDL** : soit `-T 20000`
(20 ms), soit un bloc plus court (`-k 1 -n 2`). Les deux se paient en
robustesse, c'est un arbitrage à trancher.

## RSSI du PDL : relais plutôt que parseur embarqué

### Le parseur d'origine ne pouvait pas marcher

`WFB_NG_Driver::parseStatsLine` cherchait des champs **JSON** (`"rssi"`,
`"lost"`, `"recv"`) sur un **socket TCP**. La vraie sortie de cette version de
wfb-ng est du texte tabulé sur **stdout** :

```
RX_ANT  2412:1:20  0  7:-46:-44:-44:25:27:28
```

Ni le format ni le transport ne correspondaient. Le commentaire du code
disait que les noms de champs restaient à confirmer — c'est fait, ils ne
correspondent pas. Au moins la dégradation était propre : faute de
correspondance, `rssiDbm()` restait vide et le pilote se comportait en
transport nu, sans jamais remonter de valeur inventée.

### `tools/wfb_stats_relay.py`

Lit la sortie de `wfb_rx` et republie en **datagramme UDP loopback** :

```sh
./wfb_rx -p 1 -c 127.0.0.1 -u 5601 -K drone.key wlan1 \
  | python3 tools/wfb_stats_relay.py --port 5699
```

Pourquoi un relais plutôt qu'un parseur dans le pilote : **ce format est
celui de wfb-ng, pas le nôtre**, et il change entre versions — on vient d'en
avoir la preuve. Mieux vaut qu'un script casse qu'un pilote du logiciel de
vol. La fragilité reste dehors.

Pourquoi pas un tube nommé : l'ouverture d'un FIFO en écriture **bloque
jusqu'à ce qu'un lecteur l'ouvre**. wfb-ng ne démarrerait donc pas avant le
logiciel de vol, et si celui-ci cesse de lire, les 64 Ko de tampon se
remplissent et `wfb_rx` **bloque en écriture** : la liaison radio s'arrêterait
parce que personne ne lit les statistiques. Un datagramme UDP, lui, se perd
sans conséquence si personne n'écoute.

Disposition publiée, offsets fixes comme les paquets `TELEM` :

| offset | type | champ |
|---|---|---|
| 0 | u16 | magic `0x5751` |
| 2 | u8 | version |
| 3 | i8 | `rssi` — **le pire des antennes** |
| 4 | i8 | `snr` — le pire des antennes |
| 5 | u8 | nombre d'antennes vues |
| 6 | u32 | reçus |
| 10 | u32 | perdus |

**Le pire et non le meilleur**, décision assumée : avec une diversité 2T2R le
récepteur utilise la meilleure chaîne, donc le pire RSSI ne mesure pas la
marge de liaison mais la **santé du système d'antennes**. Une antenne
débranchée se voit immédiatement, là où une moyenne la masquerait.

Le relais groupe les lignes par horodatage — `wfb_rx` en émet une par antenne,
toutes au même instant — pour ne jamais publier une vue partielle.

### Côté pilote

`connectStats()` **écoute** désormais un socket UDP au lieu de se connecter en
TCP, et `refreshStats()` vide la file pour ne garder que le dernier datagramme
(une mesure de qualité périmée n'a aucun intérêt, et accumuler du retard sur
le chemin RT serait pire que de perdre des échantillons). Le réassemblage de
lignes et son tampon de 4 Ko ont disparu : le datagramme est autoporteur.

La logique de péremption à 5 s est conservée telle quelle.

### Vérifié sur le matériel

```
qualite ici       : rssi -48 dBm
```

avec les deux antennes du drone à −44 et −48 dBm : c'est bien le pire qui est
remonté. 248 échos sur 250, 0,8 % de perte.

Les deux antennes du schéma du tableau de bord sont **PDL et SDL**, pas les
deux chaînes du PDL : `pdl_rssi` et `sdl_rssi` de `LINK_QUALITY` suffisent donc.

À noter : `main.cpp` construit encore des `UdpTelemetryDriver`, donc rien de
tout cela n'est actif dans le logiciel de vol — la plomberie existe, le
branchement reste à faire.
