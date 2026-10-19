# SysMonitoring — pipage et gestion des paquets

Documentation de référence du composant `SysMonitoring` : comment un
paquet circule de bout en bout (source de vérité → C++ → fil → GCS), et
comment sont organisées les 4 Tasks. Complète `GUIDE_COMPOSANTS.md` (guide
générique d'écriture d'un composant) et `ARCHITECTURE_IPC.md` (IPC/
watchdog) — ce document est spécifique à SysMonitoring.

Côté rôle système, `SysMonitoring` correspond à un **ECAM** (Electronic
Centralized Aircraft Monitor, terme Airbus — cohérent avec le FMA de
`NAVIGATION.md` §7 et les alarmes MW/MC déjà nommées à l'Airbus) : c'est
lui qui centralise l'état et le pousse vers l'équipage (GCS). `TelMainTask`
porte le lien **PDL**, `TelSecTask` le lien **SDL** (cf. `HARDWARE.md` §2).

## 1. Vue d'ensemble

```
TELEMETRY.csv (source de vérité)
        │
        ├─ tools/gen_telemetry.py ──────► include/drone/generated/telemetry.hpp
        │                                  (TELEM::PacketID, structs POD, kTable)
        │
        └─ tools/dashboard/telemetry_codec.py (même table, côté Python)

SysMonitoring (process isolé)
 ├─ TelMainTask ──► PacketScheduler(Main) ──► mainLink_ (ITelemetryLink) ──┐
 ├─ TelSecTask  ──► PacketScheduler(Sec)  ──► secLink_  (ITelemetryLink) ──┼──► UDP loopback (dev)
 ├─ GCSRxTask   ◄── mainLink_.poll() / secLink_.poll() ────────────────────┘      ou WFB-NG/CC1101 (prod)
 └─ VideoTask   ◄── videoSource_.pollFrame() (IVideoSource)

tools/dashboard/dashboard_server.py ◄── écoute UDP, sert dashboard.html
```

Un seul format de paquet (`TELEM::*`) est partagé par TelMain et TelSec —
TelSec (lien de secours bas débit) ne porte qu'un sous-ensemble léger
(`Links=BOTH` dans le CSV), TelMain porte tout + la vidéo.

## 2. `TELEMETRY.csv` — source de vérité

Colonnes : `PacketID;Direction;Priority;RateHz;RedundancyHz;Links;Fields`

- **PacketID** : identifiant texte. Son `TELEM::PacketID` (valeur numérique
  générée) est son **index de ligne dans le CSV** — l'ordre du fichier fait
  partie du format binaire. Ne jamais réordonner des lignes existantes ;
  ajouter uniquement en fin de fichier. (`tools/codes_editor.py`, onglet
  "Paquets télémétrie", respecte cette contrainte automatiquement : ajout
  en fin, édition en place, suppression décale — avec avertissement.)
- **Direction** : `DOWN` (drone → GCS) ou `UP` (GCS → drone).
- **Priority** : `Routine` ou `Critical` — utilisé par `PacketScheduler`
  pour arbitrer quel paquet partir en premier quand plusieurs sont dus.
- **RateHz** : fréquence d'émission périodique. `0` = paquet événementiel
  (envoyé seulement quand la donnée change, cf. `markDirty()` §4).
- **RedundancyHz** : pour un paquet événementiel uniquement — fréquence de
  renvoi périodique même sans changement, pour résister à la perte d'un
  paquet sur un lien lossy sans ACK (ex: `MISSION_STATE` à 1Hz).
- **Links** : `MAIN` (TelMain uniquement, ex: `POS` à 10Hz, trop lourd pour
  le lien de secours) ou `BOTH` (TelMain + TelSec, ex: `ARM_STATE`,
  `CMD_ARM`).
- **Fields** : `nom:type,nom:type,...`, types `u8/u16/u32/i8/i16/i32/f32`.

Édition : `python3 tools/codes_editor.py` (onglet "Paquets télémétrie"),
ou modifier le CSV à la main puis régénérer.

## 3. Génération — `tools/gen_telemetry.py`

Produit `include/drone/generated/telemetry.hpp` (régénéré automatiquement
par CMake, cf. `generate_telemetry` dans `CMakeLists.txt`, ne jamais éditer
ce header à la main) :

- `TELEM::PacketID` (enum stable, ordre du CSV)
- `TELEM::Direction`, `TELEM::Priority`, `TELEM::Link` (bitmask Main/Sec)
- une struct POD par paquet (`PosPkt`, `CmdArmPkt`, ...), chacune avec
  `kId`, `kSize`, `pack()`/`unpack()` — sérialisation little-endian
  explicite champ par champ (`memcpy`), pas de `reinterpret_cast` du
  struct entier (portable, pas de souci d'alignement/padding)
- `TELEM::kTable` : une `Meta{id, direction, priority, rateHz,
  redundancyHz, linkMask, size, name}` par paquet — c'est la table que
  `PacketScheduler` parcourt, pas les structs individuelles

## 4. Trame sur le fil — `Framing.hpp`

```
[PacketID:1 octet][payload applicatif:N octets]
```

Le lien (WFB-NG/CC1101/UDP factice) gère déjà sa propre trame bas niveau
(longueur du datagramme, CRC radio) — cet octet sert uniquement à
distinguer le type de paquet une fois le payload extrait par le driver.

```cpp
auto frame = TELEM::encode(TELEM::PosPkt{...});   // -> array<byte, 1+kSize>
mainLink_.send(frame);

auto bytes = mainLink_.poll();                     // -> span<const byte>
if (auto f = TELEM::decode(*bytes)) {
  auto pkt = TELEM::CmdModePkt::unpack(f->payload);
}
```

## 5. `PacketScheduler` — qui part quand

Un seul paquet part par cycle radio (le lien ne permet qu'un envoi à la
fois). Un `PacketScheduler` par lien (`mainSched_`, `secSched_` dans
`SysMonitoring`), filtré à la construction sur `Direction::Down` et sur le
`linkMask` du lien concerné.

Règles de choix (`nextDue()`, appelé à chaque itération de
`TelMainTask`/`TelSecTask::loop()`) :

1. **Critical bat toujours Routine**, dès que les deux sont dus.
2. **À priorité égale, le plus en retard gagne** (`now - lastSent -
   period`) — pas un round-robin naïf, pour éviter qu'un paquet haute
   fréquence (POS, 10Hz) affame silencieusement un paquet plus lent
   (HEALTH, 1Hz) sous contention.
3. **Paquet événementiel** (`rateHz == 0`) : dû si `markDirty(id)` a été
   appelé (changement détecté par la Task), OU si sa `redundancyHz`
   périodique est due même sans changement.
4. **Un événementiel en attente gagne tout départage à priorité égale** :
   son retard (`overdue`) est forcé au maximum, de sorte qu'un changement
   d'état réel passe toujours avant la simple redondance périodique d'un
   autre paquet de même priorité. Sans cette règle, la règle 2 pourrait
   faire attendre une transition de `MISSION_STATE` derrière le énième
   renvoi inchangé d'un paquet plus en retard.

```cpp
comp.mainSched_.markDirty(TELEM::PacketID::MISSION_STATE); // sur changement détecté
auto *due = comp.mainSched_.nextDue(now);
if (due) {
  // ... encode + send selon due->id ...
  comp.mainSched_.markSent(due->id, now); // seulement après envoi réussi
}
```

`markSent()` n'est appelé qu'après un `send()` réussi — un échec laisse le
paquet "dû", il repassera au cycle suivant plutôt que d'être perdu
silencieusement.

## 6. Les 4 Tasks

| Task | Rôle | Lien(s) | RT priority |
|---|---|---|---|
| `TelMainTask` | Émet tout paquet `Down` dû sur `mainSched_` | `mainLink_` | 70 |
| `TelSecTask` | Émet le sous-ensemble léger dû sur `secSched_` | `secLink_` | 75 (secours, priorité RT supérieure — arrêt d'urgence) |
| `GCSRxTask` | Poll les deux liens, décode, dispatch les commandes `Up` | `mainLink_` + `secLink_` | 65 |
| `VideoTask` | Poll les frames JPEG reçues | `videoSource_` | 40 (tolérante à la perte) |

Chaque Task est un simple dispatch `switch (due->id) { case ... encode +
send }` (`TelMainTask`/`TelSecTask`) ou un `poll()` + décodage
(`GCSRxTask`/`VideoTask`) — voir `src/SystemMonitoring/SystemMonitoring.cpp`.

**Seuls les codes MAJEURS (Critical/Emergency) sont relayés.** `CODE` est
`Priority=Critical` dans `TELEMETRY.csv`, et le ring des mineurs est à la fois
plus bruyant et dépourvu de notion de « clear » à observer (il s'auto-purge par
écrasement, cf. §5) — il n'y a donc pas de front à détecter dessus. De même,
`HEALTH` ne portant qu'un couple `(composant, santé)` par envoi, les cinq
composants surveillés **défilent** un par envoi plutôt que d'élargir le format
du paquet pour si peu de champs.

**État propre à chaque lien, jamais partagé.** `TelMainTask` et
`TelSecTask` tiennent chacune leur index de défilement `HEALTH` et leur
snapshot de codes majeurs : les deux liens n'émettent pas au même rythme,
donc leur progression doit être indépendante. Les événements `CODE` détectés
par diff (`scanCodeChanges`) sont mis en file **locale au process**, pas en
shm — même principe que `StateMachineOrderQueue` côté MissionControl, et
sans verrou ici puisque détection et consommation ont lieu dans le même
thread (`TelMainTask::loop()`).

## 7. Drivers — interface vs. implémentation

`SysMonitoring` ne connaît que des interfaces, jamais un driver concret :

```cpp
ITelemetryLink &mainLink_;   // TelMain : poll() + send()
ITelemetryLink &secLink_;    // TelSec  : poll() + send()
IVideoSource   &videoSource_; // Video  : pollFrame()
```

Injectées au constructeur par `main.cpp` — remplacer le driver de dev par
le driver hardware ne touche ni `SysMonitoring.hpp` ni `.cpp`. Portabilité
inter-véhicules : seule l'implémentation concrète change.

- **Dev** : `UdpTelemetryDriver`/`UdpVideoDriver` (`Driver/`) — UDP
  loopback, stdlib POSIX seulement, non bloquant. Ports définis dans
  `src/main.cpp` (`kUdpTelMain*`, `kUdpTelSec*`, `kUdpVideoLocalPort`).
- **Prod** : `WFB_NG_Driver` (TelMain + vidéo), `CC1101_Driver` (TelSec) —
  **squelettes déclarés, pas implémentés** (câblage hardware SPI/socket RF
  à faire séparément). Ne pas les instancier tant qu'ils n'ont pas de
  `.cpp` : le lien échouerait (méthodes déclarées sans corps).

## 8. Outillage GCS de dev

- `tools/dashboard/` : tableau de bord web (stdlib Python, pas de
  dépendance). Écoute les ports UDP de télémétrie, affiche la dernière
  valeur par paquet avec indicateur de fraîcheur, permet d'envoyer les
  commandes montantes. `python3 tools/dashboard/dashboard_server.py`.
- `tools/codes_editor.py` : onglet "Paquets télémétrie" pour éditer
  `TELEMETRY.csv` sans risque de casser l'ordre des `PacketID`.

### `sysmon_sitl` — injection shm à la main

`src/IntegrationTests/SYSMON_SITL.cpp` fait tourner `SysMonitoring` **seul**,
alimenté par des datagrammes UDP texte sur `127.0.0.1:15020` (même esprit que
`udp_bridge`, cf. `sitl/SITL.md` §2). Il écoute les **mêmes** ports de
télémétrie que `main.cpp`, donc le dashboard GCS l'observe sans aucune
configuration différente.

Commandes d'injection, directement copiables :

```sh
echo -e "CMD=STATE\nnorth=10\neast=0\ndown=-5\nvn=1\nve=0\nvd=0\nroll=0\npitch=0\nyaw=90" | nc -u -w0 127.0.0.1 15020
echo -e "CMD=GPS\nlat=48.858\nlon=2.294\nalt=42\nhdop=0.9\nvdop=1.1\nfixtype=3\nnumsats=11"  | nc -u -w0 127.0.0.1 15020
echo -e "CMD=SAT\nindex=0\nid=12\nstrength=180\nquality=5"                                     | nc -u -w0 127.0.0.1 15020
echo -e "CMD=LIDARPOINT\nbearing=45\ndist_mm=1200\nintensity=200"                               | nc -u -w0 127.0.0.1 15020
echo -e "CMD=LIDARCLEAR"                                                                           | nc -u -w0 127.0.0.1 15020
echo -e "CMD=BATTERY\nvoltage=22.1\ncurrent=8.4\npct=76"                                        | nc -u -w0 127.0.0.1 15020
echo -e "CMD=MISSIONSTATE\nstate=Mission"                                                         | nc -u -w0 127.0.0.1 15020
echo -e "CMD=FMA\nlateral_engaged=0\nlateral_armed=0\nvertical_engaged=0\nvertical_armed=0\ntarget_altitude=10\ntarget_speed=3" | nc -u -w0 127.0.0.1 15020
echo -e "CMD=HEALTH\ncomponent=SensorFusion\nhealth=SICK"                                        | nc -u -w0 127.0.0.1 15020
echo -e "CMD=CODE\ncomponent=SensorFusion\ncode=SFS004\naction=raise"                           | nc -u -w0 127.0.0.1 15020
echo -e "CMD=CODE\ncomponent=SensorFusion\ncode=SFS004\naction=clear"                           | nc -u -w0 127.0.0.1 15020
```

## 9. Ce qui n'est PAS encore fait

- **`ARM_STATE`** : toujours à valeurs par défaut — `MavlinkInterface` ne
  parse/stocke pas encore le bit `MAV_MODE_FLAG_SAFETY_ARMED` de
  `HEARTBEAT.base_mode` dans `SharedFCStatus`. Gap côté MavlinkInterface,
  pas SysMonitoring.
- **Relai vidéo** : `VideoTask::loop()` poll bien `videoSource_.pollFrame()`
  mais ne relaie nulle part (`TODO` explicite) — pas de consommateur
  défini (dashboard, export réseau...).
- **`WFB_NG_Driver`/`CC1101_Driver`** : non implémentés (cf. §7) — pas de
  RSSI/qualité de lien matérielle réelle, seul le timestamp du dernier
  `HEARTBEAT_GCS` par lien est dispo (`SharedComMem::lastHeartbeatMain/Sec`).
  Le RSSI descendant (vu par le récepteur du GCS) n'est de toute façon pas
  une donnée que le drone transmet — c'est une mesure locale du futur
  matériel radio du GCS (H562), pas un paquet `TELEM::*`.
- **`SharedComMem`** : n'a plus grand chose de "quasiment vide" (cf. §10)
  mais SysMonitoring n'a toujours pas d'état métier propre à restaurer au
  hot-restart (composant de canalisation/parsing).
- **Tracé du plan de vol côté GCS** : le plan (`MISSIONPLAN_CONV`) est un
  fichier binaire chargé localement par MissionControl, jamais transmis au
  GCS — un mécanisme de upload/streaming du plan est un choix d'archi à
  part, pas juste un paquet `TELEM::*` de plus (contenu de taille variable).

*(Section historique : "consommation côté MissionControl" a été résolue —
`TGCSCmdRouter`/`TSafetyMonitor` ont maintenant toute leur logique, cf.
MISSIONCONTROL.md.)*

## 10. `SharedComMem` — commandes GCS et heartbeat

- **`gcsCmds`** (`RingBuffer<GCSCommand, 8>`) : `CMD_ARM`/`CMD_DISARM`/
  `CMD_MODE`/`CMD_WAYPOINT`/`CMD_RTL`/`CMD_LAND` poussés par `GCSRxTask`
  (`pushGCSCommand`), dépilés côté MissionControl (`popGCSCommand`, pas
  encore appelé — cf. §9). `GCSCommand` est une struct à plat (pas
  d'union) : les champs hors-sujet pour un `type` donné restent à zéro,
  compromis simple accepté pour une commande ponctuelle GCS.
- **`CMD_EMERGENCY`** court-circuite volontairement ce FIFO : signalée via
  `CODES::SMO001` (`sysState_.raiseCode`), lue par `TSafetyMonitor`
  indépendamment du rythme de dépilement — un ordre d'urgence ne doit
  jamais attendre son tour derrière une commande normale en file.
- **`lastHeartbeatMain`/`lastHeartbeatSec`** : horodatage du dernier
  `HEARTBEAT_GCS` reçu par lien, mis à jour par `GCSRxTask`. Sert de proxy
  de fraîcheur/qualité de lien en l'absence de RSSI matériel (cf. §9).

## 11. Champs tableau — `type[N]` dans `TELEMETRY.csv`

Un champ peut être un tableau taille fixe (`u16[360]`, `u8[16]`, ...) en
plus des types scalaires — même principe que le `bytesN` de
`MISSIONPLAN.csv`, mais typé élément par élément (pas un blob opaque).
`gen_telemetry.py` n'a rien eu à changer pour le générer : `pack()`/
`unpack()` font déjà un `memcpy` générique par champ, qui fonctionne tel
quel sur un `std::array`. Côté Python (`telemetry_codec.py`), le format
`struct` utilise le préfixe de répétition natif (`"360H"`), et
`decode()`/`encode()` regroupent/aplatissent les éléments par champ.

Utilisé par `SAT_INFO` (détail par satellite, capé à 16) et `LIDAR_SCAN`
(nuage de points body-frame, 360° à 1°/index).

**Attention taille** : `LIDAR_SCAN` fait ~1085 octets sur le fil (le plus
gros paquet `TELEM::*` de loin) — `UdpTelemetryDriver::rxBuf_` a dû passer
de 512 à 1536 octets (sinon troncature silencieuse par `recv()`), et
`Meta::size` de `uint8_t` à `uint16_t` (255 octets ne suffisait plus). Si
un futur paquet tableau dépasse ~1536 octets sur un vrai lien radio bas
débit (pas juste le loopback LAN de dev), il faudra le découper en
plusieurs paquets — pas fait ici, non nécessaire tant que TelMain/TelSec
restent émulés en LAN.

## 12. `CPU_TEMP` — `ISystemTempSource`

Comme `ITelemetryLink`/`IVideoSource`, une interface injectée au
constructeur (`Driver.hpp`) — mais une seule implémentation concrète
(`LinuxThermalZoneDriver`, lit `/sys/class/thermal/thermal_zone0/temp`),
pas de variante factice dev/prod séparée : cette API kernel existe sur
toute machine Linux avec capteur thermique, y compris une machine de dev
x86 (contrairement aux liens radio WFB-NG/CC1101 qui nécessitent le
hardware).

## 13. `TASK_RATE` — fréquence réelle des `Task`

Chaque `TaskConfig` porte une `loopFrequency` *demandée*. Rien ne garantit
qu'elle soit tenue : une boucle qui déborde de son budget, un driver qui
bloque en lecture ou une préemption par une `Task` plus prioritaire font
tomber la cadence réelle sans qu'aucun watchdog ne réagisse — le heartbeat
continue d'arriver, simplement moins souvent, et tant qu'il reste sous le
`timeout` la `Task` est considérée vivante. `TASK_RATE` rend cet écart
observable.

### Qui mesure

La `Task` incrémente un `std::atomic<uint32_t>` après chaque `loop()`
(`ComponentBase.hpp`). C'est son seul coût : un `fetch_add` en
`memory_order_relaxed`, pas de lecture d'horloge ni d'accès shm sur le
chemin de vol.

Tout le calcul vit dans `ComponentWatchdog::publishRates()`
(`watchdog.hpp`), appelé depuis `loop()`. Le watchdog conserve par `Task`
le compteur précédent et l'instant du dernier échantillon — champs non
atomiques de `TaskHandle`, puisqu'un seul thread y touche. Au premier tick
au-delà de 1000 ms il calcule `delta / elapsed` et publie.

Deux raisons à ce découpage :

- le chemin RT ne porte qu'un incrément, et c'est le watchdog — déjà conçu
  pour prendre des verrous shm et déjà porteur d'une erreur sticky — qui
  paie l'accès à la mémoire partagée ;
- l'arithmétique est mono-thread, donc sans race et sans atomique. Le seul
  point de contact entre les deux threads est un `uint32_t` monotone : lire
  une valeur en retard d'une itération fausse la mesure d'un pour-cent,
  jamais la cohérence.

La fenêtre d'une seconde n'est pas un réglage de confort. Déduire la
période de deux `lastHeartbeat` successifs n'aurait rien coûté, mais le
watchdog tourne à 100 Hz : pour une `Task` à 100 Hz l'écart mesuré serait
du bruit d'ordonnancement replié, pas une fréquence. Une seconde donne ~100
échantillons par mesure à cette cadence.

Le compteur n'est pas remis à zéro par un restart de `Task` : la mesure
reste juste à travers un hot-start, au prix de ne pas signaler le trou.

### Stockage

`SharedSysStateMem` porte deux tableaux
`[ComponentID::Count][kMaxTasksPerComponent]` de `float` :

| Champ | Écrit par | Quand |
|---|---|---|
| `taskHz` | `setTaskHz` | à chaque fin de fenêtre |
| `taskTargetHz` | `setTaskTargetHz` | une fois, à la première publication |

`kMaxTasksPerComponent = 8` laisse de la marge sur le composant le plus
fourni (`MavlinkInterface`, 5 `Task`). Les `TaskID` sont numérotés 0..n-1
par composant, donc ils indexent directement le tableau. Les deux champs
entrent dans `computeChecksum(id)` : une mesure corrompue ne passe pas
inaperçue. `0` signifie « pas encore mesurée », jamais « arrêtée » — un
composant qui vient de démarrer n'a rien publié.

Les deux setters sont du self-report, comme `setHealth` : un composant ne
publie que ses propres `Task` (invariant `ARCHITECTURE_IPC.md` §2.3).

Publier la cible plutôt que de la laisser côté GCS évite de dupliquer les
19 `loopFrequency` en JavaScript, où elles dériveraient au premier
changement de `TaskConfig`.

### Transport

`TASK_RATE;DOWN;Routine;5;0;MAIN;component:u8,task_id:u8,hz:f32,target_hz:f32`

Un seul couple `(composant, Task)` par paquet, 10 octets utiles.
`buildTaskRatePkt()` fait défiler les `5 × 8` slots et saute ceux à
`hz == 0` ; à 5 Hz les 19 `Task` réelles sont rafraîchies en ~4 s, ce qui
suffit pour une mesure qui ne bouge que toutes les secondes. Le balayage
est borné au nombre de slots : au démarrage, quand aucune mesure n'existe
encore, la fonction renvoie `nullopt` et l'appelant fait un `markSent`
inconditionnel — sinon `TASK_RATE` resterait dû en permanence et
monopoliserait le lien.

`Links=MAIN` : la mesure est un outil de diagnostic, elle n'a rien à faire
sur le lien de secours. Conséquence utile, un seul des deux `switch`
d'émission est concerné (le `PacketScheduler` de `TelSec` ne voit même pas
le paquet) — contrairement à `ARM_STATE`, dont l'oubli côté `TelSec` avait
été un piège silencieux.

Le paquet est ajouté **en fin** de `TELEMETRY.csv` : son `PacketID` vaut 27
et aucun identifiant existant ne bouge.

### Côté GCS

`link.js` range chaque mesure dans `component.tasks[task_id]` et met à jour
la carte du composant. La carte n'a qu'une ligne de note : elle affiche la
`Task` la plus en retard sur sa cible, sous la forme `T2 38/50 Hz`, et
reste vide tant que toutes les `Task` sont au-dessus de 90 % de leur cible
— marge qui absorbe le jitter d'ordonnancement et la granularité de la
fenêtre de mesure. La fréquence en gros caractères reste celle de la `Task`
0, la boucle principale de chaque composant.

L'entrée de composant est créée par le premier paquet qui en parle,
`HEALTH` ou `TASK_RATE` : l'ordre d'arrivée n'est pas garanti. `hz` reste
`null` — affiché `--`, pas `0 Hz` — tant qu'aucun `TASK_RATE` n'est arrivé.

`sim.js` ne fabrique plus de sinusoïde décorrélée : il part des
`loopFrequency` réelles, applique un jitter sous la marge de 10 %, et place
volontairement une `Task` à 75 % de sa cible pour que la vitrine exerce le
chemin d'affichage de l'écart. La règle de note est partagée
(`GCS.rateDeficitNote`), donc la vitrine et le vol rendent la carte de la
même façon.

## 14. `ESC_RPM` — régime par moteur

`ESC_TELEMETRY_1_TO_4` est décodé par `MavlinkInterface` à 2 Hz et remplit
`FCEscStatus` : régime, courant et température **pour chacun des 4 moteurs**
(`SharedFCStatus.hpp`). Jusqu'ici rien de tout cela ne sortait du drone.

`ESC_RPM;DOWN;Routine;2;0;MAIN;rpm:f32[4]` transporte les régimes seuls,
16 octets utiles. 2 Hz, soit exactement la cadence de la source : émettre
plus vite ne ferait que renvoyer la même mesure. `Links=MAIN`, comme les
autres paquets de diagnostic.

Courant et température par ESC restent à bord pour l'instant. Ils sont
disponibles au même endroit et s'ajouteront par un champ supplémentaire si
le besoin se confirme — l'ordre des `PacketID` n'en sera pas affecté puisque
seule la définition du paquet change.

Le tableau reste à `{0,0,0,0}` si le FC n'a jamais émis `ESC_TELEMETRY`.
Côté affichage, c'est un cas à distinguer d'un moteur arrêté : quatre zéros
simultanés signifient « pas de donnée », pas « quatre moteurs à l'arrêt ».

### Ce que la grandeur veut dire

Le régime absolu suit les gaz : les quatre moteurs montent et descendent
ensemble. L'information utile pour le dashboard est l'**écart au régime
moyen des quatre**, qui révèle une répartition de charge déséquilibrée —
centre de gravité décalé, hélice abîmée, moteur qui faiblit. Le paquet
transporte les régimes bruts et laisse ce calcul au GCS : la moyenne n'a pas
à être calculée deux fois, et une valeur brute reste interprétable
autrement.

Le regroupement en « avant » et « arrière » pour une vue de côté dépend de
la numérotation des sorties moteur du contrôleur de vol pour la
configuration du châssis, qui n'est pas la numérotation géométrique
naturelle. Ce mapping appartient donc au GCS, pas au drone.

## 15. `WFB_NG_Driver` — lien principal PDL

### wfb-ng est un transport UDP

`wfb_tx` lit sur un port UDP local et diffuse en wifi broadcast ; `wfb_rx`
reçoit et réécrit sur un port UDP local. Les deux sont des process séparés,
sur le Pi comme sur la station sol. Le logiciel de vol n'ouvre donc aucun
socket RF : il parle à wfb-ng en UDP loopback, exactement comme il parlait au
pont de développement.

C'est pourquoi `WFB_NG_Driver` **hérite** de `UdpTelemetryDriver` au lieu de
réimplémenter un transport. Le passage du lien émulé au lien réel est un
changement de ports, pas de code.

### Ce que la classe ajoute

La santé réelle du lien. Sans elle, le drone ne distingue pas un GCS
silencieux d'une radio qui émet dans le vide : la seule information
disponible est l'absence de `HEARTBEAT_GCS`, qui arrive trop tard et ne dit
pas pourquoi.

`WFB_NG_Driver` ouvre donc une connexion TCP sur le socket de statistiques de
wfb-ng, en lit les lignes et en extrait deux grandeurs :

| Méthode | Sens |
|---|---|
| `rssiDbm()` | Niveau reçu **par le drone**, donc qualité du lien montant |
| `lossRatio()` | Paquets perdus / reçus sur la fenêtre rapportée |

La qualité du lien **descendant** ne se mesure pas ici : elle se mesure au
récepteur de la station sol, qui est le seul à la voir. Les deux chiffres
sont distincts et ne se remplacent pas.

### Dégradation

Tout est optionnel par construction :

- `statsPort = 0` désactive la fonction, le driver est alors un
  `UdpTelemetryDriver` ordinaire ;
- service absent ou connexion refusée : nouvelle tentative à la fenêtre
  suivante, jamais de blocage ;
- lignes présentes mais champs inconnus : les deux valeurs restent vides,
  le transport continue ;
- plus de ligne valide depuis 5 s : les valeurs sont **effacées** plutôt que
  laissées en place. Une qualité de lien figée est pire qu'absente.

La lecture est limitée à une tentative par seconde : les statistiques ne
bougent qu'à cette échelle, et `poll()` est appelé depuis une boucle RT où
chaque appel système compte. Le socket est non bloquant, `connect()` compris.

### Point à confirmer au banc

**Les noms de champs du parseur ne sont pas vérifiés** contre une sortie
réelle de wfb-ng. `parseStatsLine()` cherche `"rssi"`, `"lost"` et `"recv"`.
Tant qu'ils ne correspondent pas, `rssiDbm()` et `lossRatio()` restent vides
et le lien fonctionne quand même — le défaut est silencieux côté transport,
visible côté qualité. C'est le premier point à régler sur le Pi, avec la
sortie réelle sous les yeux.

Les tests de `test_drivers.cpp` exercent le parseur contre un faux serveur
TCP local : ils valident le découpage en lignes, le fragment à cheval sur
deux lectures et l'absence de service, pas le format réel.
