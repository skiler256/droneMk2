# SysMonitoring — pipage et gestion des paquets

Documentation de référence du composant `SysMonitoring` : comment un
paquet circule de bout en bout (source de vérité → C++ → fil → GCS), et
comment sont organisées les 4 Tasks. Complète `GUIDE_COMPOSANTS.md` (guide
générique d'écriture d'un composant) et `ARCHITECTURE_IPC.md` (IPC/
watchdog) — ce document est spécifique à SysMonitoring.

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

## 9. Ce qui n'est PAS encore fait

- **Sourcing réel des données** : `TelMainTask`/`TelSecTask` construisent
  encore les paquets avec des valeurs par défaut (`TELEM::PosPkt{}`, etc.)
  — `nav_`/`sf_` n'exposent pour l'instant que des champs de stub
  (`bloup`/`blip`), pas les payloads POS/VELNED/ATTITUDE/GPSFIX réels.
- **Relai vidéo** : `VideoTask::loop()` poll bien `videoSource_.pollFrame()`
  mais ne relaie nulle part (`TODO` explicite) — pas de consommateur
  défini (dashboard, export réseau...).
- **Dispatch GCSRx** : les commandes reçues (`CMD_ARM`, `CMD_WAYPOINT`,
  ...) sont juste loguées (`std::cout`), pas encore routées vers
  `MissionControl`/`Navigation` (qui n'existent pas encore pour la
  logique métier correspondante).
- **`WFB_NG_Driver`/`CC1101_Driver`** : non implémentés (cf. §7).
- **`SharedComMem`** : payload vital minimal, quasiment vide — SysMonitoring
  n'a pas vraiment d'état à restaurer au hot-restart (composant de
  canalisation/parsing, pas de state métier propre).
