# GCS — station sol

Station sol physique ("malette") : un PC (Node.js + dashboard web Chromium)
et un boîtier ESP32 dédié aux voyants d'alarme et à deux boutons
d'acquittement physiques. Télémétrie émulée en LAN (UDP loopback) tant que
les liens radio réels (wfb-ng primaire, CC1101 secondaire) ne sont pas
implémentés côté drone (cf. `HARDWARE.md` §2) — le nœud STM32 H562
initialement envisagé pour la télémétrie secondaire a été abandonné au
profit de l'ESP32 déjà en place pour le panneau d'alarme.

## 1. Architecture logicielle

```
drone (UDP loopback, TelMain/TelSec) ──► server.js ──► WebSocket ──► navigateur(s)
                                            │
                                            ├──► port série ──► boîtier ESP32 (voyants, boutons)
                                            └──► pactl (PulseAudio) ──► volume système
```

- **`server.js`** — sert `public/` (dashboard statique), relaie la
  télémétrie UDP (`TELMAIN_RX_PORT=5602`/`TELSEC_RX_PORT=5612`) vers les
  navigateurs connectés par WebSocket, et les commandes montantes
  (`CMD_ARM` etc.) en sens inverse. Calcule l'agrégat Master
  Warning/Caution pour les voyants ESP32 uniquement (`MW_GROUP` — ENG,
  PWR, FC, EKF, MAG, GPS, NAV, EMG → MW, le reste → MC), avec ack
  indépendant de celui du son (cf. §3). Sémantique "dark cockpit" :
  l'acquittement (MW_ACK/MC_ACK) éteint l'agrégat sans éteindre le voyant de
  défaut sous-jacent (seul un code qui redevient inactif l'éteint), et un
  nouveau voyant de la même catégorie réarme l'acquittement. Expose `/codes.csv`,
  `/alarms.csv`, `/mission_alarms.csv` (mêmes fichiers que la racine du
  repo, décodés côté navigateur) et `/api/volume`, `/api/volume/:percent`,
  `/api/mute/:state` (pilotent PulseAudio via `pactl`, cf. §5).
- **`public/`** — dashboard web statique. `js/link.js` est le seul fichier
  à modifier pour changer le transport (actuellement WebSocket JSON,
  format `{type:'telemetry', link, name, fields, ts}`) ; il découple
  totalement la réception réseau du rendu (`render.js`), qui tourne à sa
  propre fréquence. Le bloc « schéma drone » (`.scheme__*`, SVG inline dans
  `index.html`) a remplacé les panneaux Batterie et CPU Temp : chaque forme
  reçoit son état de deux sources distinctes — les codes de diagnostic, via
  la colonne `Schematic` d'`INDEX.csv`, et les mesures continues par seuils,
  la panne l'emportant toujours. Contrat détaillé : `SCHEMA_DRONE_SPEC.md`.
  `js/alarm.js` gère le son et la pop-up d'activation
  (cf. §3), `js/state.js`/`js/dom.js`/`js/render.js` la
  boucle d'affichage, `js/viewport3d.js` la vue 3D (three.js).
- **`debug-injector.js`** — process Node **séparé** (port 3001 par
  défaut, `GCS_DEBUG_PORT` pour changer), volontairement indépendant de
  `server.js` : un outil de debug ne doit jamais pouvoir ralentir ou
  planter le serveur réel. Sert `public-debug/` et expose `POST /api/code`
  pour injecter un faux paquet `CODE` UDP vers `server.js`, exactement
  comme le ferait le drone — teste tout le chemin réel (UDP → voyants ESP32
  → alarme sonore) sans toucher au code drone ni au dashboard.
- **`serial-bridge.js`** — pont port série ↔ ESP32, reconnexion avec
  backoff exponentiel plafonné (débranchement/reboot du boîtier tolérés),
  envoie toujours l'état **complet** des voyants (pas de delta — plus
  robuste à un paquet perdu ou une reconnexion).

## 2. Boîtier ESP32

Firmware volontairement "bête" (`GCS/ESP32/firmware/src/main.cpp`) :
aucune décision métier dedans (quel code allume quel voyant, priorité
d'alarme...), tout vit côté `serial-bridge.js`/`server.js`. Protocole série
ASCII, une commande par ligne :

```
PC  -> ESP32 : LED:label1,label2,...\n   (état complet des voyants actifs)
ESP32 -> PC  : BTN:label\n               (un appui, front descendant, debounce 30ms)
```

Câblage : `GCS/ESP32/mapingESP.csv` (labels de voyant/bouton → puce
MCP23017 + port/bit, ou GPIO direct) → génère
`firmware/include/esp_map_generated.h` via `gen_esp_map.py` (ne jamais
éditer ce header à la main). Deux MCP23017 (adresses I2C `0x20`/`0x21`,
16 voyants chacune) + 4 GPIO directs (DISARMED/ARMED/HB_Main/HB_Sec) + 2
boutons physiques (`MC_ACK`=GPIO25, `MW_ACK`=GPIO33). `Adafruit_MCP23X17`
(pas l'ancienne `Adafruit_MCP23017`), pinnée version 2.3.2.

## 3. Système de codes et d'alarmes

Deux tables séparées, éditables via `python3 tools/codes_editor.py`
(onglets "Codes" et "Alarmes") :

- **`INDEX.csv`** — dictionnaire des codes de diagnostic (même source que
  le firmware embarqué, cf. `ARCHITECTURE_IPC.md` §5). Colonne `Panel` :
  labels de voyants ESP32 à allumer. Colonne `Alarm` : id d'alarme sonore
  à jouer (optionnel — repli automatique sur `master_warning`/
  `master_caution` selon que `Panel` contient un label du groupe MW).
- **`ALARMS.csv`** — définit chaque voix sonore : `AlarmId;Label;
  SoundFile;Type;Priority;PeriodMs;Class`. `Type` = `Loop` (rejoue tant que
  la source reste active) ou `Oneshot` (un seul événement). `Class` = `MW`
  ou `MC` (détermine quel bouton d'acquittement la coupe). Fichiers son
  dans `public/sounds/<SoundFile>.{mp3,wav,mp4}` — un seul format suffit.
- **`MISSION_ALARMS.csv`** — `State;AlarmId`, déclenche une alarme sur une
  transition `MISSION_STATE` (ex. `HOLD;hold_entered`), jouée une seule
  fois à l'entrée dans l'état (jamais en boucle même si `Type=Loop`).

`public/js/alarm.js` résout et joue le son : un seul canal audio (un
haut-parleur), la voix la plus prioritaire **active** joue et préempte
immédiatement toute voix moins prioritaire à chaque tick (200ms) — jamais
coincée derrière une file d'attente. Départage par priorité puis par "le
plus en retard" pour les boucles à période fixe. L'acquittement
(`MW_ACK`/`MC_ACK`, boutons physiques du boîtier) coupe toutes les alarmes
actives de sa classe jusqu'à ce qu'une **nouvelle** occurrence réarme
automatiquement — état de son totalement indépendant de l'ack des voyants
(`server.js`), qui suit ses propres règles.

Autoplay navigateur : une pop-up bloquante au démarrage
(`.audiogate` dans `index.html`, logique dans `alarm.js`) demande
l'activation du son. Choix d'un overlay plutôt que d'un badge discret : une
alarme muette est un mode dégradé, l'opérateur doit l'avoir choisi
explicitement. « Continuer sans son » (ou Échap) ferme la pop-up et laisse
un indicateur `SON INACTIF` permanent, pour qu'un dégradé ne soit jamais
invisible.

La pop-up ne bloque jamais `.play()` lui-même — le navigateur est toujours
sollicité directement (`.play()` retenté à chaque tick en cas de rejet). En
kiosk (`--autoplay-policy=no-user-gesture-required`) la lecture aboutit sans
geste : la pop-up et l'indicateur s'effacent alors d'eux-mêmes
(`onPlaybackSucceeded`).

## 4. Volume

**Plus de widget dans la page.** Le curseur flottant (`public/js/volume.js`)
a été retiré : il se superposait au coin bas-gauche du dashboard et gênait
la lecture. Le volume se règle au niveau du système (ou par un futur bouton
physique sur la malette, qui est de toute façon plus approprié en
opérationnel qu'un curseur à la souris).

Les routes `server.js` restent en place et fonctionnelles — `/api/volume`
(GET), `/api/volume/:percent` et `/api/mute/:state` (POST), via `pactl` —
pour qu'un contrôle externe ou un futur widget puisse s'y rebrancher sans
retoucher au serveur.

## 5. Malette physique — état actuel

Deux approches, une construite et fonctionnelle aujourd'hui, une visée à
terme mais non aboutie faute de temps :

### 5.1 Install Debian classique (fonctionnelle, POC actuel)

Installeur Debian standard (pas un live) sur la machine du GCS, puis tout
le reste installé manuellement par SSH :

```bash
apt install -y openssh-server && systemctl enable --now ssh   # si pas coché à l'install
apt-get install -y nodejs npm chromium rsync                   # Node.js + navigateur
```

Déploiement du code : `GCS/` (sans `node_modules`, réinstallé sur place via
`npm install` — nécessaire pour les modules natifs comme `serialport`) plus
le dossier `data/` (`TELEMETRY.csv`, `INDEX.csv`, `ALARMS.csv`,
`MISSION_ALARMS.csv`) copié **un niveau au-dessus** du dossier `GCS/`
déployé (`server.js` les résout via `path.join(__dirname, "..", "data", "...")`).
Lancement au démarrage via un service systemd simple :

```ini
[Unit]
Description=GCS dashboard (Node.js)
After=network.target sound.target
[Service]
User=jules
WorkingDirectory=/home/jules/gcs/GCS
ExecStart=/usr/bin/node server.js
Restart=always
RestartSec=2
[Install]
WantedBy=multi-user.target
```

Prérequis utilisateur : groupe `dialout` (accès `/dev/ttyUSB0`, l'ESP32) et
`audio` (PulseAudio/`pactl`). Pas de kiosk : le navigateur (Chromium,
installé) s'ouvre manuellement sur `http://localhost:3000` depuis le
bureau normal — décision explicite pour simplifier le POC, pas
d'autologin/kiosk configuré.

### 5.2 Image Debian live (`GCS/live-image/`) — cible technique, non aboutie

Objectif : clé USB bootable en RAM (`toram`), Plymouth custom, Chromium
kiosk plein écran sans bureau visible mais avec une console d'échappement
(`Ctrl+Alt+F2`), persistance de `/opt/gcs` et du WiFi via une seconde
partition, WiFi générique (`iwd`) séparé d'un futur adaptateur RTL8812AU
dédié (DKMS, `svpcom/rtl8812au` v5.2.20) pour wfb-ng. Construite avec
`live-build`, sur une base Ubuntu — fork ancien et **très** différent du
`live-build` moderne Debian, avec de nombreux bugs contournés
explicitement dans `config/` (voir les commentaires en tête de chaque
fichier de `GCS/live-image/config/` et d'`auto/config` : mode Ubuntu
auto-détecté au lieu de Debian, `LB_INITSYSTEM` par défaut sur sysvinit au
lieu de systemd, `--build-with-chroot` cassé, hooks non exécutés à cause
d'une convention de nommage différente, `--isohybrid-options` parsé mais
jamais écrit dans la config, etc.).

**État à la mise en pause** : systemd comme PID 1, console physique,
SSH, WiFi (`iwd`), son (PulseAudio), volume, kiosk plein écran
(`matchbox-window-manager`) — tous validés au moins une fois sur la
machine cible réelle (HP Pavilion G7, BIOS legacy). **Bloqué par un bug
non résolu** : `BOOT FAIL` (erreur SYSLINUX, avant même le chargement du
noyau) au démarrage, reproduit sur 3 clés USB physiques différentes après
plusieurs reconstructions complètes. Pistes déjà écartées : clé USB
défaillante (le même symptôme est apparu sur 3 clés de modèles
différents), géométrie CHS isohybrid (`-h 255 -s 63 --forcehd0` appliqué
et vérifié — le warning `isohybrid: more than 1024 cylinders` a bien
disparu du log de build, mais le `BOOT FAIL` a persisté). Cause racine
non identifiée à l'arrêt du projet — piste suivante suggérée mais jamais
testée : comparer le comportement avec une image Debian live **officielle
non modifiée** flashée sur la même clé/machine, pour trancher entre un
problème côté build (`config/`) et un problème matériel spécifique à ce
port USB/BIOS.

Dossier volontairement conservé (non supprimé) : c'est l'approche cible
au retour de pause, techniquement supérieure à l'install classique sur
tous les plans (pas d'écriture disque en usage normal, kiosk propre, mise
à jour par simple réécriture de la clé).

## Limites connues

- `HoldSegment` n'engage jamais Nav côté drone (cf. `ARCHITECTURE_IPC.md`
  et `sitl/SITL.md`) — sans lien avec le GCS lui-même, mais affecte ce
  qu'un opérateur observe pendant un Hold.
- Pas de réponse GCS automatique aux questions `Hold`/`Emergency`
  (`queryAnswer`) — question d'architecture non tranchée, pas seulement un
  manque d'implémentation.
- `tools/dashboard/dashboard_server.py` (Python) et `GCS/server.js`
  (Node.js) sont deux implémentations indépendantes du même concept de
  dashboard télémétrie, sur les mêmes ports UDP — ne **jamais** les
  lancer en même temps (cf. `tools/README.md`). Duplication volontaire et
  temporaire, tant que les liens radio réels ne sont pas implémentés
  (cf. `HARDWARE.md` §2).
