# GCS live-image

Image Debian Live "appliance" pour le PC de la malette GCS : boot depuis la
clé USB, copie tout en RAM (`toram`), lance directement le dashboard GCS en
plein écran (Chromium kiosk) -- pas de bureau, juste une console texte
accessible en secours via `Ctrl+Alt+F2`.

Construite **sur cette machine de dev** (pas sur le PC de la malette), avec
accès réseau pour télécharger les paquets Debian. Le résultat (`.iso` hybride,
bootable en BIOS et UEFI) est ensuite flashé sur la clé.

## Pourquoi Debian (pas Alpine)

`GCS/serial-bridge.js` dépend du paquet npm `serialport`, qui embarque un
binding natif compilé contre `glibc`. Alpine utilise `musl` -- il faudrait
recompiler le module depuis les sources à chaque fois, fragile pour un
appareil qu'on veut fiable sur le terrain. Debian/Ubuntu évitent ce problème.

## Prérequis (sur cette machine)

```bash
sudo apt install live-build debian-archive-keyring dctrl-tools \
    syslinux syslinux-utils syslinux-common isolinux genisoimage mtools zsync parted
```

Le `live-build` disponible ici est un vieux fork Ubuntu (`3.0~a57-1ubuntuXX`)
qui a plusieurs bugs/incohérences avec une cible Debian bookworm moderne --
voir `auto/config` pour la liste des flags qui les contournent (mode debian
forcé, mirroirs explicites, `--build-with-chroot false`, etc., chacun
commenté avec le pourquoi).

**Correctifs supplémentaires nécessaires sur la machine de build** (pas
portables via `config/`, appliqués directement dans les fichiers du paquet
`live-build` -- à refaire sur toute nouvelle machine de build) :

```bash
# 1. Symlinks isolinux casses (le paquet live-build pointe vers d'anciens
#    chemins syslinux qui n'existent plus dans les versions modernes) :
sudo ln -sf /usr/lib/ISOLINUX/isolinux.bin /usr/share/live/build/bootloaders/isolinux/isolinux.bin
sudo ln -sf /usr/lib/syslinux/modules/bios/vesamenu.c32 /usr/share/live/build/bootloaders/isolinux/vesamenu.c32

# 2. Modules syslinux manquants (obligatoires depuis la modularisation de
#    syslinux, posterieure a l'ecriture de ce script en 2012) :
sudo cp /usr/lib/syslinux/modules/bios/ldlinux.c32 /usr/share/live/build/bootloaders/isolinux/
sudo cp /usr/lib/syslinux/modules/bios/libcom32.c32 /usr/share/live/build/bootloaders/isolinux/
sudo cp /usr/lib/syslinux/modules/bios/libutil.c32 /usr/share/live/build/bootloaders/isolinux/

# 3. Archive cpio vide pour le hack gfxboot-theme-ubuntu -- code non gate par
#    LB_MODE, plante meme en mode debian sans ce fichier :
: | cpio -o --format=newc | sudo tee /usr/share/live/build/bootloaders/isolinux/bootlogo > /dev/null

# 4. Wrapper de compatibilite "rsvg" -> "rsvg-convert" (binaire renomme
#    depuis, syntaxe differente -- voir le contenu du wrapper dans
#    config/includes.binary/isolinux/ si besoin de le recreer) :
sudo tee /usr/bin/rsvg > /dev/null <<'EOF'
#!/bin/sh
args=""; input=""; output=""; prev=""
for arg in "$@"; do
  case "$prev" in
    --format|--height|--width) args="$args $prev $arg"; prev=""; continue ;;
  esac
  case "$arg" in
    --format|--height|--width) prev="$arg" ;;
    *) if [ -z "$input" ]; then input="$arg"; else output="$arg"; fi ;;
  esac
done
exec rsvg-convert $args -o "$output" "$input"
EOF
sudo chmod +x /usr/bin/rsvg
```

Une copie de sauvegarde de `bootlogo`/`ldlinux.c32`/`libcom32.c32`/`libutil.c32`
est aussi gardée dans `config/includes.binary/isolinux/` (dans ce repo) --
pratique si tu dois reconstituer les fichiers ci-dessus sans être reconnecté
au réseau, mais **ne remplace pas** ces commandes : `lb_binary_includes`
tourne *après* `lb_binary_syslinux` dans ce fork, trop tard pour lui.

## 1. Copier le code GCS dans l'image

`node_modules/` doit être présent AVANT de construire l'image (pas d'accès
réseau garanti une fois déployée) :

```bash
cd /home/jules/droneMk2/GCS
npm install
```

Puis copier le dossier GCS (sans `.git`) dans `includes.chroot` :

```bash
cd /home/jules/droneMk2/GCS/live-image
mkdir -p config/includes.chroot/opt/gcs
rsync -a --exclude=live-image --exclude=.git ../ config/includes.chroot/opt/gcs/
```

**Important** : `server.js`/`debug-injector.js` lisent `TELEMETRY.csv`,
`INDEX.csv`, `ALARMS.csv`, `MISSION_ALARMS.csv` dans `data/`, un niveau
AU-DESSUS de `GCS/` (`path.join(__dirname, "..", "data", "...")`, cf.
structure du repo) -- ces fichiers ne font pas partie du dossier `GCS/` et
le rsync ci-dessus ne les copie donc pas. Sans eux, `node server.js`
plante immédiatement au démarrage (`ENOENT: no such file or directory,
open '/opt/data/TELEMETRY.csv'`, bug constate en test reel). Il faut les
copier a la main, a cote de `opt/gcs/` (pas dedans) :

```bash
mkdir -p config/includes.chroot/opt/data
cp ../../data/TELEMETRY.csv ../../data/INDEX.csv ../../data/ALARMS.csv ../../data/MISSION_ALARMS.csv \
  config/includes.chroot/opt/data/
```

À refaire à chaque fois que le code GCS (ou l'un de ces 4 CSV) change et que
tu veux reconstruire l'image.

## 2. Configurer et builder

```bash
cd /home/jules/droneMk2/GCS/live-image
lb config      # relit auto/config -- refaire après toute modif de auto/config
sudo lb build  # télécharge les paquets, construit le chroot -- prend du temps
```

Résultat : `live-image-amd64.hybrid.iso` dans le dossier courant.

Pour recommencer proprement après un échec :
```bash
sudo lb clean
```

## 3. Flasher sur la clé USB

**⚠️ `dd` écrase la cible sans confirmation ni corbeille. Vérifie le
périphérique avec `lsblk` AVANT de lancer la commande -- une erreur de lettre
peut effacer le mauvais disque (ton disque système par exemple).**

```bash
lsblk                    # repère la clé (ex: /dev/sdb) -- PAS une partition (/dev/sdb1)
sudo dd if=live-image-amd64.hybrid.iso of=/dev/sdX bs=4M status=progress conv=fsync
sync
```

Remplace `/dev/sdX` par le vrai chemin. Si un doute persiste, débranche tout
sauf la clé et relance `lsblk` pour être sûr.

## 4. Créer la partition de persistance (pour les mises à jour sans reflash)

Sans ça, `/opt/gcs` vit dans le squashfs (lecture seule) -- toute modif faite
à chaud disparaît au reboot suivant. La clé de 32 Go a largement la place
après l'ISO (qui fait ~1-2 Go) : on crée une partition ext4 dans l'espace
libre, avec le label exact `persistence` (obligatoire, live-boot le cherche
par ce nom).

```bash
lsblk /dev/sdX                     # repère la fin de la/des partition(s) de l'ISO
sudo parted /dev/sdX print free    # affiche l'espace libre restant

# Cree une nouvelle partition dans l'espace libre (ajuste le numero si besoin,
# generalement 2 ou 3 selon le nombre de partitions de l'ISO hybride) :
sudo parted /dev/sdX mkpart primary ext4 <debut_espace_libre> 100%
sudo mkfs.ext4 -L persistence /dev/sdX<N>

sudo mount /dev/sdX<N> /mnt
echo "/opt/gcs union" | sudo tee /mnt/persistence.conf
sudo umount /mnt
```

`persistence.conf` liste les chemins montés en overlay écriture par-dessus le
système RAM -- un par ligne, `union` = copy-on-write (garde les fichiers
d'origine du squashfs comme base, les modifs vont dans la partition). Ajoute
d'autres lignes si besoin (ex: `/etc/wpa_supplicant union` pour garder la
config WiFi entre reboots).

## Mettre à jour le code GCS sans tout reflasher

Une fois la persistance en place, `/opt/gcs` est réellement inscriptible et
survit aux reboots. Pour une mise à jour :

```bash
# Depuis un poste sur le meme reseau WiFi que la malette (cf. section WiFi) :
rsync -a --exclude=node_modules GCS/ gcs@<ip-de-la-malette>:/opt/gcs/
ssh gcs@<ip-de-la-malette> sudo systemctl restart gcs-server.service
```

Puis recharge la page dans le kiosk (ou `sudo systemctl restart
gcs-kiosk.service` si le changement touche le HTML/JS/CSS servi par
Chromium). Un reflash complet de l'ISO ne redevient nécessaire que pour un
changement au niveau OS (nouveaux paquets, service systemd modifié, thème
Plymouth...).

**Accès SSH** : `ssh.service` est activé, mais l'utilisateur `gcs` n'a ni mot
de passe ni clé autorisée par défaut (cf. section suivante) -- configure
l'un des deux avant de builder si tu veux t'y connecter.

## Ce que fait l'image au boot

1. `live-boot` + paramètre kernel `toram` (déjà dans `auto/config`) : copie
   l'intégralité du système en RAM. La clé peut ensuite être retirée sans que
   le système plante (mais autant la laisser en place en usage normal).
2. Plymouth affiche le splash (thème `gcs`, actuellement un placeholder texte
   -- voir plus bas pour mettre ton propre logo).
3. `gcs-server.service` démarre `node /opt/gcs/server.js`.
4. `gcs-kiosk.service` lance Chromium plein écran sur `tty1`. `matchbox-
   window-manager` tourne en fond (sans barre de titre) -- pas un bureau,
   juste le strict nécessaire pour que la demande plein-écran de Chromium
   (`--kiosk`) soit honorée (sans WM du tout, Chromium retombe sur une
   fenêtre de taille par défaut, bug constaté en test réel) -- cf.
   `config/includes.chroot/opt/gcs-kiosk/xinitrc`.
5. `Ctrl+Alt+F2` (etc.) donne une vraie console texte (`tty2`+) à tout moment
   -- `getty` y tourne normalement, rien de désactivé. `Ctrl+Alt+F1` revient
   au kiosk.

Aucune écriture disque en fonctionnement normal : tout vit en RAM, la clé
n'est sollicitée qu'au boot (usure minimale, comportement robuste si elle
lâche en usage terrain).

## Remplacer le splash placeholder par un vrai logo

Le thème actuel (`config/includes.chroot/usr/share/plymouth/themes/gcs/`) est
un theme "script" texte, non testé sur matériel réel (pas d'environnement
Plymouth disponible pour vérifier la syntaxe pendant son écriture -- teste
au premier boot, corrige `gcs.script` si l'affichage est cassé).

Pour un vrai logo, le plus simple est de repartir d'un theme "two-step"
existant (fourni par le paquet `plymouth-themes`, ex: `/usr/share/plymouth/themes/spinner/`
une fois `sudo lb build` fait au moins une fois) : copie-le dans
`config/includes.chroot/usr/share/plymouth/themes/gcs/`, remplace ses images
par ton logo (mêmes noms de fichiers), pas besoin de toucher aux hooks.

## Mots de passe / accès SSH

`useradd -m gcs` (cf. `config/hooks/0100-gcs-user.chroot`) crée
l'utilisateur sans mot de passe défini -- ni la console (`tty2`+) ni SSH
n'accepteront de connexion tant que l'un des deux n'est pas configuré.

**Option recommandée : clé SSH** (pas de mot de passe qui traîne sur une
malette) -- avant de builder :
```bash
mkdir -p config/includes.chroot/home/gcs/.ssh
cp ~/.ssh/id_ed25519.pub config/includes.chroot/home/gcs/.ssh/authorized_keys
```
Et dans `config/hooks/0100-gcs-user.chroot`, après le `useradd`,
ajoute :
```sh
chown -R gcs:gcs /home/gcs/.ssh
chmod 700 /home/gcs/.ssh
chmod 600 /home/gcs/.ssh/authorized_keys
```

**Option mot de passe** (plus simple pour du debug ponctuel) -- dans le même
hook :
```sh
echo 'gcs:CHANGE_MOI' | chpasswd
```

## Adaptateur wfb-ng (RTL8812AU)

`config/hooks/0300-rtl8812au-dkms.chroot` compile et installe le
driver externe (recommandé par le projet wfb-ng lui-même :
`github.com/svpcom/rtl8812au`, branche `v5.2.20`) via DKMS pendant le build
-- le driver in-tree du noyau Debian ne supporte pas le mode monitor +
injection nécessaire à wfb-ng.

**Non testé** : je n'ai pas d'environnement `live-build` ni de carte
RTL8812AU disponibles ici pour vérifier que ça compile réellement contre le
noyau qui sera installé (bookworm). Si le build DKMS échoue, regarde
`/var/lib/dkms/realtek-rtl88xxau/5.2.20.2~20190429/build/make.log` dans le
chroot (ou les logs de `sudo lb build`) -- les drivers externes de ce genre
ont parfois besoin d'un petit patch pour suivre une API noyau qui a bougé.

Le hook désactive aussi IPv6 globalement (`sysctl`), recommandation du projet
driver lui-même -- pas juste hérité en silence, mentionné ici explicitement
si jamais ça surprend ailleurs sur la machine.

wfb-ng lui-même (le logiciel, pas juste le driver) n'est **pas** installé
dans cette image -- la télémétrie tourne encore en émulation LAN pour
l'instant (cf. mémoire projet). À ajouter quand le vrai lien radio sera mis
en service.

## Ne PAS activer par défaut

`GCS/debug-injector.js` (l'outil d'injection de pannes) n'a **aucun service
systemd** dans cette image -- il ne doit tourner qu'à la demande, en dev,
jamais sur l'appareil déployé (cf. discussion sécurité : n'importe qui sur le
LAN pourrait injecter de fausses pannes si exposé). Pour un test ponctuel
depuis la console : `node /opt/gcs/debug-injector.js`.
