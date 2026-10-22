# Test HIL « chaîne d'information »

> Statut : **document de conception**, écrit avant le code. Les décisions
> prises sont marquées comme telles ; les questions ouvertes aussi. À
> corriger avant implémentation.
>
> À ne pas confondre avec `INTEGRATION_TEST.md` / `sitl/SITL.md`, qui
> décrivent la validation contre un environnement **simulé**. Ici tout le
> matériel est réel et il n'y a aucune simulation — sauf les moteurs, qui
> sont neutralisés.

---

## 1. Objectif

Faire tourner **toute la chaîne d'information** du drone sur le matériel
réel, sans sortie moteur, pour pouvoir se promener avec l'appareil à la main
et voir sur l'écran du GCS le pion du drone se déplacer, la vidéo arriver, et
les failsafes réagir.

C'est le test le plus central du projet : c'est le seul qui exerce
simultanément les capteurs, l'EKF, les deux liens radio, la vidéo, le
dashboard et les procédures de panne. Tout le reste ne valide qu'une tranche.

### 1.1 Ce qu'il valide

- **Chaîne capteur → EKF → télémétrie → écran** : une position réelle qui
  bouge doit se voir bouger sur le dashboard.
- **Les deux liens radio en exploitation**, pas sur un banc : PDL pour la
  télémétrie et la vidéo, SDL pour les commandes critiques.
- **La vidéo de bout en bout** : caméra, encodage, transport, affichage dans
  la page du GCS.
- **Les failsafes** déclenchés pour de vrai : perte de lien, perte de FC,
  capteur dégradé, EKF divergent.
- **Le dashboard** face à de vraies données, y compris dégradées.

### 1.2 Ce qu'il ne valide pas

- Rien qui dépende de la **dynamique de vol** : pas de boucle de guidage
  fermée, pas de réponse du contrôleur d'attitude, pas de comportement en
  vent. Cela reste du domaine du SITL.
- L'**autonomie réelle** : sans moteurs, la consommation n'a aucun rapport.
- La **portée radio en vol** : un drone tenu à la main près du sol n'a pas le
  bilan de liaison d'un drone à trente mètres.

---

## 2. Sûreté : pourquoi aucune hélice ne peut tourner

**Décision : trois couches indépendantes.** Aucune n'est suffisante seule, et
elles ne tombent pas pour les mêmes raisons.

| couche | nature | ce qu'elle couvre |
|---|---|---|
| hélices démontées | physique | tout, y compris une erreur de notre part |
| moteurs désactivés dans le FC | paramètre FC | un ordre d'armement qui passerait |
| commandes neutralisées à la compilation | logiciel | l'émission de l'ordre lui-même |

### 2.1 La couche logicielle, et son défaut

`MavlinkInterface` cesse d'émettre certaines commandes — armement en premier
lieu — sous un drapeau de compilation.

**Le défaut d'un drapeau de compilation est qu'il est invisible.** Rien, en
regardant tourner le système, ne distingue un binaire neutralisé d'un binaire
de vol. On peut croire tenir l'un et tenir l'autre, dans les deux sens : se
croire protégé et ne pas l'être, ou se demander pourquoi l'armement échoue
alors qu'on voulait voler.

**Donc le mode doit être observable à l'exécution**, et pas seulement à la
compilation :

- annoncé en clair au démarrage, dans le journal du composant ;
- porté dans la télémétrie, pour que le **dashboard l'affiche en permanence**
  — un bandeau, pas une ligne discrète ;
- et le refus d'une commande neutralisée doit être **remonté** au sol, pas
  avalé : l'opérateur doit voir « refusée car binaire HIL » et non un silence.

Sans ça, la couche qui devait protéger devient une source de confusion.

### 2.2 Ce que le binaire HIL diffère du binaire de vol

Une seule chose : un chemin de code gardé dans `MavlinkInterface`. Tout le
reste — les cinq composants, l'IPC, les watchdogs, les pilotes, les liens —
est identique. C'est voulu : un banc qui ne ressemble pas au logiciel de vol
ne valide pas le logiciel de vol.

> **Question ouverte** : quelles commandes exactement ? `CMD_ARM` est
> évident. Faut-il aussi neutraliser le changement de mode, le décollage, et
> ce que `MissionControl` peut décider seul en failsafe ? Un failsafe qui
> tenterait un atterrissage automatique enverrait des ordres moteur.

---

## 3. Architecture

**Décision : un banc, mais qui reprend tout du vrai logiciel.** Pas un
harnais qui réimplémente des morceaux : les cinq composants sous
`GlobalWatchdog`, l'IPC réel, les vrais pilotes.

```
  DRONE                                      STATION SOL
  ┌───────────────────────────┐              ┌──────────────────────┐
  │ GlobalWatchdog            │              │ server.js            │
  │  ├ SensorFusion  (EKF)    │   PDL        │  ├ dashboard (HTTP)  │
  │  ├ Navigation             │◄── wfb-ng ──►│  ├ WebSocket         │
  │  ├ MissionControl         │   télémétrie │  └ pont série ──┐    │
  │  ├ MavlinkInterface ──UART┼──► FC        │   + vidéo       │    │
  │  └ SysMonitoring          │              │                 ▼    │
  │     ├ TelMain  (PDL)      │   SDL        │              ESP32   │
  │     ├ TelSec   (SDL)      │◄── CC1101 ──►│           (pont SDL  │
  │     └ Video               │   commandes  │        + panneau)    │
  │                           │              │                      │
  │ caméra ──► RTP ───────────┼──────────────┼──► <video> dans la   │
  └───────────────────────────┘              │     page + HUD       │
                                             └──────────────────────┘
```

### 3.1 Ce qui manque aujourd'hui

| maillon | état |
|---|---|
| liens radio dans `main.cpp` | **absent** — `UdpTelemetryDriver` est encore instancié |
| source vidéo | **absent** — `UdpVideoDriver` attend des trames, rien ne capture |
| RTP et affichage | **absent** |
| HUD | **absent** |
| drapeau de sûreté | **absent** |
| injection de pannes | **absent** |
| convention série documentée | partielle — existe dans les commentaires, pas de version négociée |

Les composants, l'IPC, les watchdogs, l'EKF, les deux pilotes radio et le
dashboard existent et sont testés.

---

## 4. Convention du port série ESP32 ↔ station sol

La carte du boîtier porte le panneau d'alarme **et** la radio SDL. Un seul
port série sert donc deux usages, et d'autres viendront. La convention est
donc à fixer maintenant, pas à chaque ajout.

### 4.1 Règles

1. **Une ligne par message**, terminée par `\n`. ASCII imprimable uniquement.
2. **`VERBE:champ:champ:...`** — verbe en majuscules, champs séparés par
   `:`. Le verbe seul détermine le nombre et le sens des champs.
3. **Les données binaires passent en hexadécimal.** Le volume double, ce qui
   est sans conséquence sur ce lien (~5 trames de 50 octets par seconde à
   115200 bauds) et garde une trace lisible au moniteur série. Un cadrage
   binaire gagnerait un débit dont on n'a aucun besoin.
4. **Une ligne inconnue est ignorée, jamais fatale.** C'est ce qui permet
   d'ajouter un verbe d'un côté avant l'autre.
5. **L'ESP32 ne garde aucun état métier.** Il exécute et rapporte. Toute
   décision vit côté Node.js. Un firmware « bête » se remplace sans
   réapprendre la logique.
6. **Aucun échec n'est silencieux.** Tout refus produit un `ERR:`.

### 4.2 Verbes actuels

**Station sol → ESP32**

| verbe | champs | rôle |
|---|---|---|
| `LED` | liste de libellés | état **complet** des voyants, pas un delta |
| `TX` | hex | émettre sur le SDL, en respectant le CCA |
| `TXF` | hex | émettre en **forçant** — CCA contourné, arrêt d'urgence |

**ESP32 → station sol**

| verbe | champs | rôle |
|---|---|---|
| `BTN` | libellé | un appui, front descendant, jamais le relâchement |
| `RX` | hex, rssi, lqi | trame SDL reçue et sa qualité |
| `LQ` | rssi, lqi, crc, len, ovf, cca, forced | santé du lien, 1 Hz |
| `TXERR` | raison | émission refusée |
| `RADIO` | `ok` \| `absente` | au démarrage |

### 4.3 Ce qui manque à la convention

**Pas de version négociée.** Si le firmware et `server.js` divergent, ça
échoue en silence — exactement ce qui s'est produit avec `RADIO:ok`, émis
une seule fois au démarrage et donc manqué par un serveur connecté plus tard.

> **Décision proposée** : ajouter un échange d'identification.
>
> - station sol → ESP32 : `HELLO:<version du protocole>`
> - ESP32 → station sol : `HELLO:<version>:<capacités>`, par exemple
>   `HELLO:1:panel,sdl`
>
> Le serveur l'envoie à chaque ouverture du port et le répète s'il n'obtient
> rien. Cela règle trois choses d'un coup : la détection de version, la
> découverte des capacités réellement présentes, et le fait qu'une annonce
> unique au démarrage se manque. Les capacités remplacent `RADIO:ok`.

**Règle de nommage** à tenir pour la suite : un verbe par **fonction**, pas
par champ. `LQ` porte sept champs plutôt que sept verbes, parce qu'ils
décrivent un même instant et doivent être lus ensemble.

---

## 5. Chaîne vidéo

**Décision : la vraie caméra, intégrée dans la page du GCS.**

La caméra est présente (`/dev/video0`, `rpicam-vid` disponible sur le Pi).
`UdpVideoDriver` attend déjà des trames côté `SysMonitoring`. Il manque la
capture, le transport et l'affichage.

> **Question ouverte — et c'est la plus structurante du document.** Deux
> architectures s'opposent :
>
> **(a) La vidéo traverse `SysMonitoring`.** La capture publie en UDP local,
> `UdpVideoDriver` la lit, `SysMonitoring` la relaie sur le PDL. C'est ce que
> le code existant suppose. Avantage : un seul chemin de sortie, la vidéo
> hérite de la supervision des composants. Inconvénient : on fait traverser
> plusieurs mégabits par seconde à un composant temps réel dont ce n'est pas
> le métier, et une trame vidéo perdue n'a pas la même valeur qu'un paquet de
> télémétrie perdu.
>
> **(b) La vidéo contourne le logiciel de vol.** `rpicam-vid` pousse
> directement en RTP vers l'entrée de `wfb_tx`, en parallèle de la
> télémétrie. Avantage : zéro charge sur les composants, et la vidéo tombe
> sans entraîner autre chose. Inconvénient : `SysMonitoring` ne sait plus rien
> de l'état de la vidéo, donc le dashboard ne peut pas l'indiquer — sauf à
> remonter une santé par un autre chemin.
>
> Mon avis : **(b)**, avec un port radio wfb-ng distinct de la télémétrie.
> Faire passer de la vidéo dans un chemin temps réel pour le seul bénéfice de
> l'observabilité est un mauvais échange, et l'observabilité se récupère
> autrement. Mais c'est une décision d'architecture, donc à trancher.

### 5.1 Le contournement ne gêne pas le HUD

Question posée, et la réponse lève l'objection : **le HUD se fait côté GCS**,
en superposition dans le navigateur. Il n'affiche pas des informations issues
de la vidéo, il affiche de la **télémétrie** — qui arrive par `SysMonitoring`
quoi qu'il arrive. Les deux flux ne se rencontrent que dans le navigateur.

Le contournement est même un avantage pour le HUD :

- il reste lisible quand la vidéo décroche, et continue d'afficher la
  télémétrie qui, elle, arrive encore ;
- il ne coûte aucun calcul au Pi ;
- il se modifie sans toucher au drone ni redéployer un binaire de vol.

### 5.2 HUD

Superposition d'informations sur l'image dans la page du GCS : attitude,
altitude, vitesse, mode, état des liens, et **le bandeau du mode HIL**.

Côté navigateur, en SVG par-dessus l'élément vidéo — pas incrusté dans le
flux, pour les raisons données juste au-dessus.

---

### 5.3 Le vrai piège : les deux flux n'ont pas la même latence

C'est ce qu'il faut garder en tête, et ce n'est pas un détail d'affichage.

La télémétrie est faite de petits paquets et arrive vite. La vidéo subit
l'encodage, la mise en tampon réseau et le décodage du navigateur : plusieurs
centaines de millisecondes sont courantes. **Le HUD affichera donc une
attitude en AVANCE sur l'image.**

Tant que l'écart reste faible, c'est imperceptible. Au-delà de ~200 ms, un
HUD qui annonce un virage que l'image n'a pas encore montré est trompeur —
et pour quelqu'un qui pilote en regardant l'écran, trompeur veut dire
dangereux.

La correction est simple mais demande une mesure : retarder les valeurs du
HUD de la latence vidéo, pour qu'elles décrivent l'instant que l'image
montre. Encore faut-il connaître cette latence.

> **À faire, et pas en dernier** : mesurer la latence vidéo bout en bout
> (verre à verre). Le moyen le plus simple reste de filmer un chronomètre
> affiché sur l'écran du GCS et de comparer les deux valeurs sur une même
> image. Sans ce chiffre, on ne sait pas si le HUD doit être retardé, ni de
> combien — donc on ne sait pas s'il est honnête.

## 6. Injection de pannes

**Décision : un outil séparé, sur le réseau local du drone, et OPTIONNEL.**

Le drone n'a pas forcément de réseau quand on se promène dehors. L'outil doit
donc être **absent sans conséquence** : le système se comporte normalement
s'il ne se connecte jamais. C'est la même règle que pour le relais de
statistiques — un canal de diagnostic ne doit jamais prendre en otage le
chemin de données.

Ce qu'il doit permettre de provoquer :

- couper un pilote de capteur (GPS, lidar, magnétomètre) ;
- couper un lien radio, chacun séparément ;
- couper le FC ;
- dégrader plutôt que couper : bruit, latence, perte partielle ;
- tuer un composant, pour voir le `GlobalWatchdog` le reprendre.

**Décision : deux étages, et on s'arrête au premier tant qu'il suffit.**
L'approche intrusive a été jugée trop coûteuse, à raison.

### 6.1 Étage 1 — aucune ligne de code dans le logiciel de vol

Tout se fait depuis l'extérieur, sur un système qui ne sait même pas qu'on
l'éprouve. Rien à ajouter, rien à désactiver ensuite, aucune surface
d'attaque créée.

| panne | moyen |
|---|---|
| mort d'un composant | tuer le process — éprouve `GlobalWatchdog` |
| perte d'un capteur | détacher le périphérique USB, ou fermer l'UART |
| perte d'un lien radio | arrêter `wfb-tx` / `wfb-rx`, ou le pont SDL |
| perte du FC | couper son UART |
| lien dégradé | bloquer une fraction des paquets avec `nftables` |

Cet étage couvre déjà **tous les failsafes que le drone doit survivre** :
perte de lien, perte de FC, perte de capteur, mort de composant. C'est
l'essentiel de ce qu'on veut éprouver.

### 6.2 Étage 2 — dégradation fine, seulement si l'étage 1 ne suffit pas

Ce que l'extérieur ne sait pas faire : ajouter du bruit sur une mesure,
décaler une latence, perdre un paquet sur dix de façon contrôlée. Cela
demande un point d'injection **dans** les pilotes.

S'il faut en venir là, il vit **sous le même drapeau de compilation que la
sûreté moteur**, jamais dans le binaire de vol. Un point d'injection de
pannes dans un binaire qui vole est un risque qui ne se justifie pas, et le
drapeau garantit qu'il ne peut pas s'y trouver par accident.

> **À ne pas faire avant d'en avoir besoin.** L'étage 1 ne coûte rien et
> couvre les cas qui comptent. Construire l'étage 2 « au cas où » ajouterait
> de la complexité dans les pilotes pour des pannes qu'on n'a pas encore
> cherché à reproduire.

---

## 7. Ordre de travail proposé

Du plus utile au moins urgent, et chaque étape est testable seule.

1. **Brancher les vrais liens radio dans `main.cpp`.** Une ligne plus la
   décision des ports. Sans ça rien d'autre ne se voit au sol.
2. **Le drapeau de sûreté** et son observabilité (journal, télémétrie,
   bandeau au dashboard). Avant tout ce qui touche au FC.
3. **`HELLO` dans la convention série**, pour que firmware et serveur
   cessent de pouvoir diverger en silence.
4. **La chaîne vidéo**, une fois l'architecture tranchée.
5. **Le HUD**, qui dépend de la vidéo.
6. **L'injection de pannes**, étage 1 uniquement : rien à écrire dans le
   logiciel de vol, donc c'est surtout un mode opératoire à consigner.

Et une étape qui ne peut pas attendre la fin : **mesurer la latence vidéo**
dès que le flux arrive, parce que c'est elle qui dit si le HUD doit être
retardé — donc s'il est honnête (cf. 5.3).

Et la première séance utile, dès l'étape 1 faite : marcher dans le jardin et
regarder le pion bouger. C'est le critère de réussite qui compte.
