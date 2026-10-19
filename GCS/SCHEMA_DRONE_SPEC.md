# Cahier des charges — bloc « Schéma drone » du dashboard GCS

Document destiné à un outil de conception front-end (HTML/CSS/JS) extérieur
au dépôt. Il décrit le contexte, les contraintes d'intégration et le contrat
de données du bloc à produire. Le résultat sera intégré tel quel dans le
dashboard existant, puis câblé aux données réelles.

---

## 1. Contexte produit

Le dashboard est l'écran unique d'une station sol (« malette ») pilotant un
drone autonome. Il tourne en kiosk Chromium sur un PC relié au drone par
radio. L'opérateur ne clique presque pas : il surveille. Toute information
affichée doit donc être lisible d'un coup d'œil et sans ambiguïté.

Registre visuel : cockpit avionique. Fond noir, accents cyan, pas de
décoration, pas de dégradé gratuit, pas d'animation hors signalement d'un
défaut. La référence est un ECAM Airbus, pas un dashboard web grand public.

Le dashboard existant est en HTML/CSS/JS natif, sans framework ni étape de
build. Il se compose de trois colonnes :

| Colonne | Largeur | Contenu actuel |
|---|---|---|
| Gauche | 390 px | GNSS, Lidar, **Batterie**, Actions |
| Centre | reste | Bandeau composants, viewport 3D / HUD caméra, Radio |
| Droite | 446 px | **CPU Temp**, Journal d'événements |

Les deux panneaux en gras sont supprimés et remplacés par le bloc décrit
ici.

---

## 2. Ce que le bloc doit contenir

### 2.1 Schéma du drone, vue de côté

Une silhouette du drone vue de profil, avec le **pourtour simplifié** de
chaque composant embarqué, positionné à son emplacement physique réel. Pas
de rendu réaliste : des formes géométriques sobres, reconnaissables par leur
position et leur étiquette.

Composants à représenter :

| Élément | Nombre | Remarque de placement |
|---|---|---|
| Moteurs | 2 | Vue de côté : l'avant et l'arrière. Chacun représente en réalité 2 moteurs du quadrirotor. |
| ESC | 1 ou 2 | Selon l'implantation réelle. |
| Batterie | 1 | **Sous le ventre, sur toute la longueur** du châssis. |
| CPU (Raspberry Pi 5) | 1 | |
| FC (flight controller) | 1 | |
| GPS | 1 | |
| MAG (magnétomètre) | 1 | |
| Antennes | 2 | Lien principal et lien de secours. |

La géométrie exacte (proportions du châssis, position de chaque boîtier)
sera fournie séparément à partir des plans SolidWorks du drone. Le présent
document fixe la sémantique, pas les cotes.

**Format attendu : SVG inline.** Raisons : mise à l'échelle sans perte (cf.
§3.1), chaque forme ciblable individuellement par un identifiant, et aucune
ressource externe à charger — le GCS fonctionne sans Internet.

### 2.2 Couleur = état

Chaque forme change de couleur selon l'état du composant qu'elle
représente. Quatre niveaux, repris de la palette existante :

| État | Couleur | Variable CSS |
|---|---|---|
| Nominal | vert | `--ok` (`#35d67c`) |
| Attention | ambre | `--warn` (`#ffb443`) |
| Défaut | rouge | `--crit` (`#ff5c5c`) |
| Sans donnée | gris atténué | `--fg-faint` |

Le quatrième état n'est pas optionnel. Plusieurs composants n'ont
aujourd'hui **aucune source de données** (cf. §4), et certains n'en auront
qu'après l'implémentation de leur capteur. Un composant sans donnée doit
être visuellement distinct d'un composant sain : il doit se lire « inconnu »,
jamais « bon ». C'est le point le plus important de ce cahier des charges.

Prévoir un cinquième traitement, optionnel : un **clignotement lent** sur
l'état `--crit`, pour attirer l'œil sur un défaut nouveau. Pas de son dans
ce bloc, l'alarme sonore est gérée ailleurs.

### 2.3 Barres de charge dans la batterie

La batterie étant dessinée sur toute la longueur du châssis, sa forme sert
de jauge : un remplissage horizontal proportionnel au pourcentage restant,
dans le contour de la batterie.

Le niveau de remplissage change de couleur aux seuils de la batterie 6S du
drone :

| Plage | Couleur |
|---|---|
| > 30 % | `--ok` |
| 10–30 % | `--warn` |
| < 10 % | `--crit` |

Un découpage en segments visuels (par cellule ou en graduations) est
possible et bienvenu esthétiquement, **mais il ne doit pas suggérer une
mesure par cellule** : le drone ne transmet pas les tensions individuelles,
seulement la tension du pack. Des graduations décoratives régulières sont
acceptables, six segments remplis indépendamment ne le sont pas.

### 2.4 Encadré autonomie

L'autonomie est la donnée la plus importante de l'écran. Elle garde un
encadré propre, chiffré, lisible de loin — pas seulement la jauge du
schéma. Trois valeurs :

| Valeur | Unité | Priorité visuelle |
|---|---|---|
| Charge restante | % | La plus grosse |
| Autonomie estimée | minutes | Secondaire |
| Distance estimée | mètres ou km | Secondaire |

Les deux estimations doivent pouvoir afficher `--` : elles dépendent d'un
calcul qui n'est pas encore tranché (cf. §5). Prévoir également une mention
discrète du caractère estimé de ces deux valeurs — une autonomie calculée
n'a pas le même statut qu'une tension mesurée.

Tensions et courant mesurés (V, A) restent utiles et peuvent figurer en
petit dans cet encadré.

---

## 3. Contraintes d'intégration

### 3.1 Mise à l'échelle

La scène est dessinée pour **1920 × 1080 fixe**. Le navigateur applique un
`transform: scale()` global pour la faire tenir dans la fenêtre. Un bloc
conçu à ces dimensions se comporte donc correctement partout.

Conséquence : les dimensions en pixels sont acceptables et même attendues,
mais le SVG doit porter un `viewBox` pour rester net à l'agrandissement.

Enveloppe disponible : **390 px de large**, hauteur souple de l'ordre de 300
à 450 px. La colonne est un conteneur flex en `overflow: hidden` : le bloc
doit supporter d'être un peu comprimé sans que du contenu sorte.

### 3.2 Pas de valeur dynamique dans le CSS

Règle en vigueur dans le dépôt : la feuille de style ne contient aucune
valeur qui varie. Tout ce qui dépend des données est posé par le moteur de
rendu. Concrètement, le livrable fournit :

- le markup, avec un attribut `data-bind="<nom>"` sur **chaque** élément
  destiné à être mis à jour ;
- les styles, dans des classes, utilisant les variables CSS existantes
  (§3.4) ;
- aucun style inline sur les éléments dynamiques.

Un élément sans `data-bind` est considéré comme décoratif et ne sera jamais
mis à jour.

### 3.3 Cadence de rendu et allocations

Le dashboard rend à 20 Hz. Le moteur de rendu ne crée **aucun nœud DOM** en
boucle : tous les éléments répétés sont alloués une fois au démarrage, puis
montrés, masqués et mis à jour. Un livrable qui supposerait une
reconstruction du markup à chaque trame ne serait pas intégrable.

Si le bloc comporte une liste de longueur variable, elle doit être déclarée
avec un `data-list="<nom>"` sur son conteneur et un plafond d'éléments
connu. Pour ce bloc précis, aucune liste variable n'est a priori nécessaire :
tous les composants du schéma sont en nombre fixe.

### 3.4 Jetons de style disponibles

À réutiliser, sans en introduire de nouveaux :

```
Fonds       --bg #05080a    --panel rgba(13,20,24,.85)   --panel-alt
Traits      --line rgba(255,255,255,.09)                 --line-soft
Texte       --fg #e6f1f4    --fg-dim    --fg-dimmer   --fg-faint
Accent      --accent #4fd8ea                            --accent-15
États       --ok #35d67c    --warn #ffb443   --crit #ff5c5c
Typo UI     --f-ui   'Rajdhani', system-ui, sans-serif
Typo chiffres --f-mono 'JetBrains Mono', ui-monospace, monospace
```

Conventions typographiques déjà en place, à suivre : titres de panneau en
majuscules espacées (`letter-spacing` ~1.8 px, graisse 700, cyan) ;
étiquettes de champ en majuscules petites et atténuées ; valeurs numériques
toujours en police monospace, pour qu'un chiffre qui change ne fasse pas
sauter la mise en page.

Structure de panneau existante, à reprendre pour l'homogénéité :

```html
<section class="panel">
  <header class="panel__head">
    <div class="t-title">Titre</div>
    <div class="t-badge">badge d'état</div>
  </header>
  <!-- contenu -->
</section>
```

### 3.5 Dépendances

Aucune. Pas de framework, pas de CDN, pas de police web chargée en plus, pas
de bibliothèque de graphiques. Le GCS fonctionne hors réseau. Le SVG doit
être inline dans le markup, pas un fichier externe.

---

## 4. Contrat de données — ce qui existe réellement

Section déterminante pour le dessin. Le drone n'envoie pas « la santé du
GPS » : il envoie des paquets de télémétrie et des codes de diagnostic, à
partir desquels un état est déduit. La granularité disponible n'est pas la
même pour tous les composants.

| Élément du schéma | Source disponible aujourd'hui | État exploitable |
|---|---|---|
| **Batterie** | Tension, courant, pourcentage restant. Codes `batt_low` (critique) et `batt_critical` (urgence). | **3 niveaux.** Complet. |
| **GPS** | Type de fix, nombre de satellites, HDOP, détail par satellite. Codes `GPS_lock` / `GPS_lost`. | **3 niveaux.** Complet, dégradé inclus (fix faible, HDOP médiocre). |
| **CPU** | Température seule. | **3 niveaux** par seuils de température. Ni charge CPU ni RAM. |
| **FC** | **Santé du composant logiciel `MavlinkInterface`** (NOMINAL / DEGRADED / SICK / DEAD), transmise par le paquet `HEALTH`. Codes `fc_link_lost`, `fc_data_unreadable`, `command_rejected` en complément. | **4 niveaux.** Nuance à assumer : c'est la santé du lien et du composant qui parle au FC, pas celle du FC lui-même — mais c'est la seule fenêtre dont le système dispose sur le FC. |
| **MAG** | Code `Mag_lost` uniquement. | **2 niveaux** : sain ou perdu. Aucun dégradé possible. |
| **Moteurs** | **Régime par moteur** (4 valeurs), déjà décodé depuis `ESC_TELEMETRY_1_TO_4` à 2 Hz et stocké en mémoire partagée. Codes `rpm_too_low`, `rpm_too_high`, `Eng_failure` en complément, eux globaux. | **Dégradé continu**, par moteur. Voir ci-dessous pour la grandeur à représenter. |
| **ESC** | **Courant et température par ESC** (4 valeurs chacun), même source. | **3 niveaux**, par ESC, sur la température. |
| **Antennes** | Codes de perte du lien principal et du lien de secours. Qualité du lien principal à venir, mesurée localement par le récepteur de la station sol (pas un paquet de télémétrie). | **2 niveaux** aujourd'hui ; **dégradé** à terme sur l'antenne du lien principal seulement. |

**État du transport** : les régimes sont désormais transmis — paquet
`ESC_RPM`, 4 régimes à 2 Hz. Le courant et la température par ESC restent à
bord faute de besoin établi ; ils sont disponibles au même endroit et
s'ajouteront par un champ supplémentaire. Le dessin peut donc prévoir une
couleur thermique par ESC, en acceptant qu'elle reste grise d'ici là.

### 4.1 Les moteurs : quelle grandeur colorer

Le régime absolu ne dit rien de la répartition de charge : il monte et
descend avec les gaz, les quatre moteurs ensemble. Un dégradé sur le régime
absolu afficherait donc surtout la position de la manette.

Ce qui porte l'information cherchée est l'**écart au régime moyen des quatre
moteurs**. Un drone en vol stationnaire équilibré a quatre régimes voisins ;
un centre de gravité trop en avant, une hélice abîmée ou un moteur qui
faiblit se traduit par un écart durable. L'échelle de couleur doit donc être
centrée : neutre à l'écart nul, et s'écartant vers l'ambre puis le rouge de
part et d'autre.

Autre point de câblage : la vue de côté montre deux formes pour quatre
moteurs. Le regroupement avant/arrière dépend de la numérotation des sorties
du contrôleur de vol pour la configuration du châssis, qui n'est pas la
numérotation naturelle. Le livrable doit donc exposer les deux formes
séparément et laisser le regroupement au câblage.

### 4.2 Autres conséquences sur le dessin

1. **Les antennes : dégradé prévu pour le lien principal, deux états pour
   le secondaire.** La qualité du lien principal sera mesurée **côté station
   sol**, en lisant les statistiques du récepteur wfb-ng — donc sans rien
   attendre du drone. Le lien de secours n'a pas de récepteur sub-GHz côté
   station (le boîtier ESP32 ne porte aujourd'hui que des registres de
   voyants et deux boutons), il restera donc en deux états francs : lien
   présent ou perdu. Prévoir les deux traitements.
2. **La température existe pour le CPU et pour chaque ESC**, pas pour le
   reste. Ailleurs, la couleur traduit une panne, pas un échauffement.
3. **Aucun composant ne dispose d'un historique** : le bloc affiche un
   instantané, pas une tendance.

---

## 5. Décisions ouvertes

Points à trancher avant le câblage. Ils n'empêchent pas de concevoir le
bloc, mais le markup doit pouvoir les accueillir.

Trois points ont été tranchés depuis la première rédaction et ne sont plus
ouverts : le régime moteur est transmis (`ESC_RPM`), la correspondance
code de diagnostic → forme du schéma est portée par une colonne `Schematic`
de la table des codes, et la qualité du lien principal sera mesurée côté
station sol.

Restent ouverts :

1. **Calcul de l'autonomie en temps.** Rien ne la transmet. Elle doit être
   calculée depuis le courant instantané et la capacité du pack, cette
   dernière étant une constante de configuration à renseigner. Une moyenne
   glissante est nécessaire : le courant d'un multirotor varie trop d'un
   instant à l'autre pour une estimation brute.
2. **Calcul de l'autonomie en distance.** Dérive de l'autonomie en temps et
   de la vitesse sol. Pose la question de la réserve de retour : une
   distance « atteignable » et une distance « atteignable avec retour » ne
   sont pas la même information, et c'est la seconde qui compte en vol.
3. **Seuils thermiques du CPU.** Le Raspberry Pi 5 commence à se brider vers
   80–85 °C. Les seuils ambre et rouge restent à fixer.
4. **Emplacement du bloc.** Recommandation : colonne de gauche, à la place
   du panneau batterie, puisque la batterie figure désormais dans le schéma.
   La colonne de droite récupère alors de la hauteur pour le journal
   d'événements.

---

---

## 5bis. Ce qui pilote la couleur : deux mécanismes distincts

**Les pannes** passent par la table des codes de diagnostic. Chaque code
porte une colonne `Schematic` listant les formes qu'il allume (par exemple
`MOTOR_FRONT,MOTOR_REAR,ESC` pour une perte moteur), exactement comme il
porte déjà les voyants physiques et l'alarme sonore associés. La couleur
n'est pas stockée : elle découle de la sévérité du code, déjà présente dans
la table. Stocker une couleur séparément permettrait qu'un code signale en
rouge sur le schéma ce qu'il annonce en ambre au panneau d'alarme.

**Les mesures continues** — écart de régime entre moteurs, pourcentage de
batterie, température du CPU ou d'un ESC, qualité de lien — n'ont pas de
code associé et fonctionnent par seuils. Ce sont deux mécanismes qui
coexistent : l'un ne remplace pas l'autre.

Conséquence pour le livrable : une forme doit pouvoir recevoir son état des
deux sources. En pratique, une classe d'état posée sur l'élément suffit, et
la règle de priorité (une panne l'emporte sur une mesure dans le vert) est
écrite côté dépôt.

---

## 6. Livrable attendu

1. Un fragment HTML autonome du bloc (panneau schéma + encadré autonomie),
   SVG inline compris, avec tous les `data-bind` nécessaires.
2. Les règles CSS correspondantes, en classes préfixées de manière cohérente
   (par exemple `.scheme__*` et `.endurance__*`), utilisant les variables
   existantes.
3. La liste des `data-bind` produits, avec pour chacun la nature de la
   valeur attendue (texte, couleur, largeur de remplissage, classe d'état).
   Cette liste est ce qui permet le câblage sans relire le markup.

Pas de JavaScript : la mise à jour est écrite côté dépôt, dans le moteur de
rendu existant.


---

## 7. État du câblage

Bloc intégré dans la colonne de gauche, sous le panneau Lidar. Les panneaux
Batterie et CPU Temp ont été retirés, markup, styles et code de rendu
compris.

| Forme | État aujourd'hui |
|---|---|
| `batt` | **Câblé** — seuils 30/10 %, `--fill` sur la charge, clignotement sous 10 % |
| `gps` | **Câblé** — type de fix, puis HDOP > 2 ou moins de 7 satellites en `warn` |
| `cpu` | **Câblé** — seuils 72 / 82 °C, température affichée dans la forme |
| `fc` | **Câblé** — santé du composant `MavlinkInterface` |
| `lidar` | **Câblé** — fraîcheur du paquet `LIDAR_SCAN` |
| `motor_front`, `motor_rear` | **Câblé** — écart au régime moyen, `--dev` saturant à 12 % |
| `mag`, `esc`, `ant_main`, `ant_backup` | Codes de diagnostic seulement : gris tant qu'aucun code n'est actif |
| `esc_1` … `esc_4` | Gris en permanence : la température par ESC n'est pas transmise |

Deux points restent approximatifs et se calibreront au banc :

- Les seuils CPU (72 / 82 °C) sont une estimation à partir du bridage connu
  du Raspberry Pi 5, pas une mesure.
- L'écart moteur de 12 % au-delà duquel la couleur sature est arbitraire.
  Un vol stationnaire réel donnera l'ordre de grandeur juste.
- Le regroupement avant/arrière suppose un quadrirotor en X dont les sorties
  1 et 3 sont à l'avant. À confirmer au banc : une inversion donnerait un
  écart de signe opposé, pas une erreur visible.

Les deux estimations d'autonomie restent à `--` : leur calcul n'est pas
tranché (cf. §5).
