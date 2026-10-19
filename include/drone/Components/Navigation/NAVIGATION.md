# Navigation — architecture, Tasks, pièges

Documentation de référence du composant `Navigation` : rôle de chaque
Task, comment un segment de vol devient une consigne pour
`MavlinkInterface`, les décisions d'archi actées (dont le modèle
armé/engagé façon FMA Airbus), et ce qui reste délibérément différé.
Complète `GUIDE_COMPOSANTS.md` (générique) et le SDD (§3.4).

**État** : V1 = un seul type de segment (`Waypoint` — cap direct sur un
point, vitesse d'approche, altitude). Séquencement armé/engagé
opérationnel, hold de sécurité redondant opérationnel, P-controller
altitude migré depuis `MavlinkInterface`. **Validé en SITL réel**
(scénarios `002_arm_waypoints_land` et `004_lidar_course`, cf.
`sitl/SITL.md` §8 — vrais `Waypoint`, Nav pilote dessus, détection
d'obstacle confirmée en vol). Line/Curve, la vraie reprise après panne
côté MissionControl, et l'axe vitesse "constante" restent à faire.

## 1. Vue d'ensemble

```
MissionControl ──setArmedSegment──► SharedCmdMem.armedSegment
MissionControl ──pushNavCommand(MCO005, amorçage)──► SharedCmdMem.navCmds
                                          │
                                          ▼
                                       TSeq ──canEngage()──► SharedCmdMem.engagedSegment
                                          │                  (+ FMA latéral)
                                          ▼
SharedSFMem.state (position/vitesse) ──► TAP ──► SharedNavMem.cmd (NavCommand)
                                          │        (+ FMA vertical/vitesse)
                                          │        ──► MavlinkInterface (SET_POSITION_TARGET_LOCAL_NED)
                                          ▼
SharedSFMem.state + .lidar ───────────► TFS ──holdActive_──► TAP (hold si levé)
                                          (sécurité redondante, indépendante de TAP)
```

Trois Tasks, volontairement séparées par **nature du travail** plutôt que
par simple round-robin : `TSeq` (événementiel, séquencement), `TAP`
(calcul pur, latence critique), `TFS` (surveillance indépendante). Cf. §2.

## 2. Les 3 Tasks

| Task | Rôle | Fréquence |
|---|---|---|
| `TSeq` (`TSeqTask`) | Dépile `navCmds`, rafraîchit armé/engagé, promeut armé→engagé (sur amorçage ou sur `Atteint`), tient le FMA latéral | 20Hz |
| `TAP` (`APTask`) | Calcule la consigne (`NavCommand`) via la loi de correction du segment engagé + l'axe vertical, tient le FMA vertical/vitesse | 50Hz |
| `TFS` (`TFSTask`) | Sécurité redondante : fraîcheur EKF (indépendante du check de SF), obstacle LIDAR — déclenche `holdActive_` | 20Hz |

`TSeq` et `TFS` tournent à la même fréquence (20Hz) mais pour des raisons
différentes : `TFS` parce que c'est une boucle de surveillance, pas de
contrôle ; `TSeq` parce que le séquencement est événementiel (réagir en
20ms ou 100ms après une arrivée ne change rien en pratique) — **pas**
parce qu'il devrait tourner à la cadence de `TAP` (calcul de contrôle,
lui doit rester à 50Hz).

## 3. Modèle armé/engagé (façon FMA Airbus)

**Décision actée en session**, inspirée du FMA (Flight Mode Annunciator)
Airbus : `MissionControl` ne pousse plus directement "le" segment en
exécution. Il **arme** un segment (`setArmedSegment`), et soit :

- l'amorce explicitement la première fois (`pushNavCommand(MCO005, ...)`,
  dépilé par `TSeq`) ;
- laisse `TSeq` le promouvoir **automatiquement** quand le segment engagé
  atteint `SegmentStatus::Atteint` — MC n'a rien à faire tant qu'il garde
  toujours un segment armé en réserve pendant que l'engagé tourne.

Si rien n'est armé quand l'engagé se termine : `TAP` continue de
recevoir `engagedSegment_` inchangé, `computeWaypointGuidance()` renvoie
`arrived=true` en continu (vitesse nulle) — un hold de fait, sans
intervention de `TFS`.

### `NavSegment::seq` — sentinel, pas juste une clé de comparaison

`seq` est assigné par `setArmedSegment()` (jamais par l'appelant, même
raison que `PendingCommand::seq`), via un compteur dédié
(`nextSegmentSeq`) qui **démarre à 1** — `seq == 0` sert de sentinel
non-ambigu "rien d'armé" pour `TSeqTask::tryPromote()`. Une valeur par
défaut de `NavSegment{}` (armedSegment jamais écrit) a donc `seq=0`,
indiscernable d'un "vrai" premier segment seulement si celui-ci avait pu
avoir `seq=0` — d'où le démarrage à 1 plutôt que 0.

### `entryWp` — utilisé pour tous les types, pas seulement Line/Curve

Contrairement à l'intention initiale ("inutilisé pour Waypoint"),
`entryWp` sert désormais de **position attendue au début du segment**,
pour tous les types — c'est contre ce point que `TSeq` vérifie la
sécurité avant de promouvoir (`NAV_CONV::canEngage`, tolérance 2m,
`kEngageToleranceM`). Pour un `Waypoint`, ça n'a pas d'effet sur la loi
de correction elle-même (`computeWaypointGuidance` ne s'en sert pas —
elle vise `exitWp` depuis la position courante, peu importe d'où) : ça
sert uniquement de garde-fou côté séquencement, pour éviter d'engager un
segment alors que le drone est loin d'où MissionControl le pensait.

### Pas d'"armé" sur l'axe vitesse

Contrairement au latéral (un segment futur concret) et au vertical (un
changement d'altitude futur), une loi de vitesse (proportionnelle,
future loi constante) n'a pas de notion de "capture" à attendre — elle
s'applique dès qu'active. D'où `speedEngaged` seul dans `SharedNavMem`,
pas de `speedArmed`.

## 4. `TSeq` — séquencement

`TSeqTask::tryPromote()` est le point d'entrée unique de la promotion
armé→engagé, appelé soit sur réception de `MCO005` (amorçage), soit
quand `SegmentProgress.status == Atteint` (auto-séquencement). Refuse si
rien n'est armé (`seq==0`), si le segment armé est déjà celui engagé
(`armedSegment_.seq == engagedSegment_.seq` — évite de rejouer
`newSegment_` en boucle), ou si `NAV_CONV::canEngage()` échoue.

Les ordres `navCmds` autres que `MCO005` (ex: la future reprise après
panne) ne sont pas encore définis — accusé d'échec systématique via
`pushNavReport(..., false)`, même parade que le `TCmd` de
`MavlinkInterface` pour ne jamais laisser MC attendre indéfiniment.

## 5. `TAP` — calcul pur, 3 axes indépendants

`APTask::loop()` combine 2 calculs indépendants (latéral + vertical)
dans un seul `NavCommand`, plus la mise à jour de 2 des 3 codes FMA
(vertical, vitesse — le latéral est tenu par `TSeq`, cf. §3).

> **Bug trouvé en test SITL, corrigé en session** : `APTask::loop()` ne
> vérifiait pas `engagedSegment_.seq == 0` ("rien n'a jamais été engagé")
> avant de calculer/publier une consigne. Comme `SegmentType` n'a qu'une
> valeur (`Waypoint`), le `switch` matchait toujours, et `NavSegment{}`
> par défaut vise `exitWp={0,0,0}`/`targetAltitude=0.0` — Nav publiait
> donc en continu, dès le démarrage, une consigne visant le sol/l'origine.
> Conséquence concrète : le décollage (déclenché côté ArduPilot via
> `MAV_CMD_NAV_TAKEOFF`, accepté) ne décollait jamais, `vz` de Nav
> combattant activement la montée. Fix : `APTask::loop()` retourne tôt
> (ne publie rien) tant que `engagedSegment_.seq == 0`.

### Axe latéral — `computeWaypointGuidance` (`NAV_CONV`)

Cap direct (`atan2`) vers `exitWp`, vitesse = P-controller sur la
distance restante saturé à `segment.speed` (`kApproachKp=0.6`, même gain
que l'axe vertical pour cohérence de "feel") — pas de dépassement même
sur une cible proche. `arrived` (dist ≤ `kArrivalTolerance`=0.5m,
constaté suffisant en usage réel — la dérive lente de ce GPS reste
<1.5-2m) pilote `SegmentStatus::Atteint`.

Pas de PID (I/D) sur cet axe : un waypoint est une cible kinématique, pas
un suivi de trajectoire — le PID de guidage latéral est réservé au futur
suivi de ligne/courbe (Line/Curve, V2), qui en aura réellement besoin.

### Axe vertical — `computeVerticalGuidance` (`NAV_CONV`)

**Migré depuis `MavlinkInterface::altitudeToVz()`** (mêmes gains,
`kAltitudeKp=0.6`, `kMaxClimbRateMs=2.0`) — décision actée en session :
Nav possède maintenant tout le contrôle vertical, `MavlinkInterface` ne
fait plus que retransmettre `NavCommand::velNed.ned.z` tel quel.
Classification Hold/Climb/Descend par écart à la cible
(`kAltitudeHoldToleranceM=0.3m` — en dessous, Hold plutôt qu'un mode qui
oscille au bruit de mesure).

**Convention NED** : `vz<0` = monte, `error = target - current < 0` (la
cible est plus haute, donc plus négative) → `vz<0`. Cohérent avec
`Position`/`posNed` partout ailleurs dans le code.

> **Bug trouvé et corrigé pendant la migration** : la branche hold de
> `APTask::loop()` (écrite avant cette migration) faisait
> `hold.altitudeCmd = Meters{-state->posNed.ned.z}` — une négation
> erronée, puisque `posNed.ned.z` est déjà dans la même convention NED
> que `altitudeCmd`. Trouvé en re-dérivant soigneusement les signes pour
> `computeVerticalGuidance()`, corrigé (plus de négation).

### `NavCommand::altitudeCmd` — devenu indicatif seulement

Depuis la migration, `MavlinkInterface` ne lit plus `altitudeCmd` (la
vraie commande passe par `velNed.ned.z`). Le champ reste dans
`SharedNavMem` pour l'affichage GCS (`UDP_bridge`/dashboard le
sérialisent déjà) — Nav continue de l'écrire (cible en cours, ou
altitude courante pendant un hold) mais son rôle de contrôle a disparu.

## 6. `TFS` — sécurité redondante

Deux checks indépendants, chacun avec son flag edge-triggered
(`ekfStaleActive_`/`obstacleActive_`, même pattern que
`gpsFaultActive_` côté SF) — `holdActive_` (partagé, dans `Navigation`)
est l'OR des deux, recalculé à chaque tick, jamais lui-même
edge-triggered.

### Fraîcheur EKF — redondante, pas une relecture du même seuil

`TFS` vérifie `SharedSFMem.state.ts` avec `kEkfStaleThreshold=400ms` —
**pas une relecture du seuil `kAgeThrEKF` d'`EKFTask` côté SF** (1000ms,
cf. `SENSORFUSION.md` §4) : ce n'est pas le même check (`EKFTask` vérifie
la fraîcheur des messages MAVLink du FC ; `TFS` vérifie que **SF
elle-même** publie encore un état, ce qui couvre aussi un SF gelé/planté
que `EKFTask` ne peut pas
détecter sur lui-même).

### Obstacle — boîte, pas cône, alignée sur la trajectoire réelle

Ce check consomme le LIDAR **OCAS** (LD06, cf. `HARDWARE.md` §3) — c'est
la fonction d'évitement horizontal, l'équivalent embarqué d'un TCAS.

**Contrainte matérielle** : le LIDAR est monté sous le nez, devant la
batterie, champ ~200° (non mesuré précisément) — angle mort à l'arrière.
Pour que la détection reste dans ce champ, `TFS` calcule sa propre
direction "avant" (`NAV_CONV::travelDirection`) à partir de
`state.velNed` (la vitesse **réelle**, pas le cap commandé) si elle
dépasse `kVelocityDirectionThreshold=0.5m/s`, sinon repli sur le cap vers
la cible (il faut toujours une direction à observer). Ce choix suppose
que le nez pointe à peu près où le drone va (marge ±50° avec le champ
LIDAR ~200°) — cohérent avec la loi de correction latérale, qui commande
justement un cap = direction de déplacement.

**`travelDirection()` n'utilise que la vitesse HORIZONTALE**, jamais
`state.velNed.norm()` (la norme 3D). Bug réel constaté en SITL avant
correction : en montée quasi verticale, la norme 3D est dominée par `vz`,
donc `speed` franchit le seuil alors que `vx`/`vy` sont quasi nuls. Le
vecteur retourné `(vx/speed, vy/speed, 0)` n'est alors plus unitaire — sa
norme tend vers 0. Or `obstacleInBox()` **suppose `forward` unitaire** pour
projeter les points LIDAR : avec un vecteur écrasé, un obstacle réel à
plusieurs mètres se projette à quelques centimètres et tombe faussement dans
la boîte de sécurité. Symptôme observé : `NVA004` (`hold_obstacle`) se
déclenchait en pleine montée, sans aucun obstacle à proximité.

Toute modification de `travelDirection()` doit donc préserver deux
invariants : composante verticale exclue du calcul de `speed`, et vecteur de
sortie unitaire.

Zone de détection : **carré** 2×2m devant le drone (`kObstacleBoxForwardM`,
`kObstacleBoxLateralM`), pas un cône — projection de chaque point LIDAR
(déjà en NED relatif au drone, cf. `SharedSFMem::LidarMap`) sur les axes
avant/latéral (produit scalaire), test de boîte. Plus simple qu'une
trigonométrie d'angle, et les points sont déjà dans le bon repère.

`TFS` recalcule tout indépendamment de `TAP` — ne fait confiance à aucun
calcul déjà fait ailleurs, c'est le principe même d'une sécurité
redondante.

## 7. Affichage FMA — `CODES::Code` comme type, pas comme mécanisme de panne

**Décision actée en session**, point d'archi à retenir : les 5 champs
FMA de `SharedNavMem` (`lateralArmed`/`lateralEngaged`,
`verticalArmed`/`verticalEngaged`, `speedEngaged`) réutilisent le type
`CODES::Code` (encodage Component/Category/Sévérité déjà là, déjà
nommé/affichable) **sans passer par `sysState_.raiseCode`/`clearCode`** —
ce ne sont pas des pannes (potentiellement plusieurs actives à la fois,
ring buffer), mais un état mutuellement exclusif par axe. Valeurs
écrites directement en shm (`setLateralArmed`, etc.), lues par le GCS
comme n'importe quel autre champ `SharedNavMem`.

Champs à plat dans `SharedCmdMem::payload` (pas un struct imbriqué) :
`TSeq` (latéral) et `TAP` (vertical/vitesse) écrivent chacun les leurs
sans risque de s'écraser mutuellement — un `setData()` par champ, verrou
CAS individuel (cf. `SharedMemory.hpp`).

`verticalArmed` n'est **jamais écrit** pour l'instant — pas de notion de
mode vertical "futur en attente" tant qu'il n'y a qu'une seule loi
(passthrough + Hold/Climb/Descend appliqués immédiatement, pas de
capture différée comme pour un segment latéral).

## 8. Tests unitaires (`NAV_CONV`, `tests/test_navigation.cpp`)

Même principe que `MavlinkInterface`/`SensorFusion` : les `loop()` des 3
Task sont privées, appelées uniquement par le thread RT — pas testées
directement. Toute la logique numérique est dans `Conversions.hpp`
(`computeWaypointGuidance`, `travelDirection`, `obstacleInBox`,
`canEngage`, `lateralModeCode`, `computeVerticalGuidance`), 30 cas de
test au total.

## 9. Table des codes

### `CODES::Category::Action`, préfixe `NVA` (Navigation)

| Code | Nom | Sévérité | Déclencheur |
|---|---|---|---|
| `NVA001`-`NVA003` | landing_cat_1/2/3 | Critical/Info/Info | Atterrissage d'urgence/assisté/guidé — pas encore câblés (post-V1) |
| `NVA004` | hold_obstacle | Critical | `TFS` — obstacle dans la zone de sécurité |
| `NVA005` | hold_ekf_stale | Critical | `TFS` — `DynamicState` SF périmé |
| `NVA006` | lateral_waypoint | Info | FMA latéral — segment `Waypoint` armé ou engagé |
| `NVA007` | vertical_hold | Info | FMA vertical — maintien d'altitude |
| `NVA008` | vertical_climb | Info | FMA vertical — montée |
| `NVA009` | vertical_descend | Info | FMA vertical — descente |
| `NVA010` | speed_proportional | Info | FMA vitesse — loi P saturée (seule implémentée) |

### `CODES::Category::Command`, préfixe `MCO` (Mission Control → Nav)

| Code | Nom | Déclencheur |
|---|---|---|
| `MCO005` | start_segment | Amorçage — promeut `armedSegment` en `engagedSegment` si sécurité OK |

## 10. Ce qui n'est PAS encore fait / tranché

- **Line/Curve** : `SegmentType` n'a que `Waypoint`. Les switchs
  (`APTask::loop()`, `NAV_CONV::lateralModeCode`) sont volontairement
  non-exhaustifs-avec-default — `-Wswitch` forcera une mise à jour à
  l'ajout d'un nouveau type.
- **PID de guidage latéral** (suivi de ligne/courbe) : pas implémenté,
  réservé à Line/Curve.
- **Loi de vitesse "constante"** : évoquée en discussion comme
  alternative à la loi proportionnelle actuelle, jamais implémentée —
  `speedEngaged` affiche toujours `NVA010` (Proportional).
- **Reprise après panne** (MC → Nav, après un hold) : rôle de MC réduit
  à l'amorçage initial + une future reprise, cette dernière différée
  (reportée au chantier MissionControl).
- **`verticalArmed`** : jamais écrit (§7) — pas de notion de mode
  vertical futur en attente tant qu'il n'y a qu'une loi immédiate.
- **`HomeCalibrator`** côté SF toujours à `{0,0,0}` (cf. `SENSORFUSION.md`
  §4) — `Navigation` reçoit donc une position déjà "recalée" mais qui ne
  l'est en réalité pas encore.
- **HIL** (matériel réel, pas seulement SITL) : pas encore fait pour
  Navigation — `hil_sf_mv` couvre SensorFusion+MavlinkInterface mais pas
  encore Nav (cf. `sitl/SITL.md` §12).
