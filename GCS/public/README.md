# GCS Dashboard — version fonctionnelle

Portage de la maquette Claude Design en HTML/CSS/JS vanilla. Aucune dépendance
de build, aucun framework. Seule dépendance externe : `three.js` (CDN), utilisée
uniquement par le viewport 3D.

## Lancer

```bash
python3 -m http.server 8080 --directory .
# puis http://localhost:8080
```

Le simulateur tourne par défaut : l'IHM s'anime sans backend.

## Arborescence

```
index.html          structure seule, zéro style inline
css/gcs.css         tous les styles, palette en variables CSS
js/state.js         modèle de données (unités SI) + fraîcheur
js/dom.js           accès DOM : dirty-checking + pools préalloués
js/render.js        état -> DOM. Tout le formatage est ici
js/link.js          couture WebSocket  <-- le fichier à compléter
js/sim.js           source factice (démo / dev)
js/main.js          la boucle
js/viewport3d.js    custom element three.js, repris tel quel
```

## Les trois cadences

Elles sont indépendantes, et c'est le point central de l'architecture :

| | fréquence | qui pilote |
|---|---|---|
| arrivée des paquets | 1–20 Hz selon le `PacketID` | le drone, via `ws.onmessage` |
| rendu | `RENDER_HZ` (20 Hz), fixe | `main.js` |
| viewport 3D | rAF interne à three.js | `viewport3d.js` |

Conséquences : un burst de paquets ne provoque pas un burst de rendus, et une
coupure de lien ne fige pas l'IHM — elle la fait passer en « périmé ».

## Brancher le WebSocket

Deux modifications, dans `js/main.js` :

```js
var USE_SIMULATOR = false;
var WS_URL = 'ws://raspberrypi.local:8765';
```

Puis adapter `GCS.applyPacket()` dans `js/link.js` au format réel de ton pont.
Les noms de cas (`GPSFIX`, `POS`, `VELNED`, `ATTITUDE`, `BATTERY`,
`MISSION_STATE`, `ARM_STATE`, `HEALTH`, `CODE`) reprennent ceux de
`TELEMETRY.csv`.

Deux règles à respecter dans le handler :

1. **N'écrire que des grandeurs physiques en SI.** Jamais de chaîne formatée,
   jamais de couleur, jamais de pourcentage d'affichage. Le formatage appartient
   à `render.js` et à lui seul.
2. **Appeler `GCS.touch(state, groupe)` à chaque écriture.** C'est ce qui
   alimente l'indicateur de fraîcheur (`state.stale.*` → classe `.is-stale`).
   Sans ça, le panneau reste grisé en permanence.

Conventions déjà en place, alignées sur le reste du projet :

- position/vitesse en NED, `vd > 0` = descente (le HUD affiche `-vd`) ;
- angles en degrés ;
- lidar en polaire repère body, `bearingDeg` ∈ [-100, 100], nez = 0°.

## Commandes montantes

Les boutons portent un attribut `data-cmd`. Le handler unique est
`onCommand()` dans `main.js` :

```html
<div class="act__btn" data-cmd="RTL">RETURN TO LAUNCH</div>
```

Ajouter un bouton = ajouter un `data-cmd` et un `case`. Rien d'autre à câbler.

Note : en mode réel, `ARM_TOGGLE` n'inverse pas l'état local — il envoie la
commande et attend le paquet `ARM_STATE` de confirmation. Une IHM qui bascule
son propre affichage ment sur l'état réel de la machine.

## Ajouter un champ affiché

1. le champ dans `GCS.createState()` (`state.js`) ;
2. l'écriture dans `GCS.applyPacket()` (`link.js`) ;
3. `<span data-bind="monChamp">--</span>` dans `index.html` ;
4. `D.text('monChamp', fx(state.x.y, 1))` dans le sous-rendu concerné.

Pour une liste : `data-list="maListe"` côté HTML, `D.pool()` à l'init,
`D.applyPool()` au rendu.

## Choix d'implémentation

- **Dirty-checking systématique.** Chaque setter mémorise la dernière valeur
  écrite et ne touche au DOM que si elle change. Un `render()` répété à
  l'identique effectue zéro écriture (vérifié en test).
- **Pools préalloués.** Barres, graduations, points lidar, lignes de log sont
  créés une fois à leur taille max puis mis à jour ou masqués. Aucun
  `createElement` dans la boucle, aucun churn GC.
- **Seule la vue centrale visible est rendue.** Le HUD représente environ la
  moitié du coût de rendu ; il n'est pas calculé quand l'onglet 3D est actif.

## Deux corrections par rapport à la maquette

La maquette plaçait la graduation courante à 13 px au-dessus du centre des
bandeaux vitesse/altitude, et à 20 px à gauche du centre pour le cap. C'était
un décalage constant, sans lien avec la valeur. Corrigé via les constantes
`TAPE_CENTER` et `HSI_CENTER` en tête de `render.js` — remettre `113` et `150`
pour retrouver le comportement d'origine.

## À faire avant un vol réel

- **Vendoriser `three.js` et les polices Google.** Actuellement chargées depuis
  un CDN : sur le terrain sans internet, le viewport 3D ne démarrera pas et la
  typographie tombera sur les polices système.
- **Décider du transport.** `link.js` suppose du JSON. Si tu passes les trames
  `TELEM::*` binaires directement, il faudra un décodeur JS équivalent à
  `telemetry_codec.py` — l'endroit à modifier est `ws.onmessage`, rien d'autre.
