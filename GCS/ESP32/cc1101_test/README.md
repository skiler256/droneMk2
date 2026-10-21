# Banc CC1101 — station sol

Projet PlatformIO **séparé** du firmware du panneau d'alarme
(`../firmware`), pour qu'un essai de câblage radio ne puisse pas priver la
malette de ses voyants et de ses boutons d'acquittement.

Équivalent côté sol de `tools/hw_debug/CC1101_test.py`, qui remplit le même
rôle côté drone.

## Ce qu'il fait, et ce qu'il ne fait pas

Il lit l'identité de la puce (`PARTNUM`, `VERSION`) et ses registres de
fréquence, puis les affiche sur le port série à 115200 bauds, une fois par
seconde.

Il **n'écrit aucun registre de configuration** : la radio n'est jamais
programmée, rien n'est émis. Il ne touche ni aux MCP23017 ni aux boutons.

## Câblage attendu

| Signal | GPIO |
|---|---|
| SCK | 14 |
| MISO | 35 |
| MOSI | 13 |
| CS | 4 |

MISO sur 35 plutôt que sur le 12 par défaut de HSPI : le GPIO12 est une
broche de *strapping* de l'ESP32, et la tirer au niveau haut au démarrage
empêche la carte de booter. Le 35 est une entrée seule, ce qui convient à
MISO.

Ces broches ne figurent pas dans `../mapingESP.csv`, qui ne décrit que les
voyants et les boutons. À y ajouter une fois le câblage confirmé.

## Utilisation

```bash
cd GCS/ESP32/cc1101_test
pio run -t upload          # flash
pio device monitor         # lecture du port série
```

## Lecture du résultat

| Affichage | Signification |
|---|---|
| `OK PARTNUM=0x00 VERSION=0x14` | puce vivante, câblage SPI correct |
| `PARTNUM=0xFF VERSION=0xFF` | MISO probablement non connecté — lecture à vide |
| `PARTNUM=0x00 VERSION=0x00` | MISO bloqué au niveau bas, ou puce non alimentée |
| `ECHEC: la puce ne relache jamais MISO` | CS, alimentation 3V3 ou masse |

La valeur `FREQ` affichée vaut `0x1EC4EC`, soit 800 MHz avec un quartz de
26 MHz : c'est la valeur par défaut au reset, pas un réglage. La lire
confirme simplement que la liaison SPI transporte de vraies valeurs et non
des zéros ou des `0xFF`.

## Retour au firmware normal

```bash
cd ../firmware && pio run -t upload
```
