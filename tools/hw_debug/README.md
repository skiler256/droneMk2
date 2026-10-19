# hw_debug

Scripts Python autonomes pour tester un module radio/capteur isolément, sur
banc ou en réparation — pas de dépendance au reste du projet (pas de shm, pas
de CMake, pas de SITL). But : vérifier qu'un composant matériel répond avant
de le réintégrer, sans avoir à lancer tout le système.

- `CC1101_test.py` — module radio CC1101 (liaison secondaire).
- `LD06_test.py` — lidar LD06.

## Bancs C++ — les drivers de production en direct

Même esprit, mais ils exercent les **vrais drivers** (`src/Drivers/`) au lieu
de reparler le protocole en Python : ce qui est validé est exactement ce qui
volera. Sortie console uniquement, aucun privilège RT, aucune shm, aucun
dashboard.

```bash
cmake -B build_bench -DBUILD_HW_BENCH=ON && cmake --build build_bench -j
./build_bench/bench_gps                 # NEO-M8N, défaut /dev/ttyAMA1
./build_bench/bench_lidar               # LD06,    défaut /dev/ttyAMA2
./build_bench/bench_mag                 # LIS3MDL, défaut I2C 0x1C
```

Le port ou l'adresse se passe en argument (`./bench_gps /dev/ttyUSB0`).
Rafraîchissement sur place à ~4Hz, Ctrl+C pour sortir.

Ce qu'ils montrent :

- **`bench_gps`** — fix et `gpsFix` brut, satellites suivis/détaillés,
  position, altitude MSL, velNED, cap, hAcc/vAcc/sAcc, hDOP/vDOP, date UTC,
  iTOW, table par satellite, et le **compteur de trames rejetées sur
  checksum**. Ce dernier est le plus utile au bring-up : une valeur qui grimpe
  désigne le câblage ou le blindage, pas le logiciel.
- **`bench_lidar`** — nuage par secteurs de 30° (distance minimale, nombre de
  points valides, barre d'autant plus longue que l'obstacle est proche), total
  de points valides sur 360, et compteur de trames à CRC invalide. 360 valeurs
  brutes seraient illisibles en console ; les secteurs suffisent à vérifier que
  le capteur voit l'obstacle dans la bonne direction.
- **`bench_mag`** — trois axes en Gauss et la norme, avec le repère
  attendu (~0.25-0.65 G sur Terre). Le LIS3MDL n'est plus câblé sur le Pi
  (`SENSORFUSION.md` §3.5) ; l'outil sert s'il y est rebranché.

Un échec d'`init()` affiche l'erreur en clair (`UARTAttrGetFailed`,
`I2COpenFailed`, ...) et sort en code 1, plutôt que de tourner dans le vide.

Les parseurs de ces mêmes drivers sont couverts par `tests/test_drivers.cpp`
(trames forgées, CRC/checksum invalides, trames tronquées et concaténées) —
le banc valide le matériel, les tests valident le décodage.

Différent de `src/Drivers/` (drivers de production, utilisés par `drone`) et
de `sitl/*_wire.py` (codecs utilisés par le harnais SITL) : ces scripts-ci ne
sont jamais appelés par le système, uniquement lancés à la main pour du debug
ponctuel.
