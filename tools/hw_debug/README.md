# hw_debug

Scripts Python autonomes pour tester un module radio/capteur isolément, sur
banc ou en réparation — pas de dépendance au reste du projet (pas de shm, pas
de CMake, pas de SITL). But : vérifier qu'un composant matériel répond avant
de le réintégrer, sans avoir à lancer tout le système.

- `CC1101_test.py` — module radio CC1101 (liaison secondaire).
- `LD06_test.py` — lidar LD06.

Différent de `src/Drivers/` (drivers de production, utilisés par `drone`) et
de `sitl/*_wire.py` (codecs utilisés par le harnais SITL) : ces scripts-ci ne
sont jamais appelés par le système, uniquement lancés à la main pour du debug
ponctuel.
