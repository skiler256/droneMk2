# world_engine/models/

Dépose ici le modèle 3D du drone exporté depuis Blender, en **.glb**,
**échelle mètres** (cohérent avec le reste du monde, cf. `World.gd`).

Une fois le fichier présent, indique son chemin dans `Viewer.tscn` (nœud
`Viewer`, propriété exportée `drone_model_path`, ex.
`res://models/drone.glb`) — `Viewer.gd` le charge à la place du marqueur
cube/nez si le champ est renseigné et le fichier existe.

Convention d'orientation à respecter dans Blender avant export : le nez du
drone doit pointer vers **-Y** (avant l'export glTF, qui convertit vers sa
propre convention +Y haut / -Z avant). Si le modèle apparaît tourné une fois
importé, ajuste `drone_model_yaw_offset_deg` (même nœud) par pas de 90° —
c'est un simple correctif d'axe, pas un signe d'échec d'import.
