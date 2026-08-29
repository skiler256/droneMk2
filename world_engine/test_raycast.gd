# world_engine/test_raycast.gd
# ─────────────────────────────────────────────────────────────────────────────
# Test manuel en isolation du bloc [3] (geometrie pure, sans le reste du
# pipeline SIM_STATE/modele capteur/UDP) -- verifie que World.gd charge un
# monde JSON et que le raycast 360 degres detecte correctement les obstacles.
#
# Usage : godot --headless --script world_engine/test_raycast.gd
#         (execute depuis world_engine/, ou avec --path world_engine)

extends SceneTree


func _init():
	var world = load("res://World.gd").new()
	get_root().add_child(world)
	world.load_from_json("res://worlds/trivial.json")

	# Pose de test : origine, 1m d'altitude, attitude nulle (cap nord).
	var pos_ned = Vector3(0.0, 0.0, -1.0)  # north, east, down

	var distances = world.raycast_360(pos_ned, 0.0, 0.0, 0.0)

	var hits = 0
	for d in distances:
		if d > 0.0:
			hits += 1
	print("raycast_360: ", hits, " / 360 rayons ont touche un obstacle")

	# Attendu : la box est plein nord (angle 0), le cylindre plein est (angle 90).
	print("angle 0 (nord, box attendue a 9m -- centre 10m, demi-epaisseur 1m) : ", distances[0])
	print("angle 90 (est, cylindre attendu ~4.5m) : ", distances[90])
	print("angle 180 (sud, rien attendu) : ", distances[180])
	print("angle 270 (ouest, rien attendu) : ", distances[270])

	# TF Luna, attitude nulle : rien en dessous dans trivial.json -> 0.0.
	print("raycast_down, attitude nulle (rien attendu) : ", world.raycast_down(pos_ned, 0.0, 0.0, 0.0))

	# Yaw 90 deg (cap est) : le cylindre (plein est en NED) doit maintenant
	# etre vu au rayon 0 (le nez pointe dessus), plus a l'angle 90.
	var yaw90 = world.raycast_360(pos_ned, 0.0, 0.0, deg2rad(90))
	print("yaw=90, angle 0 (nez pointe est, cylindre attendu ~4.5m) : ", yaw90[0])

	# Roll 90 deg, a 5m d'altitude au-dessus du centre de la box (nord=10,
	# est=0 -- correspond au centre de la box en x/z Godot). Un roll pur ne
	# change pas l'axe avant du corps (x_body inchange par Rx) mais tourne
	# l'axe droite du corps (y_body) vers le bas -> le rayon a l'angle 90
	# (qui vise y_body) doit pointer droit vers le bas et toucher le sommet
	# de la box (alt 1.5m, cf. trivial.json : centre alt 1m, demi-hauteur
	# 0.5m) a 5 - 1.5 = 3.5m, pendant que le rayon 0 (x_body, inchange par
	# le roll) reste horizontal plein nord et ne touche rien a cette altitude.
	var pos_over_box = Vector3(10.0, 0.0, -5.0)
	var roll90 = world.raycast_360(pos_over_box, deg2rad(90), 0.0, 0.0)
	print("roll=90 au-dessus de la box, angle 0 (nez horizontal, rien attendu) : ", roll90[0])
	print("roll=90 au-dessus de la box, angle 90 (droite corps -> bas, box attendue a 3.5m) : ", roll90[90])
	print("roll=90 au-dessus de la box, raycast_down (bas du CORPS -> horizontal, rien attendu) : ", world.raycast_down(pos_over_box, deg2rad(90), 0.0, 0.0))

	quit()
