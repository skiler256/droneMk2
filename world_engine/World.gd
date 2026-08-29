# world_engine/World.gd
# ─────────────────────────────────────────────────────────────────────────────
# Bloc [3] MOTEUR 3D (INTEGRATION_TEST.md §3.3) -- responsabilite unique :
# la geometrie. Charge un monde depuis un fichier de donnees (JSON, PAS une
# scene modelee a la main -- cf. §10.3, format editable pour fabriquer des
# mondes adversariaux V2), place les obstacles comme StaticBody/CollisionShape,
# et effectue un raycast 360 degres depuis une pose donnee.
#
# Convention de repere : NED (nord/est/bas, meme convention que le reste du
# systeme) -> Godot (X=est, Z=-nord, Y=haut=-bas). Angle 0 du raycast = axe
# "nez" du drone (avant, apres rotation par le yaw), sens horaire, meme
# convention que distance_mm[360] (index=degre) cote UdpLidarDriver.
#
# Aucun bruit/modele capteur/injection de panne ici (interdit §3.3, bloc [4]
# separe) -- juste des distances geometriques brutes.

extends Spatial

const MAX_RANGE_M = 30.0  # portee max simulee, valeur de depart arbitraire

# Charge ce fichier automatiquement au _ready() si renseigne dans
# l'Inspecteur -- pratique pour un visuel direct en editeur (Play sur la
# scene), pas necessaire pour l'usage headless (test_raycast.gd appelle
# load_from_json() lui-meme).
export(String, FILE, "*.json") var auto_load_json = ""

# Materiau simple, une seule instance partagee -- juste pour distinguer
# visuellement les obstacles du sol/vide, aucune pretention esthetique
# (ce bloc ne fait QUE de la geometrie, cf. §3.3).
const OBSTACLE_COLOR = Color(0.8, 0.3, 0.2)


func _ready():
	if auto_load_json != "":
		load_from_json(auto_load_json)


func ned_to_godot(north: float, east: float, down: float) -> Vector3:
	return Vector3(east, -down, -north)


# Tourne un vecteur DIRECTION du repere corps (x=avant/nez, y=droite, z=bas)
# vers NED (x=nord, y=est, z=bas), rotation aeronautique standard
# R = Rz(yaw) Ry(pitch) Rx(roll). Necessaire car le LD06 et le TF Luna sont
# montes rigides sur le chassis (cf. discussion roadmap etape 4) : leur
# mesure suit toute l'attitude du drone, pas seulement le cap.
func body_to_ned(v: Vector3, roll: float, pitch: float, yaw: float) -> Vector3:
	var u = Vector3(v.x, v.y * cos(roll) - v.z * sin(roll), v.y * sin(roll) + v.z * cos(roll))
	var w = Vector3(u.x * cos(pitch) + u.z * sin(pitch), u.y, -u.x * sin(pitch) + u.z * cos(pitch))
	return Vector3(w.x * cos(yaw) - w.y * sin(yaw), w.x * sin(yaw) + w.y * cos(yaw), w.z)


# Meme mapping lineaire que ned_to_godot, applique a un vecteur DIRECTION
# (pas de translation a annuler, juste le changement d'axes).
func ned_dir_to_godot(v_ned: Vector3) -> Vector3:
	return Vector3(v_ned.y, -v_ned.z, -v_ned.x)


func _make_obstacle_material() -> SpatialMaterial:
	var mat = SpatialMaterial.new()
	mat.albedo_color = OBSTACLE_COLOR
	return mat


func load_from_json(path: String) -> void:
	var f = File.new()
	var err = f.open(path, File.READ)
	if err != OK:
		push_error("World.gd: impossible d'ouvrir %s (err=%d)" % [path, err])
		return
	var text = f.get_as_text()
	f.close()

	var parsed = JSON.parse(text)
	if parsed.error != OK:
		push_error("World.gd: JSON invalide dans %s: %s" % [path, parsed.error_string])
		return

	var data = parsed.result
	for obs in data.get("obstacles", []):
		var body = StaticBody.new()
		var shape = CollisionShape.new()
		var mesh_instance = MeshInstance.new()

		if obs["shape"] == "box":
			var box = BoxShape.new()
			var size = obs["size"]  # [x,y,z] pleine largeur, pas demi-extents
			box.extents = Vector3(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
			shape.shape = box

			var box_mesh = CubeMesh.new()
			box_mesh.size = Vector3(size[0], size[1], size[2])
			mesh_instance.mesh = box_mesh
		elif obs["shape"] == "cylinder":
			var cyl = CylinderShape.new()
			cyl.radius = obs["radius"]
			cyl.height = obs["height"]
			shape.shape = cyl

			var cyl_mesh = CylinderMesh.new()
			cyl_mesh.top_radius = obs["radius"]
			cyl_mesh.bottom_radius = obs["radius"]
			cyl_mesh.height = obs["height"]
			mesh_instance.mesh = cyl_mesh
		else:
			push_error("World.gd: forme d'obstacle inconnue: %s" % obs["shape"])
			continue

		mesh_instance.material_override = _make_obstacle_material()
		body.add_child(shape)
		body.add_child(mesh_instance)
		var pos = obs["position"]  # [north, east, down], metres
		body.translation = ned_to_godot(pos[0], pos[1], pos[2])
		add_child(body)


# pos_ned: Vector3(north, east, down), metres. roll/pitch/yaw : attitude
# complete du drone, radians (memes conventions que SIM_STATE). LD06 monte
# rigide sur le chassis -> le disque de mesure suit toute l'attitude, pas
# seulement le cap (cf. discussion roadmap etape 4).
# Retour : tableau de 360 floats, distance_m[i] (index=degre depuis le nez
# en repere CORPS, sens horaire) -- 0.0 = rien detecte dans MAX_RANGE_M.
# Unites metres brutes ici (pas mm/quantifie) : la quantification/le bruit
# sont la responsabilite du bloc [4] (modele capteur), pas de ce bloc [3].
func raycast_360(pos_ned: Vector3, roll: float, pitch: float, yaw: float) -> Array:
	var origin = ned_to_godot(pos_ned.x, pos_ned.y, pos_ned.z)
	var space_state = get_world().direct_space_state
	var distances = []
	for i in range(360):
		var d_body = Vector3(cos(deg2rad(i)), sin(deg2rad(i)), 0.0)
		var dir = ned_dir_to_godot(body_to_ned(d_body, roll, pitch, yaw))
		var to = origin + dir * MAX_RANGE_M
		var result = space_state.intersect_ray(origin, to)
		if result:
			distances.append(origin.distance_to(result.position))
		else:
			distances.append(0.0)
	return distances


# TF Luna : un seul rayon, axe -bas- du repere CORPS (rigide, RNGFND1_ORIENT
# vers le bas cf. INTEGRATION_TEST.md §5.3) -- tourne donc lui aussi avec
# l'attitude complete. Retour : distance_m, 0.0 = rien detecte.
func raycast_down(pos_ned: Vector3, roll: float, pitch: float, yaw: float) -> float:
	var origin = ned_to_godot(pos_ned.x, pos_ned.y, pos_ned.z)
	var space_state = get_world().direct_space_state
	var dir = ned_dir_to_godot(body_to_ned(Vector3(0.0, 0.0, 1.0), roll, pitch, yaw))
	var to = origin + dir * MAX_RANGE_M
	var result = space_state.intersect_ray(origin, to)
	if result:
		return origin.distance_to(result.position)
	return 0.0
