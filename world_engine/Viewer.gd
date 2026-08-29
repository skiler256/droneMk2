# world_engine/Viewer.gd
# ─────────────────────────────────────────────────────────────────────────────
# Viewer temps reel du bloc [3] -- DECOUPLE de LidarServer.gd (process Godot
# separe, non-headless) : jamais de rendu sur le chemin critique du raycast
# (cf. INTEGRATION_TEST.md risque connu #2, "un stutter de frame ressemble a
# une panne capteur"). Charge un monde JSON au demarrage, deplace un marqueur
# visuel (DroneMarker) selon les trames de pose "fire-and-forget" recues de
# sensor_bridge.py (cf. sitl/raycast_wire.py::encode_pose(), PAS le meme
# protocole/port que LidarServer.gd -- pas de requete/reponse ici).
#
# Optionnel/debug : rien dans le pipeline critique ne depend de ce process.
# S'il n'est pas lance, sensor_bridge.py envoie ses trames de pose dans le
# vide (personne n'ecoute), sans effet.
#
# Navigation camera libre (FreeCamera.gd) : clic droit maintenu + souris
# pour regarder, ZQSD pour se deplacer, espace/ctrl haut/bas, shift = vite.
# Touche C : cycle entre 3 vues -- libre, embarquee (DroneCam), suivi 3e
# personne (ThirdPersonCam, orbite souris autour du drone, cf.
# ThirdPersonCamera.gd). Modele 3D du drone et camera embarquee : cf.
# drone_model_path/DRONE_CAM_OFFSET/DRONE_CAM_PITCH_DEG ci-dessous.
#
# Usage : godot --path world_engine res://Viewer.tscn -- \
#         --world=/chemin/absolu/vers/monde.json [--pose-port=15103]
# (scene explicite necessaire -- project.godot pointe par defaut sur
# WorldViewer.tscn, cf. §9 SITL.md. Pas de --script : une vraie scene avec
# son MainLoop normal. --headless marche quand meme pour verifier la
# logique sans GPU/fenetre, cf. test_viewer.gd -- enlever --headless pour
# un vrai visuel, WSLg forward la fenetre automatiquement)

extends Spatial

const MAGIC_POSE := PoolByteArray([80, 79, 83, 49])  # "POS1"
const VERSION := 1
const DEFAULT_POSE_PORT := 15103

# H -- modele 3D du drone (cf. world_engine/models/README.md). Vide = garde
# le marqueur cube/nez de secours (Body/Nose dans Viewer.tscn).
export(String, FILE, "*.glb,*.gltf") var drone_model_path := ""
# Correctif d'axe si le nez du modele importe ne pointe pas dans le bon
# sens (convention d'export Blender/glTF pas toujours alignee avec le +X
# "nez" de DroneMarker) -- ajuster par pas de 90 en regardant le rendu,
# ce n'est pas un signe d'echec d'import.
export(float) var drone_model_yaw_offset_deg := -90.0
# Hauteur (metres, positif) du CG au sol -- DroneMarker represente la
# position rapportee par la simulation (repere IMU/CG), PAS le contact au
# sol, alors que l'origine du modele (cf. convention Blender : origine en Z
# au sol, X/Y au centre) est au niveau des pieds. Sans ce decalage, le
# "sol" du modele se retrouve colle sur le CG -> tout le corps flotte de
# cette hauteur. A regler a l'oeil (hauteur reelle du train d'atterrissage),
# je ne peux pas la deviner.
export(float) var drone_model_ground_offset_m := 0.0

# I -- camera embarquee, position/inclinaison de la vraie camera physique
# du drone. Constante modifiable ici (cf. demande session) : repere LOCAL
# de DroneMarker (x=avant/nez, y=haut, z=troisieme axe), PAS le repere NED.
const DRONE_CAM_OFFSET := Vector3(0.15, -0.05, 0.0)  # 15cm avant, 5cm sous le centre
const DRONE_CAM_PITCH_DEG := 0.0  # inclinaison vers le bas depuis l'horizontale

# Vue 3e personne : hauteur du point vise au-dessus de DroneMarker -- 1.0
# etait calibre sur le petit cube placeholder (0.4m), pas sur l'echelle
# reelle d'un modele importe. A regler a l'oeil aussi.
export(float) var third_person_height_offset := 0.3

onready var world = $World
onready var drone_marker = $DroneMarker
onready var camera = $Camera

var drone_cam: Camera
var third_person_cam: Camera
var _cameras: Array
var _camera_index := 0
var socket := PacketPeerUDP.new()


func _ready():
	# Cadrage par script (look_at). BUG REEL trouve en session (verifie
	# empiriquement, pas suppose) : look_at() HERITE de la base existante
	# du noeud au lieu de toujours repartir propre -- si le Camera du
	# .tscn a ete laisse avec une transform "miroir" (determinant -1, ex:
	# manipulation accidentelle du gizmo dans l'editeur), look_at() garde
	# ce miroir. Consequence concrete : cull_mode Back (defaut de
	# SpatialMaterial) elimine alors TOUTES les faces avant -> ecran gris,
	# quel que soit l'angle vise. D'ou le reset explicite en Transform.IDENTITY
	# avant translation/look_at, pour ne jamais dependre de ce qui traine
	# dans le fichier .tscn.
	camera.transform = Transform.IDENTITY
	camera.translation = Vector3(-20.0, 20.0, 25.0)
	camera.look_at(Vector3(0.0, 3.0, -15.0), Vector3.UP)

	_setup_drone_visual()
	_setup_drone_camera()
	_setup_third_person_camera()
	_cameras = [camera, drone_cam, third_person_cam]

	var world_path := ""
	var port := DEFAULT_POSE_PORT
	for arg in OS.get_cmdline_args():
		if arg.begins_with("--world="):
			world_path = arg.substr(len("--world="))
		elif arg.begins_with("--pose-port="):
			port = int(arg.substr(len("--pose-port=")))

	if world_path == "":
		push_error("Viewer: --world=<chemin> obligatoire (monde requis, cf. scenario.py)")
		return

	world.load_from_json(world_path)
	for c in world.get_children():
		print(c.name, " type=", c.get_class(), " visual=", c is VisualInstance)

	var err = socket.listen(port)
	if err != OK:
		push_error("Viewer: echec listen UDP port %d (err=%d)" % [port, err])
		return

	print("Viewer: pret, port=%d, monde=%s" % [port, world_path])


# H -- remplace le marqueur cube/nez par le modele importe si
# drone_model_path est renseigne et se charge correctement -- sinon garde
# le placeholder tel quel (aucune erreur, comportement de secours normal).
func _setup_drone_visual():
	if drone_model_path == "":
		return
	var packed = load(drone_model_path)
	if packed == null:
		push_warning("Viewer: impossible de charger %s, marqueur cube conserve" % drone_model_path)
		return
	var model = packed.instance()
	drone_marker.get_node("Body").visible = false
	drone_marker.get_node("Nose").visible = false
	model.rotation_degrees.y = drone_model_yaw_offset_deg
	model.translation.y = -drone_model_ground_offset_m
	drone_marker.add_child(model)


# I -- camera embarquee, position de la vraie camera physique du drone.
# Pas "current" par defaut (la camera libre reste la vue principale) --
# bascule avec la touche C (cf. _input()). Position/inclinaison EN DUR dans
# DRONE_CAM_OFFSET/DRONE_CAM_PITCH_DEG (repere local DroneMarker), construite
# a la main (pas via look_at(), cf. bug de base miroir trouve en session --
# look_at() herite de la base existante d'un noeud) et verifiee
# empiriquement (determinant +1, direction correcte).
func _setup_drone_camera():
	var forward_dir = Vector3(cos(deg2rad(DRONE_CAM_PITCH_DEG)), -sin(deg2rad(DRONE_CAM_PITCH_DEG)), 0.0)
	var up_ref = Vector3(0.0, 1.0, 0.0)
	var z_axis = -forward_dir
	var x_axis = up_ref.cross(z_axis).normalized()
	var y_axis = z_axis.cross(x_axis)

	drone_cam = Camera.new()
	drone_cam.name = "DroneCam"
	drone_cam.transform = Transform(Basis(x_axis, y_axis, z_axis), DRONE_CAM_OFFSET)
	drone_marker.add_child(drone_cam)


# Troisieme vue : suivi 3e personne, orbite a la souris (ThirdPersonCamera.gd)
# -- PAS enfant de DroneMarker (l'orbite doit rester independante de
# l'attitude du drone, sinon un roll ferait tourner betement toute la vue).
func _setup_third_person_camera():
	third_person_cam = Camera.new()
	third_person_cam.name = "ThirdPersonCam"
	third_person_cam.set_script(load("res://ThirdPersonCamera.gd"))
	add_child(third_person_cam)
	third_person_cam.target = drone_marker
	third_person_cam.height_offset = third_person_height_offset


# Cycle entre les 3 vues (libre -> embarquee -> suivi 3e personne -> libre...)
# avec la touche C. make_current() desactive automatiquement les autres
# cameras du viewport (pas besoin de gerer les flags "current" a la main).
func _input(event):
	if event is InputEventKey and event.pressed and not event.echo and event.scancode == KEY_C:
		_camera_index = (_camera_index + 1) % _cameras.size()
		_cameras[_camera_index].make_current()


# Pose (NED metres/radians) -> Transform du marqueur dans le repere Godot.
# Fonction pure, testable independamment de la reception reseau (cf.
# test_viewer_pose.gd) -- meme rotation aeronautique que World.gd
# (body_to_ned), appliquee cette fois a l'ORIENTATION du marqueur entier,
# pas a un seul rayon.
func pose_to_transform(pos_ned: Vector3, roll: float, pitch: float, yaw: float) -> Transform:
	var origin = world.ned_to_godot(pos_ned.x, pos_ned.y, pos_ned.z)
	var fwd = world.ned_dir_to_godot(world.body_to_ned(Vector3(1.0, 0.0, 0.0), roll, pitch, yaw))
	var up = world.ned_dir_to_godot(world.body_to_ned(Vector3(0.0, 0.0, -1.0), roll, pitch, yaw))
	var basis = Basis(fwd, up, fwd.cross(up)).orthonormalized()
	return Transform(basis, origin)


func _process(_delta):
	while socket.get_available_packet_count() > 0:
		_handle_one_packet()


func _handle_one_packet():
	var packet := socket.get_packet()
	if packet.size() != 29 or packet.subarray(0, 3) != MAGIC_POSE:
		return  # trame invalide ignoree -- pas de reponse a envoyer (fire-and-forget)

	var buf := StreamPeerBuffer.new()
	buf.big_endian = false
	buf.data_array = packet
	buf.seek(4)
	var version := buf.get_u8()
	if version != VERSION:
		return

	var north := buf.get_float()
	var east := buf.get_float()
	var down := buf.get_float()
	var roll := buf.get_float()
	var pitch := buf.get_float()
	var yaw := buf.get_float()

	drone_marker.transform = pose_to_transform(Vector3(north, east, down), roll, pitch, yaw)
