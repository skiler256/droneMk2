# world_engine/test_viewer.gd
# ─────────────────────────────────────────────────────────────────────────────
# Test en isolation de Viewer.gd::pose_to_transform() (geometrie pure, sans
# reseau) -- verifie que le marqueur visuel s'oriente/se positionne
# correctement pour une pose donnee, contre des valeurs calculees a la main.
#
# Usage : godot --headless --path world_engine --script test_viewer.gd

extends SceneTree


func _init():
	var packed = load("res://Viewer.tscn")
	var viewer = packed.instance()
	get_root().add_child(viewer)  # _ready() s'execute (pas de --world -> juste un push_error, sans consequence ici)

	# Niveau, yaw=0, 2m au-dessus du sol a (nord=5, est=0) : origine attendue
	# Godot (est=0, haut=2, -nord=-5) ; nez pointe plein nord -> Godot -Z ;
	# "haut" du corps -> Godot +Y (aucune inclinaison).
	var t1 = viewer.pose_to_transform(Vector3(5.0, 0.0, -2.0), 0.0, 0.0, 0.0)
	print("niveau, yaw=0 : origin=", t1.origin, " (attendu (0,2,-5))")
	print("  nez (basis.x)=", t1.basis.x, " (attendu ~(0,0,-1), plein nord)")
	print("  haut (basis.y)=", t1.basis.y, " (attendu ~(0,1,0))")

	# Yaw=90 (cap est) : le nez doit pointer vers Godot +X (est).
	var t2 = viewer.pose_to_transform(Vector3(0.0, 0.0, -2.0), 0.0, 0.0, deg2rad(90))
	print("yaw=90 : nez (basis.x)=", t2.basis.x, " (attendu ~(1,0,0), plein est)")

	# Roll=90 : le "haut" du corps doit pointer a l'horizontale (aile qui
	# bascule), pas vers Godot +Y.
	var t3 = viewer.pose_to_transform(Vector3(0.0, 0.0, -2.0), deg2rad(90), 0.0, 0.0)
	print("roll=90 : haut (basis.y)=", t3.basis.y, " (attendu horizontal, y proche de 0)")

	# Base miroir (determinant negatif) -> cull_mode Back elimine TOUTES les
	# faces avant -> ecran gris (bug reel trouve en session : look_at() dans
	# _ready() HERITE de la base existante du Camera au lieu de repartir
	# propre -- si le .tscn committe une transform miroir, ca survivait au
	# reset. Cf. Viewer.gd::_ready(), reset explicite en Transform.IDENTITY).
	var ok = true
	var cam = viewer.get_node("Camera")
	var det = cam.global_transform.basis.determinant()
	print("Camera basis det (apres _ready() reel)=", det, " (attendu ~+1)")
	if det < 0.99:
		print("ECHEC : base camera miroir/gauchie -> tout sera backface-culled")
		ok = false

	quit(0 if ok else 1)
