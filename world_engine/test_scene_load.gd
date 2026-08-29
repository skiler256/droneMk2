# world_engine/test_scene_load.gd
# ─────────────────────────────────────────────────────────────────────────────
# Test de fumee headless : WorldViewer.tscn se charge, s'instancie, et
# World.auto_load_json a bien peuple les obstacles -- ne verifie PAS le
# rendu visuel (impossible en headless, aucun GPU), juste que la scene
# reste valide apres modification (avant d'ouvrir l'editeur graphique).
#
# Usage : godot --headless --path world_engine --script test_scene_load.gd

extends SceneTree


func _init():
	var packed = load("res://WorldViewer.tscn")
	if packed == null:
		print("ECHEC: impossible de charger WorldViewer.tscn")
		quit(1)
		return

	var viewer = packed.instance()
	get_root().add_child(viewer)

	var world = viewer.get_node("World")
	var ground = viewer.get_node("Ground")
	var light = viewer.get_node("DirectionalLight")
	var cam = viewer.get_node("Camera")

	print("World enfants (obstacles attendus) : ", world.get_child_count())
	print("Ground present : ", ground != null)
	print("DirectionalLight present : ", light != null)
	print("Camera present, current=", cam.current)

	var ok = true
	if world.get_child_count() == 2:
		print("OK : 2 obstacles charges (box + cylindre, cf. trivial.json)")
	else:
		print("ECHEC : nombre d'obstacles inattendu")
		ok = false

	# Base miroir (determinant negatif) -> cull_mode Back elimine TOUTES les
	# faces avant -> ecran gris, quel que soit l'angle vise (bug reel trouve
	# en session, cf. WorldViewer.tscn/Viewer.tscn). Aucune camera commitee
	# ne doit repasser par la, headless ne peut pas le voir a l'oeil.
	var det = cam.global_transform.basis.determinant()
	print("Camera basis det=", det, " (attendu ~+1)")
	if det < 0.99:
		print("ECHEC : base camera miroir/gauchie -> tout sera backface-culled")
		ok = false

	quit(0 if ok else 1)
