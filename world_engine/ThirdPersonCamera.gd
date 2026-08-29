# world_engine/ThirdPersonCamera.gd
# ─────────────────────────────────────────────────────────────────────────────
# Vue troisieme personne, suit la position du drone (DroneMarker) en continu
# -- SEULE la rotation autour du drone est geree a la souris (clic droit
# maintenu + glisser), pas de deplacement libre comme FreeCamera.gd, pas de
# suivi de l'attitude du drone non plus (orbite independante, sinon un roll
# du drone ferait tourner betement toute la vue avec lui).
#
# transform reconstruite integralement chaque frame via look_at() a partir
# de Transform.IDENTITY -- jamais de transform heritee d'un etat precedent
# (cf. bug de base miroir trouve en session : look_at() peut heriter d'une
# base existante mal formee, cf. Viewer.gd::_ready()).

extends Camera

const ROTATE_SENSITIVITY = 0.005
const PITCH_LIMIT = 1.4     # ~80 deg, evite le blocage a la verticale stricte
const DEFAULT_PITCH = 0.3   # leger angle plongeant par defaut
const DEFAULT_DISTANCE = 8.0  # metres
const MIN_DISTANCE = 2.0
const MAX_DISTANCE = 40.0
const ZOOM_STEP = 1.0        # metres par cran de molette

var target: Spatial  # assigne par Viewer.gd apres instanciation (DroneMarker)
var height_offset := 0.3  # assigne par Viewer.gd (third_person_height_offset)

var _dragging = false
var _yaw = 0.0
var _pitch = DEFAULT_PITCH
var _distance = DEFAULT_DISTANCE


func _input(event):
	if event is InputEventMouseButton and event.button_index == BUTTON_RIGHT:
		_dragging = event.pressed
	elif event is InputEventMouseMotion and _dragging:
		# Sens inverse du deplacement souris (glisser vers la droite fait
		# tourner la vue comme si on tirait le monde vers la droite, pas la
		# camera -- convention orbit standard, cf. retour session).
		_yaw += event.relative.x * ROTATE_SENSITIVITY
		_pitch = clamp(_pitch + event.relative.y * ROTATE_SENSITIVITY, -PITCH_LIMIT, PITCH_LIMIT)
	elif event is InputEventMouseButton and event.pressed:
		if event.button_index == BUTTON_WHEEL_UP:
			_distance = clamp(_distance - ZOOM_STEP, MIN_DISTANCE, MAX_DISTANCE)
		elif event.button_index == BUTTON_WHEEL_DOWN:
			_distance = clamp(_distance + ZOOM_STEP, MIN_DISTANCE, MAX_DISTANCE)


func _process(_delta):
	if target == null:
		return

	var center = target.global_transform.origin + Vector3(0.0, height_offset, 0.0)
	var offset = Vector3(
		_distance * cos(_pitch) * sin(_yaw),
		_distance * sin(_pitch),
		_distance * cos(_pitch) * cos(_yaw)
	)

	transform = Transform.IDENTITY
	translation = center + offset
	look_at(center, Vector3.UP)
