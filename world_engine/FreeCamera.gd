# world_engine/FreeCamera.gd
# ─────────────────────────────────────────────────────────────────────────────
# Navigation libre pour la camera principale de Viewer.gd (bloc [3], viewer
# optionnel/debug) -- clic droit maintenu + glisser souris pour regarder,
# ZQSD pour se deplacer. Meme schema que la camera de navigation de
# l'editeur Godot (deja connu), volontairement PAS de capture souris
# permanente (MOUSE_MODE_CAPTURED) -- reste utilisable au pavé tactile,
# pas besoin d'une souris physique pour looker autour.
#
# ZQSD (AZERTY) lu par scancode direct (Input.is_key_pressed(KEY_*)), pas
# par InputMap -- evite d'avoir a configurer un remap de layout clavier.

extends Camera

const MOVE_SPEED = 8.0          # m/s
const BOOST_MULT = 3.0          # multiplicateur avec Shift maintenu
const LOOK_SENSITIVITY = 0.005  # rad par pixel de deplacement souris
const PITCH_LIMIT = 1.4         # ~80 deg, evite le blocage a la verticale stricte

var _looking = false
var _yaw = 0.0
var _pitch = 0.0


func _ready():
	# Derive yaw/pitch de l'orientation deja posee par Viewer.gd (look_at()
	# dans Viewer::_ready(), execute avant celui-ci puisque Viewer est le
	# parent) -- pour ne pas "sauter" au premier mouvement de souris.
	var fwd = -global_transform.basis.z
	_yaw = atan2(-fwd.x, -fwd.z)
	_pitch = clamp(asin(clamp(fwd.y, -1.0, 1.0)), -PITCH_LIMIT, PITCH_LIMIT)


func _input(event):
	if event is InputEventMouseButton and event.button_index == BUTTON_RIGHT:
		_looking = event.pressed
	elif event is InputEventMouseMotion and _looking:
		_yaw -= event.relative.x * LOOK_SENSITIVITY
		_pitch = clamp(_pitch - event.relative.y * LOOK_SENSITIVITY, -PITCH_LIMIT, PITCH_LIMIT)
		rotation = Vector3(_pitch, _yaw, 0.0)


func _process(delta):
	var dir = Vector3.ZERO
	if Input.is_key_pressed(KEY_Z):
		dir -= transform.basis.z
	if Input.is_key_pressed(KEY_S):
		dir += transform.basis.z
	if Input.is_key_pressed(KEY_Q):
		dir -= transform.basis.x
	if Input.is_key_pressed(KEY_D):
		dir += transform.basis.x
	if Input.is_key_pressed(KEY_SPACE):
		dir += Vector3.UP
	if Input.is_key_pressed(KEY_CONTROL):
		dir -= Vector3.UP

	if dir.length() > 0.0:
		var speed = MOVE_SPEED
		if Input.is_key_pressed(KEY_SHIFT):
			speed *= BOOST_MULT
		translation += dir.normalized() * speed * delta
