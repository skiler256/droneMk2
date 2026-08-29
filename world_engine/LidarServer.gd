# world_engine/LidarServer.gd
# ─────────────────────────────────────────────────────────────────────────────
# Serveur headless persistant du bloc [3] (INTEGRATION_TEST.md §3.3) :
# charge un monde JSON au demarrage et repond en UDP a des requetes de
# raycast (pose -> distances), une par tick reseau -- lance/tue par
# procman.py comme ArduPilot SITL, une instance par scenario.
#
# Protocole (prive, jamais vu par le systeme sous test -- pas le meme port
# ni le meme format que lidar_wire.py/UdpLidarDriver) :
#
# Requete (bridge -> serveur), petit-boutiste, 34 octets :
#   [magic:4]["RCQ1"][version:1][kind:1][seq:4]
#   [north:f32][east:f32][down:f32][roll:f32][pitch:f32][yaw:f32]
#   kind : 0 = LIDAR360 (LD06, disque complet), 1 = TFLUNA (rayon bas seul)
#   pose en metres/radians, memes conventions que SIM_STATE.
#
# Reponse (serveur -> bridge), petit-boutiste :
#   [magic:4]["RCR1"][version:1][kind:1][seq:4][donnees]
#   kind LIDAR360 : 360 * f32 (distance_m[i], index=degre depuis le nez
#                   corps, sens horaire, 0.0 = rien detecte)
#   kind TFLUNA   : 1 * f32 (distance_m, 0.0 = rien detecte)
#
# Distances en METRES bruts (pas mm/quantifie, pas de bruit) : la
# quantification/le bruit/latence/dropout sont la responsabilite du bloc [4]
# (modele capteur, cote bridge Python), interdits ici (cf. World.gd, §3.3).
#
# Usage : godot --headless --path world_engine --script LidarServer.gd --
#         --world=/chemin/absolu/vers/monde.json [--port=15102]

extends SceneTree

const MAGIC_REQ := PoolByteArray([82, 67, 81, 49])  # "RCQ1"
const MAGIC_REP := PoolByteArray([82, 67, 82, 49])  # "RCR1"
const VERSION := 1
const KIND_LIDAR360 := 0
const KIND_TFLUNA := 1
const REQUEST_SIZE := 34
const DEFAULT_PORT := 15102

var world  # World.gd instance
var socket := PacketPeerUDP.new()


func _init():
	var world_path := ""
	var port := DEFAULT_PORT
	for arg in OS.get_cmdline_args():
		if arg.begins_with("--world="):
			world_path = arg.substr(len("--world="))
		elif arg.begins_with("--port="):
			port = int(arg.substr(len("--port=")))

	if world_path == "":
		push_error("LidarServer: --world=<chemin> obligatoire (monde requis, cf. scenario.py)")
		quit(1)
		return

	world = load("res://World.gd").new()
	get_root().add_child(world)
	world.load_from_json(world_path)

	var err = socket.listen(port)
	if err != OK:
		push_error("LidarServer: echec listen UDP port %d (err=%d)" % [port, err])
		quit(1)
		return

	print("LidarServer: pret, port=%d, monde=%s" % [port, world_path])


func _idle(_delta):
	while socket.get_available_packet_count() > 0:
		_handle_one_packet()
	return false  # false = continuer a tourner


func _handle_one_packet():
	var packet := socket.get_packet()
	var sender_ip := socket.get_packet_ip()
	var sender_port := socket.get_packet_port()

	if packet.size() != REQUEST_SIZE or packet.subarray(0, 3) != MAGIC_REQ:
		push_warning("LidarServer: paquet invalide ignore (taille=%d)" % packet.size())
		return

	var buf := StreamPeerBuffer.new()
	buf.big_endian = false
	buf.data_array = packet
	buf.seek(4)  # apres le magic
	var version := buf.get_u8()
	var kind := buf.get_u8()
	var seq := buf.get_u32()
	if version != VERSION:
		push_warning("LidarServer: version requete inattendue (%d)" % version)
		return

	var north := buf.get_float()
	var east := buf.get_float()
	var down := buf.get_float()
	var roll := buf.get_float()
	var pitch := buf.get_float()
	var yaw := buf.get_float()
	var pos_ned := Vector3(north, east, down)

	var reply := StreamPeerBuffer.new()
	reply.big_endian = false
	reply.put_data(MAGIC_REP)
	reply.put_u8(VERSION)
	reply.put_u8(kind)
	reply.put_u32(seq)

	if kind == KIND_LIDAR360:
		for d in world.raycast_360(pos_ned, roll, pitch, yaw):
			reply.put_float(d)
	elif kind == KIND_TFLUNA:
		reply.put_float(world.raycast_down(pos_ned, roll, pitch, yaw))
	else:
		push_warning("LidarServer: kind inconnu (%d)" % kind)
		return

	socket.set_dest_address(sender_ip, sender_port)
	socket.put_packet(reply.data_array)
