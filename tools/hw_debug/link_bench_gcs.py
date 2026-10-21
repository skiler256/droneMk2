#!/usr/bin/env python3
"""Station sol du banc de liaison. Repond sur un ou plusieurs liens a la fois.

    python3 link_bench_gcs.py cc1101          # lien secondaire SDL
    python3 link_bench_gcs.py wfb             # lien principal PDL
    python3 link_bench_gcs.py cc1101,wfb      # LES DEUX en meme temps
    python3 link_bench_gcs.py udp             # bouclage local, sans radio

Mesurer les deux liens l'un apres l'autre ne dit rien de ce qui se passe
quand ils fonctionnent ensemble. Les deux radios sont a quelques centimetres
l'une de l'autre sur le drone, et c'est en vol qu'elles cohabitent. Une
mesure cote a cote est aussi la seule facon de comparer honnetement un lien
de secours a un lien principal.

Ce script est le repondeur des DEUX transports :

  cc1101  la radio est sur l'ESP32, dont le firmware de production n'est
          qu'un pont : il relaie en RX:<hex> et emet ce qu'on lui donne en
          TX:<hex>. L'echo se fait donc ICI.

          ATTENTION : l'aller-retour mesure inclut alors le detour par le
          port serie, quelques millisecondes de plus qu'un echo fait dans le
          firmware. En echange c'est le chemin de PRODUCTION qui est mesure.

  wfb     la radio est un adaptateur wifi pilote par wfb-ng, qui livre les
          trames sur un port UDP local. Ce script repond directement.

  udp     meme chose sans radio : les deux bouts sur la MEME machine.

Dans tous les cas le drone mesure la latence avec sa seule horloge : on
renvoie l'horodatage tel quel, en y inscrivant la qualite vue du sol.
"""
import argparse
import os
import select
import signal
import socket
import struct
import subprocess
import sys
import time

# Disposition de la trame — doit rester identique a BenchFrame dans
# src/HwBench/bench_link.cpp. Petit-boutien, offsets fixes.
ENTETE = struct.Struct("<HBBIQbBBB")
TAILLE_ENTETE = ENTETE.size            # 20
MAGIC = 0x4B4C
VERSION = 1
FAR_VALID = 0x01
# Modes sans echo : le repondeur compte sans renvoyer. En saturation parce
# que l'echo doublerait le trafic sur un lien semi-duplex ; en flux parce que
# la question posee est « combien arrivent a cette cadence », et un echo
# faussserait la mesure en occupant l'air.
MODES_SANS_ECHO = (2, 5)
MODES = {1: "ping", 2: "flood", 3: "range", 4: "ramp", 5: "stream"}

# En udp les deux bouts tournent sur la meme machine : 15611/15612 pour ne
# pas tomber sur 5602/5612, qu'occupe le serveur du dashboard. Ces valeurs
# doivent rester identiques aux defauts de bench_link.cpp.
PORTS_UDP = (15612, 15611)


def _port_udp_de(process):
    """Lit l'option -u / --udp-port de la ligne de commande d'un process."""
    try:
        sortie = subprocess.run(["pgrep", "-a", process], capture_output=True,
                                text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for ligne in sortie.splitlines():
        mots = ligne.split()
        for i, m in enumerate(mots):
            if m in ("-u", "--udp-port") and i + 1 < len(mots):
                try:
                    return int(mots[i + 1])
                except ValueError:
                    pass
    return None


def ports_wfb():
    """Deduit les deux ports UDP de wfb-ng de ses process.

    Renvoie (ecoute, renvoi), chacun None s'il est introuvable.

      wfb_rx -u <port>   la ou il LIVRE les trames recues -> on y ecoute
      wfb_tx -u <port>   la ou il LIT ce qu'il doit emettre -> on y renvoie

    Ne JAMAIS inventer ces valeurs : un port plausible mais faux donne une
    mesure a 100 % de perte sans que rien n'indique que le probleme vient
    des ports.
    """
    return _port_udp_de("wfb_rx"), _port_udp_de("wfb_tx")


def qui_tient(peripherique):
    """Processus qui gardent un peripherique ouvert, par son nom de fichier.

    Linux autorise plusieurs ouvertures d'un tty : une lecture concurrente
    « marche » tout en volant des octets a l'autre lecteur. C'est ce qui
    rendait les premieres captures series erratiques, d'ou ce controle.
    """
    porteurs = []
    for entree in os.listdir("/proc"):
        if not entree.isdigit():
            continue
        try:
            for fd in os.listdir(f"/proc/{entree}/fd"):
                if os.readlink(f"/proc/{entree}/fd/{fd}") == peripherique:
                    with open(f"/proc/{entree}/cmdline", "rb") as f:
                        cmd = f.read().replace(b"\0", b" ").decode().strip()
                    porteurs.append((int(entree), cmd))
                    break
        except (OSError, PermissionError):
            continue
    return porteurs


def trouver_port_serie(demande):
    if demande:
        return demande
    for motif in ("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"):
        if os.path.exists(motif):
            return motif
    return None


def liberer_port(peripherique, arreter):
    """Signale ou resout un conflit sur le port serie. Renvoie False si bloque."""
    porteurs = qui_tient(peripherique)
    if not porteurs:
        return True

    print(f"  ATTENTION : {peripherique} est deja ouvert par :")
    for pid, cmd in porteurs:
        print(f"    PID {pid}  {cmd}")

    serveur = any("server.js" in cmd for _, cmd in porteurs)
    if not serveur:
        print("  Lecture concurrente : les octets seront partages entre les")
        print("  deux lecteurs et la mesure sera fausse. Arreter l'autre.")
        return False

    if not arreter:
        print("\n  C'est le serveur du dashboard. Deux options :")
        print("    - relancer avec --stop-server pour que je l'arrete ;")
        print("    - ou l'arreter soi-meme :")
        print("        sudo systemctl stop gcs-server.service")
        return False

    print("\n  arret de gcs-server.service ...")
    r = subprocess.run(["sudo", "systemctl", "stop", "gcs-server.service"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"  echec : {r.stderr.strip()}")
        print("  -> l'arreter a la main : sudo systemctl stop gcs-server.service")
        return False
    time.sleep(1.5)
    if qui_tient(peripherique):
        print("  le port est toujours tenu.")
        return False
    print("  arrete. Il ne sera PAS relance automatiquement.")
    return True


def entete_commune(lien, role, detail, commande, sur_le_drone=True):
    print("═" * 64)
    print("  banc de liaison — station sol")
    print("═" * 64)
    print(f"  lien   : {lien}")
    print(f"  role   : {role}")
    print(f"  {detail}")
    print()
    if sur_le_drone:
        print("  À lancer maintenant sur le drone :")
    else:
        # Le lien udp est en bouclage (INADDR_LOOPBACK) : les deux bouts sont
        # forcement sur la meme machine.
        print("  À lancer maintenant sur CETTE machine (bouclage local) :")
    print(f"    {commande}")
    print()
    print("  Ctrl-C pour arreter et afficher le bilan.")
    print("─" * 64)
    sys.stdout.flush()


# ─────────────────────────── repondeur UDP ───────────────────────────

class CompteurUdp:
    def __init__(self):
        self.recues = 0
        self.octets = 0
        self.hors_banc = 0
        self.desordonnees = 0
        self.seq_max = -1
        self.seq_vues = set()
        self.t0 = None
        self.t_dernier = None
        self.ecarts = []
        self.par_mode = {}
        # fenetre de la seconde en cours
        self.f_recues = 0
        self.f_octets = 0

    def ajouter(self, seq, taille, mode):
        t = time.monotonic()
        if self.t0 is None:
            self.t0 = t
        elif self.t_dernier is not None:
            self.ecarts.append((t - self.t_dernier) * 1000.0)
        self.t_dernier = t
        self.recues += 1
        self.octets += taille
        self.f_recues += 1
        self.f_octets += taille
        self.par_mode[mode] = self.par_mode.get(mode, 0) + 1
        if seq < self.seq_max:
            self.desordonnees += 1
        self.seq_max = max(self.seq_max, seq)
        self.seq_vues.add(seq)

    def ligne_directe(self, ecoule):
        attendues = self.seq_max + 1
        perte = (100.0 * (attendues - len(self.seq_vues)) / attendues
                 if attendues > 0 else 0.0)
        debit = self.f_octets * 8.0 / 1000.0
        print(f"[{ecoule:4.0f}s] recues {self.recues:<8d} perte {perte:5.1f} %  "
              f"{debit:7.1f} kbit/s  {self.f_recues:5d} trames/s", flush=True)
        self.f_recues = 0
        self.f_octets = 0

    def bilan(self):
        print("─" * 64)
        print(f"  trames recues     : {self.recues}")
        if self.hors_banc:
            print(f"  hors banc ignorees: {self.hors_banc}  "
                  "(telemetrie reelle ou bruit)")
        if self.recues == 0:
            print("\n  Rien n'est arrive. A verifier, dans cet ordre :")
            print("    - le banc tourne-t-il sur le drone ?")
            print("    - les ports correspondent-ils des deux cotes ?")
            print("    - pour wfb : wfb_tx et wfb_rx tournent-ils ?")
            return
        # Le drone numerote sans trou, et seulement apres un envoi reussi :
        # le plus grand numero vu donne donc le nombre reellement emis.
        attendues = self.seq_max + 1
        manquantes = attendues - len(self.seq_vues)
        print(f"  emises (d'apres les numeros) : {attendues}")
        print(f"  manquantes        : {manquantes}   "
              f"perte : {100.0 * manquantes / attendues:.1f} %")
        print(f"  desordonnees      : {self.desordonnees}   "
              f"doublons : {self.recues - len(self.seq_vues)}")
        duree = (self.t_dernier - self.t0) if self.t0 else 0.0
        if duree > 0:
            print(f"  duree             : {duree:.1f} s")
            print(f"  debit utile recu  : {self.octets * 8.0 / duree / 1000.0:.1f}"
                  f" kbit/s   ({self.recues / duree:.1f} trames/s)")
        if self.ecarts:
            tri = sorted(self.ecarts)
            p95 = tri[min(int(0.95 * (len(tri) - 1) + 0.5), len(tri) - 1)]
            print(f"  inter-arrivee (ms): min {tri[0]:.1f}   "
                  f"moy {sum(tri) / len(tri):.1f}   p95 {p95:.1f}   "
                  f"max {tri[-1]:.1f}")
        if self.par_mode:
            print("  par mode          : " + "  ".join(
                f"{MODES.get(m, m)}:{n}" for m, n in sorted(self.par_mode.items())))


class VoieSol:
    """Statistiques de reception d'un lien, cote sol."""

    def __init__(self, nom):
        self.nom = nom
        self.recues = 0
        self.octets = 0
        self.hors_banc = 0
        self.desordonnees = 0
        self.seq_max = -1
        self.seq_vues = set()
        self.t0 = None
        self.t_dernier = None
        self.ecarts = []
        self.rssi = None
        self.lqi = None
        self.f_recues = 0
        self.f_octets = 0

    def ajouter(self, seq, taille, rssi=None, lqi=None):
        t = time.monotonic()
        if self.t0 is None:
            self.t0 = t
        elif self.t_dernier is not None:
            self.ecarts.append((t - self.t_dernier) * 1000.0)
        self.t_dernier = t
        self.recues += 1
        self.octets += taille
        self.f_recues += 1
        self.f_octets += taille
        if rssi is not None:
            self.rssi, self.lqi = rssi, lqi
        if seq < self.seq_max:
            self.desordonnees += 1
        self.seq_max = max(self.seq_max, seq)
        self.seq_vues.add(seq)

    def ligne(self, ecoule):
        attendues = self.seq_max + 1
        perte = (100.0 * (attendues - len(self.seq_vues)) / attendues
                 if attendues > 0 else 0.0)
        q = f"  {self.rssi:4d} dBm" if self.rssi is not None else ""
        print(f"[{ecoule:4.0f}s] {self.nom:7s} recues {self.recues:<7d} "
              f"perte {perte:5.1f} %  {self.f_octets * 8 / 1000.0:7.1f} kbit/s"
              f"{q}", flush=True)
        self.f_recues = 0
        self.f_octets = 0

    def bilan(self):
        print(f"\n════ {self.nom} ════")
        print(f"  trames recues     : {self.recues}")
        if self.hors_banc:
            print(f"  hors banc ignorees: {self.hors_banc}")
        if self.recues == 0:
            print("  Rien n'est arrive. Le banc tourne-t-il sur le drone avec")
            print("  ce lien dans --link ? Les ports correspondent-ils ?")
            return
        attendues = self.seq_max + 1
        manquantes = attendues - len(self.seq_vues)
        print(f"  emises (numeros)  : {attendues}")
        print(f"  manquantes        : {manquantes}   "
              f"perte : {100.0 * manquantes / attendues:.1f} %")
        print(f"  desordonnees      : {self.desordonnees}   "
              f"doublons : {self.recues - len(self.seq_vues)}")
        duree = (self.t_dernier - self.t0) if self.t0 else 0.0
        if duree > 0:
            print(f"  debit utile recu  : "
                  f"{self.octets * 8.0 / duree / 1000.0:.1f} kbit/s "
                  f"({self.recues / duree:.1f} trames/s)")
        if self.rssi is not None:
            print(f"  qualite vue d'ici : rssi {self.rssi} dBm   lqi {self.lqi}")
        if self.ecarts:
            tri = sorted(self.ecarts)
            p95 = tri[min(int(0.95 * (len(tri) - 1) + 0.5), len(tri) - 1)]
            print(f"  inter-arrivee (ms): min {tri[0]:.1f}   "
                  f"moy {sum(tri) / len(tri):.1f}   p95 {p95:.1f}   "
                  f"max {tri[-1]:.1f}")


def _echo(data, rssi, lqi):
    """Renvoie (seq, mode, trame_a_renvoyer). trame None si le mode interdit
    l'echo (saturation), et None tout court si ce n'est pas une trame de banc."""
    if len(data) < TAILLE_ENTETE:
        return None
    magic, version, mode, seq, tx_ns, _, _, _, _ = ENTETE.unpack_from(data)
    if magic != MAGIC or version != VERSION:
        return None
    if mode in MODES_SANS_ECHO:
        return (seq, mode, None)
    sortie = bytearray(data)
    if rssi is not None:
        ENTETE.pack_into(sortie, 0, magic, version, mode, seq, tx_ns,
                         max(-128, min(127, int(rssi))),
                         max(0, min(255, int(lqi or 0))), FAR_VALID, 0)
    return (seq, mode, bytes(sortie))


def repondre(args, liens):
    """Boucle unique sur tous les transports demandes, via select : pas de
    threads, donc aucune synchronisation a soigner dans un outil de mesure."""
    voies = {nom: VoieSol(nom) for nom in liens}
    sock = None
    ser = None
    tampon = b""
    sdl_stats = {}

    besoin_udp = any(n in ("wfb", "udp") for n in liens)
    besoin_serie = "cc1101" in liens

    if besoin_udp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", args.listen_port))
        except OSError as e:
            print(f"\nimpossible d'ecouter sur {args.listen_port} : {e}")
            print("le dashboard utilise 5602 et 5612 — choisir d'autres ports")
            print("avec --listen-port / --reply-port.")
            return 1

    if besoin_serie:
        try:
            import serial
        except ImportError:
            print("pyserial manquant :  pip install pyserial", file=sys.stderr)
            return 1
        port = trouver_port_serie(args.port)
        if port is None:
            print("aucun port serie trouve ; le preciser avec --port.",
                  file=sys.stderr)
            return 1
        if not liberer_port(port, args.stop_server):
            return 1
        ser = serial.Serial(port, args.baud, timeout=0)

    nom_udp = "wfb" if "wfb" in liens else "udp"
    detail = []
    if besoin_serie:
        detail.append(f"serie {trouver_port_serie(args.port)}")
    if besoin_udp:
        detail.append(f"udp {args.listen_port} -> {args.reply_port}")
    cmd = f"./tools/link_bench.sh ping --link={','.join(liens)}"
    if "wfb" in liens or "udp" in liens:
        # Le banc du drone a ses propres ports par defaut, qui ne sont pas
        # ceux de wfb-ng : les expliciter pour que la commande soit copiable.
        cmd += f" --local-port={args.reply_port} --peer-port={args.listen_port}"
    entete_commune(", ".join(liens),
                   "repondeur de tous les liens demandes",
                   "   ".join(detail), cmd,
                   sur_le_drone=(liens != ["udp"]))

    t0 = time.monotonic()
    prochain = t0 + 1.0
    fin = t0 + args.duration if args.duration else None
    sources = [x for x in (sock, ser) if x is not None]

    try:
        while True:
            maintenant = time.monotonic()
            if fin is not None and maintenant >= fin:
                break
            if maintenant >= prochain:
                prochain += 1.0
                for v in voies.values():
                    if v.recues:
                        v.ligne(maintenant - t0)

            prets, _, _ = select.select(sources, [], [], 0.2)

            if sock is not None and sock in prets:
                data, _ = sock.recvfrom(4096)
                r = _echo(data, args.rssi, 0)
                if r is None:
                    voies[nom_udp].hors_banc += 1
                else:
                    seq, _, sortie = r
                    voies[nom_udp].ajouter(seq, len(data), args.rssi, 0)
                    if sortie is not None:
                        sock.sendto(sortie, ("127.0.0.1", args.reply_port))

            if ser is not None and ser in prets:
                tampon += ser.read(4096)
                # Une ligne sans fin de ligne ne doit pas faire grossir le
                # tampon indefiniment.
                if len(tampon) > 65536:
                    tampon = tampon[-4096:]
                while b"\n" in tampon:
                    brute, tampon = tampon.split(b"\n", 1)
                    texte = brute.decode(errors="replace").strip()
                    if texte.startswith("RX:"):
                        parts = texte[3:].split(":")
                        try:
                            data = bytes.fromhex(parts[0])
                        except ValueError:
                            continue
                        rssi = int(parts[1]) if len(parts) > 1 else None
                        lqi = int(parts[2]) if len(parts) > 2 else 0
                        r = _echo(data, rssi, lqi)
                        if r is None:
                            voies["cc1101"].hors_banc += 1
                            continue
                        seq, _, sortie = r
                        voies["cc1101"].ajouter(seq, len(data), rssi, lqi)
                        if sortie is not None:
                            ser.write(b"TX:" + sortie.hex().encode() + b"\n")
                    elif texte.startswith("LQ:"):
                        ch = texte[3:].split(":")
                        if len(ch) >= 7:
                            sdl_stats = dict(zip(
                                ("rssi", "lqi", "crc", "len", "ovf", "cca",
                                 "forced"), (int(x) for x in ch[:7])))
                    elif texte.startswith("TXERR"):
                        print(f"  firmware : {texte}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if sock is not None:
            sock.close()
        if ser is not None:
            ser.close()
        print("\n" + "─" * 64)
        for v in voies.values():
            v.bilan()
        if sdl_stats:
            print(f"\n  compteurs du firmware SDL : crc={sdl_stats.get('crc')} "
                  f"len={sdl_stats.get('len')} ovf={sdl_stats.get('ovf')} "
                  f"cca={sdl_stats.get('cca')} forced={sdl_stats.get('forced')}")
    return 0


def _sur_sigterm(_signo, _frame):
    # Sans cela, un lancement sous `timeout` tue le processus sans passer par
    # les blocs finally : le bilan, qui est tout l'interet de l'outil, serait
    # perdu a chaque mesure automatisee.
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, _sur_sigterm)
    p = argparse.ArgumentParser(
        description="Station sol du banc de liaison (cf. ./bench_link)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="exemples :\n"
               "  %(prog)s cc1101\n"
               "  %(prog)s cc1101,wfb        (les deux en meme temps)\n"
               "  %(prog)s cc1101 --stop-server\n"
               "  %(prog)s udp\n")
    p.add_argument("liens",
                   help="liens a qualifier, separes par des virgules : "
                        "cc1101, wfb, udp")
    p.add_argument("--port", default=None,
                   help="port serie de l'ESP32 (cc1101) ; detecte sinon")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--stop-server", action="store_true",
                   help="arreter gcs-server.service s'il tient le port serie")
    p.add_argument("--listen-port", type=int, default=None)
    p.add_argument("--reply-port", type=int, default=None)
    p.add_argument("--rssi", type=int, default=None,
                   help="RSSI au sol a inscrire dans l'echo UDP, si connu")
    p.add_argument("--duration", type=float, default=None,
                   help="s'arreter apres N secondes au lieu d'attendre Ctrl-C")
    args = p.parse_args()

    liens = [x.strip() for x in args.liens.split(",") if x.strip()]
    inconnus = [x for x in liens if x not in ("cc1101", "wfb", "udp")]
    if inconnus:
        print(f"lien(s) inconnu(s) : {', '.join(inconnus)}", file=sys.stderr)
        return 2
    if "wfb" in liens and "udp" in liens:
        print("wfb et udp partagent le meme transport : en demander un seul.",
              file=sys.stderr)
        return 2

    if "udp" in liens:
        if args.listen_port is None:
            args.listen_port = PORTS_UDP[0]
        if args.reply_port is None:
            args.reply_port = PORTS_UDP[1]
    elif "wfb" in liens:
        # Les ports sont ceux de wfb-ng sur CETTE machine, pas ceux du drone.
        # On les deduit s'il tourne, sinon on refuse plutot que de deviner :
        # un port plausible mais faux donne 100 % de perte sans que rien
        # n'indique que le probleme vient de la.
        if args.listen_port is None or args.reply_port is None:
            ecoute, renvoi = ports_wfb()
            if args.listen_port is None:
                args.listen_port = ecoute
            if args.reply_port is None:
                args.reply_port = renvoi
            manquants = []
            if args.listen_port is None:
                manquants.append("wfb_rx (--listen-port)")
            if args.reply_port is None:
                manquants.append("wfb_tx (--reply-port)")
            if manquants:
                print("Impossible de deduire les ports de wfb-ng : "
                      + " et ".join(manquants) + " introuvable(s).")
                print("  - demarrer wfb-ng, puis relancer ; ou")
                print("  - donner les ports a la main :")
                print("      --listen-port <sortie wfb_rx> "
                      "--reply-port <entree wfb_tx>")
                return 1
            print(f"ports wfb-ng deduits : ecoute {args.listen_port}, "
                  f"renvoi {args.reply_port}")

    return repondre(args, liens)


if __name__ == "__main__":
    sys.exit(main())
