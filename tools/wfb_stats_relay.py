#!/usr/bin/env python3
"""Relais de statistiques wfb-ng : sortie texte de wfb_rx -> datagramme UDP.

    ./wfb_rx -p 0 -c 127.0.0.1 -u 5602 -K gs.key wlan1 | \
        python3 tools/wfb_stats_relay.py --port 5699

POURQUOI UN RELAIS plutot que de parser directement dans le pilote : ce
format texte est celui de wfb-ng, pas le notre, et il change entre versions.
La preuve est deja faite — le pilote cherchait des champs JSON qui n'existent
pas dans cette version. Mieux vaut qu'un script de relais casse qu'un pilote
du logiciel de vol. La fragilite reste dehors.

POURQUOI PAS UN TUBE NOMME : l'ouverture d'un FIFO en ecriture bloque jusqu'a
ce qu'un lecteur l'ouvre. wfb-ng ne demarrerait donc pas avant le logiciel de
vol, et si celui-ci cesse de lire, les 64 Ko de tampon se remplissent et
wfb_rx bloque EN ECRITURE : la liaison radio s'arreterait parce que personne
ne lit les statistiques. Inversion de dependance inacceptable.

Un datagramme UDP, lui, se perd sans consequence si personne n'ecoute.

Format publie — disposition explicite, petit-boutien, memes conventions que
les paquets TELEM (offsets fixes, pas d'attribut packed) :

      0  u16  magic    0x5751
      2  u8   version  1
      3  i8   rssi     le PIRE des antennes, en dBm
      4  i8   snr      le PIRE des antennes, en dB
      5  u8   antennes nombre d'antennes vues dans le dernier rapport
      6  u32  recus    count_p_uniq de wfb_rx
     10  u32  perdus   count_p_lost de wfb_rx (non recuperes par la FEC)

Le RSSI publie est le PIRE des antennes, pas le meilleur. Choix assume : avec
une diversite 2T2R le recepteur utilise la meilleure chaine, donc le pire
RSSI ne mesure pas la marge de liaison — il mesure la SANTE du systeme
d'antennes. Une antenne debranchee ou mal placee se voit immediatement, la
ou une moyenne la masquerait.
"""
import argparse
import socket
import struct
import sys
import time

TRAME = struct.Struct("<HBbbBII")
MAGIC = 0x5751
VERSION = 1


def main():
    p = argparse.ArgumentParser(
        description="Republie les statistiques de wfb_rx en UDP loopback")
    p.add_argument("--port", type=int, default=5699,
                   help="port UDP de publication (defaut 5699)")
    p.add_argument("--echo", action="store_true",
                   help="reecrire les lignes lues sur stdout, pour les logs")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Agregat du rapport en cours. wfb_rx emet une ligne RX_ANT par antenne,
    # toutes au meme horodatage : on les accumule et on publie au changement
    # d'horodatage, sinon on publierait une vue partielle a une antenne.
    horodatage = None
    pire_rssi = None
    pire_snr = None
    antennes = 0
    recus = 0
    perdus = 0

    def publier():
        if pire_rssi is None:
            return
        trame = TRAME.pack(MAGIC, VERSION,
                           max(-128, min(127, pire_rssi)),
                           max(-128, min(127, pire_snr if pire_snr is not None else 0)),
                           min(255, antennes), recus, perdus)
        try:
            sock.sendto(trame, ("127.0.0.1", args.port))
        except OSError:
            pass  # personne n'ecoute : sans consequence, c'est voulu

    for ligne in sys.stdin:
        if args.echo:
            sys.stdout.write(ligne)
            sys.stdout.flush()

        champs = ligne.split()
        if len(champs) < 3:
            continue

        # PKT <ts> : compteurs de l'intervalle de rapport. Les treize champs,
        # dans l'ordre exact de wfb-ng/src/rx.cpp (IPC_MSG, ~ligne 694) :
        #
        #   0 count_p_all        7 count_p_lost
        #   1 count_b_all        8 count_p_bad
        #   2 count_p_dec_err    9 count_p_outgoing
        #   3 count_p_session   10 count_b_outgoing
        #   4 count_p_data      11 count_p_frags
        #   5 count_p_uniq      12 count_p_frags_lost
        #   6 count_p_fec_recovered
        #
        # On prend la paire du MEME etage, celui de la FEC : count_p_uniq
        # (paquets uniques recus) et count_p_lost (paquets que la FEC n'a pas
        # pu reconstruire). C'est la seule paire qui ait un sens ensemble.
        #
        # Les deux champs lus auparavant etaient faux et le resultat ne pouvait
        # valoir que 0 % ou 100 %. Mesure du 22/10/2026 :
        #   33:1496:0:1:32:32:2:0:0:23:368:36:2
        # soit 2 fragments perdus, 2 RECUPERES par la FEC, 0 paquet perdu --
        # et l'ancien calcul en tirait 100 % de perte. Sur un lien critique,
        # cette valeur declenche les failsafes pour rien.
        if champs[1] == "PKT":
            vals = champs[2].split(":")
            if len(vals) >= 8:
                try:
                    recus = int(vals[5])
                    perdus = int(vals[7])
                except ValueError:
                    pass
            continue

        # RX_ANT <freq:mcs:bw> <antenne> <nb:rssi_min:rssi_moy:rssi_max:snr_min:snr_moy:snr_max>
        if champs[1] == "RX_ANT" and len(champs) >= 5:
            ts = champs[0]
            if ts != horodatage:
                # Nouveau rapport : publier le precedent, puis repartir.
                publier()
                horodatage = ts
                pire_rssi = None
                pire_snr = None
                antennes = 0
            vals = champs[4].split(":")
            if len(vals) < 7:
                continue
            try:
                rssi_moy = int(vals[2])
                snr_moy = int(vals[5])
            except ValueError:
                continue
            antennes += 1
            # Le PIRE, pas le meilleur : cf. l'en-tete de ce fichier.
            pire_rssi = rssi_moy if pire_rssi is None else min(pire_rssi, rssi_moy)
            pire_snr = snr_moy if pire_snr is None else min(pire_snr, snr_moy)

    publier()
    sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
