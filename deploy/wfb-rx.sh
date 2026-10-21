#!/bin/bash
# Réception du lien PDL, avec son relais de statistiques.
#
# wfb_rx écrit sa santé (lignes RX_ANT, dont le RSSI par antenne) sur stdout ;
# le relais la republie en datagramme UDP, où WFB_NG_Driver la lit sans
# risque de blocage. systemd ne sait pas exprimer un tube dans ExecStart,
# d'où ce script.
set -euo pipefail

# Nom résolu par wfb-iface.service, dont cette unité dépend.
IFACE="$(cat /run/wfb.iface)"

cd /home/jules/wfb-ng
exec ./wfb_rx -p "${WFB_RX_RADIO_PORT}" -c 127.0.0.1 -u "${WFB_RX_UDP_PORT}" \
     -K "${WFB_KEY}" "$IFACE" \
  | python3 /home/jules/droneMk2/tools/wfb_stats_relay.py \
      --port "${WFB_STATS_PORT}" --echo
