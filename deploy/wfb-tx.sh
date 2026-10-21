#!/bin/bash
# Émission du lien PDL.
set -euo pipefail

# Nom résolu par wfb-iface.service, dont cette unité dépend.
IFACE="$(cat /run/wfb.iface)"

cd /home/jules/wfb-ng
exec ./wfb_tx -p "${WFB_TX_RADIO_PORT}" -u "${WFB_TX_UDP_PORT}" \
     -K "${WFB_KEY}" "$IFACE"
