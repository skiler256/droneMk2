#!/bin/bash
# Prépare l'adaptateur du lien principal (PDL) : mode moniteur et canal.
#
# Lancé par wfb-iface.service au démarrage, avant wfb_tx et wfb_rx.
#
# L'interface est résolue par son PILOTE et non par son nom : `wlan1` dépend
# de l'ordre d'énumération et peut devenir `wlan2` au prochain démarrage,
# alors que le pilote, lui, ne change pas. Un nom figé dans une unité systemd
# est une panne qui n'arrive qu'une fois sur dix redémarrages — le pire genre.
set -euo pipefail

CANAL="${WFB_CHANNEL:-1}"
PILOTE="rtl88xxau_wfb"

trouver_interface() {
  local n
  for n in /sys/class/net/*; do
    [ -e "$n/device/driver" ] || continue
    if [ "$(basename "$(readlink -f "$n/device/driver")")" = "$PILOTE" ]; then
      basename "$n"
      return 0
    fi
  done
  return 1
}

IFACE="$(trouver_interface)" || {
  echo "aucune interface avec le pilote $PILOTE : adaptateur absent ?" >&2
  exit 1
}
echo "interface PDL : $IFACE"

# NetworkManager gère l'interface par défaut : il la scanne, la remet en mode
# géré et lui change de canal. C'est ce qui a fait croire pendant une session
# entière à une dégradation du pilote — la liaison marchait juste après un
# rechargement de module, avant que NM ne reprenne la main.
if command -v nmcli >/dev/null 2>&1; then
  nmcli dev set "$IFACE" managed no || true
fi

ip link set "$IFACE" down
iw dev "$IFACE" set monitor otherbss
ip link set "$IFACE" up
iw dev "$IFACE" set channel "$CANAL"

# VÉRIFIER, jamais supposer : un rechargement de module détaché avait déjà
# laissé le Pi sur le canal 116 alors que la commande demandait le canal 11.
etat="$(iw dev "$IFACE" info)"
echo "$etat"
echo "$etat" | grep -q "channel $CANAL " || {
  echo "le canal demandé ($CANAL) n'est PAS celui appliqué" >&2
  exit 1
}
echo "$etat" | grep -q "type monitor" || {
  echo "l'interface n'est pas en mode moniteur" >&2
  exit 1
}

# Le nom résolu est publié pour wfb-tx et wfb-rx : une seule résolution, un
# seul endroit où elle peut être fausse. Les dupliquer les ferait diverger
# le jour où on en corrige une seule.
echo "$IFACE" > /run/wfb.iface
