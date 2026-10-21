#!/bin/bash
# Installe la persistance du lien PDL sur la machine courante.
#
#   sudo ./deploy/install.sh drone
#   sudo ./deploy/install.sh gs
#
# Ce qui est rendu persistant : le mode moniteur et le canal de l'adaptateur,
# le réglage NetworkManager, et les processus wfb_tx / wfb_rx. Le paramètre
# rtw_tx_pwr_idx_override du module l'est déjà via /etc/modprobe.d/ — sans
# lui l'étage radio reste muet alors que les compteurs montent.
set -euo pipefail

[ "${1:-}" = "drone" ] || [ "${1:-}" = "gs" ] || {
  echo "usage : $0 { drone | gs }" >&2
  exit 2
}
ROLE="$1"
ICI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

[ "$(id -u)" = 0 ] || { echo "à lancer avec sudo" >&2; exit 1; }

install -m 0644 "$ICI/wfb.conf.$ROLE" /etc/default/wfb
echo "configuration : /etc/default/wfb (rôle $ROLE)"

for u in wfb-iface wfb-tx wfb-rx; do
  install -m 0644 "$ICI/$u.service" "/etc/systemd/system/$u.service"
done
systemctl daemon-reload

# Vérifie que le paramètre de puissance est bien persistant : l'oublier donne
# une radio silencieuse dont les compteurs montent normalement, le symptôme
# le plus trompeur de tout ce bring-up.
if ! grep -rqs 'rtw_tx_pwr_idx_override' /etc/modprobe.d/; then
  echo "ATTENTION : rtw_tx_pwr_idx_override absent de /etc/modprobe.d/." >&2
  echo "  L'étage radio sera MUET au prochain démarrage. Ajouter :" >&2
  echo "  echo 'options 88XXau_wfb rtw_tx_pwr_idx_override=30' \\" >&2
  echo "    > /etc/modprobe.d/88XXau_wfb.conf" >&2
fi

systemctl enable wfb-iface.service wfb-tx.service wfb-rx.service
echo
echo "installé et activé. Pour démarrer maintenant :"
echo "  sudo systemctl start wfb-iface wfb-tx wfb-rx"
echo
echo "Le seul test valable de la persistance est un VRAI redémarrage."
