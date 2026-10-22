#!/bin/bash
# Déploie le dashboard GCS du dépôt vers la copie de service (~/gcs).
#
#   ./deploy/deploy-gcs.sh              déploie
#   ./deploy/deploy-gcs.sh --essai      montre ce qui serait fait, sans écrire
#   ./deploy/deploy-gcs.sh --sale       autorise un dépôt non commité
#
# À lancer SUR la station sol, depuis sa propre copie du dépôt :
#
#   cd ~/droneMk2 && git pull && ./deploy/deploy-gcs.sh
#
# POURQUOI DEUX COPIES : choix assumé. La copie de service est indépendante du
# dépôt, donc on peut y intervenir en vol sans toucher à l'historique. Le prix
# est la dérive — la copie déployée était un instantané plus ANCIEN que le
# dépôt, et personne ne s'en était aperçu. Ce script est ce qui la rend
# visible et réparable.
#
# POURQUOI UNE LISTE EXPLICITE et non un rsync du dossier : la copie de
# service contient des choses qui ne viennent PAS du dépôt — node_modules au
# premier chef. Une synchronisation aveugle les effacerait, et il faudrait un
# npm install sur une machine qui n'a peut-être pas de réseau.
set -euo pipefail

DEPOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CIBLE="${GCS_DEPLOY_DIR:-$HOME/gcs}"
ESSAI=0
SALE=0
for a in "$@"; do
  case "$a" in
    --essai|--dry-run) ESSAI=1 ;;
    --sale|--dirty)    SALE=1 ;;
    *) echo "option inconnue : $a" >&2; exit 2 ;;
  esac
done

# Les CSV vont dans data/, des DEUX côtés. Ce script les déposait à plat, sur
# la foi d'un ancien agencement : telemetry-codec.js résout `../data/` depuis
# GCS/, donc le dashboard ne trouvait plus ses tables et gcs-server bouclait
# en échec. Constaté le 22/10/2026, après un déploiement réussi — la copie
# était juste, l'agencement non, et personne n'avait redémarré le service.
FICHIERS_GCS=(
  server.js serial-bridge.js telemetry-codec.js codes-table.js
  debug-injector.js package.json package-lock.json
)
DOSSIERS_GCS=(public public-debug)
# MISSIONPLAN.csv n'est pas déployé : le dashboard ne le lit pas.
CSV=(TELEMETRY.csv INDEX.csv ALARMS.csv MISSION_ALARMS.csv)

# ── contrôles avant d'écrire ──
cd "$DEPOT"
# Le contrôle ne porte que sur ce qui est RÉELLEMENT déployé : une
# modification dans GCS/ESP32 n'a rien à voir avec le dashboard, et refuser
# pour elle apprendrait à utiliser --sale par réflexe — ce qui viderait le
# garde-fou de son sens.
A_VERIFIER=("${FICHIERS_GCS[@]/#/GCS/}" "${DOSSIERS_GCS[@]/#/GCS/}" "${CSV[@]/#/data/}")
if [ -n "$(git status --porcelain -- "${A_VERIFIER[@]}" 2>/dev/null)" ]; then
  if [ "$SALE" = 0 ]; then
    echo "Le dépôt a des modifications non commitées dans GCS/ ou data/." >&2
    echo "Déployer ainsi rend intraçable ce qui tourne. Commiter d'abord," >&2
    echo "ou forcer avec --sale en sachant ce qu'on fait." >&2
      git status --short -- "${A_VERIFIER[@]}" >&2
    exit 1
  fi
  echo "ATTENTION : déploiement d'un dépôt NON commité (--sale)."
fi

[ -d "$CIBLE/GCS" ] || { echo "cible introuvable : $CIBLE/GCS" >&2; exit 1; }

COMMIT="$(git rev-parse --short HEAD)"
SUJET="$(git log -1 --format=%s)"

echo "dépôt  : $DEPOT  ($COMMIT)"
echo "cible  : $CIBLE"
echo

if [ "$ESSAI" = 1 ]; then
  echo "— essai, rien ne sera écrit —"
  for f in "${FICHIERS_GCS[@]}"; do
    if ! cmp -s "GCS/$f" "$CIBLE/GCS/$f" 2>/dev/null; then
      echo "  modifié  GCS/$f"
    fi
  done
  for d in "${DOSSIERS_GCS[@]}"; do
    diff -rq "GCS/$d" "$CIBLE/GCS/$d" 2>/dev/null | sed 's/^/  /' || true
  done
  for c in "${CSV[@]}"; do
    cmp -s "data/$c" "$CIBLE/$c" 2>/dev/null || echo "  modifié  $c"
  done
  exit 0
fi

# ── sauvegarde, pour pouvoir revenir en une commande ──
# On ne garde qu'une génération : au-delà, personne ne s'en sert et ça remplit
# le disque d'une machine qui n'en a pas beaucoup.
PRECEDENT="$CIBLE.precedent"
rm -rf "$PRECEDENT"
cp -a "$CIBLE" "$PRECEDENT"
echo "sauvegarde : $PRECEDENT"

# ── copie ──
for f in "${FICHIERS_GCS[@]}"; do
  install -m 0644 "GCS/$f" "$CIBLE/GCS/$f"
done
for d in "${DOSSIERS_GCS[@]}"; do
  rm -rf "$CIBLE/GCS/$d"
  cp -a "GCS/$d" "$CIBLE/GCS/$d"
done
install -d -m 0755 "$CIBLE/data"
for c in "${CSV[@]}"; do
  install -m 0644 "data/$c" "$CIBLE/data/$c"
done
# On signale les restes de l'ancien agencement sans les supprimer : un script
# de déploiement qui efface des fichiers qu'il n'a pas posés est un script
# qu'on finit par ne plus oser lancer.
for c in "${CSV[@]}"; do
  [ -e "$CIBLE/$c" ] && echo "note : $CIBLE/$c est un reste à plat, plus lu par personne" >&2
done
echo "copié : ${#FICHIERS_GCS[@]} fichiers, ${#DOSSIERS_GCS[@]} dossiers, ${#CSV[@]} CSV"

# ── empreinte de version ──
# Pour pouvoir répondre à « qu'est-ce qui tourne là ? » sans deviner. C'est
# l'absence de cette réponse qui a laissé la dérive s'installer.
{
  echo "commit  : $COMMIT"
  echo "sujet   : $SUJET"
  echo "déployé : $(date -Is)"
  echo "depuis  : $DEPOT"
  [ "$SALE" = 1 ] && echo "ETAT    : dépôt NON commité au moment du déploiement"
} > "$CIBLE/DEPLOYE"
echo "empreinte : $CIBLE/DEPLOYE"

# ── redémarrage ──
# server.js charge TELEMETRY.csv AU DÉMARRAGE : déployer un nouveau format
# sans relancer le service ne change rien, piège déjà rencontré.
if systemctl is-active --quiet gcs-server.service; then
  echo "redémarrage de gcs-server.service ..."
  sudo systemctl restart gcs-server.service
  sleep 2
  systemctl is-active --quiet gcs-server.service \
    && echo "  actif" \
    || { echo "  ÉCHEC — journalctl -u gcs-server -n 30" >&2; exit 1; }
else
  echo "gcs-server.service n'était pas actif : rien redémarré."
  echo "  (le format de TELEMETRY.csv n'est lu qu'au démarrage du service)"
fi

echo
echo "fait. Pour revenir en arrière :"
echo "  rm -rf $CIBLE && mv $PRECEDENT $CIBLE && sudo systemctl restart gcs-server"
