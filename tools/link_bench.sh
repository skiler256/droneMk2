#!/usr/bin/env bash
# Lance le banc de liaison sur le drone, en compilant si necessaire.
#
#   ./tools/link_bench.sh                  ping sur le lien SDL (defaut)
#   ./tools/link_bench.sh ramp             cherche le plafond de charge utile
#   ./tools/link_bench.sh ping --link=wfb  lien principal PDL
#   ./tools/link_bench.sh range --csv=/tmp/portee.csv
#
# Le pendant au sol est tools/hw_debug/link_bench_gcs.py. Son role n'est pas
# le meme selon le lien, et c'est important :
#
#   --link=cc1101  l'echo est fait par le FIRMWARE de l'ESP32. Le banc mesure
#                  donc tout seul, sans rien lancer au sol. L'outil au sol
#                  n'ajoute que la vue d'en face (compteurs, rejets).
#
#   --link=wfb     l'outil au sol EST le repondeur : sans lui, aucun echo et
#                  le banc annonce 100 % de perte.
#
#   --link=udp     bouclage local : les deux bouts doivent tourner sur la
#                  MEME machine, pas un de chaque cote.
#
# Toutes les options non reconnues ici sont passees telles quelles au binaire
# (--interval=, --duration=, --size=, --csv=, --spi-hz=, les ports UDP...).
set -euo pipefail

racine="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$racine"

binaire="build-bench/bench_link"

# Recompiler si le binaire manque ou si une source est plus recente que lui :
# mesurer avec un binaire perime est la meilleure facon de tirer des
# conclusions fausses.
besoin_build=0
if [[ ! -x "$binaire" ]]; then
  besoin_build=1
else
  while IFS= read -r src; do
    [[ "$src" -nt "$binaire" ]] && besoin_build=1 && break
  done < <(find src include CMakeLists.txt -type f \
             \( -name '*.cpp' -o -name '*.hpp' -o -name 'CMakeLists.txt' \) 2>/dev/null)
fi

if (( besoin_build )); then
  echo "compilation du banc ..."
  cmake -B build-bench -DBUILD_HW_BENCH=ON -DCMAKE_BUILD_TYPE=Release \
        >/dev/null
  cmake --build build-bench --target bench_link -j"$(nproc)" 2>&1 \
    | grep -E 'error|warning|Linking' || true
  [[ -x "$binaire" ]] || { echo "la compilation a echoue." >&2; exit 1; }
  echo
fi

# Mode : premier argument s'il ne commence pas par '--'.
mode="ping"
if [[ $# -gt 0 && "$1" != --* ]]; then
  mode="$1"
  shift
fi

# Lien : par defaut le SDL, celui qui porte les commandes montantes.
lien="cc1101"
for a in "$@"; do
  [[ "$a" == --link=* ]] && lien="${a#--link=}"
done
if [[ ! " $* " == *" --link="* ]]; then
  set -- "--link=$lien" "$@"
fi

echo "Si ce n'est pas deja fait, lancer au sol :"
echo "    python3 tools/hw_debug/link_bench_gcs.py $lien"
echo

exec "./$binaire" "$mode" "$@"
