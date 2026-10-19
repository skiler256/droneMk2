# PDL / SDL — Primary / Secondary Data Link

Rôle : liaison télémétrie/vidéo sol-drone, dimensionnée en primaire +
secours — la perte du PDL ne coupe pas la télémétrie critique, seulement
la vidéo et le débit complet.

| | |
|---|---|
| **Composition matérielle** | **PDL** : module wifi broadcast 433MHz (wfb-ng), longue portée. **SDL** : module CC1101, sub-GHz SPI, bas débit. Côté GCS : adaptateur WiFi RTL8812AU dédié (DKMS `svpcom/rtl8812au`) pour recevoir le PDL, séparé du WiFi générique de la malette. |
| **Interface logicielle** | `WFB_NG_Driver.hpp` (PDL) et `CC1101_Driver.hpp` (SDL), `include/drone/Components/SystemMonitoring/Driver/` — les deux implémentent `ITelemetryLink` (`Driver.hpp`). Consommés par `TelMainTask` (PDL) et `TelSecTask` (SDL) dans `SysMonitoring`. |
| **Répartition du trafic** | PDL porte la télémétrie complète + la vidéo. SDL ne porte que le sous-ensemble léger marqué `Links=BOTH` dans `TELEMETRY.csv` — paquets critiques dupliqués sur les deux liens, le reste seulement sur PDL. |
| **État actuel** | **Pas implémenté** — squelettes d'interface seulement (`.hpp` sans `.cpp`), pas de câblage hardware SPI/socket RF fait. En dev, `TelMainTask`/`TelSecTask` passent par de l'UDP loopback (`ITelemetryLink` mock) plutôt que par PDL/SDL réels. Script de banc `tools/hw_debug/CC1101_test.py` confirme l'interface SDL (SPI bus 0/CE0, 500kHz, mode 0) sans driver de prod. |
| **Redondance / sécurité** | SDL indépendant du PDL (fréquence, protocole et module radio différents) — pas de mode de panne commun identifié à ce stade. |

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../img/radio-antennes-dark.svg">
  <img alt="Modules radio et emplacement antennes" src="../img/radio-antennes-light.svg">
</picture>

**Antenne** : omnidirectionnelle, montée sur le dessus de la coque.

Doc détaillée : `HARDWARE.md` §2, `include/drone/Components/SystemMonitoring/SYSMONITORING.md` §1.
