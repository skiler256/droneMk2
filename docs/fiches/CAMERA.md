# Caméra / vidéo

Rôle : capture vidéo embarquée, diffusée au sol via le PDL. Pas d'acronyme
dédié — système pas encore implémenté.

| | |
|---|---|
| **Composition matérielle** | Caméra type barrette webcam de PC portable, branchée en **USB** sur le Pi 5 (pas de module CSI officiel). |
| **Interface logicielle** | Interface `IVideoSource::pollFrame()` définie côté architecture (`include/drone/Components/SystemMonitoring/`), consommée par `VideoTask` (`SysMonitoring`). Implémentation réelle prévue via `WFB_NG_Driver` (PDL, cf. fiche PDL/SDL) — **pas codée**. En dev, `UdpVideoDriver.hpp` sert de bouchon : reçoit du MJPEG déjà encodé sur UDP loopback, ne lit rien de la caméra réelle. |
| **État actuel** | **Pas implémenté** — il manque un driver `IVideoSource` concret qui capture depuis `/dev/videoX` (V4L2) et encode les frames, à écrire avant de pouvoir relier la caméra USB réelle au pipeline `VideoTask`/`WFB_NG_Driver`. |
| **Redondance / sécurité** | Aucune — capteur unique, non critique au vol (contrairement à OCAS/AHRS/GPS). |

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../img/camera-dark.svg">
  <img alt="Caméra et son support" src="../img/camera-light.svg">
</picture>

Doc détaillée : `HARDWARE.md` §6, `include/drone/Components/SystemMonitoring/SYSMONITORING.md` §1.
