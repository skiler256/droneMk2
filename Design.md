# Software Design Document — Drone Pi5
> Version 0.1 — Vision d'ensemble  
> Auteurs : à compléter  
> Statut : Draft

---

## Table des matières

1. [Vue d'ensemble du système](#1-vue-densemble-du-système)
2. [Flux de données](#2-flux-de-données)
3. Boucles des blocs *(à venir)*
4. Machines à états *(à venir)*
5. Algorigrammes de décision critique *(à venir)*
6. Interface Pi ↔ GCS *(à venir)*

---

## 1. Vue d'ensemble du système

### 1.1 Objectif

Le système pilote un drone autonome en suivant des waypoints.  
Le **Raspberry Pi 5** est le cerveau de la mission : il fusionne les capteurs,
calcule la navigation, et envoie des consignes de vitesse au **Flight Controller
ArduPilot** qui gère le contrôle bas niveau (attitude, moteurs).

En cas de défaillance du Pi, ArduPilot atterrit de façon autonome via son
propre failsafe. Le Pi ne peut pas crasher le drone — il peut seulement le
ralentir ou le faire atterrir.

### 1.2 Contraintes système

| Contrainte | Valeur | Raison |
|---|---|---|
| Fréquence boucle fusion | 50 Hz | Cohérence Kalman |
| Fréquence boucle nav | 20 Hz | Suffisant pour waypoints |
| Fréquence heartbeat FC | 1 Hz minimum | Failsafe ArduPilot |
| Latence max consigne → FC | 200 ms | Au-delà : HOLD automatique |
| Vitesse max horizontale | 5 m/s | Sécurité physique |
| Vitesse max descente | 2 m/s | Sécurité atterrissage |

### 1.3 Architecture globale des systèmes

Les 5 systèmes qui composent la solution, et leurs liens physiques :

```mermaid
graph TB
    subgraph Drone
        subgraph Pi5["Raspberry Pi 5 (Linux RT)"]
            SF[① SensorsFusion]
            NAV[② Navigation]
            MAV[③ MAVLink]
            MON[④ Monitoring]
        end

        subgraph FC["Flight Controller (ArduPilot)"]
            PID[PID Attitude]
            BARO[Baromètre]
            ESC[ESCs / Moteurs]
        end

        subgraph Capteurs
            GPS[GPS]
            MAG[Magnétomètre]
            LIDAR[Télémètre laser]
            IMU[IMU Acc+Gyro]
            CAM[Caméra]
        end
    end

    subgraph Sol
        GCS[Station sol\nPython GCS]
        OP[Opérateur]
    end

    %% Capteurs → Pi
    GPS   -->|UART| SF
    MAG   -->|I2C|  SF
    LIDAR -->|I2C/UART| SF
    IMU   -->|SPI| SF
    CAM   -->|CSI| MON

    %% Pi interne
    SF  -->|SharedState| NAV
    SF  -->|SharedState| MON
    NAV -->|VelocityCmd| MAV
    MAV -->|FCStatus|    NAV
    MAV -->|FCStatus|    MON

    %% Pi → FC
    MAV -->|MAVLink UART| PID
    PID --> ESC
    BARO -->|altitude| PID

    %% Pi → Sol
    MON -->|JSON + vidéo\nWi-Fi UDP| GCS
    GCS -->|commandes\nWi-Fi UDP| MON
    GCS --- OP
```

### 1.4 Responsabilités par système

| Système | Produit | Consomme | Failsafe propre |
|---|---|---|---|
| **① SensorsFusion** | Position, Vitesse, Attitude fusionnées | GPS, Mag, Lidar, IMU | Mode dégradé si capteur mort |
| **② Navigation** | Consignes de vitesse (VelocityCmd) | Position, Vitesse, FCStatus | HOLD si état invalide |
| **③ MAVLink** | FCStatus (baro, armé, mode) | VelocityCmd, heartbeat | HOLD si consigne trop vieille |
| **④ Monitoring** | Logs, télémétrie GCS, HUD vidéo | Tout le SharedState | Non critique |
| **ArduPilot FC** | Contrôle moteurs | Consignes vitesse Pi | Atterrissage si perte heartbeat |
| **GCS Python** | Commandes opérateur, affichage | Télémétrie Pi | Alerte opérateur |

---

## 2. Flux de données

### 2.1 Vue globale des flux internes au Pi

Les 4 blocs communiquent exclusivement via le **SharedState** —
jamais directement entre eux.

```mermaid
flowchart LR
    subgraph Drivers
        D1[GPS Driver\n1-10 Hz]
        D2[Mag Driver\n50 Hz]
        D3[Lidar Driver\n100 Hz]
        D4[IMU Driver\n200 Hz]
    end

    subgraph SF["① SensorsFusion — 50 Hz"]
        Q1[LatestMeasurement\nGPS]
        Q2[LatestMeasurement\nMag]
        Q3[LatestMeasurement\nLidar]
        Q4[SensorQueue\nIMU]
        KF[Filtre de Kalman]
    end

    subgraph SS["SharedState"]
        POS[position\nvitesse\nattitude]
        CMD[VelocityCmd]
        FCS[FCStatus]
    end

    subgraph NAV["② Navigation — 20 Hz"]
        SM[Machine à états]
        GEN[Génération consigne]
        VAL[Validation]
    end

    subgraph MAV["③ MAVLink — 50 Hz"]
        HB[Heartbeat]
        SEND[Envoi consigne]
        RECV[Réception télémétrie]
    end

    subgraph MON["④ Monitoring — 10 Hz"]
        LOG[Logs]
        TEL[Télémétrie GCS]
        HUD[HUD Vidéo]
    end

    D1 --> Q1
    D2 --> Q2
    D3 --> Q3
    D4 --> Q4

    Q1 & Q2 & Q3 & Q4 --> KF
    KF --> POS

    POS --> SM
    FCS --> SM
    SM --> GEN --> VAL --> CMD

    CMD --> SEND
    RECV --> FCS
    HB -->|1Hz| FC[(ArduPilot FC)]
    SEND -->|50Hz| FC

    POS & CMD & FCS --> LOG
    POS & CMD & FCS --> TEL
    POS --> HUD
```

### 2.2 Flux Pi → Flight Controller (MAVLink)

Communication série UART entre le Pi et ArduPilot.

```mermaid
sequenceDiagram
    participant NAV as ② Navigation
    participant MAV as ③ MAVLink
    participant FC  as ArduPilot FC

    loop Toutes les secondes
        MAV ->> FC : HEARTBEAT (type=onboard_controller)
        FC  ->> MAV : HEARTBEAT (mode, armé, état)
    end

    loop 50 Hz
        MAV ->> MAV : Lit VelocityCmd dans SharedState
        MAV ->> MAV : Vérifie fraîcheur (< 200ms)
        alt Consigne fraîche et valide
            MAV ->> FC : SET_POSITION_TARGET_LOCAL_NED\n(vx, vy, vz en m/s NED)
        else Consigne trop vieille
            MAV ->> FC : SET_POSITION_TARGET_LOCAL_NED\n(0, 0, 0) — HOLD
        end
        FC  ->> MAV : LOCAL_POSITION_NED + ATTITUDE\n(télémétrie retour)
    end

    note over FC : Si HEARTBEAT absent > 5s\n→ atterrissage autonome
```

### 2.3 Flux capteurs → SensorsFusion

Chaque driver tourne dans son propre thread et pousse ses mesures.

```mermaid
flowchart TD
    subgraph "Threads Drivers"
        GPS["GPS Driver\n1-10 Hz\nUART"]
        MAG["Mag Driver\n50 Hz\nI2C"]
        LID["Lidar Driver\n100 Hz\nI2C"]
        ACC["IMU Driver\n200 Hz\nSPI"]
    end

    subgraph "SensorsFusion — thread 50 Hz"
        subgraph "Buffers thread-safe"
            LM1["LatestMeasurement\nGPS — écrase si nouveau"]
            LM2["LatestMeasurement\nMag — écrase si nouveau"]
            LM3["LatestMeasurement\nLidar — écrase si nouveau"]
            SQ1["SensorQueue\nIMU — FIFO 16 slots"]
        end
        AGE["Vérification âge\n&lt; 100ms sinon drop"]
        KAL["Kalman\nprédiction + mise à jour"]
        OUT["SharedState\nposition · vitesse · attitude"]
    end

    GPS -->|push_GPS| LM1
    MAG -->|push_Mag| LM2
    LID -->|push_Tel| LM3
    ACC -->|push_Acc| SQ1

    LM1 & LM2 & LM3 & SQ1 --> AGE
    AGE --> KAL --> OUT
```

### 2.4 Flux Pi → GCS (télémétrie Wi-Fi)

Ce que le tableau de bord Python reçoit du Pi.

```mermaid
flowchart LR
    subgraph Pi["Pi5 — Monitoring 10Hz"]
        SS[SharedState\nsnapshot]
        SER[Sérialisation JSON]
        UDP[UDP socket]
        VID[Flux vidéo\nH264]
    end

    subgraph GCS["Station sol — Python"]
        RUDP[UDP receiver]
        RVID[Décodeur vidéo]
        DASH[Tableau de bord]
        MAP[Carte / trajectoire]
    end

    SS --> SER --> UDP -->|port 14550\n10 Hz| RUDP --> DASH
    VID -->|port 5600\nRTP| RVID --> DASH
    DASH --> MAP
```

### 2.5 Flux GCS → Pi (commandes opérateur)

Ce que le tableau de bord Python peut envoyer au Pi.

```mermaid
flowchart LR
    subgraph GCS["Station sol — Python"]
        OP[Opérateur]
        CMD[Sérialisation JSON]
        SUDP[UDP sender]
    end

    subgraph Pi["Pi5 — Monitoring"]
        RUDP[UDP receiver]
        VAL[Validation\ncommande]
        NAV[Navigation\nSharedState]
    end

    OP -->|action UI| CMD --> SUDP -->|port 14551| RUDP
    RUDP --> VAL
    VAL -->|commande valide| NAV
    VAL -->|commande invalide| RUDP
```

**Commandes disponibles :**

| Commande | Paramètres | Effet |
|---|---|---|
| `ARM` | — | Arme le drone via FC |
| `DISARM` | — | Désarme si au sol |
| `TAKEOFF` | `altitude_m` | Décollage à l'altitude cible |
| `GOTO` | `lat, lon, alt_m` | Aller à un waypoint |
| `HOLD` | — | Maintien de position immédiat |
| `LAND` | — | Atterrissage sur place |
| `RTL` | — | Retour au point de décollage |
| `EMERGENCY` | — | Coupure moteurs (sol uniquement) |