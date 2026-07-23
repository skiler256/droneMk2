#include "drone/Components/MavlinkInterface/Conversions.hpp"
#include "drone/Components/MavlinkInterface/MavlinkInterface.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <cstring>

using namespace TYPES;
using namespace MAVLINK_CONV;

namespace {
// Identité MAVLink du Pi sur ce lien : usurpe l'identité GCS pour TOUS les
// messages émis (HEARTBEAT, COMMAND_LONG, GPS_INPUT, SET_POSITION_TARGET_
// LOCAL_NED...) — pas seulement le HEARTBEAT. Décidé en session : deux
// system id différents sur le même lien cassait le routage MAVProxy/le
// filtrage SYSID_MYGCS côté ArduPilot (SET_MESSAGE_INTERVAL jamais ACK'd
// tant que les commandes partaient sous une identité "onboard computer"
// distincte du HEARTBEAT).
constexpr uint8_t kGcsSystemId = 255;
constexpr uint8_t kGcsComponentId = MAV_COMP_ID_MISSIONPLANNER;
// Cible des messages adressés (SET_POSITION_TARGET_LOCAL_NED) : le FC lui-même.
constexpr uint8_t kFcSystemId = 1;
constexpr uint8_t kFcComponentId = MAV_COMP_ID_AUTOPILOT1;

// Au-delà de cet âge, un échantillon sf_/nav_ est considéré périmé : on
// ne l'envoie pas plutôt que de nourrir le FC avec une donnée obsolète
// (cf. MAVLINKINTERFACE.md §5 — la fraîcheur prime sur la continuité).
constexpr TYPES::Ms kMaxSampleAge{300};

// SET_POSITION_TARGET_LOCAL_NED : vitesse horizontale (vx,vy) + altitude
// absolue (z, pas vz — plus stable qu'un taux de montée intégré) + cap
// absolu (yaw). Position x/y, vz, accélération et yaw_rate ignorés.
constexpr uint16_t kPosTargetTypeMask =
    POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE |
    POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
    POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
} // namespace

// RT priorities et fréquences : placeholders raisonnables, à ajuster
// quand les loop() seront implémentées (étape suivante) — rien de figé ici.
MavlinkInterface::MavlinkInterface(ComponenConfig config,
                                   SharedSysStateMemHandler &sysState,
                                   SharedFCStatusHandler &fc, SharedNavMemHandler &nav,
                                   SharedSFMemHandler &sf, IMavlinkLink &link)
    : ComponentBase<5>(config, fc, sysState), fc_(fc), sysState_(sysState),
      nav_(nav), sf_(sf), link_(link),
      TTx(*this,
          {.id = 0,
           .RTpriority = 75,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{50}, // couvre SET_POSITION_TARGET_LOCAL_NED (50Hz)
           .timeout = TYPES::Ms{200}},
          localWD),
      TTxACK(*this,
             {.id = 1,
              .RTpriority = 70,
              .core = config.CompCore,
              .loopFrequency = TYPES::Hz{20},
              .timeout = TYPES::Ms{1000}}, // laisse la place au cycle retry 500ms
             localWD),
      TRx(*this,
          {.id = 2,
           .RTpriority = 75,
           .core = config.CompCore,
           .loopFrequency = TYPES::Hz{100}, // marge sur le plus rapide des flux RX (50Hz)
           .timeout = TYPES::Ms{200}},
          localWD),
      TCmd(*this,
           {.id = 3,
            .RTpriority = 65,
            .core = config.CompCore,
            .loopFrequency = TYPES::Hz{50},
            .timeout = TYPES::Ms{200}},
           localWD),
      TFailSafe(*this,
                {.id = 4,
                 .RTpriority = 80, // supervision failsafe : priorité la plus haute des 5
                 .core = config.CompCore,
                 .loopFrequency = TYPES::Hz{20},
                 .timeout = TYPES::Ms{500}},
                localWD)

{
  registerTask(TTx);
  registerTask(TTxACK);
  registerTask(TRx);
  registerTask(TCmd);
  registerTask(TFailSafe);
  activate();
};

namespace {
// Un message d'intérêt + le taux auquel on veut le recevoir. HEARTBEAT
// n'y figure pas : ArduPilot l'envoie à 1Hz sans qu'on ait besoin de le
// demander (ce n'est pas un flux au sens SET_MESSAGE_INTERVAL).
struct StreamRequest {
  uint32_t msgid;
  uint32_t intervalUs;
};

// Taux réduits par rapport aux cibles prod (50Hz attitude/position) — le
// lien SITL PC<->Pi passe par un wifi peu fiable qui perd quasiment tout le
// trafic au-delà du HEARTBEAT à ces débits (constaté en session, cf.
// rx_queue quasi toujours vide côté Pi malgré un %lost MAVProxy qui explose).
// À revoir une fois sur un lien filaire ou en conditions réelles (UART).
constexpr std::array<StreamRequest, 5> kInitStreams{{
    {MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100000}, // 10Hz
    {MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100000},  // 10Hz
    {MAVLINK_MSG_ID_BATTERY_STATUS, 1000000},     // 1Hz
    {MAVLINK_MSG_ID_ESC_STATUS, 500000},          // 2Hz
    {MAVLINK_MSG_ID_SCALED_PRESSURE, 500000},     // 2Hz
}};
} // namespace

void MavlinkInterface::requestInitStreams() {
  for (const auto &stream : kInitStreams) {
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(kGcsSystemId, kGcsComponentId, &msg, kFcSystemId,
                                  kFcComponentId, MAV_CMD_SET_MESSAGE_INTERVAL,
                                  /*confirmation=*/0, static_cast<float>(stream.msgid),
                                  static_cast<float>(stream.intervalUs), 0.0f, 0.0f,
                                  0.0f, 0.0f, 0.0f);

    PendingAck req{};
    req.kind = PendingKind::Command;
    req.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    req.frameLen =
        static_cast<size_t>(mavlink_msg_to_send_buffer(
            reinterpret_cast<uint8_t *>(req.frame.data()), &msg));

    if (!ackQueue_.push(req)) {
      // Queue pleine (kCapacity=8, 5 demandes ici — ne devrait jamais
      // arriver) : celles déjà poussées partiront quand même.
      break;
    }
  }
}

// FSTask::loop() reste vide plus longtemps que les autres (décision
// explicite, cf. MAVLINKINTERFACE.md §2/§8).
void FSTask::loop() {}

// TCmd dépend de MissionControl (n'existe pas encore, cf. §10) — rien à
// router pour l'instant, comme GCSRxTask de SysMonitoring au même stade.
void TCmdTask::loop() {}

namespace {
// Un seul point d'envoi (pack -> buffer -> send) pour les 3 messages TX
// routine, évite de dupliquer la sérialisation à chaque cas. Renvoie
// true sur succès — l'appelant ne doit avancer son "dernier envoyé" que
// dans ce cas (même logique que markSent() de PacketScheduler côté
// SysMonitoring) : un échec laisse la donnée "due", elle repart au tour
// suivant plutôt que d'être perdue silencieusement.
[[nodiscard]] bool packAndSend(IMavlinkLink &link, const mavlink_message_t &msg) {
  std::array<std::byte, MAVLINK_MAX_PACKET_LEN> buf{};
  uint16_t len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t *>(buf.data()), &msg);
  return !link.send(std::span<const std::byte>(buf.data(), len));
}
} // namespace

// HEARTBEAT : périodique pur, 1Hz, jamais événementiel — doit toujours
// partir pour ne jamais déclencher le failsafe GCS du FC (cf. §5).
// GPS_INPUT/SET_POSITION_TARGET_LOCAL_NED : événementiels — envoyés
// seulement si sf_/nav_ ont produit un nouvel échantillon (ts différent du
// dernier envoyé) ET que cet échantillon n'est pas déjà périmé. Pas de
// redondance périodique : une source muette doit se traduire par un
// silence radio, pas par la répétition d'une dernière valeur obsolète.
void TTxTask::loop() {
  auto now = Clock::now();

  if (UTILITIES::msBetween(comp.lastHeartbeatSent_, now) >= Ms{1000}) {
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(kGcsSystemId, kGcsComponentId, &msg, MAV_TYPE_GCS,
                              MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    if (packAndSend(comp.link_, msg))
      comp.lastHeartbeatSent_ = now;
  }

  if (auto sample = comp.sf_.getGpsMag()) {
    bool isNew = sample->ts != comp.lastGpsMagSampleTs_;
    bool isFresh = UTILITIES::msBetween(sample->ts, now) < kMaxSampleAge;

    if (isNew && isFresh) {
      int32_t latE7 = degToE7(sample->latitude);
      int32_t lonE7 = degToE7(sample->longitude);
      uint16_t yawCdeg = headingToYawCdeg(sample->heading);

      auto timeUsec = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(sample->ts.time_since_epoch())
              .count());

      mavlink_message_t msg{};
      mavlink_msg_gps_input_pack(
          kGcsSystemId, kGcsComponentId, &msg, timeUsec, /*gps_id=*/0, /*ignore_flags=*/0,
          /*time_week_ms=*/0, /*time_week=*/0, // TODO: semaine GPS non trackée, cf. §10
          sample->fixType, latE7, lonE7, sample->altMsl, sample->hdop, sample->vdop,
          sample->velNed.ned.x, sample->velNed.ned.y, sample->velNed.ned.z,
          sample->speedAccuracy, sample->horizAccuracy, sample->vertAccuracy,
          sample->satellitesVisible, yawCdeg);
      if (packAndSend(comp.link_, msg))
        comp.lastGpsMagSampleTs_ = sample->ts;
    }
  }

  if (auto cmd = comp.nav_.getCommand()) {
    bool isNew = cmd->ts != comp.lastNavCmdSampleTs_;
    bool isFresh = UTILITIES::msBetween(cmd->ts, now) < kMaxSampleAge;

    if (isNew && isFresh) {
      auto timeBootMs = static_cast<uint32_t>(
          std::chrono::duration_cast<Ms>(cmd->ts.time_since_epoch()).count());

      mavlink_message_t msg{};
      mavlink_msg_set_position_target_local_ned_pack(
          kGcsSystemId, kGcsComponentId, &msg, timeBootMs, kFcSystemId, kFcComponentId,
          MAV_FRAME_LOCAL_NED, kPosTargetTypeMask,
          /*x=*/0.0f, /*y=*/0.0f, /*z=*/cmd->altitudeCmd.v, cmd->velNed.ned.x,
          cmd->velNed.ned.y, /*vz=*/0.0f, /*afx=*/0.0f, /*afy=*/0.0f, /*afz=*/0.0f,
          /*yaw=*/cmd->headingCmd.v, /*yaw_rate=*/0.0f);
      if (packAndSend(comp.link_, msg))
        comp.lastNavCmdSampleTs_ = cmd->ts;
    }
  }
}

// TTxACK est la SEULE Task à modifier comp.pendingAck_ — pas de mutex
// nécessaire ici (contrairement à AckFifo, lu/écrit par deux Tasks).
void TTxAckTask::loop() {
  auto now = Clock::now();

  // 1) Traiter les ACK que TRx a routés depuis le dernier tour.
  while (auto msg = comp.ackFifo_.pop()) {
    if (comp.pendingAck_.kind == PendingKind::None) {
      // ACK reçu alors qu'on n'attend rien : FC en retard sur un ACK déjà
      // abandonné (timeout épuisé), ou désync — cf. CODES::MVC001.
      comp.sysState_.raiseCode(CODES::MVC001);
      continue;
    }

    if (ackMatchesPending(*msg, comp.pendingAck_))
      comp.pendingAck_.kind = PendingKind::None; // résolu, prêt pour la prochaine commande
    else
      comp.sysState_.raiseCode(CODES::MVC001); // ACK d'une autre commande (déjà abandonnée ?)
  }

  // 2) Retry/timeout de la commande en attente.
  if (comp.pendingAck_.kind != PendingKind::None) {
    if (UTILITIES::msBetween(comp.pendingAck_.sentAt, now) < Ms{500})
      return; // toujours dans le délai, rien à faire ce tour-ci

    if (comp.pendingAck_.retries >= 3) {
      // Échec définitif après 3 tentatives : cascade décidée mais pas
      // encore câblée (reconnexion UART, degraded, FSTask) — cf. §6/§10.
      comp.pendingAck_.kind = PendingKind::None;
      return;
    }

    ++comp.pendingAck_.retries;
    comp.pendingAck_.sentAt = now;
    // Échec d'écriture transitoire non traité spécifiquement ici : le
    // compteur de retry est déjà incrémenté, le prochain timeout (500ms)
    // retentera de toute façon jusqu'à épuisement des 3 tentatives.
    [[maybe_unused]] auto writeResult = comp.link_.send(std::span<const std::byte>(
        comp.pendingAck_.frame.data(), comp.pendingAck_.frameLen));
    return;
  }

  // 3) Rien en attente : dépiler la prochaine commande de ackQueue_
  // (alimentée par requestInitStreams() pour l'instant — TCmd/MissionControl
  // y poussera plus tard, cf. §10). La frame est déjà packée, il ne reste
  // qu'à l'envoyer et amorcer le suivi ACK/retry.
  if (auto req = comp.ackQueue_.pop()) {
    comp.pendingAck_ = *req;
    comp.pendingAck_.sentAt = now;
    comp.pendingAck_.retries = 0;
    [[maybe_unused]] auto sendResult = comp.link_.send(std::span<const std::byte>(
        comp.pendingAck_.frame.data(), comp.pendingAck_.frameLen));
  }
}

// TRx est la SEULE Task à appeler link_.poll() (cf. AckFifo.hpp) : deux
// lecteurs concurrents du même fd UART mélangeraient le flux d'octets.
// mavlink_parse_char() maintient son état de reconstruction (canal 0)
// entre deux appels à loop() — gère nativement les messages tronqués ou
// concaténés sur un flux d'octets brut.
void TRxTask::loop() {
  auto now = Clock::now();

  mavlink_message_t msg{};
  mavlink_status_t parseStatus{};

  // Draine TOUT ce qui est disponible ce tour-ci, pas un seul datagramme :
  // à 100Hz on ne lirait qu'au plus 100 paquets/s, alors que les flux par
  // défaut d'ArduPilot dépassent largement ce débit — le tampon noyau
  // s'accumule indéfiniment et on traite en permanence des données de plus
  // en plus vieilles (constaté en session face au SITL). Plafonné pour ne
  // pas monopoliser le coeur RT si le tampon est énorme (ex: après un
  // blocage) — le reste attend simplement le tour suivant.
  constexpr int kMaxDatagramsPerTick = 64;
  for (int datagramIdx = 0; datagramIdx < kMaxDatagramsPerTick; ++datagramIdx) {
    auto bytes = comp.link_.poll();
    if (!bytes)
      break; // NoNewData, ou erreur driver — plus rien à parser ce tour-ci

    for (std::byte b : *bytes) {
      if (!mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(b), &msg, &parseStatus))
        continue; // message incomplet ou CRC invalide : pas encore prêt

      switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_ACK:
    case MAVLINK_MSG_ID_PARAM_VALUE:
      // TTxACK dépile et matche à la commande en attente — TRx ne fait
      // que router, pas d'interprétation ici.
      comp.ackFifo_.push(msg);
      break;

    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
      mavlink_attitude_quaternion_t att{};
      mavlink_msg_attitude_quaternion_decode(&msg, &att);
      // q1=w, q2=x, q3=y, q4=z (cf. commentaire du champ dans le message).
      comp.fc_.setAttitude(Attitude{Quat{att.q1, att.q2, att.q3, att.q4}});
      break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
      mavlink_local_position_ned_t pos{};
      mavlink_msg_local_position_ned_decode(&msg, &pos);
      comp.fc_.setLocalPosVel(Position{Vec3{pos.x, pos.y, pos.z}},
                              Velocity{Vec3{pos.vx, pos.vy, pos.vz}});
      break;
    }

    case MAVLINK_MSG_ID_BATTERY_STATUS: {
      mavlink_battery_status_t bat{};
      mavlink_msg_battery_status_decode(&msg, &bat);
      FCBatteryStatus batStatus{.voltage = mvToVolts(bat.voltages[0]),
                                .current = caToAmps(bat.current_battery),
                                .remainingPct = clampBatteryPct(bat.battery_remaining)};
      comp.fc_.setBattery(batStatus);
      break;
    }

    case MAVLINK_MSG_ID_ESC_STATUS: {
      mavlink_esc_status_t esc{};
      mavlink_msg_esc_status_decode(&msg, &esc);
      FCEscStatus escStatus{};
      for (size_t i = 0; i < 4; ++i) {
        escStatus.rpmValue[i] = static_cast<float>(esc.rpm[i]);
        escStatus.currentAmps[i] = esc.current[i];
        escStatus.temperatureC[i] = 0.0f; // ESC_STATUS ne fournit pas de température
      }
      comp.fc_.setEscStatus(escStatus);
      break;
    }

    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
      mavlink_scaled_pressure_t pres{};
      mavlink_msg_scaled_pressure_decode(&msg, &pres);
      FCPressure presStatus{.absHpa = pres.press_abs,
                            .diffHpa = pres.press_diff,
                            .temperatureC = cdegcToC(pres.temperature)};
      comp.fc_.setPressure(presStatus);
      break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
      comp.fc_.setLastHeartbeat(now);
      break;

    default:
      break; // message reçu mais pas dans notre liste d'intérêt : ignoré
      }
    }
  }
}
