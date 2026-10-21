#pragma once
#include "drone/Components/SystemMonitoring/Driver.hpp"
#include <array>
#include <cstdint>
#include <span>

// Lien secondaire SDL (CC1101, sub-GHz SPI) : sous-ensemble Links=BOTH de
// TELEMETRY.csv, soit 19 paquets et 8 Hz cumulés. C'est le lien qui porte
// toutes les commandes montantes — armement, RTL, atterrissage, urgence.
//
// Deux contraintes structurantes, cf. SYSMONITORING.md §16 :
//  - aucune ligne GDO câblée : la réception se fait par interrogation du
//    registre RXBYTES, pas par interruption ;
//  - FIFO de 64 octets seulement, d'où un plafond sur la charge utile et un
//    débordement possible si poll() n'est pas appelé assez souvent.
class CC1101_Driver : public ITelemetryLink {
public:
  // Le quartz conditionne toute la synthèse de fréquence : un écart entre
  // les deux bouts décale la porteuse de plusieurs dizaines de MHz sans
  // qu'aucun symptôme ne l'indique (les deux se parlent, hors bande).
  explicit CC1101_Driver(const char *spiDevice = "/dev/spidev0.0",
                         uint32_t speedHz = 500000);
  ~CC1101_Driver() override;

  // Hors constructeur : un constructeur ne peut pas échouer proprement, et
  // la configuration de la puce peut échouer de six façons.
  // cf. ARCHITECTURE_IPC.md §4.1, même raison que ComponentWatchdog::start().
  [[nodiscard]] std::optional<TYPES::DriverError> init();

  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() override;

  [[nodiscard]] std::optional<TYPES::DriverError>
  send(std::span<const std::byte> payload) override;

  // Désactive le CCA le temps d'une trame, puis le rétablit. Le seul lien des
  // trois qui écoute avant d'émettre, donc le seul qui ait à redéfinir ceci.
  [[nodiscard]] std::optional<TYPES::DriverError>
  sendForced(std::span<const std::byte> payload) override;

  [[nodiscard]] LinkQuality quality() const override {
    return LinkQuality{.rssiDbm = rssiDbm_,
                       .lqi = lqi_,
                       .lossRatio = std::nullopt, // non mesurable sans numérotation
                       .crcErrors = crcErrors_,
                       .lengthErrors = lengthErrors_,
                       .overflows = overflows_,
                       .ccaRefusals = ccaRefusals_};
  }

  // Qualité du dernier paquet reçu, extraite des octets de statut que la
  // puce ajoute à chaque trame. Vide tant que rien n'a été reçu.
  [[nodiscard]] std::optional<int8_t> lastRssiDbm() const { return rssiDbm_; }
  [[nodiscard]] std::optional<uint8_t> lastLqi() const { return lqi_; }

  // Débordements de FIFO depuis le démarrage. Un compteur qui monte signale
  // que poll() n'est pas appelé assez souvent pour le trafic reçu.
  [[nodiscard]] uint32_t overflowCount() const { return overflows_; }

  // Trames rejetées à la réception. Sans ces compteurs, une liaison qui se
  // dégrade est indiscernable d'une liaison silencieuse : on ne peut pas
  // distinguer « personne ne parle » de « tout arrive corrompu ».
  [[nodiscard]] uint32_t crcErrorCount() const { return crcErrors_; }
  [[nodiscard]] uint32_t lengthErrorCount() const { return lengthErrors_; }

  // Émissions refusées par le CCA : le canal était occupé. Un compteur qui
  // monte n'est pas une panne, c'est la mesure de la contention — et c'est
  // précisément ce qu'on voulait voir en activant le CCA.
  [[nodiscard]] uint32_t ccaRefusalCount() const { return ccaRefusals_; }

  // Émissions ayant contourné le CCA. Doit rester rare : un compteur qui
  // monte signifie qu'on passe en force régulièrement, donc qu'on collisionne
  // délibérément — à surveiller, pas à ignorer.
  [[nodiscard]] uint32_t forcedSendCount() const { return forcedSends_; }

  // Dernière trame rejetée au CRC, pour diagnostic. Un CRC faux ne dit pas
  // COMMENT la trame est abîmée — quelques bits retournés, un octet répété,
  // du vide — et c'est cette forme qui désigne la cause.
  [[nodiscard]] std::span<const std::byte> lastRejected() const {
    return {rejectedBuf_.data(), rejectedLen_};
  }

  // Charge utile maximale théorique : 64 octets de FIFO moins l'octet de
  // longueur et les deux octets de statut.
  //
  // ATTENTION, mesuré : au-delà de ~50 octets la réception s'effondre (89 %
  // de perte à 55 o, 100 % à 60 o), parce qu'à 61 octets utiles la FIFO
  // contient exactement ses 64 octets de capacité, sans aucune marge de
  // synchronisation. Cf. BRINGUP_PDL.md. Le plus gros paquet Links=BOTH
  // (CMD_WAYPOINT, 24 octets depuis l'ajout de seq) reste bien en dessous.
  static constexpr size_t kMaxPayload = 61;

private:
  [[nodiscard]] std::optional<TYPES::DriverError> transfer(const uint8_t *tx,
                                                           uint8_t *rx,
                                                           size_t len);
  [[nodiscard]] std::optional<TYPES::DriverError> writeReg(uint8_t addr,
                                                           uint8_t value);
  [[nodiscard]] std::expected<uint8_t, TYPES::DriverError> readReg(uint8_t addr);
  // Les registres d'état exigent le bit de rafale, sans quoi la puce renvoie
  // une valeur périmée — piège classique du CC1101.
  [[nodiscard]] std::expected<uint8_t, TYPES::DriverError>
  readStatusReg(uint8_t addr);
  // Erratum CC1101 : RXBYTES lu pendant que la puce le met à jour renvoie un
  // compte corrompu. TI prescrit deux lectures concordantes.
  [[nodiscard]] std::expected<uint8_t, TYPES::DriverError> readRxBytesStable();
  [[nodiscard]] std::optional<TYPES::DriverError> flushRxAndListen();
  [[nodiscard]] std::optional<TYPES::DriverError> strobe(uint8_t cmd);
  [[nodiscard]] std::optional<TYPES::DriverError> applyConfig();

  int fd_{-1};
  const char *device_;
  uint32_t speedHz_;

  std::array<std::byte, kMaxPayload> rxBuf_{};
  std::optional<int8_t> rssiDbm_;
  std::optional<uint8_t> lqi_;
  uint32_t overflows_{0};
  uint32_t crcErrors_{0};
  uint32_t lengthErrors_{0};
  uint32_t ccaRefusals_{0};
  uint32_t forcedSends_{0};
  std::array<std::byte, kMaxPayload> rejectedBuf_{};
  size_t rejectedLen_{0};
  // Longueur déjà retirée de la FIFO alors que le paquet n'était pas encore
  // entièrement arrivé. poll() ne peut pas attendre — il tourne sur le chemin
  // RT — et l'octet de longueur est consommé par sa lecture : on le retient
  // pour reprendre au tour suivant au lieu de le relire, ce qui décalerait
  // tout le reste.
  uint8_t pendingLength_{0};
  // Instant a partir duquel le paquet en attente est certainement arrive.
  // RXBYTES annonce le paquet complet bien avant qu'il le soit ; lire sur sa
  // foi deregle le pointeur de FIFO de la puce, qui renvoie alors son dernier
  // octet en boucle. Sur un chemin RT on ne peut pas attendre, donc on arme
  // une echeance et on rend NoNewData jusqu'a elle.
  TYPES::TimePoint pendingReadyAt_{};
};
