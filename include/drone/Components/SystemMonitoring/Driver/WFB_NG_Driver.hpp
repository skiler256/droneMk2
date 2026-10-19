#pragma once
#include "drone/Components/SystemMonitoring/Driver/UdpTelemetryDriver.hpp"
#include <optional>

// Lien principal PDL (wfb-ng, wifi broadcast).
//
// wfb-ng EST un transport UDP : wfb_tx lit sur un port local et diffuse,
// wfb_rx reçoit et réécrit sur un port local. Le transport est donc celui
// d'UdpTelemetryDriver, dont cette classe hérite sans le réécrire. Ce qu'elle
// ajoute est la santé réelle du lien, lue sur le socket de statistiques de
// wfb-ng — sans quoi le drone ne sait pas s'il émet dans le vide.
//
// Dégradation : statistiques indisponibles -> le driver se comporte
// exactement comme UdpTelemetryDriver. Jamais de blocage sur leur absence.
// cf. SYSMONITORING.md §15.
class WFB_NG_Driver : public UdpTelemetryDriver {
public:
  // statsPort = 0 : pas de lecture de statistiques, transport seul.
  WFB_NG_Driver(uint16_t localPort, uint16_t peerPort, uint16_t statsPort);
  ~WFB_NG_Driver() override;

  // Transport hérité, plus une tentative de rafraîchissement des stats
  // limitée en cadence (cf. kStatsPeriod dans le .cpp).
  [[nodiscard]] std::expected<std::span<const std::byte>, TYPES::DriverError>
  poll() override;

  // Qualité du lien MONTANT, telle que vue par le récepteur du drone. La
  // qualité descendante se mesure au sol, pas ici.
  [[nodiscard]] std::optional<int8_t> rssiDbm() const { return rssiDbm_; }
  [[nodiscard]] std::optional<float> lossRatio() const { return lossRatio_; }

private:
  void refreshStats(TYPES::TimePoint now);
  bool connectStats();
  void dropStats();
  void parseStatsLine(std::string_view line);

  uint16_t statsPort_;
  int statsFd_{-1};
  TYPES::TimePoint lastStatsAttempt_{};
  TYPES::TimePoint lastStatsOk_{};

  // Les lignes du socket de stats arrivent par morceaux : on accumule
  // jusqu'au '\n' plutôt que de supposer une ligne par lecture.
  std::array<char, 4096> statsBuf_{};
  size_t statsLen_{0};

  std::optional<int8_t> rssiDbm_;
  std::optional<float> lossRatio_;
};
