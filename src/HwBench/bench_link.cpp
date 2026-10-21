// Banc HIL de qualification d'un lien radio : latence, perte, débit, portée.
//
// Un seul binaire pour les deux liens, derrière ITelemetryLink :
//   --link=cc1101   lien secondaire SDL (SPI, ~9,6 kbit/s, 61 o utiles)
//   --link=wfb      lien principal PDL (wfb-ng, UDP loopback vers wfb_tx/rx)
//   --link=udp      UDP loopback nu — pour valider le banc sans radio
//
// Le pendant au sol est tools/hw_debug/link_bench_gcs.py (PDL), ou le
// firmware GCS/ESP32/cc1101_link en mode echo (SDL).
//
// MESURE DE LATENCE — le drone émet une trame horodatée, le répondeur la
// renvoie telle quelle, le drone calcule l'aller-retour. Une seule horloge
// est donc en jeu : aucune synchronisation entre les deux machines, aucune
// dérive à corriger. C'est la seule façon honnête de mesurer une latence
// sans NTP/PTP entre les deux bouts.
//
// Usage : ./bench_link <mode> [options]
//   ping     une trame toutes les --interval ms, mesure RTT et perte
//   flood    émission saturée pendant --duration s, mesure le débit utile
//   range    émission lente et continue, RSSI/LQI en direct (CSV)
//   ramp     fait varier la taille des trames, cherche le seuil d'erreur
//
// Options : --link=  --interval=ms  --duration=s  --size=octets  --csv=fichier
//           --device=  --local-port=  --peer-port=  --stats-port=  --spi-hz=

#include "drone/Components/SystemMonitoring/Driver/CC1101_Driver.hpp"
#include "drone/Components/SystemMonitoring/Driver/UdpTelemetryDriver.hpp"
#include "drone/Components/SystemMonitoring/Driver/WFB_NG_Driver.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> running{true};
void onSignal(int) { running.store(false); }

using Clock = std::chrono::steady_clock;

// ───────────────────────── trame de banc ─────────────────────────
//
// Disposition explicite, petit-boutien, même convention que les paquets
// générés de TELEM : memcpy à offsets fixes plutôt qu'un attribut packed.
// C'est ce qui garantit que le C++, le Python du sol et le C++ de l'ESP32
// voient exactement les mêmes octets.
//
//   0  u16  magic    0x4B4C
//   2  u8   version  1
//   3  u8   mode
//   4  u32  seq
//   8  u64  txNs     horloge monotone du drone, réémise telle quelle
//  16  i8   farRssi  rempli par le répondeur
//  17  u8   farLqi   rempli par le répondeur
//  18  u8   farFlags bit 0 : farRssi/farLqi renseignés
//  19  u8   reserved
//
// Le magic et la version ne sont pas décoratifs : le même lien porte la
// télémétrie réelle, et un paquet TELEM ne doit jamais être compté comme
// un écho de banc.
// Stream : sens unique a cadence reglee, sans echo. C'est le mode qui repond
// a « ce lien peut-il porter mes N paquets par seconde ? » — flood emet sans
// limite et ne dit rien d'une cadence donnee, ping impose un aller-retour qui
// coute le double en temps d'air.
enum class Mode : uint8_t {
  Ping = 1,
  Flood = 2,
  Range = 3,
  Ramp = 4,
  Stream = 5
};

struct BenchFrame {
  static constexpr uint16_t kMagic = 0x4B4C; // "LK"
  static constexpr uint8_t kVersion = 1;
  static constexpr size_t kSize = 20;
  static constexpr uint8_t kFarValid = 0x01;

  uint16_t magic{kMagic};
  uint8_t version{kVersion};
  uint8_t mode{};
  uint32_t seq{};
  uint64_t txNs{};
  int8_t farRssi{};
  uint8_t farLqi{};
  uint8_t farFlags{};
  uint8_t reserved{};

  void packInto(std::byte *buf) const noexcept {
    std::memcpy(buf + 0, &magic, sizeof(magic));
    std::memcpy(buf + 2, &version, sizeof(version));
    std::memcpy(buf + 3, &mode, sizeof(mode));
    std::memcpy(buf + 4, &seq, sizeof(seq));
    std::memcpy(buf + 8, &txNs, sizeof(txNs));
    std::memcpy(buf + 16, &farRssi, sizeof(farRssi));
    std::memcpy(buf + 17, &farLqi, sizeof(farLqi));
    std::memcpy(buf + 18, &farFlags, sizeof(farFlags));
    std::memcpy(buf + 19, &reserved, sizeof(reserved));
  }

  [[nodiscard]] static bool unpack(std::span<const std::byte> bytes,
                                   BenchFrame &out) noexcept {
    if (bytes.size() < kSize)
      return false;
    std::memcpy(&out.magic, bytes.data() + 0, sizeof(out.magic));
    std::memcpy(&out.version, bytes.data() + 2, sizeof(out.version));
    std::memcpy(&out.mode, bytes.data() + 3, sizeof(out.mode));
    std::memcpy(&out.seq, bytes.data() + 4, sizeof(out.seq));
    std::memcpy(&out.txNs, bytes.data() + 8, sizeof(out.txNs));
    std::memcpy(&out.farRssi, bytes.data() + 16, sizeof(out.farRssi));
    std::memcpy(&out.farLqi, bytes.data() + 17, sizeof(out.farLqi));
    std::memcpy(&out.farFlags, bytes.data() + 18, sizeof(out.farFlags));
    std::memcpy(&out.reserved, bytes.data() + 19, sizeof(out.reserved));
    return out.magic == kMagic && out.version == kVersion;
  }
};

// ───────────────────────── options ─────────────────────────

struct Options {
  std::string mode{"ping"};
  std::string link{"cc1101"};
  std::string device{"/dev/spidev0.0"};
  std::string csvPath;
  uint32_t spiHz{500000};
  uint32_t intervalMs{100};
  uint32_t durationS{20};
  size_t frameSize{32};
  uint16_t localPort{5611};
  uint16_t peerPort{5612};
  uint16_t statsPort{0};
  bool forced{false};
  bool localPortSet{false};
  bool peerPortSet{false};
};

bool parseUint(std::string_view value, uint32_t &out) {
  if (value.empty())
    return false;
  char *end = nullptr;
  const unsigned long v = std::strtoul(std::string(value).c_str(), &end, 10);
  if (end == nullptr || *end != '\0' || v > 0xFFFFFFFFUL)
    return false;
  out = static_cast<uint32_t>(v);
  return true;
}

bool parseArgs(int argc, char **argv, Options &opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};
    if (arg == "--forced") {
      // Exerce sendForced() : contourne le CCA le temps de chaque trame.
      // Sert a verifier sur le materiel que le contournement emet bien et
      // que le CCA est retabli apres — un oubli de retablissement
      // transformerait l'exception en comportement permanent.
      opt.forced = true;
      continue;
    }
    if (!arg.starts_with("--")) {
      opt.mode = std::string(arg);
      continue;
    }
    const size_t eq = arg.find('=');
    if (eq == std::string_view::npos) {
      std::cerr << "option sans valeur : " << arg << "\n";
      return false;
    }
    const std::string_view key = arg.substr(2, eq - 2);
    const std::string_view val = arg.substr(eq + 1);
    uint32_t n = 0;

    if (key == "link")
      opt.link = std::string(val);
    else if (key == "device")
      opt.device = std::string(val);
    else if (key == "csv")
      opt.csvPath = std::string(val);
    else if (key == "spi-hz" && parseUint(val, n))
      opt.spiHz = n;
    else if (key == "interval" && parseUint(val, n))
      opt.intervalMs = n;
    else if (key == "duration" && parseUint(val, n))
      opt.durationS = n;
    else if (key == "size" && parseUint(val, n))
      opt.frameSize = n;
    else if (key == "local-port" && parseUint(val, n)) {
      opt.localPort = static_cast<uint16_t>(n);
      opt.localPortSet = true;
    } else if (key == "peer-port" && parseUint(val, n)) {
      opt.peerPort = static_cast<uint16_t>(n);
      opt.peerPortSet = true;
    }
    else if (key == "stats-port" && parseUint(val, n))
      opt.statsPort = static_cast<uint16_t>(n);
    else {
      std::cerr << "option inconnue ou valeur invalide : " << arg << "\n";
      return false;
    }
  }
  return true;
}

// ───────────────────────── statistiques ─────────────────────────

// Les échantillons sont conservés pour pouvoir donner les centiles : une
// moyenne seule masque exactement ce qui compte sur un lien de commande,
// à savoir les quelques aller-retours très lents.
struct Stats {
  std::vector<double> rttMs;
  uint32_t sent{0};
  uint32_t echoes{0};
  uint32_t reordered{0};
  // Deux causes bien distinctes, qu'il ne faut pas confondre : une trame que
  // le pilote a rejetee (CRC faux, longueur aberrante) signale un probleme de
  // reception ; une trame valide qui n'est pas du banc est de la telemetrie
  // reelle ou du bruit, et n'est pas un defaut.
  uint32_t rejetees{0};
  uint32_t horsBanc{0};
  uint32_t sendFail{0};
  uint32_t lastSeqSeen{0};
  bool haveLastSeq{false};
  long localRssiSum{0};
  long localLqiSum{0};
  uint32_t localSamples{0};
  long farRssiSum{0};
  long farLqiSum{0};
  uint32_t farSamples{0};
  size_t payloadBytes{0};

  void addRtt(double ms) { rttMs.push_back(ms); }

  [[nodiscard]] double centile(double p) const {
    if (rttMs.empty())
      return 0.0;
    std::vector<double> tri = rttMs;
    std::sort(tri.begin(), tri.end());
    const size_t idx = static_cast<size_t>(
        p / 100.0 * static_cast<double>(tri.size() - 1) + 0.5);
    return tri[std::min(idx, tri.size() - 1)];
  }

  [[nodiscard]] double moyenne() const {
    if (rttMs.empty())
      return 0.0;
    double s = 0.0;
    for (double v : rttMs)
      s += v;
    return s / static_cast<double>(rttMs.size());
  }

  // Écart type : la gigue, qui décide si un lien est utilisable pour du
  // contrôle même quand la latence moyenne est bonne.
  [[nodiscard]] double ecartType() const {
    if (rttMs.size() < 2)
      return 0.0;
    const double m = moyenne();
    double s = 0.0;
    for (double v : rttMs)
      s += (v - m) * (v - m);
    return std::sqrt(s / static_cast<double>(rttMs.size() - 1));
  }
};

// Statistiques de la seconde en cours. Un cumul depuis le debut masque ce
// qui vient de changer : c'est l'instantane qui sert quand on marche avec le
// drone ou qu'on oriente une antenne.
struct Fenetre {
  uint32_t sent{0};
  uint32_t echoes{0};
  uint32_t sendFail{0};
  size_t octets{0};
  double rttSum{0.0};
  uint32_t rttCount{0};
  int lastRssi{0};
  int lastFar{0};
  bool haveRssi{false};
  bool haveFar{false};

  void reset() { *this = Fenetre{}; }
};

// ───────────────────────── une voie de mesure ─────────────────────────
//
// Le banc mesure plusieurs liens EN MEME TEMPS, chacun avec sa propre
// numerotation, ses propres statistiques et son propre rythme d'emission.
//
// Pourquoi simultanement : mesurer le PDL puis le SDL l'un apres l'autre ne
// dit rien de ce qui se passe quand les deux fonctionnent ensemble. Les deux
// radios sont a quelques centimetres l'une de l'autre sur le drone, et c'est
// precisement en vol qu'elles cohabitent. Une mesure cote a cote, dans les
// memes conditions, est aussi la seule facon de comparer honnetement un lien
// de secours a un lien principal.
struct Voie {
  std::string nom;
  std::unique_ptr<ITelemetryLink> lien;
  CC1101_Driver *radio = nullptr; // pour RSSI/LQI et le plafond de charge
  WFB_NG_Driver *wfb = nullptr;   // pour les stats wfb-ng
  size_t maxPayload = 1024;

  // Numerotation PROPRE a la voie : la perte doit etre imputable a un lien,
  // pas diluee dans un compteur commun.
  uint32_t seq = 0;
  Clock::time_point prochainEnvoi{};
  size_t taillePalier = 0;
  size_t palierIdx = 0;
  std::vector<size_t> paliers;

  Stats st;
  Fenetre fen;
  Stats palierStats;
};


void afficherDirect(const Options &opt, const Stats &st, const Fenetre &f,
                    double ecoule) {
  if (opt.mode == "flood" || opt.mode == "stream") {
    const double debit = f.octets > 0 ? static_cast<double>(f.octets) * 8.0 / 1000.0 : 0.0;
    std::printf("[%4.0fs] emises %-8u refus %-9u %7.1f kbit/s  %6u trames/s\n",
                ecoule, st.sent, st.sendFail, debit, f.sent);
    std::fflush(stdout);
    return;
  }

  // Perte CUMULEE, pas celle de la fenetre : des que l'aller-retour depasse
  // la seconde, les echos arrivent dans une fenetre posterieure a celle de
  // leur envoi, et le rapport local donne des valeurs impossibles — on a vu
  // -166 % sur le PDL. Le cumul, lui, reste juste.
  const double perte =
      st.sent == 0 ? 0.0
                   : 100.0 * (1.0 - static_cast<double>(st.echoes) /
                                        static_cast<double>(st.sent));
  const double rtt =
      f.rttCount == 0 ? 0.0 : f.rttSum / static_cast<double>(f.rttCount);

  std::printf("[%4.0fs] emises %-5u echos %-5u perte %5.1f %%  rtt %7.2f ms",
              ecoule, st.sent, st.echoes, perte, rtt);
  if (f.haveRssi)
    std::printf("  ici %4d dBm", f.lastRssi);
  if (f.haveFar)
    std::printf("  face %4d dBm", f.lastFar);
  if (f.rttCount == 0 && f.sent > 0)
    std::printf("   (aucun echo cette seconde)");
  std::printf("\n");
  std::fflush(stdout);
}

void afficherBilan(const Options &opt, const Stats &st, double dureeS,
                   const std::string &nomVoie) {
  std::cout << "\n──────── bilan ────────\n";
  std::cout << "lien              : " << nomVoie << "   mode : " << opt.mode
            << "\n";
  std::cout << "trames emises     : " << st.sent << "\n";
  if (st.sendFail > 0) {
    const double refus =
        100.0 * static_cast<double>(st.sendFail) /
        static_cast<double>(st.sendFail + st.sent);
    std::cout << "envois refuses    : " << st.sendFail << "   (" << refus
              << " % de la charge offerte)\n"
              << "  -> le lien n'a pas pu absorber la cadence demandee ;\n"
                 "     c'est son plafond, pas une perte radio.\n";
  }

  if (opt.mode == "flood") {
    // En saturation le répondeur n'écho pas : le débit mesuré ici est celui
    // de l'émission, et c'est l'outil au sol qui dit combien sont arrivées.
    const double octets = static_cast<double>(st.payloadBytes);
    std::cout << "octets emis       : " << st.payloadBytes << "\n";
    if (dureeS > 0.0) {
      std::cout << "debit emission    : " << (octets * 8.0 / dureeS / 1000.0)
                << " kbit/s  (" << (static_cast<double>(st.sent) / dureeS)
                << " trames/s)\n";
    }
    std::cout << "\nLe taux de reception est a lire sur l'outil au sol : sans\n"
                 "echo le banc ne peut rien savoir de ce qui est arrive. Ne\n"
                 "PAS lire ce debit comme un debit livre.\n";
    return;
  }

  std::cout << "echos recus       : " << st.echoes;
  if (st.sent > 0) {
    const double perte = 100.0 * (1.0 - static_cast<double>(st.echoes) /
                                            static_cast<double>(st.sent));
    std::cout << "   perte : " << perte << " %";
  }
  std::cout << "\n";
  std::cout << "desordonnees      : " << st.reordered << "\n";
  std::cout << "rejetees par le pilote : " << st.rejetees
            << "   (CRC faux ou longueur aberrante — defaut de reception)\n";
  if (st.horsBanc > 0)
    std::cout << "hors banc         : " << st.horsBanc
              << "   (trames valides etrangeres au banc, pas un defaut)\n";

  // Les deux bouts comptent separement : si le repondeur a renvoye plus
  // d'echos qu'on en a recus, la perte est sur le trajet de RETOUR. C'est
  // ce que l'outil au sol affiche, et c'est la comparaison qui localise le
  // sens fautif — information qu'aucun des deux cotes ne donne seul.
  if (st.rejetees > 0 && st.sent > st.echoes &&
      st.rejetees * 2 >= st.sent - st.echoes)
    std::cout << "  -> les rejets expliquent au moins la moitie des echos\n"
                 "     manquants : la perte est surtout sur le trajet de\n"
                 "     RETOUR, pas a l'aller. A confirmer en comparant avec\n"
                 "     le nombre d'echos renvoyes qu'affiche l'outil au sol.\n";

  if (!st.rttMs.empty()) {
    const auto mn = *std::min_element(st.rttMs.begin(), st.rttMs.end());
    const auto mx = *std::max_element(st.rttMs.begin(), st.rttMs.end());
    std::cout << "aller-retour (ms) : min " << mn << "   moy " << st.moyenne()
              << "   p95 " << st.centile(95.0) << "   max " << mx << "\n";
    std::cout << "gigue (ecart-type): " << st.ecartType() << " ms\n";
    if (dureeS > 0.0) {
      const double octets = static_cast<double>(st.payloadBytes);
      std::cout << "debit utile       : " << (octets * 8.0 / dureeS / 1000.0)
                << " kbit/s aller-retour\n";
    }
  } else {
    std::cout << "aller-retour      : aucun echantillon\n";
  }

  // Sur un lien semi-duplex, un intervalle plus court que l'aller-retour
  // garantit que l'echo revient pendant que l'on emet — et la radio ne peut
  // pas recevoir en emettant. La perte est alors totale, pas progressive :
  // mesure a l'appui, 0 % de perte a 200 ms d'intervalle pour 129 ms de RTT,
  // et 100 % a 100 ms. Sans cet avertissement, on conclurait a un probleme
  // radio la ou il n'y a qu'un rythme intenable.
  if (opt.mode != "flood" && opt.mode != "stream" && !st.rttMs.empty()) {
    const double moy = st.moyenne();
    if (moy > static_cast<double>(opt.intervalMs) * 0.9) {
      std::cout << "\nATTENTION : l'intervalle demande (" << opt.intervalMs
                << " ms) n'excede pas l'aller-retour (" << moy << " ms).\n"
                   "  Sur un lien semi-duplex l'echo revient pendant qu'on\n"
                   "  emet, et la radio ne peut pas recevoir en emettant.\n"
                   "  Prendre --interval superieur a l'aller-retour mesure\n"
                   "  (ici au moins " << static_cast<int>(moy * 1.2 + 0.5)
                << " ms) avant de conclure a un defaut.\n";
    }
  } else if (opt.mode != "flood" && opt.mode != "stream" &&
             st.sent > 10 && st.echoes == 0 &&
             opt.intervalMs < 200) {
    std::cout << "\nATTENTION : aucun echo, et l'intervalle est court ("
              << opt.intervalMs << " ms).\n"
                 "  Sur SDL l'aller-retour vaut environ 130 ms : en dessous,\n"
                 "  chaque echo collisionne avec l'emission suivante et la\n"
                 "  perte est totale. Essayer --interval=200 avant de\n"
                 "  chercher ailleurs.\n";
  }

  if (st.localSamples > 0) {
    std::cout << "qualite ici       : rssi "
              << (st.localRssiSum / static_cast<long>(st.localSamples))
              << " dBm   lqi "
              << (st.localLqiSum / static_cast<long>(st.localSamples)) << "\n";
  }
  if (st.farSamples > 0) {
    std::cout << "qualite en face   : rssi "
              << (st.farRssiSum / static_cast<long>(st.farSamples))
              << " dBm   lqi "
              << (st.farLqiSum / static_cast<long>(st.farSamples)) << "\n";
  }
}

} // namespace

int main(int argc, char **argv) {
  // Sortie ligne par ligne : a travers ssh, stdout est un tuyau et donc
  // bufferise par blocs — rien n'apparaitrait avant la fin du banc.
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  std::cout.setf(std::ios::unitbuf);

  Options opt;
  if (!parseArgs(argc, argv, opt))
    return 2;

  if (opt.mode != "ping" && opt.mode != "flood" && opt.mode != "range" &&
      opt.mode != "ramp" && opt.mode != "stream") {
    std::cerr << "mode inconnu : " << opt.mode
              << "   (ping | flood | range | ramp | stream)\n";
    return 2;
  }
  // Sans echo, le banc ne peut rien savoir de ce qui est arrive : le chiffre
  // utile est celui de l'outil au sol. On le dit, pour que personne ne lise
  // le debit d'emission comme un debit livre.
  const bool sansEcho = opt.mode == "flood" || opt.mode == "stream";

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  // ── construction des voies ──
  //
  // --link accepte une liste : « --link=cc1101,wfb » mesure les deux liens
  // en meme temps, chacun avec sa numerotation et ses statistiques.
  std::vector<std::string> noms;
  {
    std::string courant;
    for (char c : opt.link) {
      if (c == ',') {
        if (!courant.empty())
          noms.push_back(courant);
        courant.clear();
      } else {
        courant += c;
      }
    }
    if (!courant.empty())
      noms.push_back(courant);
  }
  if (noms.empty()) {
    std::cerr << "aucun lien demande.\n";
    return 2;
  }

  std::vector<Voie> voies;
  for (const auto &nom : noms) {
    Voie v;
    v.nom = nom;
    if (nom == "cc1101") {
      auto cc = std::make_unique<CC1101_Driver>(opt.device.c_str(), opt.spiHz);
      std::cout << "init CC1101 sur " << opt.device << " a " << opt.spiHz
                << " Hz ...\n";
      if (auto err = cc->init()) {
        std::cerr << "echec init() : " << TYPES::toString(*err) << "\n";
        return 1;
      }
      v.maxPayload = CC1101_Driver::kMaxPayload;
      v.radio = cc.get();
      v.lien = std::move(cc);
    } else if (nom == "wfb") {
      auto w = std::make_unique<WFB_NG_Driver>(opt.localPort, opt.peerPort,
                                               opt.statsPort);
      v.wfb = w.get();
      v.lien = std::move(w);
      std::cout << "wfb-ng : local " << opt.localPort << ", pair "
                << opt.peerPort << "\n";
    } else if (nom == "udp") {
      // Les ports par defaut du lien de dev (5611/5612) sont ceux de TelSec,
      // et le dashboard occupe deja 5602/5612 : sans ce decalage le banc et
      // le repondeur ne se parleraient pas, et la perte afficherait 100 %
      // sans dire pourquoi.
      const uint16_t local = opt.localPortSet ? opt.localPort : 15611;
      const uint16_t pair = opt.peerPortSet ? opt.peerPort : 15612;
      v.lien = std::make_unique<UdpTelemetryDriver>(local, pair);
      std::cout << "UDP loopback : local " << local << ", pair " << pair
                << "  (repondeur sur CETTE machine)\n";
    } else {
      std::cerr << "lien inconnu : " << nom << "   (cc1101 | wfb | udp)\n";
      return 2;
    }

    if (v.lien->getHealth() == TYPES::DriverHealth::Unconnected) {
      std::cerr << nom << " : lien non connecte apres construction.\n";
      return 1;
    }

    // Taille de trame bornee par CE lien : le SDL plafonne a 61 octets quand
    // le PDL en accepte 1024.
    v.taillePalier = std::clamp(opt.frameSize, BenchFrame::kSize, v.maxPayload);

    // Un nombre de paliers constant, et non un pas constant : sur le PDL un
    // pas de 8 octets donnerait 126 paliers de 4 trames chacun, echantillon
    // trop maigre pour que le taux d'erreur veuille dire quoi que ce soit.
    if (opt.mode == "ramp") {
      constexpr size_t kNbPaliers = 8;
      const size_t etendue = v.maxPayload - BenchFrame::kSize;
      const size_t pas = std::max<size_t>(1, etendue / (kNbPaliers - 1));
      for (size_t t = BenchFrame::kSize; t < v.maxPayload; t += pas)
        v.paliers.push_back(t);
      v.paliers.push_back(v.maxPayload);
      v.taillePalier = v.paliers[0];
    }

    if (nom == "cc1101" && sansEcho) {
      std::cout
          << "\nATTENTION, deux reserves sur ce mode en SDL :\n"
             "  1. il viole franchement le rapport cyclique admis en 433 MHz ;\n"
             "  2. le debit annonce est celui de l'EMISSION, pas du livre :\n"
             "     le recepteur d'en face decroche bien avant la saturation.\n";
    }
    voies.push_back(std::move(v));
  }

  // ── fichier CSV ──
  std::FILE *csv = nullptr;
  if (!opt.csvPath.empty()) {
    csv = std::fopen(opt.csvPath.c_str(), "w");
    if (csv == nullptr) {
      std::cerr << "impossible d'ouvrir " << opt.csvPath << "\n";
      return 1;
    }
    // Colonne `lien` : sans elle, un CSV de mesure simultanee serait
    // inexploitable, les deux liens etant melanges.
    std::fprintf(csv, "t_ms,lien,seq,taille,rtt_ms,rssi_local,lqi_local,"
                      "rssi_distant,lqi_distant\n");
  }

  const uint32_t interval = opt.mode == "range"  ? std::max(opt.intervalMs, 1000u)
                            : opt.mode == "flood" ? 0u
                                                  : opt.intervalMs;

  std::cout << "\nmode " << opt.mode << "   duree " << opt.durationS << " s";
  if (interval > 0)
    std::cout << "   intervalle " << interval << " ms";
  std::cout << "\nvoies :";
  for (const auto &v : voies)
    std::cout << " " << v.nom << "(" << v.taillePalier << "o)";
  std::cout << "\nCtrl-C pour arreter et afficher le bilan.\n\n";

  std::vector<std::byte> txBuf(1024);
  const auto debut = Clock::now();
  const auto fin = debut + std::chrono::seconds(opt.durationS);
  // Les trames encore en vol a l'instant final seraient comptees perdues :
  // sur SDL l'aller-retour vaut 130 ms, soit une a deux trames injustement
  // portees au passif.
  constexpr auto kGrace = std::chrono::milliseconds(500);
  const auto finEcoute = fin + kGrace;

  for (auto &v : voies)
    v.prochainEnvoi = debut;

  auto prochainAffichage = debut + std::chrono::seconds(1);

  while (running.load() && Clock::now() < finEcoute) {
    const auto maintenant = Clock::now();
    const bool encoreEmettre = maintenant < fin;

    // ── affichage en direct, une ligne par voie ──
    if (opt.mode != "range" && maintenant >= prochainAffichage) {
      const double ecoule = std::chrono::duration<double>(maintenant - debut).count();
      for (auto &v : voies) {
        std::printf("%-7s ", v.nom.c_str());
        afficherDirect(opt, v.st, v.fen, ecoule);
        v.fen.reset();
      }
      prochainAffichage += std::chrono::seconds(1);
    }

    // ── emission, voie par voie ──
    for (auto &v : voies) {
      if (!encoreEmettre || maintenant < v.prochainEnvoi)
        continue;

      if (opt.mode == "ramp" && !v.paliers.empty()) {
        const double avance =
            std::chrono::duration<double>(maintenant - debut).count() /
            static_cast<double>(opt.durationS);
        const size_t idx = std::min(
            static_cast<size_t>(avance * static_cast<double>(v.paliers.size())),
            v.paliers.size() - 1);
        if (idx != v.palierIdx) {
          const double perte =
              v.palierStats.sent == 0
                  ? 0.0
                  : 100.0 * (1.0 - static_cast<double>(v.palierStats.echoes) /
                                       static_cast<double>(v.palierStats.sent));
          std::printf("%-7s palier %4zu o : %3u emises, %3u echos, "
                      "perte %5.1f %%, rtt moy %6.2f ms\n",
                      v.nom.c_str(), v.paliers[v.palierIdx], v.palierStats.sent,
                      v.palierStats.echoes, perte, v.palierStats.moyenne());
          std::fflush(stdout);
          v.palierIdx = idx;
          v.palierStats = Stats{};
        }
        v.taillePalier = v.paliers[v.palierIdx];
      }

      BenchFrame out;
      out.mode = static_cast<uint8_t>(
          opt.mode == "ping"     ? Mode::Ping
          : opt.mode == "flood"  ? Mode::Flood
          : opt.mode == "range"  ? Mode::Range
          : opt.mode == "stream" ? Mode::Stream
                                 : Mode::Ramp);
      out.seq = v.seq;
      out.txNs = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              Clock::now().time_since_epoch())
              .count());
      out.packInto(txBuf.data());
      // Bourrage incremental, pas des zeros : une troncature devient visible.
      for (size_t i = BenchFrame::kSize; i < v.taillePalier; ++i)
        txBuf[i] = static_cast<std::byte>(i & 0xFF);

      const auto trame =
          std::span<const std::byte>(txBuf.data(), v.taillePalier);
      if (auto err = opt.forced ? v.lien->sendForced(trame)
                                : v.lien->send(trame)) {
        // Un envoi refuse n'avance pas le compteur : sinon les numeros vus au
        // sol mesureraient la somme perte + refus.
        ++v.st.sendFail;
        ++v.fen.sendFail;
        if (*err != TYPES::DriverError::Timeout &&
            *err != TYPES::DriverError::NoNewData)
          ++v.st.rejetees;
      } else {
        ++v.seq;
        ++v.st.sent;
        ++v.palierStats.sent;
        v.st.payloadBytes += v.taillePalier;
        ++v.fen.sent;
        v.fen.octets += v.taillePalier;
      }
      v.prochainEnvoi = maintenant + std::chrono::milliseconds(interval);
    }

    // ── ecoute des echos, voie par voie ──
    for (auto &v : voies) {
      auto res = v.lien->poll();
      if (!res) {
        if (res.error() != TYPES::DriverError::NoNewData &&
            res.error() != TYPES::DriverError::Timeout) {
          ++v.st.rejetees;
          // Montrer les octets : un CRC faux ne dit pas COMMENT la trame est
          // abimee, et c'est cette forme qui designe la cause.
          if (v.radio != nullptr && v.st.rejetees <= 4) {
            auto bytes = v.radio->lastRejected();
            std::printf("%-7s REJET %zu o |", v.nom.c_str(), bytes.size());
            for (size_t i = 0; i < bytes.size(); ++i)
              std::printf(" %02X", static_cast<unsigned>(bytes[i]));
            std::printf("\n");
            std::fflush(stdout);
          }
        }
        continue;
      }

      BenchFrame in;
      if (!BenchFrame::unpack(*res, in)) {
        // Trame valide mais pas du banc : telemetrie reelle ou bruit. Ce
        // n'est pas un defaut de reception, d'ou un compteur separe.
        ++v.st.horsBanc;
        continue;
      }

      const auto t = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              Clock::now().time_since_epoch())
              .count());
      const double rtt =
          t > in.txNs ? static_cast<double>(t - in.txNs) / 1e6 : 0.0;

      ++v.st.echoes;
      ++v.palierStats.echoes;
      v.st.addRtt(rtt);
      v.palierStats.addRtt(rtt);
      ++v.fen.echoes;
      v.fen.rttSum += rtt;
      ++v.fen.rttCount;

      if (v.st.haveLastSeq && in.seq < v.st.lastSeqSeen)
        ++v.st.reordered;
      v.st.lastSeqSeen = in.seq;
      v.st.haveLastSeq = true;

      if (v.radio != nullptr) {
        if (auto r = v.radio->lastRssiDbm()) {
          v.fen.lastRssi = *r;
          v.fen.haveRssi = true;
          v.st.localRssiSum += *r;
          if (auto l = v.radio->lastLqi())
            v.st.localLqiSum += *l;
          ++v.st.localSamples;
        }
      } else if (v.wfb != nullptr) {
        if (auto r = v.wfb->rssiDbm()) {
          v.fen.lastRssi = *r;
          v.fen.haveRssi = true;
          v.st.localRssiSum += *r;
          ++v.st.localSamples;
        }
      }
      if ((in.farFlags & BenchFrame::kFarValid) != 0) {
        v.fen.lastFar = in.farRssi;
        v.fen.haveFar = true;
        v.st.farRssiSum += in.farRssi;
        v.st.farLqiSum += in.farLqi;
        ++v.st.farSamples;
      }

      if (opt.mode == "range") {
        std::printf("%-7s seq %6u   rtt %7.2f ms   ici %4d dBm / lqi %3d   "
                    "en face %4d dBm / lqi %3d\n",
                    v.nom.c_str(), in.seq, rtt,
                    v.fen.haveRssi ? v.fen.lastRssi : 0,
                    v.radio != nullptr && v.radio->lastLqi() ? *v.radio->lastLqi() : 0,
                    in.farRssi, in.farLqi);
        std::fflush(stdout);
      }

      if (csv != nullptr) {
        const double tms =
            std::chrono::duration<double, std::milli>(Clock::now() - debut).count();
        std::fprintf(csv, "%.1f,%s,%u,%zu,%.3f,%d,%d,%d,%d\n", tms,
                     v.nom.c_str(), in.seq, v.taillePalier, rtt,
                     v.fen.haveRssi ? v.fen.lastRssi : 0,
                     v.radio != nullptr && v.radio->lastLqi() ? *v.radio->lastLqi() : 0,
                     in.farRssi, in.farLqi);
      }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }

  // ── bilans ──
  const double duree = std::min(
      std::chrono::duration<double>(Clock::now() - debut).count(),
      static_cast<double>(opt.durationS));

  for (auto &v : voies) {
    // La ligne d'un palier s'imprime en le quittant : sans ceci le dernier,
    // celui qui porte la plus grosse charge utile, ne serait jamais affiche.
    if (opt.mode == "ramp" && v.palierStats.sent > 0 && !v.paliers.empty()) {
      const double perte =
          100.0 * (1.0 - static_cast<double>(v.palierStats.echoes) /
                             static_cast<double>(v.palierStats.sent));
      std::printf("%-7s palier %4zu o : %3u emises, %3u echos, perte %5.1f %%, "
                  "rtt moy %6.2f ms\n",
                  v.nom.c_str(), v.paliers[v.palierIdx], v.palierStats.sent,
                  v.palierStats.echoes, perte, v.palierStats.moyenne());
    }

    std::cout << "\n════ " << v.nom << " ════";
    afficherBilan(opt, v.st, duree, v.nom);

    if (v.radio != nullptr) {
      std::cout << "\n── compteurs du pilote ──\n";
      std::cout << "CRC faux          : " << v.radio->crcErrorCount() << "\n";
      std::cout << "longueurs aberr.  : " << v.radio->lengthErrorCount() << "\n";
      std::cout << "refus CCA         : " << v.radio->ccaRefusalCount()
                << "  (canal occupe : contention, pas une panne)\n";
      if (v.radio->forcedSendCount() > 0)
        std::cout << "emissions forcees : " << v.radio->forcedSendCount()
                  << "  (CCA contourne)\n";
      if (v.radio->overflowCount() > 0)
        std::cout << "debordements FIFO : " << v.radio->overflowCount()
                  << "  (poll() pas appele assez souvent)\n";
    }
  }

  // ── comparaison, le vrai interet d'une mesure simultanee ──
  if (voies.size() > 1) {
    std::cout << "\n════════ comparaison cote a cote ════════\n";
    std::printf("%-8s %9s %9s %11s %10s\n", "lien", "emises", "perte",
                "rtt moy", "gigue");
    for (const auto &v : voies) {
      const double perte =
          v.st.sent == 0 ? 0.0
                         : 100.0 * (1.0 - static_cast<double>(v.st.echoes) /
                                              static_cast<double>(v.st.sent));
      std::printf("%-8s %9u %8.1f%% %9.2fms %9.2fms\n", v.nom.c_str(),
                  v.st.sent, perte, v.st.moyenne(), v.st.ecartType());
    }
    std::cout << "\nMesure simultanee : les deux liens ont subi les memes\n"
                 "conditions radio au meme instant, et la cohabitation des\n"
                 "deux emetteurs est incluse dans le resultat.\n";
  }

  if (csv != nullptr) {
    std::fclose(csv);
    std::cout << "\nCSV ecrit : " << opt.csvPath << "\n";
  }
  return 0;
}
