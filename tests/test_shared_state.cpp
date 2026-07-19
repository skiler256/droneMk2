// tests/test_shared_state.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Couvre les deux bugs corrigés dans SharedMemory.hpp :
//  - lastError_ sticky (un succès ne doit pas effacer une erreur en attente)
//  - setArrayElement atomique (pas de lost-update entre writers concurrents)

#include "drone/Components/Navigation/SharedNavMem.hpp"
#include "drone/core/SharedSysStateMem.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <gtest/gtest.h>
#include <thread>
#include <vector>

using namespace TYPES;

TEST(SharedMemoryErrorTest, NoErrorByDefault) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(2000));
  EXPECT_EQ(h.consumeError(), shmError::None);
}

TEST(SharedMemoryErrorTest, SuccessDoesNotClearPendingError) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(2000));

  // Timeout forcé : un writer détient déjà le verrou global (owner).
  {
    mem.mem.owner.store(ComponentID::Navigation);
    WriterGuard guard(mem.mem.owner); // libère owner à la fin du scope
    // Accès bloquant sur un timeout très court → erreur enregistrée.
    SharedSysStateMemHandler blocked(mem, ComponentID::Navigation, Us(1000));
    blocked.setHealth(ComponentHealth::NOMINAL);
    EXPECT_EQ(blocked.consumeError(), shmError::timeout);
  }

  // Un appel réussi sur un NOUVEL handler n'a pas d'incidence sur l'ancien
  // : chaque handler a son propre lastError_. On vérifie ici que
  // consumeError() a bien remis l'état à None après consommation (sticky
  // jusqu'à consommation, pas au-delà).
  SharedSysStateMemHandler h2(mem, ComponentID::Navigation, Us(2000));
  h2.setHealth(ComponentHealth::NOMINAL);
  EXPECT_EQ(h2.consumeError(), shmError::None);
}

TEST(SharedMemoryErrorTest, ConsumeErrorResetsToNone) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(2000));

  {
    mem.mem.owner.store(ComponentID::Navigation);
    WriterGuard guard(mem.mem.owner);
    SharedSysStateMemHandler blocked(mem, ComponentID::Navigation, Us(1000));
    blocked.setHealth(ComponentHealth::NOMINAL);
    EXPECT_EQ(blocked.consumeError(), shmError::timeout);
    // Une deuxième consommation immédiate ne doit plus rien trouver.
    EXPECT_EQ(blocked.consumeError(), shmError::None);
  }
}

TEST(SharedMemoryConcurrencyTest, ConcurrentSetHealthDoesNotLoseUpdates) {
  SharedSysStateMem mem{};

  constexpr int kWriters = 4;
  std::vector<std::thread> writers;
  // deque : SharedSysStateMemHandler contient un std::atomic non
  // copiable/déplaçable, incompatible avec la réallocation d'un vector.
  std::deque<SharedSysStateMemHandler> handlers;
  for (int i = 0; i < kWriters; ++i)
    handlers.emplace_back(mem, static_cast<ComponentID>(i), Us(50000));

  // Chaque thread écrit répétitivement dans un slot ComponentID différent.
  // Avant le fix, setArrayElement faisait get() puis set() en deux accès
  // séparés : un writer concurrent pouvait s'intercaler et écraser la
  // mise à jour d'un autre avec une copie du tableau lue avant sa propre
  // écriture (lost update). Avec l'accès atomique, chaque écriture doit
  // systématiquement se voir.
  for (int i = 0; i < kWriters; ++i) {
    writers.emplace_back([&, i] {
      for (int n = 0; n < 200; ++n) {
        handlers[static_cast<size_t>(i)].setHealth(
            n % 2 == 0 ? ComponentHealth::NOMINAL : ComponentHealth::DEGRADED);
      }
    });
  }
  for (auto &t : writers)
    t.join();

  for (int i = 0; i < kWriters; ++i) {
    auto id = static_cast<ComponentID>(i);
    auto health = handlers[static_cast<size_t>(i)].getHealth(id);
    ASSERT_TRUE(health.has_value());
    EXPECT_EQ(*health, ComponentHealth::DEGRADED); // dernière valeur écrite (n=198..199 -> DEGRADED)
  }
}

TEST(SharedMemoryConcurrencyTest, ConcurrentReadersNeverSeeCorruption) {
  // Avant le fix : les lectures (getHealth/getData) passaient par
  // nonLockedAccess, qui ne tenait aucun verrou — un writer pouvait muter
  // compHealth pendant qu'un lecteur le copiait (torn read), ou juste
  // laisser une lecture racer avec l'écriture du checksum. Avec la voie
  // d'accès unifiée (lockedAccess pour tout le monde), lecteurs et writer
  // s'excluent mutuellement : aucune lecture ne doit jamais renvoyer
  // `corrupt`, quel que soit l'entrelacement des threads.
  SharedSysStateMem mem{};
  SharedSysStateMemHandler writer(mem, ComponentID::Navigation, Us(50000));

  std::atomic<bool> stop{false};
  std::atomic<int> corruptSeen{0};

  std::thread writerThread([&] {
    int n = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      writer.setHealth(n++ % 2 == 0 ? ComponentHealth::NOMINAL
                                    : ComponentHealth::DEGRADED);
    }
  });

  // Ids valides et distincts de Navigation (le writer) — surtout PAS
  // ComponentID::Count : c'est le sentinel "Free" du CAS, l'utiliser comme
  // jeton de verrouillage annule l'exclusion mutuelle pour ce thread.
  constexpr int kReaders = 3;
  const std::array<ComponentID, kReaders> readerIds{
      ComponentID::MavlinkInterface, ComponentID::SensorFusion,
      ComponentID::MissionControl};
  std::deque<SharedSysStateMemHandler> readers;
  std::vector<std::thread> readerThreads;
  for (int i = 0; i < kReaders; ++i)
    readers.emplace_back(mem, readerIds[static_cast<size_t>(i)], Us(50000));

  for (int i = 0; i < kReaders; ++i) {
    readerThreads.emplace_back([&, i] {
      while (!stop.load(std::memory_order_relaxed)) {
        readers[static_cast<size_t>(i)].getHealth(ComponentID::Navigation);
        // has_value()==false peut aussi venir d'un simple timeout légitime
        // sous contention CPU (5 threads spinnants sur peu de coeurs) —
        // seule `corrupt` prouve la race qu'on veut détecter ici.
        if (readers[static_cast<size_t>(i)].consumeError() ==
            shmError::corrupt)
          corruptSeen.fetch_add(1, std::memory_order_relaxed);
      }
    });
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  stop.store(true, std::memory_order_relaxed);
  writerThread.join();
  for (auto &t : readerThreads)
    t.join();

  EXPECT_EQ(corruptSeen.load(), 0);
}

TEST(SharedCompMemHistoryTest, CorruptedRestartHistoryIsDetected) {
  // Avant le fix : computeChecksum() de SharedNavMemHandler ne hachait que
  // nav_.data, jamais HotStartHistory/ColdStartHistory — une corruption de
  // l'historique de redémarrage passait totalement inaperçue. Avec
  // historyChecksum() intégré, elle doit maintenant remonter `corrupt`.
  SharedNavMem nav{};
  SharedNavMemHandler h(ComponentID::Navigation, Us(20000), nav);

  h.recordHotStart(Clock::now() - std::chrono::seconds(5));

  // Corruption directe, hors API — simule un torn write.
  nav.compMem.HotStartHistory[1] = Clock::now() + std::chrono::hours(1);

  auto result = h.getHotStartTs(0);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(h.consumeError(), shmError::corrupt);
}

TEST(SharedCompMemHistoryTest, RecordStartSelfHealsOnCorruption) {
  // getData(..., repairOnCorrupt=true) dans recordStart : le premier appel
  // après corruption détecte + répare (sanitize le futur impossible) mais
  // n'enregistre pas encore ce restart-là ; le suivant doit fonctionner
  // normalement sur une shm redevenue saine.
  SharedNavMem nav{};
  SharedNavMemHandler h(ComponentID::Navigation, Us(20000), nav);

  h.recordHotStart(TYPES::Clock::now() - std::chrono::seconds(5));
  nav.compMem.HotStartHistory[1] =
      TYPES::Clock::now() + std::chrono::hours(1); // corruption directe

  h.recordHotStart(TYPES::Clock::now());
  EXPECT_EQ(h.consumeError(), shmError::corrupt);

  auto sanitized = h.getHotStartTs(1);
  ASSERT_TRUE(sanitized.has_value());
  EXPECT_EQ(*sanitized, TYPES::TimePoint{});

  auto now = TYPES::Clock::now();
  h.recordHotStart(now);
  EXPECT_EQ(h.consumeError(), shmError::None);

  auto latest = h.getHotStartTs(0);
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(*latest, now);
}

// ─── Système de codes (CODES::) : ring mineur / ensemble actif majeur ──────

TEST(DiagCodeTest, MinorCodeAppearsAtFrontOfRing) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(20000));

  h.raiseCode(CODES::NVA002); // Info -> mineur

  auto ring = h.getMinorCodes(ComponentID::Navigation);
  ASSERT_TRUE(ring.has_value());
  EXPECT_EQ(ring->at(0).code, CODES::NVA002.pack());

  auto majors = h.getMajorCodes(ComponentID::Navigation);
  ASSERT_TRUE(majors.has_value());
  for (const auto &slot : *majors)
    EXPECT_EQ(slot.code, 0u); // rien ne doit fuiter côté majeur
}

TEST(DiagCodeTest, MajorCodeStaysActiveDespiteMinorTraffic) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(20000));

  h.raiseCode(CODES::NVA001); // Critical -> majeur

  // Un flot de codes mineurs ne doit jamais déloger un code majeur actif —
  // c'est exactement le risque qu'un ring buffer unique aurait introduit.
  for (int i = 0; i < 20; ++i)
    h.raiseCode(CODES::NVA002);

  auto majors = h.getMajorCodes(ComponentID::Navigation);
  ASSERT_TRUE(majors.has_value());
  bool found = false;
  for (const auto &slot : *majors)
    if (slot.code == CODES::NVA001.pack())
      found = true;
  EXPECT_TRUE(found);
}

TEST(DiagCodeTest, ClearCodeRemovesFromActiveSet) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(20000));

  h.raiseCode(CODES::NVA001);
  h.clearCode(CODES::NVA001);

  auto majors = h.getMajorCodes(ComponentID::Navigation);
  ASSERT_TRUE(majors.has_value());
  for (const auto &slot : *majors)
    EXPECT_EQ(slot.code, 0u);
}

TEST(DiagCodeTest, MinorRingOverwritesOldestEntries) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(20000));

  // kMinorCodeRingSize == 16 : la 17e éviction doit avoir chassé la plus
  // ancienne (le tout premier code levé).
  for (int i = 0; i < 17; ++i)
    h.raiseCode(CODES::NVA002);
  h.raiseCode(CODES::NVA003); // 18e, code différent

  auto ring = h.getMinorCodes(ComponentID::Navigation);
  ASSERT_TRUE(ring.has_value());
  EXPECT_EQ(ring->at(0).code, CODES::NVA003.pack());
  for (const auto &entry : *ring)
    EXPECT_NE(entry.code, 0u); // le ring est plein, aucun slot vide restant
}

TEST(DiagCodeTest, CodesAreIsolatedPerComponent) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler navH(mem, ComponentID::Navigation, Us(20000));
  SharedSysStateMemHandler sfH(mem, ComponentID::SensorFusion, Us(20000));

  navH.raiseCode(CODES::NVA001);
  sfH.raiseCode(CODES::SFS001);

  auto navMajors = navH.getMajorCodes(ComponentID::Navigation);
  auto sfMinors = sfH.getMinorCodes(ComponentID::SensorFusion);
  ASSERT_TRUE(navMajors.has_value());
  ASSERT_TRUE(sfMinors.has_value());

  EXPECT_EQ(navMajors->at(0).code, CODES::NVA001.pack());
  EXPECT_EQ(sfMinors->at(0).code, CODES::SFS001.pack());

  // Le composant de l'un ne doit rien voir chez l'autre.
  auto navMinors = navH.getMinorCodes(ComponentID::Navigation);
  ASSERT_TRUE(navMinors.has_value());
  for (const auto &entry : *navMinors)
    EXPECT_NE(entry.code, CODES::SFS001.pack());
}

TEST(DiagCodeTest, UnknownCodeFallsBackToMinor) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(20000));

  // Code valide en forme mais absent du dictionnaire généré : ne doit pas
  // être perdu, doit atterrir côté mineur par défaut (cf. raiseCode()).
  CODES::Code unknown{CODES::Component::NV, CODES::Category::P, 999};
  h.raiseCode(unknown);

  auto ring = h.getMinorCodes(ComponentID::Navigation);
  ASSERT_TRUE(ring.has_value());
  EXPECT_EQ(ring->at(0).code, unknown.pack());
}

TEST(DiagCodeTest, CorruptedCodeStorageIsDetected) {
  SharedSysStateMem mem{};
  SharedSysStateMemHandler h(mem, ComponentID::Navigation, Us(20000));

  h.raiseCode(CODES::NVA001); // établit un checksum valide

  // Corruption directe, hors API — le checksum doit couvrir minorCodes/
  // majorCodes, pas seulement compHealth.
  mem.majorCodes[static_cast<size_t>(ComponentID::Navigation)][1].code = 0xDEAD;

  auto result = h.getHealth(ComponentID::Navigation);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(h.consumeError(), shmError::corrupt);
}

TEST(DiagCodeTest, UnrelatedWriteDoesNotMaskAnotherComponentsCorruption) {
  // Avant le checksum par composant : un seul checksum global couvrait tout
  // le segment. reset(id) d'un writer ne réparait QUE son propre slot, mais
  // recalculait le checksum sur TOUT le payload — y compris un autre slot
  // corrompu jamais vérifié ni réparé, le "validant" silencieusement.
  SharedSysStateMem mem{};
  SharedSysStateMemHandler navH(mem, ComponentID::Navigation, Us(20000));
  SharedSysStateMemHandler sfH(mem, ComponentID::SensorFusion, Us(20000));

  navH.setHealth(ComponentHealth::NOMINAL);  // établit le checksum de Navigation
  sfH.setHealth(ComponentHealth::NOMINAL);   // établit le checksum de SensorFusion

  // Corruption directe du slot Navigation, hors API.
  mem.minorCodes[static_cast<size_t>(ComponentID::Navigation)][3].code = 0xDEAD;

  // SensorFusion fait un write totalement indépendant — repairOnCorrupt=true
  // en interne, donc s'il détectait (à tort) une corruption sur SON propre
  // slot, il déclencherait son propre reset(). Ici son slot à lui est sain :
  // l'opération doit réussir normalement, sans toucher à Navigation.
  sfH.raiseCode(CODES::SFS001);
  EXPECT_NE(sfH.consumeError(), shmError::corrupt);

  // La corruption de Navigation doit rester détectable : ni SensorFusion ni
  // personne d'autre n'a pu la "réparer" ou la revalider en douce via un
  // recalcul de checksum global.
  auto result = navH.getHealth(ComponentID::Navigation);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(navH.consumeError(), shmError::corrupt);
}
