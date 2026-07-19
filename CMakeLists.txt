cmake_minimum_required(VERSION 3.20)
project(drone_pi CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ─── Options de build ───────────────────────────────────────────────────────
option(ENABLE_SANITIZERS "Activer ASan + UBSan (Debug uniquement)" ON)
option(ENABLE_CLANG_TIDY  "Activer clang-tidy à la compilation"    OFF)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    add_compile_options(-mcpu=cortex-a76)
endif()

# ─── Flags communs ──────────────────────────────────────────────────────────
add_compile_options(
    -Wall -Wextra -Werror -Wshadow -Wconversion -Wnull-dereference
    -fno-exceptions
    -fno-rtti
)

if(ENABLE_SANITIZERS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-fsanitize=address,undefined)
    add_link_options   (-fsanitize=address,undefined)
    message(STATUS "✓ ASan + UBSan activés")
endif()

if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY clang-tidy REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY}")
    message(STATUS "✓ clang-tidy activé")
endif()

# ─── Dépendances externes ────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# ─── Dictionnaire de codes (CODES::) ──────────────────────────────────────────
# codes.hpp est généré depuis INDEX.csv (édité à la main ou via
# tools/codes_editor.py) — régénéré automatiquement à chaque build si le CSV
# ou le générateur a changé. Tourne sur la machine de build (host), donc pas
# de souci pour le cross-compile ARM64 : Python n'est requis que pour build,
# pas sur la cible.
set(CODES_CSV ${CMAKE_SOURCE_DIR}/INDEX.csv)
set(CODES_HPP ${CMAKE_SOURCE_DIR}/include/drone/generated/codes.hpp)

add_custom_command(
    OUTPUT  ${CODES_HPP}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/gen_codes.py
            --csv ${CODES_CSV} --out ${CODES_HPP}
    DEPENDS ${CODES_CSV}
            ${CMAKE_SOURCE_DIR}/tools/gen_codes.py
            ${CMAKE_SOURCE_DIR}/tools/codes_common.py
    COMMENT "Génération de codes.hpp depuis INDEX.csv"
    VERBATIM
)
# ALL : construit à chaque build par défaut (le custom command ne
# régénère réellement le fichier que si CSV/script ont changé, grâce à
# OUTPUT/DEPENDS — sinon no-op rapide).
add_custom_target(generate_codes ALL DEPENDS ${CODES_HPP})

# ─── drone_core : headers publics + Eigen ────────────────────────────────────
add_library(drone_core INTERFACE)
target_include_directories(drone_core INTERFACE include)
target_link_libraries     (drone_core INTERFACE Eigen3::Eigen)

# ════════════════════════════════════════════════════════════════════════════
# drone_lib — TOUT le code source des composants, en une seule bibliothèque.
#
# Un seul binaire final avec plusieurs points d'entrée (main.cpp dispatch sur
# argv), donc pas de découpage en libs par module : main ET les tests ont de
# toute façon besoin de tous les symboles. Chaque nouveau .cpp de composant
# s'ajoute simplement à cette liste.
# ════════════════════════════════════════════════════════════════════════════
add_library(drone_lib STATIC
    src/SensorsFusion/SensorFusion.cpp
    src/Navigation/Navigation.cpp
    # src/MissionControl/MissionControl.cpp
    # src/MavlinkInterface/MavlinkInterface.cpp
    # src/SystemMonitoring/SystemMonitoring.cpp
    # src/Watchdog/GlobalWatchdog.cpp

    src/Drivers/NEO-M8N.cpp
    src/Drivers/LIS3MDL.cpp
    src/Drivers/LD06.cpp
)
target_link_libraries     (drone_lib PUBLIC drone_core)
target_include_directories(drone_lib PUBLIC include)
add_dependencies          (drone_lib generate_codes)

# ════════════════════════════════════════════════════════════════════════════
# BINAIRE PRINCIPAL — un seul exécutable, dispatch interne par argv
# ════════════════════════════════════════════════════════════════════════════
add_executable(drone src/main.cpp)
target_link_libraries(drone PRIVATE drone_lib)
add_dependencies(drone generate_codes)

# ════════════════════════════════════════════════════════════════════════════
# TESTS
#
# Règle : un test = un executable GTest, linké contre drone_lib entier.
# -fexceptions est autorisé pour GTest uniquement.
# ════════════════════════════════════════════════════════════════════════════
enable_testing()
find_package(GTest REQUIRED)

macro(drone_test TARGET SRC)
    add_executable        (${TARGET} ${SRC})
    target_link_libraries (${TARGET} PRIVATE drone_lib GTest::gtest_main ${ARGN})
    target_compile_options(${TARGET} PRIVATE -fexceptions)
    add_dependencies      (${TARGET} generate_codes)
    add_test              (NAME ${TARGET} COMMAND ${TARGET})
endmacro()

drone_test(test_types        tests/test_types.cpp)
drone_test(test_shared_state tests/test_shared_state.cpp)
drone_test(test_watchdog     tests/test_watchdog.cpp)
# drone_test(test_navigation   tests/test_navigation.cpp)
# drone_test(test_kalman       tests/test_kalman.cpp)
# drone_test(test_mavlink      tests/test_mavlink.cpp)

# ─── test_global_watchdog : boîte noire, process réels ───────────────────────
# Binaire compagnon minimal (pas de dispatch main.cpp, trop lourd) que
# GlobalWatchdog::spawn() exec pour jouer les rôles "navigation"/
# "sensorfusion" pendant les tests. Voir tests/helpers/stub_child.cpp.
add_executable(test_stub_child tests/helpers/stub_child.cpp)

drone_test(test_global_watchdog tests/test_global_watchdog.cpp)
target_compile_definitions(test_global_watchdog PRIVATE
    STUB_CHILD_PATH="$<TARGET_FILE:test_stub_child>")
add_dependencies(test_global_watchdog test_stub_child)

# ─── Cross-compilation ARM64 Pi5 ─────────────────────────────────────────────
# cmake -B build/arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/arm64.cmake -DCMAKE_BUILD_TYPE=Release
# cmake --build build/arm64