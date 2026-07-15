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
)
target_link_libraries     (drone_lib PUBLIC drone_core)
target_include_directories(drone_lib PUBLIC include)

# ════════════════════════════════════════════════════════════════════════════
# BINAIRE PRINCIPAL — un seul exécutable, dispatch interne par argv
# ════════════════════════════════════════════════════════════════════════════
add_executable(drone src/main.cpp)
target_link_libraries(drone PRIVATE drone_lib)

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
    add_test              (NAME ${TARGET} COMMAND ${TARGET})
endmacro()

drone_test(test_types        tests/test_types.cpp)
drone_test(test_shared_state tests/test_shared_state.cpp)
# drone_test(test_navigation   tests/test_navigation.cpp)
# drone_test(test_kalman       tests/test_kalman.cpp)
# drone_test(test_mavlink      tests/test_mavlink.cpp)
# drone_test(test_watchdog     tests/test_watchdog.cpp)

# ─── Cross-compilation ARM64 Pi5 ─────────────────────────────────────────────
# cmake -B build/arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/arm64.cmake -DCMAKE_BUILD_TYPE=Release
# cmake --build build/arm64