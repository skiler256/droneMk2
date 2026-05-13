cmake_minimum_required(VERSION 3.20)
project(drone_pi CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ─── Options de build ───────────────────────────────────────────────────────
option(ENABLE_SANITIZERS "Activer ASan + UBSan (Debug uniquement)" ON)
option(ENABLE_CLANG_TIDY  "Activer clang-tidy à la compilation"    OFF)

# ─── Flags communs (tous les targets) ───────────────────────────────────────
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
# Tous les blocs et tous les tests linkent contre drone_core.
# C'est une INTERFACE library : pas de .cpp, juste des headers + flags.
add_library(drone_core INTERFACE)
target_include_directories(drone_core INTERFACE include)
target_link_libraries     (drone_core INTERFACE Eigen3::Eigen)

# ════════════════════════════════════════════════════════════════════════════
# BLOCS — ajouter un bloc = décommenter + lister ses .cpp
#
# Règle : un bloc = une static library.
# Elle linke drone_core et expose ses propres headers via target_include_dirs.
# Le main et les tests linkent la lib du bloc, pas les .cpp directement.
# ════════════════════════════════════════════════════════════════════════════

# ── ② Navigation ─────────────────────────────────────────────────────────────
# Décommenter quand src/navigation/navigation_block.cpp existe
#
# add_library(nav_lib STATIC
#     src/navigation/navigation_block.cpp
# )
# target_link_libraries     (nav_lib PUBLIC drone_core)
# target_include_directories(nav_lib PUBLIC src/navigation)

# ── ① Drivers + Fusion ───────────────────────────────────────────────────────
# add_library(drivers_lib STATIC
#     src/drivers/sensor_fusion_block.cpp
#     src/drivers/kalman_filter.cpp
# )
# target_link_libraries     (drivers_lib PUBLIC drone_core)
# target_include_directories(drivers_lib PUBLIC src/drivers)

# ── ③ MAVLink ────────────────────────────────────────────────────────────────
# add_library(mavlink_lib STATIC
#     src/mavlink/mavlink_block.cpp
# )
# target_link_libraries     (mavlink_lib PUBLIC drone_core)
# target_include_directories(mavlink_lib PUBLIC src/mavlink)

# ── ④ Monitoring ─────────────────────────────────────────────────────────────
# add_library(monitoring_lib STATIC
#     src/monitoring/monitoring_block.cpp
# )
# target_link_libraries     (monitoring_lib PUBLIC drone_core)
# target_include_directories(monitoring_lib PUBLIC src/monitoring)

# ════════════════════════════════════════════════════════════════════════════
# BINAIRE PRINCIPAL
#
# Décommenter quand TOUS les blocs existent.
# Linker uniquement les libs des blocs — jamais les .cpp directement.
# ════════════════════════════════════════════════════════════════════════════
#
# add_executable(drone src/main.cpp)
# target_link_libraries(drone PRIVATE
#     drone_core
#     nav_lib
#     drivers_lib
#     mavlink_lib
#     monitoring_lib
# )

# ════════════════════════════════════════════════════════════════════════════
# TESTS
#
# Règle : un test = un executable GTest.
# Un test linke drone_core + la lib du bloc qu'il teste.
# -fexceptions est autorisé pour GTest uniquement.
# ════════════════════════════════════════════════════════════════════════════
enable_testing()
find_package(GTest REQUIRED)

# Helper macro pour ne pas répéter 4 lignes par test
macro(drone_test TARGET SRC)
    add_executable        (${TARGET} ${SRC})
    target_link_libraries (${TARGET} PRIVATE drone_core GTest::gtest_main ${ARGN})
    target_compile_options(${TARGET} PRIVATE -fexceptions)
    add_test              (NAME ${TARGET} COMMAND ${TARGET})
endmacro()

# ── Tests existants ───────────────────────────────────────────────────────────
drone_test(test_types        tests/test_types.cpp)
drone_test(test_shared_state tests/test_shared_state.cpp)

# ── Tests à ajouter au fur et à mesure ───────────────────────────────────────
# drone_test(test_navigation  tests/test_navigation.cpp  nav_lib)
# drone_test(test_kalman      tests/test_kalman.cpp      drivers_lib)
# drone_test(test_mavlink     tests/test_mavlink.cpp     mavlink_lib)
# drone_test(test_watchdog    tests/test_watchdog.cpp)

# ─── Cross-compilation ARM64 Pi5 ─────────────────────────────────────────────
# cmake -B build/arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/arm64.cmake -DCMAKE_BUILD_TYPE=Release
# cmake --build build/arm64