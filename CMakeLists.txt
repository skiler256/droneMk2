# cmake/arm64.cmake
# Toolchain pour cross-compiler vers Raspberry Pi 5 (ARM64 / aarch64)
#
# Usage :
#   cmake -B build/arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/arm64.cmake -DCMAKE_BUILD_TYPE=Release
#   cmake --build build/arm64
#   scp build/arm64/... pi@raspberrypi.local:~/drone/

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Compilateur ARM64 (installer avec : sudo apt install g++-aarch64-linux-gnu)
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Sysroot (optionnel — pour linker avec les libs du Pi)
# set(CMAKE_SYSROOT /path/to/pi/sysroot)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)