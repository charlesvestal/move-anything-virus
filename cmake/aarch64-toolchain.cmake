# CMake toolchain file for cross-compiling to aarch64 Linux (Ableton Move)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Optimize for Move's CM4 (Cortex-A72 / ARMv8.0)
set(CMAKE_C_FLAGS_INIT "-march=armv8-a -mtune=cortex-a72")
set(CMAKE_CXX_FLAGS_INIT "-march=armv8-a -mtune=cortex-a72")
