cmake_minimum_required(VERSION 3.15)
project(move-anything-virus VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# Build position-independent code (required for shared library)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Gearmulator dependencies ----
# We add only the specific subdirectories needed for virusLib,
# bypassing the full gearmulator build system (JUCE, bridge, etc.)

set(GEARMULATOR_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/libs/gearmulator/source)

# DSP56300 emulator (includes asmjit JIT compiler)
set(ASMJIT_STATIC TRUE)
set(ASMJIT_NO_INSTALL TRUE)
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(${GEARMULATOR_SOURCE}/dsp56300/source ${CMAKE_BINARY_DIR}/dsp56300)

# Base library
add_subdirectory(${GEARMULATOR_SOURCE}/baseLib ${CMAKE_BINARY_DIR}/baseLib)

# Synth library (core device abstraction)
add_subdirectory(${GEARMULATOR_SOURCE}/synthLib ${CMAKE_BINARY_DIR}/synthLib)

# Libresample (sample rate conversion)
add_subdirectory(${GEARMULATOR_SOURCE}/libresample ${CMAKE_BINARY_DIR}/libresample)

# Virus library (DSP56300-based Virus emulator)
add_subdirectory(${GEARMULATOR_SOURCE}/virusLib ${CMAKE_BINARY_DIR}/virusLib)

# ---- Move Anything plugin ----

add_library(virus-move-plugin SHARED
    src/dsp/virus_plugin.cpp
)

target_include_directories(virus-move-plugin PRIVATE
    ${GEARMULATOR_SOURCE}
)

target_link_libraries(virus-move-plugin PRIVATE
    virusLib
    synthLib
)

# Statically link libstdc++ - the Move device may have an older GLIBCXX
target_link_options(virus-move-plugin PRIVATE -static-libstdc++ -static-libgcc)

# Output as dsp.so
set_target_properties(virus-move-plugin PROPERTIES
    OUTPUT_NAME "dsp"
    PREFIX ""
    SUFFIX ".so"
)
