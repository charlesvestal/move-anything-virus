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

# MC68k emulator (needed by hardwareLib for XT/MQ)
add_subdirectory(${GEARMULATOR_SOURCE}/mc68k ${CMAKE_BINARY_DIR}/mc68k)

# Hardware library (MC68331 emulation)
add_subdirectory(${GEARMULATOR_SOURCE}/hardwareLib ${CMAKE_BINARY_DIR}/hardwareLib)

# Waldorf base library (shared by XT and MQ)
add_subdirectory(${GEARMULATOR_SOURCE}/wLib ${CMAKE_BINARY_DIR}/wLib)

# Waldorf XT / Xenia
add_subdirectory(${GEARMULATOR_SOURCE}/xtLib ${CMAKE_BINARY_DIR}/xtLib)

# Waldorf Micro Q / Vavra
add_subdirectory(${GEARMULATOR_SOURCE}/mqLib ${CMAKE_BINARY_DIR}/mqLib)

# ---- Move Anything plugin ----

add_library(virus-move-plugin SHARED
    src/dsp/virus_plugin.cpp
    src/dsp/program_selection.cpp
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

# ---- DSP Benchmarks (headless, for testing on device) ----

add_executable(bench_virus src/benchmark/dsp_bench.cpp)
target_include_directories(bench_virus PRIVATE ${GEARMULATOR_SOURCE})
target_compile_definitions(bench_virus PRIVATE BENCH_VIRUS=1)
target_link_libraries(bench_virus PRIVATE virusLib synthLib baseLib)
target_link_options(bench_virus PRIVATE -static-libstdc++ -static-libgcc)

add_executable(bench_xt src/benchmark/dsp_bench.cpp)
target_include_directories(bench_xt PRIVATE ${GEARMULATOR_SOURCE})
target_compile_definitions(bench_xt PRIVATE BENCH_XT=1)
target_link_libraries(bench_xt PRIVATE xtLib synthLib baseLib)
target_link_options(bench_xt PRIVATE -static-libstdc++ -static-libgcc)

add_executable(bench_mq src/benchmark/dsp_bench.cpp)
target_include_directories(bench_mq PRIVATE ${GEARMULATOR_SOURCE})
target_compile_definitions(bench_mq PRIVATE BENCH_MQ=1)
target_link_libraries(bench_mq PRIVATE mqLib synthLib baseLib)
target_link_options(bench_mq PRIVATE -static-libstdc++ -static-libgcc)
