/*
 * Headless DSP benchmark for gearmulator targets.
 * Cross-compile for Move, scp to device, run with ROM path.
 *
 * Built as separate binaries per target (to avoid m68k symbol conflicts):
 *   bench_virus <rom_dir>
 *   bench_xt <rom_file>
 *   bench_mq <rom_file>
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <fstream>
#include <vector>
#include <string>

#ifdef __linux__
#include <sched.h>
#include <unistd.h>
#endif

#include "baseLib/os.h"
#include "synthLib/audioTypes.h"

#ifdef BENCH_VIRUS
#include "virusLib/romfile.h"
#include "virusLib/romloader.h"
#include "virusLib/device.h"
#include "virusLib/dspSingle.h"
#include "virusLib/microcontroller.h"
#include "virusLib/deviceModel.h"
#include "dsp56kEmu/audio.h"
#include "synthLib/midiTypes.h"
#endif

#ifdef BENCH_XT
#include "xtLib/xt.h"
#endif

#ifdef BENCH_MQ
#include "mqLib/microq.h"
#endif

static std::vector<uint8_t> loadFile(const char* path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return {};
    f.seekg(0, std::ios::end);
    size_t sz = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> data(sz);
    f.read(reinterpret_cast<char*>(data.data()), sz);
    return data;
}

#ifdef BENCH_VIRUS
static int bench_virus(const char* romPath) {
    auto roms = virusLib::ROMLoader::findROMs(std::string(romPath));
    if (roms.empty()) {
        fprintf(stderr, "No valid Virus ROM found at %s\n", romPath);
        return 1;
    }
    auto* rom = new virusLib::ROMFile(std::move(roms.front()));
    printf("ROM: %s model=%s\n", rom->getFilename().c_str(), rom->getModelName().c_str());

    virusLib::DspSingle* dsp1 = nullptr;
    virusLib::DspSingle* dsp2 = nullptr;
    virusLib::Device::createDspInstances(dsp1, dsp2, *rom, 46875.0f);
    virusLib::Device::bootDSPs(dsp1, dsp2, *rom, false);

    /* Boot drain */
    constexpr int BOOT_CHUNK = 8;
    float dummy_l[BOOT_CHUNK], dummy_r[BOOT_CHUNK];
    synthLib::TAudioInputs inputs = {};
    synthLib::TAudioOutputs outputs = {};
    outputs[0] = dummy_l; outputs[1] = dummy_r;

    if (rom->getModel() == virusLib::DeviceModel::A) {
        for (int i = 0; i < 32; i++)
            dsp1->processAudio(inputs, outputs, BOOT_CHUNK, 0);
        dsp1->disableESSI1();
    } else {
        auto* mc = new virusLib::Microcontroller(*dsp1, *rom, false);
        int retries = 0;
        while (!mc->dspHasBooted() && retries < 512) {
            dsp1->processAudio(inputs, outputs, BOOT_CHUNK, 0);
            retries++;
        }
        mc->sendInitControlCommands(127);
        for (int i = 0; i < 8; i++) dsp1->processAudio(inputs, outputs, BOOT_CHUNK, 0);
        mc->createDefaultState();
        for (int i = 0; i < 8; i++) dsp1->processAudio(inputs, outputs, BOOT_CHUNK, 0);

        /* Send note on to exercise DSP */
        synthLib::SMidiEvent noteOn(synthLib::MidiEventSource::Host, 0x90, 60, 100);
        mc->sendMIDI(noteOn);
        delete mc;
    }
    printf("Boot complete.\n");

    /* Report clock configuration */
    auto& clock = dsp1->getEsxiClock();
    printf("DSP clock: %llu Hz (%.1f MHz)\n",
           (unsigned long long)clock.getSpeedInHz(),
           clock.getSpeedInHz() / 1e6);

    /* Benchmark: 10 seconds of audio at 46875 Hz */
    constexpr int BLOCK = 64;
    float bl[BLOCK], br[BLOCK];
    outputs[0] = bl; outputs[1] = br;
    constexpr int totalFrames = 46875 * 10;
    constexpr int totalBlocks = totalFrames / BLOCK;

    printf("Running %d blocks (%d frames, ~10s at 46875 Hz)...\n", totalBlocks, totalFrames);
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < totalBlocks; i++) {
        dsp1->processAudio(inputs, outputs, BLOCK, 0);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    double ratio = 10.0 / elapsed;
    printf("Elapsed: %.2f s for 10s audio => %.2fx real-time\n", elapsed, ratio);

    delete dsp1;
    delete rom;
    return 0;
}
#endif

#ifdef BENCH_XT
static int bench_xt(const char* romPath) {
    auto romData = loadFile(romPath);
    if (romData.empty()) {
        fprintf(stderr, "Cannot load ROM: %s\n", romPath);
        return 1;
    }
    printf("ROM: %s (%zu bytes)\n", romPath, romData.size());

    xt::Xt device(romData, std::string(romPath));
    if (!device.isValid()) {
        fprintf(stderr, "Invalid XT ROM\n");
        return 1;
    }

    printf("Booting XT...\n");
    while (!device.isBootCompleted())
        device.process(64);
    printf("Boot complete.\n");

    /* Benchmark: 10 seconds at ~40000 Hz */
    constexpr int BLOCK = 64;
    constexpr int totalFrames = 40000 * 10;
    constexpr int totalBlocks = totalFrames / BLOCK;

    printf("Running %d blocks (%d frames, ~10s)...\n", totalBlocks, totalFrames);
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < totalBlocks; i++) {
        device.process(BLOCK);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    double ratio = 10.0 / elapsed;
    printf("Elapsed: %.2f s for 10s audio => %.2fx real-time\n", elapsed, ratio);

    return 0;
}
#endif

#ifdef BENCH_MQ
static int bench_mq(const char* romPath) {
    /* romPath is a directory containing ROM files.
     * Pass empty data so the MicroQ constructor uses its ROM loader. */
    printf("MQ ROM dir: %s\n", romPath);
    chdir(romPath);

    mqLib::MicroQ device(mqLib::BootMode::Default);
    if (!device.isValid()) {
        fprintf(stderr, "Invalid MQ ROM\n");
        return 1;
    }

    printf("Booting MQ...\n");
    while (!device.isBootCompleted())
        device.process(64);
    printf("Boot complete.\n");

    /* Benchmark: 10 seconds at 44100 Hz */
    constexpr int BLOCK = 64;
    constexpr int totalFrames = 44100 * 10;
    constexpr int totalBlocks = totalFrames / BLOCK;

    printf("Running %d blocks (%d frames, ~10s)...\n", totalBlocks, totalFrames);
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < totalBlocks; i++) {
        device.process(BLOCK);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    double ratio = 10.0 / elapsed;
    printf("Elapsed: %.2f s for 10s audio => %.2fx real-time\n", elapsed, ratio);

    return 0;
}
#endif

int main(int argc, char* argv[]) {
    if (argc < 2) {
#ifdef BENCH_VIRUS
        fprintf(stderr, "Usage: %s <rom_directory>\n", argv[0]);
#else
        fprintf(stderr, "Usage: %s <rom_file>\n", argv[0]);
#endif
        return 1;
    }

#ifdef __linux__
    /* Pin main thread to core 2 so it doesn't compete with DSP thread on core 3.
     * The DSP thread pins itself to core 3 via dspthread.cpp. */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    sched_setaffinity(0, sizeof(cpuset), &cpuset);
#endif

    baseLib::setFlushDenormalsToZero();

#ifdef BENCH_VIRUS
    return bench_virus(argv[1]);
#elif defined(BENCH_XT)
    return bench_xt(argv[1]);
#elif defined(BENCH_MQ)
    return bench_mq(argv[1]);
#else
    fprintf(stderr, "No target defined\n");
    return 1;
#endif
}
