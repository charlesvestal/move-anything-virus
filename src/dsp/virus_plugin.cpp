/*
 * Virus DSP Plugin for Move Anything
 *
 * Runs the DSP56300 JIT emulator in a CHILD PROCESS to avoid sharing
 * MoveOriginal's mmap_lock, heap allocator, and other kernel resources.
 * Communication between parent (plugin API) and child (DSP) uses a
 * shared memory region with an audio ring buffer and MIDI FIFO.
 *
 * Architecture:
 *   Parent (MoveOriginal process):
 *     - Plugin API v2 (create/destroy/render_block/on_midi/get_param/set_param)
 *     - render_block reads audio from shared memory ring buffer
 *     - on_midi writes to shared memory MIDI FIFO
 *     - get_param/set_param read/write shared memory status
 *
 *   Child (forked process):
 *     - Owns all gearmulator objects (DspSingle, Microcontroller, ROMFile)
 *     - DSP thread runs JIT-compiled DSP56300 code
 *     - Emu thread reads audio from DSP, resamples 46875→44100, writes to shared ring
 *     - MIDI consumer thread reads from shared MIDI FIFO, sends to Microcontroller
 *
 * GPL-3.0 License
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <dirent.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <signal.h>
#include <fcntl.h>
#include <stdarg.h>
#include <algorithm>
#include <vector>
#include <string>
#include <atomic>

/* Gearmulator headers */
#include "virusLib/device.h"
#include "virusLib/dspSingle.h"
#include "virusLib/microcontroller.h"
#include "virusLib/romfile.h"
#include "virusLib/romloader.h"
#include "virusLib/deviceModel.h"
#include "dsp56kEmu/audio.h"
#include "dsp56kEmu/semaphore.h"
#include "synthLib/audioTypes.h"
#include "synthLib/midiTypes.h"
#include "program_selection.h"

/* Plugin API v2 (inline definitions to avoid path issues) */
extern "C" {

#define MOVE_PLUGIN_API_VERSION_2 2
#define MOVE_SAMPLE_RATE 44100
#define MOVE_FRAMES_PER_BLOCK 128

typedef struct host_api_v1 {
    uint32_t api_version;
    int sample_rate;
    int frames_per_block;
    uint8_t *mapped_memory;
    int audio_out_offset;
    int audio_in_offset;
    void (*log)(const char *msg);
    int (*midi_send_internal)(const uint8_t *msg, int len);
    int (*midi_send_external)(const uint8_t *msg, int len);
} host_api_v1_t;

typedef struct plugin_api_v2 {
    uint32_t api_version;
    void* (*create_instance)(const char *module_dir, const char *json_defaults);
    void (*destroy_instance)(void *instance);
    void (*on_midi)(void *instance, const uint8_t *msg, int len, int source);
    void (*set_param)(void *instance, const char *key, const char *val);
    int (*get_param)(void *instance, const char *key, char *buf, int buf_len);
    int (*get_error)(void *instance, char *buf, int buf_len);
    void (*render_block)(void *instance, int16_t *out_interleaved_lr, int frames);
} plugin_api_v2_t;

} /* extern "C" */

/* =====================================================================
 * Constants
 * ===================================================================== */

#define DEVICE_RATE         46875.0f
#define AUDIO_RING_SIZE     8192
#define EMU_CHUNK           64
#define OUTPUT_GAIN         0.7f
#define MIDI_FIFO_SIZE      4096  /* bytes */
#define RING_TARGET_FILL    768   /* ~17ms at 44100 Hz — extra headroom for note-on bursts */

static const host_api_v1_t *g_host = nullptr;

/* Microsecond clock */
static int64_t now_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* Logging helper */
static void plugin_log(const char *fmt, ...) {
    if (!g_host || !g_host->log) return;
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    char msg[560];
    snprintf(msg, sizeof(msg), "[virus] %s", buf);
    g_host->log(msg);
}

/* Crash-safe log: writes to a dedicated file with immediate flush. */
static FILE *g_vlog = nullptr;
static void vlog(const char *fmt, ...) {
    if (!g_vlog)
        g_vlog = fopen("/data/UserData/move-anything/virus_debug.log", "a");
    if (!g_vlog) return;
    va_list args;
    va_start(args, fmt);
    vfprintf(g_vlog, fmt, args);
    va_end(args);
    fputc('\n', g_vlog);
    fflush(g_vlog);
}

/* =====================================================================
 * Virus CC parameter mapping
 * ===================================================================== */

struct virus_param_t {
    const char *key;
    const char *name;
    int cc;
    int min_val;
    int max_val;
};

/* Virus Page A CC numbers (from parameterDescriptions_C.json cc-to-param map).
 * These are NOT standard MIDI CCs — they're the Virus's own parameter indices. */
static const virus_param_t g_params[] = {
    {"cutoff",       "Cutoff",        40, 0, 127},
    {"resonance",    "Resonance",     42, 0, 127},
    {"filter_env",   "Filt Env Amt",  44, 0, 127},
    {"flt_attack",   "Flt Attack",    54, 0, 127},
    {"flt_decay",    "Flt Decay",     55, 0, 127},
    {"flt_sustain",  "Flt Sustain",   56, 0, 127},
    {"flt_release",  "Flt Release",   58, 0, 127},
    {"amp_attack",   "Amp Attack",    59, 0, 127},
    {"amp_decay",    "Amp Decay",     60, 0, 127},
    {"amp_sustain",  "Amp Sustain",   61, 0, 127},
    {"amp_release",  "Amp Release",   63, 0, 127},
    {"filter_mode",  "Filter Mode",   51, 0, 7},
    {"osc1_shape",   "Osc1 Shape",    17, 0, 127},
    {"osc2_shape",   "Osc2 Shape",    22, 0, 127},
    {"osc_balance",  "Osc Balance",   33, 0, 127},
    {"patch_volume", "Volume",        91, 0, 127},
};
static const int NUM_PARAMS = sizeof(g_params) / sizeof(g_params[0]);
static constexpr int VIRUS_MAX_BANKS = 32;
static constexpr int VIRUS_MAX_PRESETS_PER_BANK = 128;
static constexpr int VIRUS_PRESET_NAME_BYTES = 24;

/* =====================================================================
 * Shared memory structure (parent <-> child process)
 * ===================================================================== */

struct virus_shm_t {
    /* Audio ring buffer (child writes, parent reads) */
    int16_t audio_ring[AUDIO_RING_SIZE * 2];
    volatile int ring_read;
    volatile int ring_write;

    /* MIDI input FIFO (parent writes, child reads)
     * Format: [len, byte0, byte1, byte2, ...] per message, len=1..8 */
    uint8_t midi_buf[MIDI_FIFO_SIZE];
    volatile int midi_read;
    volatile int midi_write;

    /* Control flags */
    volatile int child_ready;       /* child sets when boot complete */
    volatile int child_shutdown;    /* parent sets to request shutdown */
    volatile int child_alive;       /* child increments periodically */

    /* Status (child writes, parent reads) */
    volatile int initialized;
    volatile int loading_complete;
    char loading_status[128];
    char load_error[256];
    char preset_name[64];
    char bank_name[32];
    volatile int current_bank;
    volatile int current_preset;
    volatile int bank_count;
    volatile int preset_count;
    volatile int preset_name_cache_ready;
    char preset_name_cache[VIRUS_MAX_BANKS][VIRUS_MAX_PRESETS_PER_BANK][VIRUS_PRESET_NAME_BYTES];
    volatile int octave_transpose;
    volatile int cc_values[128];

    /* Profiling */
    volatile int underrun_count;
    volatile int emu_blocks;
    volatile int render_count;
    volatile int64_t prof_process_us_total;
    volatile int prof_process_max_us;
    volatile float prof_peak_level;
    volatile int prof_ring_min;
    volatile int64_t prof_start_us;

    /* Module directory (set by parent before fork) */
    char module_dir[256];

    /* DSP clock percent (parent writes, child reads and applies) */
    volatile int dsp_clock_percent;  /* 0 = auto (100 for A, 50 for B/C) */
    volatile int dsp_clock_applied;  /* last value applied by child */
    char rom_model_name[16];         /* "A", "B", "C", etc. */

    /* Output gain (parent writes, child reads — applied during resample) */
    volatile int gain_percent;       /* 0 = auto (70), else 1..100 */

    /* ROM selection (child enumerates, parent selects) */
    volatile int rom_index;          /* which ROM to load (0-based) */
    volatile int rom_count;          /* how many ROMs found (child writes) */
    char rom_names[8][64];           /* ROM display names (child writes) */
};

/* =====================================================================
 * Shared memory ring buffer helpers
 * ===================================================================== */

static int shm_ring_available(virus_shm_t *shm) {
    int avail = shm->ring_write - shm->ring_read;
    if (avail < 0) avail += AUDIO_RING_SIZE;
    return avail;
}

static int shm_ring_free(virus_shm_t *shm) {
    return AUDIO_RING_SIZE - 1 - shm_ring_available(shm);
}

/* MIDI FIFO helpers */
static int midi_fifo_available(virus_shm_t *shm) {
    int avail = shm->midi_write - shm->midi_read;
    if (avail < 0) avail += MIDI_FIFO_SIZE;
    return avail;
}

static int midi_fifo_free(virus_shm_t *shm) {
    return MIDI_FIFO_SIZE - 1 - midi_fifo_available(shm);
}

static uint8_t bank_index_to_midi_lsb(int bank_index, int bank_count) {
    int idx = bank_index;
    int max_idx = bank_count > 0 ? bank_count - 1 : 0;
    if (idx < 0) idx = 0;
    if (idx > max_idx) idx = max_idx;
    int midi = idx + 1; /* Access uses 1-based bank numbers: A=1..H=8. */
    if (midi < 1) midi = 1;
    if (midi > 127) midi = 127;
    return (uint8_t)midi;
}

static const char *shm_lookup_preset_name(virus_shm_t *shm, int bank, int preset) {
    if (!shm || !shm->preset_name_cache_ready) return nullptr;
    if (bank < 0 || preset < 0) return nullptr;
    if (bank >= shm->bank_count || preset >= shm->preset_count) return nullptr;
    if (bank >= VIRUS_MAX_BANKS || preset >= VIRUS_MAX_PRESETS_PER_BANK) return nullptr;
    const char *name = shm->preset_name_cache[bank][preset];
    if (!name[0]) return nullptr;
    return name;
}

static void shm_refresh_current_preset_name(virus_shm_t *shm) {
    if (!shm) return;
    const char *name = shm_lookup_preset_name(shm, shm->current_bank, shm->current_preset);
    if (name) {
        snprintf((char*)shm->preset_name, sizeof(shm->preset_name), "%s", name);
    } else {
        snprintf((char*)shm->preset_name, sizeof(shm->preset_name), "---");
    }
}

static void midi_fifo_push(virus_shm_t *shm, const uint8_t *msg, int len) {
    if (len < 1 || len > 8) return;
    if (midi_fifo_free(shm) < len + 1) return; /* drop if full */
    int wr = shm->midi_write;
    shm->midi_buf[wr] = (uint8_t)len;
    wr = (wr + 1) % MIDI_FIFO_SIZE;
    for (int i = 0; i < len; i++) {
        shm->midi_buf[wr] = msg[i];
        wr = (wr + 1) % MIDI_FIFO_SIZE;
    }
    shm->midi_write = wr;
}

/* =====================================================================
 * Instance structure (parent process only)
 * ===================================================================== */

struct virus_instance_t {
    virus_shm_t *shm;
    pid_t child_pid;
    pthread_t boot_thread;
    volatile int boot_thread_running;
    char *pending_state;
    int pending_state_valid;
};

/* =====================================================================
 * Child process — all DSP work happens here
 * ===================================================================== */

/* Resample ratio: 46875 Hz source → 44100 Hz output */
static constexpr double RESAMPLE_RATIO = 46875.0 / 44100.0;
#define RESAMPLE_MAX_OUT (EMU_CHUNK + 4)

static void child_send_midi(virusLib::Microcontroller *mc, const uint8_t *msg, int len) {
    if (!mc || len < 1) return;
    synthLib::SMidiEvent ev(synthLib::MidiEventSource::Host,
                            msg[0],
                            len > 1 ? msg[1] : 0,
                            len > 2 ? msg[2] : 0);
    mc->sendMIDI(ev);
}

static bool child_use_mc_preset_map(virusLib::ROMFile *rom) {
    return rom && rom->getModel() == virusLib::DeviceModel::A;
}

static int child_map_browser_bank_to_mc_bank(virusLib::ROMFile *rom, int bank) {
    if (bank < 0) return bank;
    if (!child_use_mc_preset_map(rom)) return bank;
    /* Virus A MC single-bank table starts with 2 RAM banks, then ROM A..H.
     * Browser bank 0..7 should map to ROM banks, not RAM mirror slots. */
    return bank + 2;
}

static bool child_get_single_preset(virusLib::Microcontroller *mc,
                                    virusLib::ROMFile *rom,
                                    int bank,
                                    int preset,
                                    virusLib::ROMFile::TPreset *out) {
    if (!out || bank < 0 || preset < 0) return false;

    if (child_use_mc_preset_map(rom) && mc) {
        const int mapped_bank = child_map_browser_bank_to_mc_bank(rom, bank);
        if (mapped_bank < 0 || mapped_bank >= VIRUS_MAX_BANKS || preset >= VIRUS_MAX_PRESETS_PER_BANK)
            return false;
        return mc->requestSingle(virusLib::fromArrayIndex((uint8_t)mapped_bank), (uint8_t)preset, *out);
    }

    if (!rom) return false;
    return rom->getSingle(bank, preset, *out);
}

static bool child_validate_virus_a_bank(virusLib::Microcontroller *mc,
                                        virusLib::ROMFile *rom,
                                        int bank,
                                        int preset_count) {
    if (!child_use_mc_preset_map(rom)) return true;
    if (!mc || !rom || bank < 0 || preset_count <= 0) return false;

    int p0 = 0;
    int p1 = preset_count > 1 ? 1 : 0;
    int plast = preset_count - 1;
    const int probes[3] = {p0, p1, plast};

    for (int i = 0; i < 3; ++i) {
        int p = probes[i];
        virusLib::ROMFile::TPreset pd{};
        if (!child_get_single_preset(mc, rom, bank, p, &pd)) return false;

        if (pd[2] != (uint8_t)bank) return false;
        if (pd[3] != (uint8_t)p) return false;
        if (virusLib::ROMFile::getSingleName(pd).size() != 10) return false;
    }

    return true;
}

static void child_build_preset_name_cache(virus_shm_t *shm,
                                          virusLib::Microcontroller *mc,
                                          virusLib::ROMFile *rom);
static void child_update_preset_name(virus_shm_t *shm,
                                     virusLib::Microcontroller *mc,
                                     virusLib::ROMFile *rom);
static int child_detect_valid_bank_count(virusLib::Microcontroller *mc,
                                         virusLib::ROMFile *rom,
                                         int fallback_bank_count);

/* Process at most max_msgs MIDI messages per call.
 * Rate-limiting spreads note-on voice allocation across emu blocks,
 * preventing DSP cycle bursts that cause audio dropouts. */
static void child_process_midi_fifo(virus_shm_t *shm,
                                    virusLib::Microcontroller *mc,
                                    virusLib::ROMFile *rom,
                                    int max_msgs = 2) {
    int processed = 0;
    while (midi_fifo_available(shm) > 0 && processed < max_msgs) {
        int rd = shm->midi_read;
        int len = shm->midi_buf[rd];
        rd = (rd + 1) % MIDI_FIFO_SIZE;
        if (len < 1 || len > 8 || midi_fifo_available(shm) < len + 1) break;
        uint8_t msg[8];
        for (int i = 0; i < len; i++) {
            msg[i] = shm->midi_buf[rd];
            rd = (rd + 1) % MIDI_FIFO_SIZE;
        }
        shm->midi_read = rd;

        /* Track CC values in shared memory */
        uint8_t status = msg[0] & 0xF0;
        if (status == 0xB0 && len >= 3)
            shm->cc_values[msg[1] & 0x7F] = msg[2] & 0x7F;

        int bank = shm->current_bank;
        int preset = shm->current_preset;
        const int change_mask = apply_program_selection_midi(msg, len, shm->bank_count, shm->preset_count, &bank, &preset);
        if (change_mask != PROGRAM_SELECTION_NONE) {
            shm->current_bank = bank;
            shm->current_preset = preset;
            if ((change_mask & PROGRAM_SELECTION_BANK_CHANGED) != 0)
                snprintf((char*)shm->bank_name, sizeof(shm->bank_name), "Bank %c", 'A' + bank);
        }

        const bool is_bank_select = (status == 0xB0 && len >= 3 && (msg[1] == 0 || msg[1] == 32));
        const bool is_program_change = (status == 0xC0 && len >= 2);

        if (is_program_change) {
            /* Load from the same source we use for the browser cache so names
             * and audible patch content stay in lockstep. */
            virusLib::ROMFile::TPreset single{};
            if (child_get_single_preset(mc, rom, shm->current_bank, shm->current_preset, &single)) {
                mc->writeSingle(virusLib::BankNumber::EditBuffer, virusLib::SINGLE, single);
            }
            child_update_preset_name(shm, mc, rom);
            processed++;
            continue;
        }

        if (is_bank_select) {
            /* Bank select is pending until Program Change commit. */
            if ((change_mask & PROGRAM_SELECTION_BANK_CHANGED) != 0)
                child_update_preset_name(shm, mc, rom);
            processed++;
            continue;
        }

        child_send_midi(mc, msg, len);

        if ((change_mask & (PROGRAM_SELECTION_BANK_CHANGED | PROGRAM_SELECTION_PRESET_CHANGED)) != 0) {
            child_update_preset_name(shm, mc, rom);
        }

        processed++;
    }
}

static int child_detect_valid_bank_count(virusLib::Microcontroller *mc,
                                         virusLib::ROMFile *rom,
                                         int fallback_bank_count) {
    if (!rom) return fallback_bank_count > 0 ? fallback_bank_count : 1;

    int preset_count = rom->getPresetsPerBank();
    if (preset_count <= 0) preset_count = VIRUS_MAX_PRESETS_PER_BANK;
    if (preset_count > VIRUS_MAX_PRESETS_PER_BANK) preset_count = VIRUS_MAX_PRESETS_PER_BANK;

    int detected = 0;
    virusLib::ROMFile::TPreset pd{};
    for (int bank = 0; bank < VIRUS_MAX_BANKS; ++bank) {
        if (!child_get_single_preset(mc, rom, bank, 0, &pd)) break;

        if (child_use_mc_preset_map(rom)) {
            if (!child_validate_virus_a_bank(mc, rom, bank, preset_count)) break;
        }

        detected++;
    }

    if (detected > 0) return detected;
    return fallback_bank_count > 0 ? fallback_bank_count : 1;
}

static void child_build_preset_name_cache(virus_shm_t *shm,
                                          virusLib::Microcontroller *mc,
                                          virusLib::ROMFile *rom) {
    if (!shm) return;
    shm->preset_name_cache_ready = 0;
    memset((void*)shm->preset_name_cache, 0, sizeof(shm->preset_name_cache));
    if (!rom || !mc) return;

    int bank_count = shm->bank_count;
    if (bank_count < 0) bank_count = 0;
    if (bank_count > VIRUS_MAX_BANKS) bank_count = VIRUS_MAX_BANKS;
    int preset_count = shm->preset_count;
    if (preset_count < 0) preset_count = 0;
    if (preset_count > VIRUS_MAX_PRESETS_PER_BANK) preset_count = VIRUS_MAX_PRESETS_PER_BANK;

    virusLib::ROMFile::TPreset pd{};
    for (int bank = 0; bank < bank_count; ++bank) {
        for (int preset = 0; preset < preset_count; ++preset) {
            if (!child_get_single_preset(mc, rom, bank, preset, &pd)) continue;
            const std::string name = virusLib::ROMFile::getSingleName(pd);
            snprintf(shm->preset_name_cache[bank][preset],
                     sizeof(shm->preset_name_cache[bank][preset]),
                     "%s",
                     name.empty() ? "---" : name.c_str());
        }
    }

    shm->preset_name_cache_ready = 1;
}

static void child_update_preset_name(virus_shm_t *shm,
                                     virusLib::Microcontroller *mc,
                                     virusLib::ROMFile *rom) {
    const char *cached = shm_lookup_preset_name(shm, shm->current_bank, shm->current_preset);
    if (cached) {
        snprintf((char*)shm->preset_name, sizeof(shm->preset_name), "%s", cached);
        return;
    }

    virusLib::ROMFile::TPreset pd{};
    if (child_get_single_preset(mc, rom, shm->current_bank, shm->current_preset, &pd))
        snprintf((char*)shm->preset_name, sizeof(shm->preset_name), "%s",
                 virusLib::ROMFile::getSingleName(pd).c_str());
    else
        snprintf((char*)shm->preset_name, sizeof(shm->preset_name), "---");
}

static virus_shm_t *g_child_shm = nullptr;

static void child_crash_handler(int sig) {
    /* Write crash info to debug log before dying */
    FILE *f = fopen("/data/UserData/move-anything/virus_debug.log", "a");
    if (f) {
        fprintf(f, "[child] CRASHED signal=%d (%s) pid=%d\n",
                sig, sig == 11 ? "SIGSEGV" : sig == 7 ? "SIGBUS" : sig == 6 ? "SIGABRT" : "?",
                (int)getpid());
        fflush(f);
        fclose(f);
    }
    signal(sig, SIG_DFL);
    raise(sig);
}

static void child_main(virus_shm_t *shm) {
    g_child_shm = shm;

    /* Install crash handler so we get diagnostics */
    signal(SIGSEGV, child_crash_handler);
    signal(SIGBUS, child_crash_handler);
    signal(SIGABRT, child_crash_handler);

    /* After fork in a multithreaded process, avoid stdio teardown inherited
     * from parent; just drop the pointer and reopen lazily in child. */
    g_vlog = nullptr;

    vlog("[child] started, pid=%d", (int)getpid());
    fprintf(stderr, "Virus child: started pid=%d\n", (int)getpid());

    /* 1. Load ROM */
    snprintf((char*)shm->loading_status, sizeof(shm->loading_status), "Loading ROM...");
    char roms_dir[512];
    snprintf(roms_dir, sizeof(roms_dir), "%s/roms", shm->module_dir);
    auto roms = virusLib::ROMLoader::findROMs(std::string(roms_dir));
    if (roms.empty()) {
        snprintf((char*)shm->load_error, sizeof(shm->load_error),
                 "No Virus ROM found in roms/ directory.");
        shm->initialized = 1; shm->loading_complete = 1;
        vlog("[child] no ROM found, exiting");
        return;
    }

    /* Sort ROMs by model (A=0, B=1, C=2, ...) so enum goes A → B → C */
    std::sort(roms.begin(), roms.end(), [](const virusLib::ROMFile &a, const virusLib::ROMFile &b) {
        return static_cast<int>(a.getModel()) < static_cast<int>(b.getModel());
    });

    /* Enumerate all ROMs for the UI */
    shm->rom_count = (int)roms.size();
    if (shm->rom_count > 8) shm->rom_count = 8;
    for (int i = 0; i < shm->rom_count; i++) {
        virusLib::ROMFile tmp{roms[i]};
        snprintf(shm->rom_names[i], sizeof(shm->rom_names[i]), "Virus %s",
                 tmp.getModelName().c_str());
        vlog("[child] ROM[%d]: %s", i, shm->rom_names[i]);
    }

    /* Pick the selected ROM */
    int idx = shm->rom_index;
    if (idx < 0 || idx >= shm->rom_count) idx = 0;
    virusLib::ROMFile *rom = new virusLib::ROMFile(std::move(roms[idx]));
    vlog("[child] ROM loaded: %s model=%s (index %d/%d)",
         rom->getFilename().c_str(), rom->getModelName().c_str(), idx, shm->rom_count);

    /* 2. Create DSP instances */
    snprintf((char*)shm->loading_status, sizeof(shm->loading_status), "Creating DSP...");
    vlog("[child] creating DspSingle...");
    virusLib::DspSingle *dsp1 = nullptr;
    virusLib::DspSingle *dsp2 = nullptr;
    try {
        virusLib::Device::createDspInstances(dsp1, dsp2, *rom, DEVICE_RATE);
    } catch (const std::exception& e) {
        snprintf((char*)shm->load_error, sizeof(shm->load_error), "DSP creation failed: %s", e.what());
        shm->initialized = 1; shm->loading_complete = 1;
        vlog("[child] DSP creation failed: %s", e.what());
        delete rom;
        return;
    }
    vlog("[child] createDspInstances OK");

    /* 3. Create Microcontroller */
    vlog("[child] creating Microcontroller...");
    virusLib::Microcontroller *mc = new virusLib::Microcontroller(*dsp1, *rom, false);

    /* 4. Set up semaphore and audio callback */
    dsp56k::SpscSemaphore audio_sem(1);
    std::atomic<int32_t> notify_timeout{0};
    std::atomic<uint32_t> callback_count{0};
    std::atomic<bool> sem_active{false};

    auto& audio = dsp1->getAudio();
    audio.setCallback([&](dsp56k::Audio* a) {
        mc->getMidiQueue(0).onAudioWritten();
        uint32_t count = callback_count.fetch_add(1) + 1;
        if ((count & 0x3) == 0) {
            std::vector<synthLib::SMidiEvent> midiOut;
            mc->readMidiOut(midiOut);
            mc->process();
        }
        if (sem_active.load(std::memory_order_relaxed)) {
            const auto avail = a->getAudioOutputs().size();
            int32_t timeout = notify_timeout.load();
            timeout--;
            if (timeout <= 0 && avail >= (EMU_CHUNK - 4)) {
                timeout = (EMU_CHUNK - 4);
                audio_sem.notify();
            }
            notify_timeout.store(timeout);
        }
    }, 0);

    /* 5. Boot DSPs */
    snprintf((char*)shm->loading_status, sizeof(shm->loading_status), "Booting DSP...");
    vlog("[child] bootDSPs...");
    virusLib::Device::bootDSPs(dsp1, dsp2, *rom, false);
    vlog("[child] bootDSPs OK");

    /* 6. Boot drain */
    {
        constexpr int BOOT_CHUNK = 8;
        float dummy_l[BOOT_CHUNK], dummy_r[BOOT_CHUNK];
        synthLib::TAudioInputs inputs = {};
        synthLib::TAudioOutputs outputs = {};
        outputs[0] = dummy_l; outputs[1] = dummy_r;

        int retries = 0;
        if (rom->getModel() == virusLib::DeviceModel::A) {
            for (int i = 0; i < 32; i++)
                dsp1->processAudio(inputs, outputs, BOOT_CHUNK, 0);
            dsp1->disableESSI1();
            retries = 32;
        } else {
            while (!mc->dspHasBooted() && retries < 512) {
                if (shm->child_shutdown) goto cleanup;
                dsp1->processAudio(inputs, outputs, BOOT_CHUNK, 0);
                retries++;
            }
        }
        vlog("[child] boot drain: %d cycles (%d frames)", retries, retries * BOOT_CHUNK);

        /* 7. Initialize */
        mc->sendInitControlCommands(127);
        for (int i = 0; i < 8; i++)
            dsp1->processAudio(inputs, outputs, BOOT_CHUNK, 0);
        mc->createDefaultState();
        for (int i = 0; i < 8; i++)
            dsp1->processAudio(inputs, outputs, BOOT_CHUNK, 0);
    }
    vlog("[child] DSP initialized");

    /* 7b. Apply DSP clock scaling.
     * Virus A (72 MHz) runs fine at 100%. Virus B/C (108 MHz) need reduction
     * to fit Move's A72 CPU budget. User can override via dsp_clock param. */
    {
        int pct = shm->dsp_clock_percent;
        if (pct <= 0) {
            switch (rom->getModel()) {
                case virusLib::DeviceModel::A: pct = 100; break;
                case virusLib::DeviceModel::B: pct = 45;  break;
                default:                       pct = 35;  break; /* C and others */
            }
        }
        dsp1->getEsxiClock().setSpeedPercent(pct);
        shm->dsp_clock_applied = pct;
        vlog("[child] DSP clock set to %d%% for model %s", pct, rom->getModelName().c_str());
    }
    strncpy((char*)shm->rom_model_name, rom->getModelName().c_str(), sizeof(shm->rom_model_name) - 1);

    /* 8. Set up presets */
    shm->bank_count = child_detect_valid_bank_count(
        mc, rom, (int)virusLib::ROMFile::getRomBankCount(rom->getModel()));
    shm->preset_count = rom->getPresetsPerBank();
    shm->current_bank = 0;
    shm->current_preset = 0;
    snprintf((char*)shm->bank_name, sizeof(shm->bank_name), "Bank A");
    child_build_preset_name_cache(shm, mc, rom);
    child_update_preset_name(shm, mc, rom);

    { /* Initial preset */
        if (child_use_mc_preset_map(rom)) {
            virusLib::ROMFile::TPreset single{};
            if (child_get_single_preset(mc, rom, shm->current_bank, shm->current_preset, &single))
                mc->writeSingle(virusLib::BankNumber::EditBuffer, virusLib::SINGLE, single);
        } else {
            synthLib::SMidiEvent bankSel(synthLib::MidiEventSource::Host, 0xB0, 32,
                                         bank_index_to_midi_lsb(shm->current_bank, shm->bank_count));
            mc->sendMIDI(bankSel);
            synthLib::SMidiEvent progChg(synthLib::MidiEventSource::Host, 0xC0, 0, 0);
            mc->sendMIDI(progChg);
        }
    }

    /* 9. Pre-fill ring buffer */
    snprintf((char*)shm->loading_status, sizeof(shm->loading_status), "Warming up...");
    {
        float wl[EMU_CHUNK], wr_buf[EMU_CHUNK];
        synthLib::TAudioInputs inputs = {};
        synthLib::TAudioOutputs outputs = {};
        outputs[0] = wl; outputs[1] = wr_buf;

        for (int fill = 0; fill < 64 && shm_ring_free(shm) >= EMU_CHUNK; fill++) {
            dsp1->processAudio(inputs, outputs, EMU_CHUNK, 0);
            int wr = shm->ring_write;
            for (int i = 0; i < EMU_CHUNK; i++) {
                int32_t l = (int32_t)(wl[i] * 32767.0f);
                int32_t r = (int32_t)(wr_buf[i] * 32767.0f);
                if (l > 32767) l = 32767; if (l < -32768) l = -32768;
                if (r > 32767) r = 32767; if (r < -32768) r = -32768;
                shm->audio_ring[wr * 2 + 0] = (int16_t)l;
                shm->audio_ring[wr * 2 + 1] = (int16_t)r;
                wr = (wr + 1) % AUDIO_RING_SIZE;
            }
            shm->ring_write = wr;
        }
    }
    vlog("[child] pre-filled %d frames", shm_ring_available(shm));

    /* 10. Signal ready and enter emu loop */
    shm->initialized = 1;
    shm->loading_complete = 1;
    shm->child_ready = 1;
    sem_active.store(true);
    notify_timeout.store(0);
    snprintf((char*)shm->loading_status, sizeof(shm->loading_status),
             "Ready: %d banks, %d presets/bank", shm->bank_count, shm->preset_count);
    vlog("[child] READY! entering emu loop");

    /* === Emu loop (runs until shutdown) === */
    {
        shm->prof_start_us = now_us();
        float proc_l[EMU_CHUNK], proc_r[EMU_CHUNK];
        double resample_phase = 0.0;
        float resample_prev_l = 0.0f, resample_prev_r = 0.0f;

        while (!shm->child_shutdown) {
            /* Process incoming MIDI from parent */
            child_process_midi_fifo(shm, mc, rom);

            /* Throttle: don't let ring fill beyond target (keeps latency low) */
            if (shm_ring_available(shm) >= RING_TARGET_FILL) {
                usleep(500);
                continue;
            }

            /* Wait for DSP to produce enough frames */
            audio_sem.wait();
            if (shm->child_shutdown) break;

            synthLib::TAudioInputs inputs = {};
            synthLib::TAudioOutputs outputs = {};
            outputs[0] = proc_l; outputs[1] = proc_r;

            int64_t t0 = now_us();
            dsp1->processAudio(inputs, outputs, EMU_CHUNK, EMU_CHUNK);
            int64_t dt = now_us() - t0;
            shm->prof_process_us_total += dt;
            if ((int)dt > shm->prof_process_max_us) shm->prof_process_max_us = (int)dt;
            shm->emu_blocks++;
            shm->child_alive++;

            /* Peak tracking */
            for (int i = 0; i < EMU_CHUNK; i++) {
                float peak = fabsf(proc_l[i]);
                float pr = fabsf(proc_r[i]);
                if (pr > peak) peak = pr;
                if (peak > shm->prof_peak_level) shm->prof_peak_level = peak;
            }

            /* Check for dynamic clock change */
            {
                int want = shm->dsp_clock_percent;
                if (want > 0 && want != shm->dsp_clock_applied) {
                    dsp1->getEsxiClock().setSpeedPercent(want);
                    shm->dsp_clock_applied = want;
                    vlog("[child] DSP clock changed to %d%%", want);
                }
            }

            /* Periodic stats */
            if ((shm->emu_blocks % 1465) == 1) {
                int64_t now = now_us();
                double elapsed_s = (now - shm->prof_start_us) / 1000000.0;
                double mips = 0;
                if (elapsed_s > 0.1)
                    mips = (double)shm->emu_blocks * EMU_CHUNK * 2304.0 / elapsed_s / 1000000.0;
                vlog("[child] blk=%d buf=%d ur=%d mips=%.1f peak=%.3f",
                     shm->emu_blocks, shm_ring_available(shm), shm->underrun_count, mips,
                     (double)shm->prof_peak_level);
                shm->prof_peak_level = 0.0f;
            }

            /* Resample 46875→44100 */
            float ext_l[EMU_CHUNK + 1], ext_r[EMU_CHUNK + 1];
            ext_l[0] = resample_prev_l; ext_r[0] = resample_prev_r;
            memcpy(ext_l + 1, proc_l, EMU_CHUNK * sizeof(float));
            memcpy(ext_r + 1, proc_r, EMU_CHUNK * sizeof(float));

            float gain = (shm->gain_percent > 0) ? (shm->gain_percent / 100.0f) : OUTPUT_GAIN;

            int16_t resampled[RESAMPLE_MAX_OUT * 2];
            int out_count = 0;
            while (resample_phase < (double)EMU_CHUNK && out_count < RESAMPLE_MAX_OUT) {
                double ext_pos = resample_phase + 1.0;
                int idx = (int)ext_pos;
                double frac = ext_pos - idx;
                if (idx >= EMU_CHUNK) break;
                float l = ext_l[idx] * (float)(1.0 - frac) + ext_l[idx + 1] * (float)frac;
                float r = ext_r[idx] * (float)(1.0 - frac) + ext_r[idx + 1] * (float)frac;
                int32_t li = (int32_t)(l * gain * 32767.0f);
                int32_t ri = (int32_t)(r * gain * 32767.0f);
                if (li > 32767) li = 32767; if (li < -32768) li = -32768;
                if (ri > 32767) ri = 32767; if (ri < -32768) ri = -32768;
                resampled[out_count * 2 + 0] = (int16_t)li;
                resampled[out_count * 2 + 1] = (int16_t)ri;
                out_count++;
                resample_phase += RESAMPLE_RATIO;
            }
            resample_phase -= (double)EMU_CHUNK;
            resample_prev_l = proc_l[EMU_CHUNK - 1];
            resample_prev_r = proc_r[EMU_CHUNK - 1];

            /* Write to shared ring buffer */
            if (shm_ring_free(shm) < out_count) continue;
            int wr = shm->ring_write;
            for (int i = 0; i < out_count; i++) {
                shm->audio_ring[wr * 2 + 0] = resampled[i * 2 + 0];
                shm->audio_ring[wr * 2 + 1] = resampled[i * 2 + 1];
                wr = (wr + 1) % AUDIO_RING_SIZE;
            }
            shm->ring_write = wr;
        }
    }

cleanup:
    vlog("[child] shutting down");
    audio.setCallback(nullptr, 0);
    audio.terminate();
    delete mc;
    delete dsp1;
    delete rom;
    vlog("[child] exiting");
}

/* =====================================================================
 * Boot / restart helpers (parent process)
 * ===================================================================== */

/* Fork child and wait for it to be ready. Returns 0 on success. */
static int fork_and_wait_child(virus_instance_t *inst) {
    virus_shm_t *shm = inst->shm;

    vlog("fork_and_wait: forking child process for DSP...");

    pid_t pid = fork();
    if (pid < 0) {
        snprintf((char*)shm->load_error, sizeof(shm->load_error), "fork() failed: %s", strerror(errno));
        shm->initialized = 1; shm->loading_complete = 1;
        return -1;
    }

    if (pid == 0) {
        /* === CHILD PROCESS === */
        g_vlog = nullptr;
        child_main(shm);
        _exit(0);
    }

    /* === PARENT PROCESS === */
    inst->child_pid = pid;
    vlog("fork_and_wait: child forked, pid=%d", (int)pid);

    for (int i = 0; i < 600 && !shm->child_ready; i++) {
        int status;
        pid_t res = waitpid(pid, &status, WNOHANG);
        if (res == pid) {
            snprintf((char*)shm->load_error, sizeof(shm->load_error),
                     "DSP process exited unexpectedly (status=%d)", status);
            shm->initialized = 1; shm->loading_complete = 1;
            inst->child_pid = 0;
            vlog("fork_and_wait: child died during boot, status=%d", status);
            return -1;
        }
        usleep(100000);
    }

    if (!shm->child_ready) {
        snprintf((char*)shm->load_error, sizeof(shm->load_error), "DSP boot timed out (60s)");
        shm->initialized = 1; shm->loading_complete = 1;
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, 0);
        inst->child_pid = 0;
        return -1;
    }

    vlog("fork_and_wait: child ready");
    return 0;
}

/* Kill existing child and reset shm state for a fresh boot.
 * Preserves: module_dir, rom_index, dsp_clock_percent, gain_percent */
static void kill_child_and_reset(virus_instance_t *inst) {
    virus_shm_t *shm = inst->shm;
    if (inst->child_pid > 0) {
        shm->child_shutdown = 1;
        for (int i = 0; i < 30; i++) {
            int status;
            if (waitpid(inst->child_pid, &status, WNOHANG) == inst->child_pid) break;
            usleep(100000);
        }
        kill(inst->child_pid, SIGKILL);
        waitpid(inst->child_pid, nullptr, 0);
        inst->child_pid = 0;
    }

    /* Reset transient state but keep config */
    shm->ring_read = 0;
    shm->ring_write = 0;
    shm->midi_read = 0;
    shm->midi_write = 0;
    shm->child_ready = 0;
    shm->child_shutdown = 0;
    shm->child_alive = 0;
    shm->initialized = 0;
    shm->loading_complete = 0;
    shm->load_error[0] = '\0';
    shm->underrun_count = 0;
    shm->emu_blocks = 0;
    shm->render_count = 0;
    shm->prof_process_us_total = 0;
    shm->prof_process_max_us = 0;
    shm->prof_peak_level = 0.0f;
    shm->prof_ring_min = 0;
    shm->dsp_clock_applied = 0;
}

static void* boot_thread_func(void *arg) {
    virus_instance_t *inst = (virus_instance_t*)arg;
    virus_shm_t *shm = inst->shm;

    vlog("boot thread: waiting 3s for system to stabilize...");
    snprintf((char*)shm->loading_status, sizeof(shm->loading_status), "Waiting...");
    for (int i = 0; i < 3; i++) sleep(1);

    fork_and_wait_child(inst);

    vlog("boot thread: boot complete");
    inst->boot_thread_running = 0;
    return nullptr;
}

/* Restart thread — kills existing child and boots a new one with different ROM */
static void* restart_thread_func(void *arg) {
    virus_instance_t *inst = (virus_instance_t*)arg;
    virus_shm_t *shm = inst->shm;

    vlog("restart thread: killing child for ROM switch...");
    snprintf((char*)shm->loading_status, sizeof(shm->loading_status), "Switching ROM...");

    kill_child_and_reset(inst);

    vlog("restart thread: waiting 1s...");
    sleep(1);

    fork_and_wait_child(inst);

    vlog("restart thread: restart complete");
    inst->boot_thread_running = 0;
    return nullptr;
}

/* =====================================================================
 * Plugin API v2
 * ===================================================================== */

static void* v2_create_instance(const char *module_dir, const char *json_defaults) {
    (void)json_defaults;

    virus_instance_t *inst = (virus_instance_t*)calloc(1, sizeof(virus_instance_t));
    if (!inst) return nullptr;

    /* Allocate shared memory (MAP_SHARED so it persists across fork) */
    inst->shm = (virus_shm_t*)mmap(nullptr, sizeof(virus_shm_t),
                                    PROT_READ | PROT_WRITE,
                                    MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (inst->shm == MAP_FAILED) {
        fprintf(stderr, "Virus: mmap failed: %s\n", strerror(errno));
        free(inst);
        return nullptr;
    }
    memset(inst->shm, 0, sizeof(virus_shm_t));
    strncpy((char*)inst->shm->module_dir, module_dir, sizeof(inst->shm->module_dir) - 1);

    /* Set default CC values (Virus Page A indices) */
    inst->shm->cc_values[40] = 127;  /* Cutoff */
    inst->shm->cc_values[42] = 0;    /* Resonance */
    inst->shm->cc_values[91] = 100;  /* Patch Volume */

    fprintf(stderr, "Virus: creating instance from %s (fork mode)\n", module_dir);
    snprintf((char*)inst->shm->loading_status, sizeof(inst->shm->loading_status), "Initializing...");

    inst->boot_thread_running = 1;
    pthread_create(&inst->boot_thread, nullptr, boot_thread_func, inst);
    return inst;
}

static void v2_destroy_instance(void *instance) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst) return;
    fprintf(stderr, "Virus: destroying\n");

    /* Signal child to shutdown */
    if (inst->shm) inst->shm->child_shutdown = 1;

    /* Wait for boot thread */
    if (inst->boot_thread_running)
        pthread_join(inst->boot_thread, nullptr);

    /* Kill child process */
    if (inst->child_pid > 0) {
        kill(inst->child_pid, SIGTERM);
        /* Wait up to 3 seconds */
        for (int i = 0; i < 30; i++) {
            int status;
            if (waitpid(inst->child_pid, &status, WNOHANG) == inst->child_pid) break;
            usleep(100000);
        }
        kill(inst->child_pid, SIGKILL);
        waitpid(inst->child_pid, nullptr, 0);
    }

    /* Unmap shared memory */
    if (inst->shm && inst->shm != MAP_FAILED)
        munmap(inst->shm, sizeof(virus_shm_t));

    if (inst->pending_state) free(inst->pending_state);
    free(inst);
    fprintf(stderr, "Virus: destroyed\n");
}

static void v2_on_midi(void *instance, const uint8_t *msg, int len, int source) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst || !inst->shm || !inst->shm->initialized || len < 1) return;
    (void)source;

    uint8_t modified[8];
    int n = len > 8 ? 8 : len;
    memcpy(modified, msg, n);

    uint8_t status = msg[0] & 0xF0;

    /* Apply octave transpose to notes */
    if ((status == 0x90 || status == 0x80) && len >= 2) {
        int note = msg[1] + inst->shm->octave_transpose * 12;
        if (note < 0) note = 0; if (note > 127) note = 127;
        modified[1] = (uint8_t)note;
    }

    /* Track CC values locally too */
    if (status == 0xB0 && len >= 3)
        inst->shm->cc_values[msg[1] & 0x7F] = msg[2] & 0x7F;

    /* Push to shared MIDI FIFO for child to consume */
    midi_fifo_push(inst->shm, modified, n);
}

static int json_get_int(const char *json, const char *key, int *out) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *pos = strstr(json, search);
    if (!pos) return -1;
    pos += strlen(search);
    while (*pos == ' ') pos++;
    *out = atoi(pos);
    return 0;
}

static void v2_set_param(void *instance, const char *key, const char *val) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst || !inst->shm) return;
    virus_shm_t *shm = inst->shm;

    if (strcmp(key, "state") == 0) {
        if (!shm->loading_complete || !shm->child_ready) {
            if (inst->pending_state) free(inst->pending_state);
            inst->pending_state = strdup(val);
            inst->pending_state_valid = 1;
            return;
        }
        int ival;
        bool has_preset = false;
        int preset_from_state = 0;
        if (json_get_int(val, "bank", &ival) == 0 && ival >= 0 && ival < shm->bank_count) {
            shm->current_bank = ival;
            snprintf((char*)shm->bank_name, sizeof(shm->bank_name), "Bank %c", 'A' + ival);
        }
        if (json_get_int(val, "preset", &ival) == 0 && ival >= 0 && ival < shm->preset_count) {
            has_preset = true;
            preset_from_state = ival;
        }
        if (has_preset) {
            shm->current_preset = preset_from_state;
            uint8_t cc32[3] = { 0xB0, 32, bank_index_to_midi_lsb(shm->current_bank, shm->bank_count) };
            midi_fifo_push(shm, cc32, 3);
            uint8_t pc[2] = { 0xC0, (uint8_t)shm->current_preset };
            midi_fifo_push(shm, pc, 2);
        }
        if (json_get_int(val, "octave_transpose", &ival) == 0) {
            if (ival < -4) ival = -4; if (ival > 4) ival = 4;
            shm->octave_transpose = ival;
        }
        if (json_get_int(val, "dsp_clock", &ival) == 0) {
            if (ival < 10) ival = 10; if (ival > 100) ival = 100;
            shm->dsp_clock_percent = ival;
        }
        if (json_get_int(val, "rom_index", &ival) == 0) {
            if (ival >= 0 && (shm->rom_count == 0 || ival < shm->rom_count) && ival != shm->rom_index) {
                shm->rom_index = ival;
                if (shm->child_ready && !inst->boot_thread_running) {
                    inst->boot_thread_running = 1;
                    pthread_t t;
                    pthread_create(&t, nullptr, restart_thread_func, inst);
                    pthread_detach(t);
                }
            }
        }
        if (json_get_int(val, "gain", &ival) == 0) {
            if (ival < 1) ival = 1; if (ival > 100) ival = 100;
            shm->gain_percent = ival;
        }
        for (int i = 0; i < NUM_PARAMS; i++) {
            if (json_get_int(val, g_params[i].key, &ival) == 0) {
                if (ival < g_params[i].min_val) ival = g_params[i].min_val;
                if (ival > g_params[i].max_val) ival = g_params[i].max_val;
                uint8_t cc[3] = { 0xB0, (uint8_t)g_params[i].cc, (uint8_t)ival };
                midi_fifo_push(shm, cc, 3);
            }
        }
        shm_refresh_current_preset_name(shm);
        return;
    }
    if (strcmp(key, "preset") == 0) {
        int idx = atoi(val);
        if (idx >= 0 && idx < shm->preset_count) {
            if (idx == shm->current_preset) {
                shm_refresh_current_preset_name(shm);
                return;
            }
            shm->current_preset = idx;
            uint8_t pc[2] = { 0xC0, (uint8_t)idx };
            midi_fifo_push(shm, pc, 2);
            shm_refresh_current_preset_name(shm);
        }
        return;
    }
    if (strcmp(key, "bank_index") == 0) {
        int idx = atoi(val);
        if (idx >= 0 && idx < shm->bank_count) {
            if (idx == shm->current_bank) {
                shm_refresh_current_preset_name(shm);
                return;
            }
            shm->current_bank = idx;
            shm->current_preset = 0;
            snprintf((char*)shm->bank_name, sizeof(shm->bank_name), "Bank %c", 'A' + idx);
            uint8_t cc32[3] = { 0xB0, 32, bank_index_to_midi_lsb(idx, shm->bank_count) };
            midi_fifo_push(shm, cc32, 3);
            uint8_t pc[2] = { 0xC0, 0 };
            midi_fifo_push(shm, pc, 2);
            shm_refresh_current_preset_name(shm);
        }
        return;
    }
    if (strcmp(key, "octave_transpose") == 0) {
        int v = atoi(val);
        if (v < -4) v = -4; if (v > 4) v = 4;
        shm->octave_transpose = v;
        return;
    }
    if (strcmp(key, "rom_index") == 0) {
        /* Accept either a numeric index or a ROM name string */
        int v = -1;
        for (int i = 0; i < shm->rom_count; i++) {
            if (strcmp(val, shm->rom_names[i]) == 0) { v = i; break; }
        }
        if (v < 0) v = atoi(val);  /* fall back to numeric */
        if (v < 0) v = 0;
        if (shm->rom_count > 0 && v >= shm->rom_count) v = shm->rom_count - 1;
        if (v == shm->rom_index && shm->child_ready) return; /* no change */
        shm->rom_index = v;
        /* Reset clock to auto so new ROM gets its appropriate default */
        shm->dsp_clock_percent = 0;
        /* Restart child with new ROM (in background thread) */
        if (!inst->boot_thread_running) {
            inst->boot_thread_running = 1;
            pthread_t t;
            pthread_create(&t, nullptr, restart_thread_func, inst);
            pthread_detach(t);
        }
        return;
    }
    if (strcmp(key, "dsp_clock") == 0) {
        int v = atoi(val);
        if (v < 10) v = 10; if (v > 100) v = 100;
        shm->dsp_clock_percent = v;
        return;
    }
    if (strcmp(key, "gain") == 0) {
        int v = atoi(val);
        if (v < 1) v = 1; if (v > 100) v = 100;
        shm->gain_percent = v;
        return;
    }
    if (strcmp(key, "all_notes_off") == 0) {
        uint8_t msg[3] = { 0xB0, 123, 0 };
        midi_fifo_push(shm, msg, 3);
        return;
    }
    for (int i = 0; i < NUM_PARAMS; i++) {
        if (strcmp(key, g_params[i].key) == 0) {
            int ival = atoi(val);
            if (ival < g_params[i].min_val) ival = g_params[i].min_val;
            if (ival > g_params[i].max_val) ival = g_params[i].max_val;
            uint8_t cc[3] = { 0xB0, (uint8_t)g_params[i].cc, (uint8_t)ival };
            midi_fifo_push(shm, cc, 3);
            return;
        }
    }
}

static int v2_get_param(void *instance, const char *key, char *buf, int buf_len) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst || !inst->shm) return -1;
    virus_shm_t *shm = inst->shm;

    if (strcmp(key, "preset") == 0) return snprintf(buf, buf_len, "%d", shm->current_preset);
    if (strcmp(key, "preset_count") == 0) return snprintf(buf, buf_len, "%d", shm->preset_count);
    if (strcmp(key, "preset_name") == 0) {
        shm_refresh_current_preset_name(shm);
        return snprintf(buf, buf_len, "%s", (const char*)shm->preset_name);
    }
    if (strcmp(key, "name") == 0) return snprintf(buf, buf_len, "Virus");
    if (strcmp(key, "bank_index") == 0) return snprintf(buf, buf_len, "%d", shm->current_bank);
    if (strcmp(key, "bank_count") == 0) return snprintf(buf, buf_len, "%d", shm->bank_count);
    if (strcmp(key, "bank_name") == 0) return snprintf(buf, buf_len, "%s", (const char*)shm->bank_name);
    if (strcmp(key, "patch_in_bank") == 0) return snprintf(buf, buf_len, "%d", shm->current_preset + 1);
    if (strcmp(key, "octave_transpose") == 0) return snprintf(buf, buf_len, "%d", shm->octave_transpose);
    if (strcmp(key, "dsp_clock") == 0) return snprintf(buf, buf_len, "%d", shm->dsp_clock_applied > 0 ? shm->dsp_clock_applied : (shm->dsp_clock_percent > 0 ? shm->dsp_clock_percent : 40));
    if (strcmp(key, "rom_model") == 0) return snprintf(buf, buf_len, "%s", shm->rom_model_name[0] ? (const char*)shm->rom_model_name : "?");
    if (strcmp(key, "rom_index") == 0) {
        int idx = shm->rom_index;
        if (idx >= 0 && idx < shm->rom_count && shm->rom_names[idx][0])
            return snprintf(buf, buf_len, "%s", shm->rom_names[idx]);
        return snprintf(buf, buf_len, "%d", idx);
    }
    if (strcmp(key, "rom_count") == 0) return snprintf(buf, buf_len, "%d", shm->rom_count);
    if (strcmp(key, "rom_name") == 0) {
        int idx = shm->rom_index;
        if (idx >= 0 && idx < shm->rom_count && shm->rom_names[idx][0])
            return snprintf(buf, buf_len, "%s", shm->rom_names[idx]);
        return snprintf(buf, buf_len, "---");
    }
    if (strcmp(key, "rom_list") == 0) {
        int off = 0;
        for (int i = 0; i < shm->rom_count && off < buf_len - 80; i++) {
            if (i > 0) off += snprintf(buf+off, buf_len-off, ",");
            off += snprintf(buf+off, buf_len-off, "%s", shm->rom_names[i]);
        }
        return off;
    }
    if (strcmp(key, "gain") == 0) return snprintf(buf, buf_len, "%d", shm->gain_percent > 0 ? shm->gain_percent : 70);
    if (strcmp(key, "loading_status") == 0) return snprintf(buf, buf_len, "%s", (const char*)shm->loading_status);
    if (strcmp(key, "debug_info") == 0) {
        int avail = shm_ring_available(shm);
        int blocks = shm->emu_blocks > 0 ? shm->emu_blocks : 1;
        return snprintf(buf, buf_len,
            "buf=%d min=%d ur=%d blk=%d proc_avg=%lld proc_max=%d peak=%.2f pid=%d",
            avail, shm->prof_ring_min, shm->underrun_count, shm->emu_blocks,
            (long long)(shm->prof_process_us_total / blocks),
            shm->prof_process_max_us,
            (double)shm->prof_peak_level,
            (int)inst->child_pid);
    }
    if (strcmp(key, "prof_reset") == 0) {
        shm->prof_process_us_total = 0;
        shm->prof_ring_min = AUDIO_RING_SIZE;
        shm->prof_process_max_us = 0;
        shm->prof_peak_level = 0.0f;
        shm->underrun_count = 0;
        shm->emu_blocks = 0;
        return snprintf(buf, buf_len, "reset");
    }
    for (int i = 0; i < NUM_PARAMS; i++)
        if (strcmp(key, g_params[i].key) == 0)
            return snprintf(buf, buf_len, "%d", shm->cc_values[g_params[i].cc]);

    if (strcmp(key, "state") == 0) {
        int off = 0;
        off += snprintf(buf+off, buf_len-off, "{\"bank\":%d,\"preset\":%d,\"octave_transpose\":%d",
            shm->current_bank, shm->current_preset, shm->octave_transpose);
        off += snprintf(buf+off, buf_len-off, ",\"dsp_clock\":%d,\"gain\":%d,\"rom_index\":%d",
            shm->dsp_clock_applied > 0 ? shm->dsp_clock_applied : 40,
            shm->gain_percent > 0 ? shm->gain_percent : 70, shm->rom_index);
        for (int i = 0; i < NUM_PARAMS; i++)
            off += snprintf(buf+off, buf_len-off, ",\"%s\":%d", g_params[i].key, shm->cc_values[g_params[i].cc]);
        off += snprintf(buf+off, buf_len-off, "}");
        return off;
    }
    if (strcmp(key, "ui_hierarchy") == 0) {
        const char *h = R"JSON({"modes":null,"levels":{"root":{"list_param":"preset","count_param":"preset_count","name_param":"preset_name","children":null,"knobs":["cutoff","resonance","filter_env","flt_attack","flt_decay","flt_sustain","flt_release","octave_transpose"],"params":[{"key":"bank_index","label":"Bank"},{"level":"filter","label":"Filter"},{"level":"amp","label":"Amp Env"},{"level":"osc","label":"Oscillators"},{"level":"settings","label":"Settings"}]},"filter":{"children":null,"knobs":["cutoff","resonance","filter_env","filter_mode"],"params":[{"key":"cutoff","label":"Cutoff"},{"key":"resonance","label":"Resonance"},{"key":"filter_env","label":"Filter Env"},{"key":"filter_mode","label":"Filter Mode"}]},"amp":{"children":null,"knobs":["amp_attack","amp_decay","amp_sustain","amp_release"],"params":[{"key":"amp_attack","label":"Amp Attack"},{"key":"amp_decay","label":"Amp Decay"},{"key":"amp_sustain","label":"Amp Sustain"},{"key":"amp_release","label":"Amp Release"}]},"osc":{"children":null,"knobs":["osc1_shape","osc2_shape","osc_balance","patch_volume"],"params":[{"key":"osc1_shape","label":"Osc1 Shape"},{"key":"osc2_shape","label":"Osc2 Shape"},{"key":"osc_balance","label":"Osc Balance"},{"key":"patch_volume","label":"Volume"}]},"settings":{"children":null,"knobs":["dsp_clock","gain"],"params":[{"key":"rom_index","label":"ROM"},{"key":"dsp_clock","label":"DSP Clock"},{"key":"gain","label":"Gain"}]}}})JSON";
        int len = strlen(h);
        if (len < buf_len) { strcpy(buf, h); return len; }
        return -1;
    }
    if (strcmp(key, "chain_params") == 0) {
        int off = 0;
        int bank_max = shm->bank_count > 0 ? shm->bank_count - 1 : 0;
        off += snprintf(buf+off, buf_len-off,
            "[{\"key\":\"preset\",\"name\":\"Preset\",\"type\":\"int\",\"min\":0,\"max\":127},"
            "{\"key\":\"bank_index\",\"name\":\"Bank\",\"type\":\"int\",\"min\":0,\"max\":%d},"
            "{\"key\":\"octave_transpose\",\"name\":\"Octave\",\"type\":\"int\",\"min\":-4,\"max\":4},"
            "{\"key\":\"rom_index\",\"name\":\"ROM\",\"type\":\"enum\",\"options\":[", bank_max);
        for (int i = 0; i < shm->rom_count; i++) {
            if (i > 0) off += snprintf(buf+off, buf_len-off, ",");
            off += snprintf(buf+off, buf_len-off, "\"%s\"", shm->rom_names[i]);
        }
        if (shm->rom_count == 0)
            off += snprintf(buf+off, buf_len-off, "\"(loading)\"");
        off += snprintf(buf+off, buf_len-off, "]},"
            "{\"key\":\"dsp_clock\",\"name\":\"DSP Clock %%\",\"type\":\"int\",\"min\":10,\"max\":100,\"step\":5},"
            "{\"key\":\"gain\",\"name\":\"Gain %%\",\"type\":\"int\",\"min\":1,\"max\":100}");
        for (int i = 0; i < NUM_PARAMS && off < buf_len - 100; i++)
            off += snprintf(buf+off, buf_len-off,
                ",{\"key\":\"%s\",\"name\":\"%s\",\"type\":\"int\",\"min\":%d,\"max\":%d}",
                g_params[i].key, g_params[i].name, g_params[i].min_val, g_params[i].max_val);
        off += snprintf(buf+off, buf_len-off, "]");
        return off;
    }
    return -1;
}

static int v2_get_error(void *instance, char *buf, int buf_len) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst || !inst->shm || inst->shm->load_error[0] == '\0') return 0;
    return snprintf(buf, buf_len, "%s", (const char*)inst->shm->load_error);
}

static void v2_render_block(void *instance, int16_t *out, int frames) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst || !inst->shm || !inst->shm->loading_complete || !inst->shm->child_ready) {
        memset(out, 0, frames * 2 * sizeof(int16_t));
        return;
    }

    /* Apply pending state that arrived before child was ready */
    if (inst->pending_state_valid && inst->pending_state) {
        inst->pending_state_valid = 0;
        char *state = inst->pending_state;
        inst->pending_state = nullptr;
        vlog("[parent] applying pending state after boot");
        v2_set_param(instance, "state", state);
        free(state);
    }

    virus_shm_t *shm = inst->shm;

    int avail = shm_ring_available(shm);
    if (shm->prof_ring_min == 0 || avail < shm->prof_ring_min)
        shm->prof_ring_min = avail;
    int to_read = (avail < frames) ? avail : frames;
    int rd = shm->ring_read;
    for (int i = 0; i < to_read; i++) {
        out[i*2+0] = shm->audio_ring[rd*2+0];
        out[i*2+1] = shm->audio_ring[rd*2+1];
        rd = (rd + 1) % AUDIO_RING_SIZE;
    }
    shm->ring_read = rd;

    if (to_read < frames) {
        shm->underrun_count++;
        memset(out + to_read * 2, 0, (frames - to_read) * 2 * sizeof(int16_t));
    }
    shm->render_count++;
}

/* =====================================================================
 * Entry point
 * ===================================================================== */

static plugin_api_v2_t g_plugin_api_v2;

extern "C" plugin_api_v2_t* move_plugin_init_v2(const host_api_v1_t *host) {
    g_host = host;
    memset(&g_plugin_api_v2, 0, sizeof(g_plugin_api_v2));
    g_plugin_api_v2.api_version = MOVE_PLUGIN_API_VERSION_2;
    g_plugin_api_v2.create_instance = v2_create_instance;
    g_plugin_api_v2.destroy_instance = v2_destroy_instance;
    g_plugin_api_v2.on_midi = v2_on_midi;
    g_plugin_api_v2.set_param = v2_set_param;
    g_plugin_api_v2.get_param = v2_get_param;
    g_plugin_api_v2.get_error = v2_get_error;
    g_plugin_api_v2.render_block = v2_render_block;
    return &g_plugin_api_v2;
}
