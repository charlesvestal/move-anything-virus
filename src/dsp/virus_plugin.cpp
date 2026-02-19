/*
 * Virus DSP Plugin for Move Anything
 *
 * Uses DspSingle + Microcontroller directly (bypassing Device/Plugin wrapper)
 * with a semaphore-based callback pattern from the gearmulator console app.
 * This eliminates the thread contention in waitNotEmpty() that caused the
 * Device+Plugin approach to achieve only ~63% real-time throughput.
 *
 * The DSP thread runs the JIT-compiled DSP56300 code and calls our callback
 * for each output frame. The callback:
 *   1. Advances MIDI timing (getMidiQueue.onAudioWritten)
 *   2. Periodically processes the Microcontroller
 *   3. Signals a semaphore when enough frames are available
 *
 * The emu thread waits on the semaphore, then reads a full block of audio
 * from processAudio() — since data is guaranteed available, waitNotEmpty()
 * never blocks, eliminating mutex contention entirely.
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
#include <vector>
#include <string>
#include <array>
#include <mutex>
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

/* Virus ABC native rate: 12000000/256 = 46875 Hz.
 * Audio will be ~6% slow / ~1 semitone flat at 44100 playback.
 * TODO: add proper 46875->44100 resampling. */
#define DEVICE_RATE         46875.0f

#define AUDIO_RING_SIZE     8192    /* Stereo frame pairs */
#define EMU_CHUNK           64      /* Frames per processAudio() call (matches console app) */
#define OUTPUT_GAIN         1.0f    /* Peak levels measured at ~0.4, no gain reduction needed */

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

/* =====================================================================
 * WAV capture helpers
 * ===================================================================== */

#define CAPTURE_SECONDS 10
#define CAPTURE_MAX_FRAMES (MOVE_SAMPLE_RATE * CAPTURE_SECONDS)

static FILE* wav_open(const char *path) {
    FILE *f = fopen(path, "wb");
    if (!f) return nullptr;
    uint8_t header[44] = {};
    header[0]='R'; header[1]='I'; header[2]='F'; header[3]='F';
    header[8]='W'; header[9]='A'; header[10]='V'; header[11]='E';
    header[12]='f'; header[13]='m'; header[14]='t'; header[15]=' ';
    header[16]=16;
    header[20]=1; /* PCM */
    header[22]=2; /* stereo */
    header[24]=(MOVE_SAMPLE_RATE>>0)&0xFF; header[25]=(MOVE_SAMPLE_RATE>>8)&0xFF;
    header[26]=(MOVE_SAMPLE_RATE>>16)&0xFF; header[27]=(MOVE_SAMPLE_RATE>>24)&0xFF;
    int br=MOVE_SAMPLE_RATE*4;
    header[28]=(br>>0)&0xFF; header[29]=(br>>8)&0xFF;
    header[30]=(br>>16)&0xFF; header[31]=(br>>24)&0xFF;
    header[32]=4; header[34]=16;
    header[36]='d'; header[37]='a'; header[38]='t'; header[39]='a';
    fwrite(header, 1, 44, f);
    return f;
}

static void wav_close(FILE *f, int frames_written) {
    if (!f) return;
    int ds = frames_written*4, fs = ds+36;
    uint8_t b[4];
    fseek(f, 4, SEEK_SET);
    b[0]=(fs>>0)&0xFF; b[1]=(fs>>8)&0xFF; b[2]=(fs>>16)&0xFF; b[3]=(fs>>24)&0xFF;
    fwrite(b, 1, 4, f);
    fseek(f, 40, SEEK_SET);
    b[0]=(ds>>0)&0xFF; b[1]=(ds>>8)&0xFF; b[2]=(ds>>16)&0xFF; b[3]=(ds>>24)&0xFF;
    fwrite(b, 1, 4, f);
    fclose(f);
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

static const virus_param_t g_params[] = {
    {"cutoff",       "Cutoff",        74, 0, 127},
    {"resonance",    "Resonance",     71, 0, 127},
    {"filter_env",   "Filter Env",    77, 0, 127},
    {"attack",       "Attack",        73, 0, 127},
    {"decay",        "Decay",         75, 0, 127},
    {"sustain",      "Sustain",       76, 0, 127},
    {"release",      "Release",       72, 0, 127},
    {"filter_type",  "Filter Type",   68, 0, 127},
    {"osc1_shape",   "Osc1 Shape",    69, 0, 127},
    {"osc2_shape",   "Osc2 Shape",    70, 0, 127},
    {"osc_balance",  "Osc Balance",   21, 0, 127},
    {"chorus_mix",   "Chorus Mix",    93, 0, 127},
    {"delay_time",   "Delay Time",    89, 0, 127},
    {"delay_fb",     "Delay FB",      90, 0, 127},
    {"reverb_mix",   "Reverb Mix",    91, 0, 127},
    {"lfo1_rate",    "LFO1 Rate",     67, 0, 127},
};
static const int NUM_PARAMS = sizeof(g_params) / sizeof(g_params[0]);

/* =====================================================================
 * Instance structure
 * ===================================================================== */

struct virus_instance_t {
    char module_dir[256];

    /* Direct DSP access (no Device/Plugin wrapper) */
    virusLib::DspSingle *dsp;
    virusLib::Microcontroller *mc;
    virusLib::ROMFile *rom;
    dsp56k::SpscSemaphore *audio_sem;

    /* Callback state (accessed from DSP thread) */
    std::atomic<int32_t> notify_timeout;
    std::atomic<uint32_t> callback_count;

    pthread_t boot_thread;
    pthread_t emu_thread;
    volatile int boot_thread_running;
    volatile int emu_thread_running;
    volatile int initialized;
    volatile int loading_complete;
    volatile int shutting_down;

    /* Audio ring buffer (stereo interleaved int16) */
    int16_t audio_ring[AUDIO_RING_SIZE * 2];
    volatile int ring_read;
    volatile int ring_write;

    /* Preset state */
    int current_bank;
    int current_preset;
    int bank_count;
    int preset_count;
    char preset_name[64];
    char bank_name[32];
    int octave_transpose;

    int cc_values[128];

    /* Resampler state (46875 → 44100) */
    double resample_phase;   /* fractional position in source block */
    float resample_prev_l;   /* last sample from previous block (for interpolation) */
    float resample_prev_r;

    char loading_status[128];
    char load_error[256];
    volatile int underrun_count;
    volatile int render_count;
    volatile int emu_blocks;

    /* Profiling stats (updated by emu thread, read by get_param) */
    volatile int64_t prof_poll_us_total;   /* cumulative poll wait µs */
    volatile int64_t prof_process_us_total; /* cumulative processAudio µs */
    volatile int prof_timeout_count;       /* polls that hit 50-iteration limit */
    volatile int prof_ring_min;            /* lowest ring_available seen in render */
    volatile int prof_poll_max_us;         /* worst-case single poll wait µs */
    volatile int prof_process_max_us;      /* worst-case single processAudio µs */
    volatile float prof_peak_level;        /* peak absolute sample level (float, pre-clamp) */
    volatile int64_t prof_start_us;        /* timestamp for MIPS calculation */

    char *pending_state;
    int pending_state_valid;

    /* WAV capture */
    FILE *capture_emu_file;
    FILE *capture_render_file;
    int capture_frames_written;
    int capture_max_frames;
    volatile int capture_armed;
};

/* =====================================================================
 * Ring buffer helpers
 * ===================================================================== */

static int ring_available(virus_instance_t *inst) {
    int avail = inst->ring_write - inst->ring_read;
    if (avail < 0) avail += AUDIO_RING_SIZE;
    return avail;
}

static int ring_free(virus_instance_t *inst) {
    return AUDIO_RING_SIZE - 1 - ring_available(inst);
}

/* =====================================================================
 * MIDI helpers (direct to Microcontroller, no Plugin wrapper)
 * ===================================================================== */

static void send_midi_to_mc(virus_instance_t *inst, const uint8_t *msg, int len) {
    if (!inst->mc || len < 1) return;
    synthLib::SMidiEvent ev(synthLib::MidiEventSource::Host,
                            msg[0],
                            len > 1 ? msg[1] : 0,
                            len > 2 ? msg[2] : 0);
    inst->mc->sendMIDI(ev);
}

static void send_cc(virus_instance_t *inst, int cc, int value) {
    uint8_t msg[3] = { 0xB0, (uint8_t)cc, (uint8_t)value };
    send_midi_to_mc(inst, msg, 3);
    inst->cc_values[cc] = value;
}

/* =====================================================================
 * ROM loading
 * ===================================================================== */

static bool find_and_load_rom(virus_instance_t *inst) {
    char roms_dir[512];
    snprintf(roms_dir, sizeof(roms_dir), "%s/roms", inst->module_dir);
    fprintf(stderr, "Virus: searching for ROMs in %s\n", roms_dir);

    /* Use gearmulator's ROMLoader which handles both .bin and .mid files,
     * does MIDI-to-ROM conversion, and auto-detects Model A/B/C. */
    auto roms = virusLib::ROMLoader::findROMs(std::string(roms_dir));
    if (roms.empty()) {
        snprintf(inst->load_error, sizeof(inst->load_error),
                 "No valid ROM found. Place a Virus ROM (.bin or .mid) in roms/");
        return false;
    }

    /* Use the first valid ROM found */
    inst->rom = new virusLib::ROMFile(std::move(roms.front()));
    fprintf(stderr, "Virus: loaded ROM %s (model: %s, rate: %u)\n",
            inst->rom->getFilename().c_str(),
            inst->rom->getModelName().c_str(),
            inst->rom->getSamplerate());
    return true;
}

/* =====================================================================
 * Preset enumeration
 * ===================================================================== */

static void update_bank_name(virus_instance_t *inst) {
    snprintf(inst->bank_name, sizeof(inst->bank_name), "Bank %c", 'A' + inst->current_bank);
}

static void update_preset_name(virus_instance_t *inst) {
    if (!inst->rom) { snprintf(inst->preset_name, sizeof(inst->preset_name), "No ROM"); return; }
    virusLib::ROMFile::TPreset pd;
    if (inst->rom->getSingle(inst->current_bank, inst->current_preset, pd))
        snprintf(inst->preset_name, sizeof(inst->preset_name), "%s",
                 virusLib::ROMFile::getSingleName(pd).c_str());
    else
        snprintf(inst->preset_name, sizeof(inst->preset_name), "---");
}

/* =====================================================================
 * Emulation thread (semaphore-driven, no thread contention)
 * ===================================================================== */

/* Resample ratio: 46875 Hz source → 44100 Hz output */
static constexpr double RESAMPLE_RATIO = 46875.0 / 44100.0; /* ~1.06293 src frames per dst frame */
/* Max output frames per EMU_CHUNK input: ceil(64 / 1.06293) + 1 = ~62 */
#define RESAMPLE_MAX_OUT (EMU_CHUNK + 4)

static void* emu_thread_func(void *arg) {
    virus_instance_t *inst = (virus_instance_t*)arg;

    /* Set real-time priority for emu thread */
    struct sched_param sp;
    sp.sched_priority = 40;  /* Below Move's audio thread but above normal */
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0)
        fprintf(stderr, "Virus: emu thread RT priority set (SCHED_FIFO, pri=40)\n");
    else
        fprintf(stderr, "Virus: emu thread RT priority failed (running as normal)\n");

    fprintf(stderr, "Virus: emu thread started (semaphore mode, chunk=%d, resample %.0f->%d)\n",
            EMU_CHUNK, DEVICE_RATE, MOVE_SAMPLE_RATE);

    inst->prof_start_us = now_us();

    float proc_l[EMU_CHUNK];
    float proc_r[EMU_CHUNK];
    auto& audioOutputs = inst->dsp->getAudio().getAudioOutputs();

    while (inst->emu_thread_running) {
        /* Check our ring buffer space first */
        if (ring_free(inst) < RESAMPLE_MAX_OUT) {
            usleep(200);
            continue;
        }

        /* Wait for DSP callback to signal that enough frames are ready.
         * This matches the console app pattern — when the semaphore fires,
         * the ring buffer already has EMU_CHUNK-4 frames, so processAudio's
         * internal waitNotEmpty() returns immediately with zero contention.
         * This eliminates the ~50-73 MIPS of overhead from manual polling. */
        inst->audio_sem->wait();
        if (!inst->emu_thread_running) goto done;

        synthLib::TAudioInputs inputs = {};
        synthLib::TAudioOutputs outputs = {};
        outputs[0] = proc_l;
        outputs[1] = proc_r;

        int64_t t_proc_start = now_us();
        inst->dsp->processAudio(inputs, outputs, EMU_CHUNK, EMU_CHUNK);
        int64_t t_proc_end = now_us();
        int64_t proc_us = t_proc_end - t_proc_start;
        inst->prof_process_us_total += proc_us;
        if ((int)proc_us > inst->prof_process_max_us) inst->prof_process_max_us = (int)proc_us;
        inst->emu_blocks++;

        /* Track peak level (pre-clamp) for distortion diagnosis */
        for (int i = 0; i < EMU_CHUNK; i++) {
            float al = fabsf(proc_l[i]);
            float ar = fabsf(proc_r[i]);
            float peak = al > ar ? al : ar;
            if (peak > inst->prof_peak_level) inst->prof_peak_level = peak;
        }
        /* Log stats every ~2 seconds (2 * 46875 / 64 ≈ 1465 blocks) */
        if ((inst->emu_blocks % 1465) == 0) {
            /* Calculate MIPS: blocks * 64 frames * (instructions per frame at 108 MHz)
             * At 46875 Hz, each frame = 108M/46875 = 2304 instructions.
             * MIPS = (emu_blocks * 64 * 2304) / elapsed_us */
            int64_t now = now_us();
            double elapsed_s = (now - inst->prof_start_us) / 1000000.0;
            double mips = 0;
            if (elapsed_s > 0.1) {
                double total_instructions = (double)inst->emu_blocks * EMU_CHUNK * 2304.0;
                mips = total_instructions / elapsed_s / 1000000.0;
            }
            fprintf(stderr, "Virus: peak=%.3f buf=%d ur=%d blk=%d mips=%.1f\n",
                    (double)inst->prof_peak_level, ring_available(inst),
                    inst->underrun_count, inst->emu_blocks, mips);
            inst->prof_peak_level = 0.0f; /* reset for next interval */
        }

        /* Resample 46875→44100 using linear interpolation.
         * resample_phase tracks our fractional position in the source block.
         * We use the previous block's last sample for cross-boundary interpolation. */
        float ext_l[EMU_CHUNK + 1], ext_r[EMU_CHUNK + 1];
        ext_l[0] = inst->resample_prev_l;
        ext_r[0] = inst->resample_prev_r;
        memcpy(ext_l + 1, proc_l, EMU_CHUNK * sizeof(float));
        memcpy(ext_r + 1, proc_r, EMU_CHUNK * sizeof(float));

        int16_t resampled[RESAMPLE_MAX_OUT * 2];
        int out_count = 0;

        while (inst->resample_phase < (double)EMU_CHUNK && out_count < RESAMPLE_MAX_OUT) {
            /* ext buffer is offset by 1: ext[0]=prev, ext[1]=proc[0], etc. */
            double ext_pos = inst->resample_phase + 1.0;
            int idx = (int)ext_pos;
            double frac = ext_pos - idx;
            if (idx >= EMU_CHUNK) break;

            float l = ext_l[idx] * (float)(1.0 - frac) + ext_l[idx + 1] * (float)frac;
            float r = ext_r[idx] * (float)(1.0 - frac) + ext_r[idx + 1] * (float)frac;

            int32_t li = (int32_t)(l * OUTPUT_GAIN * 32767.0f);
            int32_t ri = (int32_t)(r * OUTPUT_GAIN * 32767.0f);
            if (li > 32767) li = 32767; if (li < -32768) li = -32768;
            if (ri > 32767) ri = 32767; if (ri < -32768) ri = -32768;
            resampled[out_count * 2 + 0] = (int16_t)li;
            resampled[out_count * 2 + 1] = (int16_t)ri;
            out_count++;
            inst->resample_phase += RESAMPLE_RATIO;
        }
        inst->resample_phase -= (double)EMU_CHUNK;
        inst->resample_prev_l = proc_l[EMU_CHUNK - 1];
        inst->resample_prev_r = proc_r[EMU_CHUNK - 1];

        /* Write resampled frames to ring buffer */
        if (ring_free(inst) < out_count) continue; /* drop if full */

        int wr = inst->ring_write;
        for (int i = 0; i < out_count; i++) {
            inst->audio_ring[wr * 2 + 0] = resampled[i * 2 + 0];
            inst->audio_ring[wr * 2 + 1] = resampled[i * 2 + 1];
            wr = (wr + 1) % AUDIO_RING_SIZE;
        }
        inst->ring_write = wr;

        /* WAV capture */
        if (inst->capture_emu_file && inst->capture_frames_written < inst->capture_max_frames) {
            int to_cap = out_count;
            if (inst->capture_frames_written + to_cap > inst->capture_max_frames)
                to_cap = inst->capture_max_frames - inst->capture_frames_written;
            fwrite(resampled, sizeof(int16_t) * 2, to_cap, inst->capture_emu_file);
        }
    }

done:
    fprintf(stderr, "Virus: emu thread stopped (%d blocks)\n", inst->emu_blocks);
    return nullptr;
}

/* =====================================================================
 * Boot thread
 * ===================================================================== */

static void* boot_thread_func(void *arg) {
    virus_instance_t *inst = (virus_instance_t*)arg;
    fprintf(stderr, "Virus: boot thread started\n");

    /* 1. Load ROM */
    snprintf(inst->loading_status, sizeof(inst->loading_status), "Loading ROM...");
    if (!find_and_load_rom(inst)) {
        inst->initialized = 1; inst->loading_complete = 1;
        inst->boot_thread_running = 0;
        return nullptr;
    }

    /* 2. Create DspSingle via Device's static factory */
    snprintf(inst->loading_status, sizeof(inst->loading_status), "Creating DSP...");
    fprintf(stderr, "Virus: creating DspSingle (JIT)...\n");

    virusLib::DspSingle *dsp1 = nullptr;
    virusLib::DspSingle *dsp2 = nullptr;

    try {
        virusLib::Device::createDspInstances(dsp1, dsp2, *inst->rom, DEVICE_RATE);
    } catch (const std::exception& e) {
        fprintf(stderr, "Virus: DSP creation failed: %s\n", e.what());
        snprintf(inst->load_error, sizeof(inst->load_error), "DSP creation failed: %s", e.what());
        inst->initialized = 1; inst->loading_complete = 1;
        inst->boot_thread_running = 0;
        return nullptr;
    }

    inst->dsp = dsp1;

    /* 3. Create Microcontroller */
    fprintf(stderr, "Virus: creating Microcontroller...\n");
    inst->mc = new virusLib::Microcontroller(*inst->dsp, *inst->rom, false);

    /* 4. Set up semaphore and audio callback.
     * This callback runs on the DSP thread context for every output frame.
     * It replaces Device::onAudioWritten() with additional semaphore logic. */
    inst->audio_sem = new dsp56k::SpscSemaphore(1);
    inst->notify_timeout.store(0);
    inst->callback_count.store(0);

    auto& audio = inst->dsp->getAudio();

    audio.setCallback([inst](dsp56k::Audio* a) {
        /* Advance MIDI timing — critical for event dispatch.
         * Without this, MIDI events (init commands, preset data, notes)
         * queued via sendMIDI are never dispatched to the DSP. */
        inst->mc->getMidiQueue(0).onAudioWritten();

        /* Periodically process microcontroller and read MIDI output.
         * Every 4th frame matches the console app pattern. */
        uint32_t count = inst->callback_count.fetch_add(1) + 1;
        if ((count & 0x3) == 0) {
            std::vector<synthLib::SMidiEvent> midiOut;
            inst->mc->readMidiOut(midiOut);
            inst->mc->process();
        }

        /* Signal semaphore when enough frames are available.
         * This is the key optimization: the emu thread waits on this
         * semaphore instead of having processAudio block in waitNotEmpty.
         * When the semaphore fires, data is guaranteed ready, so
         * waitNotEmpty returns immediately with no mutex contention. */
        const auto avail = a->getAudioOutputs().size();
        int32_t timeout = inst->notify_timeout.load();
        timeout--;
        if (timeout <= 0 && avail >= (EMU_CHUNK - 4)) {
            timeout = (EMU_CHUNK - 4);
            inst->audio_sem->notify();
        }
        inst->notify_timeout.store(timeout);
    }, 0);

    /* 5. Boot DSPs (JIT compilation + start DSP thread) */
    snprintf(inst->loading_status, sizeof(inst->loading_status), "Booting DSP...");
    fprintf(stderr, "Virus: booting DSPs...\n");

    virusLib::Device::bootDSPs(inst->dsp, dsp2, *inst->rom, false);
    fprintf(stderr, "Virus: DSP boot returned\n");

    /* 6. Wait for DSP to finish booting.
     * Use small 8-frame chunks (same as Device constructor's dummyProcess)
     * and call processAudio directly — no semaphore during boot.
     * The DSP thread is starting up and producing frames slowly
     * during JIT compilation. */
    {
        constexpr int BOOT_CHUNK = 8;
        float dummy_l[BOOT_CHUNK], dummy_r[BOOT_CHUNK];
        synthLib::TAudioInputs inputs = {};
        synthLib::TAudioOutputs outputs = {};
        outputs[0] = dummy_l; outputs[1] = dummy_r;

        int retries = 0;
        fprintf(stderr, "Virus: waiting for DSP boot (model=%d)...\n",
                (int)inst->rom->getModel());

        if (inst->rom->getModel() == virusLib::DeviceModel::A) {
            /* Model A never signals dspHasBooted — just process 32 chunks */
            for (int i = 0; i < 32; i++) {
                if (inst->shutting_down) goto cleanup_and_exit;
                inst->dsp->processAudio(inputs, outputs, BOOT_CHUNK, 0);
            }
            inst->dsp->disableESSI1();
            retries = 32;
        } else {
            /* Model B/C: wait for boot signal with timeout */
            while (!inst->mc->dspHasBooted() && retries < 512) {
                if (inst->shutting_down) {
                    fprintf(stderr, "Virus: abort during boot wait\n");
                    goto cleanup_and_exit;
                }
                inst->dsp->processAudio(inputs, outputs, BOOT_CHUNK, 0);
                retries++;
            }
        }
        fprintf(stderr, "Virus: DSP booted after %d drain cycles (%d frames)\n",
                retries, retries * BOOT_CHUNK);

        /* Run at full DSP clock. Model A needs ~66 MIPS (achievable),
         * Model B needs ~108 MIPS (marginal on Move hardware). */
        fprintf(stderr, "Virus: DSP clock at 100%% (model=%s)\n",
                inst->rom->getModelName().c_str());

        /* 7. Initialize (mirrors Device constructor post-boot sequence) */
        inst->mc->sendInitControlCommands(127);

        /* Process to let init commands settle */
        for (int i = 0; i < 8; i++) {
            if (inst->shutting_down) goto cleanup_and_exit;
            inst->dsp->processAudio(inputs, outputs, BOOT_CHUNK, 0);
        }

        inst->mc->createDefaultState();

        /* Process a few more blocks after default state */
        for (int i = 0; i < 8; i++) {
            if (inst->shutting_down) goto cleanup_and_exit;
            inst->dsp->processAudio(inputs, outputs, BOOT_CHUNK, 0);
        }
    }

    fprintf(stderr, "Virus: DSP initialized successfully\n");

    /* 8. Set up presets */
    inst->bank_count = virusLib::ROMFile::getRomBankCount(inst->rom->getModel());
    inst->preset_count = inst->rom->getPresetsPerBank();
    inst->current_bank = 0;
    inst->current_preset = 0;
    update_bank_name(inst);
    update_preset_name(inst);

    /* Send initial preset via MIDI */
    {
        synthLib::SMidiEvent bankSel(synthLib::MidiEventSource::Host, 0xB0, 0, 0);
        inst->mc->sendMIDI(bankSel);
        synthLib::SMidiEvent progChg(synthLib::MidiEventSource::Host, 0xC0, 0, 0);
        inst->mc->sendMIDI(progChg);
    }

    /* 9. Pre-fill ring buffer (direct processAudio, no semaphore) */
    snprintf(inst->loading_status, sizeof(inst->loading_status), "Warming up...");
    inst->ring_read = 0;
    inst->ring_write = 0;

    {
        float wl[EMU_CHUNK], wr[EMU_CHUNK];
        synthLib::TAudioInputs inputs = {};
        synthLib::TAudioOutputs outputs = {};
        outputs[0] = wl; outputs[1] = wr;

        for (int fill = 0; fill < 4 && ring_free(inst) >= EMU_CHUNK; fill++) {
            inst->dsp->processAudio(inputs, outputs, EMU_CHUNK, 0);
            int wr_pos = inst->ring_write;
            for (int i = 0; i < EMU_CHUNK; i++) {
                int32_t l = (int32_t)(wl[i] * 32767.0f);
                int32_t r = (int32_t)(wr[i] * 32767.0f);
                if (l > 32767) l = 32767; if (l < -32768) l = -32768;
                if (r > 32767) r = 32767; if (r < -32768) r = -32768;
                inst->audio_ring[wr_pos * 2 + 0] = (int16_t)l;
                inst->audio_ring[wr_pos * 2 + 1] = (int16_t)r;
                wr_pos = (wr_pos + 1) % AUDIO_RING_SIZE;
            }
            inst->ring_write = wr_pos;
        }
    }
    fprintf(stderr, "Virus: pre-filled %d frames\n", ring_available(inst));

    /* Drain semaphore signals accumulated during boot.
     * Replace with a fresh semaphore so the emu thread starts clean. */
    delete inst->audio_sem;
    inst->audio_sem = new dsp56k::SpscSemaphore(1);
    inst->notify_timeout.store(0);

    /* 10. Start emu thread */
    inst->emu_thread_running = 1;
    inst->initialized = 1;
    pthread_create(&inst->emu_thread, nullptr, emu_thread_func, inst);

    inst->loading_complete = 1;
    snprintf(inst->loading_status, sizeof(inst->loading_status),
             "Ready: %d banks, %d presets/bank", inst->bank_count, inst->preset_count);

    if (inst->pending_state_valid && inst->pending_state) {
        fprintf(stderr, "Virus: applying deferred state\n");
        inst->pending_state_valid = 0;
    }

    inst->capture_armed = 1;
    fprintf(stderr, "Virus: ready! (semaphore mode, no Device/Plugin wrapper)\n");
    inst->boot_thread_running = 0;
    return nullptr;

cleanup_and_exit:
    /* Shutdown requested during boot */
    audio.setCallback(nullptr, 0);
    inst->boot_thread_running = 0;
    return nullptr;
}

/* =====================================================================
 * Plugin API v2
 * ===================================================================== */

static void v2_set_param(void *instance, const char *key, const char *val);

static void* v2_create_instance(const char *module_dir, const char *json_defaults) {
    (void)json_defaults;
    virus_instance_t *inst = (virus_instance_t*)calloc(1, sizeof(virus_instance_t));
    if (!inst) return nullptr;
    strncpy(inst->module_dir, module_dir, sizeof(inst->module_dir) - 1);
    fprintf(stderr, "Virus: creating instance from %s\n", module_dir);
    snprintf(inst->loading_status, sizeof(inst->loading_status), "Initializing...");
    inst->cc_values[74] = 127;
    inst->cc_values[75] = 64;
    inst->cc_values[76] = 64;
    inst->cc_values[72] = 64;
    inst->boot_thread_running = 1;
    pthread_create(&inst->boot_thread, nullptr, boot_thread_func, inst);
    return inst;
}

static void v2_destroy_instance(void *instance) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst) return;
    fprintf(stderr, "Virus: destroying\n");

    /* Signal shutdown to boot thread */
    inst->shutting_down = 1;

    /* Stop emu thread */
    if (inst->emu_thread_running) {
        inst->emu_thread_running = 0;
        /* Unblock semaphore wait */
        if (inst->audio_sem) inst->audio_sem->notify();
        pthread_join(inst->emu_thread, nullptr);
    }

    /* Wait for boot thread */
    if (inst->boot_thread_running) {
        /* Unblock semaphore wait during boot */
        if (inst->audio_sem) inst->audio_sem->notify();
        pthread_join(inst->boot_thread, nullptr);
    }

    /* Clear callback before destroying DSP */
    if (inst->dsp) {
        inst->dsp->getAudio().setCallback(nullptr, 0);
        inst->dsp->getAudio().terminate();
    }

    /* Destroy in reverse order */
    if (inst->mc) { delete inst->mc; inst->mc = nullptr; }
    if (inst->dsp) { delete inst->dsp; inst->dsp = nullptr; }
    if (inst->rom) { delete inst->rom; inst->rom = nullptr; }
    if (inst->audio_sem) { delete inst->audio_sem; inst->audio_sem = nullptr; }

    if (inst->capture_emu_file) { wav_close(inst->capture_emu_file, inst->capture_frames_written); }
    if (inst->capture_render_file) { wav_close(inst->capture_render_file, inst->capture_frames_written); }
    if (inst->pending_state) free(inst->pending_state);
    free(inst);
    fprintf(stderr, "Virus: destroyed\n");
}

static void v2_on_midi(void *instance, const uint8_t *msg, int len, int source) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst || !inst->initialized || !inst->mc || len < 1) return;
    (void)source;
    uint8_t status = msg[0] & 0xF0;
    uint8_t modified[8];
    int n = len > 8 ? 8 : len;
    memcpy(modified, msg, n);

    if (status == 0x90 && len >= 3 && msg[2] > 0 && inst->capture_armed) {
        inst->capture_armed = 0;
        char ep[512], rp[512];
        snprintf(ep, sizeof(ep), "%s/capture_emu.wav", inst->module_dir);
        snprintf(rp, sizeof(rp), "%s/capture_render.wav", inst->module_dir);
        inst->capture_emu_file = wav_open(ep);
        inst->capture_render_file = wav_open(rp);
        inst->capture_frames_written = 0;
        inst->capture_max_frames = CAPTURE_MAX_FRAMES;
        fprintf(stderr, "Virus: capture started\n");
    }

    if ((status == 0x90 || status == 0x80) && len >= 2) {
        int note = msg[1] + inst->octave_transpose * 12;
        if (note < 0) note = 0; if (note > 127) note = 127;
        modified[1] = (uint8_t)note;
    }
    if (status == 0xB0 && len >= 3)
        inst->cc_values[msg[1] & 0x7F] = msg[2] & 0x7F;

    send_midi_to_mc(inst, modified, n);
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
    if (!inst) return;

    if (strcmp(key, "state") == 0) {
        if (!inst->loading_complete) {
            if (inst->pending_state) free(inst->pending_state);
            inst->pending_state = strdup(val);
            inst->pending_state_valid = 1;
            return;
        }
        int ival;
        if (json_get_int(val, "bank", &ival) == 0 && ival >= 0 && ival < inst->bank_count) {
            inst->current_bank = ival; update_bank_name(inst);
        }
        if (json_get_int(val, "preset", &ival) == 0 && ival >= 0 && ival < inst->preset_count) {
            inst->current_preset = ival;
            send_cc(inst, 0, inst->current_bank);
            uint8_t pc[2] = { 0xC0, (uint8_t)inst->current_preset };
            send_midi_to_mc(inst, pc, 2);
            update_preset_name(inst);
        }
        if (json_get_int(val, "octave_transpose", &ival) == 0) {
            inst->octave_transpose = ival;
            if (inst->octave_transpose < -4) inst->octave_transpose = -4;
            if (inst->octave_transpose > 4) inst->octave_transpose = 4;
        }
        for (int i = 0; i < NUM_PARAMS; i++) {
            if (json_get_int(val, g_params[i].key, &ival) == 0) {
                if (ival < g_params[i].min_val) ival = g_params[i].min_val;
                if (ival > g_params[i].max_val) ival = g_params[i].max_val;
                send_cc(inst, g_params[i].cc, ival);
            }
        }
        return;
    }
    if (strcmp(key, "preset") == 0) {
        int idx = atoi(val);
        if (idx >= 0 && idx < inst->preset_count && inst->mc) {
            inst->current_preset = idx;
            send_cc(inst, 0, inst->current_bank);
            uint8_t pc[2] = { 0xC0, (uint8_t)inst->current_preset };
            send_midi_to_mc(inst, pc, 2);
            update_preset_name(inst);
        }
        return;
    }
    if (strcmp(key, "bank_index") == 0) {
        int idx = atoi(val);
        if (idx >= 0 && idx < inst->bank_count) {
            inst->current_bank = idx; update_bank_name(inst);
            inst->current_preset = 0;
            send_cc(inst, 0, inst->current_bank);
            uint8_t pc[2] = { 0xC0, 0 };
            send_midi_to_mc(inst, pc, 2);
            update_preset_name(inst);
        }
        return;
    }
    if (strcmp(key, "octave_transpose") == 0) {
        inst->octave_transpose = atoi(val);
        if (inst->octave_transpose < -4) inst->octave_transpose = -4;
        if (inst->octave_transpose > 4) inst->octave_transpose = 4;
        return;
    }
    if (strcmp(key, "all_notes_off") == 0) {
        uint8_t msg[3] = { 0xB0, 123, 0 };
        send_midi_to_mc(inst, msg, 3);
        return;
    }
    if (strcmp(key, "capture") == 0) {
        if (!inst->loading_complete || !inst->emu_thread_running) return;
        if (inst->capture_emu_file || inst->capture_render_file) return;
        char ep[512], rp[512];
        snprintf(ep, sizeof(ep), "%s/capture_emu.wav", inst->module_dir);
        snprintf(rp, sizeof(rp), "%s/capture_render.wav", inst->module_dir);
        inst->capture_emu_file = wav_open(ep);
        inst->capture_render_file = wav_open(rp);
        inst->capture_frames_written = 0;
        inst->capture_max_frames = CAPTURE_MAX_FRAMES;
        fprintf(stderr, "Virus: capture started\n");
        return;
    }
    for (int i = 0; i < NUM_PARAMS; i++) {
        if (strcmp(key, g_params[i].key) == 0) {
            int ival = atoi(val);
            if (ival < g_params[i].min_val) ival = g_params[i].min_val;
            if (ival > g_params[i].max_val) ival = g_params[i].max_val;
            send_cc(inst, g_params[i].cc, ival);
            return;
        }
    }
}

static int v2_get_param(void *instance, const char *key, char *buf, int buf_len) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst) return -1;

    if (strcmp(key, "preset") == 0) return snprintf(buf, buf_len, "%d", inst->current_preset);
    if (strcmp(key, "preset_count") == 0) return snprintf(buf, buf_len, "%d", inst->preset_count);
    if (strcmp(key, "preset_name") == 0) return snprintf(buf, buf_len, "%s", inst->preset_name);
    if (strcmp(key, "name") == 0) return snprintf(buf, buf_len, "Virus");
    if (strcmp(key, "bank_index") == 0) return snprintf(buf, buf_len, "%d", inst->current_bank);
    if (strcmp(key, "bank_count") == 0) return snprintf(buf, buf_len, "%d", inst->bank_count);
    if (strcmp(key, "bank_name") == 0) return snprintf(buf, buf_len, "%s", inst->bank_name);
    if (strcmp(key, "patch_in_bank") == 0) return snprintf(buf, buf_len, "%d", inst->current_preset + 1);
    if (strcmp(key, "octave_transpose") == 0) return snprintf(buf, buf_len, "%d", inst->octave_transpose);
    if (strcmp(key, "loading_status") == 0) return snprintf(buf, buf_len, "%s", inst->loading_status);
    if (strcmp(key, "debug_info") == 0) {
        int avail = ring_available(inst);
        int blocks = inst->emu_blocks > 0 ? inst->emu_blocks : 1;
        return snprintf(buf, buf_len,
            "buf=%d min=%d ur=%d blk=%d "
            "proc_avg=%lld proc_max=%d peak=%.2f",
            avail, inst->prof_ring_min, inst->underrun_count, inst->emu_blocks,
            (long long)(inst->prof_process_us_total / blocks),
            inst->prof_process_max_us,
            (double)inst->prof_peak_level);
    }
    if (strcmp(key, "prof_reset") == 0) {
        inst->prof_poll_us_total = 0;
        inst->prof_process_us_total = 0;
        inst->prof_timeout_count = 0;
        inst->prof_ring_min = AUDIO_RING_SIZE;
        inst->prof_poll_max_us = 0;
        inst->prof_process_max_us = 0;
        inst->prof_peak_level = 0.0f;
        inst->underrun_count = 0;
        inst->emu_blocks = 0;
        return snprintf(buf, buf_len, "reset");
    }
    for (int i = 0; i < NUM_PARAMS; i++)
        if (strcmp(key, g_params[i].key) == 0)
            return snprintf(buf, buf_len, "%d", inst->cc_values[g_params[i].cc]);

    if (strcmp(key, "state") == 0) {
        int off = 0;
        off += snprintf(buf+off, buf_len-off, "{\"bank\":%d,\"preset\":%d,\"octave_transpose\":%d",
            inst->current_bank, inst->current_preset, inst->octave_transpose);
        for (int i = 0; i < NUM_PARAMS; i++)
            off += snprintf(buf+off, buf_len-off, ",\"%s\":%d", g_params[i].key, inst->cc_values[g_params[i].cc]);
        off += snprintf(buf+off, buf_len-off, "}");
        return off;
    }
    if (strcmp(key, "ui_hierarchy") == 0) {
        const char *h = "{\"modes\":null,\"levels\":{\"root\":{\"list_param\":\"preset\",\"count_param\":\"preset_count\",\"name_param\":\"preset_name\",\"children\":null,\"knobs\":[\"cutoff\",\"resonance\",\"filter_env\",\"attack\",\"decay\",\"sustain\",\"release\",\"octave_transpose\"],\"params\":[{\"level\":\"filter\",\"label\":\"Filter\"},{\"level\":\"osc\",\"label\":\"Oscillators\"},{\"level\":\"fx\",\"label\":\"Effects\"}]},\"filter\":{\"children\":null,\"knobs\":[\"cutoff\",\"resonance\",\"filter_env\",\"filter_type\"],\"params\":[\"cutoff\",\"resonance\",\"filter_env\",\"filter_type\"]},\"osc\":{\"children\":null,\"knobs\":[\"osc1_shape\",\"osc2_shape\",\"osc_balance\"],\"params\":[\"osc1_shape\",\"osc2_shape\",\"osc_balance\"]},\"fx\":{\"children\":null,\"knobs\":[\"chorus_mix\",\"delay_time\",\"delay_fb\",\"reverb_mix\"],\"params\":[\"chorus_mix\",\"delay_time\",\"delay_fb\",\"reverb_mix\"]}}}";
        int len = strlen(h);
        if (len < buf_len) { strcpy(buf, h); return len; }
        return -1;
    }
    if (strcmp(key, "chain_params") == 0) {
        int off = 0;
        off += snprintf(buf+off, buf_len-off,
            "[{\"key\":\"preset\",\"name\":\"Preset\",\"type\":\"int\",\"min\":0,\"max\":127},"
            "{\"key\":\"octave_transpose\",\"name\":\"Octave\",\"type\":\"int\",\"min\":-4,\"max\":4}");
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
    if (!inst || inst->load_error[0] == '\0') return 0;
    return snprintf(buf, buf_len, "%s", inst->load_error);
}

static void v2_render_block(void *instance, int16_t *out, int frames) {
    virus_instance_t *inst = (virus_instance_t*)instance;
    if (!inst || !inst->loading_complete || !inst->emu_thread_running) {
        memset(out, 0, frames * 2 * sizeof(int16_t));
        return;
    }

    int avail = ring_available(inst);
    if (inst->prof_ring_min == 0 || avail < inst->prof_ring_min)
        inst->prof_ring_min = avail;
    int to_read = (avail < frames) ? avail : frames;
    int rd = inst->ring_read;
    for (int i = 0; i < to_read; i++) {
        out[i*2+0] = inst->audio_ring[rd*2+0];
        out[i*2+1] = inst->audio_ring[rd*2+1];
        rd = (rd + 1) % AUDIO_RING_SIZE;
    }
    inst->ring_read = rd;

    if (to_read < frames) {
        inst->underrun_count++;
        memset(out + to_read * 2, 0, (frames - to_read) * 2 * sizeof(int16_t));
    }
    inst->render_count++;

    if (inst->capture_render_file && inst->capture_frames_written < inst->capture_max_frames) {
        int to_cap = frames;
        if (inst->capture_frames_written + to_cap > inst->capture_max_frames)
            to_cap = inst->capture_max_frames - inst->capture_frames_written;
        fwrite(out, sizeof(int16_t) * 2, to_cap, inst->capture_render_file);
        inst->capture_frames_written += to_cap;
        if (inst->capture_frames_written >= inst->capture_max_frames) {
            fprintf(stderr, "Virus: capture done (%d frames)\n", inst->capture_frames_written);
            wav_close(inst->capture_emu_file, inst->capture_frames_written);
            wav_close(inst->capture_render_file, inst->capture_frames_written);
            inst->capture_emu_file = nullptr;
            inst->capture_render_file = nullptr;
            inst->capture_max_frames = 0;
        }
    }
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
