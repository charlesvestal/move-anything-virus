# Move Everything: Osirus

Access Virus synthesizer emulation for [Move Everything](https://github.com/charlesvestal/move-anything) on Ableton Move hardware. Named "Osirus" after [Gearmulator's Virus emulation](https://github.com/dsp56300/gearmulator).

Uses the [Gearmulator](https://github.com/dsp56300/gearmulator) DSP56300 JIT emulator to run the original Virus firmware.

## Requirements

- Move Everything host installed on your Ableton Move
- A Virus A ROM file (`.mid` format) — not included

## Installation

Install via the Module Store on your Move, or manually:

```bash
./scripts/build.sh
./scripts/install.sh
```

Then place your Virus ROM file in the module's `roms/` directory on the device:
```
/data/UserData/move-anything/modules/sound_generators/osirus/roms/
```

## Supported ROMs

- **Virus A** (`.mid` boot ROM format) — recommended, tested on Move hardware

Virus B/C ROMs are not supported on Move. The Virus B/C DSP models require more processing power than the Move's Cortex-A72 can provide in real time.

## Architecture

The DSP emulator runs in a **forked child process** to avoid sharing kernel resources (mmap_lock, heap allocator) with MoveOriginal. Communication between the plugin API and the DSP child uses shared memory:

- **Audio**: Lock-free ring buffer (8192 frames, target fill ~384 frames / 9ms)
- **MIDI**: FIFO queue in shared memory
- **Control**: Atomic flags for status, preset info, and profiling

The DSP56300 JIT compiler translates Motorola DSP56300 instructions to ARM64 (aarch64) machine code at runtime using [asmjit](https://github.com/asmjit/asmjit).

## Signal Chain Integration

Works as a sound generator in Move Everything's Signal Chain. Exposed parameters:

- Filter: Cutoff, Resonance, Filter Env Amount, Filter Mode
- Filter Envelope: Attack, Decay, Sustain, Release
- Amp Envelope: Attack, Decay, Sustain, Release
- Oscillators: Osc1 Shape, Osc2 Shape, Osc Balance, Volume

Preset browsing via jog wheel (8 banks x 128 presets for ABC models).

## Building

Requires Docker for cross-compilation:

```bash
./scripts/build.sh    # Build via Docker (Ubuntu 22.04 + aarch64 toolchain)
```

Output: `dist/osirus/dsp.so` (ARM64 shared library)

## Credits

- [dsp56300/gearmulator](https://github.com/dsp56300/gearmulator) — DSP56300 emulator and Virus synth library
- [asmjit](https://github.com/asmjit/asmjit) — JIT assembler for ARM64/x86-64

## License

GPL-3.0 (following gearmulator's license)
