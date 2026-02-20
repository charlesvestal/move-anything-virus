#include "program_selection.h"

namespace {

int clamp_index(int value, int count) {
    if (count <= 1) return 0;
    if (value < 0) return 0;
    if (value >= count) return count - 1;
    return value;
}

int midi_bank_to_index(int midi_value, int bank_count) {
    if (bank_count <= 1) return 0;
    /* Access Virus bank-select values are 1-based (A=1..H=8). */
    if (midi_value >= 1 && midi_value <= bank_count) return midi_value - 1;
    /* Legacy compatibility: accept already-0-based bank values too. */
    return clamp_index(midi_value, bank_count);
}

}

int apply_program_selection_midi(const uint8_t *msg,
                                 int len,
                                 int bank_count,
                                 int preset_count,
                                 int *current_bank,
                                 int *current_preset) {
    if (!msg || len < 1 || !current_bank || !current_preset) return PROGRAM_SELECTION_NONE;

    const uint8_t status = msg[0] & 0xF0;
    int change_mask = PROGRAM_SELECTION_NONE;

    if (status == 0xB0 && len >= 3 && (msg[1] == 0 || msg[1] == 32)) {
        const int next_bank = (msg[1] == 32)
                                  ? midi_bank_to_index((int)msg[2], bank_count)
                                  : clamp_index((int)msg[2], bank_count);
        if (next_bank != *current_bank) {
            *current_bank = next_bank;
            change_mask |= PROGRAM_SELECTION_BANK_CHANGED;
        }
        return change_mask;
    }

    if (status == 0xC0 && len >= 2) {
        const int next_preset = clamp_index((int)msg[1], preset_count);
        if (next_preset != *current_preset) {
            *current_preset = next_preset;
            change_mask |= PROGRAM_SELECTION_PRESET_CHANGED;
        }
        return change_mask;
    }

    return PROGRAM_SELECTION_NONE;
}
