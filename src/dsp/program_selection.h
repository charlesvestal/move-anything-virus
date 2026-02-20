#pragma once

#include <cstdint>

enum ProgramSelectionChange {
    PROGRAM_SELECTION_NONE = 0,
    PROGRAM_SELECTION_BANK_CHANGED = 1 << 0,
    PROGRAM_SELECTION_PRESET_CHANGED = 1 << 1,
};

int apply_program_selection_midi(const uint8_t *msg,
                                 int len,
                                 int bank_count,
                                 int preset_count,
                                 int *current_bank,
                                 int *current_preset);
