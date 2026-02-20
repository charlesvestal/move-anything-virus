#include "program_selection.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

static void expect(bool cond, const char *msg) {
    if (cond) return;
    std::fprintf(stderr, "program_selection_test FAILED: %s\n", msg);
    std::abort();
}

int main() {
    int bank = 0;
    int preset = 0;

    {
        const uint8_t msg[2] = {0xC0, 5};
        const int changed = apply_program_selection_midi(msg, 2, 8, 128, &bank, &preset);
        expect((changed & PROGRAM_SELECTION_PRESET_CHANGED) != 0, "program change must flag preset update");
        expect(preset == 5, "program change must update preset index");
        expect(bank == 0, "program change must not change bank");
    }

    {
        const uint8_t msg[3] = {0xB0, 0, 2};
        const int changed = apply_program_selection_midi(msg, 3, 8, 128, &bank, &preset);
        expect((changed & PROGRAM_SELECTION_BANK_CHANGED) != 0, "bank select must flag bank update");
        expect(bank == 2, "bank select must update bank index");
        expect(preset == 5, "bank select must not change preset");
    }

    {
        const uint8_t msg[3] = {0xB0, 0, 127};
        const int changed = apply_program_selection_midi(msg, 3, 3, 128, &bank, &preset);
        expect((changed & PROGRAM_SELECTION_BANK_CHANGED) == 0, "already-clamped bank should not report change");
        expect(bank == 2, "bank select must clamp to bank_count - 1");
    }

    {
        const uint8_t msg[2] = {0xC0, 127};
        const int changed = apply_program_selection_midi(msg, 2, 8, 64, &bank, &preset);
        expect((changed & PROGRAM_SELECTION_PRESET_CHANGED) != 0, "program change must report preset update");
        expect(preset == 63, "program change must clamp to preset_count - 1");
    }

    {
        const uint8_t msg[3] = {0xB0, 7, 99};
        const int before_bank = bank;
        const int before_preset = preset;
        const int changed = apply_program_selection_midi(msg, 3, 8, 128, &bank, &preset);
        expect(changed == PROGRAM_SELECTION_NONE, "non bank-select CC must be ignored");
        expect(bank == before_bank, "ignored message must not alter bank");
        expect(preset == before_preset, "ignored message must not alter preset");
    }

    {
        const uint8_t msg[3] = {0xB0, 32, 1};
        const int changed = apply_program_selection_midi(msg, 3, 8, 128, &bank, &preset);
        expect((changed & PROGRAM_SELECTION_BANK_CHANGED) != 0, "bank select LSB must update bank");
        expect(bank == 0, "bank select LSB value 1 must map to internal bank A (index 0)");
    }

    {
        const uint8_t msg[3] = {0xB0, 32, 0};
        const int changed = apply_program_selection_midi(msg, 3, 8, 128, &bank, &preset);
        expect((changed & PROGRAM_SELECTION_BANK_CHANGED) == 0, "legacy bank value 0 should map to current bank A");
        expect(bank == 0, "legacy bank value 0 must remain bank A (index 0)");
    }

    std::puts("program_selection_test PASS");
    return 0;
}
