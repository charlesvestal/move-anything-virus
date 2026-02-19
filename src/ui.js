/*
 * Virus UI for Move Anything
 *
 * Uses shared sound generator UI base for consistent preset browsing.
 * Parameter editing via shadow UI hierarchy when in chain context.
 */

/* Shared utilities - absolute path for module location independence */
import { createSoundGeneratorUI } from '/data/UserData/move-anything/shared/sound_generator_ui.mjs';

/* Create the UI with Virus-specific customizations */
const ui = createSoundGeneratorUI({
    moduleName: 'Virus',

    onInit: (state) => {
        /* Virus-specific initialization */
    },

    onTick: (state) => {
        /* Virus-specific per-tick updates */
    },

    onPresetChange: (preset) => {
        /* Send all notes off on preset change */
        host_module_set_param('all_notes_off', '1');
    },

    showPolyphony: false,
    showOctave: true,
});

/* Export required callbacks */
globalThis.init = ui.init;
globalThis.tick = ui.tick;
globalThis.onMidiMessageInternal = ui.onMidiMessageInternal;
globalThis.onMidiMessageExternal = ui.onMidiMessageExternal;
