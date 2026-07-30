#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "usb_audio_probe.h"

typedef struct {
    bool device_connected;
    bool midi_connected;
    bool lfo_page;
    uint16_t bpm;
    uint8_t bars;
    uar_loop_state_t loop_state;
    uint32_t loop_frames;
    uint32_t loop_position;
    bool record_armed;
    bool overdubbing;
    bool metronome;
    uint8_t selected_track;
    uint8_t track_volumes[4];
    uint32_t track_frames[4];
    uint32_t track_positions[4];
    uint16_t track_waveforms[4][UAR_WAVEFORM_BINS];
    uint8_t lfo_field;
    uint8_t lfo_shape;
    uint8_t lfo_rate;
    uint8_t lfo_relation;
    uint8_t lfo_depth;
    uint8_t lfo_center;
    bool lfo_enabled[2];
    uint8_t lfo_channel[2];
    uint8_t lfo_target[2];
    float lfo_phase[2];
} sidekick_ui_state_t;

void sidekick_lvgl_init(void *framebuffer_a, void *framebuffer_b,
                        uint16_t physical_width, uint16_t physical_height);
void sidekick_lvgl_update(const sidekick_ui_state_t *state, bool force);
void sidekick_lvgl_run(void);
