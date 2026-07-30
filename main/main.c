#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/input.h"
#include "bsp/power.h"
#include "esp_err.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"
#include "lvgl_ui.h"
#include "usb_audio_probe.h"

#define COLOR_BG 0xffc8c7c5
#define COLOR_PANEL 0xff111313
#define COLOR_TEXT 0xfff1f0ec
#define COLOR_DARK 0xff262728
#define COLOR_DIM 0xff696968
#define COLOR_ACCENT 0xffff4b16
#define COLOR_GOOD 0xffff762d
#define COLOR_BAD 0xffff3010
#define COLOR_SILVER 0xffe2e0dc
#define COLOR_SHADOW 0xff8e8d8b

static const char *TAG = "usb_recorder";
static pax_buf_t s_framebuffer;
static pax_buf_t s_framebuffers[2];
static unsigned s_framebuffer_index;
static size_t s_width;
static size_t s_height;
static size_t s_physical_width;
static size_t s_physical_height;
static void *s_display_pixels[2];
static QueueHandle_t s_input_queue;
static uint16_t s_display_levels[8];
static uint16_t s_peak_hold[8];
static uint8_t s_peak_hold_ticks[8];
static uint32_t s_audio_packets;
static uint32_t s_audio_errors;
static uint64_t s_audio_bytes;
static uint64_t s_nonzero_bytes;
static uar_loop_state_t s_loop_state;
static uint32_t s_loop_frames;
static uint32_t s_loop_position;
static uint16_t s_loop_bpm = 120;
static uint8_t s_loop_bars = 4;
static uint8_t s_selected_track;
static uint32_t s_track_frames[4];
static uint32_t s_track_positions[4];
static bool s_metronome;
static bool s_overdubbing;
static bool s_record_armed;
static uint8_t s_track_volumes[4] = {100, 100, 100, 100};
static uint16_t s_track_waveforms[4][UAR_WAVEFORM_BINS];
static uint32_t s_ui_draw_ms;
static uint32_t s_ui_present_ms;
static int16_t s_previous_main_cursor[2] = {-1, -1};
static int16_t s_previous_track_cursor[2][4] = {
    {-1, -1, -1, -1}, {-1, -1, -1, -1}
};
static uint16_t s_previous_beat[2] = {UINT16_MAX, UINT16_MAX};
typedef enum {
    UI_PAGE_LOOPER,
    UI_PAGE_LFO,
} ui_page_t;
static ui_page_t s_ui_page;
static bool s_ui_dirty;
typedef enum {
    LFO_FIELD_BIND_A,
    LFO_FIELD_BIND_B,
    LFO_FIELD_SHAPE,
    LFO_FIELD_RATE,
    LFO_FIELD_RELATION,
    LFO_FIELD_DEPTH,
    LFO_FIELD_CENTER,
    LFO_FIELD_COUNT,
} lfo_field_t;
typedef struct {
    bool enabled;
    uint8_t channel;
    uint8_t target;
    uint8_t shape;
    uint8_t rate;
    uint8_t depth;
    uint8_t center;
    float phase;
    int16_t last_value;
} midi_lfo_t;
#define LFO_TARGET_COUNT 3
#define LFO_BINDING_COUNT 7
#define LFO_SHAPE_COUNT 6
#define LFO_RATE_COUNT 13
#define LFO_RELATION_COUNT 8
static midi_lfo_t s_lfos[2] = {
    {.enabled = true, .channel = 0, .target = 1, .shape = 0, .rate = 9,
     .depth = 64, .center = 64, .last_value = -1},
    {.enabled = false, .channel = 1, .target = 1, .shape = 0, .rate = 9,
     .depth = 64, .center = 64, .last_value = -1}
};
static uint8_t s_lfo_relation;
static uint8_t s_lfo_field;
static int64_t s_lfo_previous_tick_us;
static int16_t s_lfo_previous_cursor[2][2] = {
    {-1, -1}, {-1, -1}
};
static const char *const s_lfo_target_names[LFO_TARGET_COUNT] = {
    "VOLUME", "FX Y · FORCE", "FX X · STICK"
};
static const uint8_t s_lfo_target_cc[LFO_TARGET_COUNT] = {
    7, 1, 0
};
static const char *const s_lfo_shape_names[LFO_SHAPE_COUNT] = {
    "SINE", "TRIANGLE", "SAW UP", "SAW DOWN", "SQUARE", "RANDOM"
};
static const char *const s_lfo_rate_names[LFO_RATE_COUNT] = {
    "0.10 Hz", "0.25 Hz", "0.50 Hz", "1.00 Hz", "2.00 Hz", "4.00 Hz", "8.00 Hz",
    "1/16", "1/8", "1/4", "1/2", "1 BAR", "2 BAR"
};
static const char *const s_lfo_relation_names[LFO_RELATION_COUNT] = {
    "SAME", "INVERTED", "B ×2", "B ×3", "B ×4", "B ÷2", "B ÷3", "B ÷4"
};
static const float s_lfo_relation_ratios[LFO_RELATION_COUNT] = {
    1.0f, 1.0f, 2.0f, 3.0f, 4.0f, 0.5f, 1.0f / 3.0f, 0.25f
};
static void text(float x, float y, float size, pax_col_t color, const char *value) {
    pax_draw_text(&s_framebuffer, color, pax_font_sky_mono, size, x, y, value);
}

static float lfo_frequency(uint8_t rate) {
    static const float free_rates[] = {0.10f, 0.25f, 0.50f, 1.0f, 2.0f, 4.0f, 8.0f};
    if (rate < 7) return free_rates[rate];
    float beats_per_second = s_loop_bpm / 60.0f;
    switch (rate) {
        case 7: return beats_per_second * 4.0f;
        case 8: return beats_per_second * 2.0f;
        case 9: return beats_per_second;
        case 10: return beats_per_second * 0.5f;
        case 11: return beats_per_second * 0.25f;
        default: return beats_per_second * 0.125f;
    }
}

static float lfo_shape_value(midi_lfo_t *lfo) {
    float phase = lfo->phase;
    switch (lfo->shape) {
        case 0: return sinf(phase * 6.2831853f);
        case 1: return 1.0f - 4.0f * fabsf(phase - 0.5f);
        case 2: return phase * 2.0f - 1.0f;
        case 3: return 1.0f - phase * 2.0f;
        case 4: return phase < 0.5f ? 1.0f : -1.0f;
        default:
            return ((int32_t)(esp_random() & 0xffff) - 32768) / 32768.0f;
    }
}

static float lfo_visual_value(uint8_t shape, float phase) {
    switch (shape) {
        case 0: return sinf(phase * 6.2831853f);
        case 1: return 1.0f - 4.0f * fabsf(phase - 0.5f);
        case 2: return phase * 2.0f - 1.0f;
        case 3: return 1.0f - phase * 2.0f;
        case 4: return phase < 0.5f ? 1.0f : -1.0f;
        default: {
            uint32_t step = (uint32_t)(phase * 16.0f);
            uint32_t hash = step * 1103515245u + 12345u;
            return ((hash >> 16) & 0x7fff) / 16383.5f - 1.0f;
        }
    }
}

static void draw_lfo_curve_column(pax_buf_t *target, uint8_t twin, int x) {
    const int graph_x = 42;
    const int graph_w = 716;
    const int graph_y = 148 + twin * 78;
    const int graph_h = 68;
    if (x < graph_x || x >= graph_x + graph_w) return;
    float phase = (float)(x - graph_x) / graph_w;
    float wave = lfo_visual_value(s_lfos[0].shape, phase);
    if (twin == 1 && s_lfo_relation == 1) wave = -wave;
    int y = graph_y + graph_h / 2 - (int)(wave * 26.0f);
    pax_draw_rect(target, COLOR_GOOD, x, y - 1, 1, 3);
}

static void render_lfo_fast(void) {
    pax_buf_t *target = &s_framebuffers[s_framebuffer_index ^ 1];
    unsigned displayed = s_framebuffer_index ^ 1;
    const int graph_x = 42;
    const int graph_w = 716;
    const int graph_h = 68;
    for (uint8_t twin = 0; twin < 2; twin++) {
        const int graph_y = 148 + twin * 78;
        int previous = s_lfo_previous_cursor[displayed][twin];
        if (previous >= graph_x) {
            pax_draw_rect(target, 0xff080909, previous - 6, graph_y, 13, graph_h);
            pax_draw_rect(target, COLOR_DIM, previous - 6,
                          graph_y + graph_h / 2, 13, 1);
            for (int x = previous - 6; x <= previous + 6; x++) {
                draw_lfo_curve_column(target, twin, x);
            }
        }
        midi_lfo_t *lfo = &s_lfos[twin];
        int cursor = graph_x + (int)(lfo->phase * (graph_w - 1));
        float wave = lfo_visual_value(s_lfos[0].shape, lfo->phase);
        if (twin == 1 && s_lfo_relation == 1) wave = -wave;
        int point_y = graph_y + graph_h / 2 - (int)(wave * 26.0f);
        pax_draw_rect(target, twin ? COLOR_TEXT : COLOR_ACCENT,
                      cursor, graph_y, 2, graph_h);
        pax_draw_circle(target, twin ? COLOR_ACCENT : COLOR_TEXT,
                        cursor + 1, point_y, 4);
        s_lfo_previous_cursor[displayed][twin] = cursor;
    }
}

static void lfo_tick(void) {
    int64_t now = esp_timer_get_time();
    if (s_lfo_previous_tick_us == 0) {
        s_lfo_previous_tick_us = now;
        return;
    }
    int64_t elapsed = now - s_lfo_previous_tick_us;
    if (elapsed < 20000) return;
    s_lfo_previous_tick_us = now;
    float dt = elapsed / 1000000.0f;
    if (dt > 0.1f) dt = 0.1f;
    float base_frequency = lfo_frequency(s_lfos[0].rate);
    for (uint8_t twin = 0; twin < 2; twin++) {
        midi_lfo_t *lfo = &s_lfos[twin];
        float ratio = twin ? s_lfo_relation_ratios[s_lfo_relation] : 1.0f;
        lfo->phase += base_frequency * ratio * dt;
        lfo->phase -= floorf(lfo->phase);
        if (!lfo->enabled) continue;
        float wave = lfo_shape_value(lfo);
        if (twin == 1 && s_lfo_relation == 1) wave = -wave;
        int value = (int)lroundf(lfo->center + wave * lfo->depth * 0.5f);
        if (value < 0) value = 0;
        if (value > 127) value = 127;
        if (value == lfo->last_value) continue;
        lfo->last_value = value;
        uint8_t channel = lfo->channel + 1;
        if (lfo->target == 2) {
            uar_midi_pitch_bend(channel, (int16_t)(value * 129 - 8192));
        } else {
            uar_midi_control_change(channel, s_lfo_target_cc[lfo->target], value);
        }
    }
}

static void lfo_adjust_binding(midi_lfo_t *lfo, int direction) {
    int binding = lfo->enabled ?
        1 + lfo->channel * LFO_TARGET_COUNT + lfo->target : 0;
    binding = (binding + LFO_BINDING_COUNT +
               (direction > 0 ? 1 : -1)) % LFO_BINDING_COUNT;
    lfo->enabled = binding != 0;
    if (lfo->enabled) {
        binding--;
        lfo->channel = binding / LFO_TARGET_COUNT;
        lfo->target = binding % LFO_TARGET_COUNT;
        lfo->last_value = -1;
    }
}

static void lfo_adjust_selected(int direction) {
    midi_lfo_t *lfo = &s_lfos[0];
    switch (s_lfo_field) {
        case LFO_FIELD_BIND_A:
            lfo_adjust_binding(&s_lfos[0], direction);
            break;
        case LFO_FIELD_BIND_B:
            lfo_adjust_binding(&s_lfos[1], direction);
            break;
        case LFO_FIELD_SHAPE:
            lfo->shape =
                (lfo->shape + LFO_SHAPE_COUNT + (direction > 0 ? 1 : -1)) %
                LFO_SHAPE_COUNT;
            s_lfos[1].shape = lfo->shape;
            break;
        case LFO_FIELD_RATE:
            lfo->rate =
                (lfo->rate + LFO_RATE_COUNT + (direction > 0 ? 1 : -1)) %
                LFO_RATE_COUNT;
            s_lfos[1].rate = lfo->rate;
            break;
        case LFO_FIELD_RELATION:
            s_lfo_relation =
                (s_lfo_relation + LFO_RELATION_COUNT +
                 (direction > 0 ? 1 : -1)) % LFO_RELATION_COUNT;
            s_lfos[1].phase = s_lfos[0].phase;
            break;
        case LFO_FIELD_DEPTH: {
            int value = lfo->depth + direction * 4;
            if (value < 0) value = 0;
            if (value > 127) value = 127;
            lfo->depth = value;
            s_lfos[1].depth = lfo->depth;
            break;
        }
        case LFO_FIELD_CENTER: {
            int value = lfo->center + direction * 4;
            if (value < 0) value = 0;
            if (value > 127) value = 127;
            lfo->center = value;
            s_lfos[1].center = lfo->center;
            break;
        }
        default: break;
    }
    s_ui_dirty = true;
}

static void render_lfo_panel(void) {
    pax_draw_round_rect(&s_framebuffer, COLOR_PANEL, 20, 103, s_width - 40, 357, 5);
    pax_draw_rect(&s_framebuffer, COLOR_ACCENT, 20, 103, 8, 357);
    text(42, 115, 14, COLOR_ACCENT, "TWIN LFO");
    text(150, 112, 18, COLOR_TEXT, s_lfo_relation_names[s_lfo_relation]);

    for (uint8_t twin = 0; twin < 2; twin++) {
        midi_lfo_t *current = &s_lfos[twin];
        int graph_y = 148 + twin * 78;
        char label[72];
        if (current->enabled) {
            snprintf(label, sizeof(label), "%c  %s · CHANNEL %u",
                     'A' + twin, s_lfo_target_names[current->target],
                     current->channel + 1);
        } else {
            snprintf(label, sizeof(label), "%c  OFF", 'A' + twin);
        }
        text(42, graph_y - 18, 13,
             current->enabled ? (twin ? COLOR_TEXT : COLOR_ACCENT) : COLOR_DIM,
             label);
        pax_draw_rect(&s_framebuffer, 0xff080909, 42, graph_y, 716, 68);
        pax_draw_rect(&s_framebuffer, COLOR_DIM, 42, graph_y + 34, 716, 1);
        for (int x = 42; x < 758; x++) {
            draw_lfo_curve_column(&s_framebuffer, twin, x);
        }
        int cursor = 42 + (int)(current->phase * 715);
        float wave = lfo_visual_value(s_lfos[0].shape, current->phase);
        if (twin == 1 && s_lfo_relation == 1) wave = -wave;
        int point_y = graph_y + 34 - (int)(wave * 26.0f);
        pax_draw_rect(&s_framebuffer, twin ? COLOR_TEXT : COLOR_ACCENT,
                      cursor, graph_y, 2, 68);
        pax_draw_circle(&s_framebuffer, twin ? COLOR_ACCENT : COLOR_TEXT,
                        cursor + 1, point_y, 4);
        s_lfo_previous_cursor[s_framebuffer_index][twin] = cursor;
    }

    midi_lfo_t *lfo = &s_lfos[0];
    static const char *const field_names[LFO_FIELD_COUNT] = {
        "BIND A", "BIND B", "SHAPE", "RATE", "TWIN MODE", "AMOUNT", "CENTER"
    };
    for (uint8_t field = 0; field < LFO_FIELD_COUNT; field++) {
        uint8_t column = field % 3;
        uint8_t row = field / 3;
        float x = 42 + column * 240;
        float y = 327 + row * 38;
        bool selected = field == s_lfo_field;
        if (selected) pax_draw_rect(&s_framebuffer, COLOR_ACCENT, x, y - 5, 220, 29);
        text(x + 10, y, 13, selected ? COLOR_TEXT : COLOR_DIM, field_names[field]);
        char value[48];
        switch (field) {
            case LFO_FIELD_BIND_A:
            case LFO_FIELD_BIND_B: {
                midi_lfo_t *bound = &s_lfos[field == LFO_FIELD_BIND_B];
                if (bound->enabled) {
                    snprintf(value, sizeof(value), "%s CH%u",
                             s_lfo_target_names[bound->target], bound->channel + 1);
                } else {
                    snprintf(value, sizeof(value), "OFF");
                }
                break;
            }
            case LFO_FIELD_RELATION:
                snprintf(value, sizeof(value), "%s",
                         s_lfo_relation_names[s_lfo_relation]);
                break;
            case LFO_FIELD_SHAPE:
                snprintf(value, sizeof(value), "%s", s_lfo_shape_names[lfo->shape]);
                break;
            case LFO_FIELD_RATE:
                snprintf(value, sizeof(value), "%s", s_lfo_rate_names[lfo->rate]);
                break;
            case LFO_FIELD_DEPTH:
                snprintf(value, sizeof(value), "%u", lfo->depth);
                break;
            default:
                snprintf(value, sizeof(value), "%u", lfo->center);
                break;
        }
        text(x + 10, y + 16, 11, COLOR_TEXT, value);
    }
}

static void render_transport_fast(void) {
    // Transport animation writes into the framebuffer currently scanned by
    // the display. It therefore follows the audio positions without waiting
    // for a complete UI render or another double-buffer swap.
    pax_buf_t *target = &s_framebuffers[s_framebuffer_index ^ 1];
    float progress = s_loop_frames > 0 ?
                     (float)s_loop_position / (float)s_loop_frames : 0;
    if (s_loop_state == UAR_LOOP_RECORDING) {
        uint32_t target_frames =
            ((uint32_t)s_loop_bars * 4U * 60U * 48000U) / s_loop_bpm;
        progress = target_frames > 0 ? (float)s_loop_frames / target_frames : 0;
    }
    if (progress > 1) progress = 1;
    const float timeline_x = 40;
    const float timeline_y = 218;
    const float timeline_w = (float)s_width - 80;
    pax_col_t transport_color = s_loop_state == UAR_LOOP_RECORDING ? COLOR_BAD :
                                s_loop_state == UAR_LOOP_PLAYING ? COLOR_GOOD :
                                COLOR_ACCENT;
    unsigned displayed = s_framebuffer_index ^ 1;
    int16_t cursor = (int16_t)(timeline_x + timeline_w * progress);
    if (s_previous_main_cursor[displayed] >= 0) {
        pax_draw_rect(target, 0xff343535, s_previous_main_cursor[displayed] - 3,
                      timeline_y, 10, 24);
    }
    uint16_t total_beats = (uint16_t)s_loop_bars * 4;
    for (uint16_t beat = 1; beat < total_beats; beat++) {
        float x = timeline_x + timeline_w * ((float)beat / total_beats);
        bool bar_line = (beat % 4) == 0;
        pax_draw_rect(target, bar_line ? COLOR_TEXT : COLOR_DIM, x,
                      timeline_y + (bar_line ? 0 : 16),
                      bar_line ? 2 : 1, bar_line ? 24 : 8);
    }
    pax_draw_rect(target, transport_color, cursor, timeline_y, 4, 24);
    pax_draw_rect(target, transport_color, cursor - 3, timeline_y, 10, 4);
    s_previous_main_cursor[displayed] = cursor;
    uint16_t current_beat = total_beats > 0 ?
        (uint16_t)(progress * total_beats) : 0;
    if (current_beat >= total_beats && total_beats > 0) current_beat = total_beats - 1;
    if (s_previous_beat[displayed] != current_beat) {
        char position[32];
        snprintf(position, sizeof(position), "BAR %02u   BEAT %u/4",
                 current_beat / 4 + 1, current_beat % 4 + 1);
        pax_draw_rect(target, COLOR_PANEL, 36, 246, 210, 27);
        pax_draw_text(target, COLOR_TEXT, pax_font_sky_mono, 16,
                      40, 251, position);
        s_previous_beat[displayed] = current_beat;
    }

    for (uint8_t track = 0; track < 4; track++) {
        float x = 34 + track * 187;
        float width = 170;
        float bar_x = x + 10;
        float bar_y = 343;
        float bar_w = width - 20;
        int16_t previous = s_previous_track_cursor[displayed][track];
        if (previous >= 0) {
            for (uint8_t dx = 0; dx < 2; dx++) {
                uint16_t bin = (uint16_t)(previous + dx - bar_x);
                pax_draw_rect(target, 0xff3b3c3c, previous + dx, bar_y, 1, 18);
                if (bin < UAR_WAVEFORM_BINS) {
                    float amplitude =
                        8.0f * s_track_waveforms[track][bin] / 32767.0f;
                    pax_draw_rect(target, COLOR_GOOD, previous + dx,
                                  bar_y + 9 - amplitude, 1, amplitude * 2 + 1);
                }
            }
        }
        if (s_track_frames[track] > 0) {
            float phase = (float)s_track_positions[track] / s_track_frames[track];
            int16_t track_cursor = (int16_t)(bar_x + (bar_w - 1) * phase);
            pax_draw_rect(target, COLOR_ACCENT, track_cursor, bar_y, 2, 18);
            s_previous_track_cursor[displayed][track] = track_cursor;
        }
    }
}

static void render(const uar_probe_snapshot_t *probe, char diagnostics[][72]) {
    int64_t draw_started = esp_timer_get_time();
    pax_background(&s_framebuffer, COLOR_BG);
    pax_draw_rect(&s_framebuffer, COLOR_SHADOW, 0, 0, s_width, 5);
    pax_draw_rect(&s_framebuffer, COLOR_SILVER, 0, 5, s_width, 86);
    pax_draw_rect(&s_framebuffer, COLOR_DIM, 0, 88, s_width, 3);

    pax_draw_rect(&s_framebuffer, COLOR_TEXT, 18, 5, 92, 25);
    pax_draw_rect(&s_framebuffer, COLOR_ACCENT, 110, 5, 116, 25);
    pax_draw_rect(&s_framebuffer, COLOR_DARK, 226, 5, 92, 25);
    text(38, 10, 13, COLOR_DIM, "OUTPUT");
    text(145, 10, 13, COLOR_TEXT, "INPUT");
    text(252, 10, 13, COLOR_TEXT, "USB");

    text(24, 39, 30, COLOR_DARK, "K.O.");
    text(102, 47, 19, COLOR_DIM, "SIDEKICK SIDEKICK");
    text(25, 68, 14, COLOR_ACCENT, "ミキサー  //  LOOP STATION");
    pax_draw_circle(&s_framebuffer, COLOR_SHADOW, 10, 78, 6);
    pax_draw_circle(&s_framebuffer, COLOR_SHADOW, s_width - 10, 78, 6);
    text(s_width - 180, 51, 14, probe->connected ? COLOR_ACCENT : COLOR_DIM,
         probe->connected ? "● EP-136 LIVE" : "○ NO DEVICE");
    char performance[32];
    snprintf(performance, sizeof(performance), "UI %lu+%lu ms",
             (unsigned long)s_ui_draw_ms, (unsigned long)s_ui_present_ms);
    text(s_width - 178, 70, 11, COLOR_DIM, performance);

    if (!probe->connected) {
        pax_draw_round_rect(&s_framebuffer, COLOR_PANEL, 20, 105, s_width - 40, 250, 5);
        text(150, 190, 28, COLOR_ACCENT, "CONNECT EP-136 TO START");
        text(218, 238, 16, COLOR_TEXT, "USB AUDIO  //  48 kHz");
    } else if (s_ui_page == UI_PAGE_LFO) {
        render_lfo_panel();
    } else {
        char line[96];
        static const char *const states[] = {"READY", "RECORDING", "PLAYING", "PAUSED"};
        pax_col_t transport_color = s_loop_state == UAR_LOOP_RECORDING ? COLOR_BAD :
                                    s_loop_state == UAR_LOOP_PLAYING ? COLOR_GOOD :
                                    COLOR_ACCENT;
        pax_draw_round_rect(&s_framebuffer, COLOR_PANEL, 20, 103, s_width - 40, 178, 5);
        pax_draw_rect(&s_framebuffer, COLOR_ACCENT, 20, 103, 8, 178);
        text(42, 115, 14, COLOR_ACCENT, "TEMPO");
        snprintf(line, sizeof(line), "%u", s_loop_bpm);
        text(38, 135, 52, COLOR_TEXT, line);
        text(202, 115, 14, COLOR_ACCENT, "BAR");
        snprintf(line, sizeof(line), "%u", s_loop_bars);
        text(203, 135, 52, COLOR_TEXT, line);
        text(336, 115, 14, COLOR_DIM, "LOOPER STATUS");
        text(336, 142, 28, transport_color,
             s_record_armed ? "WAITING FOR BAR" :
             s_overdubbing ? "OVERDUB" : states[s_loop_state]);

        float progress = s_loop_frames > 0 ?
                         (float)s_loop_position / (float)s_loop_frames : 0;
        if (s_loop_state == UAR_LOOP_RECORDING) {
            uint32_t target = ((uint32_t)s_loop_bars * 4U * 60U * 48000U) / s_loop_bpm;
            progress = target > 0 ? (float)s_loop_frames / (float)target : 0;
        }
        if (progress > 1) progress = 1;
        const float timeline_x = 40;
        const float timeline_y = 218;
        const float timeline_w = (float)s_width - 80;
        pax_draw_rect(&s_framebuffer, 0xff343535, timeline_x, timeline_y, timeline_w, 24);
        pax_draw_rect(&s_framebuffer, transport_color,
                      timeline_x + timeline_w * progress, timeline_y, 3, 24);
        s_previous_main_cursor[s_framebuffer_index] =
            (int16_t)(timeline_x + timeline_w * progress);
        uint16_t total_beats = (uint16_t)s_loop_bars * 4;
        for (uint16_t beat = 1; beat < total_beats; beat++) {
            float x = timeline_x + timeline_w * ((float)beat / total_beats);
            bool bar_line = (beat % 4) == 0;
            pax_draw_rect(&s_framebuffer, bar_line ? COLOR_TEXT : COLOR_DIM, x,
                          timeline_y + (bar_line ? 0 : 16),
                          bar_line ? 2 : 1, bar_line ? 24 : 8);
        }
        uint16_t current_beat = total_beats > 0 ?
            (uint16_t)(progress * total_beats) : 0;
        if (current_beat >= total_beats && total_beats > 0) current_beat = total_beats - 1;
        snprintf(line, sizeof(line), "BAR %02u   BEAT %u/4",
                 current_beat / 4 + 1, current_beat % 4 + 1);
        text(40, 251, 16, COLOR_TEXT, line);
        s_previous_beat[s_framebuffer_index] = current_beat;
        snprintf(line, sizeof(line), "METRO %s", s_metronome ? "ON" : "OFF");
        text(164, 251, 14, s_metronome ? COLOR_ACCENT : COLOR_DIM, line);

        for (uint8_t track = 0; track < 4; track++) {
            float x = 34 + track * 187;
            float y = 296;
            float width = 170;
            float height = 98;
            pax_col_t color = track == s_selected_track ? COLOR_ACCENT :
                              s_track_frames[track] ? COLOR_DARK : COLOR_SHADOW;
            pax_draw_rect(&s_framebuffer, COLOR_SHADOW, x + 3, y + 4, width, height);
            pax_draw_rect(&s_framebuffer, COLOR_DARK, x, y, width, height);
            pax_draw_rect(&s_framebuffer, color, x, y, width, 5);
            pax_draw_rect(&s_framebuffer, 0xff3b3c3c, x + 10, y + 47, width - 20, 18);
            if (s_track_frames[track] > 0) {
                float phase = (float)s_track_positions[track] / s_track_frames[track];
                for (uint16_t bin = 0; bin < UAR_WAVEFORM_BINS; bin++) {
                    float amplitude =
                        8.0f * s_track_waveforms[track][bin] / 32767.0f;
                    pax_draw_rect(&s_framebuffer, COLOR_GOOD, x + 10 + bin,
                                  y + 56 - amplitude, 1, amplitude * 2 + 1);
                }
                pax_draw_rect(&s_framebuffer, COLOR_ACCENT,
                              x + 10 + (width - 21) * phase, y + 47, 2, 18);
                s_previous_track_cursor[s_framebuffer_index][track] =
                    (int16_t)(x + 10 + (width - 21) * phase);
            } else {
                s_previous_track_cursor[s_framebuffer_index][track] = -1;
            }
            snprintf(line, sizeof(line), "%u", track + 1);
            text(x + 11, y + 12, 25, COLOR_TEXT, line);
            text(x + 43, y + 18, 12, color,
                 track == s_selected_track ? "SELECT" : "LOOP");
            snprintf(line, sizeof(line), "%u%%", s_track_volumes[track]);
            text(x + 10, y + 72, 13, COLOR_TEXT, line);
        }

    }

    s_ui_draw_ms = (uint32_t)((esp_timer_get_time() - draw_started + 500) / 1000);
    int64_t present_started = esp_timer_get_time();
    bsp_display_blit(0, 0, s_physical_width, s_physical_height,
                     pax_buf_get_pixels(&s_framebuffer));
    s_ui_present_ms = (uint32_t)((esp_timer_get_time() - present_started + 500) / 1000);
    s_framebuffer_index ^= 1;
    s_framebuffer = s_framebuffers[s_framebuffer_index];
}

static void graphics_initialize(void) {
    bsp_display_color_format_t format;
    bsp_display_endianness_t endianness;
    ESP_ERROR_CHECK(bsp_display_get_parameters(
        &s_physical_width, &s_physical_height, &format, &endianness
    ));
    ESP_ERROR_CHECK(format == BSP_DISPLAY_COLOR_FORMAT_16_565RGB ?
                    ESP_OK : ESP_ERR_NOT_SUPPORTED);
    esp_lcd_panel_handle_t panel;
    ESP_ERROR_CHECK(bsp_display_get_panel(&panel));
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(
        panel, 2, &s_display_pixels[0], &s_display_pixels[1]
    ));
    pax_orientation_t orientation = PAX_O_UPRIGHT;
    switch (bsp_display_get_default_rotation()) {
        case BSP_DISPLAY_ROTATION_90: orientation = PAX_O_ROT_CCW; break;
        case BSP_DISPLAY_ROTATION_180: orientation = PAX_O_ROT_HALF; break;
        case BSP_DISPLAY_ROTATION_270: orientation = PAX_O_ROT_CW; break;
        default: break;
    }
    for (size_t index = 0; index < 2; index++) {
        ESP_ERROR_CHECK(pax_buf_init(
            &s_framebuffers[index], s_display_pixels[index],
            s_physical_width, s_physical_height, PAX_BUF_16_565RGB
        ) ? ESP_OK : ESP_ERR_NO_MEM);
        pax_buf_reversed(&s_framebuffers[index], endianness == BSP_DISPLAY_ENDIAN_BIG);
        pax_buf_set_orientation(&s_framebuffers[index], orientation);
    }
    s_framebuffer_index = 0;
    s_framebuffer = s_framebuffers[0];
    s_width = pax_buf_get_width(&s_framebuffer);
    s_height = pax_buf_get_height(&s_framebuffer);
    ESP_LOGI(TAG, "Display: physical %ux%u, logical %ux%u",
             (unsigned)s_physical_width, (unsigned)s_physical_height,
             (unsigned)s_width, (unsigned)s_height);
}

void app_main(void) {
    const bsp_configuration_t configuration = {
        .display = {
            .requested_color_format = BSP_DISPLAY_COLOR_FORMAT_16_565RGB,
            .num_fbs = 2,
        },
    };
    ESP_ERROR_CHECK(bsp_device_initialize(&configuration));
    graphics_initialize();
    ESP_ERROR_CHECK(bsp_input_get_queue(&s_input_queue));
    sidekick_lvgl_init(s_display_pixels[0], s_display_pixels[1],
                       s_physical_width, s_physical_height);
    vTaskPrioritySet(NULL, 5);

    uar_probe_snapshot_t previous = {0};
    char diagnostics[6][72] = {{0}};
    uint32_t diagnostic_revision = 0;
    uint32_t previous_diagnostic_revision = UINT32_MAX;
    uar_probe_snapshot(&previous);
    uar_probe_diagnostics(diagnostics, 6, &diagnostic_revision);
    sidekick_lvgl_run();

    esp_err_t power_error = bsp_power_set_usb_host_boost_enabled(true);
    if (power_error != ESP_OK) {
        // Keep going: USB host signalling can still work when the EP-136 is
        // battery-powered, and this mirrors the proven DiscoMatsu USB path.
        ESP_LOGW(TAG, "USB VBUS boost request failed: %s", esp_err_to_name(power_error));
    }
    // Give the PMIC/VBUS rail time to settle before the host controller starts
    // looking for a device already attached at application launch.
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_err_t error = uar_probe_start();
    bool usb_start_failed = error != ESP_OK;
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "USB host start failed: %s", esp_err_to_name(error));
        snprintf(previous.status, sizeof(previous.status), "USB host error: %s",
                 esp_err_to_name(error));
    }

    while (true) {
        bsp_input_event_t event;
        while (xQueueReceive(s_input_queue, &event, 0) == pdTRUE) {
            if (event.type == INPUT_EVENT_TYPE_NAVIGATION &&
                event.args_navigation.state) {
                if (s_ui_page == UI_PAGE_LFO) {
                    switch (event.args_navigation.key) {
                        case BSP_INPUT_NAVIGATION_KEY_F1:
                            bsp_device_restart_to_launcher();
                            break;
                        case BSP_INPUT_NAVIGATION_KEY_UP:
                            s_lfo_field =
                                (s_lfo_field + LFO_FIELD_COUNT - 1) % LFO_FIELD_COUNT;
                            s_ui_dirty = true;
                            break;
                        case BSP_INPUT_NAVIGATION_KEY_DOWN:
                            s_lfo_field = (s_lfo_field + 1) % LFO_FIELD_COUNT;
                            s_ui_dirty = true;
                            break;
                        case BSP_INPUT_NAVIGATION_KEY_LEFT:
                            lfo_adjust_selected(-1);
                            break;
                        case BSP_INPUT_NAVIGATION_KEY_RIGHT:
                            lfo_adjust_selected(1);
                            break;
                        case BSP_INPUT_NAVIGATION_KEY_VOLUME_UP:
                            uar_monitor_adjust_volume(5);
                            break;
                        case BSP_INPUT_NAVIGATION_KEY_VOLUME_DOWN:
                            uar_monitor_adjust_volume(-5);
                            break;
                        default:
                            break;
                    }
                    continue;
                }
                switch (event.args_navigation.key) {
                    case BSP_INPUT_NAVIGATION_KEY_F1:
                        bsp_device_restart_to_launcher();
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_F2:
                        uar_loop_record_toggle();
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_F3:
                        uar_loop_play_toggle();
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_F4:
                        uar_loop_clear();
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_F5:
                        uar_loop_select_next_track();
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_UP: {
                        bool bpm_modifier = false;
                        bsp_input_read_scancode(BSP_INPUT_SCANCODE_B, &bpm_modifier);
                        if (bpm_modifier) uar_loop_adjust_bpm(1);
                        else uar_loop_adjust_track_volume(5);
                        break;
                    }
                    case BSP_INPUT_NAVIGATION_KEY_DOWN: {
                        bool bpm_modifier = false;
                        bsp_input_read_scancode(BSP_INPUT_SCANCODE_B, &bpm_modifier);
                        if (bpm_modifier) uar_loop_adjust_bpm(-1);
                        else uar_loop_adjust_track_volume(-5);
                        break;
                    }
                    case BSP_INPUT_NAVIGATION_KEY_LEFT:
                        uar_loop_adjust_bars(-1);
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_RIGHT:
                        uar_loop_adjust_bars(1);
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_VOLUME_UP:
                        uar_monitor_adjust_volume(5);
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_VOLUME_DOWN:
                        uar_monitor_adjust_volume(-5);
                        break;
                    default:
                        break;
                }
            } else if (event.type == INPUT_EVENT_TYPE_KEYBOARD &&
                       event.args_keyboard.ascii == '\t') {
                s_ui_page = (ui_page_t)((s_ui_page + 1) % 2);
                s_ui_dirty = true;
            } else if (event.type == INPUT_EVENT_TYPE_KEYBOARD &&
                       s_ui_page == UI_PAGE_LOOPER &&
                       (event.args_keyboard.ascii == 'm' ||
                        event.args_keyboard.ascii == 'M')) {
                uar_loop_toggle_metronome();
            } else if (event.type == INPUT_EVENT_TYPE_KEYBOARD &&
                       s_ui_page == UI_PAGE_LOOPER &&
                       event.args_keyboard.ascii >= '1' &&
                       event.args_keyboard.ascii <= '4') {
                uar_loop_select_track((uint8_t)(event.args_keyboard.ascii - '1'));
            } else if (event.type == INPUT_EVENT_TYPE_KEYBOARD &&
                       s_ui_page == UI_PAGE_LOOPER &&
                       (event.args_keyboard.ascii == 't' ||
                        event.args_keyboard.ascii == 'T')) {
                uar_loop_tap_tempo();
            }
        }
        if (usb_start_failed) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        lfo_tick();
        uar_probe_snapshot_t current;
        uar_probe_snapshot(&current);
        uar_probe_diagnostics(diagnostics, 6, &diagnostic_revision);
        uint16_t fresh_levels[8];
        uint32_t packet_count = 0, error_count = 0;
        uint64_t audio_bytes = 0, nonzero_bytes = 0;
        uar_probe_read_levels(
            fresh_levels, &packet_count, &error_count, &audio_bytes, &nonzero_bytes
        );
        uar_loop_state_t loop_state;
        uint32_t loop_frames = 0, loop_position = 0;
        uint16_t loop_bpm = 120;
        uint8_t loop_bars = 4;
        uint8_t selected_track = 0;
        uint32_t track_frames[4] = {0};
        uint32_t track_positions[4] = {0};
        bool metronome = false;
        bool overdubbing = uar_loop_is_overdubbing();
        bool record_armed = uar_loop_is_record_armed();
        uint8_t track_volumes[4];
        uint16_t track_waveforms[4][UAR_WAVEFORM_BINS];
        uar_loop_get_track_volumes(track_volumes);
        uar_loop_get_waveforms(track_waveforms);
        uar_loop_get_state(&loop_state, &loop_frames, &loop_position);
        uar_loop_get_settings(&loop_bpm, &loop_bars);
        uar_loop_get_tracks(&selected_track, track_frames, track_positions, &metronome);
        bool track_content_changed = false;
        for (uint8_t track = 0; track < 4; track++) {
            if ((track_frames[track] == 0) != (s_track_frames[track] == 0)) {
                track_content_changed = true;
            }
        }
        bool loop_changed = loop_state != s_loop_state || loop_bpm != s_loop_bpm ||
                            loop_bars != s_loop_bars || selected_track != s_selected_track ||
                            metronome != s_metronome ||
                            overdubbing != s_overdubbing ||
                            record_armed != s_record_armed ||
                            memcmp(track_volumes, s_track_volumes,
                                   sizeof(track_volumes)) != 0 ||
                            track_content_changed;
        s_loop_state = loop_state;
        s_loop_frames = loop_frames;
        s_loop_position = loop_position;
        s_loop_bpm = loop_bpm;
        s_loop_bars = loop_bars;
        s_selected_track = selected_track;
        s_overdubbing = overdubbing;
        s_record_armed = record_armed;
        memcpy(s_track_volumes, track_volumes, sizeof(s_track_volumes));
        memcpy(s_track_waveforms, track_waveforms, sizeof(s_track_waveforms));
        memcpy(s_track_frames, track_frames, sizeof(s_track_frames));
        memcpy(s_track_positions, track_positions, sizeof(s_track_positions));
        s_metronome = metronome;
        s_audio_packets = packet_count;
        s_audio_errors = error_count;
        s_audio_bytes = audio_bytes;
        s_nonzero_bytes = nonzero_bytes;
        for (uint8_t channel = 0; channel < 8; channel++) {
            if (fresh_levels[channel] >= s_display_levels[channel]) {
                s_display_levels[channel] = fresh_levels[channel];
            } else {
                s_display_levels[channel] =
                    (uint16_t)(((uint32_t)s_display_levels[channel] * 82) / 100);
            }
            if (fresh_levels[channel] >= s_peak_hold[channel]) {
                s_peak_hold[channel] = fresh_levels[channel];
                s_peak_hold_ticks[channel] = 18;
            } else if (s_peak_hold_ticks[channel] > 0) {
                s_peak_hold_ticks[channel]--;
            } else {
                s_peak_hold[channel] = (uint16_t)(((uint32_t)s_peak_hold[channel] * 94) / 100);
            }
        }
        sidekick_ui_state_t ui = {
            .device_connected = current.connected,
            .midi_connected = uar_midi_is_connected(),
            .lfo_page = s_ui_page == UI_PAGE_LFO,
            .bpm = s_loop_bpm,
            .bars = s_loop_bars,
            .loop_state = s_loop_state,
            .loop_frames = s_loop_frames,
            .loop_position = s_loop_position,
            .record_armed = s_record_armed,
            .overdubbing = s_overdubbing,
            .metronome = s_metronome,
            .selected_track = s_selected_track,
            .lfo_field = s_lfo_field,
            .lfo_shape = s_lfos[0].shape,
            .lfo_rate = s_lfos[0].rate,
            .lfo_relation = s_lfo_relation,
            .lfo_depth = s_lfos[0].depth,
            .lfo_center = s_lfos[0].center,
        };
        memcpy(ui.track_volumes, s_track_volumes, sizeof(ui.track_volumes));
        memcpy(ui.track_frames, s_track_frames, sizeof(ui.track_frames));
        memcpy(ui.track_positions, s_track_positions, sizeof(ui.track_positions));
        memcpy(ui.track_waveforms, s_track_waveforms, sizeof(ui.track_waveforms));
        for (uint8_t twin = 0; twin < 2; twin++) {
            ui.lfo_enabled[twin] = s_lfos[twin].enabled;
            ui.lfo_channel[twin] = s_lfos[twin].channel;
            ui.lfo_target[twin] = s_lfos[twin].target;
            ui.lfo_phase[twin] = s_lfos[twin].phase;
        }
        bool full_update = memcmp(&current, &previous, sizeof(current)) != 0 ||
                           diagnostic_revision != previous_diagnostic_revision ||
                           loop_changed || s_ui_dirty;
        sidekick_lvgl_update(&ui, full_update);
        sidekick_lvgl_run();
        previous = current;
        previous_diagnostic_revision = diagnostic_revision;
        s_ui_dirty = false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
