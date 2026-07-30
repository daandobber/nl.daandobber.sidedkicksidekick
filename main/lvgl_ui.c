#include "lvgl_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bsp/display.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "lvgl.h"

#define UI_W 800
#define UI_H 480
#define ORANGE lv_color_hex(0xff4b16)
#define ORANGE_2 lv_color_hex(0xff762d)
#define OFF_WHITE lv_color_hex(0xf1f0ec)
#define SILVER lv_color_hex(0xc8c7c5)
#define PANEL lv_color_hex(0x111313)
#define DARK lv_color_hex(0x262728)
#define DIM lv_color_hex(0x696968)

static uint16_t *s_hw_fb[2];
static uint16_t s_physical_width;
static uint16_t s_physical_height;
static uint8_t s_present_fb;
static lv_obj_t *s_looper;
static lv_obj_t *s_lfo;
static lv_obj_t *s_device;
static lv_obj_t *s_tempo;
static lv_obj_t *s_bars;
static lv_obj_t *s_status;
static lv_obj_t *s_timeline;
static lv_obj_t *s_timeline_cursor;
static lv_obj_t *s_track[4];
static lv_obj_t *s_track_title[4];
static lv_obj_t *s_track_volume[4];
static lv_obj_t *s_track_wave[4];
static lv_point_precise_t s_track_points[4][UAR_WAVEFORM_BINS];
static lv_obj_t *s_lfo_binding[2];
static lv_obj_t *s_lfo_wave[2];
static lv_obj_t *s_lfo_cursor[2];
static lv_point_precise_t s_lfo_points[2][120];
static lv_obj_t *s_fields[7];
static lv_obj_t *s_field_value[7];

static uint32_t tick_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void flush_cb(lv_display_t *display, const lv_area_t *area,
                     uint8_t *pixels) {
    const uint16_t *source = (const uint16_t *)pixels;
    int width = lv_area_get_width(area);
    for (int y = area->y1; y <= area->y2; y++) {
        for (int x = area->x1; x <= area->x2; x++) {
            uint16_t pixel = source[(y - area->y1) * width + (x - area->x1)];
            size_t physical =
                (size_t)x * s_physical_width + (s_physical_width - 1 - y);
            s_hw_fb[0][physical] = pixel;
            s_hw_fb[1][physical] = pixel;
        }
    }
    if (lv_display_flush_is_last(display)) {
        s_present_fb ^= 1;
        bsp_display_blit(0, 0, s_physical_width, s_physical_height,
                         s_hw_fb[s_present_fb]);
    }
    lv_display_flush_ready(display);
}

static void clean(lv_obj_t *object, lv_color_t color) {
    lv_obj_remove_style_all(object);
    lv_obj_set_style_bg_color(object, color, 0);
    lv_obj_set_style_bg_opa(object, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(object, 0, 0);
    lv_obj_set_style_pad_all(object, 0, 0);
}

static lv_obj_t *label(lv_obj_t *parent, const char *value, int x, int y,
                       lv_color_t color) {
    lv_obj_t *result = lv_label_create(parent);
    lv_label_set_text(result, value);
    lv_obj_set_pos(result, x, y);
    lv_obj_set_style_text_color(result, color, 0);
    return result;
}

static lv_obj_t *panel(lv_obj_t *parent, int x, int y, int w, int h) {
    lv_obj_t *result = lv_obj_create(parent);
    clean(result, PANEL);
    lv_obj_set_pos(result, x, y);
    lv_obj_set_size(result, w, h);
    lv_obj_set_style_radius(result, 6, 0);
    return result;
}

static void build_header(lv_obj_t *screen) {
    lv_obj_t *tabs = lv_obj_create(screen);
    clean(tabs, OFF_WHITE);
    lv_obj_set_pos(tabs, 0, 0);
    lv_obj_set_size(tabs, UI_W, 88);
    lv_obj_t *accent = lv_obj_create(tabs);
    clean(accent, ORANGE);
    lv_obj_set_pos(accent, 0, 0);
    lv_obj_set_size(accent, 8, 88);
    label(tabs, "K.O. SIDEKICK", 24, 14, PANEL);
    label(tabs, "SIDEKICK SIDEKICK", 24, 42, DIM);
    s_device = label(tabs, "NO DEVICE", 625, 25, DIM);
}

static void build_looper(lv_obj_t *screen) {
    s_looper = lv_obj_create(screen);
    clean(s_looper, SILVER);
    lv_obj_set_pos(s_looper, 0, 88);
    lv_obj_set_size(s_looper, UI_W, UI_H - 88);

    lv_obj_t *transport = panel(s_looper, 20, 15, 760, 178);
    lv_obj_t *edge = lv_obj_create(transport);
    clean(edge, ORANGE);
    lv_obj_set_size(edge, 8, 178);
    label(transport, "TEMPO", 22, 12, ORANGE);
    s_tempo = label(transport, "120", 20, 38, OFF_WHITE);
    lv_obj_set_style_text_font(s_tempo, &lv_font_montserrat_48, 0);
    label(transport, "BAR", 182, 12, ORANGE);
    s_bars = label(transport, "4", 182, 38, OFF_WHITE);
    lv_obj_set_style_text_font(s_bars, &lv_font_montserrat_48, 0);
    label(transport, "LOOPER STATUS", 316, 12, DIM);
    s_status = label(transport, "READY", 316, 42, ORANGE);
    lv_obj_set_style_text_font(s_status, &lv_font_montserrat_28, 0);
    s_timeline = lv_obj_create(transport);
    clean(s_timeline, lv_color_hex(0x343535));
    lv_obj_set_pos(s_timeline, 20, 115);
    lv_obj_set_size(s_timeline, 720, 28);
    s_timeline_cursor = lv_obj_create(s_timeline);
    clean(s_timeline_cursor, ORANGE);
    lv_obj_set_size(s_timeline_cursor, 4, 28);

    for (int track = 0; track < 4; track++) {
        s_track[track] = panel(s_looper, 34 + track * 187, 208, 170, 145);
        s_track_title[track] = label(s_track[track], "1  LOOP", 10, 10, OFF_WHITE);
        s_track_wave[track] = lv_line_create(s_track[track]);
        lv_obj_set_pos(s_track_wave[track], 10, 55);
        lv_obj_set_style_line_color(s_track_wave[track], ORANGE_2, 0);
        lv_obj_set_style_line_width(s_track_wave[track], 2, 0);
        s_track_volume[track] = label(s_track[track], "100%", 10, 112, OFF_WHITE);
    }
}

static void build_lfo(lv_obj_t *screen) {
    s_lfo = lv_obj_create(screen);
    clean(s_lfo, SILVER);
    lv_obj_set_pos(s_lfo, 0, 88);
    lv_obj_set_size(s_lfo, UI_W, UI_H - 88);
    lv_obj_t *body = panel(s_lfo, 20, 15, 760, 362);
    lv_obj_t *edge = lv_obj_create(body);
    clean(edge, ORANGE);
    lv_obj_set_size(edge, 8, 362);
    label(body, "TWIN LFO", 22, 10, ORANGE);

    for (int twin = 0; twin < 2; twin++) {
        s_lfo_binding[twin] = label(body, twin ? "B  OFF" : "A  FX Y", 22,
                                    38 + twin * 82, twin ? OFF_WHITE : ORANGE);
        lv_obj_t *graph = lv_obj_create(body);
        clean(graph, lv_color_hex(0x080909));
        lv_obj_set_pos(graph, 22, 58 + twin * 82);
        lv_obj_set_size(graph, 716, 58);
        s_lfo_wave[twin] = lv_line_create(graph);
        lv_obj_set_style_line_color(s_lfo_wave[twin], ORANGE_2, 0);
        lv_obj_set_style_line_width(s_lfo_wave[twin], 2, 0);
        s_lfo_cursor[twin] = lv_obj_create(graph);
        clean(s_lfo_cursor[twin], twin ? OFF_WHITE : ORANGE);
        lv_obj_set_size(s_lfo_cursor[twin], 3, 58);
    }

    static const char *names[7] = {
        "BIND A", "BIND B", "SHAPE", "RATE", "TWIN MODE", "AMOUNT", "CENTER"
    };
    for (int field = 0; field < 7; field++) {
        int x = 22 + (field % 3) * 240;
        int y = 229 + (field / 3) * 42;
        s_fields[field] = lv_obj_create(body);
        clean(s_fields[field], DARK);
        lv_obj_set_pos(s_fields[field], x, y);
        lv_obj_set_size(s_fields[field], 220, 38);
        label(s_fields[field], names[field], 8, 3, DIM);
        s_field_value[field] = label(s_fields[field], "", 90, 3, OFF_WHITE);
    }
}

void sidekick_lvgl_init(void *framebuffer_a, void *framebuffer_b,
                        uint16_t physical_width, uint16_t physical_height) {
    s_hw_fb[0] = framebuffer_a;
    s_hw_fb[1] = framebuffer_b;
    s_physical_width = physical_width;
    s_physical_height = physical_height;
    memset(framebuffer_a, 0, (size_t)physical_width * physical_height * 2);
    memset(framebuffer_b, 0, (size_t)physical_width * physical_height * 2);

    lv_init();
    lv_tick_set_cb(tick_ms);
    lv_display_t *display = lv_display_create(UI_W, UI_H);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    size_t buffer_size = UI_W * 48 * sizeof(lv_color16_t);
    void *buffer_a = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    void *buffer_b = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    lv_display_set_buffers(display, buffer_a, buffer_b, buffer_size,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, flush_cb);

    lv_obj_t *screen = lv_screen_active();
    clean(screen, SILVER);
    build_header(screen);
    build_looper(screen);
    build_lfo(screen);
    lv_obj_add_flag(s_lfo, LV_OBJ_FLAG_HIDDEN);
}

static float visual_value(uint8_t shape, float phase) {
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

void sidekick_lvgl_update(const sidekick_ui_state_t *state, bool force) {
    static bool previous_page;
    static uint32_t previous_waveform_ms;
    uint32_t now = tick_ms();
    bool waveform_refresh = force || now - previous_waveform_ms >= 80;
    if (waveform_refresh) previous_waveform_ms = now;
    if (force || state->lfo_page != previous_page) {
        if (state->lfo_page) {
            lv_obj_add_flag(s_looper, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(s_lfo, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_lfo, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(s_looper, LV_OBJ_FLAG_HIDDEN);
        }
        previous_page = state->lfo_page;
    }
    lv_label_set_text(s_device, state->device_connected ? "EP-136 LIVE" : "NO DEVICE");
    lv_obj_set_style_text_color(s_device,
        state->device_connected ? ORANGE : DIM, 0);

    char value[64];
    snprintf(value, sizeof(value), "%u", state->bpm);
    lv_label_set_text(s_tempo, value);
    snprintf(value, sizeof(value), "%u", state->bars);
    lv_label_set_text(s_bars, value);
    const char *status = state->record_armed ? "WAITING FOR BAR" :
                         state->overdubbing ? "OVERDUB" :
                         state->loop_state == UAR_LOOP_RECORDING ? "RECORDING" :
                         state->loop_state == UAR_LOOP_PLAYING ? "PLAYING" :
                         state->loop_state == UAR_LOOP_PAUSED ? "PAUSED" : "READY";
    lv_label_set_text(s_status, status);
    lv_obj_set_style_bg_color(s_timeline_cursor,
        state->loop_state == UAR_LOOP_RECORDING ? lv_color_hex(0xff3010) :
        state->loop_state == UAR_LOOP_PLAYING ? ORANGE_2 : ORANGE, 0);
    float progress = state->loop_frames ?
        (float)state->loop_position / state->loop_frames : 0;
    lv_obj_set_x(s_timeline_cursor, (int)(716 * progress));

    for (int track = 0; track < 4; track++) {
        if (waveform_refresh) {
            snprintf(value, sizeof(value), "%d  %s", track + 1,
                     track == state->selected_track ? "SELECT" : "LOOP");
            lv_label_set_text(s_track_title[track], value);
            lv_obj_set_style_bg_color(s_track[track],
                track == state->selected_track ? lv_color_hex(0x343535) : PANEL, 0);
            snprintf(value, sizeof(value), "%u%%", state->track_volumes[track]);
            lv_label_set_text(s_track_volume[track], value);
            for (int bin = 0; bin < UAR_WAVEFORM_BINS; bin++) {
                s_track_points[track][bin].x = bin * 148 / (UAR_WAVEFORM_BINS - 1);
                s_track_points[track][bin].y =
                    25 - (int32_t)(22.0f * state->track_waveforms[track][bin] / 32767.0f);
            }
            lv_line_set_points(s_track_wave[track], s_track_points[track],
                               UAR_WAVEFORM_BINS);
        }
    }

    static const char *targets[] = {"VOLUME", "FX Y · FORCE", "FX X · STICK"};
    static const char *shapes[] = {
        "SINE", "TRIANGLE", "SAW UP", "SAW DOWN", "SQUARE", "RANDOM"
    };
    static const char *rates[] = {
        "0.10 Hz", "0.25 Hz", "0.50 Hz", "1.00 Hz", "2.00 Hz", "4.00 Hz",
        "8.00 Hz", "1/16", "1/8", "1/4", "1/2", "1 BAR", "2 BAR"
    };
    static const char *relations[] = {
        "SAME", "INVERTED", "B ×2", "B ×3", "B ×4", "B ÷2", "B ÷3", "B ÷4"
    };
    for (int twin = 0; twin < 2; twin++) {
        if (force) {
            if (state->lfo_enabled[twin]) {
                snprintf(value, sizeof(value), "%c  %s · CHANNEL %u", 'A' + twin,
                         targets[state->lfo_target[twin]], state->lfo_channel[twin] + 1);
            } else {
                snprintf(value, sizeof(value), "%c  OFF", 'A' + twin);
            }
            lv_label_set_text(s_lfo_binding[twin], value);
            for (int point = 0; point < 120; point++) {
                float phase = (float)point / 119.0f;
                float wave = visual_value(state->lfo_shape, phase);
                if (twin == 1 && state->lfo_relation == 1) wave = -wave;
                s_lfo_points[twin][point].x = point * 712 / 119;
                s_lfo_points[twin][point].y = 29 - (int)(wave * 23);
            }
            lv_line_set_points(s_lfo_wave[twin], s_lfo_points[twin], 120);
        }
        lv_obj_set_x(s_lfo_cursor[twin], (int)(state->lfo_phase[twin] * 713));
    }
    if (force) {
        for (int field = 0; field < 7; field++) {
            lv_obj_set_style_bg_color(s_fields[field],
                field == state->lfo_field ? ORANGE : DARK, 0);
        }
        for (int twin = 0; twin < 2; twin++) {
            if (state->lfo_enabled[twin]) {
                snprintf(value, sizeof(value), "%s CH%u",
                         targets[state->lfo_target[twin]], state->lfo_channel[twin] + 1);
            } else snprintf(value, sizeof(value), "OFF");
            lv_label_set_text(s_field_value[twin], value);
        }
        lv_label_set_text(s_field_value[2], shapes[state->lfo_shape]);
        lv_label_set_text(s_field_value[3], rates[state->lfo_rate]);
        lv_label_set_text(s_field_value[4], relations[state->lfo_relation]);
        snprintf(value, sizeof(value), "%u", state->lfo_depth);
        lv_label_set_text(s_field_value[5], value);
        snprintf(value, sizeof(value), "%u", state->lfo_center);
        lv_label_set_text(s_field_value[6], value);
    }
}

void sidekick_lvgl_run(void) {
    lv_timer_handler();
}
