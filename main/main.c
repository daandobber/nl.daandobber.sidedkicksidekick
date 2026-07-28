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
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"
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
static uint32_t s_ui_draw_ms;
static uint32_t s_ui_present_ms;

static void text(float x, float y, float size, pax_col_t color, const char *value) {
    pax_draw_text(&s_framebuffer, color, pax_font_sky_mono, size, x, y, value);
}

static void render_meters_fast(void) {
    // The most recently presented hardware framebuffer is the other buffer.
    // Updating this small region directly lets the DPI engine pick it up on
    // its next scan without waiting for a complete UI render or buffer swap.
    pax_buf_t *target = &s_framebuffers[s_framebuffer_index ^ 1];
    const float meter_top = 306;
    const float meter_w = 22;
    for (uint8_t channel = 0; channel < 6; channel++) {
        float x = s_width - 154 + channel * (meter_w + 2);
        pax_draw_rect(target, COLOR_DARK, x, meter_top, meter_w, 68);
        float normalized = sqrtf((float)s_display_levels[channel] / 32767.0f);
        float peak = sqrtf((float)s_peak_hold[channel] / 32767.0f);
        for (uint8_t segment = 0; segment < 10; segment++) {
            float threshold = (float)(segment + 1) / 10.0f;
            float y = meter_top + 61 - segment * 7;
            pax_col_t active = segment >= 9 ? COLOR_BAD :
                               segment >= 7 ? COLOR_ACCENT : COLOR_GOOD;
            pax_draw_rect(target, normalized >= threshold ? active : 0xff383939,
                          x + 2, y, meter_w - 4, 5);
        }
        float peak_y = meter_top + 67 - peak * 67;
        pax_draw_rect(target, COLOR_TEXT, x + 1, peak_y, meter_w - 2, 2);
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
        pax_draw_rect(&s_framebuffer, transport_color, timeline_x, timeline_y,
                      timeline_w * progress, 24);
        for (uint8_t bar = 1; bar < s_loop_bars; bar++) {
            float x = timeline_x + timeline_w * ((float)bar / s_loop_bars);
            pax_draw_rect(&s_framebuffer, COLOR_TEXT, x, timeline_y, 2, 24);
        }
        snprintf(line, sizeof(line), "BAR %u / %u",
                 s_loop_frames ? (unsigned)(progress * s_loop_bars) + 1 : 0,
                 s_loop_bars);
        text(40, 251, 14, COLOR_TEXT, line);
        snprintf(line, sizeof(line), "METRO %s", s_metronome ? "ON" : "OFF");
        text(164, 251, 14, s_metronome ? COLOR_ACCENT : COLOR_DIM, line);

        for (uint8_t track = 0; track < 4; track++) {
            float x = 34 + track * 145;
            float y = 296;
            float width = 128;
            float height = 98;
            pax_col_t color = track == s_selected_track ? COLOR_ACCENT :
                              s_track_frames[track] ? COLOR_DARK : COLOR_SHADOW;
            pax_draw_rect(&s_framebuffer, COLOR_SHADOW, x + 3, y + 4, width, height);
            pax_draw_rect(&s_framebuffer, COLOR_DARK, x, y, width, height);
            pax_draw_rect(&s_framebuffer, color, x, y, width, 5);
            pax_draw_rect(&s_framebuffer, 0xff3b3c3c, x + 10, y + 47, width - 20, 18);
            if (s_track_frames[track] > 0) {
                float phase = (float)s_track_positions[track] / s_track_frames[track];
                pax_draw_rect(&s_framebuffer, COLOR_ACCENT, x + 10, y + 47,
                              (width - 20) * phase, 18);
                for (uint8_t division = 1; division < 4; division++) {
                    pax_draw_rect(&s_framebuffer, COLOR_TEXT,
                                  x + 10 + (width - 20) * division / 4,
                                  y + 47, 1, 18);
                }
            }
            snprintf(line, sizeof(line), "%u", track + 1);
            text(x + 11, y + 12, 25, COLOR_TEXT, line);
            text(x + 43, y + 18, 12, color,
                 track == s_selected_track ? "SELECT" : "LOOP");
            snprintf(line, sizeof(line), "%u%%", s_track_volumes[track]);
            text(x + 10, y + 72, 13, COLOR_TEXT, line);
        }

        static const char *const input_names[] = {"1L", "1R", "2L", "2R", "AL", "AR"};
        const float meter_top = 306;
        const float meter_w = 22;
        for (uint8_t channel = 0; channel < 6; channel++) {
            float x = s_width - 154 + channel * (meter_w + 2);
            text(x, meter_top - 22, 11, COLOR_DIM, input_names[channel]);
            pax_draw_rect(&s_framebuffer, COLOR_DARK, x, meter_top, meter_w, 68);
            float normalized = sqrtf((float)s_display_levels[channel] / 32767.0f);
            float peak = sqrtf((float)s_peak_hold[channel] / 32767.0f);
            for (uint8_t segment = 0; segment < 10; segment++) {
                float threshold = (float)(segment + 1) / 10.0f;
                float y = meter_top + 61 - segment * 7;
                pax_col_t active = segment >= 9 ? COLOR_BAD :
                                   segment >= 7 ? COLOR_ACCENT : COLOR_GOOD;
                pax_draw_rect(&s_framebuffer,
                              normalized >= threshold ? active : 0xff383939,
                              x + 2, y, meter_w - 4, 5);
            }
            float peak_y = meter_top + 67 - peak * 67;
            pax_draw_rect(&s_framebuffer, COLOR_TEXT, x + 1, peak_y, meter_w - 2, 2);
        }
    }

    pax_draw_rect(&s_framebuffer, COLOR_DARK, 0, s_height - 55, s_width, 55);
    pax_draw_rect(&s_framebuffer, COLOR_ACCENT, 20, s_height - 55, 92, 55);
    text(34, s_height - 42, 15, COLOR_TEXT, "F2 REC");
    text(34, s_height - 23, 12, COLOR_TEXT, "/ OVERDUB");
    text(132, s_height - 38, 13, COLOR_TEXT,
        "↑↓ LOOP VOL   B+↑↓ BPM   ←→ BARS   1-4 TRACK   T TAP   M METRO   F1 EXIT");
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
    void *display_pixels[2] = {0};
    ESP_ERROR_CHECK(bsp_display_get_panel(&panel));
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(
        panel, 2, &display_pixels[0], &display_pixels[1]
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
            &s_framebuffers[index], display_pixels[index],
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
    vTaskPrioritySet(NULL, 5);

    uar_probe_snapshot_t previous = {0};
    char diagnostics[6][72] = {{0}};
    uint32_t diagnostic_revision = 0;
    uint32_t previous_diagnostic_revision = UINT32_MAX;
    int64_t previous_animation_us = 0;
    uar_probe_snapshot(&previous);
    uar_probe_diagnostics(diagnostics, 6, &diagnostic_revision);
    render(&previous, diagnostics);

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
        render(&previous, diagnostics);
    }

    while (true) {
        bsp_input_event_t event;
        while (xQueueReceive(s_input_queue, &event, 0) == pdTRUE) {
            if (event.type == INPUT_EVENT_TYPE_NAVIGATION &&
                event.args_navigation.state) {
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
                       (event.args_keyboard.ascii == 'm' ||
                        event.args_keyboard.ascii == 'M')) {
                uar_loop_toggle_metronome();
            } else if (event.type == INPUT_EVENT_TYPE_KEYBOARD &&
                       event.args_keyboard.ascii >= '1' &&
                       event.args_keyboard.ascii <= '4') {
                uar_loop_select_track((uint8_t)(event.args_keyboard.ascii - '1'));
            } else if (event.type == INPUT_EVENT_TYPE_KEYBOARD &&
                       (event.args_keyboard.ascii == 't' ||
                        event.args_keyboard.ascii == 'T')) {
                uar_loop_tap_tempo();
            }
        }
        if (usb_start_failed) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
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
        uar_loop_get_track_volumes(track_volumes);
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
        render_meters_fast();
        int64_t now_us = esp_timer_get_time();
        bool transport_animating =
            loop_state == UAR_LOOP_PLAYING || loop_state == UAR_LOOP_RECORDING ||
            record_armed;
        bool animation_due =
            transport_animating && now_us - previous_animation_us >= 33333;
        if (memcmp(&current, &previous, sizeof(current)) != 0 ||
            diagnostic_revision != previous_diagnostic_revision || loop_changed ||
            animation_due) {
            render(&current, diagnostics);
            previous = current;
            previous_diagnostic_revision = diagnostic_revision;
            previous_animation_us = now_us;
        }
        // The display driver already gates blits on the previous DMA transfer.
        // A short yield keeps input responsive and queues the next visual frame
        // for the earliest panel refresh instead of adding a second frame delay.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
