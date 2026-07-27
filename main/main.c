#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/input.h"
#include "bsp/power.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"
#include "usb_audio_probe.h"

#define COLOR_BG 0xff171a1c
#define COLOR_PANEL 0xff24292d
#define COLOR_TEXT 0xffeee9df
#define COLOR_DIM 0xff9ca7a8
#define COLOR_ACCENT 0xfff2c14e
#define COLOR_GOOD 0xff72c69c
#define COLOR_BAD 0xffdc7c73

static const char *TAG = "usb_recorder";
static pax_buf_t s_framebuffer;
static size_t s_width;
static size_t s_height;
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
static uint8_t s_track_volumes[4] = {100, 100, 100, 100};

static void text(float x, float y, float size, pax_col_t color, const char *value) {
    pax_draw_text(&s_framebuffer, color, pax_font_sky_mono, size, x, y, value);
}

static void render(const uar_probe_snapshot_t *probe, char diagnostics[][72]) {
    pax_background(&s_framebuffer, COLOR_BG);
    pax_draw_rect(&s_framebuffer, COLOR_PANEL, 22, 18, s_width - 44, 68);
    text(40, 34, 30, COLOR_ACCENT, "SIDEKICK SIDEKICK");
    text(s_width - 190, 45, 16, probe->connected ? COLOR_GOOD : COLOR_BAD,
         probe->connected ? "●  EP-136" : "○  NO DEVICE");

    if (!probe->connected) {
        text(150, 190, 28, COLOR_TEXT, "CONNECT EP-136 TO START");
        text(218, 238, 16, COLOR_DIM, "USB AUDIO  •  48 kHz");
    } else {
        char line[96];
        static const char *const states[] = {"READY", "RECORDING", "PLAYING", "PAUSED"};
        pax_col_t transport_color = s_loop_state == UAR_LOOP_RECORDING ? COLOR_BAD :
                                    s_loop_state == UAR_LOOP_PLAYING ? COLOR_GOOD :
                                    COLOR_ACCENT;
        text(38, 108, 18, COLOR_DIM, "BPM");
        snprintf(line, sizeof(line), "%u", s_loop_bpm);
        text(34, 128, 58, COLOR_TEXT, line);
        text(220, 108, 18, COLOR_DIM, "BARS");
        snprintf(line, sizeof(line), "%u", s_loop_bars);
        text(224, 128, 58, COLOR_TEXT, line);
        text(390, 108, 18, COLOR_DIM, "TRANSPORT");
        text(390, 142, 34, transport_color,
             s_overdubbing ? "OVERDUB" : states[s_loop_state]);

        float progress = s_loop_frames > 0 ?
                         (float)s_loop_position / (float)s_loop_frames : 0;
        if (s_loop_state == UAR_LOOP_RECORDING) {
            uint32_t target = ((uint32_t)s_loop_bars * 4U * 60U * 48000U) / s_loop_bpm;
            progress = target > 0 ? (float)s_loop_frames / (float)target : 0;
        }
        if (progress > 1) progress = 1;
        const float timeline_x = 34;
        const float timeline_y = 218;
        const float timeline_w = (float)s_width - 68;
        pax_draw_rect(&s_framebuffer, 0xff0d0f10, timeline_x, timeline_y, timeline_w, 32);
        pax_draw_rect(&s_framebuffer, transport_color, timeline_x, timeline_y,
                      timeline_w * progress, 32);
        for (uint8_t bar = 1; bar < s_loop_bars; bar++) {
            float x = timeline_x + timeline_w * ((float)bar / s_loop_bars);
            pax_draw_rect(&s_framebuffer, COLOR_TEXT, x, timeline_y, 2, 32);
        }
        snprintf(line, sizeof(line), "BAR %u / %u",
                 s_loop_frames ? (unsigned)(progress * s_loop_bars) + 1 : 0,
                 s_loop_bars);
        text(34, 266, 18, COLOR_TEXT, line);
        snprintf(line, sizeof(line), "METRO %s", s_metronome ? "ON" : "OFF");
        text(170, 266, 18, s_metronome ? COLOR_GOOD : COLOR_DIM, line);
        for (uint8_t track = 0; track < 4; track++) {
            float x = 380 + track * 82;
            float y = 284;
            pax_col_t color = track == s_selected_track ? COLOR_ACCENT :
                              s_track_frames[track] ? COLOR_GOOD : COLOR_DIM;
            for (uint8_t ring = 0; ring < (track == s_selected_track ? 4 : 2); ring++) {
                pax_draw_circle(&s_framebuffer, color, x, y, 27 + ring);
            }
            if (s_track_frames[track] > 0) {
                float phase = (float)s_track_positions[track] / s_track_frames[track];
                for (uint8_t ring = 0; ring < 4; ring++) {
                    pax_draw_arc(&s_framebuffer, COLOR_TEXT, x, y, 21 + ring,
                                 -1.5708f, -1.5708f + phase * 6.28318f);
                }
            }
            snprintf(line, sizeof(line), "%u", track + 1);
            text(x - 5, y - 9, 19, color, line);
            snprintf(line, sizeof(line), "%u%%", s_track_volumes[track]);
            text(x - 17, y + 35, 12, color, line);
        }

        static const char *const input_names[] = {"1L", "1R", "2L", "2R", "AL", "AR"};
        const float meter_top = 356;
        const float meter_w = ((float)s_width - 90) / 6;
        for (uint8_t channel = 0; channel < 6; channel++) {
            float x = 34 + channel * (meter_w + 4);
            text(x, meter_top - 24, 14, COLOR_DIM, input_names[channel]);
            pax_draw_rect(&s_framebuffer, 0xff0d0f10, x, meter_top, meter_w, 18);
            float normalized = (float)s_display_levels[channel] / 32767.0f;
            pax_col_t meter_color = normalized > 0.90f ? COLOR_BAD :
                                    normalized > 0.68f ? COLOR_ACCENT : COLOR_GOOD;
            pax_draw_rect(&s_framebuffer, meter_color, x, meter_top,
                          meter_w * normalized, 18);
        }
    }

    text(30, s_height - 27, 14, COLOR_DIM,
        "↑↓ TRACK VOL  B+↑↓ BPM  ←→ BARS  1-4 TRACK  F2 REC/OD  T TAP  M METRO");
    bsp_display_blit(0, 0, s_width, s_height, pax_buf_get_pixels(&s_framebuffer));
}

static void graphics_initialize(void) {
    bsp_display_color_format_t format;
    bsp_display_endianness_t endianness;
    ESP_ERROR_CHECK(bsp_display_get_parameters(&s_width, &s_height, &format, &endianness));
    pax_buf_type_t type = PAX_BUF_24_888RGB;
    if (format == BSP_DISPLAY_COLOR_FORMAT_16_565RGB) type = PAX_BUF_16_565RGB;
    if (format == BSP_DISPLAY_COLOR_FORMAT_32_8888ARGB) type = PAX_BUF_32_8888ARGB;
    pax_buf_init(&s_framebuffer, NULL, s_width, s_height, type);
    pax_buf_reversed(&s_framebuffer, endianness == BSP_DISPLAY_ENDIAN_BIG);
    pax_orientation_t orientation = PAX_O_UPRIGHT;
    switch (bsp_display_get_default_rotation()) {
        case BSP_DISPLAY_ROTATION_90: orientation = PAX_O_ROT_CCW; break;
        case BSP_DISPLAY_ROTATION_180: orientation = PAX_O_ROT_HALF; break;
        case BSP_DISPLAY_ROTATION_270: orientation = PAX_O_ROT_CW; break;
        default: break;
    }
    pax_buf_set_orientation(&s_framebuffer, orientation);
}

void app_main(void) {
    const bsp_configuration_t configuration = {
        .display = {
            .requested_color_format = BSP_DISPLAY_COLOR_FORMAT_24_888RGB,
            .num_fbs = 1,
        },
    };
    ESP_ERROR_CHECK(bsp_device_initialize(&configuration));
    graphics_initialize();
    ESP_ERROR_CHECK(bsp_input_get_queue(&s_input_queue));

    uar_probe_snapshot_t previous = {0};
    char diagnostics[6][72] = {{0}};
    uint32_t diagnostic_revision = 0;
    uint32_t previous_diagnostic_revision = UINT32_MAX;
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
        uint8_t track_volumes[4];
        uar_loop_get_track_volumes(track_volumes);
        uar_loop_get_state(&loop_state, &loop_frames, &loop_position);
        uar_loop_get_settings(&loop_bpm, &loop_bars);
        uar_loop_get_tracks(&selected_track, track_frames, track_positions, &metronome);
        bool loop_changed = loop_state != s_loop_state || loop_frames != s_loop_frames ||
                            loop_position != s_loop_position || loop_bpm != s_loop_bpm ||
                            loop_bars != s_loop_bars || selected_track != s_selected_track ||
                            metronome != s_metronome ||
                            overdubbing != s_overdubbing ||
                            memcmp(track_volumes, s_track_volumes,
                                   sizeof(track_volumes)) != 0 ||
                            memcmp(track_frames, s_track_frames, sizeof(track_frames)) != 0 ||
                            memcmp(track_positions, s_track_positions,
                                   sizeof(track_positions)) != 0;
        s_loop_state = loop_state;
        s_loop_frames = loop_frames;
        s_loop_position = loop_position;
        s_loop_bpm = loop_bpm;
        s_loop_bars = loop_bars;
        s_selected_track = selected_track;
        s_overdubbing = overdubbing;
        memcpy(s_track_volumes, track_volumes, sizeof(s_track_volumes));
        memcpy(s_track_frames, track_frames, sizeof(s_track_frames));
        memcpy(s_track_positions, track_positions, sizeof(s_track_positions));
        s_metronome = metronome;
        bool meter_changed = packet_count != s_audio_packets || error_count != s_audio_errors;
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
        if (memcmp(&current, &previous, sizeof(current)) != 0 ||
            diagnostic_revision != previous_diagnostic_revision || meter_changed ||
            loop_changed) {
            render(&current, diagnostics);
            previous = current;
            previous_diagnostic_revision = diagnostic_revision;
        }
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}
