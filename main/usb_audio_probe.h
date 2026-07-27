#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define UAR_MAX_STREAMS 12

typedef struct {
    uint8_t interface_number;
    uint8_t alternate_setting;
    uint8_t endpoint_address;
    uint8_t channels;
    uint8_t subslot_bytes;
    uint8_t bit_resolution;
    uint16_t max_packet_size;
    uint8_t interval;
    uint8_t clock_source_id;
    bool device_to_host;
    bool asynchronous;
    bool has_feedback;
} uar_stream_info_t;

typedef struct {
    bool connected;
    bool high_speed;
    bool audio_class_2;
    uint16_t vid;
    uint16_t pid;
    uint8_t stream_count;
    uint8_t input_channels;
    uint8_t output_channels;
    uar_stream_info_t streams[UAR_MAX_STREAMS];
    char status[80];
} uar_probe_snapshot_t;

esp_err_t uar_probe_start(void);
void uar_probe_snapshot(uar_probe_snapshot_t *snapshot);
void uar_probe_diagnostics(char lines[][72], uint8_t max_lines, uint32_t *revision);
void uar_probe_read_levels(
    uint16_t levels[8], uint32_t *packet_count, uint32_t *error_count,
    uint64_t *audio_bytes, uint64_t *nonzero_bytes
);

typedef enum {
    UAR_LOOP_EMPTY = 0,
    UAR_LOOP_RECORDING,
    UAR_LOOP_PLAYING,
    UAR_LOOP_PAUSED,
} uar_loop_state_t;

void uar_loop_record_toggle(void);
void uar_loop_play_toggle(void);
void uar_loop_clear(void);
void uar_loop_get_state(uar_loop_state_t *state, uint32_t *frames, uint32_t *position);
void uar_loop_adjust_bpm(int delta);
void uar_loop_adjust_bars(int direction);
void uar_loop_get_settings(uint16_t *bpm, uint8_t *bars);
void uar_loop_select_next_track(void);
void uar_loop_toggle_metronome(void);
void uar_loop_get_tracks(
    uint8_t *selected, uint32_t frames[4], uint32_t positions[4], bool *metronome
);
void uar_loop_select_track(uint8_t track);
bool uar_loop_is_overdubbing(void);
void uar_loop_tap_tempo(void);
void uar_loop_adjust_track_volume(int delta);
void uar_loop_get_track_volumes(uint8_t volumes[4]);
