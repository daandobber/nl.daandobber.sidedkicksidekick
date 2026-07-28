#include "usb_audio_probe.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "bsp/audio.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

#define USB_CLASS_AUDIO 0x01
#define USB_SUBCLASS_AUDIOCONTROL 0x01
#define USB_SUBCLASS_AUDIOSTREAMING 0x02
#define USB_DT_INTERFACE 0x04
#define USB_DT_ENDPOINT 0x05
#define USB_DT_CS_INTERFACE 0x24
#define USB_AS_GENERAL 0x01
#define USB_FORMAT_TYPE 0x02
#define USB_AC_INPUT_TERMINAL 0x02
#define USB_AC_OUTPUT_TERMINAL 0x03
#define USB_AC_CLOCK_SOURCE 0x0a
#define METER_CHANNELS 6
#define PHYSICAL_INPUT_FIRST_USB_CHANNEL 2
// High-speed audio has a service opportunity every 125 us. Keep a deep set
// of URBs queued so normal FreeRTOS scheduling cannot starve the endpoint.
#define ISO_TRANSFER_COUNT 24
#define ISO_PACKETS_PER_TRANSFER 32
#define MONITOR_FRAMES_PER_BLOCK 256
#define LOOP_MAX_SECONDS 30
#define LOOP_MAX_FRAMES (48000 * LOOP_MAX_SECONDS)

static const char *TAG = "uac_probe";
static usb_host_client_handle_t s_client;
static usb_device_handle_t s_device;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uar_probe_snapshot_t s_snapshot = {.status = "Waiting for USB audio device"};
static char s_diagnostics[6][72];
static uint8_t s_diagnostic_count;
static uint32_t s_diagnostic_revision;
static vprintf_like_t s_original_vprintf;
static usb_transfer_t *s_iso_transfers[ISO_TRANSFER_COUNT];
static volatile bool s_streaming;
static uint8_t s_stream_interface;
static uint8_t s_stream_alt;
static uint8_t s_stream_channels;
static uint8_t s_stream_subslot;
static uint8_t s_stream_resolution;
static uint16_t s_stream_packet_size;
static uint16_t s_meter_peaks[METER_CHANNELS];
static uint32_t s_packet_count;
static uint32_t s_error_count;
static uint64_t s_audio_bytes;
static uint64_t s_nonzero_bytes;
static uint8_t s_audio_control_interface;
static uint8_t s_clock_source_id;
static uint8_t s_clock_controls[256];
static uar_stream_info_t s_pending_stream;
static SemaphoreHandle_t s_control_done;
static QueueHandle_t s_iso_resubmit_queue;
static esp_err_t s_clock_result;
typedef struct {
    uint16_t frames;
    int16_t samples[MONITOR_FRAMES_PER_BLOCK * 2];
} monitor_block_t;
static QueueHandle_t s_monitor_queue;
static i2s_chan_handle_t s_i2s;
static int16_t *s_track_samples[4];
static volatile uar_loop_state_t s_loop_state = UAR_LOOP_EMPTY;
static volatile uint32_t s_track_frames[4];
static volatile uint32_t s_track_position[4];
static volatile uint8_t s_selected_track;
static volatile bool s_metronome;
static uint64_t s_transport_frame;
static volatile bool s_overdubbing;
static volatile bool s_record_armed;
static volatile bool s_pending_overdub;
static volatile uint32_t s_overdub_frames;
static volatile uint8_t s_track_volume[4] = {100, 100, 100, 100};
static volatile uint8_t s_monitor_volume = 55;
static int64_t s_last_tap_us;
static uint32_t s_tap_intervals[3];
static uint8_t s_tap_count;
static uint8_t s_tap_index;
static volatile uint16_t s_loop_bpm = 120;
static volatile uint8_t s_loop_bars = 4;

static void clock_control_done(usb_transfer_t *transfer);
static esp_err_t set_stream_interface_alt(
    usb_device_handle_t device, uint8_t interface_number, uint8_t alternate_setting
);

static const char *transfer_status_name(usb_transfer_status_t status) {
    switch (status) {
        case USB_TRANSFER_STATUS_COMPLETED: return "COMPLETED";
        case USB_TRANSFER_STATUS_ERROR: return "ERROR/CRC";
        case USB_TRANSFER_STATUS_TIMED_OUT: return "TIMEOUT";
        case USB_TRANSFER_STATUS_CANCELED: return "CANCELED";
        case USB_TRANSFER_STATUS_STALL: return "STALL";
        case USB_TRANSFER_STATUS_OVERFLOW: return "OVERFLOW";
        case USB_TRANSFER_STATUS_SKIPPED: return "SKIPPED";
        case USB_TRANSFER_STATUS_NO_DEVICE: return "NO DEVICE";
        default: return "UNKNOWN";
    }
}

static void append_diagnostic(const char *formatted) {
    char clean[72];
    size_t output = 0;
    bool escape = false;
    for (size_t i = 0; formatted[i] != 0 && output + 1 < sizeof(clean); i++) {
        unsigned char c = (unsigned char)formatted[i];
        if (escape) {
            if (c >= '@' && c <= '~') escape = false;
            continue;
        }
        if (c == 0x1b) {
            escape = true;
            continue;
        }
        if (c == '\r' || c == '\n') continue;
        if (isprint(c)) clean[output++] = (char)c;
    }
    clean[output] = 0;
    if (output == 0) return;

    taskENTER_CRITICAL(&s_lock);
    if (s_diagnostic_count < 6) {
        memcpy(s_diagnostics[s_diagnostic_count++], clean, output + 1);
    } else {
        memmove(s_diagnostics[0], s_diagnostics[1], sizeof(s_diagnostics[0]) * 5);
        memcpy(s_diagnostics[5], clean, output + 1);
    }
    s_diagnostic_revision++;
    taskEXIT_CRITICAL(&s_lock);
}

static int diagnostic_vprintf(const char *format, va_list arguments) {
    char formatted[192];
    va_list copy;
    va_copy(copy, arguments);
    int result = vsnprintf(formatted, sizeof(formatted), format, copy);
    va_end(copy);
    append_diagnostic(formatted);
    return s_original_vprintf ? s_original_vprintf(format, arguments) : result;
}

static uint16_t le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void publish(const uar_probe_snapshot_t *next) {
    taskENTER_CRITICAL(&s_lock);
    s_snapshot = *next;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_probe_snapshot(uar_probe_snapshot_t *snapshot) {
    if (snapshot == NULL) return;
    taskENTER_CRITICAL(&s_lock);
    *snapshot = s_snapshot;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_probe_diagnostics(char lines[][72], uint8_t max_lines, uint32_t *revision) {
    if (lines == NULL || max_lines == 0) return;
    taskENTER_CRITICAL(&s_lock);
    uint8_t count = s_diagnostic_count < max_lines ? s_diagnostic_count : max_lines;
    uint8_t first = s_diagnostic_count - count;
    for (uint8_t i = 0; i < count; i++) memcpy(lines[i], s_diagnostics[first + i], 72);
    for (uint8_t i = count; i < max_lines; i++) lines[i][0] = 0;
    if (revision != NULL) *revision = s_diagnostic_revision;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_probe_read_levels(
    uint16_t levels[8], uint32_t *packet_count, uint32_t *error_count,
    uint64_t *audio_bytes, uint64_t *nonzero_bytes
) {
    if (levels == NULL) return;
    taskENTER_CRITICAL(&s_lock);
    memset(levels, 0, sizeof(uint16_t) * 8);
    memcpy(levels, s_meter_peaks, sizeof(s_meter_peaks));
    memset(s_meter_peaks, 0, sizeof(s_meter_peaks));
    if (packet_count != NULL) *packet_count = s_packet_count;
    if (error_count != NULL) *error_count = s_error_count;
    if (audio_bytes != NULL) *audio_bytes = s_audio_bytes;
    if (nonzero_bytes != NULL) *nonzero_bytes = s_nonzero_bytes;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_record_toggle(void) {
    taskENTER_CRITICAL(&s_lock);
    if (s_track_samples[s_selected_track] == NULL) {
        taskEXIT_CRITICAL(&s_lock);
        return;
    }
    if (s_record_armed) {
        s_record_armed = false;
        s_pending_overdub = false;
    } else if (s_loop_state == UAR_LOOP_RECORDING) {
        s_track_position[s_selected_track] = 0;
        s_loop_state = s_track_frames[s_selected_track] > 0 ?
                       UAR_LOOP_PLAYING : UAR_LOOP_EMPTY;
        s_overdubbing = false;
    } else {
        s_pending_overdub = s_track_frames[s_selected_track] > 0;
        s_record_armed = true;
    }
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_play_toggle(void) {
    taskENTER_CRITICAL(&s_lock);
    bool any = false;
    for (uint8_t i = 0; i < 4; i++) any |= s_track_frames[i] > 0;
    if (any) {
        s_loop_state = s_loop_state == UAR_LOOP_PLAYING ?
                       UAR_LOOP_PAUSED : UAR_LOOP_PLAYING;
    }
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_clear(void) {
    taskENTER_CRITICAL(&s_lock);
    s_record_armed = false;
    s_pending_overdub = false;
    s_track_frames[s_selected_track] = 0;
    s_track_position[s_selected_track] = 0;
    bool any = false;
    for (uint8_t i = 0; i < 4; i++) any |= s_track_frames[i] > 0;
    s_loop_state = any ? UAR_LOOP_PLAYING : UAR_LOOP_EMPTY;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_get_state(uar_loop_state_t *state, uint32_t *frames, uint32_t *position) {
    taskENTER_CRITICAL(&s_lock);
    if (state != NULL) *state = s_loop_state;
    if (frames != NULL) *frames = s_track_frames[s_selected_track];
    if (position != NULL) *position = s_track_position[s_selected_track];
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_select_next_track(void) {
    taskENTER_CRITICAL(&s_lock);
    if (s_loop_state != UAR_LOOP_RECORDING && !s_record_armed) {
        s_selected_track = (s_selected_track + 1) % 4;
    }
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_select_track(uint8_t track) {
    if (track >= 4) return;
    taskENTER_CRITICAL(&s_lock);
    if (s_loop_state != UAR_LOOP_RECORDING && !s_record_armed) s_selected_track = track;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_adjust_track_volume(int delta) {
    int volume = (int)s_track_volume[s_selected_track] + delta;
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    s_track_volume[s_selected_track] = volume;
}

void uar_loop_get_track_volumes(uint8_t volumes[4]) {
    if (volumes == NULL) return;
    taskENTER_CRITICAL(&s_lock);
    memcpy(volumes, (const void *)s_track_volume, sizeof(s_track_volume));
    taskEXIT_CRITICAL(&s_lock);
}

void uar_monitor_adjust_volume(int delta) {
    int volume = (int)s_monitor_volume + delta;
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    if (bsp_audio_set_volume((float)volume) == ESP_OK) {
        s_monitor_volume = (uint8_t)volume;
    }
}

void uar_loop_tap_tempo(void) {
    int64_t now = esp_timer_get_time();
    if (s_last_tap_us == 0 || now - s_last_tap_us > 2000000) {
        s_tap_count = 0;
        s_tap_index = 0;
        s_last_tap_us = now;
        return;
    }
    uint32_t interval = (uint32_t)(now - s_last_tap_us);
    s_last_tap_us = now;
    s_tap_intervals[s_tap_index] = interval;
    s_tap_index = (s_tap_index + 1) % 3;
    if (s_tap_count < 3) s_tap_count++;
    uint64_t total = 0;
    for (uint8_t i = 0; i < s_tap_count; i++) total += s_tap_intervals[i];
    uint32_t average = (uint32_t)(total / s_tap_count);
    if (average > 0) {
        uint32_t bpm = 60000000U / average;
        if (bpm < 40) bpm = 40;
        if (bpm > 240) bpm = 240;
        s_loop_bpm = bpm;
    }
}

bool uar_loop_is_overdubbing(void) {
    return s_overdubbing;
}

bool uar_loop_is_record_armed(void) {
    return s_record_armed;
}

void uar_loop_toggle_metronome(void) {
    taskENTER_CRITICAL(&s_lock);
    s_metronome = !s_metronome;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_get_tracks(
    uint8_t *selected, uint32_t frames[4], uint32_t positions[4], bool *metronome
) {
    taskENTER_CRITICAL(&s_lock);
    if (selected != NULL) *selected = s_selected_track;
    if (frames != NULL) memcpy(frames, (const void *)s_track_frames, sizeof(s_track_frames));
    if (positions != NULL) {
        memcpy(positions, (const void *)s_track_position, sizeof(s_track_position));
    }
    if (metronome != NULL) *metronome = s_metronome;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_adjust_bpm(int delta) {
    taskENTER_CRITICAL(&s_lock);
    int next = (int)s_loop_bpm + delta;
    if (next < 40) next = 40;
    if (next > 240) next = 240;
    s_loop_bpm = (uint16_t)next;
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_adjust_bars(int direction) {
    static const uint8_t choices[] = {1, 2, 4, 8, 16};
    taskENTER_CRITICAL(&s_lock);
    uint8_t index = 0;
    while (index < 4 && choices[index] != s_loop_bars) index++;
    if (direction > 0 && index < 4) index++;
    if (direction < 0 && index > 0) index--;
    s_loop_bars = choices[index];
    taskEXIT_CRITICAL(&s_lock);
}

void uar_loop_get_settings(uint16_t *bpm, uint8_t *bars) {
    taskENTER_CRITICAL(&s_lock);
    if (bpm != NULL) *bpm = s_loop_bpm;
    if (bars != NULL) *bars = s_loop_bars;
    taskEXIT_CRITICAL(&s_lock);
}

static int32_t read_sample(const uint8_t *sample, uint8_t bytes, uint8_t resolution) {
    if (bytes == 3) {
        int32_t value = (int32_t)sample[0] | ((int32_t)sample[1] << 8) |
                        ((int32_t)sample[2] << 16);
        if (value & 0x00800000) value |= (int32_t)0xff000000;
        return value >> 8;
    }
    if (bytes == 4) {
        int32_t value = (int32_t)sample[0] | ((int32_t)sample[1] << 8) |
                        ((int32_t)sample[2] << 16) | ((int32_t)sample[3] << 24);
        if (resolution == 24) {
            // UAC2 permits 24 valid bits in a four-byte subslot. Most devices
            // (including XMOS-style interfaces) sign-extend the 24-bit value
            // into byte 3 instead of left-aligning it.
            bool sign_extended = sample[3] == ((sample[2] & 0x80) ? 0xff : 0x00);
            return sign_extended ? value >> 8 : value >> 16;
        }
        return value >> 16;
    }
    if (bytes == 2) return (int16_t)((uint16_t)sample[0] | ((uint16_t)sample[1] << 8));
    return 0;
}

static void process_iso_transfer(usb_transfer_t *transfer) {
    size_t offset = 0;
    uint16_t peaks[METER_CHANNELS] = {0};
    uint64_t received_bytes = 0;
    uint64_t active_bytes = 0;
    uint32_t completed_packets = 0;
    uint32_t failed_packets = 0;
    usb_transfer_status_t first_failed_status = USB_TRANSFER_STATUS_COMPLETED;
    monitor_block_t monitor = {0};
    for (int packet = 0; packet < transfer->num_isoc_packets; packet++) {
        usb_isoc_packet_desc_t *descriptor = &transfer->isoc_packet_desc[packet];
        if (descriptor->status == USB_TRANSFER_STATUS_COMPLETED) {
            completed_packets++;
        } else {
            if (failed_packets == 0) first_failed_status = descriptor->status;
            failed_packets++;
        }
        if (descriptor->status == USB_TRANSFER_STATUS_COMPLETED &&
            descriptor->actual_num_bytes > 0) {
            size_t frame_bytes = (size_t)s_stream_channels * s_stream_subslot;
            size_t packet_bytes = (size_t)descriptor->actual_num_bytes;
            received_bytes += packet_bytes;
            for (size_t byte = 0; byte < packet_bytes; byte++) {
                if (transfer->data_buffer[offset + byte] != 0) active_bytes++;
            }
            for (size_t frame = 0; frame + frame_bytes <= packet_bytes; frame += frame_bytes) {
                const uint8_t *samples = transfer->data_buffer + offset + frame;
                // EP-136 USB 1/2 is the internal Main Mix. Physical sockets are
                // USB 3/4 (CH1), 5/6 (CH2), and 7/8 (AUX).
                for (uint8_t channel = 0; channel < METER_CHANNELS; channel++) {
                    uint8_t usb_channel = channel + PHYSICAL_INPUT_FIRST_USB_CHANNEL;
                    if (usb_channel >= s_stream_channels) break;
                    int32_t value = read_sample(
                        samples + usb_channel * s_stream_subslot,
                        s_stream_subslot, s_stream_resolution
                    );
                    uint32_t magnitude = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
                    if (magnitude > 32767) magnitude = 32767;
                    if (magnitude > peaks[channel]) peaks[channel] = (uint16_t)magnitude;
                }
                if (s_stream_channels >= 8 && monitor.frames < MONITOR_FRAMES_PER_BLOCK) {
                    int32_t left = 0;
                    int32_t right = 0;
                    for (uint8_t pair = 0; pair < 3; pair++) {
                        left += read_sample(samples + (2 + pair * 2) * s_stream_subslot,
                                            s_stream_subslot, s_stream_resolution);
                        right += read_sample(samples + (3 + pair * 2) * s_stream_subslot,
                                             s_stream_subslot, s_stream_resolution);
                    }
                    left /= 3;
                    right /= 3;
                    if (left > 32767) left = 32767;
                    if (left < -32768) left = -32768;
                    if (right > 32767) right = 32767;
                    if (right < -32768) right = -32768;
                    monitor.samples[monitor.frames * 2] = (int16_t)left;
                    monitor.samples[monitor.frames * 2 + 1] = (int16_t)right;
                    monitor.frames++;
                }
            }
        }
        offset += (size_t)descriptor->num_bytes;
    }
    taskENTER_CRITICAL(&s_lock);
    for (uint8_t channel = 0; channel < METER_CHANNELS; channel++) {
        if (peaks[channel] > s_meter_peaks[channel]) s_meter_peaks[channel] = peaks[channel];
    }
    s_packet_count += completed_packets;
    s_error_count += failed_packets;
    s_audio_bytes += received_bytes;
    s_nonzero_bytes += active_bytes;
    if (failed_packets > 0) {
        snprintf(s_snapshot.status, sizeof(s_snapshot.status),
                 "EP82 %s • clk%u %s • ok%lu bad%lu",
                 transfer_status_name(first_failed_status),
                 s_clock_source_id, esp_err_to_name(s_clock_result),
                 (unsigned long)completed_packets, (unsigned long)failed_packets);
    }
    taskEXIT_CRITICAL(&s_lock);
    if (monitor.frames > 0 && s_monitor_queue != NULL) {
        if (xQueueSend(s_monitor_queue, &monitor, 0) != pdTRUE) {
            monitor_block_t discarded;
            xQueueReceive(s_monitor_queue, &discarded, 0);
            xQueueSend(s_monitor_queue, &monitor, 0);
        }
    }
}

static void iso_transfer_done(usb_transfer_t *transfer) {
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        process_iso_transfer(transfer);
    } else {
        // Stop on the first transport failure. Resubmitting a permanently
        // failing URB only hides the useful error in a rapidly rising counter.
        s_streaming = false;
        taskENTER_CRITICAL(&s_lock);
        s_error_count++;
        snprintf(s_snapshot.status, sizeof(s_snapshot.status),
                 "EP82 ISO %s • clock %s",
                 transfer_status_name(transfer->status),
                 esp_err_to_name(s_clock_result));
        taskEXIT_CRITICAL(&s_lock);
        ESP_LOGE(TAG, "Isochronous transfer failed: %s (%d)",
                 transfer_status_name(transfer->status), transfer->status);
    }
    if (s_streaming) {
        if (xQueueSend(s_iso_resubmit_queue, &transfer, 0) != pdTRUE) {
            s_streaming = false;
            taskENTER_CRITICAL(&s_lock);
            s_error_count++;
            taskEXIT_CRITICAL(&s_lock);
            ESP_LOGE(TAG, "Isochronous resubmit queue full");
        }
    }
}

static void iso_resubmit_task(void *argument) {
    (void)argument;
    usb_transfer_t *transfer;
    while (true) {
        if (xQueueReceive(s_iso_resubmit_queue, &transfer, portMAX_DELAY) != pdTRUE) continue;
        if (!s_streaming) continue;
        esp_err_t error = usb_host_transfer_submit(transfer);
        if (error != ESP_OK) {
            s_streaming = false;
            taskENTER_CRITICAL(&s_lock);
            s_error_count++;
            taskEXIT_CRITICAL(&s_lock);
            ESP_LOGE(TAG, "Isochronous resubmit failed: %s", esp_err_to_name(error));
        }
    }
}

static esp_err_t start_meter_stream(
    usb_device_handle_t device, const uar_stream_info_t *stream
) {
    if (stream == NULL || stream->channels == 0 || stream->subslot_bytes < 2 ||
        stream->subslot_bytes > 4 || !stream->device_to_host) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    uint16_t packet_size = stream->max_packet_size & 0x07ff;
    uint8_t transactions = 1 + ((stream->max_packet_size >> 11) & 0x03);
    packet_size = (uint16_t)(packet_size * transactions);
    if (packet_size == 0) return ESP_ERR_INVALID_SIZE;

    esp_err_t error = usb_host_interface_claim(
        s_client, device, stream->interface_number, stream->alternate_setting
    );
    if (error != ESP_OK) return error;

    // Claiming selects the endpoint descriptors inside ESP-IDF, but does not
    // send USB SET_INTERFACE to the physical device. Without this request the
    // EP-136 remains in alt 0 (zero-bandwidth) and EP82 never answers.
    error = set_stream_interface_alt(
        device, stream->interface_number, stream->alternate_setting
    );
    if (error != ESP_OK) {
        usb_host_interface_release(s_client, device, stream->interface_number);
        return error;
    }

    s_stream_interface = stream->interface_number;
    s_stream_alt = stream->alternate_setting;
    s_stream_channels = stream->channels;
    s_stream_subslot = stream->subslot_bytes;
    s_stream_resolution = stream->bit_resolution;
    s_stream_packet_size = packet_size;
    s_streaming = true;
    memset(s_meter_peaks, 0, sizeof(s_meter_peaks));
    s_packet_count = s_error_count = 0;
    s_audio_bytes = s_nonzero_bytes = 0;

    for (int i = 0; i < ISO_TRANSFER_COUNT; i++) {
        error = usb_host_transfer_alloc(
            (size_t)packet_size * ISO_PACKETS_PER_TRANSFER,
            ISO_PACKETS_PER_TRANSFER, &s_iso_transfers[i]
        );
        if (error != ESP_OK) {
            s_streaming = false;
            return error;
        }
        usb_transfer_t *transfer = s_iso_transfers[i];
        transfer->device_handle = device;
        transfer->bEndpointAddress = stream->endpoint_address;
        transfer->callback = iso_transfer_done;
        transfer->context = NULL;
        transfer->num_bytes = packet_size * ISO_PACKETS_PER_TRANSFER;
        for (int packet = 0; packet < ISO_PACKETS_PER_TRANSFER; packet++) {
            transfer->isoc_packet_desc[packet].num_bytes = packet_size;
        }
        error = usb_host_transfer_submit(transfer);
        if (error != ESP_OK) {
            s_streaming = false;
            return error;
        }
    }
    ESP_LOGI(TAG, "Meter stream started: %uch, %u-byte samples, packet %u, endpoint %02x",
             s_stream_channels, s_stream_subslot, s_stream_packet_size,
             stream->endpoint_address);
    return ESP_OK;
}

static void clock_control_done(usb_transfer_t *transfer) {
    (void)transfer;
    if (s_control_done != NULL) xSemaphoreGive(s_control_done);
}

static esp_err_t set_stream_interface_alt(
    usb_device_handle_t device, uint8_t interface_number, uint8_t alternate_setting
) {
    usb_transfer_t *transfer = NULL;
    esp_err_t error = usb_host_transfer_alloc(sizeof(usb_setup_packet_t), 0, &transfer);
    if (error != ESP_OK) return error;

    usb_setup_packet_t *setup = (usb_setup_packet_t *)transfer->data_buffer;
    setup->bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT |
                           USB_BM_REQUEST_TYPE_TYPE_STANDARD |
                           USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
    setup->bRequest = USB_B_REQUEST_SET_INTERFACE;
    setup->wValue = alternate_setting;
    setup->wIndex = interface_number;
    setup->wLength = 0;
    transfer->device_handle = device;
    transfer->bEndpointAddress = 0;
    transfer->num_bytes = sizeof(usb_setup_packet_t);
    transfer->callback = clock_control_done;

    xSemaphoreTake(s_control_done, 0);
    error = usb_host_transfer_submit_control(s_client, transfer);
    if (error == ESP_OK) {
        if (xSemaphoreTake(s_control_done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            error = ESP_ERR_TIMEOUT;
        } else if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
            error = ESP_ERR_INVALID_RESPONSE;
        }
    }
    ESP_LOGI(TAG, "SET_INTERFACE if%u alt%u: %s",
             interface_number, alternate_setting, esp_err_to_name(error));
    usb_host_transfer_free(transfer);
    return error;
}

static void stream_start_task(void *argument) {
    usb_device_handle_t device = (usb_device_handle_t)argument;
    // EP-136 runs a fixed 48 kHz device clock. The stream starts after
    // SET_INTERFACE; clock entity requests are unnecessary and are rejected
    // by this firmware revision.
    s_clock_result = ESP_OK;
    esp_err_t error = start_meter_stream(device, &s_pending_stream);
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!s_streaming) {
            vTaskDelete(NULL);
            return;
        }
    }
    uar_probe_snapshot_t result;
    uar_probe_snapshot(&result);
    snprintf(result.status, sizeof(result.status),
             "UAC%s %uch IN • if%u alt%u ep%02x %uB i%u • %s",
             result.audio_class_2 ? "2" : "1",
             s_pending_stream.channels, s_pending_stream.interface_number,
             s_pending_stream.alternate_setting, s_pending_stream.endpoint_address,
             s_stream_packet_size, s_pending_stream.interval,
             error == ESP_OK ? "LIVE" : "stream failed");
    publish(&result);
    vTaskDelete(NULL);
}

static void log_descriptor(const uint8_t *p, size_t length) {
    char line[3 * 32 + 1];
    size_t position = 0;
    for (size_t i = 0; i < length && i < 32; i++) {
        int written = snprintf(line + position, sizeof(line) - position, "%02x ", p[i]);
        if (written <= 0) break;
        position += (size_t)written;
    }
    ESP_LOGI(TAG, "  raw: %s", line);
}

static void add_stream(
    uar_probe_snapshot_t *result, uint8_t interface_number, uint8_t alternate_setting,
    uint8_t endpoint, uint8_t attributes, uint16_t max_packet, uint8_t interval,
    uint8_t channels, uint8_t subslot, uint8_t resolution, uint8_t clock_source,
    bool feedback
) {
    if (result->stream_count >= UAR_MAX_STREAMS) return;
    uar_stream_info_t *stream = &result->streams[result->stream_count++];
    *stream = (uar_stream_info_t){
        .interface_number = interface_number,
        .alternate_setting = alternate_setting,
        .endpoint_address = endpoint,
        .channels = channels,
        .subslot_bytes = subslot,
        .bit_resolution = resolution,
        .max_packet_size = max_packet,
        .interval = interval,
        .clock_source_id = clock_source,
        .device_to_host = (endpoint & 0x80) != 0,
        .asynchronous = ((attributes >> 2) & 0x03) == 1,
        .has_feedback = feedback,
    };
    if (stream->device_to_host && channels > result->input_channels) {
        result->input_channels = channels;
    }
    if (!stream->device_to_host && channels > result->output_channels) {
        result->output_channels = channels;
    }
}

static bool parse_audio_configuration(
    const usb_config_desc_t *configuration, uar_probe_snapshot_t *result
) {
    const uint8_t *bytes = (const uint8_t *)configuration;
    size_t total = configuration->wTotalLength;
    size_t offset = 0;
    uint8_t interface_number = 0;
    uint8_t alternate_setting = 0;
    uint8_t interface_class = 0;
    uint8_t interface_subclass = 0;
    uint8_t channels = 0;
    uint8_t subslot = 0;
    uint8_t resolution = 0;
    bool found_audio = false;
    bool feedback_in_alt = false;
    uint8_t stream_clock_source = 0;
    uint8_t clock_for_terminal[256] = {0};
    s_audio_control_interface = 0;
    s_clock_source_id = 0;
    memset(s_clock_controls, 0, sizeof(s_clock_controls));

    ESP_LOGI(TAG, "Configuration: %u interfaces, %u bytes",
             configuration->bNumInterfaces, configuration->wTotalLength);

    while (offset + 2 <= total) {
        const uint8_t *descriptor = bytes + offset;
        uint8_t length = descriptor[0];
        uint8_t type = descriptor[1];
        if (length < 2 || offset + length > total) {
            ESP_LOGW(TAG, "Malformed descriptor at offset %u", (unsigned)offset);
            break;
        }

        if (type == USB_DT_INTERFACE && length >= 9) {
            interface_number = descriptor[2];
            alternate_setting = descriptor[3];
            interface_class = descriptor[5];
            interface_subclass = descriptor[6];
            channels = subslot = resolution = 0;
            stream_clock_source = 0;
            feedback_in_alt = false;
            ESP_LOGI(TAG, "Interface %u alt %u class %02x subclass %02x protocol %02x endpoints %u",
                     interface_number, alternate_setting, interface_class,
                     interface_subclass, descriptor[7], descriptor[4]);
            if (interface_class == USB_CLASS_AUDIO) {
                found_audio = true;
                if (descriptor[7] == 0x20) result->audio_class_2 = true;
                if (interface_subclass == USB_SUBCLASS_AUDIOCONTROL) {
                    s_audio_control_interface = interface_number;
                }
            }
        } else if (type == USB_DT_CS_INTERFACE && interface_class == USB_CLASS_AUDIO) {
            ESP_LOGI(TAG, "Class descriptor intf %u alt %u subtype %02x len %u",
                     interface_number, alternate_setting,
                     length >= 3 ? descriptor[2] : 0xff, length);
            log_descriptor(descriptor, length);
            if (interface_subclass == USB_SUBCLASS_AUDIOCONTROL &&
                result->audio_class_2 && descriptor[2] == USB_AC_CLOCK_SOURCE &&
                length >= 8 && s_clock_source_id == 0) {
                s_clock_source_id = descriptor[3];
                s_clock_controls[descriptor[3]] = descriptor[5];
                ESP_LOGI(TAG, "UAC2 clock source %u controls %02x",
                         s_clock_source_id, descriptor[5]);
            } else if (interface_subclass == USB_SUBCLASS_AUDIOCONTROL &&
                       result->audio_class_2 && descriptor[2] == USB_AC_CLOCK_SOURCE &&
                       length >= 8) {
                s_clock_controls[descriptor[3]] = descriptor[5];
                ESP_LOGI(TAG, "UAC2 clock source %u controls %02x",
                         descriptor[3], descriptor[5]);
            }
            if (interface_subclass == USB_SUBCLASS_AUDIOCONTROL &&
                result->audio_class_2 && descriptor[2] == USB_AC_INPUT_TERMINAL &&
                length >= 8) {
                clock_for_terminal[descriptor[3]] = descriptor[7];
                ESP_LOGI(TAG, "UAC2 input terminal %u uses clock %u",
                         descriptor[3], descriptor[7]);
            }
            if (interface_subclass == USB_SUBCLASS_AUDIOCONTROL &&
                result->audio_class_2 && descriptor[2] == USB_AC_OUTPUT_TERMINAL &&
                length >= 9) {
                clock_for_terminal[descriptor[3]] = descriptor[8];
                ESP_LOGI(TAG, "UAC2 output terminal %u uses clock %u",
                         descriptor[3], descriptor[8]);
            }
            if (interface_subclass == USB_SUBCLASS_AUDIOSTREAMING && length >= 3) {
                if (descriptor[2] == USB_AS_GENERAL && length >= 11) {
                    // UAC2 AS_GENERAL has bNrChannels at byte 10.
                    channels = descriptor[10];
                    stream_clock_source = clock_for_terminal[descriptor[3]];
                    ESP_LOGI(TAG, "Stream terminal %u uses clock %u",
                             descriptor[3], stream_clock_source);
                } else if (descriptor[2] == USB_FORMAT_TYPE) {
                    if (result->audio_class_2 && length >= 6) {
                        // UAC2 FORMAT_TYPE_I.
                        subslot = descriptor[4];
                        resolution = descriptor[5];
                    } else if (length >= 8) {
                        // UAC1 FORMAT_TYPE_I carries bNrChannels at byte 4.
                        channels = descriptor[4];
                        subslot = descriptor[5];
                        resolution = descriptor[6];
                    }
                }
            }
        } else if (type == USB_DT_ENDPOINT && length >= 7) {
            uint8_t endpoint = descriptor[2];
            uint8_t attributes = descriptor[3];
            uint8_t usage = (attributes >> 4) & 0x03;
            if ((attributes & 0x03) == 1 && usage == 1) {
                feedback_in_alt = true;
                if (result->stream_count > 0) {
                    uar_stream_info_t *previous = &result->streams[result->stream_count - 1];
                    if (previous->interface_number == interface_number &&
                        previous->alternate_setting == alternate_setting) {
                        previous->has_feedback = true;
                    }
                }
            }
            ESP_LOGI(TAG, "Endpoint %02x attr %02x max-packet %u interval %u%s",
                     endpoint, attributes, le16(descriptor + 4), descriptor[6],
                     usage == 1 ? " feedback" : "");
            if (interface_class == USB_CLASS_AUDIO &&
                interface_subclass == USB_SUBCLASS_AUDIOSTREAMING &&
                alternate_setting != 0 && (attributes & 0x03) == 1 && usage != 1) {
                add_stream(result, interface_number, alternate_setting, endpoint,
                           attributes, le16(descriptor + 4), descriptor[6],
                           channels, subslot, resolution, stream_clock_source,
                           feedback_in_alt);
            }
        } else {
            ESP_LOGI(TAG, "Descriptor type %02x len %u", type, length);
        }
        offset += length;
    }
    return found_audio;
}

static void handle_new_device(uint8_t address) {
    uar_probe_snapshot_t progress = {0};
    snprintf(progress.status, sizeof(progress.status), "USB device %u detected; opening...", address);
    publish(&progress);

    if (s_device != NULL) {
        ESP_LOGW(TAG, "Ignoring additional USB device");
        snprintf(progress.status, sizeof(progress.status), "Extra USB device ignored");
        publish(&progress);
        return;
    }
    usb_device_handle_t device = NULL;
    esp_err_t error = usb_host_device_open(s_client, address, &device);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Cannot open USB device: %s", esp_err_to_name(error));
        snprintf(progress.status, sizeof(progress.status), "USB open failed: %s",
                 esp_err_to_name(error));
        publish(&progress);
        return;
    }

    uar_probe_snapshot_t result = {0};
    const usb_device_desc_t *device_descriptor = NULL;
    const usb_config_desc_t *configuration = NULL;
    usb_device_info_t info = {0};
    if (usb_host_get_device_descriptor(device, &device_descriptor) == ESP_OK) {
        result.vid = device_descriptor->idVendor;
        result.pid = device_descriptor->idProduct;
        ESP_LOGI(TAG, "Device VID:PID %04x:%04x USB %x.%02x",
                 result.vid, result.pid, device_descriptor->bcdUSB >> 8,
                 device_descriptor->bcdUSB & 0xff);
    } else {
        snprintf(result.status, sizeof(result.status), "Device descriptor read failed");
        publish(&result);
        usb_host_device_close(s_client, device);
        return;
    }
    if (usb_host_device_info(device, &info) == ESP_OK) {
        result.high_speed = info.speed == USB_SPEED_HIGH;
    }
    error = usb_host_get_active_config_descriptor(device, &configuration);
    if (error != ESP_OK) {
        snprintf(result.status, sizeof(result.status), "Config read failed: %s (%04x:%04x)",
                 esp_err_to_name(error), result.vid, result.pid);
        publish(&result);
        usb_host_device_close(s_client, device);
        return;
    }
    snprintf(result.status, sizeof(result.status), "Parsing %u-byte config (%04x:%04x)",
             configuration->wTotalLength, result.vid, result.pid);
    publish(&result);
    if (!parse_audio_configuration(configuration, &result)) {
        snprintf(result.status, sizeof(result.status), "Not a USB audio device (%04x:%04x)",
                 result.vid, result.pid);
        publish(&result);
        usb_host_device_close(s_client, device);
        return;
    }

    result.connected = true;
    const uar_stream_info_t *best_input = NULL;
    for (uint8_t i = 0; i < result.stream_count; i++) {
        const uar_stream_info_t *candidate = &result.streams[i];
        if (candidate->device_to_host &&
            (best_input == NULL || candidate->channels > best_input->channels ||
             (candidate->channels == best_input->channels &&
              candidate->bit_resolution > best_input->bit_resolution))) {
            best_input = candidate;
        }
    }
    s_device = device;
    snprintf(result.status, sizeof(result.status), "%s UAC%s: %u in / %u out • clock",
             result.high_speed ? "High-speed" : "Full-speed",
             result.audio_class_2 ? "2" : "1",
             result.input_channels, result.output_channels);
    publish(&result);
    if (best_input != NULL) {
        s_pending_stream = *best_input;
        if (best_input->clock_source_id != 0) {
            s_clock_source_id = best_input->clock_source_id;
        }
        if (xTaskCreate(stream_start_task, "uac_stream_start", 4096, device, 19, NULL) != pdPASS) {
            snprintf(result.status, sizeof(result.status), "Stream task allocation failed");
            publish(&result);
        }
    }
    ESP_LOGI(TAG, "Audio probe complete: %s, %u streaming alternatives",
             result.status, result.stream_count);
}

static void handle_device_gone(usb_device_handle_t device) {
    if (device != s_device) return;
    s_streaming = false;
    if (s_stream_interface != 0 || s_stream_alt != 0) {
        usb_host_interface_release(s_client, s_device, s_stream_interface);
    }
    usb_host_device_close(s_client, s_device);
    s_device = NULL;
    uar_probe_snapshot_t result = {.status = "USB audio device disconnected"};
    publish(&result);
}

static void client_event(const usb_host_client_event_msg_t *event, void *argument) {
    (void)argument;
    if (event->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        handle_new_device(event->new_dev.address);
    } else if (event->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        handle_device_gone(event->dev_gone.dev_hdl);
    }
}

static void library_task(void *argument) {
    (void)argument;
    while (true) {
        uint32_t flags = 0;
        esp_err_t error = usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (error != ESP_OK) vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void client_task(void *argument) {
    (void)argument;
    while (true) {
        esp_err_t error = usb_host_client_handle_events(s_client, portMAX_DELAY);
        if (error != ESP_OK) vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void monitor_task(void *argument) {
    (void)argument;
    monitor_block_t block;
    while (true) {
        if (xQueueReceive(s_monitor_queue, &block, portMAX_DELAY) != pdTRUE) continue;
        uint16_t record_offset = 0;
        if (s_record_armed) {
            uint32_t bar_frames = (4U * 60U * 48000U) / s_loop_bpm;
            uint32_t phase = (uint32_t)(s_transport_frame % bar_frames);
            uint32_t until_bar = phase == 0 ? 0 : bar_frames - phase;
            if (until_bar < block.frames) {
                record_offset = (uint16_t)until_bar;
                s_overdubbing = s_pending_overdub;
                s_pending_overdub = false;
                s_overdub_frames = 0;
                if (!s_overdubbing) {
                    s_track_frames[s_selected_track] = 0;
                    s_track_position[s_selected_track] = 0;
                }
                s_record_armed = false;
                s_loop_state = UAR_LOOP_RECORDING;
            }
        }
        if (s_loop_state == UAR_LOOP_RECORDING) {
            uint8_t track = s_selected_track;
            uint16_t input_frames = block.frames - record_offset;
            uint32_t target_frames =
                ((uint32_t)s_loop_bars * 4U * 60U * 48000U) / s_loop_bpm;
            if (target_frames > LOOP_MAX_FRAMES) target_frames = LOOP_MAX_FRAMES;
            if (s_overdubbing) {
                for (uint16_t frame = 0; frame < input_frames; frame++) {
                    uint32_t position = (s_track_position[track] + record_offset + frame) %
                                        s_track_frames[track];
                    for (uint8_t channel = 0; channel < 2; channel++) {
                        int32_t mixed = s_track_samples[track][position * 2 + channel] / 2 +
                                        block.samples[(record_offset + frame) * 2 + channel] / 2;
                        s_track_samples[track][position * 2 + channel] = (int16_t)mixed;
                    }
                }
                s_overdub_frames += input_frames;
            } else {
                uint32_t available = target_frames - s_track_frames[track];
                uint32_t copy_frames = input_frames < available ? input_frames : available;
                if (copy_frames > 0) {
                    memcpy(s_track_samples[track] + (size_t)s_track_frames[track] * 2,
                           block.samples + (size_t)record_offset * 2,
                           (size_t)copy_frames * 2 * sizeof(int16_t));
                    s_track_frames[track] += copy_frames;
                }
            }
            if ((!s_overdubbing && s_track_frames[track] >= target_frames) ||
                (s_overdubbing && s_overdub_frames >= s_track_frames[track])) {
                s_track_position[track] = 0;
                s_loop_state = UAR_LOOP_PLAYING;
                s_overdubbing = false;
            }
        }
        if (s_loop_state == UAR_LOOP_PLAYING ||
            s_loop_state == UAR_LOOP_RECORDING || s_record_armed || s_metronome) {
            uint8_t active_tracks = 0;
            for (uint8_t track = 0; track < 4; track++) {
                if (s_loop_state == UAR_LOOP_RECORDING && track == s_selected_track &&
                    !s_overdubbing) continue;
                if (s_track_frames[track] > 0) active_tracks++;
            }
            for (uint16_t frame = 0; frame < block.frames; frame++) {
                for (uint8_t channel = 0; channel < 2; channel++) {
                    int32_t mixed = active_tracks ?
                                    (int32_t)block.samples[frame * 2 + channel] / 2 :
                                    block.samples[frame * 2 + channel];
                    for (uint8_t track = 0; track < 4; track++) {
                        if (s_loop_state == UAR_LOOP_RECORDING &&
                            track == s_selected_track && !s_overdubbing) continue;
                        if (s_track_frames[track] == 0) continue;
                        int32_t track_sample =
                            s_track_samples[track][s_track_position[track] * 2 + channel];
                        mixed += (track_sample * s_track_volume[track]) /
                                 (100 * active_tracks);
                    }
                    if (s_metronome) {
                        uint32_t beat_frames = (60U * 48000U) / s_loop_bpm;
                        uint32_t beat_phase = (uint32_t)(s_transport_frame % beat_frames);
                        uint32_t beat = (uint32_t)(s_transport_frame / beat_frames);
                        bool downbeat = (beat % 4U) == 0;
                        if (beat_phase < 480) {
                            uint32_t half_period = downbeat ? 12 : 24;
                            int32_t level = downbeat ? 7000 : 4000;
                            mixed += ((beat_phase / half_period) & 1) ? level : -level;
                        }
                    }
                    if (mixed > 32767) mixed = 32767;
                    if (mixed < -32768) mixed = -32768;
                    block.samples[frame * 2 + channel] = (int16_t)mixed;
                }
                for (uint8_t track = 0; track < 4; track++) {
                    if (s_loop_state == UAR_LOOP_RECORDING &&
                        track == s_selected_track && !s_overdubbing) continue;
                    if (s_track_frames[track] == 0) continue;
                    if (++s_track_position[track] >= s_track_frames[track]) {
                        s_track_position[track] = 0;
                    }
                }
                s_transport_frame++;
            }
        }
        size_t written = 0;
        esp_err_t error = i2s_channel_write(
            s_i2s, block.samples, (size_t)block.frames * 2 * sizeof(int16_t),
            &written, pdMS_TO_TICKS(20)
        );
        if (error != ESP_OK) {
            ESP_LOGW(TAG, "Monitor I2S write failed: %s", esp_err_to_name(error));
        }
    }
}

esp_err_t uar_probe_start(void) {
    s_original_vprintf = esp_log_set_vprintf(diagnostic_vprintf);
    esp_log_level_set("USB_HOST", ESP_LOG_VERBOSE);
    esp_log_level_set("USBH", ESP_LOG_VERBOSE);
    esp_log_level_set("HUB", ESP_LOG_VERBOSE);
    esp_log_level_set("ENUM", ESP_LOG_VERBOSE);
    esp_log_level_set("usb", ESP_LOG_VERBOSE);
    ESP_RETURN_ON_ERROR(bsp_audio_get_i2s_handle(&s_i2s), TAG, "Cannot get I2S");
    esp_err_t i2s_error = i2s_channel_disable(s_i2s);
    if (i2s_error != ESP_OK && i2s_error != ESP_ERR_INVALID_STATE) {
        return i2s_error;
    }
    ESP_RETURN_ON_ERROR(bsp_audio_set_rate(48000), TAG, "Cannot set audio rate");
    i2s_error = i2s_channel_enable(s_i2s);
    if (i2s_error != ESP_OK && i2s_error != ESP_ERR_INVALID_STATE) {
        return i2s_error;
    }
    ESP_RETURN_ON_ERROR(bsp_audio_set_volume(55.0f), TAG, "Cannot set volume");
    ESP_RETURN_ON_ERROR(bsp_audio_set_amplifier(true), TAG, "Cannot enable amplifier");
    s_monitor_queue = xQueueCreate(4, sizeof(monitor_block_t));
    if (s_monitor_queue == NULL) return ESP_ERR_NO_MEM;
    for (uint8_t track = 0; track < 4; track++) {
        s_track_samples[track] = heap_caps_malloc(
            (size_t)LOOP_MAX_FRAMES * 2 * sizeof(int16_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        );
        if (s_track_samples[track] == NULL) {
            ESP_LOGW(TAG, "No PSRAM for looper track %u", track + 1);
        }
    }
    const usb_host_config_t host_configuration = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t error = usb_host_install(&host_configuration);
    if (error != ESP_OK) return error;

    const usb_host_client_config_t client_configuration = {
        .is_synchronous = false,
        .max_num_event_msg = 8,
        .async = {
            .client_event_callback = client_event,
            .callback_arg = NULL,
        },
    };
    error = usb_host_client_register(&client_configuration, &s_client);
    if (error != ESP_OK) return error;
    s_control_done = xSemaphoreCreateBinary();
    if (s_control_done == NULL) return ESP_ERR_NO_MEM;
    s_iso_resubmit_queue = xQueueCreate(ISO_TRANSFER_COUNT * 2, sizeof(usb_transfer_t *));
    if (s_iso_resubmit_queue == NULL) return ESP_ERR_NO_MEM;
    if (xTaskCreate(library_task, "uac_host", 4096, NULL, 20, NULL) != pdPASS ||
        xTaskCreate(client_task, "uac_probe", 6144, NULL, 20, NULL) != pdPASS ||
        xTaskCreate(iso_resubmit_task, "uac_iso", 4096, NULL, 21, NULL) != pdPASS ||
        xTaskCreate(monitor_task, "uac_monitor", 4096, NULL, 22, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
