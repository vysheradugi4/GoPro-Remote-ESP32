/*
 * GoPro BLE (GP-Control) client interface.
 *
 * This module turns the ESP32 into a BLE central that talks to a GoPro camera
 * over the GP-Control service (UUID 0000FEA6-0000-1000-8000-00805F9B34FB).
 * It handles scanning, connecting, bonding, GATT discovery, notifications and
 * the periodic keep-alive, then exposes a small thread-safe API for the HTTP
 * layer (see main.cpp) to query the camera state and send commands.
 *
 * The wire protocol implemented here follows the official OpenGoPro V2 BLE
 * protocol, split into at most 20-byte packets with an OpenGoPro packet header.
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace gopro {

// Connection state machine reported to the UI.
enum class State : uint8_t {
    Idle = 0,      // Bluetooth stack not started yet.
    Scanning,      // Looking for a GoPro advertising the GP-Control service.
    Connecting,    // Link established, waiting for encryption/bonding.
    Connected,     // Ready: service discovered and notifications registered.
    Disconnected,  // Was connected, link dropped (we try to reconnect).
    Failed,        // A terminal error happened; user should retry.
};

// One camera setting value that is rendered on the web page.
struct SettingView {
    bool known;        // false -> value not received yet.
    uint8_t id;        // GoPro setting id (RESOLUTION, FRAMERATE, ...).
    uint32_t raw;      // Raw value as received from the camera.
};

// Aggregated camera state snapshot. All strings are null-terminated and are
// safe to use while a status snapshot is being rendered; the caller must call
// gopro::copy_status() to obtain a consistent copy.
struct Status {
    State state;
    int model_id;              // Mapped model index (see model_id_string()).
    uint8_t raw_model_id;      // Raw model byte from the advertisement.
    bool encoding;             // Shutter is open (recording / capturing).
    uint32_t encoding_duration;  // Elapsed recording time, seconds.
    int battery_percent;       // -1 when unknown.
    bool sd_remaining_known;
    uint32_t sd_remaining_seconds;
    bool resolution_known;
    uint32_t resolution_raw;
    bool framerate_known;
    uint32_t framerate_raw;
    bool lens_known;
    uint32_t lens_raw;
    bool flicker_known;
    uint32_t flicker_raw;
    bool hypersmooth_known;
    uint32_t hypersmooth_raw;
    bool gps_known;
    uint32_t gps_raw;
    bool led_known;
    uint32_t led_raw;
    bool preset_group_known;   // Current UI mode group has been reported.
    uint32_t preset_group;     // 1000 video / 1001 photo / 1002 timelapse.
    char device_name[32];      // BLE advertised name of the camera.

    // Diagnostics: helps to understand what the scanner actually sees.
    uint32_t adv_seen;         // Advertisements processed since boot.
    int last_adv_rssi;         // RSSI of the most recent advertisement.
    bool last_adv_is_gopro;    // Whether the most recent advertiser exposed FEA6.
    char last_adv_name[32];    // Name of the most recent advertiser.
};

// Initialize the BLE stack and start scanning for a GoPro. Safe to call once.
void init();

// Request a connection attempt (starts scanning if needed). Returns false if
// the request cannot be queued (already connected / busy).
bool connect();

// Drop the current connection and return to the idle state.
void disconnect();

// Action button: trigger the shutter, i.e. start recording/capture, or stop it
// if a recording is already running.
void shutter();

// Mode button: switch to the next preset group, cycling through the camera
// video / photo / timelapse modes (the camera keeps its own presets).
void next_mode();

// Copy the current camera state into *out. Returns true when *out is valid.
bool copy_status(Status *out);

// Human readable model name for a mapped model index (e.g. "HERO13").
const char *model_id_string(int model_id);

// Render the current status as a JSON document into buf. Returns the number of
// bytes written (excluding the terminator) or a negative value on error.
int status_json(char *buf, size_t buf_size);

}  // namespace gopro
