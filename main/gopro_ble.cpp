/*
 * GoPro BLE (GP-Control) client implementation.
 *
 * Flow:
 *   1. Start the Bluedroid BLE stack as a central (GATT client).
 *   2. Scan for a device advertising the GP-Control service (0xFEA6).
 *   3. Open a link, bond/encrypt it, discover the GP-Control service.
 *   4. Cache the six "gpxx" characteristic handles (command / settings / query
 *      and their notification counterparts).
 *   5. Subscribe to the three notification characteristics.
 *   6. Register interest in settings/status and send a periodic keep-alive.
 *   7. Decode incoming notifications (length framing + TLV) into a status
 *      snapshot exposed through the HTTP layer.
 *
 * Commands and responses use the OpenGoPro V2 BLE protocol (the official GoPro
 * BLE API): a write is a payload of [command, param_len, params...] with the
 * parameter length omitted when there are no parameters, split into at most
 * 20-byte packets prefixed with an OpenGoPro packet header.
 * All comments in this project are written in English on purpose.
 */
#include "gopro_ble.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
// Logging is compiled out for now: the ESP_LOG* calls stay in the source but
// expand to nothing, which keeps the code easy to restore while saving power
// and code size in the battery-powered build. Define GOPRO_ENABLE_LOGS to bring
// every log line back.
#ifndef GOPRO_ENABLE_LOGS
#undef ESP_LOGE
#undef ESP_LOGW
#undef ESP_LOGI
#undef ESP_LOGD
#undef ESP_LOGV
#define ESP_LOGE(tag, format, ...) ((void)0)
#define ESP_LOGW(tag, format, ...) ((void)0)
#define ESP_LOGI(tag, format, ...) ((void)0)
#define ESP_LOGD(tag, format, ...) ((void)0)
#define ESP_LOGV(tag, format, ...) ((void)0)
#endif

namespace gopro {
namespace {

[[maybe_unused]] constexpr const char *TAG = "gopro_ble";

// ---------------------------------------------------------------------------
// GP-Control service and characteristics
// ---------------------------------------------------------------------------

constexpr uint16_t kServiceUuid16 = 0xFEA6;

// The GoPro characteristics use UUIDs derived from "gpxx":
//   B5F9<gpxx>-AA8D-11E3-9046-0002A5D5C51B
// where <gpxx> is a 4 digit decimal number.
constexpr uint16_t kCharCommand = 0x0072;
constexpr uint16_t kCharCommandResp = 0x0073;
constexpr uint16_t kCharSettings = 0x0074;
constexpr uint16_t kCharSettingsResp = 0x0075;
constexpr uint16_t kCharQuery = 0x0076;
constexpr uint16_t kCharQueryResp = 0x0077;

constexpr int kCharCount = 6;
const uint16_t kCharIds[kCharCount] = {
    kCharCommand, kCharCommandResp, kCharSettings,
    kCharSettingsResp, kCharQuery, kCharQueryResp,
};

// Indexes into the handle array, aligned with kCharIds.
enum CharIndex : int {
    IDX_COMMAND = 0,
    IDX_COMMAND_RESP = 1,
    IDX_SETTINGS = 2,
    IDX_SETTINGS_RESP = 3,
    IDX_QUERY = 4,
    IDX_QUERY_RESP = 5,
};

// Base of the 128-bit UUID in the little-endian order used by Bluedroid.
// Only bytes 12/13 carry the "gpxx" value (low byte then high byte).
const uint8_t kUuidBase[16] = {
    0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x46, 0x90,
    0xE3, 0x11, 0x8D, 0xAA, 0x00, 0x00, 0xF9, 0xB5,
};

void make_char_uuid(uint16_t gpxx, esp_bt_uuid_t *out)
{
    out->len = ESP_UUID_LEN_128;
    std::memcpy(out->uuid.uuid128, kUuidBase, sizeof(kUuidBase));
    out->uuid.uuid128[12] = static_cast<uint8_t>(gpxx & 0xFF);
    out->uuid.uuid128[13] = static_cast<uint8_t>((gpxx >> 8) & 0xFF);
}

// ---------------------------------------------------------------------------
// Protocol identifiers
// ---------------------------------------------------------------------------

// Settings we register/read from the camera.
enum SettingId : uint8_t {
    SETTING_RESOLUTION = 2,
    SETTING_FRAMERATE = 3,
    SETTING_GPS = 83,
    SETTING_LED = 91,
    SETTING_LENS = 121,
    SETTING_FLICKER = 134,
    SETTING_HYPERSMOOTH = 135,
};

// Status values we register/read.
enum StatusId : uint8_t {
    STATUS_ENCODING = 10,
    STATUS_ENCODING_DURATION = 13,
    STATUS_SD_REMAINING = 35,
    STATUS_BATTERY = 70,
    // Current preset group / UI mode: an Int32ub (1000 video, 1001 photo,
    // 1002 timelapse). Reported whenever the camera changes mode.
    STATUS_PRESET_GROUP = 96,
};

// Query/response opcodes.
enum QueryId : uint8_t {
    QUERY_REGISTER_SETTING = 0x52,
    QUERY_REGISTER_STATUS = 0x53,
};

enum CommandId : uint8_t {
    COMMAND_SHUTTER = 0x01,
    COMMAND_LOAD_PRESET_GROUP = 0x3E,
    COMMAND_KEEP_ALIVE = 0x5B,
};

// Preset groups cycled by the "Mode" button (proto.EnumPresetGroup).
constexpr uint16_t kPresetGroups[] = {1000, 1001, 1002};  // video, photo, timelapse
constexpr int kPresetGroupCount = sizeof(kPresetGroups) / sizeof(kPresetGroups[0]);

// Model byte -> index mapping (from the advertisement).
const uint8_t kModelTable[] = {
    0, 12, 13, 19, 21, 22, 24, 30, 32, 33, 34, 50,
    51, 55, 57, 58, 60, 62, 64, 65, 66, 70, 69, 71,
};
constexpr int kModelTableSize = sizeof(kModelTable) / sizeof(kModelTable[0]);

const char *const kModelNames[kModelTableSize] = {
    "Unknown",     "HERO4 Silver", "HERO4 Black",  "HERO5 Black",
    "HERO5 Session", "Fusion",     "HERO6",        "HERO7 Black",
    "HERO7 White", "HERO7 Silver", "HERO 2018",    "HERO8",
    "MAX",         "HERO9",        "HERO10",       "HERO11",
    "HERO11 Mini", "HERO12",       "MAX2",         "HERO13",
    "HERO 2024",   "HERO Lit",     "Mission1 Pro", "Mission1",
};

// The camera presets themselves are managed by the camera; the "Mode" button
// only switches between the video / photo / timelapse preset groups.

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

constexpr uint16_t kConnIdInvalid = 0xFFFF;
constexpr int64_t kKeepAlivePeriodUs = 15LL * 1000 * 1000;  // 15 s

// Scanning runs in short sessions: scan for a moment, then rest for a while,
// and repeat. The radio (and the CPU feeding it) is idle most of the time, so
// this is markedly cheaper on the battery than a continuous scan.
constexpr uint32_t kScanSessionSeconds = 1;                 // 1 s per session.
// Keep the rest between sessions short: while the user is actively asking to
// connect, a long pause is pure added latency (the radio is only idle anyway
// once the scan session ends), so 300 ms is a good latency/battery trade-off.
constexpr int64_t kScanSessionPauseUs = 300LL * 1000;       // 300 ms between sessions.

// Preferred connection parameters (units of 1.25 ms), requested right before the
// link is opened. 15 ms / 30 ms with no slave latency is the price/quality
// compromise: responsive enough to wake the camera quickly, without paying for
// the most power-hungry 7.5 ms interval.
constexpr uint16_t kPreferredConnIntMin = 12;    // 15 ms
constexpr uint16_t kPreferredConnIntMax = 24;    // 30 ms
constexpr uint16_t kPreferredConnLatency = 0;
constexpr uint16_t kPreferredConnTimeout = 600;  // 6 s supervision timeout

SemaphoreHandle_t s_mutex = nullptr;

esp_gatt_if_t s_gattc_if = ESP_GATT_IF_NONE;
uint16_t s_conn_id = kConnIdInvalid;
esp_bd_addr_t s_remote_bda = {};
uint16_t s_char_handles[kCharCount] = {};
esp_gattc_service_elem_t s_service = {};
bool s_service_found = false;
int s_preset_group_index = 0;  // Next preset group to load (Mode button).
bool s_command_pending = false;  // A command is awaiting its response.
int64_t s_command_sent_us = 0;   // When the pending command was sent.

esp_timer_handle_t s_keepalive_timer = nullptr;
esp_timer_handle_t s_reconnect_timer = nullptr;
esp_timer_handle_t s_scan_resume_timer = nullptr;

// Notification reassembly buffer for chunked query responses.
uint8_t s_reply_buf[512];
size_t s_reply_len = 0;
size_t s_reply_expected = 0;

Status s_status = {};
bool s_auto_reconnect = false;

// ---------------------------------------------------------------------------
// Onboard status LED
// ---------------------------------------------------------------------------

#if CONFIG_GOPRO_ONBOARD_LED

#if CONFIG_GOPRO_ONBOARD_LED_TYPE_WS2812
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

constexpr gpio_num_t kLedGpio = static_cast<gpio_num_t>(CONFIG_GOPRO_ONBOARD_LED_GPIO);
constexpr uint32_t kLedResolutionHz = 10 * 1000 * 1000;  // 0.1 us per tick.

rmt_channel_handle_t s_led_chan = nullptr;
rmt_encoder_handle_t s_led_encoder = nullptr;
rmt_symbol_word_t s_led_symbols[25];  // 24 data bits + one reset symbol.

void led_channel_init()
{
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num = kLedGpio;
    tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz = kLedResolutionHz;
    tx_cfg.mem_block_symbols = 64;
    tx_cfg.trans_queue_depth = 4;
    if (rmt_new_tx_channel(&tx_cfg, &s_led_chan) != ESP_OK) {
        ESP_LOGW(TAG, "onboard LED: RMT channel init failed");
        s_led_chan = nullptr;
        return;
    }
    rmt_copy_encoder_config_t copy_cfg = {};
    if (rmt_new_copy_encoder(&copy_cfg, &s_led_encoder) != ESP_OK) {
        ESP_LOGW(TAG, "onboard LED: RMT encoder init failed");
        s_led_encoder = nullptr;
        return;
    }
    if (rmt_enable(s_led_chan) != ESP_OK) {
        ESP_LOGW(TAG, "onboard LED: RMT enable failed");
        s_led_chan = nullptr;
    }
}

// Drive a single WS2812 (data MSB first as GRB). Timings are for a 10 MHz RMT
// clock (1 tick = 0.1 us) and match the WS2812B datasheet:
//   bit 0 -> T0H 0.4 us / T0L 0.9 us   (4 / 9 ticks)
//   bit 1 -> T1H 0.8 us / T1L 0.4 us   (8 / 4 ticks)
// The old ones used T1H = 0.6 us, below the WS2812B minimum of ~0.65 us, so the
// chip misjudged the bits: the "off" pattern never latched and the LED just
// stayed lit instead of blinking. Keep the pulses inside the datasheet window.
void led_set(bool on)
{
    if (s_led_chan == nullptr || s_led_encoder == nullptr) {
        return;
    }
    const uint8_t grb[3] = {0x00, 0x00, static_cast<uint8_t>(on ? 0xFF : 0x00)};
    int idx = 0;
    for (int byte = 0; byte < 3; ++byte) {
        for (int bit = 7; bit >= 0; --bit) {
            rmt_symbol_word_t &sym = s_led_symbols[idx++];
            const bool one = ((grb[byte] >> bit) & 1) != 0;
            sym.level0 = 1;
            sym.duration0 = one ? 8 : 4;
            sym.level1 = 0;
            sym.duration1 = one ? 4 : 9;
        }
    }
    // Reset / latch pulse: the WS2812 needs the data line held low for >= 50 us
    // to latch the frame. Both halves must have a non-zero duration - a symbol
    // with a zero-length half makes the copy encoder emit a truncated frame, so
    // the "off" colour never latched and the LED stayed lit instead of blinking.
    // 400 + 400 ticks at 0.1 us/tick = 80 us low, comfortably above the minimum.
    s_led_symbols[idx].level0 = 0;
    s_led_symbols[idx].duration0 = 400;
    s_led_symbols[idx].level1 = 0;
    s_led_symbols[idx].duration1 = 400;
    idx++;

    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0;
    tx_cfg.flags.eot_level = 0;  // Idle low after the frame (keeps the latch).
    if (rmt_transmit(s_led_chan, s_led_encoder, s_led_symbols,
                     idx * sizeof(rmt_symbol_word_t), &tx_cfg) == ESP_OK) {
        // Bounded wait: never block the shared esp_timer task (which also drives
        // the blink toggle and the keep-alive) with portMAX_DELAY.
        rmt_tx_wait_all_done(s_led_chan, 100);
    }
}

#else  // A plain LED wired to a GPIO.

#include "driver/gpio.h"

constexpr gpio_num_t kLedGpio = static_cast<gpio_num_t>(CONFIG_GOPRO_ONBOARD_LED_GPIO);

void led_channel_init()
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << static_cast<int>(kLedGpio);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
    const int off_level = CONFIG_GOPRO_ONBOARD_LED_ACTIVE_LOW ? 1 : 0;
    gpio_set_level(kLedGpio, off_level);
}

void led_set(bool on)
{
#if CONFIG_GOPRO_ONBOARD_LED_ACTIVE_LOW
    gpio_set_level(kLedGpio, on ? 0 : 1);
#else
    gpio_set_level(kLedGpio, on ? 1 : 0);
#endif
}

#endif  // CONFIG_GOPRO_ONBOARD_LED_TYPE_WS2812

// Blink the LED slowly (one toggle per second) while there is no BLE link, and
// keep it steadily lit once the camera is connected.
bool s_led_blink_state = false;
esp_timer_handle_t s_led_timer = nullptr;

void led_blink_timer_cb(void *)
{
    s_led_blink_state = !s_led_blink_state;
    led_set(s_led_blink_state);
}

// Start (or restart) slow blinking. Called while disconnected / scanning.
void led_blink_start()
{
    if (s_led_timer == nullptr) {
        const esp_timer_create_args_t args = {
            .callback = &led_blink_timer_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "gopro_led",
            .skip_unhandled_events = false,
        };
        if (esp_timer_create(&args, &s_led_timer) != ESP_OK) {
            s_led_timer = nullptr;
            return;
        }
    }
    esp_timer_stop(s_led_timer);
    s_led_blink_state = true;
    led_set(true);
    esp_timer_start_periodic(s_led_timer, 1000LL * 1000);
}

// Stop blinking and hold the LED at a fixed level. Called once connected.
void led_solid(bool on)
{
    if (s_led_timer != nullptr) {
        esp_timer_stop(s_led_timer);
    }
    s_led_blink_state = on;
    led_set(on);
}

#else  // Onboard LED disabled.

void led_channel_init() {}
void led_set(bool) {}
void led_blink_start() {}
void led_solid(bool) {}

#endif  // CONFIG_GOPRO_ONBOARD_LED

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

void lock()
{
    if (s_mutex != nullptr) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
}

void unlock()
{
    if (s_mutex != nullptr) {
        xSemaphoreGive(s_mutex);
    }
}

// True while a command has been sent but its response has not arrived yet.
// The pending flag expires after a few seconds so the UI never stays stuck.
bool command_busy()
{
    lock();
    const bool busy = s_command_pending &&
                      (esp_timer_get_time() - s_command_sent_us) < (3LL * 1000 * 1000);
    unlock();
    return busy;
}

void set_state(State state)
{
    lock();
    s_status.state = state;
    unlock();
}

bool adv_has_gopro_service(const uint8_t *data, uint8_t len)
{
    uint8_t i = 0;
    while (i + 1 < len) {
        const uint8_t field_len = data[i];
        if (field_len == 0 || i + 1 + field_len > len) {
            break;
        }
        const uint8_t type = data[i + 1];
        const uint8_t *payload = &data[i + 2];
        const uint8_t payload_len = static_cast<uint8_t>(field_len - 1);
        // Incomplete/complete list of 16-bit service class UUIDs (0x02/0x03).
        if ((type == 0x02 || type == 0x03) && payload_len >= 2) {
            for (uint8_t j = 0; j + 1 < payload_len; j += 2) {
                const uint16_t uuid = static_cast<uint16_t>(payload[j] | (payload[j + 1] << 8));
                if (uuid == kServiceUuid16) {
                    return true;
                }
            }
        }
        // Incomplete/complete list of 128-bit service class UUIDs (0x06/0x07).
        // The GP-Control base UUID ends with ...0000FEA6..., which appears as
        // the bytes 0xA6 0xFE at offset 12 of every 16-byte little-endian UUID.
        if ((type == 0x06 || type == 0x07) && payload_len >= 4) {
            for (uint8_t j = 0; j + 15 < payload_len; j += 16) {
                if (payload[j + 12] == 0xA6 && payload[j + 13] == 0xFE) {
                    return true;
                }
            }
        }
        i += field_len + 1;
    }
    return false;
}

// Build a space separated hex dump of an advertisement payload (truncated to
// keep the log line readable). Used only for debugging.
void log_adv_hex(const char *label, const uint8_t *data, uint8_t len)
{
    char hex[3 * 40 + 1];
    size_t n = 0;
    const uint8_t shown = len > 40 ? 40 : len;
    for (uint8_t i = 0; i < shown && n + 3 < sizeof(hex); ++i) {
        n += static_cast<size_t>(std::snprintf(&hex[n], sizeof(hex) - n, "%02X ", data[i]));
    }
    hex[n] = '\0';
    ESP_LOGI(TAG, "%s (%u bytes): %s", label, len, hex);
}

// Extract the GoPro model byte and the advertised name.
void adv_parse(const uint8_t *data, uint8_t len, int *model_raw, char *name, size_t name_size)
{
    uint8_t i = 0;
    while (i + 1 < len) {
        const uint8_t field_len = data[i];
        if (field_len == 0 || i + 1 + field_len > len) {
            break;
        }
        const uint8_t type = data[i + 1];
        const uint8_t *payload = &data[i + 2];
        const uint8_t payload_len = static_cast<uint8_t>(field_len - 1);

        if (type == 0xFF && payload_len > 13) {
            // Manufacturer specific data: the model byte sits at index 13.
            *model_raw = payload[13];
        } else if ((type == 0x08 || type == 0x09) && name_size > 1) {
            const size_t n = payload_len < name_size - 1 ? payload_len : name_size - 1;
            std::memcpy(name, payload, n);
            name[n] = '\0';
        }
        i += field_len + 1;
    }
}

int model_index_from_raw(int raw)
{
    for (int i = 0; i < kModelTableSize; ++i) {
        if (kModelTable[i] == raw) {
            return i;
        }
    }
    return 0;  // Unknown
}

// ---------------------------------------------------------------------------
// Characteristic write helpers
// ---------------------------------------------------------------------------

void write_char(int index, const uint8_t *data, size_t len)
{
    if (s_conn_id == kConnIdInvalid || s_char_handles[index] == 0) {
        ESP_LOGW(TAG, "write to char %d ignored, not connected", index);
        return;
    }
    esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_char_handles[index],
                             static_cast<uint16_t>(len), const_cast<uint8_t *>(data),
                             ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
}

// OpenGoPro BLE packets are at most 20 bytes and carry a small header:
//   - extended-13:  continuation(0) + type(0b01) + 13-bit length
//   - extended-16:  continuation(0) + type(0b10) + 5-bit pad + 16-bit length
//   - continuation: continuation(1) + 7-bit padding (0x80)
// The first packet carries the total payload length; every following packet is
// a continuation packet with room for 19 payload bytes.
constexpr size_t kMaxBlePacketLen = 20;

void send_fragmented(int index, const uint8_t *data, size_t len)
{
    if (len == 0) {
        return;
    }
    uint8_t packet[kMaxBlePacketLen];
    size_t offset = 0;
    bool first = true;
    while (offset < len) {
        size_t header_len = 0;
        if (first) {
            if (len < ((1u << 13) - 1)) {
                const uint16_t header = static_cast<uint16_t>(0x2000u | (len & 0x1FFFu));
                packet[0] = static_cast<uint8_t>(header >> 8);
                packet[1] = static_cast<uint8_t>(header & 0xFFu);
                header_len = 2;
            } else {
                packet[0] = 0x40;
                packet[1] = static_cast<uint8_t>((len >> 8) & 0xFFu);
                packet[2] = static_cast<uint8_t>(len & 0xFFu);
                header_len = 3;
            }
            first = false;
        } else {
            packet[0] = 0x80;  // Continuation header.
            header_len = 1;
        }

        size_t chunk = len - offset;
        const size_t room = kMaxBlePacketLen - header_len;
        if (chunk > room) {
            chunk = room;
        }
        std::memcpy(packet + header_len, data + offset, chunk);
        offset += chunk;
        write_char(index, packet, header_len + chunk);
    }
}

// Send a command over the command characteristic. The payload is
// [command, param_len, params...]; the parameter length is omitted when the
// command takes no parameters (OpenGoPro V2).
void send_camera_command(uint8_t command, const uint8_t *params, uint8_t params_len,
                         bool expect_response)
{
    uint8_t payload[8];
    size_t n = 0;
    payload[n++] = command;
    if (params_len > 0) {
        payload[n++] = params_len;
        for (uint8_t i = 0; i < params_len; ++i) {
            payload[n++] = params[i];
        }
    }

    if (expect_response) {
        lock();
        s_command_pending = true;
        s_command_sent_us = esp_timer_get_time();
        unlock();
    }
    send_fragmented(IDX_COMMAND, payload, n);
}

// Subscribe to the settings and statuses we render. Registration uses the query
// characteristic with the payload [query_id, identifier] (OpenGoPro V2).
void register_for_updates()
{
    const uint8_t setting_ids[] = {
        SETTING_RESOLUTION, SETTING_FRAMERATE, SETTING_GPS,
        SETTING_LED, SETTING_LENS, SETTING_FLICKER, SETTING_HYPERSMOOTH,
    };
    for (uint8_t id : setting_ids) {
        const uint8_t payload[2] = {QUERY_REGISTER_SETTING, id};
        send_fragmented(IDX_QUERY, payload, sizeof(payload));
    }

    const uint8_t status_ids[] = {
        STATUS_ENCODING, STATUS_ENCODING_DURATION, STATUS_BATTERY, STATUS_SD_REMAINING,
        STATUS_PRESET_GROUP,
    };
    for (uint8_t id : status_ids) {
        const uint8_t payload[2] = {QUERY_REGISTER_STATUS, id};
        send_fragmented(IDX_QUERY, payload, sizeof(payload));
    }
}

void keepalive()
{
    if (s_conn_id == kConnIdInvalid) {
        return;
    }
    // KEEP_ALIVE with the "BLE keep alive" LED behaviour parameter (0x42).
    const uint8_t param = 0x42;
    send_camera_command(COMMAND_KEEP_ALIVE, &param, 1, false);
}

// ---------------------------------------------------------------------------
// Decoding of TLV notifications
// ---------------------------------------------------------------------------

const char *resolution_label(uint8_t id, char *out, size_t out_size)
{
    static const uint8_t ids[] = {
        1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 18, 21, 24, 25,
        26, 27, 28, 31, 35, 36, 37, 38, 39, 40, 44, 100, 107, 108, 109, 110,
        111, 112, 113,
    };
    static const uint16_t vres[] = {
        4000, 4000, 2700, 2700, 2700, 1440, 1080, 1080, 960, 720, 720, 480,
        5200, 3000, 480, 4000, 5600, 5000, 5000, 5300, 5300, 4000, 8000, 5300,
        4000, 4000, 900, 4000, 8000, 1440, 5300, 5300, 4000, 4000, 1080, 2700,
        4000, 5300,
    };
    static const char *const aspect[] = {
        "16:9", "16:9", "16:9", "16:9", "4:3", "4:3", "16:9", "16:9", "4:3",
        "16:9", "16:9", "16:9", "360", "360", "16:9", "4:3", "360", "16:9",
        "4:3", "8:7", "4:3", "8:7", "16:9", "21:9", "21:9", "1:1", "16:9",
        "360", "4:3", "4:3", "16:9", "8:7", "8:7", "9:16", "9:16", "4:3",
        "4:3", "4:3",
    };
    constexpr size_t count = sizeof(ids) / sizeof(ids[0]);

    for (size_t i = 0; i < count; ++i) {
        if (ids[i] != id) {
            continue;
        }
        const uint16_t r = vres[i];
        if (r < 2000) {
            std::snprintf(out, out_size, "%up %s", static_cast<unsigned>(r), aspect[i]);
        } else if (r % 1000 == 0) {
            std::snprintf(out, out_size, "%uK %s", static_cast<unsigned>(r / 1000), aspect[i]);
        } else {
            std::snprintf(out, out_size, "%.1fK %s",
                          static_cast<double>(r) / 1000.0, aspect[i]);
        }
        return out;
    }
    std::snprintf(out, out_size, "Unknown");
    return out;
}

uint16_t framerate_value(uint8_t id)
{
    switch (id) {
    case 0: return 240;
    case 1: return 120;
    case 2: return 100;
    case 3: return 90;
    case 4: return 80;
    case 5: return 60;
    case 6: return 50;
    case 7: return 48;
    case 8: return 30;
    case 9: return 25;
    case 10: return 24;
    case 11: return 15;
    case 12: return 12;
    case 13: return 200;
    case 15: return 400;
    case 16: return 360;
    case 17: return 300;
    case 18: return 480;
    case 19: return 960;
    case 20: return 800;
    default: return 0;
    }
}

const char *lens_label(uint8_t id)
{
    switch (id) {
    case 0: return "Wide";
    case 1: return "Medium";
    case 2: return "Narrow";
    case 3: return "SuperView";
    case 4: return "Linear";
    case 5: return "Dual 360";
    case 6: return "Narrow Max";
    case 7: return "Max SuperView";
    case 8: return "Linear + Level";
    case 9: return "Linear + Horizon Lock";
    case 104: return "Ultra HyperView";
    default: return "Lens";
    }
}

const char *flicker_label(uint8_t id)
{
    switch (id) {
    case 0: return "NTSC";
    case 1: return "PAL";
    case 2: return "60Hz";
    case 3: return "50Hz";
    default: return "Unknown";
    }
}

const char *hypersmooth_label(uint8_t id)
{
    switch (id) {
    case 0: return "Off";
    case 1: return "Low";
    case 2: return "High";
    case 3: return "Boost";
    case 4: return "Auto Boost";
    case 100: return "Standard";
    default: return "Unknown";
    }
}

const char *led_label(uint8_t id)
{
    switch (id) {
    case 0: return "Off";
    case 1: return "Front Off";
    case 2: return "On";
    case 3: return "All On";
    case 4: return "All Off";
    case 5: return "Front Off";
    case 100: return "Back Only";
    default: return "Unknown";
    }
}

void on_receive_setting(uint8_t id, const uint8_t *value, size_t length)
{
    if (length == 0) {
        return;
    }
    const uint8_t v = value[0];
    lock();
    switch (id) {
    case SETTING_RESOLUTION:
        s_status.resolution_known = true;
        s_status.resolution_raw = v;
        break;
    case SETTING_FRAMERATE:
        s_status.framerate_known = true;
        s_status.framerate_raw = v;
        break;
    case SETTING_LENS:
        s_status.lens_known = true;
        s_status.lens_raw = v;
        break;
    case SETTING_FLICKER:
        s_status.flicker_known = true;
        s_status.flicker_raw = v;
        break;
    case SETTING_HYPERSMOOTH:
        s_status.hypersmooth_known = true;
        s_status.hypersmooth_raw = v;
        break;
    case SETTING_GPS:
        s_status.gps_known = true;
        s_status.gps_raw = v;
        break;
    case SETTING_LED:
        s_status.led_known = true;
        s_status.led_raw = v;
        break;
    default:
        break;
    }
    unlock();
}

void on_receive_status(uint8_t id, const uint8_t *value, size_t length)
{
    if (length == 0) {
        return;
    }
    lock();
    switch (id) {
    case STATUS_ENCODING:
        s_status.encoding = (value[0] == 1);
        break;
    case STATUS_ENCODING_DURATION:
        if (length >= 4) {
            s_status.encoding_duration =
                (static_cast<uint32_t>(value[0]) << 24) |
                (static_cast<uint32_t>(value[1]) << 16) |
                (static_cast<uint32_t>(value[2]) << 8) |
                static_cast<uint32_t>(value[3]);
        }
        break;
    case STATUS_SD_REMAINING:
        if (length >= 4) {
            s_status.sd_remaining_known = true;
            s_status.sd_remaining_seconds =
                (static_cast<uint32_t>(value[0]) << 24) |
                (static_cast<uint32_t>(value[1]) << 16) |
                (static_cast<uint32_t>(value[2]) << 8) |
                static_cast<uint32_t>(value[3]);
        }
        break;
    case STATUS_BATTERY:
        s_status.battery_percent = value[0];
        break;
    case STATUS_PRESET_GROUP:
        if (length >= 4) {
            s_status.preset_group_known = true;
            s_status.preset_group =
                (static_cast<uint32_t>(value[0]) << 24) |
                (static_cast<uint32_t>(value[1]) << 16) |
                (static_cast<uint32_t>(value[2]) << 8) |
                static_cast<uint32_t>(value[3]);
        }
        break;
    default:
        break;
    }
    unlock();
}

// Read a TLV message: [queryId, status, TLV...].
void read_tlv_message(const uint8_t *message, size_t length)
{
    if (length < 2) {
        return;
    }
    const uint8_t query_id = message[0];
    const uint8_t *data = message + 2;
    const size_t data_len = length - 2;

    const uint8_t mask = query_id & 0x1F;
    const bool is_setting = ((mask ^ 0x12) == 0 && query_id != 0x32) || (mask ^ 0x15) == 0;
    const bool is_status = (mask ^ 0x13) == 0 || (mask ^ 0x16) == 0;

    for (size_t i = 0; i + 1 < data_len;) {
        uint8_t type = data[i];
        size_t offset = i + 2;
        if (type == 0xFF) {
            // Grouped value: skip the group id and use the real type.
            if (i + 2 >= data_len) {
                break;
            }
            type = data[i + 2];
            offset = i + 3;
        }
        if (offset > data_len) {
            break;
        }
        const uint8_t value_len = data[i + 1];
        if (offset + value_len > data_len) {
            break;
        }
        const uint8_t *value = data + offset;

        if (is_setting) {
            on_receive_setting(type, value, value_len);
        } else if (is_status) {
            on_receive_status(type, value, value_len);
        }
        i = offset + value_len;
    }
}

// Token used to reset the reassembly buffer when a new response starts.
void decode_query(const uint8_t *response, size_t length)
{
    if (length == 0) {
        return;
    }
    const uint8_t header = response[0];

    if ((header & 0xE0) == 0x00) {
        // Single packet: drop the length byte and decode.
        read_tlv_message(response + 1, length - 1);
    } else if ((header & 0xE0) == 0x20) {
        // Multi packet: 5-bit high length + low length, first chunk follows.
        if (length < 2) {
            return;
        }
        s_reply_expected = static_cast<size_t>(((header & 0x1F) << 8) | response[1]);
        s_reply_len = 0;
        const size_t chunk = length - 2;
        if (chunk <= sizeof(s_reply_buf)) {
            std::memcpy(s_reply_buf, response + 2, chunk);
            s_reply_len = chunk;
        }
    } else if ((header & 0xE0) == 0x40) {
        // Multi packet with 16-bit length.
        if (length < 3) {
            return;
        }
        s_reply_expected = static_cast<size_t>((response[1] << 8) | response[2]);
        s_reply_len = 0;
        const size_t chunk = length - 3;
        if (chunk <= sizeof(s_reply_buf)) {
            std::memcpy(s_reply_buf, response + 3, chunk);
            s_reply_len = chunk;
        }
    } else if ((header & 0x80) == 0x80) {
        // Continuation chunk.
        const size_t chunk = length - 1;
        if (s_reply_len + chunk <= sizeof(s_reply_buf)) {
            std::memcpy(s_reply_buf + s_reply_len, response + 1, chunk);
            s_reply_len += chunk;
        }
        if (s_reply_expected != 0 && s_reply_len >= s_reply_expected) {
            read_tlv_message(s_reply_buf, s_reply_len);
            s_reply_len = 0;
            s_reply_expected = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// GATT client
// ---------------------------------------------------------------------------

void start_scan();

void start_scan_if_needed()
{
    lock();
    const State state = s_status.state;
    unlock();
    if (state == State::Connected || state == State::Connecting) {
        return;
    }
    start_scan();
}

void configure_and_scan()
{
    esp_ble_scan_params_t scan_params = {};
    scan_params.scan_type = BLE_SCAN_TYPE_ACTIVE;
    scan_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
    scan_params.scan_interval = 0x50;
    scan_params.scan_window = 0x30;
    scan_params.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE;
    esp_ble_gap_set_scan_params(&scan_params);
}

void start_scan()
{
    set_state(State::Scanning);
    // Short sessions instead of a continuous scan: the stack stops by itself
    // after kScanSessionSeconds and ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT schedules
    // the next session after a pause, so the radio rests between bursts.
    esp_ble_gap_start_scanning(kScanSessionSeconds);
}

void on_scan_result(esp_ble_gap_cb_param_t *param)
{
    if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
        return;
    }
    const uint8_t *adv = param->scan_rst.ble_adv;
    const uint8_t adv_len = param->scan_rst.adv_data_len;
    // ble_adv holds the advertising data immediately followed by the scan
    // response, so the GoPro UUID can live in either part: always inspect the
    // combined buffer instead of the advertising chunk alone.
    const uint8_t total_len = static_cast<uint8_t>(adv_len + param->scan_rst.scan_rsp_len);
    const bool is_gopro = adv_has_gopro_service(adv, total_len);

    int model_raw = -1;
    char name[32] = {0};
    adv_parse(adv, total_len, &model_raw, name, sizeof(name));

    // Keep the diagnostics snapshot fresh even after we are connected.
    lock();
    s_status.adv_seen++;
    s_status.last_adv_rssi = param->scan_rst.rssi;
    s_status.last_adv_is_gopro = is_gopro;
    std::strncpy(s_status.last_adv_name, name, sizeof(s_status.last_adv_name) - 1);
    s_status.last_adv_name[sizeof(s_status.last_adv_name) - 1] = '\0';
    const bool busy =
        (s_status.state == State::Connected || s_status.state == State::Connecting);
    unlock();

    if (!is_gopro) {
        // Verbose per-packet dump: invaluable while bringing the link up.
        ESP_LOGI(TAG, "adv %02X:%02X:%02X:%02X:%02X:%02X rssi=%d name='%s'",
                 param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                 param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5],
                 param->scan_rst.rssi, name);
        log_adv_hex("  adv", adv, adv_len);
        if (param->scan_rst.scan_rsp_len > 0) {
            log_adv_hex("  rsp", adv + adv_len, param->scan_rst.scan_rsp_len);
        }
        return;
    }

    if (busy) {
        return;
    }

    lock();
    s_status.raw_model_id = static_cast<uint8_t>(model_raw < 0 ? 0 : model_raw);
    s_status.model_id = model_index_from_raw(model_raw);
    std::strncpy(s_status.device_name, name, sizeof(s_status.device_name) - 1);
    s_status.device_name[sizeof(s_status.device_name) - 1] = '\0';
    std::memcpy(s_remote_bda, param->scan_rst.bda, sizeof(esp_bd_addr_t));
    const esp_ble_addr_type_t addr_type =
        static_cast<esp_ble_addr_type_t>(param->scan_rst.ble_addr_type);
    unlock();

    set_state(State::Connecting);

    ESP_LOGI(TAG, "GoPro found: %s (%s), connecting...", name,
             model_id_string(s_status.model_id));

    // Request the preferred connection interval before opening the link (the
    // parameter only takes effect in the master role and only before connecting).
    esp_ble_gap_set_prefer_conn_params(s_remote_bda, kPreferredConnIntMin,
                                       kPreferredConnIntMax, kPreferredConnLatency,
                                       kPreferredConnTimeout);

    esp_ble_gap_stop_scanning();
    esp_ble_gattc_open(s_gattc_if, s_remote_bda, addr_type, true);
}

void discover_characteristics()
{
    // First ask how many characteristics the service exposes, then fetch them.
    uint16_t count = 0;
    const esp_gatt_status_t count_err = esp_ble_gattc_get_attr_count(
        s_gattc_if, s_conn_id, ESP_GATT_DB_CHARACTERISTIC,
        s_service.start_handle, s_service.end_handle, 0, &count);
    if (count_err != ESP_OK || count == 0) {
        ESP_LOGW(TAG, "no characteristics in GP-Control service");
        return;
    }

    auto *elems = static_cast<esp_gattc_char_elem_t *>(
        malloc(sizeof(esp_gattc_char_elem_t) * count));
    if (elems == nullptr) {
        return;
    }

    uint16_t elem_count = count;
    if (esp_ble_gattc_get_all_char(s_gattc_if, s_conn_id, s_service.start_handle,
                                   s_service.end_handle, elems, &elem_count, 0) == ESP_OK) {
        for (uint16_t i = 0; i < elem_count; ++i) {
            if (elems[i].uuid.len != ESP_UUID_LEN_128) {
                continue;
            }
            for (int c = 0; c < kCharCount; ++c) {
                esp_bt_uuid_t target = {};
                make_char_uuid(kCharIds[c], &target);
                if (std::memcmp(elems[i].uuid.uuid.uuid128, target.uuid.uuid128, 16) == 0) {
                    s_char_handles[c] = elems[i].char_handle;
                }
            }
        }
    }
    free(elems);
}

void subscribe_notifications()
{
    const int resp_indexes[] = {IDX_COMMAND_RESP, IDX_SETTINGS_RESP, IDX_QUERY_RESP};
    for (int idx : resp_indexes) {
        if (s_char_handles[idx] != 0) {
            esp_ble_gattc_register_for_notify(s_gattc_if, s_remote_bda, s_char_handles[idx]);
        }
    }
}

void on_connected()
{
    lock();
    s_status.encoding = false;
    s_status.encoding_duration = 0;
    s_status.battery_percent = -1;
    unlock();

    set_state(State::Connected);
    ESP_LOGI(TAG, "Connected to GoPro (%s)", kModelNames[s_status.model_id]);

    // Light the onboard LED steadily to signal an established BLE link.
    led_solid(true);

    register_for_updates();

    if (s_keepalive_timer != nullptr) {
        esp_timer_start_periodic(s_keepalive_timer, kKeepAlivePeriodUs);
    }
}

void on_disconnected()
{
    if (s_keepalive_timer != nullptr) {
        esp_timer_stop(s_keepalive_timer);
    }
    lock();
    const bool was_connected = s_status.state == State::Connected;
    s_conn_id = kConnIdInvalid;
    s_service_found = false;
    std::memset(s_char_handles, 0, sizeof(s_char_handles));
    s_reply_len = 0;
    s_reply_expected = 0;
    s_status.state = was_connected ? State::Disconnected : State::Disconnected;
    const bool auto_reconnect = s_auto_reconnect;
    unlock();
    ESP_LOGW(TAG, "Disconnected from GoPro");

    // Slow-blink the onboard LED again now that the link is gone.
    led_blink_start();

    if (auto_reconnect && s_reconnect_timer != nullptr) {
        esp_timer_stop(s_reconnect_timer);
        esp_timer_start_once(s_reconnect_timer, 3LL * 1000 * 1000);
    }
}

void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                         esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        s_gattc_if = gattc_if;
        configure_and_scan();
        break;

    case ESP_GATTC_CONNECT_EVT:
        s_conn_id = param->connect.conn_id;
        std::memcpy(s_remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "Link established, requesting encryption...");
        esp_ble_set_encryption(s_remote_bda, ESP_BLE_SEC_ENCRYPT);
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "open failed, status %d", param->open.status);
            set_state(State::Failed);
            break;
        }
        s_conn_id = param->open.conn_id;
        ESP_LOGI(TAG, "GATT link open, searching services...");
        esp_ble_gattc_search_service(s_gattc_if, s_conn_id, nullptr);
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        // Filter for the GP-Control service (16-bit UUID FEA6).
        ESP_LOGI(TAG, "service found: uuid_len=%d start=%d end=%d", param->search_res.srvc_id.uuid.len,
                 param->search_res.start_handle, param->search_res.end_handle);
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
            param->search_res.srvc_id.uuid.uuid.uuid16 == kServiceUuid16) {
            s_service.uuid = param->search_res.srvc_id.uuid;
            s_service.start_handle = param->search_res.start_handle;
            s_service.end_handle = param->search_res.end_handle;
            s_service.is_primary = param->search_res.is_primary;
            s_service_found = true;
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (!s_service_found) {
            ESP_LOGE(TAG, "GP-Control service not found");
            set_state(State::Failed);
            esp_ble_gattc_close(s_gattc_if, s_conn_id);
            break;
        }
        ESP_LOGI(TAG, "GP-Control service discovered, handles %d..%d",
                 s_service.start_handle, s_service.end_handle);
        discover_characteristics();
        for (int c = 0; c < kCharCount; ++c) {
            ESP_LOGI(TAG, "char %d handle: %u", c, s_char_handles[c]);
        }
        subscribe_notifications();
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(TAG, "register for notify: status=%d handle=%d",
                 param->reg_for_notify.status, param->reg_for_notify.handle);
        if (param->reg_for_notify.status == ESP_GATT_OK) {
            // Enable the CCCD descriptor (it follows the value handle).
            uint8_t notify_en[2] = {0x01, 0x00};
            esp_ble_gattc_write_char_descr(
                s_gattc_if, s_conn_id, param->reg_for_notify.handle + 1,
                sizeof(notify_en), notify_en, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        }
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        // Once the first CCCD write completes we consider notifications ready.
        on_connected();
        break;

    case ESP_GATTC_NOTIFY_EVT:
        if (param->notify.handle == s_char_handles[IDX_COMMAND_RESP]) {
            // The camera acknowledged a command: release the pending flag so the
            // web UI can enable the button again.
            lock();
            s_command_pending = false;
            unlock();
        } else if (param->notify.handle == s_char_handles[IDX_QUERY_RESP] ||
                   param->notify.handle == s_char_handles[IDX_SETTINGS_RESP]) {
            decode_query(param->notify.value, param->notify.value_len);
        }
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        on_disconnected();
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(TAG, "MTU negotiated: %d", param->cfg_mtu.mtu);
        break;

    default:
        break;
    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        start_scan();
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan start failed");
        }
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        // A scan session ended. If we are still looking for the camera, wait a
        // bit and start the next session; if we stopped in order to connect the
        // state is no longer Scanning and we leave the radio idle.
        {
            lock();
            const State state = s_status.state;
            unlock();
            if (state == State::Scanning && s_scan_resume_timer != nullptr) {
                esp_timer_stop(s_scan_resume_timer);
                esp_timer_start_once(s_scan_resume_timer, kScanSessionPauseUs);
            }
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        on_scan_result(param);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "Pairing/encryption complete");
        } else {
            ESP_LOGW(TAG, "Pairing failed, reason 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;

    default:
        break;
    }
}

void keepalive_timer_cb(void *)
{
    keepalive();
}

void reconnect_timer_cb(void *)
{
    start_scan_if_needed();
}

// Fires after the pause between two scan sessions and starts the next one.
void scan_resume_timer_cb(void *)
{
    start_scan_if_needed();
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void init()
{
    if (s_mutex == nullptr) {
        s_mutex = xSemaphoreCreateMutex();
    }

    lock();
    s_status.state = State::Idle;
    s_status.model_id = 0;
    s_status.battery_percent = -1;
    unlock();

    // Prepare the onboard LED used as a connection indicator and start it
    // blinking until a camera connects.
    led_channel_init();
    led_blink_start();

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_event_handler));
    esp_ble_gattc_app_register(0);
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(512));

    // Security: "just works" bonding, no MITM.
    uint8_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    uint8_t iocap = ESP_IO_CAP_NONE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));

    const esp_timer_create_args_t keepalive_args = {
        .callback = &keepalive_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gopro_keepalive",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&keepalive_args, &s_keepalive_timer));

    const esp_timer_create_args_t reconnect_args = {
        .callback = &reconnect_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gopro_reconnect",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&reconnect_args, &s_reconnect_timer));

    const esp_timer_create_args_t scan_resume_args = {
        .callback = &scan_resume_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gopro_scan",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&scan_resume_args, &s_scan_resume_timer));

    // Scan parameters are applied once the GATT app is registered.
    ESP_LOGI(TAG, "BLE client initialised");
}

bool connect()
{
    lock();
    const State state = s_status.state;
    s_auto_reconnect = true;
    unlock();

    if (state == State::Connected || state == State::Connecting || state == State::Scanning) {
        return true;
    }
    start_scan_if_needed();
    return true;
}

void disconnect()
{
    lock();
    s_auto_reconnect = false;
    const uint16_t conn_id = s_conn_id;
    const State state = s_status.state;
    unlock();

    if (s_keepalive_timer != nullptr) {
        esp_timer_stop(s_keepalive_timer);
    }
    if (s_reconnect_timer != nullptr) {
        esp_timer_stop(s_reconnect_timer);
    }
    if (s_scan_resume_timer != nullptr) {
        esp_timer_stop(s_scan_resume_timer);
    }
    if (state == State::Scanning) {
        // Drop the radio out of the scan session we may be in the middle of.
        esp_ble_gap_stop_scanning();
    }
    if (conn_id != kConnIdInvalid) {
        esp_ble_gattc_close(s_gattc_if, conn_id);
    }
    set_state(State::Idle);
}

void shutter()
{
    lock();
    const bool recording = s_status.encoding;
    unlock();
    // Param 0x01 starts recording, 0x00 stops it (OpenGoPro SET_SHUTTER, Int8ub).
    const uint8_t param = recording ? 0x00 : 0x01;
    send_camera_command(COMMAND_SHUTTER, &param, 1, true);
}

void next_mode()
{
    // Cycle through the video / photo / timelapse preset groups. When the camera
    // has reported its current group we advance relative to it, so the button
    // stays in sync even if the mode was changed on the camera itself.
    lock();
    const bool known = s_status.preset_group_known;
    const uint32_t current = s_status.preset_group;
    unlock();

    int index = s_preset_group_index;
    if (known) {
        for (int i = 0; i < kPresetGroupCount; ++i) {
            if (kPresetGroups[i] == current) {
                index = (i + 1) % kPresetGroupCount;
                break;
            }
        }
    }
    s_preset_group_index = (index + 1) % kPresetGroupCount;
    const uint16_t group = kPresetGroups[index];
    ESP_LOGI(TAG, "Loading preset group %u", static_cast<unsigned>(group));

    // LOAD_PRESET_GROUP takes an unsigned 16-bit big-endian preset group id.
    const uint8_t params[2] = {
        static_cast<uint8_t>((group >> 8) & 0xFFu),
        static_cast<uint8_t>(group & 0xFFu),
    };
    send_camera_command(COMMAND_LOAD_PRESET_GROUP, params, sizeof(params), true);
}

bool copy_status(Status *out)
{
    if (out == nullptr) {
        return false;
    }
    lock();
    *out = s_status;
    unlock();
    return true;
}

const char *model_id_string(int model_id)
{
    if (model_id < 0 || model_id >= kModelTableSize) {
        return "Unknown";
    }
    return kModelNames[model_id];
}

const char *state_string(State state)
{
    switch (state) {
    case State::Idle: return "idle";
    case State::Scanning: return "scanning";
    case State::Connecting: return "connecting";
    case State::Connected: return "connected";
    case State::Disconnected: return "disconnected";
    case State::Failed: return "failed";
    default: return "unknown";
    }
}

// Human readable name for the preset group reported by the camera.
const char *preset_group_label(uint32_t group)
{
    switch (group) {
    case 1000: return "Video";
    case 1001: return "Photo";
    case 1002: return "Timelapse";
    default: return "-";
    }
}

int status_json(char *buf, size_t buf_size)
{
    Status st = {};
    copy_status(&st);

    char res[32];
    char fps[16];
    resolution_label(static_cast<uint8_t>(st.resolution_raw), res, sizeof(res));
    std::snprintf(fps, sizeof(fps), "%ufps", framerate_value(static_cast<uint8_t>(st.framerate_raw)));

    const int written = std::snprintf(
        buf, buf_size,
        "{"
        "\"state\":\"%s\","
        "\"connected\":%s,"
        "\"model\":\"%s\","
        "\"device\":\"%s\","
        "\"mode\":\"%s\","
        "\"mode_known\":%s,"
        "\"recording\":%s,"
        "\"encoding_duration\":%u,"
        "\"battery\":%d,"
        "\"sd_remaining\":%u,"
        "\"resolution\":\"%s\","
        "\"resolution_known\":%s,"
        "\"framerate\":\"%s\","
        "\"framerate_known\":%s,"
        "\"lens\":\"%s\","
        "\"lens_known\":%s,"
        "\"flicker\":\"%s\","
        "\"flicker_known\":%s,"
        "\"hypersmooth\":\"%s\","
        "\"hypersmooth_known\":%s,"
        "\"gps\":\"%s\","
        "\"gps_known\":%s,"
        "\"led\":\"%s\","
        "\"led_known\":%s,"
        "\"command_pending\":%s,"
        "\"adv_seen\":%u,"
        "\"last_adv_rssi\":%d,"
        "\"last_adv_is_gopro\":%s,"
        "\"last_adv_name\":\"%s\""
        "}",
        state_string(st.state),
        (st.state == State::Connected) ? "true" : "false",
        model_id_string(st.model_id),
        st.device_name,
        st.preset_group_known ? preset_group_label(st.preset_group) : "-",
        st.preset_group_known ? "true" : "false",
        st.encoding ? "true" : "false",
        static_cast<unsigned>(st.encoding_duration),
        st.battery_percent,
        static_cast<unsigned>(st.sd_remaining_known ? st.sd_remaining_seconds : 0),
        st.resolution_known ? res : "-",
        st.resolution_known ? "true" : "false",
        st.framerate_known ? fps : "-",
        st.framerate_known ? "true" : "false",
        st.lens_known ? lens_label(static_cast<uint8_t>(st.lens_raw)) : "-",
        st.lens_known ? "true" : "false",
        st.flicker_known ? flicker_label(static_cast<uint8_t>(st.flicker_raw)) : "-",
        st.flicker_known ? "true" : "false",
        st.hypersmooth_known ? hypersmooth_label(static_cast<uint8_t>(st.hypersmooth_raw)) : "-",
        st.hypersmooth_known ? "true" : "false",
        st.gps_known ? (st.gps_raw ? "On" : "Off") : "-",
        st.gps_known ? "true" : "false",
        st.led_known ? led_label(static_cast<uint8_t>(st.led_raw)) : "-",
        st.led_known ? "true" : "false",
        command_busy() ? "true" : "false",
        static_cast<unsigned>(st.adv_seen),
        st.last_adv_rssi,
        st.last_adv_is_gopro ? "true" : "false",
        st.last_adv_name);

    if (written < 0 || static_cast<size_t>(written) >= buf_size) {
        return -1;
    }
    return written;
}

}  // namespace gopro
