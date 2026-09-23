# GoPro Remote (ESP32, ESP-IDF, C++)

An ESP-IDF (C++) project that brings up Wi-Fi, an HTTP server, and a BLE client
to a GoPro camera. The `index.html` web page acts as a remote control: it shows
the camera status and controls recording. The camera control buttons appear only
after a successful BLE connection.

## What's inside

```
gopro_remote/
├── CMakeLists.txt          # ESP-IDF project root CMake
├── main/
│   ├── CMakeLists.txt      # component build + EMBED_FILES www/index.html
│   ├── Kconfig.projbuild   # Wi-Fi and port settings (idf.py menuconfig)
│   ├── main.cpp            # Wi-Fi + HTTP server + HTTP API (C++)
│   ├── gopro_ble.h         # public BLE client interface
│   ├── gopro_ble.cpp       # GP-Control BLE client (central)
│   └── www/
│       └── index.html      # page: camera status + control buttons
├── sdkconfig.defaults      # esp32c3 target, USB Serial/JTAG console, Bluetooth
└── README.md
```

`index.html` is not stored in a filesystem — it is embedded into the firmware at
build time (`EMBED_FILES` in `main/CMakeLists.txt`) and served from flash. In code
it is available as the symbols `_binary_index_html_start` / `_binary_index_html_end`.

## HTTP API

| Method | Path              | Response                                                  |
|--------|-------------------|-----------------------------------------------------------|
| GET    | `/`               | the `index.html` page                                     |
| GET    | `/api/status`     | JSON snapshot of the camera status                        |
| POST   | `/api/connect`    | `{"status":"ok","state":"connecting"}` or `{"status":"busy"}` |
| POST   | `/api/disconnect` | `{"status":"ok","state":"idle"}`                          |
| POST   | `/api/shutter`    | `{"status":"ok"}` — start/stop recording (SET_SHUTTER)    |
| POST   | `/api/mode`       | `{"status":"ok"}` — next mode: video/photo/timelapse      |

The page requests `/api/status` every 2 seconds and refreshes the panel.
Example `/api/status` response:

```json
{
  "state": "connected",
  "connected": true,
  "model": "HERO13",
  "device": "GoPro 1234",
  "recording": false,
  "encoding_duration": 0,
  "battery": 87,
  "sd_remaining": 5400,
  "resolution": "4K",
  "resolution_known": true,
  "framerate": "30fps",
  "framerate_known": true,
  "lens": "Wide",
  "lens_known": true,
  "flicker": "60Hz",
  "flicker_known": true,
  "hypersmooth": "On",
  "hypersmooth_known": true,
  "gps": "On",
  "gps_known": true,
  "led": "Off",
  "led_known": true,
  "command_pending": false
}
```

The `command_pending` field is `true` until the camera acknowledges the last
command (`Action`/`Mode`). While it is `true`, the control buttons stay
disabled — this prevents sending a command "a second time in a row" before a
response arrives.

The `state` field takes the values: `idle`, `scanning`, `connecting`, `connected`,
`disconnected`, `failed`. The control buttons on the page are shown only when
`connected`.

## How it works over BLE

The ESP32 acts as the central device (GATT client) and connects to the camera
through the GoPro GP-Control service `0000FEA6-0000-1000-8000-00805F9B34FB`. Six
characteristics (command / command-response / settings / settings-response /
query / query-response) are derived from the base UUID by number `B5F9xxxx-...`
(values 72–77).

It uses the official **OpenGoPro V2** protocol: a command is transmitted as
`[cmd_id, param_len, params…]`, and packets longer than 20 bytes are split into
fragments with OpenGoPro headers (extended-13 / continuation). Older camera
firmware versions expected a `0xFF` prefix, which caused commands to the HERO13
to be ignored — now the correct V2 framing is used (see `main/gopro_ble.cpp`).

The workflow:

1. `gopro::init()` initializes the Bluetooth stack (Bluedroid) and starts
   scanning; the camera is found by an advertising packet with the GP-Control
   service.
2. `gopro::connect()` (the **Connect** button or `POST /api/connect`) initiates
   the connection and bonding.
3. After connecting, the client discovers the characteristics, subscribes to
   notifications, and registers the required settings/statuses (`query`), and
   also lights the onboard LED as an indicator of an active BLE connection.
4. The camera sends values as TLV messages; the client decodes them into the
   `Status` struct. A keep-alive is sent every 15 seconds, and on connection
   loss auto-reconnect is performed.
5. `gopro::shutter()` (the **Action** button) sends `SET_SHUTTER` (0x01) —
   start/stop recording depending on the current state.
6. `gopro::next_mode()` (the **Mode** button) sends `LOAD_PRESET_GROUP` (0x3E)
   with a 16-bit group identifier: video `1000`, photo `1001`,
   timelapse `1002`. The camera switches to its own preset group — its
   settings (resolution / lens / frame rate) are not changed.

> The **Action** and **Mode** buttons are locked while a command is being sent
> and are unlocked only after the camera responds — see the `command_pending`
> field in `/api/status`.

### Status LED

The onboard LED shows the BLE connection state: while the camera is not
connected, it **blinks slowly** (once per second), and after a successful
connection it is **steadily on**. When the connection is lost, it starts
blinking again. Settings in `menuconfig` (`GoPro Remote configuration`):

- `GOPRO_ONBOARD_LED` — enable the indication;
- `GOPRO_ONBOARD_LED_TYPE` — LED type: addressable `WS2812` (default)
  or plain `GPIO`;
- `GOPRO_ONBOARD_LED_GPIO` — pin number (default `8`);
- `GOPRO_ONBOARD_LED_ACTIVE_LOW` — for a plain LED with active-low level.

## Wi-Fi modes

Selected in `menuconfig` (`GoPro Remote configuration` → `Wi-Fi mode`):

- **AP (default)** — the ESP32 creates its own access point.
  - SSID: `ESP32-WebServer`, password: `12345678`
  - After connecting, open <http://192.168.4.1>
- **STA** — the ESP32 connects to an existing network. Specify the SSID/password;
  the device IP address is printed to the log (UART).

> The GoPro is controlled over BLE, so the Wi-Fi network is needed only to access
> the web remote, and the camera does not connect to it.

## Build and flash

The project is built for **ESP-IDF v5.3.1** and the **esp32c3** target
(board: ESP32-C3, 4 MB built-in flash, USB Serial/JTAG).

```bash
# 1. Activate the ESP-IDF environment
source ~/esp/v5.3.1/esp-idf/export.sh

# 2. (once) select the target
idf.py set-target esp32c3

# 3. Build
idf.py build

# 4. Flash (native USB port)
idf.py -p /dev/ttyACM0 flash

# 5. Monitor the log
idf.py -p /dev/ttyACM0 monitor
```

To exit the monitor: `Ctrl+]`.

> **About the console.** The board has no separate USB-UART bridge, so the log
> goes through the built-in USB Serial/JTAG. This is set in `sdkconfig.defaults`:
> `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`. If you switch to a board with an
> external USB-UART (or to a classic ESP32) — remove this line and select the
> `esp32` target.

> **Bluetooth** is enabled by the `CONFIG_BT_*` block in `sdkconfig.defaults`
> (Bluedroid, BLE central/GATT client, BLE 4.2 feature set). If Bluetooth was
> previously disabled, delete `sdkconfig` so the settings from `sdkconfig.defaults`
> are applied again, and run `idf.py build`.
>
> **Partition table.** The Bluedroid stack noticeably increases the image size, so
> the standard scheme with a single 1 MB `factory` partition is not enough. The
> project uses its own `partitions.csv` table (`factory` partition 1.9 MB),
> enabled via `CONFIG_PARTITION_TABLE_CUSTOM` in `sdkconfig.defaults`.

## Usage

1. Flash the board and connect to its Wi-Fi (AP or STA).
2. Open the web remote (by default <http://192.168.4.1>).
3. Turn on the GoPro camera and press **Connect**. When the state becomes
   `Connected to GoPro`, the **Shutter** and **Next mode** buttons appear.
4. Control recording; the status panel updates automatically.

## How to connect the camera (walkthrough)

The ESP32 looks for the camera by an advertising BLE packet with the GP-Control
service (`0xFEA6`), so it is important that the camera advertises this service.

1. **Turn on the camera.** It must be out of USB-connection mode and not
   actively recording "on the lock screen".
2. **Enable the camera's wireless interfaces** (on GoPro 12/13 this is
   *Preferences → Wireless Connections → Wireless*). The BLE command goes over
   `FEA6`, which is brought up only when wireless is enabled.
3. **Do not connect the camera to a phone (GoPro Quik) and do not keep the
   pairing screen open.** One camera — one active BLE connection: if the phone
   already holds a session, the ESP32 will not see the camera. Turn off Bluetooth
   on the phone or close the app.
4. **Open the web remote** and press **Connect**. The log (UART) and the
   *Scan debug* block on the page show what is happening with scanning.
5. Wait for the `Connected to GoPro` state. On the first connection the camera
   shows a pairing request — confirm it on the camera (the confirmation /
   Shutter button).

> **Why doesn't the "new device" show up in the camera search.** The ESP32 is a
> central device: it searches for the camera itself, not the other way around.
> The camera does not list the remote among "devices" — this is normal. The
> reverse search (the camera seeing the remote) is only possible in BLE
> peripheral mode (GATT server), while our project works as a GATT client.

## BLE diagnostics

If the camera is not found, look at the UART log (`idf.py monitor`) and the
*Scan debug* block on the web page:

- **Adv. packets seen** — how many advertising packets were processed. `0` means
  scanning is not running; a growing number means the air is visible.
- **Last RSSI / Last name / Last was GoPro** — parameters of the last packet.
  "Last was GoPro = No" with a growing counter means there is traffic, but the
  camera has either turned off wireless or is already taken by another master.
- Every packet is printed in the log: `adv <MAC> rssi=<..> name='..'` plus a hex
  dump of the advertisement (`adv`) and scan-response (`rsp`). Useful to make
  sure the bytes `A6 FE` (service `0xFEA6`) are actually present in the packet.

The device filter accepts the `0xFEA6` service both from 16-bit UUID lists
(types `0x02`/`0x03`) and from 128-bit ones (`0x06`/`0x07`), and scanning
merges advertisement and scan-response — the camera may advertise the UUID in
either of them.

## Wi-Fi configuration

```bash
idf.py menuconfig
# GoPro Remote configuration -> Wi-Fi mode / SSID / password / port
```

The default values are set in `main/Kconfig.projbuild`.
To check the current values, look at the generated `sdkconfig`.
