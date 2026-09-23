idf_component_register(
    SRCS
        "main.cpp"
        "gopro_ble.cpp"
    INCLUDE_DIRS
        "."
    # The BLE (Bluetooth) component provides the ESP32 GATT-central stack
    # used by gopro_ble.cpp to talk to the GoPro GP-Control service.
    # The remaining components are used by main.cpp (Wi-Fi, HTTP server, NVS).
    REQUIRES
        bt
        esp_wifi
        esp_http_server
        esp_event
        esp_netif
        nvs_flash
        esp_timer
        # Drivers for the onboard status LED (RMT-driven WS2812 / plain GPIO).
        esp_driver_rmt
        esp_driver_gpio
    # index.html is embedded into the firmware at build time.
    # In the code it is available as _binary_index_html_start / _binary_index_html_end.
    EMBED_FILES
        "www/index.html"
)
