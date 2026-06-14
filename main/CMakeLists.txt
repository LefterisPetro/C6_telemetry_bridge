idf_component_register(
    # Source files that belong to this component.
    SRCS "c6_telemetry_bridge.c"

    # Local include directory for this component.
    INCLUDE_DIRS "."

    #Public ESP-IDF components required by this component.
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_http_server
        esp_driver_uart
        esp_timer
)