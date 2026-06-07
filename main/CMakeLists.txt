idf_component_register(
    SRCS "c6_telemetry_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES
           esp_wifi
           esp_event
           esp_netif
           nvs_flash
           esp_http_server
)                
