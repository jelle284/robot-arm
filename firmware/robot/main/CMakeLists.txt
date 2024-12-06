idf_component_register(
    SRCS "parse_moves.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_netif
        esp_event
        esp_wifi
        wifi_provisioning
        mqtt
        json
        motion
        esp_timer
)
