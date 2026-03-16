idf_component_register(
    SRCS "main.c" "hardware.c" "network.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        vl53l0x
        esp_wifi
        esp_http_server
        esp_event
        nvs_flash
        esp_netif
        freertos
        driver
)