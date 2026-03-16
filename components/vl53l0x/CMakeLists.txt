idf_component_register(
			SRCS "vl53l0x.c"
			INCLUDE_DIRS "include"
			REQUIRES driver esp_timer esp_wifi esp_http_server esp_event nvs_flash esp_netif freertos
)
