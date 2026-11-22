set(reqs nvs_flash esp_wifi esp_netif esp_event esp_http_server
         esp_driver_uart esp_driver_gpio lwip esp_timer app_update esp_app_format)
if(IDF_VERSION_MAJOR LESS 6)
    list(APPEND reqs mqtt json)
endif()
idf_component_register(
    SRCS "main.c" "config.c" "../shared/e52.c" "../shared/lmesh.c" "modbus_gw.c" "meshrate.c" "../shared/ota.c"
         "mqtt_gw.c" "wifi.c" "web.c" "udp_bridge.c"
    INCLUDE_DIRS "." "../shared"
    EMBED_TXTFILES "web/index.html"
    PRIV_REQUIRES ${reqs})
