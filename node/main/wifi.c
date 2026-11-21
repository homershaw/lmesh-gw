#include <string.h>
#include <stdio.h>
#include "config.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

static const char *TAG = "wifi";

/* Seeed XIAO ESP32-C6: GPIO3 low powers the RF switch, GPIO14 picks the
 * antenna (low = on-board ceramic, high = U.FL connector). */
static void antenna_select(void)
{
#if CONFIG_LMN_XIAO_C6_ANT
    gpio_config_t g = { .pin_bit_mask = (1ULL << 3) | (1ULL << 14), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&g);
    gpio_set_level(3, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(14, g_cfg.ext_ant ? 1 : 0);
    ESP_LOGI(TAG, "Wi-Fi antenna: %s", g_cfg.ext_ant ? "external U.FL" : "on-board");
#endif
}

void wifi_start(void)
{
    antenna_select();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));

    wifi_config_t ap = { 0 };
    if (g_cfg.ap_ssid[0]) strncpy((char *)ap.ap.ssid, g_cfg.ap_ssid, sizeof ap.ap.ssid);
    else snprintf((char *)ap.ap.ssid, sizeof ap.ap.ssid, "LMESH-NODE-%s", g_node_id);
    ap.ap.ssid_len = strlen((char *)ap.ap.ssid);
    strncpy((char *)ap.ap.password, g_cfg.ap_pass, sizeof ap.ap.password);
    ap.ap.channel = 6;
    ap.ap.max_connection = 4;
    ap.ap.authmode = strlen(g_cfg.ap_pass) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "AP '%s' up at 192.168.4.1", ap.ap.ssid);
}
