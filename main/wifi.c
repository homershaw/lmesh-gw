#include <string.h>
#include <stdio.h>
#include "gw.h"
#include "config.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "wifi";
static esp_netif_t *s_sta, *s_ap;
static volatile bool s_connected;
static volatile int s_retry;
static esp_timer_handle_t s_reconn;

static void reconn_cb(void *a) { esp_wifi_connect(); }

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (g_cfg.sta_ssid[0]) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_connected) mqtt_gw_on_network(false);
        s_connected = false;
        if (g_cfg.sta_ssid[0]) {
            /* back off: 1 s .. 30 s */
            int d = s_retry < 30 ? s_retry + 1 : 30;
            s_retry++;
            esp_timer_stop(s_reconn);
            esp_timer_start_once(s_reconn, (uint64_t)d * 1000000ULL);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "STA got IP " IPSTR, IP2STR(&e->ip_info.ip));
        s_connected = true;
        s_retry = 0;
        mqtt_gw_on_network(true);
    }
}

bool wifi_sta_connected(void) { return s_connected; }

void wifi_status(char *sta_ip, size_t n1, char *ap_ip, size_t n2, int *rssi)
{
    esp_netif_ip_info_t ip;
    snprintf(sta_ip, n1, "-");
    if (s_connected && esp_netif_get_ip_info(s_sta, &ip) == ESP_OK) snprintf(sta_ip, n1, IPSTR, IP2STR(&ip.ip));
    if (esp_netif_get_ip_info(s_ap, &ip) == ESP_OK) snprintf(ap_ip, n2, IPSTR, IP2STR(&ip.ip));
    wifi_ap_record_t r;
    *rssi = (s_connected && esp_wifi_sta_get_ap_info(&r) == ESP_OK) ? r.rssi : 0;
}

void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta = esp_netif_create_default_wifi_sta();
    s_ap = esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(s_sta, g_cfg.hostname);

    const esp_timer_create_args_t ta = { .callback = reconn_cb, .name = "wifi_rc" };
    esp_timer_create(&ta, &s_reconn);

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(g_gw_id, sizeof g_gw_id, "%02X%02X", mac[4], mac[5]);

    wifi_config_t ap = { 0 };
    if (g_cfg.ap_ssid[0]) strncpy((char *)ap.ap.ssid, g_cfg.ap_ssid, sizeof ap.ap.ssid);
    else snprintf((char *)ap.ap.ssid, sizeof ap.ap.ssid, "LMESH-GW-%s", g_gw_id);
    ap.ap.ssid_len = strlen((char *)ap.ap.ssid);
    strncpy((char *)ap.ap.password, g_cfg.ap_pass, sizeof ap.ap.password);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = strlen(g_cfg.ap_pass) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    wifi_config_t sta = { 0 };
    strncpy((char *)sta.sta.ssid, g_cfg.sta_ssid, sizeof sta.sta.ssid);
    strncpy((char *)sta.sta.password, g_cfg.sta_pass, sizeof sta.sta.password);
    sta.sta.threshold.authmode = g_cfg.sta_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP '%s' up at 192.168.4.1, STA ssid '%s'", ap.ap.ssid, g_cfg.sta_ssid);
}
