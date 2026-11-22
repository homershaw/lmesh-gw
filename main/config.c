#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "cJSON.h"
#include "sdkconfig.h"

static const char *TAG = "cfg";
gw_cfg_t g_cfg;
char g_gw_id[8] = "0000";

#define MASK "********"

float e52_chan_mhz(uint8_t ch) { return 850.125f + (float)ch; }

void cfg_defaults(gw_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    strcpy(c->ap_pass, "lmesh1234");
    strcpy(c->hostname, "lmesh-gw");
    c->mqtt_en = false;
    strcpy(c->mqtt_uri, "mqtt://192.168.1.10:1883");
    c->e52_channel = 65;            /* 915.125 MHz, mid-band */
    c->e52_rate = 1;                /* 21.875 kbps: ~3.5x faster than 7k, moderate range */
    c->e52_power = 22;
    c->e52_panid = 0x1234;
    c->e52_addr = 0x0001;
    c->e52_csma = 40;               /* ms; 127 -> 40 cut 1-hop RTT median 371 -> 224 ms */
    c->uart_tx = CONFIG_LMGW_E52_TX_GPIO;
    c->uart_rx = CONFIG_LMGW_E52_RX_GPIO;
    c->uart_rst = CONFIG_LMGW_E52_RST_GPIO;
    c->uart_baud = CONFIG_LMGW_E52_BAUD;
    c->mb_port = 502;
    c->mb_timeout_ms = 6000;
    c->gw_lat = 0; c->gw_lon = 0;
    c->ant_gain_db = 4.0f;          /* two ~2 dBi whips */
    c->mb_map_n = 0;
}

static void jstr(cJSON *o, const char *k, char *dst, size_t n, bool secret)
{
    cJSON *v = cJSON_GetObjectItem(o, k);
    if (!cJSON_IsString(v)) return;
    if (secret && strcmp(v->valuestring, MASK) == 0) return;
    strncpy(dst, v->valuestring, n - 1);
    dst[n - 1] = 0;
}

static bool jnum(cJSON *o, const char *k, double *out)
{
    cJSON *v = cJSON_GetObjectItem(o, k);
    if (cJSON_IsNumber(v)) { *out = v->valuedouble; return true; }
    if (cJSON_IsString(v) && v->valuestring[0]) {   /* allow "0x1234" from forms */
        *out = (double)strtol(v->valuestring, NULL, 0);
        return true;
    }
    return false;
}

#define JN(key, field, lo, hi) do { double d; if (jnum(o, key, &d) && d >= (lo) && d <= (hi)) c->field = d; } while (0)

bool cfg_from_json(const char *json)
{
    cJSON *o = cJSON_Parse(json);
    if (!o) return false;
    gw_cfg_t *c = &g_cfg;

    jstr(o, "sta_ssid", c->sta_ssid, sizeof c->sta_ssid, false);
    jstr(o, "sta_pass", c->sta_pass, sizeof c->sta_pass, true);
    jstr(o, "ap_ssid", c->ap_ssid, sizeof c->ap_ssid, false);
    jstr(o, "ap_pass", c->ap_pass, sizeof c->ap_pass, true);
    jstr(o, "hostname", c->hostname, sizeof c->hostname, false);
    jstr(o, "web_pass", c->web_pass, sizeof c->web_pass, true);

    cJSON *b = cJSON_GetObjectItem(o, "mqtt_en");
    if (cJSON_IsBool(b)) c->mqtt_en = cJSON_IsTrue(b);
    jstr(o, "mqtt_uri", c->mqtt_uri, sizeof c->mqtt_uri, false);
    jstr(o, "mqtt_user", c->mqtt_user, sizeof c->mqtt_user, false);
    jstr(o, "mqtt_pass", c->mqtt_pass, sizeof c->mqtt_pass, true);
    jstr(o, "mqtt_base", c->mqtt_base, sizeof c->mqtt_base, false);

    JN("e52_channel", e52_channel, 52, 77);       /* keep inside 902-928 MHz */
    JN("e52_rate", e52_rate, 0, 2);
    JN("e52_power", e52_power, -9, 22);
    JN("e52_panid", e52_panid, 0, 0xFFFE);
    JN("e52_addr", e52_addr, 0, 0x7FFF);           /* gateway must be a router */
    JN("e52_csma", e52_csma, 20, 65535);
    JN("uart_tx", uart_tx, -1, 60);
    JN("uart_rx", uart_rx, -1, 60);
    JN("uart_rst", uart_rst, -1, 60);
    JN("uart_baud", uart_baud, 1200, 460800);
    JN("mb_port", mb_port, 1, 65535);
    JN("mb_timeout_ms", mb_timeout_ms, 500, 60000);
    JN("gw_lat", gw_lat, -90, 90);
    JN("gw_lon", gw_lon, -180, 180);
    JN("ant_gain_db", ant_gain_db, -20, 40);

    cJSON *m = cJSON_GetObjectItem(o, "mb_map");
    if (cJSON_IsArray(m)) {
        int n = 0;
        cJSON *e;
        cJSON_ArrayForEach(e, m) {
            if (n >= MB_MAP_MAX) break;
            double u, nd, du = 0;
            if (!jnum(e, "unit", &u) || !jnum(e, "node", &nd)) continue;
            jnum(e, "dst_unit", &du);
            c->mb_map[n].unit = (uint8_t)u;
            c->mb_map[n].node = (uint16_t)nd;
            c->mb_map[n].dst_unit = (uint8_t)du;
            n++;
        }
        c->mb_map_n = n;
    }
    cJSON_Delete(o);
    return true;
}

char *cfg_to_json(bool secrets)
{
    gw_cfg_t *c = &g_cfg;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "sta_ssid", c->sta_ssid);
    cJSON_AddStringToObject(o, "sta_pass", secrets ? c->sta_pass : (c->sta_pass[0] ? MASK : ""));
    cJSON_AddStringToObject(o, "ap_ssid", c->ap_ssid);
    cJSON_AddStringToObject(o, "ap_pass", secrets ? c->ap_pass : MASK);
    cJSON_AddStringToObject(o, "hostname", c->hostname);
    cJSON_AddStringToObject(o, "web_pass", secrets ? c->web_pass : (c->web_pass[0] ? MASK : ""));
    cJSON_AddBoolToObject(o, "mqtt_en", c->mqtt_en);
    cJSON_AddStringToObject(o, "mqtt_uri", c->mqtt_uri);
    cJSON_AddStringToObject(o, "mqtt_user", c->mqtt_user);
    cJSON_AddStringToObject(o, "mqtt_pass", secrets ? c->mqtt_pass : (c->mqtt_pass[0] ? MASK : ""));
    cJSON_AddStringToObject(o, "mqtt_base", c->mqtt_base);
    cJSON_AddNumberToObject(o, "e52_channel", c->e52_channel);
    cJSON_AddNumberToObject(o, "e52_rate", c->e52_rate);
    cJSON_AddNumberToObject(o, "e52_power", c->e52_power);
    cJSON_AddNumberToObject(o, "e52_panid", c->e52_panid);
    cJSON_AddNumberToObject(o, "e52_addr", c->e52_addr);
    cJSON_AddNumberToObject(o, "e52_csma", c->e52_csma);
    cJSON_AddNumberToObject(o, "uart_tx", c->uart_tx);
    cJSON_AddNumberToObject(o, "uart_rx", c->uart_rx);
    cJSON_AddNumberToObject(o, "uart_rst", c->uart_rst);
    cJSON_AddNumberToObject(o, "uart_baud", c->uart_baud);
    cJSON_AddNumberToObject(o, "mb_port", c->mb_port);
    cJSON_AddNumberToObject(o, "mb_timeout_ms", c->mb_timeout_ms);
    cJSON_AddNumberToObject(o, "gw_lat", c->gw_lat);
    cJSON_AddNumberToObject(o, "gw_lon", c->gw_lon);
    cJSON_AddNumberToObject(o, "ant_gain_db", c->ant_gain_db);
    cJSON *m = cJSON_AddArrayToObject(o, "mb_map");
    for (int i = 0; i < c->mb_map_n; i++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddNumberToObject(e, "unit", c->mb_map[i].unit);
        cJSON_AddNumberToObject(e, "node", c->mb_map[i].node);
        cJSON_AddNumberToObject(e, "dst_unit", c->mb_map[i].dst_unit);
        cJSON_AddItemToArray(m, e);
    }
    char *s = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    return s;
}

void cfg_load(void)
{
    cfg_defaults(&g_cfg);
    nvs_handle_t h;
    if (nvs_open("lmgw", NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "no stored config, using defaults");
        return;
    }
    size_t len = 0;
    if (nvs_get_str(h, "cfg", NULL, &len) == ESP_OK && len > 1) {
        char *buf = malloc(len);
        if (buf && nvs_get_str(h, "cfg", buf, &len) == ESP_OK) {
            cfg_from_json(buf);
            ESP_LOGI(TAG, "config loaded (%u bytes)", (unsigned)len);
        }
        free(buf);
    }
    nvs_close(h);
}

bool cfg_save(void)
{
    char *s = cfg_to_json(true);
    if (!s) return false;
    nvs_handle_t h;
    esp_err_t err = nvs_open("lmgw", NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_str(h, "cfg", s);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    free(s);
    if (err != ESP_OK) ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}
