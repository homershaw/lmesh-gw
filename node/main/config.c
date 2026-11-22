#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "cJSON.h"
#include "sdkconfig.h"

static const char *TAG = "cfg";
node_cfg_t g_cfg;
SemaphoreHandle_t g_cfg_lock;
char g_node_id[8] = "0000";

#define MASK "********"

static const char *PT_NAMES[PT_COUNT] = { "u16", "s16", "u32", "s32", "f32", "u32sw", "s32sw", "f32sw", "bit" };

float e52_chan_mhz(uint8_t ch) { return 850.125f + (float)ch; }
const char *pt_name(uint8_t t) { return t < PT_COUNT ? PT_NAMES[t] : "u16"; }
int pt_regs(uint8_t t) { return (t >= PT_U32 && t <= PT_F32_SW) ? 2 : 1; }

static uint8_t pt_parse(const char *s)
{
    for (int i = 0; i < PT_COUNT; i++) if (strcmp(s, PT_NAMES[i]) == 0) return i;
    return PT_U16;
}

void cfg_defaults(node_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(g_node_id, sizeof g_node_id, "%02X%02X", mac[4], mac[5]);

    strcpy(c->ap_pass, "lmesh1234");
    strcpy(c->name, "lmesh-node");
    c->ext_ant = false;
    c->oled_sda = 20;            /* XIAO D9 */
    c->oled_scl = 18;            /* XIAO D10 */
    c->oled_sh1106 = true;       /* the 1.3" modules in use; SSD1306 selectable in Settings */
    /* must match the gateway */
    c->e52_channel = 65;            /* 915.125 MHz */
    c->e52_rate = 1;                /* 21.875 kbps */
    c->e52_power = 22;
    c->e52_panid = 0x1234;
    c->e52_csma = 40;
    c->gw_addr = 0x0001;
    /* unique-ish router address from the MAC: 0x0100..0x7FFE */
    c->e52_addr = 0x0100 + (((mac[4] << 8) | mac[5]) % 0x7EFF);
    c->uart_tx = CONFIG_LMGW_E52_TX_GPIO;
    c->uart_rx = CONFIG_LMGW_E52_RX_GPIO;
    c->uart_rst = CONFIG_LMGW_E52_RST_GPIO;
    c->uart_baud = CONFIG_LMGW_E52_BAUD;

    c->rs_baud = 9600;
    c->rs_parity = 0;
    c->rs_stop = 1;
    c->rtu_timeout_ms = 500;
    c->poll_s = 60;
    c->poll_n = 0;
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
        char *e;
        *out = strtod(v->valuestring, &e);
        if (*e) *out = (double)strtol(v->valuestring, NULL, 0);
        return true;
    }
    return false;
}

#define JN(key, field, lo, hi) do { double d; if (jnum(o, key, &d) && d >= (lo) && d <= (hi)) c->field = d; } while (0)

bool cfg_from_json(const char *json)
{
    cJSON *o = cJSON_Parse(json);
    if (!o) return false;
    xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
    node_cfg_t *c = &g_cfg;

    jstr(o, "site", c->site, sizeof c->site, false);
    jstr(o, "name", c->name, sizeof c->name, false);
    jstr(o, "ap_ssid", c->ap_ssid, sizeof c->ap_ssid, false);
    jstr(o, "ap_pass", c->ap_pass, sizeof c->ap_pass, true);
    jstr(o, "web_pass", c->web_pass, sizeof c->web_pass, true);
    cJSON *b = cJSON_GetObjectItem(o, "ext_ant");
    if (cJSON_IsBool(b)) c->ext_ant = cJSON_IsTrue(b);
    b = cJSON_GetObjectItem(o, "oled_sh1106");
    if (cJSON_IsBool(b)) c->oled_sh1106 = cJSON_IsTrue(b);
    b = cJSON_GetObjectItem(o, "oled_sda");
    if (cJSON_IsNumber(b) && b->valueint >= -1 && b->valueint <= 23) c->oled_sda = (int8_t)b->valueint;
    b = cJSON_GetObjectItem(o, "oled_scl");
    if (cJSON_IsNumber(b) && b->valueint >= -1 && b->valueint <= 23) c->oled_scl = (int8_t)b->valueint;

    JN("e52_channel", e52_channel, 52, 77);
    JN("e52_rate", e52_rate, 0, 2);
    JN("e52_power", e52_power, -9, 22);
    JN("e52_panid", e52_panid, 0, 0xFFFE);
    JN("e52_addr", e52_addr, 2, 0x7FFF);
    JN("e52_csma", e52_csma, 20, 65535);
    JN("gw_addr", gw_addr, 0, 0x7FFF);
    JN("uart_tx", uart_tx, -1, 30);
    JN("uart_rx", uart_rx, -1, 30);
    JN("uart_rst", uart_rst, -1, 30);
    JN("uart_baud", uart_baud, 1200, 921600);

    JN("rs_baud", rs_baud, 1200, 921600);
    JN("rs_parity", rs_parity, 0, 2);
    JN("rs_stop", rs_stop, 1, 2);
    JN("rtu_timeout_ms", rtu_timeout_ms, 50, 5000);
    JN("poll_s", poll_s, 0, 86400);

    cJSON *p = cJSON_GetObjectItem(o, "poll");
    if (cJSON_IsArray(p)) {
        int n = 0;
        cJSON *e;
        cJSON_ArrayForEach(e, p) {
            if (n >= POLL_MAX) break;
            poll_item_t *it = &c->poll[n];
            memset(it, 0, sizeof *it);
            double v;
            jstr(e, "name", it->name, sizeof it->name, false);
            if (!it->name[0]) continue;
            it->unit = jnum(e, "unit", &v) ? (uint8_t)v : 1;
            it->fc = jnum(e, "fc", &v) && v >= 1 && v <= 4 ? (uint8_t)v : 3;
            it->addr = jnum(e, "addr", &v) ? (uint16_t)v : 0;
            cJSON *t = cJSON_GetObjectItem(e, "type");
            it->type = cJSON_IsString(t) ? pt_parse(t->valuestring) : PT_U16;
            if (it->fc <= 2) it->type = PT_BIT;
            else if (it->type == PT_BIT) it->type = PT_U16;
            it->scale = jnum(e, "scale", &v) && v != 0 ? (float)v : 1.0f;
            it->offset = jnum(e, "offset", &v) ? (float)v : 0.0f;
            n++;
        }
        c->poll_n = n;
    }
    xSemaphoreGive(g_cfg_lock);
    cJSON_Delete(o);
    return true;
}

char *cfg_to_json(bool secrets)
{
    xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
    node_cfg_t *c = &g_cfg;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "site", c->site);
    cJSON_AddStringToObject(o, "name", c->name);
    cJSON_AddStringToObject(o, "ap_ssid", c->ap_ssid);
    cJSON_AddStringToObject(o, "ap_pass", secrets ? c->ap_pass : MASK);
    cJSON_AddStringToObject(o, "web_pass", secrets ? c->web_pass : (c->web_pass[0] ? MASK : ""));
    cJSON_AddBoolToObject(o, "ext_ant", c->ext_ant);
    cJSON_AddBoolToObject(o, "oled_sh1106", c->oled_sh1106);
    cJSON_AddNumberToObject(o, "oled_sda", c->oled_sda);
    cJSON_AddNumberToObject(o, "oled_scl", c->oled_scl);
    cJSON_AddNumberToObject(o, "e52_channel", c->e52_channel);
    cJSON_AddNumberToObject(o, "e52_rate", c->e52_rate);
    cJSON_AddNumberToObject(o, "e52_power", c->e52_power);
    cJSON_AddNumberToObject(o, "e52_panid", c->e52_panid);
    cJSON_AddNumberToObject(o, "e52_addr", c->e52_addr);
    cJSON_AddNumberToObject(o, "e52_csma", c->e52_csma);
    cJSON_AddNumberToObject(o, "gw_addr", c->gw_addr);
    cJSON_AddNumberToObject(o, "uart_tx", c->uart_tx);
    cJSON_AddNumberToObject(o, "uart_rx", c->uart_rx);
    cJSON_AddNumberToObject(o, "uart_rst", c->uart_rst);
    cJSON_AddNumberToObject(o, "uart_baud", c->uart_baud);
    cJSON_AddNumberToObject(o, "rs_baud", c->rs_baud);
    cJSON_AddNumberToObject(o, "rs_parity", c->rs_parity);
    cJSON_AddNumberToObject(o, "rs_stop", c->rs_stop);
    cJSON_AddNumberToObject(o, "rtu_timeout_ms", c->rtu_timeout_ms);
    cJSON_AddNumberToObject(o, "poll_s", c->poll_s);
    cJSON *p = cJSON_AddArrayToObject(o, "poll");
    for (int i = 0; i < c->poll_n; i++) {
        poll_item_t *it = &c->poll[i];
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "name", it->name);
        cJSON_AddNumberToObject(e, "unit", it->unit);
        cJSON_AddNumberToObject(e, "fc", it->fc);
        cJSON_AddNumberToObject(e, "addr", it->addr);
        cJSON_AddStringToObject(e, "type", pt_name(it->type));
        cJSON_AddNumberToObject(e, "scale", it->scale);
        cJSON_AddNumberToObject(e, "offset", it->offset);
        cJSON_AddItemToArray(p, e);
    }
    xSemaphoreGive(g_cfg_lock);
    char *s = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    return s;
}

void cfg_load(void)
{
    if (!g_cfg_lock) g_cfg_lock = xSemaphoreCreateMutex();
    cfg_defaults(&g_cfg);
    nvs_handle_t h;
    if (nvs_open("lmnode", NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "no stored config, using defaults (mesh addr 0x%04X)", g_cfg.e52_addr);
        return;
    }
    size_t len = 0;
    if (nvs_get_blob(h, "cfg", NULL, &len) == ESP_OK && len > 1) {
        char *buf = malloc(len);
        if (buf && nvs_get_blob(h, "cfg", buf, &len) == ESP_OK) {
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
    esp_err_t err = nvs_open("lmnode", NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, "cfg", s, strlen(s) + 1);   /* blob: poll list can exceed the 4000 B string limit */
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    free(s);
    if (err != ESP_OK) ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}
