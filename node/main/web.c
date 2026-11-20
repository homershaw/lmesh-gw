#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "e52.h"
#include "lmesh.h"
#include "rtu.h"
#include "node.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "cJSON.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "web";
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

static esp_err_t send_json(httpd_req_t *r, cJSON *o)
{
    char *s = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_sendstr(r, s ? s : "{}");
    free(s);
    return e;
}

static esp_err_t send_str_json(httpd_req_t *r, char *s)
{
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_sendstr(r, s ? s : "{}");
    free(s);
    return e;
}

static char *read_body(httpd_req_t *r, int max)
{
    if (r->content_len <= 0 || r->content_len > max) return NULL;
    char *b = malloc(r->content_len + 1);
    if (!b) return NULL;
    int got = 0;
    while (got < r->content_len) {
        int n = httpd_req_recv(r, b + got, r->content_len - got);
        if (n <= 0) { free(b); return NULL; }
        got += n;
    }
    b[got] = 0;
    return b;
}

static cJSON *read_json(httpd_req_t *r)
{
    char *b = read_body(r, 4096);
    if (!b) return NULL;
    cJSON *o = cJSON_Parse(b);
    free(b);
    return o;
}

static int jint(cJSON *o, const char *k, int def)
{
    cJSON *v = cJSON_GetObjectItem(o, k);
    if (cJSON_IsNumber(v)) return v->valueint;
    if (cJSON_IsString(v) && v->valuestring[0]) return (int)strtol(v->valuestring, NULL, 0);
    return def;
}

/* ---------------- login (HTTP Basic, user "admin") ---------------- */

static void b64(const char *in, char *out)
{
    static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t n = strlen(in), i = 0;
    for (; i + 2 < n; i += 3) {
        uint32_t v = (uint8_t)in[i] << 16 | (uint8_t)in[i + 1] << 8 | (uint8_t)in[i + 2];
        *out++ = T[v >> 18]; *out++ = T[(v >> 12) & 63]; *out++ = T[(v >> 6) & 63]; *out++ = T[v & 63];
    }
    if (i < n) {
        uint32_t v = (uint8_t)in[i] << 16 | (i + 1 < n ? (uint8_t)in[i + 1] << 8 : 0);
        *out++ = T[v >> 18]; *out++ = T[(v >> 12) & 63];
        *out++ = i + 1 < n ? T[(v >> 6) & 63] : '=';
        *out++ = '=';
    }
    *out = 0;
}

static bool auth_ok(httpd_req_t *r)
{
    const char *pw = g_cfg.web_pass[0] ? g_cfg.web_pass : g_cfg.ap_pass;
    if (!pw[0]) return true;
    char cred[80], want[120], got[128];
    snprintf(cred, sizeof cred, "admin:%s", pw);
    strcpy(want, "Basic ");
    b64(cred, want + 6);
    bool ok = httpd_req_get_hdr_value_str(r, "Authorization", got, sizeof got) == ESP_OK;
    if (ok) {
        size_t lw = strlen(want), lg = strlen(got);
        uint8_t d = lw != lg;
        for (size_t i = 0; i < lw && i < lg; i++) d |= want[i] ^ got[i];
        ok = d == 0;
    }
    if (!ok) {
        httpd_resp_set_status(r, "401 Unauthorized");
        httpd_resp_set_hdr(r, "WWW-Authenticate", "Basic realm=\"LMESH node\"");
        httpd_resp_sendstr(r, "login required");
    }
    return ok;
}

/* ---------------- handlers ---------------- */

static esp_err_t h_index(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html");
    return httpd_resp_send(r, index_html_start, index_html_end - index_html_start - 1);
}

static esp_err_t h_status(httpd_req_t *r)
{
    cJSON *o = cJSON_CreateObject();
    e52_stats_t es; e52_get_stats(&es);
    rtu_stats_t rs; rtu_get_stats(&rs);
    node_stats_t ns; node_get_stats(&ns);
    uint32_t up = esp_timer_get_time() / 1000000LL;
    char h[8], f[16];
    cJSON_AddStringToObject(o, "fw", NODE_FW_VERSION);
    cJSON_AddStringToObject(o, "id", g_node_id);
    cJSON_AddStringToObject(o, "site", g_cfg.site);
    cJSON_AddStringToObject(o, "name", g_cfg.name);
    cJSON_AddNumberToObject(o, "uptime", up);
    cJSON_AddNumberToObject(o, "heap", esp_get_free_heap_size());
    snprintf(h, sizeof h, "%04X", g_cfg.e52_addr); cJSON_AddStringToObject(o, "addr", h);
    snprintf(h, sizeof h, "%04X", ns.gw_addr); cJSON_AddStringToObject(o, "gw", h);
    cJSON_AddNumberToObject(o, "gw_seen", ns.gw_seen_s ? (double)(up - ns.gw_seen_s) : -1);
    snprintf(f, sizeof f, "%.3f", e52_chan_mhz(g_cfg.e52_channel)); cJSON_AddStringToObject(o, "freq", f);
    cJSON_AddNumberToObject(o, "rate", g_cfg.e52_rate);
    cJSON_AddNumberToObject(o, "tx", es.tx_frames);
    cJSON_AddNumberToObject(o, "rx", es.rx_frames);
    cJSON_AddNumberToObject(o, "tx_fail", es.tx_fail);
    cJSON_AddNumberToObject(o, "at_err", es.at_err);
    cJSON_AddNumberToObject(o, "rtu_ok", rs.ok);
    cJSON_AddNumberToObject(o, "rtu_exc", rs.exceptions);
    cJSON_AddNumberToObject(o, "rtu_timeout", rs.timeouts);
    cJSON_AddNumberToObject(o, "rtu_crc", rs.crc_err + rs.bad);
    cJSON_AddNumberToObject(o, "mb_reqs", ns.mb_reqs);
    cJSON_AddNumberToObject(o, "mb_dups", ns.mb_dups);
    cJSON_AddNumberToObject(o, "polls", ns.polls);
    cJSON_AddNumberToObject(o, "pubs", ns.pubs);
    cJSON_AddNumberToObject(o, "pub_fail", ns.pub_fail);
    cJSON_AddNumberToObject(o, "last_pub", ns.last_pub_s ? (double)(up - ns.last_pub_s) : -1);
    return send_json(r, o);
}

static esp_err_t h_values(httpd_req_t *r) { return send_str_json(r, node_values_json()); }
static esp_err_t h_cfg_get(httpd_req_t *r) { return send_str_json(r, cfg_to_json(false)); }

static esp_err_t h_cfg_post(httpd_req_t *r)
{
    char *b = read_body(r, 8192);
    if (!b) return httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "bad body");
    char old_site[40], old_name[24];
    strcpy(old_site, g_cfg.site); strcpy(old_name, g_cfg.name);
    bool ok = cfg_from_json(b) && cfg_save();
    free(b);
    if (ok && (strcmp(old_site, g_cfg.site) || strcmp(old_name, g_cfg.name))) node_identity_changed();
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", ok);
    cJSON_AddStringToObject(o, "note", "Saved. Site/poll list apply now; radio needs 'Apply to E52'; Wi-Fi and RS-485 need reboot.");
    return send_json(r, o);
}

static esp_err_t h_e52_apply(httpd_req_t *r)
{
    char *log = calloc(1, 1024);
    if (!log) return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    esp_err_t e = e52_apply_config(log, 1024);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", e == ESP_OK);
    cJSON_AddStringToObject(o, "log", log);
    free(log);
    return send_json(r, o);
}

static esp_err_t h_e52_at(httpd_req_t *r)
{
    cJSON *in = read_json(r);
    cJSON *c = in ? cJSON_GetObjectItem(in, "cmd") : NULL;
    cJSON *o = cJSON_CreateObject();
    if (cJSON_IsString(c) && strncmp(c->valuestring, "AT+", 3) == 0 && strlen(c->valuestring) < 64) {
        char resp[512];
        esp_err_t e = e52_at(c->valuestring, resp, sizeof resp, 1500);
        if (strncmp(c->valuestring, "AT+RESET", 8) == 0 || strncmp(c->valuestring, "AT+DEFAULT", 10) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1500));
            e52_runtime_setup();
        }
        cJSON_AddBoolToObject(o, "ok", e == ESP_OK);
        cJSON_AddStringToObject(o, "resp", e == ESP_ERR_TIMEOUT ? "(timeout)" : resp);
    } else {
        cJSON_AddBoolToObject(o, "ok", false);
        cJSON_AddStringToObject(o, "resp", "command must start with AT+");
    }
    cJSON_Delete(in);
    return send_json(r, o);
}

static esp_err_t h_ping(httpd_req_t *r)
{
    cJSON *in = read_json(r);
    node_stats_t ns; node_get_stats(&ns);
    int node = in ? jint(in, "node", ns.gw_addr) : ns.gw_addr;
    cJSON_Delete(in);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "rtt", lmesh_ping(node, 8000));
    return send_json(r, o);
}

/* local RS-485 test: {"unit":1,"fc":3,"addr":0,"count":4} or fc 6/16 with "values":[..] */
static esp_err_t h_rtu(httpd_req_t *r)
{
    cJSON *in = read_json(r);
    cJSON *o = cJSON_CreateObject();
    if (!in) { cJSON_AddStringToObject(o, "err", "bad json"); return send_json(r, o); }
    int unit = jint(in, "unit", 1), fc = jint(in, "fc", 3), addr = jint(in, "addr", 0), cnt = jint(in, "count", 1);
    cJSON *vals = cJSON_GetObjectItem(in, "values");
    int nv = cJSON_IsArray(vals) ? cJSON_GetArraySize(vals) : 0;
    uint8_t pdu[256]; int pl = 0;
    pdu[pl++] = fc; pdu[pl++] = addr >> 8; pdu[pl++] = addr & 0xFF;
    if (fc >= 1 && fc <= 4) {
        if (cnt < 1 || cnt > (fc <= 2 ? 2000 : 125)) goto bad;
        pdu[pl++] = cnt >> 8; pdu[pl++] = cnt & 0xFF;
    } else if (fc == 5 && nv) {
        pdu[pl++] = cJSON_GetArrayItem(vals, 0)->valueint ? 0xFF : 0; pdu[pl++] = 0;
    } else if (fc == 6 && nv) {
        int v = cJSON_GetArrayItem(vals, 0)->valueint; pdu[pl++] = (v >> 8) & 0xFF; pdu[pl++] = v & 0xFF;
    } else if (fc == 16 && nv && nv <= 123) {
        pdu[pl++] = 0; pdu[pl++] = nv; pdu[pl++] = nv * 2;
        for (int i = 0; i < nv; i++) { int v = cJSON_GetArrayItem(vals, i)->valueint; pdu[pl++] = (v >> 8) & 0xFF; pdu[pl++] = v & 0xFF; }
    } else goto bad;
    {
        uint8_t rsp[256];
        int64_t t0 = esp_timer_get_time();
        int n = rtu_txn(unit, pdu, pl, rsp, sizeof rsp, 0);
        cJSON_AddNumberToObject(o, "ms", (double)((esp_timer_get_time() - t0) / 1000));
        if (n < 0) {
            cJSON_AddStringToObject(o, "err", n == RTU_ERR_TIMEOUT ? "timeout (no reply on RS-485)" :
                                              n == RTU_ERR_CRC ? "CRC error (baud/parity? noise?)" : "unexpected reply");
        } else if (rsp[0] & 0x80) {
            char e[48]; snprintf(e, sizeof e, "Modbus exception %u", n > 1 ? rsp[1] : 0);
            cJSON_AddStringToObject(o, "err", e);
        } else {
            char hex[3 * 256 + 1]; int hl = 0;
            for (int i = 0; i < n; i++) hl += sprintf(hex + hl, "%02X ", rsp[i]);
            cJSON_AddStringToObject(o, "raw", hex);
            cJSON *a = cJSON_AddArrayToObject(o, "values");
            if ((fc == 3 || fc == 4) && n >= 2)
                for (int i = 0; i < rsp[1] / 2 && 3 + i * 2 < n; i++) cJSON_AddItemToArray(a, cJSON_CreateNumber((rsp[2 + i * 2] << 8) | rsp[3 + i * 2]));
            if ((fc == 1 || fc == 2) && n >= 2)
                for (int i = 0; i < cnt && 2 + i / 8 < n; i++) cJSON_AddItemToArray(a, cJSON_CreateNumber((rsp[2 + i / 8] >> (i % 8)) & 1));
            cJSON_AddBoolToObject(o, "ok", true);
        }
    }
    cJSON_Delete(in);
    return send_json(r, o);
bad:
    cJSON_Delete(in);
    cJSON_AddStringToObject(o, "err", "bad fc/count/values");
    return send_json(r, o);
}

static esp_err_t h_poll(httpd_req_t *r)
{
    node_poll_now();
    return httpd_resp_sendstr(r, "{\"ok\":true}");
}

static void reboot_task(void *a) { vTaskDelay(pdMS_TO_TICKS(500)); esp_restart(); }

static esp_err_t h_reboot(httpd_req_t *r)
{
    httpd_resp_sendstr(r, "{\"ok\":true}");
    xTaskCreate(reboot_task, "rb", 2048, NULL, 5, NULL);
    return ESP_OK;
}

/* ---- slow handlers on worker tasks so the page stays responsive ---- */
#define WEB_WORKERS 2
static QueueHandle_t s_async_q;

static void async_worker(void *arg)
{
    for (;;) {
        httpd_req_t *r;
        xQueueReceive(s_async_q, &r, portMAX_DELAY);
        ((esp_err_t (*)(httpd_req_t *))r->user_ctx)(r);
        httpd_req_async_handler_complete(r);
    }
}

static esp_err_t h_sync(httpd_req_t *r)
{
    if (!auth_ok(r)) return ESP_OK;
    return ((esp_err_t (*)(httpd_req_t *))r->user_ctx)(r);
}

static esp_err_t h_async(httpd_req_t *r)
{
    if (!auth_ok(r)) return ESP_OK;
    httpd_req_t *copy;
    if (httpd_req_async_handler_begin(r, &copy) != ESP_OK)
        return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    if (xQueueSend(s_async_q, &copy, 0) != pdTRUE) {
        httpd_req_async_handler_complete(copy);
        httpd_resp_set_status(r, "503 Busy");
        return httpd_resp_sendstr(r, "{\"err\":\"busy, try again\"}");
    }
    return ESP_OK;
}

static esp_err_t on_open(httpd_handle_t hd, int fd)
{
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    return ESP_OK;
}

void web_start(void)
{
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.stack_size = 8192;
    c.max_uri_handlers = 16;
    c.lru_purge_enable = true;
    c.open_fn = on_open;
    s_async_q = xQueueCreate(4, sizeof(httpd_req_t *));
    for (int i = 0; i < WEB_WORKERS; i++) xTaskCreate(async_worker, "web_async", 8192, NULL, 5, NULL);
    httpd_handle_t s;
    if (httpd_start(&s, &c) != ESP_OK) { ESP_LOGE(TAG, "httpd start failed"); return; }
    const httpd_uri_t u[] = {
        { .uri = "/", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_index },
        { .uri = "/api/status", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_status },
        { .uri = "/api/values", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_values },
        { .uri = "/api/config", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_cfg_get },
        { .uri = "/api/config", .method = HTTP_POST, .handler = h_sync, .user_ctx = h_cfg_post },
        { .uri = "/api/poll", .method = HTTP_POST, .handler = h_sync, .user_ctx = h_poll },
        { .uri = "/api/reboot", .method = HTTP_POST, .handler = h_sync, .user_ctx = h_reboot },
        { .uri = "/api/e52/apply", .method = HTTP_POST, .handler = h_async, .user_ctx = h_e52_apply },
        { .uri = "/api/e52/at", .method = HTTP_POST, .handler = h_async, .user_ctx = h_e52_at },
        { .uri = "/api/ping", .method = HTTP_POST, .handler = h_async, .user_ctx = h_ping },
        { .uri = "/api/rtu", .method = HTTP_POST, .handler = h_async, .user_ctx = h_rtu },
    };
    for (size_t i = 0; i < sizeof u / sizeof u[0]; i++) httpd_register_uri_handler(s, &u[i]);
    ESP_LOGI(TAG, "web UI on port 80");
}
