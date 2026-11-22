#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "gw.h"
#include "config.h"
#include "e52.h"
#include "lmesh.h"
#include "modbus_gw.h"
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

static cJSON *read_json(httpd_req_t *r)
{
    if (r->content_len <= 0 || r->content_len > 4096) return NULL;
    char *b = malloc(r->content_len + 1);
    if (!b) return NULL;
    int got = 0;
    while (got < r->content_len) {
        int n = httpd_req_recv(r, b + got, r->content_len - got);
        if (n <= 0) { free(b); return NULL; }
        got += n;
    }
    b[got] = 0;
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

/* true if allowed; otherwise a 401 has been sent */
static bool auth_ok(httpd_req_t *r)
{
    const char *pw = g_cfg.web_pass[0] ? g_cfg.web_pass : g_cfg.ap_pass;
    if (!pw[0]) return true;                      /* open AP and no web password */
    char cred[80], want[120], got[128];
    snprintf(cred, sizeof cred, "admin:%s", pw);
    strcpy(want, "Basic ");
    b64(cred, want + 6);
    bool ok = httpd_req_get_hdr_value_str(r, "Authorization", got, sizeof got) == ESP_OK;
    if (ok) {                                     /* length + constant-time compare */
        size_t lw = strlen(want), lg = strlen(got);
        uint8_t d = lw != lg;
        for (size_t i = 0; i < lw && i < lg; i++) d |= want[i] ^ got[i];
        ok = d == 0;
    }
    if (!ok) {
        httpd_resp_set_status(r, "401 Unauthorized");
        httpd_resp_set_hdr(r, "WWW-Authenticate", "Basic realm=\"LMESH gateway\"");
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
    char sta[20], ap[20]; int rssi;
    wifi_status(sta, sizeof sta, ap, sizeof ap, &rssi);
    e52_stats_t st; e52_get_stats(&st);
    uint32_t mr, mt, mc, mrt; modbus_gw_stats(&mr, &mt, &mc, &mrt);
    uint32_t up, dn; mqtt_gw_stats(&up, &dn);
    char ip[20]; lmesh_ip_str(g_cfg.e52_addr, ip, sizeof ip);
    cJSON_AddStringToObject(o, "fw", GW_FW_VERSION);
    cJSON_AddStringToObject(o, "id", g_gw_id);
    cJSON_AddNumberToObject(o, "uptime", (double)(esp_timer_get_time() / 1000000LL));
    cJSON_AddNumberToObject(o, "heap", esp_get_free_heap_size());
    cJSON_AddStringToObject(o, "sta_ip", sta);
    cJSON_AddStringToObject(o, "ap_ip", ap);
    cJSON_AddNumberToObject(o, "rssi", rssi);
    cJSON_AddBoolToObject(o, "mqtt", mqtt_gw_connected());
    cJSON_AddStringToObject(o, "mqtt_base", mqtt_gw_base());
    cJSON_AddNumberToObject(o, "mqtt_up", up);
    cJSON_AddNumberToObject(o, "mqtt_down", dn);
    cJSON_AddStringToObject(o, "mesh_ip", ip);
    char f[16]; snprintf(f, sizeof f, "%.3f", e52_chan_mhz(g_cfg.e52_channel));
    cJSON_AddStringToObject(o, "freq", f);
    cJSON_AddNumberToObject(o, "tx", st.tx_frames);
    cJSON_AddNumberToObject(o, "rx", st.rx_frames);
    cJSON_AddNumberToObject(o, "at_err", st.at_err);
    cJSON_AddNumberToObject(o, "tx_fail", st.tx_fail);
    cJSON_AddNumberToObject(o, "rx_junk", st.rx_junk);
    cJSON_AddNumberToObject(o, "mb_reqs", mr);
    cJSON_AddNumberToObject(o, "mb_timeouts", mt);
    cJSON_AddNumberToObject(o, "mb_clients", mc);
    cJSON_AddNumberToObject(o, "mb_retries", mrt);
    return send_json(r, o);
}

static esp_err_t h_cfg_get(httpd_req_t *r)
{
    char *s = cfg_to_json(false);
    httpd_resp_set_type(r, "application/json");
    esp_err_t e = httpd_resp_sendstr(r, s ? s : "{}");
    free(s);
    return e;
}

static esp_err_t h_cfg_post(httpd_req_t *r)
{
    int len = r->content_len;
    if (len <= 0 || len > 4096) return httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "bad length");
    char *b = malloc(len + 1);
    int got = 0;
    while (got < len) { int n = httpd_req_recv(r, b + got, len - got); if (n <= 0) { free(b); return ESP_FAIL; } got += n; }
    b[got] = 0;
    bool ok = cfg_from_json(b) && cfg_save();
    free(b);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", ok);
    cJSON_AddStringToObject(o, "note", "Saved. Wi-Fi/MQTT/UART changes need reboot; radio changes need 'Apply to E52'.");
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
            vTaskDelay(pdMS_TO_TICKS(1500));   /* module reboot */
            e52_runtime_setup();               /* else CSMA/HEAD revert until next gateway boot */
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

static esp_err_t h_nodes(httpd_req_t *r)
{
    static lmesh_node_t n[LMESH_MAX_NODES];   /* httpd is single-threaded */
    int cnt = lmesh_nodes(n, LMESH_MAX_NODES);
    uint32_t now = esp_timer_get_time() / 1000000LL;
    static e52_route_t rt[E52_MAX_ROUTES];
    int rn = e52_routes(rt, E52_MAX_ROUTES, 10000);
    cJSON *a = cJSON_CreateArray();
    for (int i = 0; i < cnt; i++) {
        cJSON *e = cJSON_CreateObject();
        char h[8], ip[20];
        for (int k = 0; k < rn; k++) if (rt[k].dst == n[i].addr) {
            snprintf(h, sizeof h, "%04X", rt[k].hop);
            cJSON_AddStringToObject(e, "hop", h);            /* next hop; == addr when direct */
            cJSON_AddNumberToObject(e, "rssi", rt[k].rssi);  /* of the next hop, dBm */
            break;
        }
        snprintf(h, sizeof h, "%04X", n[i].addr);
        lmesh_ip_str(n[i].addr, ip, sizeof ip);
        cJSON_AddStringToObject(e, "addr", h);
        cJSON_AddNumberToObject(e, "addr_dec", n[i].addr);
        cJSON_AddStringToObject(e, "ip", ip);
        cJSON_AddStringToObject(e, "type", n[i].addr & 0x8000 ? "terminal" : "router");
        cJSON_AddStringToObject(e, "name", n[i].name);
        cJSON_AddStringToObject(e, "site", n[i].site);
        cJSON_AddBoolToObject(e, "site_dup", n[i].site_dup);
        cJSON_AddStringToObject(e, "kind", n[i].kind);
        cJSON_AddStringToObject(e, "fw", n[i].fw);
        cJSON_AddNumberToObject(e, "age", now - n[i].last_seen_s);
        cJSON_AddNumberToObject(e, "pkts", n[i].rx_pkts);
        cJSON_AddNumberToObject(e, "rtt", n[i].rtt_ms);
        cJSON_AddItemToArray(a, e);
    }
    return send_json(r, a);
}

static esp_err_t h_ping(httpd_req_t *r)
{
    cJSON *in = read_json(r);
    int node = in ? jint(in, "node", -1) : -1;
    cJSON_Delete(in);
    cJSON *o = cJSON_CreateObject();
    if (node < 0 || node > 0xFFFE) { cJSON_AddStringToObject(o, "err", "bad node"); return send_json(r, o); }
    int32_t rtt = lmesh_ping(node, 8000);
    cJSON_AddNumberToObject(o, "rtt", rtt);
    return send_json(r, o);
}

/* set a field node's site id / name over the mesh (node answers on config/ack + re-ANNOUNCE) */
static esp_err_t h_node_cfg(httpd_req_t *r)
{
    cJSON *in = read_json(r);
    cJSON *o = cJSON_CreateObject();
    int node = in ? jint(in, "node", -1) : -1;
    if (node < 1 || node > 0x7FFF) { cJSON_Delete(in); cJSON_AddStringToObject(o, "err", "bad node"); return send_json(r, o); }
    cJSON *p = cJSON_CreateObject();
    cJSON *s = cJSON_GetObjectItem(in, "site"), *n = cJSON_GetObjectItem(in, "name");
    if (cJSON_IsString(s)) cJSON_AddStringToObject(p, "site", s->valuestring);
    if (cJSON_IsString(n)) cJSON_AddStringToObject(p, "name", n->valuestring);
    char *js = cJSON_PrintUnformatted(p);
    cJSON_Delete(p);
    cJSON_Delete(in);
    uint8_t buf[200];
    int l = js ? snprintf((char *)buf, sizeof buf, "config%c%s", 0, js) : -1;
    free(js);
    bool ok = l > 0 && l < (int)sizeof buf && gw_downlink(node, buf, l);
    cJSON_AddBoolToObject(o, "ok", ok);
    return send_json(r, o);
}

static esp_err_t h_discover(httpd_req_t *r)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", lmesh_discover() == ESP_OK);
    return send_json(r, o);
}

static esp_err_t h_modbus(httpd_req_t *r)
{
    cJSON *in = read_json(r);
    cJSON *o = cJSON_CreateObject();
    if (!in) { cJSON_AddStringToObject(o, "err", "bad json"); return send_json(r, o); }
    int unit = jint(in, "unit", 1), node = jint(in, "node", -1);
    int fc = jint(in, "fc", 3), addr = jint(in, "addr", 0), cnt = jint(in, "count", 1);
    cJSON *vals = cJSON_GetObjectItem(in, "values");

    uint8_t pdu[256]; int pl = 0;
    pdu[pl++] = fc;
    pdu[pl++] = addr >> 8; pdu[pl++] = addr & 0xFF;
    int nv = cJSON_IsArray(vals) ? cJSON_GetArraySize(vals) : 0;
    switch (fc) {
    case 1: case 2: case 3: case 4:
        if (cnt < 1 || cnt > (fc <= 2 ? 2000 : 125)) goto bad;
        pdu[pl++] = cnt >> 8; pdu[pl++] = cnt & 0xFF; break;
    case 5:
        if (nv < 1) goto bad;
        pdu[pl++] = cJSON_GetArrayItem(vals, 0)->valueint ? 0xFF : 0; pdu[pl++] = 0; break;
    case 6:
        if (nv < 1) goto bad;
        { int v = cJSON_GetArrayItem(vals, 0)->valueint; pdu[pl++] = (v >> 8) & 0xFF; pdu[pl++] = v & 0xFF; } break;
    case 15:
        if (nv < 1 || nv > 1968) goto bad;
        pdu[pl++] = nv >> 8; pdu[pl++] = nv & 0xFF; pdu[pl++] = (nv + 7) / 8;
        memset(pdu + pl, 0, (nv + 7) / 8);
        for (int i = 0; i < nv; i++) if (cJSON_GetArrayItem(vals, i)->valueint) pdu[pl + i / 8] |= 1 << (i % 8);
        pl += (nv + 7) / 8; break;
    case 16:
        if (nv < 1 || nv > 123) goto bad;
        pdu[pl++] = nv >> 8; pdu[pl++] = nv & 0xFF; pdu[pl++] = nv * 2;
        for (int i = 0; i < nv; i++) { int v = cJSON_GetArrayItem(vals, i)->valueint; pdu[pl++] = (v >> 8) & 0xFF; pdu[pl++] = v & 0xFF; }
        break;
    default: goto bad;
    }
    {
        uint16_t nd; uint8_t du;
        if (node >= 0) { nd = node; du = unit; } else modbus_gw_resolve(unit, &nd, &du);
        uint8_t rsp[256];
        int64_t t0 = esp_timer_get_time();
        int n = modbus_gw_txn(nd, du, pdu, pl, rsp, sizeof rsp, g_cfg.mb_timeout_ms);
        char ndh[8]; snprintf(ndh, sizeof ndh, "%04X", nd);
        cJSON_AddStringToObject(o, "node", ndh);
        cJSON_AddNumberToObject(o, "ms", (double)((esp_timer_get_time() - t0) / 1000));
        if (n <= 0) {
            cJSON_AddStringToObject(o, "err", n == MB_ERR_TIMEOUT ? "timeout (no reply over mesh)" :
                                              n == MB_ERR_BUSY ? "node busy" : "radio send failed");
        } else if (rsp[0] & 0x80) {
            char e[48]; snprintf(e, sizeof e, "Modbus exception %u", n > 1 ? rsp[1] : 0);
            cJSON_AddStringToObject(o, "err", e);
        } else {
            char hex[3 * 256 + 1]; int hl = 0;
            for (int i = 0; i < n; i++) hl += sprintf(hex + hl, "%02X ", rsp[i]);
            cJSON_AddStringToObject(o, "raw", hex);
            cJSON *a = cJSON_AddArrayToObject(o, "values");
            if ((fc == 3 || fc == 4) && n >= 2)
                for (int i = 0; i < rsp[1] / 2 && 2 + i * 2 + 1 < n; i++) cJSON_AddItemToArray(a, cJSON_CreateNumber((rsp[2 + i * 2] << 8) | rsp[3 + i * 2]));
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

static void reboot_task(void *a) { vTaskDelay(pdMS_TO_TICKS(500)); esp_restart(); }

static esp_err_t h_reboot(httpd_req_t *r)
{
    httpd_resp_sendstr(r, "{\"ok\":true}");
    xTaskCreate(reboot_task, "rb", 2048, NULL, 5, NULL);
    return ESP_OK;
}

/* ---- slow handlers (radio round trips, up to ~10 s) run on worker tasks so
 * the single httpd task keeps serving status/UI requests meanwhile ---- */
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

const char *meshrate_start(int rate, bool force);
int meshrate_status(const char **log);

/* POST {"rate":2[,"force":true]} starts a mesh-wide rate change; GET reports progress */
static esp_err_t h_meshrate(httpd_req_t *r)
{
    cJSON *o = cJSON_CreateObject();
    if (r->method == HTTP_POST) {
        cJSON *in = read_json(r);
        cJSON *f = in ? cJSON_GetObjectItem(in, "force") : NULL;
        const char *err = in ? meshrate_start(jint(in, "rate", -1), cJSON_IsTrue(f)) : "bad json";
        cJSON_Delete(in);
        if (err) cJSON_AddStringToObject(o, "err", err);
    }
    const char *log;
    int st = meshrate_status(&log);
    cJSON_AddNumberToObject(o, "state", st);
    cJSON_AddNumberToObject(o, "rate", g_cfg.e52_rate);
    cJSON_AddStringToObject(o, "log", log);
    return send_json(r, o);
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

/* Responses go out as header + body segments; without NODELAY the body waits
 * for the client's ACK (Nagle) and a slow/lost ACK costs a ~1.5 s RTO. */
static esp_err_t on_open(httpd_handle_t hd, int fd)
{
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    return ESP_OK;
}

void web_start(void)
{
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.stack_size = 10240;
    c.max_uri_handlers = 20;
    c.lru_purge_enable = true;
    c.open_fn = on_open;
    s_async_q = xQueueCreate(4, sizeof(httpd_req_t *));
    for (int i = 0; i < WEB_WORKERS; i++) xTaskCreate(async_worker, "web_async", 8192, NULL, 5, NULL);
    httpd_handle_t s;
    if (httpd_start(&s, &c) != ESP_OK) { ESP_LOGE(TAG, "httpd start failed"); return; }
    const httpd_uri_t u[] = {
        { .uri = "/", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_index },
        { .uri = "/api/status", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_status },
        { .uri = "/api/config", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_cfg_get },
        { .uri = "/api/config", .method = HTTP_POST, .handler = h_sync, .user_ctx = h_cfg_post },
        { .uri = "/api/e52/apply", .method = HTTP_POST, .handler = h_async, .user_ctx = h_e52_apply },
        { .uri = "/api/e52/at", .method = HTTP_POST, .handler = h_async, .user_ctx = h_e52_at },
        { .uri = "/api/nodes", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_nodes },
        { .uri = "/api/ping", .method = HTTP_POST, .handler = h_async, .user_ctx = h_ping },
        { .uri = "/api/node/config", .method = HTTP_POST, .handler = h_async, .user_ctx = h_node_cfg },
        { .uri = "/api/discover", .method = HTTP_POST, .handler = h_sync, .user_ctx = h_discover },
        { .uri = "/api/modbus", .method = HTTP_POST, .handler = h_async, .user_ctx = h_modbus },
        { .uri = "/api/mesh/rate", .method = HTTP_GET, .handler = h_sync, .user_ctx = h_meshrate },
        { .uri = "/api/mesh/rate", .method = HTTP_POST, .handler = h_sync, .user_ctx = h_meshrate },
        { .uri = "/api/reboot", .method = HTTP_POST, .handler = h_sync, .user_ctx = h_reboot },
    };
    for (size_t i = 0; i < sizeof u / sizeof u[0]; i++) httpd_register_uri_handler(s, &u[i]);
    ESP_LOGI(TAG, "web UI on port 80");
}
