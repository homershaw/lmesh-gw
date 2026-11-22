/*
 * MQTT bridge. Gateway is an MQTT client of the broker (e.g. riotdash).
 *
 *  Uplink  : node -> LMESH port 1883 "sub/topic\0payload"
 *            => publish  <base>/<NODE>/sub/topic   (NODE = 4 hex digits)
 *  Downlink: subscribe   <base>/+/set/#
 *            <base>/<NODE>/set/sub/topic => LMESH port 1884 "sub/topic\0payload"
 *  Status  : <base>/gw/status  online|offline (retained, LWT)
 *            <base>/gw/stats   JSON every 60 s
 *            <base>/gw/nodes   JSON node table every 60 s (retained)
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "gw.h"
#include "config.h"
#include "lmesh.h"
#include "e52.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "mqttgw";
static esp_mqtt_client_handle_t s_cli;
static volatile bool s_conn, s_started;
static char s_base[72];
static QueueHandle_t s_dnq;
static uint32_t s_up, s_down;

typedef struct { uint16_t node; uint16_t len; uint8_t data[LMESH_MAX_DGRAM]; } dn_msg_t;

bool mqtt_gw_connected(void) { return s_conn; }
const char *mqtt_gw_base(void) { return s_base; }
void mqtt_gw_stats(uint32_t *u, uint32_t *d) { *u = s_up; *d = s_down; }

/* node -> broker */
int survey_on_pos(uint16_t src, const char *json, int len, char *out, size_t outmax);

static void on_mesh_1883(const lmesh_dgram_t *d)
{
    /* survey points are kept on the gateway whether or not a broker is connected */
    static char sv[440];
    int svl = 0;
    if (d->len > 4 && memcmp(d->data, "pos", 4) == 0)
        svl = survey_on_pos(d->src, (const char *)d->data + 4, d->len - 4, sv, sizeof sv);
    if (!s_conn) return;
    const uint8_t *z = memchr(d->data, 0, d->len);
    if (!z || z == d->data) return;
    size_t tl = z - d->data;
    char topic[160], lvl[40];
    lmesh_node_topic(d->src, lvl, sizeof lvl);
    int n = snprintf(topic, sizeof topic, "%s/%s/", s_base, lvl);
    if (n + tl >= sizeof topic) return;
    memcpy(topic + n, d->data, tl);
    topic[n + tl] = 0;
    const char *pl = (const char *)z + 1;
    int pll = d->len - tl - 1;
    esp_mqtt_client_enqueue(s_cli, topic, pl, pll, 0, 0, true);
    s_up++;
    if (svl > 0 && snprintf(topic, sizeof topic, "%s/%s/survey", s_base, lvl) < (int)sizeof topic)
        esp_mqtt_client_enqueue(s_cli, topic, sv, svl, 0, 0, true);
}

/* Downlinks are acknowledged by the node (empty UDP back to port 1884, same
 * seq) and resent with the same seq until acked, so a lost radio frame does
 * not silently drop a config change or a write. Nodes ignore repeats. */
static SemaphoreHandle_t s_dn_lock, s_dn_ack;
static volatile uint16_t s_dn_node;
static volatile int s_dn_seq = -1;
static uint32_t s_down_fail;

static void on_dn_ack(const lmesh_dgram_t *d)
{
    if (d->src == s_dn_node && d->seq == s_dn_seq) xSemaphoreGive(s_dn_ack);
}

bool gw_downlink(uint16_t node, const uint8_t *data, size_t len)
{
    static const uint32_t base[] = { 900, 1200, 3500 };
    uint32_t wait = base[g_cfg.e52_rate > 2 ? 2 : g_cfg.e52_rate] + 2 * e52_airtime_ms(len > 200 ? 200 : len) * ((len + 192) / 193);
    xSemaphoreTake(s_dn_lock, portMAX_DELAY);
    uint8_t seq = lmesh_next_seq();
    s_dn_node = node;
    s_dn_seq = seq;
    xSemaphoreTake(s_dn_ack, 0);
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
        if (attempt) ESP_LOGW(TAG, "downlink to 0x%04X seq %u: no ack, retry %d", node, seq, attempt);
        if (lmesh_send_udp(node, seq, LMESH_PORT_MQTT_DN, LMESH_PORT_MQTT_DN, data, len) != ESP_OK) continue;
        ok = xSemaphoreTake(s_dn_ack, pdMS_TO_TICKS(wait)) == pdTRUE;
    }
    s_dn_seq = -1;
    xSemaphoreGive(s_dn_lock);
    if (ok) s_down++; else { s_down_fail++; ESP_LOGW(TAG, "downlink to 0x%04X not acknowledged", node); }
    return ok;
}

static void dn_task(void *a)
{
    static dn_msg_t m;
    for (;;)
        if (xQueueReceive(s_dnq, &m, portMAX_DELAY) == pdTRUE) gw_downlink(m.node, m.data, m.len);
}

/* broker -> node */
static void handle_downlink(const char *topic, int tl, const char *data, int dl)
{
    size_t bl = strlen(s_base);
    /* <base>/<SITE or NODE>/set/<sub> */
    if (tl < (int)bl + 7 || strncmp(topic, s_base, bl) != 0 || topic[bl] != '/') return;
    const char *p = topic + bl + 1, *end = topic + tl;
    const char *sl_ = memchr(p, '/', end - p);
    if (!sl_ || end - sl_ < 6 || strncmp(sl_, "/set/", 5) != 0) return;
    uint16_t node;
    if (!lmesh_topic_to_addr(p, sl_ - p, &node)) { ESP_LOGW(TAG, "downlink: unknown site '%.*s'", (int)(sl_ - p), p); return; }
    const char *sub = sl_ + 5;
    int sl = tl - (sub - topic);
    if (sl <= 0) return;
    static dn_msg_t m;           /* only used from MQTT task */
    if (sl + 1 + dl > LMESH_MAX_DGRAM) { ESP_LOGW(TAG, "downlink too large"); return; }
    m.node = node;
    memcpy(m.data, sub, sl);
    m.data[sl] = 0;
    memcpy(m.data + sl + 1, data, dl);
    m.len = sl + 1 + dl;
    if (xQueueSend(s_dnq, &m, 0) != pdTRUE) ESP_LOGW(TAG, "downlink queue full");
}

static void on_mqtt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t ev = data;
    char t[96];
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_conn = true;
        ESP_LOGI(TAG, "connected, base '%s'", s_base);
        snprintf(t, sizeof t, "%s/gw/status", s_base);
        esp_mqtt_client_publish(s_cli, t, "online", 0, 1, 1);
        snprintf(t, sizeof t, "%s/+/set/#", s_base);
        esp_mqtt_client_subscribe(s_cli, t, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_conn = false;
        break;
    case MQTT_EVENT_DATA:
        if (ev->current_data_offset == 0 && ev->data_len == ev->total_data_len)
            handle_downlink(ev->topic, ev->topic_len, ev->data, ev->data_len);
        break;
    default:
        break;
    }
}

static void stats_task(void *a)
{
    char t[96];
    static char buf[2048];
    static lmesh_node_t nodes[LMESH_MAX_NODES];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        if (!s_conn) continue;
        e52_stats_t st; e52_get_stats(&st);
        snprintf(buf, sizeof buf,
                 "{\"uptime\":%lld,\"tx\":%lu,\"rx\":%lu,\"at_err\":%lu,\"tx_fail\":%lu,\"up\":%lu,\"down\":%lu}",
                 esp_timer_get_time() / 1000000LL, (unsigned long)st.tx_frames, (unsigned long)st.rx_frames,
                 (unsigned long)st.at_err, (unsigned long)st.tx_fail, (unsigned long)s_up, (unsigned long)s_down);
        snprintf(t, sizeof t, "%s/gw/stats", s_base);
        esp_mqtt_client_enqueue(s_cli, t, buf, 0, 0, 0, true);

        int n = lmesh_nodes(nodes, LMESH_MAX_NODES);
        uint32_t now = esp_timer_get_time() / 1000000LL;
        size_t o = snprintf(buf, sizeof buf, "[");
        for (int i = 0; i < n && o < sizeof buf - 200; i++)
            o += snprintf(buf + o, sizeof buf - o, "%s{\"addr\":\"%04X\",\"site\":\"%s\",\"name\":\"%s\",\"kind\":\"%s\",\"age\":%lu,\"rtt\":%ld}",
                          i ? "," : "", nodes[i].addr, nodes[i].site, nodes[i].name, nodes[i].kind,
                          (unsigned long)(now - nodes[i].last_seen_s), (long)nodes[i].rtt_ms);
        snprintf(buf + o, sizeof buf - o, "]");
        snprintf(t, sizeof t, "%s/gw/nodes", s_base);
        esp_mqtt_client_enqueue(s_cli, t, buf, 0, 0, 1, true);
    }
}

void mqtt_gw_on_network(bool up)
{
    if (up && !s_started && s_cli) { esp_mqtt_client_start(s_cli); s_started = true; }
}

void mqtt_gw_start(void)
{
    if (g_cfg.mqtt_base[0]) snprintf(s_base, sizeof s_base, "%s", g_cfg.mqtt_base);
    else snprintf(s_base, sizeof s_base, "lmesh/%s", g_gw_id);

    lmesh_bind(LMESH_PORT_MQTT_UP, on_mesh_1883);
    s_dn_lock = xSemaphoreCreateMutex();
    s_dn_ack = xSemaphoreCreateBinary();
    lmesh_bind(LMESH_PORT_MQTT_DN, on_dn_ack);
    if (!g_cfg.mqtt_en || !g_cfg.mqtt_uri[0]) { ESP_LOGI(TAG, "MQTT disabled"); return; }

    s_dnq = xQueueCreate(4, sizeof(dn_msg_t));
    xTaskCreate(dn_task, "mqtt_dn", 3072, NULL, 5, NULL);

    static char lwt[96];
    snprintf(lwt, sizeof lwt, "%s/gw/status", s_base);
    esp_mqtt_client_config_t mc = {
        .broker.address.uri = g_cfg.mqtt_uri,
        .credentials.username = g_cfg.mqtt_user[0] ? g_cfg.mqtt_user : NULL,
        .credentials.authentication.password = g_cfg.mqtt_pass[0] ? g_cfg.mqtt_pass : NULL,
        .session.last_will = { .topic = lwt, .msg = "offline", .qos = 1, .retain = 1 },
        .session.keepalive = 60,
        .network.reconnect_timeout_ms = 10000,
    };
    s_cli = esp_mqtt_client_init(&mc);
    esp_mqtt_client_register_event(s_cli, ESP_EVENT_ANY_ID, on_mqtt, NULL);
    xTaskCreate(stats_task, "mqtt_st", 4096, NULL, 3, NULL);
    /* AP-only gateway: the broker can be on the AP side (laptop/Pi on site) */
    if (wifi_sta_connected() || !g_cfg.sta_ssid[0]) mqtt_gw_on_network(true);
}
