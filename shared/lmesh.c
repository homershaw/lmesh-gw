#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include "lmesh.h"
#include "e52.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "lmesh";

#define MAX_BINDS 8
#define REASM_SLOTS 4
#define REASM_TIMEOUT_US (15 * 1000000LL)

static struct { uint16_t port; lmesh_handler_t h; } s_binds[MAX_BINDS];
static lmesh_handler_t s_default;
static QueueHandle_t s_rxq;
static SemaphoreHandle_t s_nodes_lock, s_ping_lock, s_ping_sem;
static lmesh_node_t s_nodes[LMESH_MAX_NODES];
static int s_nnodes;
static lmesh_announce_cb_t s_ann_cb;
static uint8_t s_seq;
static portMUX_TYPE s_seq_mux = portMUX_INITIALIZER_UNLOCKED;

static struct { volatile uint16_t node; volatile uint8_t seq; volatile bool active; int64_t t0; volatile int32_t rtt; } s_ping;

typedef struct {
    bool used;
    uint16_t src;
    uint8_t seq, total, mask;
    uint16_t last_len;
    int64_t t;
    uint8_t hdr[LMESH_HDR];
    uint8_t buf[LMESH_MAX_DGRAM];
} reasm_t;
static reasm_t s_ra[REASM_SLOTS];

static inline uint32_t now_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

uint32_t lmesh_addr_to_ip(uint16_t a) { return (10u << 24) | (52u << 16) | a; }

bool lmesh_ip_to_addr(uint32_t ip, uint16_t *a)
{
    if ((ip >> 16) != ((10u << 8) | 52u)) return false;
    *a = ip & 0xFFFF;
    return true;
}

void lmesh_ip_str(uint16_t a, char *b, size_t n) { snprintf(b, n, "10.52.%u.%u", a >> 8, a & 0xFF); }

uint8_t lmesh_next_seq(void)
{
    portENTER_CRITICAL(&s_seq_mux);
    uint8_t v = ++s_seq;
    portEXIT_CRITICAL(&s_seq_mux);
    return v;
}

esp_err_t lmesh_bind(uint16_t port, lmesh_handler_t h)
{
    for (int i = 0; i < MAX_BINDS; i++)
        if (!s_binds[i].h) { s_binds[i].port = port; s_binds[i].h = h; return ESP_OK; }
    return ESP_ERR_NO_MEM;
}

void lmesh_bind_default(lmesh_handler_t h) { s_default = h; }

/* ---------------- node table ---------------- */

static lmesh_node_t *node_touch(uint16_t a)
{
    lmesh_node_t *n = NULL;
    for (int i = 0; i < s_nnodes; i++) if (s_nodes[i].addr == a) { n = &s_nodes[i]; break; }
    if (!n) {
        if (s_nnodes < LMESH_MAX_NODES) n = &s_nodes[s_nnodes++];
        else {  /* evict stalest */
            n = &s_nodes[0];
            for (int i = 1; i < s_nnodes; i++) if (s_nodes[i].last_seen_s < n->last_seen_s) n = &s_nodes[i];
        }
        memset(n, 0, sizeof *n);
        n->addr = a;
        n->rtt_ms = -1;
        n->first_seen_s = now_s();
        ESP_LOGI(TAG, "new node 0x%04X", a);
    }
    n->last_seen_s = now_s();
    n->rx_pkts++;
    return n;
}

int lmesh_nodes(lmesh_node_t *out, int max)
{
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    int n = s_nnodes < max ? s_nnodes : max;
    memcpy(out, s_nodes, n * sizeof *out);
    xSemaphoreGive(s_nodes_lock);
    return n;
}

int lmesh_node_count(void) { return s_nnodes; }

void lmesh_node_topic(uint16_t addr, char *out, size_t n)
{
    snprintf(out, n, "%04X", addr);
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    for (int i = 0; i < s_nnodes; i++)
        if (s_nodes[i].addr == addr && s_nodes[i].site[0] && !s_nodes[i].site_dup)
            snprintf(out, n, "%s", s_nodes[i].site);
    xSemaphoreGive(s_nodes_lock);
}

bool lmesh_topic_to_addr(const char *lvl, size_t len, uint16_t *addr)
{
    bool ok = false;
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    for (int i = 0; i < s_nnodes && !ok; i++)
        if (s_nodes[i].site[0] && !s_nodes[i].site_dup && strlen(s_nodes[i].site) == len &&
            strncmp(s_nodes[i].site, lvl, len) == 0) { *addr = s_nodes[i].addr; ok = true; }
    xSemaphoreGive(s_nodes_lock);
    if (ok || len != 4) return ok;
    for (size_t i = 0; i < 4; i++) if (!isxdigit((unsigned char)lvl[i])) return false;
    char h[5] = { lvl[0], lvl[1], lvl[2], lvl[3], 0 };
    *addr = (uint16_t)strtol(h, NULL, 16);
    return true;
}

/* MQTT-safe topic level: no '/', '+', '#', spaces or control chars */
static void site_clean(char *s)
{
    for (; *s; s++) if (*s == '/' || *s == '+' || *s == '#' || *s <= ' ' || *s > '~') *s = '-';
}

/* caller holds s_nodes_lock. First live owner keeps a site id; a node silent
 * for over an hour gives it up (radio swapped on the same site). */
static void site_claim(lmesh_node_t *n, uint32_t now)
{
    n->site_dup = false;
    if (!n->site[0]) return;
    for (int i = 0; i < s_nnodes; i++) {
        lmesh_node_t *o = &s_nodes[i];
        if (o == n || o->site_dup || strcmp(o->site, n->site) != 0) continue;
        if (now - o->last_seen_s > 3600) { o->site[0] = 0; continue; }
        n->site_dup = true;
        ESP_LOGW(TAG, "site '%s' from 0x%04X already used by 0x%04X; using node number", n->site, n->addr, o->addr);
    }
}

static void copy_field(char *dst, size_t n, const uint8_t **p, const uint8_t *end)
{
    size_t i = 0;
    while (*p < end && **p) { if (i < n - 1) dst[i++] = (char)**p; (*p)++; }
    dst[i] = 0;
    if (*p < end) (*p)++;
}

/* ---------------- TX ---------------- */

esp_err_t lmesh_send(uint16_t node, uint8_t type, uint8_t seq,
                     uint16_t sport, uint16_t dport, const uint8_t *data, size_t len)
{
    if (len > LMESH_MAX_DGRAM) return ESP_ERR_INVALID_SIZE;
    uint8_t total = len ? (len + LMESH_SEG - 1) / LMESH_SEG : 1;
    uint8_t f[E52_MAX_PAYLOAD];
    for (uint8_t i = 0; i < total; i++) {
        size_t off = (size_t)i * LMESH_SEG;
        size_t n = len - off > LMESH_SEG ? LMESH_SEG : len - off;
        f[0] = LMESH_VER | (type & 0x0F);
        f[1] = seq;
        f[2] = (i << 4) | total;
        f[3] = sport >> 8; f[4] = sport & 0xFF;
        f[5] = dport >> 8; f[6] = dport & 0xFF;
        if (n) memcpy(f + LMESH_HDR, data + off, n);
        esp_err_t e = e52_send(node, f, LMESH_HDR + n);
        if (e != ESP_OK) return e;
    }
    return ESP_OK;
}

/* ---------------- RX ---------------- */

static void deliver(uint16_t src, const uint8_t *hdr, const uint8_t *data, uint16_t len)
{
    uint8_t type = hdr[0] & 0x0F;
    uint8_t seq = hdr[1];
    lmesh_dgram_t d = {
        .src = src, .seq = seq, .len = len, .data = data,
        .sport = (hdr[3] << 8) | hdr[4],
        .dport = (hdr[5] << 8) | hdr[6],
    };

    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    lmesh_node_t *n = node_touch(src);
    if (type == LMESH_T_ANNOUNCE) {
        const uint8_t *p = data, *end = data + len;
        copy_field(n->name, sizeof n->name, &p, end);
        copy_field(n->kind, sizeof n->kind, &p, end);
        copy_field(n->fw, sizeof n->fw, &p, end);
        copy_field(n->site, sizeof n->site, &p, end);
        site_clean(n->site);
        site_claim(n, n->last_seen_s);
        ESP_LOGI(TAG, "announce 0x%04X name=%s kind=%s fw=%s site=%s%s", src, n->name, n->kind, n->fw,
                 n->site, n->site_dup ? " (DUPLICATE)" : "");
    }
    xSemaphoreGive(s_nodes_lock);

    switch (type) {
    case LMESH_T_PING:
        lmesh_send(src, LMESH_T_PONG, seq, 0, 0, NULL, 0);
        return;
    case LMESH_T_PONG:
        if (s_ping.active && s_ping.seq == seq && (s_ping.node == src || s_ping.node == LMESH_BCAST)) {
            s_ping.rtt = (int32_t)((esp_timer_get_time() - s_ping.t0) / 1000);
            xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
            node_touch(src)->rtt_ms = s_ping.rtt;
            xSemaphoreGive(s_nodes_lock);
            if (s_ping.node != LMESH_BCAST) xSemaphoreGive(s_ping_sem);
        }
        return;
    case LMESH_T_ANNOUNCE:
        if (s_ann_cb) {
            char kind[16]; const uint8_t *p = data, *end = data + len;
            char skip[24];
            copy_field(skip, sizeof skip, &p, end);
            copy_field(kind, sizeof kind, &p, end);
            s_ann_cb(src, kind);
        }
        return;
    case LMESH_T_UDP:
        for (int i = 0; i < MAX_BINDS; i++)
            if (s_binds[i].h && s_binds[i].port == d.dport) { s_binds[i].h(&d); return; }
        if (s_default) s_default(&d);
        else ESP_LOGD(TAG, "no handler for port %u from 0x%04X", d.dport, src);
        return;
    default:
        ESP_LOGW(TAG, "unknown type %u from 0x%04X", type, src);
    }
}

static void handle_frame(const e52_frame_t *f)
{
    if (f->len < LMESH_HDR || (f->data[0] & 0xF0) != LMESH_VER) {
        ESP_LOGW(TAG, "non-LMESH frame from 0x%04X len %u", f->src, f->len);
        return;
    }
    uint8_t idx = f->data[2] >> 4, total = f->data[2] & 0x0F;
    const uint8_t *pl = f->data + LMESH_HDR;
    uint16_t plen = f->len - LMESH_HDR;

    if (total <= 1) { deliver(f->src, f->data, pl, plen); return; }
    if (total > LMESH_MAX_FRAGS || idx >= total) return;
    if (idx < total - 1 && plen != LMESH_SEG) return;   /* non-last fragments are full */

    int64_t now = esp_timer_get_time();
    reasm_t *r = NULL, *freeslot = NULL, *oldest = &s_ra[0];
    for (int i = 0; i < REASM_SLOTS; i++) {
        reasm_t *s = &s_ra[i];
        if (s->used && now - s->t > REASM_TIMEOUT_US) s->used = false;
        if (s->used && s->src == f->src && s->seq == f->data[1]) r = s;
        if (!s->used && !freeslot) freeslot = s;
        if (s->t < oldest->t) oldest = s;
    }
    if (!r) {
        r = freeslot ? freeslot : oldest;
        memset(r, 0, offsetof(reasm_t, buf));
        r->used = true; r->src = f->src; r->seq = f->data[1]; r->total = total;
        memcpy(r->hdr, f->data, LMESH_HDR);
    }
    r->t = now;
    memcpy(r->buf + idx * LMESH_SEG, pl, plen);
    r->mask |= 1 << idx;
    if (idx == total - 1) r->last_len = plen;
    if (r->mask == (1 << total) - 1) {
        r->used = false;
        deliver(r->src, r->hdr, r->buf, (total - 1) * LMESH_SEG + r->last_len);
    }
}

static void rx_cb(const e52_frame_t *f)
{
    if (xQueueSend(s_rxq, f, 0) != pdTRUE) ESP_LOGW(TAG, "rx queue full, dropped");
}

static void lmesh_task(void *arg)
{
    static e52_frame_t f;
    for (;;) {
        if (xQueueReceive(s_rxq, &f, portMAX_DELAY) == pdTRUE) handle_frame(&f);
    }
}

/* ---------------- ping ---------------- */

int32_t lmesh_ping(uint16_t node, int timeout_ms)
{
    xSemaphoreTake(s_ping_lock, portMAX_DELAY);
    xSemaphoreTake(s_ping_sem, 0);
    s_ping.node = node;
    s_ping.seq = lmesh_next_seq();
    s_ping.rtt = -1;
    s_ping.t0 = esp_timer_get_time();
    s_ping.active = true;
    int32_t rtt = -1;
    if (lmesh_send(node, LMESH_T_PING, s_ping.seq, 0, 0, NULL, 0) == ESP_OK &&
        xSemaphoreTake(s_ping_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
        rtt = s_ping.rtt;
    s_ping.active = false;
    xSemaphoreGive(s_ping_lock);
    return rtt;
}

static void discover_task(void *arg)
{
    xSemaphoreTake(s_ping_lock, portMAX_DELAY);
    s_ping.node = LMESH_BCAST;
    s_ping.seq = lmesh_next_seq();
    s_ping.t0 = esp_timer_get_time();
    s_ping.active = true;
    lmesh_send(LMESH_BCAST, LMESH_T_PING, s_ping.seq, 0, 0, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(15000));      /* collect PONGs from whole mesh */
    s_ping.active = false;
    xSemaphoreGive(s_ping_lock);
    vTaskDelete(NULL);
}

esp_err_t lmesh_discover(void)
{
    return xTaskCreate(discover_task, "discover", 3072, NULL, 5, NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}

/* ---------------- identity / announce ---------------- */

static struct { char name[24], kind[16], fw[16], site[40]; uint16_t boot_dst; } s_id = {
    .name = "lmesh", .kind = "gateway", .fw = "1.0.0", .boot_dst = LMESH_BCAST };

void lmesh_set_identity(const char *name, const char *kind, const char *fw, const char *site, uint16_t boot_dst)
{
    snprintf(s_id.name, sizeof s_id.name, "%s", name ? name : "");
    snprintf(s_id.kind, sizeof s_id.kind, "%s", kind ? kind : "");
    snprintf(s_id.fw, sizeof s_id.fw, "%s", fw ? fw : "");
    snprintf(s_id.site, sizeof s_id.site, "%s", site ? site : "");
    s_id.boot_dst = boot_dst;
}

void lmesh_on_announce(lmesh_announce_cb_t cb) { s_ann_cb = cb; }

esp_err_t lmesh_announce(uint16_t dst)
{
    char buf[112];
    int n = snprintf(buf, sizeof buf, "%s", s_id.name) + 1;
    n += snprintf(buf + n, sizeof buf - n, "%s", s_id.kind) + 1;
    n += snprintf(buf + n, sizeof buf - n, "%s", s_id.fw) + 1;
    if (s_id.site[0]) n += snprintf(buf + n, sizeof buf - n, "%s", s_id.site) + 1;
    return lmesh_send(dst, LMESH_T_ANNOUNCE, lmesh_next_seq(), 0, 0, (uint8_t *)buf, n);
}

void lmesh_init(void)
{
    s_rxq = xQueueCreate(16, sizeof(e52_frame_t));
    s_nodes_lock = xSemaphoreCreateMutex();
    s_ping_lock = xSemaphoreCreateMutex();
    s_ping_sem = xSemaphoreCreateBinary();
    xTaskCreate(lmesh_task, "lmesh", 6144, NULL, 10, NULL);
}

esp_err_t lmesh_start(void)
{
    esp_err_t e = e52_init(rx_cb);
    if (e == ESP_OK) lmesh_announce(s_id.boot_dst);
    return e;
}
