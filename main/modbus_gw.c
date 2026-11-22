/*
 * Modbus TCP server (default port 502) terminating TCP at the gateway.
 * Each request is converted to an LMESH UDP datagram to port 502 on the
 * target node:  [unit][PDU]. The node's agent runs it on its RS-485/UART
 * (Modbus RTU) and returns [unit][response PDU] with the same LMESH seq.
 *
 * Unit routing: g_cfg.mb_map entries first; otherwise unit N -> mesh node N.
 * Unit 255 is answered locally with gateway diagnostics (FC 3/4).
 */
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include "modbus_gw.h"
#include "lmesh.h"
#include "e52.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "mbgw";

#define MAX_CLIENTS 4
#define SLOTS 8
#define MB_MAX_PDU 253

typedef struct {
    volatile bool active;
    uint16_t node;
    uint8_t seq;
    SemaphoreHandle_t sem;
    uint16_t len;
    uint8_t buf[MB_MAX_PDU + 1];
} slot_t;

static slot_t s_slots[SLOTS];
static SemaphoreHandle_t s_lock;
static volatile uint32_t s_reqs, s_timeouts, s_clients, s_retries;

/* retransmit interval: ~3x measured 1-hop RTT per air rate, + time for fragments */
static int mb_retry_ms(size_t req_len)
{
    static const int base[] = { 900, 1200, 1800 };     /* rate 0 / 1 / 2; at 7k the airtime terms below add ~640 ms */
    int b = base[g_cfg.e52_rate > 2 ? 2 : g_cfg.e52_rate];
    return b + (int)e52_airtime_ms(req_len) + 2 * (int)e52_airtime_ms(E52_MAX_PAYLOAD);
}

void modbus_gw_stats(uint32_t *r, uint32_t *t, uint32_t *c, uint32_t *rt) { *r = s_reqs; *t = s_timeouts; *c = s_clients; *rt = s_retries; }

void modbus_gw_resolve(uint8_t unit, uint16_t *node, uint8_t *dst_unit)
{
    for (int i = 0; i < g_cfg.mb_map_n; i++) {
        if (g_cfg.mb_map[i].unit == unit) {
            *node = g_cfg.mb_map[i].node;
            *dst_unit = g_cfg.mb_map[i].dst_unit ? g_cfg.mb_map[i].dst_unit : unit;
            return;
        }
    }
    *node = unit;
    *dst_unit = unit;
}

/* mesh -> gateway response */
static void on_mesh_502(const lmesh_dgram_t *d)
{
    if (d->len < 2 || d->len > MB_MAX_PDU + 1) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < SLOTS; i++) {
        slot_t *s = &s_slots[i];
        if (s->active && s->node == d->src && s->seq == d->seq) {
            memcpy(s->buf, d->data, d->len);
            s->len = d->len;
            s->active = false;
            xSemaphoreGive(s->sem);
            break;
        }
    }
    xSemaphoreGive(s_lock);
}

int modbus_gw_txn(uint16_t node, uint8_t dst_unit, const uint8_t *pdu, size_t plen,
                  uint8_t *rsp, size_t rmax, int timeout_ms)
{
    if (plen < 1 || plen > MB_MAX_PDU) return MB_ERR_PARAM;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    slot_t *s = NULL;

    /* one outstanding transaction per node; wait for the node to be free */
    while (!s) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool node_busy = false;
        slot_t *fs = NULL;
        for (int i = 0; i < SLOTS; i++) {
            if (s_slots[i].active && s_slots[i].node == node) node_busy = true;
            if (!s_slots[i].active && !fs) fs = &s_slots[i];
        }
        if (!node_busy && fs) {
            s = fs;
            s->active = true;
            s->node = node;
            s->seq = lmesh_next_seq();
            s->len = 0;
            xSemaphoreTake(s->sem, 0);
        }
        xSemaphoreGive(s_lock);
        if (!s) {
            if (esp_timer_get_time() > deadline) return MB_ERR_BUSY;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    uint8_t req[MB_MAX_PDU + 1];
    req[0] = dst_unit;
    memcpy(req + 1, pdu, plen);
    s_reqs++;

    /* The mesh drops a few % of frames and LMESH has no ACK, so retransmit the
     * same seq if no reply comes within ~3x a typical RTT. Duplicate replies
     * are ignored (slot already inactive). Reads/writes are idempotent. */
    int retry_ms = mb_retry_ms(plen + 1);
    bool got = false;
    for (int attempt = 0; !got; attempt++) {
        int64_t left = (deadline - esp_timer_get_time()) / 1000;
        if (left < 50) break;
        if (attempt) { s_retries++; ESP_LOGW(TAG, "node 0x%04X seq %u: no reply, retry %d", node, s->seq, attempt); }
        if (lmesh_send_udp(node, s->seq, LMESH_PORT_MODBUS, LMESH_PORT_MODBUS, req, plen + 1) != ESP_OK) {
            if (attempt == 0) { s->active = false; return MB_ERR_SEND; }
            continue;
        }
        int wait = left < retry_ms ? (int)left : retry_ms;
        got = xSemaphoreTake(s->sem, pdMS_TO_TICKS(wait)) == pdTRUE;
    }
    if (!got) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool late = !s->active;            /* reply raced the deadline */
        s->active = false;
        xSemaphoreGive(s_lock);
        if (!late) { s_timeouts++; return MB_ERR_TIMEOUT; }
        xSemaphoreTake(s->sem, 0);
    }
    int n = s->len - 1;                     /* strip unit byte */
    if ((size_t)n > rmax) n = rmax;
    memcpy(rsp, s->buf + 1, n);
    return n;
}

/* ---------------- local diagnostics (unit 255) ---------------- */

static int local_regs(const uint8_t *pdu, size_t plen, uint8_t *rsp)
{
    uint8_t fc = pdu[0];
    if ((fc != 3 && fc != 4) || plen < 5) { rsp[0] = fc | 0x80; rsp[1] = 1; return 2; }
    uint16_t start = (pdu[1] << 8) | pdu[2], cnt = (pdu[3] << 8) | pdu[4];
    if (cnt == 0 || cnt > 125) { rsp[0] = fc | 0x80; rsp[1] = 3; return 2; }

    e52_stats_t st; e52_get_stats(&st);
    uint16_t regs[16] = {
        (uint16_t)lmesh_node_count(),
        st.tx_frames >> 16, st.tx_frames & 0xFFFF,
        st.rx_frames >> 16, st.rx_frames & 0xFFFF,
        (uint16_t)st.at_err, (uint16_t)st.tx_fail, (uint16_t)st.rx_junk,
        s_reqs >> 16, s_reqs & 0xFFFF, (uint16_t)s_timeouts,
        (uint16_t)(esp_timer_get_time() / 60000000LL),   /* uptime minutes */
        g_cfg.e52_channel, g_cfg.e52_rate, (uint16_t)g_cfg.e52_power, g_cfg.e52_addr,
    };
    if (start + cnt > 16) { rsp[0] = fc | 0x80; rsp[1] = 2; return 2; }
    rsp[0] = fc; rsp[1] = cnt * 2;
    for (int i = 0; i < cnt; i++) { rsp[2 + i * 2] = regs[start + i] >> 8; rsp[3 + i * 2] = regs[start + i] & 0xFF; }
    return 2 + cnt * 2;
}

/* ---------------- TCP server ---------------- */

static int recv_all(int fd, uint8_t *b, int n)
{
    int got = 0;
    while (got < n) {
        int r = recv(fd, b + got, n - got, 0);
        if (r <= 0) return -1;
        got += r;
    }
    return got;
}

static void client_task(void *arg)
{
    int fd = (int)(intptr_t)arg;
    uint8_t req[7 + MB_MAX_PDU], rsp[7 + MB_MAX_PDU];
    s_clients++;
    for (;;) {
        if (recv_all(fd, req, 7) < 0) break;
        uint16_t proto = (req[2] << 8) | req[3];
        uint16_t len = (req[4] << 8) | req[5];
        if (proto != 0 || len < 2 || len > MB_MAX_PDU + 1) break;
        if (recv_all(fd, req + 7, len - 1) < 0) break;

        uint8_t unit = req[6];
        uint8_t *pdu = req + 7;
        size_t plen = len - 1;
        int n;
        if (unit == MB_GW_UNIT_LOCAL) {
            n = local_regs(pdu, plen, rsp + 7);
        } else {
            uint16_t node; uint8_t du;
            modbus_gw_resolve(unit, &node, &du);
            n = modbus_gw_txn(node, du, pdu, plen, rsp + 7, MB_MAX_PDU, g_cfg.mb_timeout_ms);
            if (n <= 0) {
                ESP_LOGW(TAG, "unit %u -> node 0x%04X fc %u failed (%d)", unit, node, pdu[0], n);
                rsp[7] = pdu[0] | 0x80;
                rsp[8] = (n == MB_ERR_TIMEOUT || n == MB_ERR_BUSY) ? 0x0B : 0x0A;   /* gw target no response / path unavailable */
                n = 2;
            }
        }
        memcpy(rsp, req, 4);                    /* transaction id + protocol id */
        rsp[4] = (n + 1) >> 8; rsp[5] = (n + 1) & 0xFF;
        rsp[6] = unit;
        if (send(fd, rsp, 7 + n, 0) < 0) break;
    }
    close(fd);
    s_clients--;
    vTaskDelete(NULL);
}

static void server_task(void *arg)
{
    int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(g_cfg.mb_port), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(ls, (struct sockaddr *)&a, sizeof a) < 0 || listen(ls, 4) < 0) {
        ESP_LOGE(TAG, "bind/listen port %u failed: %d", g_cfg.mb_port, errno);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Modbus TCP listening on %u", g_cfg.mb_port);
    for (;;) {
        struct sockaddr_in ca; socklen_t cl = sizeof ca;
        int fd = accept(ls, (struct sockaddr *)&ca, &cl);
        if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        if (s_clients >= MAX_CLIENTS) { close(fd); continue; }
        int ka = 1; setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof ka);
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &ka, sizeof ka);
        struct timeval tv = { .tv_sec = 300 };   /* idle client timeout */
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        if (xTaskCreate(client_task, "mbcli", 4096, (void *)(intptr_t)fd, 6, NULL) != pdPASS) close(fd);
    }
}

void modbus_gw_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    for (int i = 0; i < SLOTS; i++) s_slots[i].sem = xSemaphoreCreateBinary();
    lmesh_bind(LMESH_PORT_MODBUS, on_mesh_502);
    xTaskCreate(server_task, "mbsrv", 4096, NULL, 6, NULL);
}
