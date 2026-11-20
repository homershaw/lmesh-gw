/*
 * Generic UDP/IPv4 bridge so any LAN host can exchange datagrams with any
 * mesh node's IPv4 address (10.52.x.y) without routes on the LAN.
 *
 * LAN -> gateway UDP :5252   [node IPv4 (4)][node port (2, BE)][payload]
 * gateway -> LAN             [node IPv4 (4)][node port (2, BE)][payload]
 *
 * The gateway allocates a mesh source port 0xC000+slot per (LAN peer, node,
 * port) tuple, so node replies (sent to that port) find their way back.
 */
#include <string.h>
#include <unistd.h>
#include "gw.h"
#include "lmesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "udpbr";
#define NSLOT 16
#define PORT_BASE 0xC000

typedef struct { bool used; struct sockaddr_in peer; uint16_t node, nport; int64_t t; } map_t;
static map_t s_map[NSLOT];
static SemaphoreHandle_t s_lock;
static int s_sock = -1;

static int slot_for(const struct sockaddr_in *peer, uint16_t node, uint16_t nport)
{
    int freei = -1, oldest = 0;
    for (int i = 0; i < NSLOT; i++) {
        map_t *m = &s_map[i];
        if (m->used && m->node == node && m->nport == nport &&
            m->peer.sin_addr.s_addr == peer->sin_addr.s_addr && m->peer.sin_port == peer->sin_port) return i;
        if (!m->used && freei < 0) freei = i;
        if (m->t < s_map[oldest].t) oldest = i;
    }
    int i = freei >= 0 ? freei : oldest;
    s_map[i] = (map_t){ .used = true, .peer = *peer, .node = node, .nport = nport };
    return i;
}

static void on_mesh_any(const lmesh_dgram_t *d)
{
    if (s_sock < 0 || d->dport < PORT_BASE || d->dport >= PORT_BASE + NSLOT) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    map_t m = s_map[d->dport - PORT_BASE];
    if (m.used) s_map[d->dport - PORT_BASE].t = esp_timer_get_time();
    xSemaphoreGive(s_lock);
    if (!m.used) return;

    uint8_t buf[6 + LMESH_MAX_DGRAM];
    uint32_t ip = htonl(lmesh_addr_to_ip(d->src));
    memcpy(buf, &ip, 4);
    buf[4] = d->sport >> 8; buf[5] = d->sport & 0xFF;
    memcpy(buf + 6, d->data, d->len);
    sendto(s_sock, buf, 6 + d->len, 0, (struct sockaddr *)&m.peer, sizeof m.peer);
}

static void bridge_task(void *a)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in ba = { .sin_family = AF_INET, .sin_port = htons(UDP_BRIDGE_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(s_sock, (struct sockaddr *)&ba, sizeof ba) < 0) { ESP_LOGE(TAG, "bind failed"); vTaskDelete(NULL); }
    ESP_LOGI(TAG, "UDP bridge on %u", UDP_BRIDGE_PORT);
    static uint8_t buf[6 + LMESH_MAX_DGRAM];
    for (;;) {
        struct sockaddr_in from; socklen_t fl = sizeof from;
        int n = recvfrom(s_sock, buf, sizeof buf, 0, (struct sockaddr *)&from, &fl);
        if (n < 7) continue;
        uint32_t ip; memcpy(&ip, buf, 4);
        uint16_t node;
        if (!lmesh_ip_to_addr(ntohl(ip), &node)) continue;
        uint16_t nport = (buf[4] << 8) | buf[5];
        xSemaphoreTake(s_lock, portMAX_DELAY);
        int sl = slot_for(&from, node, nport);
        s_map[sl].t = esp_timer_get_time();
        xSemaphoreGive(s_lock);
        if (lmesh_send_udp(node, lmesh_next_seq(), PORT_BASE + sl, nport, buf + 6, n - 6) != ESP_OK)
            ESP_LOGW(TAG, "send to 0x%04X:%u failed", node, nport);
    }
}

void udp_bridge_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    lmesh_bind_default(on_mesh_any);
    xTaskCreate(bridge_task, "udpbr", 4096, NULL, 5, NULL);
}
