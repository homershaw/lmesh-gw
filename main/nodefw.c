/*
 * Gateway side of the node firmware update over the mesh (shared/fwproto.h).
 *
 *  1. The node image is uploaded to the gateway once and kept in the
 *     gateway's idle firmware slot (it is only needed for the gateway's own
 *     next update, which simply overwrites it).
 *  2. A transfer task feeds it to one node at a time, one radio frame per
 *     packet, waiting for the node's ACK each time. The ACK names the next
 *     byte the node expects, so a lost packet is just sent again and an
 *     interrupted transfer resumes where it stopped.
 *  3. The node verifies the image, restarts into it on trial and keeps it
 *     only once it can reach the gateway again; otherwise it rolls back.
 */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "gw.h"
#include "config.h"
#include "lmesh.h"
#include "e52.h"
#include "ota.h"
#include "fwproto.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_rom_crc.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define NODE_PROJECT  "lmesh_node"
#define MIN_NODE_FW   "0.6.1"      /* first node firmware with the mesh update */

static const char *TAG = "nodefw";

/* ---- the stored image ---- */
static struct { uint32_t size, crc; char ver[40]; } s_img;

static const esp_partition_t *store(void) { return esp_ota_get_next_update_partition(NULL); }

static void img_save(void)
{
    nvs_handle_t h;
    if (nvs_open("lmgw", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "nodefw", &s_img, sizeof s_img);
    nvs_commit(h);
    nvs_close(h);
}

void nodefw_init(void)
{
    nvs_handle_t h;
    size_t l = sizeof s_img;
    if (nvs_open("lmgw", NVS_READONLY, &h) != ESP_OK) return;
    if (nvs_get_blob(h, "nodefw", &s_img, &l) != ESP_OK || l != sizeof s_img) memset(&s_img, 0, sizeof s_img);
    nvs_close(h);
    if (s_img.size) ESP_LOGI(TAG, "holding node firmware %s, %lu bytes", s_img.ver, (unsigned long)s_img.size);
}

/* the gateway's own update is about to overwrite the slot */
void nodefw_invalidate(void) { if (s_img.size) { memset(&s_img, 0, sizeof s_img); img_save(); } }

/* ---- transfer state ---- */
static volatile int s_state;          /* 0 idle, 1 running, 2 done ok, 3 failed, 4 cancelled */
static volatile bool s_cancel;
static uint16_t s_node;
static volatile uint32_t s_sent;      /* bytes the node has confirmed */
static uint32_t s_t0;
static char s_log[900];

static SemaphoreHandle_t s_ack;
static volatile uint32_t s_ack_exp;
static volatile uint8_t s_ack_st;
static char s_ack_txt[84];

static uint32_t up_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

static void say(const char *fmt, ...)
{
    size_t l = strlen(s_log);
    if (l > sizeof s_log - 120) return;
    l += snprintf(s_log + l, sizeof s_log - l, "[%lus] ", (unsigned long)(up_s() - s_t0));
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_log + l, sizeof s_log - l, fmt, ap);
    va_end(ap);
    ESP_LOGI(TAG, "%s", s_log + l);
    strlcat(s_log, "\n", sizeof s_log);
}

static void on_mesh_fw(const lmesh_dgram_t *d)
{
    if (s_state != 1 || d->src != s_node || d->len < 6 || d->data[0] != FW_ACK) return;
    s_ack_exp = d->data[1] | (d->data[2] << 8) | (d->data[3] << 16) | ((uint32_t)d->data[4] << 24);
    s_ack_st = d->data[5];
    size_t n = d->len - 6;
    if (n > sizeof s_ack_txt - 1) n = sizeof s_ack_txt - 1;
    memcpy(s_ack_txt, d->data + 6, n);
    s_ack_txt[n] = 0;
    xSemaphoreGive(s_ack);
}

static bool tx(const uint8_t *b, size_t n) { return lmesh_send_udp(s_node, lmesh_next_seq(), LMESH_PORT_FW, LMESH_PORT_FW, b, n) == ESP_OK; }

static void wr32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

static void finish(int st)
{
    if (st != 2) { uint8_t a = FW_ABORT; if (st == 4) tx(&a, 1); }
    s_state = st;
    vTaskDelete(NULL);
}

static void node_fw(uint16_t addr, char *out, size_t n)
{
    static lmesh_node_t all[LMESH_MAX_NODES];
    int cnt = lmesh_nodes(all, LMESH_MAX_NODES);
    out[0] = 0;
    for (int i = 0; i < cnt; i++) if (all[i].addr == addr) { snprintf(out, n, "%s", all[i].fw); return; }
}

static int fw_cmp(const char *a, const char *b)
{
    int x[3] = { 0 }, y[3] = { 0 };
    sscanf(a, "%d.%d.%d", &x[0], &x[1], &x[2]);
    sscanf(b, "%d.%d.%d", &y[0], &y[1], &y[2]);
    for (int i = 0; i < 3; i++) if (x[i] != y[i]) return x[i] - y[i];
    return 0;
}

static void task(void *arg)
{
    static const int wait_ms[] = { 1200, 1600, 4500 };            /* ACK wait per air rate */
    int w = wait_ms[g_cfg.e52_rate > 2 ? 2 : g_cfg.e52_rate];
    const esp_partition_t *p = store();
    uint8_t pkt[FW_DATA_HDR + FW_CHUNK];
    char was[16];
    node_fw(s_node, was, sizeof was);
    say("node %04X runs %s; sending %s, %lu bytes", s_node, was[0] ? was : "?", s_img.ver, (unsigned long)s_img.size);

    /* BEGIN: the node erases its idle slot before it answers, which takes a while */
    pkt[0] = FW_BEGIN; wr32(pkt + 1, s_img.size); wr32(pkt + 5, s_img.crc);
    bool ok = false;
    for (int i = 0; i < 6 && !ok && !s_cancel; i++) {
        xSemaphoreTake(s_ack, 0);
        if (tx(pkt, 9)) ok = xSemaphoreTake(s_ack, pdMS_TO_TICKS(15000)) == pdTRUE;
    }
    if (s_cancel) { say("cancelled"); finish(4); }
    if (!ok) { say("FAILED: node does not answer"); finish(3); }
    if (s_ack_st == FW_ST_ERR) { say("FAILED: node refused: %s", s_ack_txt); finish(3); }
    uint32_t off = s_ack_exp;
    if (off) say("resuming at %lu %%", (unsigned long)((uint64_t)off * 100 / s_img.size));
    s_sent = off;

    int misses = 0, resent = 0;
    while (off < s_img.size) {
        if (s_cancel) { say("cancelled at %lu %%", (unsigned long)((uint64_t)off * 100 / s_img.size)); finish(4); }
        size_t n = s_img.size - off > FW_CHUNK ? FW_CHUNK : s_img.size - off;
        pkt[0] = FW_DATA; wr32(pkt + 1, off);
        if (esp_partition_read(p, off, pkt + FW_DATA_HDR, n) != ESP_OK) { say("FAILED: cannot read the stored image"); finish(3); }
        uint16_t c16 = esp_rom_crc16_le(0, pkt + FW_DATA_HDR, n);
        pkt[5] = c16; pkt[6] = c16 >> 8;
        xSemaphoreTake(s_ack, 0);
        bool got = tx(pkt, FW_DATA_HDR + n);
        /* wait for an ACK that moves us; ACKs that repeat the old position are echoes of duplicates */
        int64_t until = esp_timer_get_time() + (int64_t)w * 1000;
        bool moved = false;
        while (got && !moved) {
            int64_t left = (until - esp_timer_get_time()) / 1000;
            if (left <= 0 || xSemaphoreTake(s_ack, pdMS_TO_TICKS(left)) != pdTRUE) break;
            if (s_ack_st == FW_ST_ERR) { say("FAILED at %lu %%: %s", (unsigned long)((uint64_t)off * 100 / s_img.size), s_ack_txt); finish(3); }
            if (s_ack_exp != off) { off = s_ack_exp; moved = true; }
        }
        if (moved) { misses = 0; s_sent = off; continue; }
        resent++;
        if (++misses >= 40) {
            say("FAILED at %lu %%: link lost. Start again to resume from here.", (unsigned long)((uint64_t)off * 100 / s_img.size));
            s_state = 3; vTaskDelete(NULL);              /* no ABORT: the node keeps what it has */
        }
        if (misses % 5 == 0) vTaskDelay(pdMS_TO_TICKS(3000));   /* let the mesh find its route again */
    }
    uint32_t secs = up_s() - s_t0;
    say("all %lu bytes delivered in %lu min %02lu s (%d packets repeated); node is checking the image",
        (unsigned long)s_img.size, (unsigned long)(secs / 60), (unsigned long)(secs % 60), resent);

    pkt[0] = FW_END;
    ok = false;
    for (int i = 0; i < 8 && !ok; i++) {
        xSemaphoreTake(s_ack, 0);
        if (tx(pkt, 1) && xSemaphoreTake(s_ack, pdMS_TO_TICKS(w + 4000)) == pdTRUE) {
            if (s_ack_st == FW_ST_ERR) { say("FAILED: node rejected the image: %s", s_ack_txt); finish(3); }
            ok = s_ack_st == FW_ST_DONE;
        }
    }
    if (!ok) say("no final answer from the node; checking whether it restarted anyway");
    else say("node accepted the image and is restarting (%s)", s_ack_txt[0] ? s_ack_txt : "no note");

    /* the node keeps the new firmware only after it reaches us; watch for it */
    vTaskDelay(pdMS_TO_TICKS(8000));
    for (int i = 0; i < 30; i++) {
        if (lmesh_ping(s_node, w + 4000) >= 0) {
            char now[16];
            vTaskDelay(pdMS_TO_TICKS(3000));
            node_fw(s_node, now, sizeof now);
            say("DONE: node %04X is back and runs %s", s_node, now[0] ? now : "?");
            finish(2);
        }
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
    say("FAILED: node did not come back within 5 minutes. If the new firmware cannot reach the gateway it returns to the old one by itself after 10 minutes.");
    finish(3);
}

/* ---- web ---- */

/* POST body = lmesh_node.bin; stored in the idle slot */
esp_err_t nodefw_http_upload(httpd_req_t *r)
{
    char msg[200];
    const char *err = NULL;
    const esp_partition_t *p = store();
    int left = r->content_len;
    uint8_t *buf = malloc(4096);
    uint32_t off = 0, crc = 0;
    char ver[40] = "";

    if (!buf) err = "out of memory";
    else if (s_state == 1) err = "a transfer to a node is running";
    else if (ota_pending()) err = "gateway firmware is on trial; confirm it first";
    else if (!p) err = "no second firmware slot on this gateway";
    else if (left < 1024 || (uint32_t)left > p->size) err = "file size is not plausible";
    if (!err) {
        memset(&s_img, 0, sizeof s_img);
        img_save();                                       /* a half-written image must not look valid */
        if (esp_partition_erase_range(p, 0, (left + 4095) & ~4095u) != ESP_OK) err = "flash erase failed";
    }
    while (!err && left > 0) {
        size_t fill = 0;
        while (fill < 4096 && left > 0) {                 /* whole blocks: the header check needs the first one */
            int n = httpd_req_recv(r, (char *)buf + fill, left < (int)(4096 - fill) ? left : (int)(4096 - fill));
            if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
            if (n <= 0) { err = "upload interrupted"; break; }
            fill += n; left -= n;
        }
        if (err) break;
        if (off == 0) {
            const esp_app_desc_t *d = (const esp_app_desc_t *)(buf + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
            if (buf[0] != ESP_IMAGE_HEADER_MAGIC || d->magic_word != ESP_APP_DESC_MAGIC_WORD) { err = "not an ESP firmware image"; break; }
            if (strncmp(d->project_name, NODE_PROJECT, sizeof d->project_name) != 0) { err = "this is not node firmware (expected lmesh_node.bin)"; break; }
            snprintf(ver, sizeof ver, "%.10s %.11s %.8s", d->version, d->date, d->time);
        }
        if (esp_partition_write(p, off, buf, fill) != ESP_OK) { err = "flash write failed"; break; }
        crc = esp_rom_crc32_le(crc, buf, fill);
        off += fill;
    }
    free(buf);
    httpd_resp_set_type(r, "application/json");
    if (err) {
        snprintf(msg, sizeof msg, "{\"ok\":false,\"err\":\"%s\"}", err);
        return httpd_resp_sendstr(r, msg);
    }
    s_img.size = off; s_img.crc = crc;
    snprintf(s_img.ver, sizeof s_img.ver, "%s", ver);
    img_save();
    ESP_LOGI(TAG, "stored node firmware %s, %lu bytes, crc %08lx", s_img.ver, (unsigned long)off, (unsigned long)crc);
    return httpd_resp_sendstr(r, "{\"ok\":true}");
}

/* returns NULL when started */
const char *nodefw_send(int node)
{
    char fw[16];
    if (s_state == 1) return "a transfer is already running";
    if (!s_img.size) return "upload the node firmware to the gateway first";
    if (node < 1 || node > 0x7FFF) return "bad node";
    node_fw(node, fw, sizeof fw);
    if (!fw[0]) return "that node is not in the node list";
    if (fw_cmp(fw, MIN_NODE_FW) < 0) return "that node's firmware is too old for updates over the mesh; update it once over its Wi-Fi page";
    if (!s_ack) { s_ack = xSemaphoreCreateBinary(); lmesh_bind(LMESH_PORT_FW, on_mesh_fw); }
    s_log[0] = 0;
    s_node = node; s_sent = 0; s_cancel = false; s_t0 = up_s();
    s_state = 1;
    if (xTaskCreate(task, "nodefw", 6144, NULL, 4, NULL) != pdPASS) { s_state = 0; return "no memory"; }
    return NULL;
}

void nodefw_cancel(void) { if (s_state == 1) s_cancel = true; }

/* fills a JSON-ready snapshot */
void nodefw_status(int *state, uint32_t *size, uint32_t *sent, uint32_t *secs, uint16_t *node, const char **ver, const char **log)
{
    *state = s_state; *size = s_img.size; *sent = s_sent; *node = s_node; *ver = s_img.ver; *log = s_log;
    *secs = s_state == 1 ? up_s() - s_t0 : 0;
}
