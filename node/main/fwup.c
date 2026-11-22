/*
 * Node side of the firmware update over the mesh (see shared/fwproto.h).
 * Runs on its own task so flash erases and writes never hold up the radio
 * or the RS-485 worker. The finished image goes through the same checks,
 * trial boot and rollback as an upload over Wi-Fi (shared/ota.c).
 */
#include <string.h>
#include <stdio.h>
#include "fwproto.h"
#include "ota.h"
#include "lmesh.h"
#include "esp_rom_crc.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "fwup";

typedef struct { uint16_t src; uint16_t len; uint8_t data[200]; } pkt_t;
static QueueHandle_t s_q;

static struct {
    bool active, done;
    uint32_t size, crc_want, got, crc;
    uint32_t bad;                 /* packets whose contents did not match their checksum */
    uint8_t buf[2048 + 200];
    size_t fill;
} s;

static uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

int fwup_percent(void) { return s.active && s.size ? (int)((uint64_t)s.got * 100 / s.size) : -1; }

static void ack(uint16_t to, uint8_t status, const char *text)
{
    uint8_t b[6 + 80];
    b[0] = FW_ACK;
    b[1] = s.got; b[2] = s.got >> 8; b[3] = s.got >> 16; b[4] = s.got >> 24;
    b[5] = status;
    size_t n = 6;
    if (text) { size_t l = strlen(text); if (l > 80) l = 80; memcpy(b + 6, text, l); n += l; }
    lmesh_send_udp(to, lmesh_next_seq(), LMESH_PORT_FW, LMESH_PORT_FW, b, n);
}

static void fail(uint16_t to, const char *why)
{
    ESP_LOGE(TAG, "update stopped: %s", why);
    ota_abort();
    s.active = false;
    ack(to, FW_ST_ERR, why);
}

static bool flush(void)
{
    if (!s.fill) return true;
    bool ok = ota_write(s.buf, s.fill) == ESP_OK;
    s.fill = 0;
    return ok;
}

static void handle(const pkt_t *p)
{
    const uint8_t *d = p->data;
    switch (d[0]) {
    case FW_BEGIN: {
        if (p->len < 9) return;
        uint32_t size = rd32(d + 1), crc = rd32(d + 5);
        if (s.active && size == s.size && crc == s.crc_want) {        /* resume, or a repeated BEGIN */
            ESP_LOGI(TAG, "transfer continues at %lu of %lu", (unsigned long)s.got, (unsigned long)s.size);
            ack(p->src, FW_ST_OK, NULL);
            return;
        }
        ota_abort();
        memset(&s, 0, sizeof s);
        s.size = size; s.crc_want = crc;
        ESP_LOGW(TAG, "firmware update from 0x%04X: %lu bytes", p->src, (unsigned long)size);
        if (ota_begin(size) != ESP_OK) { fail(p->src, ota_error()); return; }   /* erases the idle slot: seconds */
        s.active = true;
        ack(p->src, FW_ST_OK, NULL);
        return;
    }
    case FW_DATA: {
        if (p->len <= FW_DATA_HDR) return;
        if (!s.active) { if (!s.done) ack(p->src, FW_ST_ERR, "no transfer open"); return; }
        uint32_t off = rd32(d + 1);
        const uint8_t *data = d + FW_DATA_HDR;
        size_t n = p->len - FW_DATA_HDR;
        /* The radio's own checks are not enough: a byte lost between an MCU and
         * its module shortens the frame and shifts the rest, and that still
         * looks like a valid packet. (Seen in the first 1 MB test transfer.) */
        if (esp_rom_crc16_le(0, data, n) != (uint16_t)(d[5] | (d[6] << 8))) {
            s.bad++;
            ESP_LOGW(TAG, "damaged packet at %lu dropped (%lu so far)", (unsigned long)off, (unsigned long)s.bad);
        } else if (off == s.got && s.got + n <= s.size) {
            memcpy(s.buf + s.fill, data, n);
            s.fill += n;
            s.got += n;
            s.crc = esp_rom_crc32_le(s.crc, data, n);
            if (s.fill >= 2048 && !flush()) { fail(p->src, ota_error()); return; }
        }
        ack(p->src, FW_ST_OK, NULL);          /* always: tells the gateway where we really are */
        return;
    }
    case FW_END:
        if (s.done) { ack(p->src, FW_ST_DONE, NULL); return; }       /* our first answer was lost */
        if (!s.active) { ack(p->src, FW_ST_ERR, "no transfer open"); return; }
        if (s.got != s.size) { ack(p->src, FW_ST_OK, NULL); return; }  /* not all there yet: gateway resumes */
        if (!flush()) { fail(p->src, ota_error()); return; }
        if (s.crc != s.crc_want) { fail(p->src, "checksum mismatch"); return; }
        if (ota_finish() != ESP_OK) { s.active = false; ack(p->src, FW_ST_ERR, ota_error()); return; }
        s.active = false;
        s.done = true;
        ESP_LOGW(TAG, "update complete, restarting into the new firmware");
        char note[32];
        snprintf(note, sizeof note, "damaged=%lu", (unsigned long)s.bad);
        ack(p->src, FW_ST_DONE, note);
        vTaskDelay(pdMS_TO_TICKS(2500));
        ack(p->src, FW_ST_DONE, note);
        vTaskDelay(pdMS_TO_TICKS(2500));
        esp_restart();
        return;
    case FW_ABORT:
        if (s.active) { ESP_LOGW(TAG, "update cancelled by the gateway"); ota_abort(); s.active = false; }
        return;
    }
}

static void task(void *arg)
{
    static pkt_t p;
    for (;;) {
        xQueueReceive(s_q, &p, portMAX_DELAY);
        handle(&p);
    }
}

/* lmesh task context: just hand it over */
static void on_fw(const lmesh_dgram_t *d)
{
    static pkt_t p;
    if (d->len < 1 || d->len > sizeof p.data) return;
    p.src = d->src;
    p.len = d->len;
    memcpy(p.data, d->data, d->len);
    xQueueSend(s_q, &p, 0);
}

void fwup_start(void)
{
    s_q = xQueueCreate(3, sizeof(pkt_t));
    xTaskCreate(task, "fwup", 4096, NULL, 4, NULL);
    lmesh_bind(LMESH_PORT_FW, on_fw);
}
