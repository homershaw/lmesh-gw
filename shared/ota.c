/*
 * Firmware update, shared by the gateway and the node.
 *
 * The flash holds two app slots. An upload is written to the idle slot,
 * verified, and booted "on trial": the bootloader goes back to the previous
 * firmware on the next reset unless the new one has been confirmed. Each
 * project supplies the test that proves it is still reachable (ota_watch);
 * if the test does not pass in time the device restarts into the old firmware.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ota.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota";
static volatile bool s_pending;          /* running firmware is on trial */
static volatile uint32_t s_deadline_s;   /* uptime at which an unconfirmed trial is abandoned */
static bool (*s_ok_fn)(void);

static uint32_t up_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

bool ota_pending(void) { return s_pending; }
int  ota_trial_left_s(void) { return s_pending ? (int)(s_deadline_s - up_s()) : 0; }
const char *ota_slot(void) { const esp_partition_t *p = esp_ota_get_running_partition(); return p ? p->label : "?"; }

void ota_confirm(void)
{
    if (!s_pending) return;
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        s_pending = false;
        ESP_LOGW(TAG, "new firmware confirmed, keeping it");
    }
}

static void watch_task(void *arg)
{
    while (s_pending) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!s_pending) break;
        if (s_ok_fn && s_ok_fn()) { ota_confirm(); break; }
        if (up_s() >= s_deadline_s) {
            ESP_LOGE(TAG, "new firmware was not confirmed in time: restarting into the previous one");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_ota_mark_app_invalid_rollback_and_reboot();
            esp_restart();               /* only reached if there is nothing to go back to */
        }
    }
    vTaskDelete(NULL);
}

void ota_watch(bool (*ok_fn)(void), int timeout_s)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (!run || esp_ota_get_state_partition(run, &st) != ESP_OK || st != ESP_OTA_IMG_PENDING_VERIFY) return;
    s_ok_fn = ok_fn;
    s_deadline_s = up_s() + timeout_s;
    s_pending = true;
    ESP_LOGW(TAG, "running new firmware on trial from %s; %d s to confirm", run->label, timeout_s);
    xTaskCreate(watch_task, "ota_watch", 4096, NULL, 3, NULL);
}

/* ---- streamed write, used by the HTTP upload now and the mesh transfer later ---- */

static esp_ota_handle_t s_h;
static const esp_partition_t *s_part;
static size_t s_got;
static bool s_open;
static char s_err[96];

const char *ota_error(void) { return s_err; }

esp_err_t ota_begin(size_t size)
{
    if (s_open) { esp_ota_abort(s_h); s_open = false; }
    s_err[0] = 0;
    s_got = 0;
    s_part = esp_ota_get_next_update_partition(NULL);
    if (!s_part) { snprintf(s_err, sizeof s_err, "no second firmware slot (flash this build once over USB)"); return ESP_ERR_NOT_FOUND; }
    if (size > s_part->size) { snprintf(s_err, sizeof s_err, "image is larger than the firmware slot"); return ESP_ERR_INVALID_SIZE; }
    esp_err_t e = esp_ota_begin(s_part, size ? size : OTA_SIZE_UNKNOWN, &s_h);
    if (e != ESP_OK) { snprintf(s_err, sizeof s_err, "cannot open slot %s: %s", s_part->label, esp_err_to_name(e)); return e; }
    s_open = true;
    return ESP_OK;
}

esp_err_t ota_write(const uint8_t *d, size_t n)
{
    if (!s_open) return ESP_ERR_INVALID_STATE;
    /* the app description sits right after the image and first segment headers */
    const size_t off = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
    if (s_got == 0) {
        if (n < off + sizeof(esp_app_desc_t)) { snprintf(s_err, sizeof s_err, "first block too short"); goto fail; }
        const esp_app_desc_t *nw = (const esp_app_desc_t *)(d + off), *cur = esp_app_get_description();
        if (d[0] != ESP_IMAGE_HEADER_MAGIC || nw->magic_word != ESP_APP_DESC_MAGIC_WORD) {
            snprintf(s_err, sizeof s_err, "not an ESP firmware image"); goto fail;
        }
        if (strncmp(nw->project_name, cur->project_name, sizeof nw->project_name) != 0) {
            snprintf(s_err, sizeof s_err, "wrong firmware: this is '%.24s', the file is '%.24s'", cur->project_name, nw->project_name);
            goto fail;
        }
    }
    esp_err_t e = esp_ota_write(s_h, d, n);
    if (e != ESP_OK) { snprintf(s_err, sizeof s_err, "flash write failed: %s", esp_err_to_name(e)); goto fail; }
    s_got += n;
    return ESP_OK;
fail:
    esp_ota_abort(s_h);
    s_open = false;
    return ESP_FAIL;
}

void ota_abort(void) { if (s_open) { esp_ota_abort(s_h); s_open = false; } }

/* verify the image and select it for the next boot */
esp_err_t ota_finish(void)
{
    if (!s_open) return ESP_ERR_INVALID_STATE;
    s_open = false;
    esp_err_t e = esp_ota_end(s_h);
    if (e != ESP_OK) { snprintf(s_err, sizeof s_err, "image check failed (%s): file damaged or for another chip", esp_err_to_name(e)); return e; }
    e = esp_ota_set_boot_partition(s_part);
    if (e != ESP_OK) { snprintf(s_err, sizeof s_err, "cannot select new firmware: %s", esp_err_to_name(e)); return e; }
    ESP_LOGW(TAG, "%u bytes written to %s; it runs on trial after the restart", (unsigned)s_got, s_part->label);
    return ESP_OK;
}

static void reboot_task(void *a) { vTaskDelay(pdMS_TO_TICKS(1200)); esp_restart(); }

/* POST body = the .bin file. Replies with JSON and restarts on success. */
esp_err_t ota_http_upload(httpd_req_t *r)
{
    char msg[160];
    int left = r->content_len;
    uint8_t *buf = malloc(4096);
    esp_err_t e = buf ? ota_begin(r->content_len) : ESP_ERR_NO_MEM;
    if (!buf) snprintf(s_err, sizeof s_err, "out of memory");
    size_t fill = 0;
    bool first = true;
    while (e == ESP_OK && left > 0) {
        int n = httpd_req_recv(r, (char *)buf + fill, left < (int)(4096 - fill) ? left : (int)(4096 - fill));
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) { snprintf(s_err, sizeof s_err, "upload interrupted"); e = ESP_FAIL; break; }
        left -= n;
        fill += n;
        if (first && fill < 512 && left > 0) continue;      /* need the whole header in the first block */
        first = false;
        e = ota_write(buf, fill);
        fill = 0;
    }
    if (e == ESP_OK && r->content_len < 1024) { snprintf(s_err, sizeof s_err, "file too small"); e = ESP_FAIL; }
    if (e == ESP_OK) e = ota_finish(); else ota_abort();
    free(buf);
    httpd_resp_set_type(r, "application/json");
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "upload rejected: %s", s_err);
        snprintf(msg, sizeof msg, "{\"ok\":false,\"err\":\"%s\"}", s_err);
        return httpd_resp_sendstr(r, msg);
    }
    httpd_resp_sendstr(r, "{\"ok\":true}");
    xTaskCreate(reboot_task, "ota_rb", 2048, NULL, 5, NULL);
    return ESP_OK;
}
