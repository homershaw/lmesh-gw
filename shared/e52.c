/*
 * Ebyte E52-900NW22S LoRa MESH module driver.
 *
 * Module facts used here (E52-400/900NW22S user manual):
 *  - AT commands, 115200 8N1 default, sent WITHOUT CR/LF, reply "AT+X=OK" / "=CMD_ERR" / "=CMD_VALUE_ERR"
 *  - Anything not starting with "AT+" is transmitted as payload, max 200 bytes
 *  - With AT+HEAD=1 every received packet is output as:
 *      type(C1..C4) len panid(LE16) src(LE16) dst(LE16) data[len]
 *  - Channel n => 850.125 + n MHz (n 0..79). 902-928 MHz => n 52..77
 *  - AT+RATE 0=62.5k 1=21.875k 2=7k
 *
 * TX path: unicast destination is selected with AT+DST_ADDR=<a>,0 (RAM only,
 * no flash wear) before writing the payload. The last destination is cached.
 */
#include <string.h>
#include <stdio.h>
#include "e52.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/message_buffer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/stream_buffer.h"
#include "sdkconfig.h"
#if CONFIG_LMGW_E52_USB
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.h"
#endif

static const char *TAG = "e52";

#define UARTN       CONFIG_LMGW_E52_UART_NUM
#define RXBUF       2048
#define TEXT_MAX    512

static SemaphoreHandle_t s_tx_lock;
static MessageBufferHandle_t s_at_mb;
static e52_rx_cb_t s_cb;
static e52_stats_t s_st;
static int32_t s_cur_dst = -1;      /* cached DST_ADDR */
static int s_cur_opt = -1;          /* cached OPTION */
static int64_t s_tx_free_at_us;     /* pacing: earliest time next frame may go out */
static volatile bool s_at_waiting;
static volatile bool s_uart_ok;     /* link to module is usable (UART up, or USB device open) */

/* ---------------- link: UART pins or CH340 on USB-HS ---------------- */

#if CONFIG_LMGW_E52_USB
static StreamBufferHandle_t s_usb_rx;
static volatile cdc_acm_dev_hdl_t s_usb;
static SemaphoreHandle_t s_usb_up, s_usb_gone;

static bool usb_rx_cb(const uint8_t *d, size_t n, void *arg)
{
    if (xStreamBufferSend(s_usb_rx, d, n, 0) != n) s_st.rx_junk++;
    return true;
}

static void usb_ev_cb(const cdc_acm_host_dev_event_data_t *e, void *arg)
{
    if (e->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        s_uart_ok = false;
        s_usb = NULL;
        xSemaphoreGive(s_usb_gone);
    } else if (e->type == CDC_ACM_HOST_ERROR) {
        ESP_LOGW(TAG, "USB CDC error %d", e->data.error);
    }
}

static void usb_lib_task(void *arg)
{
    for (;;) {
        uint32_t f = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &f);
        if (f & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

/* opens the CH34x, keeps it open, reopens after unplug */
static void usb_conn_task(void *arg)
{
    const cdc_acm_host_device_config_t dc = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 256,
        .in_buffer_size = 0,
        .event_cb = usb_ev_cb,
        .data_cb = usb_rx_cb,
    };
    for (;;) {
        cdc_acm_dev_hdl_t h = NULL;
        if (ch34x_vcp_open(CH34X_PID_AUTO, 0, &dc, &h) != ESP_OK) continue;   /* 1 s poll */
        cdc_acm_line_coding_t lc = { .dwDTERate = g_cfg.uart_baud, .bCharFormat = 0,
                                     .bParityType = 0, .bDataBits = 8 };
        esp_err_t e = cdc_acm_host_line_coding_set(h, &lc);
        cdc_acm_host_set_control_line_state(h, true, true);
        ESP_LOGI(TAG, "E52 board (CH34x) connected on USB-HS @%lu %s", (unsigned long)g_cfg.uart_baud,
                 e == ESP_OK ? "" : "(line coding failed)");
        xStreamBufferReset(s_usb_rx);
        s_usb = h;
        s_uart_ok = true;
        xSemaphoreGive(s_usb_up);

        xSemaphoreTake(s_usb_gone, portMAX_DELAY);
        ESP_LOGW(TAG, "E52 board unplugged from USB");
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
        cdc_acm_host_close(h);
        s_cur_dst = -1; s_cur_opt = -1;
        xSemaphoreGive(s_tx_lock);
    }
}

static esp_err_t link_init(void)
{
    s_usb_rx = xStreamBufferCreate(RXBUF, 1);
    s_usb_up = xSemaphoreCreateBinary();
    s_usb_gone = xSemaphoreCreateBinary();
    usb_host_config_t hc = { .skip_phy_setup = false, .intr_flags = 0 };
    esp_err_t e = usb_host_install(&hc);
    if (e != ESP_OK) { ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(e)); return e; }
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 11, NULL);
    e = cdc_acm_host_install(NULL);
    if (e != ESP_OK) { ESP_LOGE(TAG, "cdc_acm_host_install: %s", esp_err_to_name(e)); return e; }
    xTaskCreate(usb_conn_task, "usb_e52", 4096, NULL, 10, NULL);
    if (xSemaphoreTake(s_usb_up, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "no CH34x E52 board on USB-HS port (will keep looking)");
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static int link_read(uint8_t *b, size_t n, int ms)
{
    return (int)xStreamBufferReceive(s_usb_rx, b, n, pdMS_TO_TICKS(ms));
}

static void link_write(const void *d, size_t n)
{
    cdc_acm_dev_hdl_t h = s_usb;
    if (h && cdc_acm_host_data_tx_blocking(h, d, n, 300) != ESP_OK) s_st.tx_fail++;
}
#else
static esp_err_t link_init(void)
{
    uart_config_t uc = {
        .baud_rate = (int)g_cfg.uart_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* not fatal: a bad pin/baud setting must not boot-loop; web UI stays up to fix it */
    esp_err_t err = uart_driver_install(UARTN, RXBUF, 1024, 0, NULL, 0);
    if (err == ESP_OK) err = uart_param_config(UARTN, &uc);
    if (err == ESP_OK) err = uart_set_pin(UARTN, g_cfg.uart_tx, g_cfg.uart_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART%d setup failed: %s", UARTN, esp_err_to_name(err));
        return err;
    }
    s_uart_ok = true;
    return ESP_OK;
}

static int link_read(uint8_t *b, size_t n, int ms) { return uart_read_bytes(UARTN, b, n, pdMS_TO_TICKS(ms)); }

static void link_write(const void *d, size_t n)
{
    uart_write_bytes(UARTN, d, n);
    uart_wait_tx_done(UARTN, pdMS_TO_TICKS(100));
}
#endif

uint32_t e52_airtime_ms(size_t len)
{
    static const uint32_t bps[] = {62500, 21875, 7000};
    uint32_t r = bps[g_cfg.e52_rate > 2 ? 2 : g_cfg.e52_rate];
    /* + ~16 bytes of module mesh header, + preamble/turnaround margin */
    return (uint32_t)(((len + 16) * 8 * 1000) / r) + 40;
}

/* ---------------- RX ---------------- */

static void rx_task(void *arg)
{
    static uint8_t buf[256];
    static char text[TEXT_MAX];
    static e52_frame_t fr;
    size_t tlen = 0;
    enum { S_IDLE, S_HDR, S_DATA } st = S_IDLE;
    uint8_t hdr[8];
    int hpos = 0, dpos = 0;

    for (;;) {
        int n = link_read(buf, sizeof buf, 20);
        if (n <= 0) {
            /* idle gap: flush text reply, abandon partial binary frame */
            if (tlen) {
                text[tlen] = 0;
                if (s_at_waiting) xMessageBufferSend(s_at_mb, text, tlen + 1, 0);
                else ESP_LOGI(TAG, "module: %s", text);
                tlen = 0;
            }
            if (st != S_IDLE) { s_st.rx_junk++; st = S_IDLE; }
            continue;
        }
        for (int i = 0; i < n; i++) {
            uint8_t b = buf[i];
            switch (st) {
            case S_IDLE:
                if (b >= 0xC1 && b <= 0xC4) {
                    hdr[0] = b; hpos = 1; st = S_HDR;
                } else if (b == '\r' || b == '\n' || (b >= 0x20 && b < 0x7F)) {
                    if (tlen < TEXT_MAX - 1) text[tlen++] = (char)b;
                } else {
                    s_st.rx_junk++;
                }
                break;
            case S_HDR:
                hdr[hpos++] = b;
                if (hpos == 8) {
                    fr.type  = hdr[0];
                    fr.len   = hdr[1];
                    fr.panid = hdr[2] | (hdr[3] << 8);
                    fr.src   = hdr[4] | (hdr[5] << 8);
                    fr.dst   = hdr[6] | (hdr[7] << 8);
                    if (fr.len == 0 || fr.len > E52_MAX_PAYLOAD) { s_st.rx_junk++; st = S_IDLE; }
                    else { dpos = 0; st = S_DATA; }
                }
                break;
            case S_DATA:
                fr.data[dpos++] = b;
                if (dpos == fr.len) {
                    s_st.rx_frames++;
                    s_st.rx_bytes += fr.len;
                    if (s_cb) s_cb(&fr);
                    st = S_IDLE;
                }
                break;
            }
        }
    }
}

/* ---------------- TX / AT ---------------- */

static esp_err_t at_locked(const char *cmd, char *resp, size_t rl, int timeout_ms)
{
    char tmp[TEXT_MAX];
    xMessageBufferReset(s_at_mb);
    s_at_waiting = true;
    link_write(cmd, strlen(cmd));
    size_t n = xMessageBufferReceive(s_at_mb, tmp, sizeof tmp, pdMS_TO_TICKS(timeout_ms));
    s_at_waiting = false;
    if (n == 0) {
        s_st.at_err++;
        if (resp && rl) resp[0] = 0;
        ESP_LOGW(TAG, "%s -> (timeout)", cmd);
        return ESP_ERR_TIMEOUT;
    }
    if (resp && rl) { strncpy(resp, tmp, rl - 1); resp[rl - 1] = 0; }
    bool err = strstr(tmp, "ERR") != NULL;
    if (err) s_st.at_err++; else s_st.at_ok++;
    ESP_LOGD(TAG, "%s -> %s", cmd, tmp);
    return err ? ESP_FAIL : ESP_OK;
}

esp_err_t e52_at(const char *cmd, char *resp, size_t rl, int timeout_ms)
{
    if (!s_uart_ok) { if (resp && rl) resp[0] = 0; return ESP_ERR_INVALID_STATE; }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    esp_err_t e = at_locked(cmd, resp, rl, timeout_ms);
    /* anything could have changed; force re-selection on next send */
    if (strncmp(cmd, "AT+DST_ADDR", 11) == 0 || strncmp(cmd, "AT+OPTION", 9) == 0 ||
        strncmp(cmd, "AT+RESET", 8) == 0 || strncmp(cmd, "AT+DEFAULT", 10) == 0) {
        s_cur_dst = -1; s_cur_opt = -1;
    }
    xSemaphoreGive(s_tx_lock);
    return e;
}

static void pace_wait(void)
{
    int64_t now = esp_timer_get_time();
    if (now < s_tx_free_at_us) vTaskDelay(pdMS_TO_TICKS((s_tx_free_at_us - now) / 1000 + 1));
}

esp_err_t e52_send(uint16_t dst, const uint8_t *data, size_t len)
{
    if (len == 0 || len > E52_MAX_PAYLOAD) return ESP_ERR_INVALID_SIZE;
    if (!s_uart_ok) return ESP_ERR_INVALID_STATE;
    char cmd[32];
    esp_err_t e = ESP_OK;
    int opt = (dst == 0xFFFF) ? 3 : 1;

    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    pace_wait();
    if (opt != s_cur_opt) {
        snprintf(cmd, sizeof cmd, "AT+OPTION=%d,0", opt);
        e = at_locked(cmd, NULL, 0, 500);
        s_cur_opt = (e == ESP_OK) ? opt : -1;
    }
    if (e == ESP_OK && opt == 1 && (int32_t)dst != s_cur_dst) {
        snprintf(cmd, sizeof cmd, "AT+DST_ADDR=%u,0", dst);
        e = at_locked(cmd, NULL, 0, 500);
        s_cur_dst = (e == ESP_OK) ? dst : -1;
    }
    if (e == ESP_OK) {
        link_write(data, len);
        s_tx_free_at_us = esp_timer_get_time() + (int64_t)e52_airtime_ms(len) * 1000;
        s_st.tx_frames++;
        s_st.tx_bytes += len;
    } else {
        s_st.tx_fail++;
    }
    xSemaphoreGive(s_tx_lock);
    return e;
}

esp_err_t e52_reset(void)
{
    if (g_cfg.uart_rst >= 0) {
        gpio_set_level(g_cfg.uart_rst, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(g_cfg.uart_rst, 1);
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
        s_cur_dst = -1; s_cur_opt = -1;
        xSemaphoreGive(s_tx_lock);
    } else {
        e52_at("AT+RESET", NULL, 0, 1000);
    }
    vTaskDelay(pdMS_TO_TICKS(1500));
    return ESP_OK;
}

/* Settings the module does not keep across a reset (CSMA_RNG verified on
 * FW 7460-1-15), plus HEAD/BACK which our parser depends on. */
void e52_runtime_setup(void)
{
    char cmd[32];
    if (e52_at("AT+HEAD=1", NULL, 0, 800) == ESP_ERR_TIMEOUT) e52_at("AT+HEAD=1", NULL, 0, 800);
    e52_at("AT+BACK=0", NULL, 0, 800);
    snprintf(cmd, sizeof cmd, "AT+CSMA_RNG=%u", g_cfg.e52_csma);
    e52_at(cmd, NULL, 0, 800);
}

esp_err_t e52_apply_config(char *log, size_t ll)
{
    char cmds[12][40];
    int n = 0;
    __typeof__(g_cfg) *c = &g_cfg;    /* shared with node firmware (different cfg struct) */
    snprintf(cmds[n++], 40, "AT+HEAD=1");
    snprintf(cmds[n++], 40, "AT+BACK=0");
    snprintf(cmds[n++], 40, "AT+SRC_PORT=1,1");
    snprintf(cmds[n++], 40, "AT+CHANNEL=%u,1", c->e52_channel);
    snprintf(cmds[n++], 40, "AT+POWER=%d,1", c->e52_power);
    snprintf(cmds[n++], 40, "AT+PANID=%u,1", c->e52_panid);
    snprintf(cmds[n++], 40, "AT+SRC_ADDR=%u,1", c->e52_addr);
    snprintf(cmds[n++], 40, "AT+TYPE=0");                 /* gateway = routing node; after SRC_ADDR */
    snprintf(cmds[n++], 40, "AT+CSMA_RNG=%u", c->e52_csma);
    snprintf(cmds[n++], 40, "AT+RATE=%u", c->e52_rate);
    snprintf(cmds[n++], 40, "AT+OPTION=1,1");

    size_t off = 0;
    int fails = 0;
    if (log && ll) log[0] = 0;
    for (int i = 0; i < n; i++) {
        char r[96];
        esp_err_t e = e52_at(cmds[i], r, sizeof r, 800);
        if (e == ESP_ERR_TIMEOUT) e = e52_at(cmds[i], r, sizeof r, 800);   /* module can miss the first byte after idle */
        if (e != ESP_OK) fails++;
        for (char *p = r; *p; p++) if (*p == '\r' || *p == '\n') *p = ' ';
        if (log && off < ll)
            off += snprintf(log + off, ll - off, "%s -> %s\n", cmds[i], e == ESP_ERR_TIMEOUT ? "(timeout)" : r);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    e52_reset();
    e52_runtime_setup();
    ESP_LOGI(TAG, "config applied, %d failures", fails);
    return fails ? ESP_FAIL : ESP_OK;
}

void e52_get_stats(e52_stats_t *s) { *s = s_st; }

esp_err_t e52_init(e52_rx_cb_t cb)
{
    s_cb = cb;
    s_tx_lock = xSemaphoreCreateMutex();
    s_at_mb = xMessageBufferCreate(TEXT_MAX + 8);

    /* not fatal: web UI stays up so pins/baud can be fixed */
    esp_err_t err = link_init();
    if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) return err;

    if (g_cfg.uart_rst >= 0) {
        gpio_config_t g = { .pin_bit_mask = 1ULL << g_cfg.uart_rst, .mode = GPIO_MODE_OUTPUT_OD,
                            .pull_up_en = GPIO_PULLUP_ENABLE };
        gpio_config(&g);
        gpio_set_level(g_cfg.uart_rst, 1);
    }

    xTaskCreate(rx_task, "e52_rx", 4096, NULL, 12, NULL);
    if (err == ESP_ERR_NOT_FOUND) return err;
    vTaskDelay(pdMS_TO_TICKS(300));

    char r[128];
    if (e52_at("AT+HEAD=1", r, sizeof r, 800) == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "E52 not answering on UART%d TX=%d RX=%d @%lu - check wiring/baud",
                 UARTN, g_cfg.uart_tx, g_cfg.uart_rx, (unsigned long)g_cfg.uart_baud);
        return ESP_ERR_TIMEOUT;
    }
    e52_runtime_setup();
    if (e52_at("AT+SRC_ADDR=?", r, sizeof r, 800) == ESP_OK) ESP_LOGI(TAG, "SRC_ADDR: %s", r);
    if (e52_at("AT+CHANNEL=?", r, sizeof r, 800) == ESP_OK) ESP_LOGI(TAG, "CHANNEL: %s", r);
    return ESP_OK;
}
