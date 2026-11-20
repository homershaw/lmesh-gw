/*
 * Modbus RTU master on RS-485 (auto-direction transceiver: TX/RX only).
 * One transaction at a time; callers are the mesh pass-through and the poller.
 */
#include <string.h>
#include "rtu.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "rtu";
#define UARTN CONFIG_LMN_RS485_UART_NUM

static SemaphoreHandle_t s_lock;
static rtu_stats_t s_st;
static int64_t s_idle_at_us;     /* bus free after the 3.5-char gap */
static uint32_t s_char_us;

static uint16_t crc16(const uint8_t *p, int n)
{
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
    }
    return c;
}

esp_err_t rtu_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    uart_config_t u = {
        .baud_rate = (int)g_cfg.rs_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = g_cfg.rs_parity == 1 ? UART_PARITY_EVEN : g_cfg.rs_parity == 2 ? UART_PARITY_ODD : UART_PARITY_DISABLE,
        .stop_bits = g_cfg.rs_stop == 2 ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t e = uart_driver_install(UARTN, 512, 0, 0, NULL, 0);
    if (e == ESP_OK) e = uart_param_config(UARTN, &u);
    if (e == ESP_OK) e = uart_set_pin(UARTN, CONFIG_LMN_RS485_TX_GPIO, CONFIG_LMN_RS485_RX_GPIO,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    /* 11 bits per char (start + 8 + parity/stop + stop) */
    s_char_us = 11000000UL / g_cfg.rs_baud;
    ESP_LOGI(TAG, "RS-485 UART%d TX=%d RX=%d %lu %c%u: %s", UARTN, CONFIG_LMN_RS485_TX_GPIO, CONFIG_LMN_RS485_RX_GPIO,
             (unsigned long)g_cfg.rs_baud, "NEO"[g_cfg.rs_parity % 3], g_cfg.rs_stop, esp_err_to_name(e));
    return e;
}

int rtu_txn(uint8_t unit, const uint8_t *pdu, int pl, uint8_t *rsp, int rmax, int timeout_ms)
{
    if (pl < 1 || pl > 252) return RTU_ERR_BAD;
    uint8_t f[256];
    f[0] = unit;
    memcpy(f + 1, pdu, pl);
    uint16_t c = crc16(f, pl + 1);
    f[pl + 1] = c & 0xFF;
    f[pl + 2] = c >> 8;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    if (now < s_idle_at_us) vTaskDelay(pdMS_TO_TICKS((s_idle_at_us - now) / 1000 + 1));
    uart_flush_input(UARTN);
    uart_write_bytes(UARTN, f, pl + 3);
    uart_wait_tx_done(UARTN, pdMS_TO_TICKS(500));
    s_st.reqs++;

    /* first byte within timeout, then frame ends at an inter-char gap */
    int gap_ms = (int)((s_char_us * 4) / 1000) + 3;
    /* uart_read_bytes waits for the whole buffer or the timeout, so read in
     * short slices: return as soon as the reply is complete */
    int n = 0, r;
    uint8_t b[256];
    int64_t deadline = esp_timer_get_time() + (int64_t)(timeout_ms > 0 ? timeout_ms : g_cfg.rtu_timeout_ms) * 1000;
    while (n == 0 && esp_timer_get_time() < deadline) {
        r = uart_read_bytes(UARTN, b, sizeof b, pdMS_TO_TICKS(5));
        if (r > 0) n = r;
    }
    while (n > 0 && n < (int)sizeof b) {
        r = uart_read_bytes(UARTN, b + n, sizeof b - n, pdMS_TO_TICKS(gap_ms));
        if (r <= 0) break;
        n += r;
    }
    s_idle_at_us = esp_timer_get_time() + (int64_t)s_char_us * 4;

    int ret;
    if (n == 0) { ret = RTU_ERR_TIMEOUT; s_st.timeouts++; }
    else if (n < 4 || crc16(b, n - 2) != (uint16_t)(b[n - 2] | (b[n - 1] << 8))) { ret = RTU_ERR_CRC; s_st.crc_err++; }
    else if (b[0] != unit || (b[1] & 0x7F) != pdu[0]) { ret = RTU_ERR_BAD; s_st.bad++; }
    else {
        ret = n - 3;                          /* PDU length (fc + data) */
        if (ret > rmax) ret = rmax;
        memcpy(rsp, b + 1, ret);
        if (b[1] & 0x80) s_st.exceptions++; else s_st.ok++;
    }
    xSemaphoreGive(s_lock);
    if (ret < 0) ESP_LOGD(TAG, "unit %u fc %u: err %d (%d bytes)", unit, pdu[0], ret, n);
    return ret;
}

void rtu_get_stats(rtu_stats_t *s) { *s = s_st; }
