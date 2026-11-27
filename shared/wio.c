/*
 * Seeed Wio-E5 (STM32WLE5) as a plain LoRa radio in a star network.
 *
 * The factory AT firmware's test mode sends and receives raw LoRa packets
 * (AT+TEST=TXLRPKT / AT+TEST=RXLRPKT, payload as hex text) on a frequency,
 * spreading factor, bandwidth and power we choose. It adds nothing else: no
 * addressing, no acknowledgement, no retries. This file adds those and
 * implements the E52 driver's interface (e52.h), so the LMESH datagram layer
 * and everything above it (MQTT, Modbus, firmware updates, survey) run
 * unchanged. Build with CONFIG_LMESH_RADIO_WIO.
 *
 * Air frame (the LoRa payload):
 *   b0      0xB0 | kind   1 = data, ack wanted   2 = data, no ack (broadcast)   3 = ack
 *   b1..b2  network id    (e52_panid, little endian): frames of other networks are ignored
 *   b3..b4  destination   (0xFFFF = everyone)
 *   b5..b6  source
 *   b7      sequence      (an ack echoes it; a repeated frame keeps it, so duplicates are dropped)
 *   b8..    payload       (<= 200 bytes: one LMESH frame)
 *
 * Every unicast frame is acknowledged by the receiver straight away; the
 * sender repeats it up to 3 times, after a random pause, if no ack comes.
 * One frame is in flight at a time. There is no routing: every node must
 * hear the gateway (or, later, a configured repeater).
 *
 * Radio settings: 500 kHz bandwidth on one fixed channel (as we read RSS-247 /
 * FCC 15.247, a signal this wide may use up to 1 W without hopping). Channel
 * numbers are the E52's (850.125 + n MHz). e52_rate picks the spreading
 * factor: 0..5 = SF5, SF7, SF9, SF10, SF11, SF12, so 0..2 match the E52's
 * three rates in speed and sensitivity.
 *
 * Measured with AT firmware 4.0.11 (bench, 24 Nov 2026): from a TXLRPKT
 * command to the end of the transmission takes the airtime plus about 100 ms,
 * at every spreading factor. That, not airtime, sets the pace of short
 * exchanges. A firmware of our own on the STM32WL would remove it.
 *
 * Link to the board: UART pins (node), or its USB-serial chip on the S31's
 * USB host port (gateway; CP2102N on the Wio-E5 mini, CH340 on the dev
 * board). The AT firmware's UART stays at whatever baud rate was saved in
 * the board (AT+UART=BR,115200 once); g_cfg.uart_baud must match it.
 * On the Wio-E5 mini the header TX/RX pins (PB6/PB7) are also driven by the
 * CP2102N: a node wired there needs R3 removed (see docs).
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "e52.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "sdkconfig.h"
#if CONFIG_LMGW_E52_USB
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.h"
#include "usb/vcp_cp210x.h"
#endif

static const char *TAG = "wio";

#define UARTN       CONFIG_LMGW_E52_UART_NUM
#define RXBUF       2048
#define WIO_LINE    600         /* +TEST: RX "<2 x 208 hex>" */
#define HDR         8
#define MAGIC       0xB0
#define K_DATA      1
#define K_BCAST     2
#define K_ACK       3
#define TRIES       4           /* first send + 3 repeats */
#define AT_TX_MS    100         /* AT firmware's own time per transmission (measured) */
#define PEERS       32
#define ACKQ        4
#define PEER_FRESH_MS (10 * 60 * 1000)

typedef struct { uint16_t dst; uint8_t len; uint8_t data[E52_MAX_PAYLOAD]; } txreq_t;
typedef struct { uint16_t addr; uint8_t seq; bool seq_ok; int16_t rssi; int8_t snr; uint32_t heard_ms; } peer_t;

static e52_rx_cb_t s_cb;
static e52_stats_t s_st;
static QueueHandle_t s_txq;
static SemaphoreHandle_t s_port;        /* whoever talks to the module holds this */
static SemaphoreHandle_t s_peer_lock;
static volatile bool s_link_ok, s_need_setup;
static peer_t s_peers[PEERS];
static uint8_t s_seq;
static int16_t s_rx_rssi;               /* from the "+TEST: LEN" line before each packet */
static int8_t s_rx_snr;

/* frame waiting for its ack (radio task only) */
static struct {
    bool active, acked;
    uint16_t dst;
    uint8_t seq, tries, len;
    int64_t deadline_us, resend_at_us;
    uint8_t f[HDR + E52_MAX_PAYLOAD];
} s_out;

/* acks owed (set while reading lines, sent by the radio task) */
static struct { uint16_t dst; uint8_t seq; } s_ackq[ACKQ];
static int s_ackn;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/* ---------------- link: UART pins, or USB-serial chip on USB-HS ---------------- */

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
        s_link_ok = false;
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

/* opens the board (CP2102N or CH340), keeps it open, reopens after unplug */
static void usb_conn_task(void *arg)
{
    const cdc_acm_host_device_config_t dc = {
        .connection_timeout_ms = 500,
        .out_buffer_size = 512,             /* one TXLRPKT line is up to ~440 bytes */
        .in_buffer_size = 0,
        .event_cb = usb_ev_cb,
        .data_cb = usb_rx_cb,
    };
    for (;;) {
        cdc_acm_dev_hdl_t h = NULL;
        const char *chip = "CP2102N";
        if (cp210x_vcp_open(CP210X_PID_AUTO, 0, &dc, &h) != ESP_OK) {
            chip = "CH340";
            if (ch34x_vcp_open(CH34X_PID_AUTO, 0, &dc, &h) != ESP_OK) continue;
        }
        cdc_acm_line_coding_t lc = { .dwDTERate = g_cfg.uart_baud, .bCharFormat = 0,
                                     .bParityType = 0, .bDataBits = 8 };
        esp_err_t e = cdc_acm_host_line_coding_set(h, &lc);
        /* DTR is wired to the module's reset through a capacitor on the mini:
         * leave the control lines alone so opening the port does not reset it */
        ESP_LOGI(TAG, "Wio-E5 board (%s) connected on USB-HS @%lu %s", chip, (unsigned long)g_cfg.uart_baud,
                 e == ESP_OK ? "" : "(line coding failed)");
        xStreamBufferReset(s_usb_rx);
        s_usb = h;
        s_link_ok = true;
        s_need_setup = true;
        xSemaphoreGive(s_usb_up);

        xSemaphoreTake(s_usb_gone, portMAX_DELAY);
        ESP_LOGW(TAG, "Wio-E5 board unplugged from USB");
        xSemaphoreTake(s_port, portMAX_DELAY);
        cdc_acm_host_close(h);
        xSemaphoreGive(s_port);
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
    xTaskCreate(usb_conn_task, "usb_wio", 4096, NULL, 10, NULL);
    if (xSemaphoreTake(s_usb_up, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "no Wio-E5 board on USB-HS port (will keep looking)");
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static int link_read(uint8_t *b, size_t n, TickType_t ticks)
{
    return (int)xStreamBufferReceive(s_usb_rx, b, n, ticks);
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
    s_link_ok = true;
    s_need_setup = true;
    return ESP_OK;
}

/* Whatever has arrived, waiting up to ticks for the first byte only, like the
 * USB stream buffer. uart_read_bytes() alone waits until all n bytes are in,
 * so every reply would sit in the buffer until the caller's timeout ran out. */
static int link_read(uint8_t *b, size_t n, TickType_t ticks)
{
    size_t avail = 0;
    uart_get_buffered_data_len(UARTN, &avail);
    if (avail == 0) {
        int got = uart_read_bytes(UARTN, b, 1, ticks);
        if (got <= 0) return got;
        uart_get_buffered_data_len(UARTN, &avail);
        if (avail > n - 1) avail = n - 1;
        int more = avail ? uart_read_bytes(UARTN, b + 1, avail, 0) : 0;
        return 1 + (more > 0 ? more : 0);
    }
    if (avail > n) avail = n;
    return uart_read_bytes(UARTN, b, avail, 0);
}

static void link_write(const void *d, size_t n)
{
    uart_write_bytes(UARTN, d, n);
}
#endif

/* ---------------- radio settings, airtime ---------------- */

static int sf(void)
{
    static const uint8_t t[] = { 5, 7, 9, 10, 11, 12 };
    return t[g_cfg.e52_rate > 5 ? 5 : g_cfg.e52_rate];
}

/* LoRa time on air of pl payload bytes: 500 kHz, CR 4/5, 12-symbol preamble,
 * explicit header, CRC on (Semtech AN1200.13; low data rate optimisation is
 * never needed at 500 kHz) */
static uint32_t air_ms(size_t pl)
{
    int s = sf();
    double ts = (double)(1 << s) / 500.0;
    int num = 8 * (int)pl - 4 * s + 28 + 16, den = 4 * s;
    int n = num > 0 ? (num + den - 1) / den : 0;
    double sym = 12 + 4.25 + (s < 7 ? 2 : 0) + 8 + n * 5;
    return (uint32_t)(sym * ts + 0.999);
}

/* time from handing a frame to the module to the end of its transmission */
uint32_t e52_airtime_ms(size_t len) { return air_ms(len + HDR) + AT_TX_MS; }

/* ---------------- lines from the module ---------------- */

static char s_lbuf[WIO_LINE];
static size_t s_llen;

/* Next complete line (no CR/LF) within ms, or 0. Caller holds s_port. */
static int read_line(char *out, size_t max, int ms)
{
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    bool first = true;
    for (;;) {
        char *nl = memchr(s_lbuf, '\n', s_llen);
        if (nl) {
            size_t n = nl - s_lbuf, c = n;
            if (c && s_lbuf[c - 1] == '\r') c--;
            if (c >= max) c = max - 1;
            memcpy(out, s_lbuf, c);
            out[c] = 0;
            memmove(s_lbuf, nl + 1, s_llen - n - 1);
            s_llen -= n + 1;
            if (c) return (int)c;
            continue;
        }
        if (s_llen == sizeof s_lbuf) { s_llen = 0; s_st.rx_junk++; }   /* no line end in 600 bytes */
        int64_t left = end - esp_timer_get_time();
        if (left <= 0 && !first) return 0;
        first = false;
        TickType_t tk = left > 0 ? pdMS_TO_TICKS((left + 999) / 1000) : 0;
        if (left > 0 && tk == 0) tk = 1;
        int got = link_read((uint8_t *)s_lbuf + s_llen, sizeof s_lbuf - s_llen, tk);
        if (got > 0) s_llen += got;
    }
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static peer_t *peer(uint16_t a, bool make)
{
    peer_t *free_p = NULL, *old = &s_peers[0];
    for (int i = 0; i < PEERS; i++) {
        if (s_peers[i].addr == a && s_peers[i].heard_ms) return &s_peers[i];
        if (!s_peers[i].heard_ms && !free_p) free_p = &s_peers[i];
        if (s_peers[i].heard_ms < old->heard_ms) old = &s_peers[i];
    }
    if (!make) return NULL;
    peer_t *p = free_p ? free_p : old;
    memset(p, 0, sizeof *p);
    p->addr = a;
    return p;
}

/* a LoRa packet arrived (radio task or e52_at caller, holding s_port) */
static void on_packet(const uint8_t *f, size_t n)
{
    static e52_frame_t fr;
    if (n < HDR || (f[0] & 0xF0) != MAGIC) { s_st.rx_junk++; return; }   /* not ours, e.g. an E52 mesh frame */
    uint8_t kind = f[0] & 0x0F, seq = f[7];
    uint16_t net = f[1] | (f[2] << 8), dst = f[3] | (f[4] << 8), src = f[5] | (f[6] << 8);
    if (net != g_cfg.e52_panid) return;

    xSemaphoreTake(s_peer_lock, portMAX_DELAY);
    peer_t *p = peer(src, true);
    p->rssi = s_rx_rssi;
    p->snr = s_rx_snr;
    p->heard_ms = now_ms();
    bool dup = false;
    if (kind == K_DATA && dst == g_cfg.e52_addr) {
        dup = p->seq_ok && p->seq == seq;
        p->seq = seq;
        p->seq_ok = true;
    }
    xSemaphoreGive(s_peer_lock);

    if (dst != g_cfg.e52_addr && dst != 0xFFFF) return;        /* someone else's traffic */
    if (kind == K_ACK) {
        if (s_out.active && src == s_out.dst && seq == s_out.seq) s_out.acked = true;
        return;
    }
    if (kind == K_DATA) {
        if (s_ackn < ACKQ) { s_ackq[s_ackn].dst = src; s_ackq[s_ackn].seq = seq; s_ackn++; }
        if (dup) return;                                         /* our ack was lost; ack again, deliver once */
    } else if (kind != K_BCAST) {
        s_st.rx_junk++;
        return;
    }
    size_t len = n - HDR;
    if (len == 0 || len > E52_MAX_PAYLOAD) { s_st.rx_junk++; return; }
    fr.type = kind == K_BCAST ? 0xC3 : 0xC1;
    fr.panid = net;
    fr.src = src;
    fr.dst = dst;
    fr.len = (uint8_t)len;
    memcpy(fr.data, f + HDR, len);
    s_st.rx_frames++;
    s_st.rx_bytes += len;
    if (s_cb) s_cb(&fr);
}

/* Handle a line if it is a received-packet report; false if it is something else. */
static bool on_line(const char *l)
{
    int len, rssi, snr;
    if (sscanf(l, "+TEST: LEN:%d, RSSI:%d, SNR:%d", &len, &rssi, &snr) == 3) {
        s_rx_rssi = (int16_t)rssi;
        s_rx_snr = (int8_t)snr;
        return true;
    }
    if (strncmp(l, "+TEST: RX \"", 11) == 0) {
        static uint8_t f[HDR + E52_MAX_PAYLOAD + 47];
        size_t n = 0;
        const char *h = l + 11;
        while (h[0] && h[1] && h[0] != '"' && n < sizeof f) {
            int a = hexval(h[0]), b = hexval(h[1]);
            if (a < 0 || b < 0) { s_st.rx_junk++; return true; }
            f[n++] = (uint8_t)(a << 4 | b);
            h += 2;
        }
        on_packet(f, n);
        return true;
    }
    return false;
}

/* Read lines until one contains want; packet reports seen meanwhile are handled. */
static bool wait_for(const char *want, int ms, char *got, size_t gl)
{
    static char l[WIO_LINE];
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    for (;;) {
        int64_t left = (end - esp_timer_get_time()) / 1000;
        if (left <= 0) return false;
        if (!read_line(l, sizeof l, (int)left)) return false;
        if (on_line(l)) continue;                       /* a packet, not the reply we wait for */
        if (strstr(l, want)) { if (got && gl) snprintf(got, gl, "%s", l); return true; }
        ESP_LOGD(TAG, "module: %s", l);
    }
}

/* Send one AT command and wait for its reply line (+X: ...). Caller holds s_port. */
static esp_err_t cmd_locked(const char *cmd, char *resp, size_t rl, int ms)
{
    link_write(cmd, strlen(cmd));
    link_write("\r\n", 2);
    char r[160];
    if (!wait_for("+", ms, r, sizeof r)) {
        s_st.at_err++;
        if (resp && rl) resp[0] = 0;
        ESP_LOGW(TAG, "%s -> (timeout)", cmd);
        return ESP_ERR_TIMEOUT;
    }
    if (resp && rl) snprintf(resp, rl, "%s", r);
    bool err = strstr(r, "ERROR") != NULL;
    if (err) { s_st.at_err++; ESP_LOGW(TAG, "%s -> %s", cmd, r); }
    else s_st.at_ok++;
    return err ? ESP_FAIL : ESP_OK;
}

static esp_err_t rx_on_locked(void)
{
    link_write("AT+TEST=RXLRPKT\r\n", 17);
    return wait_for("+TEST: RXLRPKT", 500, NULL, 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* test mode, radio settings, receive. Caller holds s_port. */
static esp_err_t setup_locked(char *log, size_t ll)
{
    char c[96], r[160];
    size_t off = 0;
    int fails = 0;
    float mhz = e52_chan_mhz(g_cfg.e52_channel);
    /* test mode takes -1..22 dBm; outside that RFCFG fails and the old settings stay */
    int pw = g_cfg.e52_power > 22 ? 22 : g_cfg.e52_power < -1 ? -1 : g_cfg.e52_power;
    const char *cmds[4];
    char rf[96];
    snprintf(rf, sizeof rf, "AT+TEST=RFCFG,%.3f,SF%d,500,12,15,%d,ON,OFF,OFF", mhz, sf(), pw);
    cmds[0] = "AT";
    cmds[1] = "AT+MODE=TEST";
    cmds[2] = rf;
    cmds[3] = NULL;
    if (log && ll) log[0] = 0;
    for (int i = 0; cmds[i]; i++) {
        snprintf(c, sizeof c, "%s", cmds[i]);
        esp_err_t e = cmd_locked(c, r, sizeof r, 1000);
        if (e == ESP_ERR_TIMEOUT && i == 0) e = cmd_locked(c, r, sizeof r, 1000);   /* first command after reset */
        if (e != ESP_OK) fails++;
        if (log && off < ll) off += snprintf(log + off, ll - off, "%s -> %s\n", c, e == ESP_ERR_TIMEOUT ? "(timeout)" : r);
        if (e == ESP_OK && i == 2) ESP_LOGI(TAG, "%s", r);
    }
    esp_err_t e = rx_on_locked();
    if (e != ESP_OK) fails++;
    if (log && off < ll) off += snprintf(log + off, ll - off, "AT+TEST=RXLRPKT -> %s\n", e == ESP_OK ? "receiving" : "(timeout)");
    s_need_setup = fails != 0;
    return fails ? ESP_FAIL : ESP_OK;
}

/* ---------------- transmit ---------------- */

static char s_txline[32 + 2 * (HDR + E52_MAX_PAYLOAD)];

/* Transmit one frame and go back to receiving. Caller holds s_port. */
static bool tx_locked(const uint8_t *f, size_t n)
{
    static const char hx[] = "0123456789ABCDEF";
    size_t o = (size_t)snprintf(s_txline, sizeof s_txline, "AT+TEST=TXLRPKT,\"");
    for (size_t i = 0; i < n; i++) { s_txline[o++] = hx[f[i] >> 4]; s_txline[o++] = hx[f[i] & 15]; }
    s_txline[o++] = '"';
    s_txline[o++] = '\r';
    s_txline[o++] = '\n';
    link_write(s_txline, o);
    bool ok = wait_for("TX DONE", (int)(air_ms(n) + AT_TX_MS + 600), NULL, 0);
    if (ok) { s_st.tx_frames++; s_st.tx_bytes += n > HDR ? n - HDR : 0; }
    else { s_st.tx_fail++; ESP_LOGW(TAG, "no TX DONE for a %u-byte frame", (unsigned)n); }
    if (rx_on_locked() != ESP_OK) s_need_setup = true;
    return ok;
}

static void make_hdr(uint8_t *f, uint8_t kind, uint16_t dst, uint8_t seq)
{
    f[0] = MAGIC | kind;
    f[1] = g_cfg.e52_panid & 0xFF; f[2] = g_cfg.e52_panid >> 8;
    f[3] = dst & 0xFF;             f[4] = dst >> 8;
    f[5] = g_cfg.e52_addr & 0xFF;  f[6] = g_cfg.e52_addr >> 8;
    f[7] = seq;
}

/* How long to listen for an ack: the receiver's AT firmware needs ~100 ms
 * before its ack goes out, plus the ack's airtime, plus slack for its own
 * line handling. */
static uint32_t ack_wait_ms(void) { return AT_TX_MS + air_ms(HDR) * 2 + 250; }

static void radio_task(void *arg)
{
    static char l[WIO_LINE];
    static txreq_t req;
    for (;;) {
        if (!s_link_ok) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        xSemaphoreTake(s_port, portMAX_DELAY);
        if (s_need_setup && setup_locked(NULL, 0) != ESP_OK) {
            xSemaphoreGive(s_port);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        /* incoming reports: block until the next thing falls due, at most 10 ms */
        int wait = 10;
        if (s_ackn || (s_out.active && s_out.acked) || (!s_out.active && uxQueueMessagesWaiting(s_txq))) {
            wait = 0;
        } else if (s_out.active) {                          /* waiting for an ack or a resend slot */
            int64_t t = esp_timer_get_time();
            int64_t due = t < s_out.deadline_us ? s_out.deadline_us : s_out.resend_at_us;
            int64_t ms = (due - t) / 1000;
            wait = ms <= 0 ? 0 : ms < 10 ? (int)ms : 10;
        }
        while (read_line(l, sizeof l, wait)) {
            if (!on_line(l)) ESP_LOGD(TAG, "module: %s", l);
            wait = 0;
        }
        int64_t now = esp_timer_get_time();

        if (s_ackn) {                                       /* acks first: the sender is listening now */
            uint8_t f[HDR];
            make_hdr(f, K_ACK, s_ackq[0].dst, s_ackq[0].seq);
            memmove(&s_ackq[0], &s_ackq[1], (ACKQ - 1) * sizeof s_ackq[0]);
            s_ackn--;
            tx_locked(f, HDR);
        } else if (s_out.active) {
            if (s_out.acked) {
                s_st.back_ok++;
                s_out.active = false;
            } else if (now >= s_out.deadline_us) {
                if (s_out.tries >= TRIES) {
                    s_st.back_noack++;
                    s_st.resend_giveup++;
                    ESP_LOGD(TAG, "no ack from 0x%04X after %d tries", s_out.dst, TRIES);
                    s_out.active = false;
                } else if (now >= s_out.resend_at_us) {
                    s_st.resends++;
                    s_out.tries++;
                    tx_locked(s_out.f, s_out.len);
                    s_out.deadline_us = esp_timer_get_time() + (int64_t)ack_wait_ms() * 1000;
                    s_out.resend_at_us = s_out.deadline_us +
                        (int64_t)(esp_random() % (air_ms(s_out.len) + AT_TX_MS + 1)) * 1000;
                }
            }
        } else if (xQueueReceive(s_txq, &req, 0) == pdTRUE) {
            bool bc = req.dst == 0xFFFF;
            uint8_t seq = ++s_seq;
            make_hdr(s_out.f, bc ? K_BCAST : K_DATA, req.dst, seq);
            memcpy(s_out.f + HDR, req.data, req.len);
            s_out.len = HDR + req.len;
            tx_locked(s_out.f, s_out.len);
            if (!bc) {
                s_out.active = true;
                s_out.acked = false;
                s_out.dst = req.dst;
                s_out.seq = seq;
                s_out.tries = 1;
                s_out.deadline_us = esp_timer_get_time() + (int64_t)ack_wait_ms() * 1000;
                s_out.resend_at_us = s_out.deadline_us +
                    (int64_t)(esp_random() % (air_ms(s_out.len) + AT_TX_MS + 1)) * 1000;
            }
        }
        xSemaphoreGive(s_port);
        /* Sleep a tick with the port free. Without it this task (priority 12)
         * takes s_port back at once and lower-priority callers - app_main in
         * e52_init, the web AT console - wait for it forever. */
        vTaskDelay(1);
    }
}

/* ---------------- e52.h interface ---------------- */

esp_err_t e52_send(uint16_t dst, const uint8_t *data, size_t len)
{
    if (len == 0 || len > E52_MAX_PAYLOAD) return ESP_ERR_INVALID_SIZE;
    if (!s_link_ok) return ESP_ERR_INVALID_STATE;
    txreq_t t;
    t.dst = dst;
    t.len = (uint8_t)len;
    memcpy(t.data, data, len);
    if (xQueueSend(s_txq, &t, pdMS_TO_TICKS(2000)) != pdTRUE) { s_st.tx_fail++; return ESP_ERR_TIMEOUT; }
    return ESP_OK;
}

/* Raw AT command to the module (web console). Its reply lines are returned;
 * the module is put back into receive afterwards. */
esp_err_t e52_at(const char *cmd, char *resp, size_t rl, int timeout_ms)
{
    if (!s_link_ok) { if (resp && rl) resp[0] = 0; return ESP_ERR_INVALID_STATE; }
    static char l[WIO_LINE];
    xSemaphoreTake(s_port, portMAX_DELAY);
    link_write(cmd, strlen(cmd));
    link_write("\r\n", 2);
    size_t off = 0;
    bool any = false, err = false;
    if (resp && rl) resp[0] = 0;
    int64_t end = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    for (;;) {
        int64_t left = (end - esp_timer_get_time()) / 1000;
        if (left <= 0) break;
        /* after the first reply line, stop at a short pause */
        if (!read_line(l, sizeof l, any ? (left < 150 ? (int)left : 150) : (int)left)) break;
        if (on_line(l)) continue;
        any = true;
        if (strstr(l, "ERROR")) err = true;
        if (resp && off + 2 < rl) off += snprintf(resp + off, rl - off, "%s%s", off ? "\n" : "", l);
    }
    if (strncmp(cmd, "AT+TEST=RXLRPKT", 15) != 0 && strncmp(cmd, "AT+RESET", 8) != 0) rx_on_locked();
    if (strncmp(cmd, "AT+RESET", 8) == 0 || strncmp(cmd, "AT+MODE", 7) == 0) s_need_setup = true;
    xSemaphoreGive(s_port);
    if (!any) { s_st.at_err++; return ESP_ERR_TIMEOUT; }
    if (err) { s_st.at_err++; return ESP_FAIL; }
    s_st.at_ok++;
    return ESP_OK;
}

esp_err_t e52_reset(void)
{
    e52_at("AT+RESET", NULL, 0, 1500);
    vTaskDelay(pdMS_TO_TICKS(500));
    s_need_setup = true;
    return ESP_OK;
}

void e52_runtime_setup(void)
{
    if (!s_link_ok) return;
    xSemaphoreTake(s_port, portMAX_DELAY);
    setup_locked(NULL, 0);
    xSemaphoreGive(s_port);
}

/* Nothing is stored in the module: address and network id live in our own
 * settings. Re-send the radio settings (frequency, spreading factor, power). */
esp_err_t e52_apply_config(char *log, size_t ll)
{
    if (!s_link_ok) { if (log && ll) snprintf(log, ll, "Wio-E5 not connected\n"); return ESP_ERR_INVALID_STATE; }
    xSemaphoreTake(s_port, portMAX_DELAY);
    esp_err_t e = setup_locked(log, ll);
    xSemaphoreGive(s_port);
    ESP_LOGI(TAG, "radio settings applied: %s", e == ESP_OK ? "ok" : "failed");
    return e;
}

void e52_get_stats(e52_stats_t *s) { *s = s_st; }

/* The module keeps no address or channel of its own, so there is nothing to
 * mismatch; report whether it answers. */
bool e52_matches_cfg(void)
{
    return e52_at("AT", NULL, 0, 800) == ESP_OK;
}

/* Every peer is direct in a star: hop = the peer itself, score = last SNR. */
int e52_routes(e52_route_t *out, int max, uint32_t max_age_ms)
{
    (void)max_age_ms;
    if (!s_peer_lock) return 0;
    int n = 0;
    uint32_t t = now_ms();
    xSemaphoreTake(s_peer_lock, portMAX_DELAY);
    for (int i = 0; i < PEERS && n < max; i++) {
        const peer_t *p = &s_peers[i];
        if (!p->heard_ms || t - p->heard_ms > PEER_FRESH_MS) continue;
        if (out) out[n] = (e52_route_t){ p->addr, p->addr, p->snr, p->rssi };
        n++;
    }
    xSemaphoreGive(s_peer_lock);
    return n;
}

bool e52_route_to(uint16_t dst, e52_route_t *r, uint32_t max_age_ms)
{
    (void)max_age_ms;
    if (!s_peer_lock) return false;
    bool ok = false;
    xSemaphoreTake(s_peer_lock, portMAX_DELAY);
    peer_t *p = peer(dst, false);
    if (p && now_ms() - p->heard_ms <= PEER_FRESH_MS) {
        if (r) *r = (e52_route_t){ p->addr, p->addr, p->snr, p->rssi };
        ok = true;
    }
    xSemaphoreGive(s_peer_lock);
    return ok;
}

esp_err_t e52_init(e52_rx_cb_t cb)
{
    s_cb = cb;
    s_port = xSemaphoreCreateMutex();
    s_peer_lock = xSemaphoreCreateMutex();
    s_txq = xQueueCreate(8, sizeof(txreq_t));

    /* not fatal: web UI stays up so pins/baud can be fixed */
    esp_err_t err = link_init();
    if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) return err;
    xTaskCreate(radio_task, "wio", 6144, NULL, 12, NULL);
    if (err == ESP_ERR_NOT_FOUND) return err;

    vTaskDelay(pdMS_TO_TICKS(100));
    char log[400];
    xSemaphoreTake(s_port, portMAX_DELAY);
    esp_err_t e = setup_locked(log, sizeof log);
    xSemaphoreGive(s_port);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "Wio-E5 not answering properly - check wiring/baud (UART%d TX=%d RX=%d @%lu)\n%s",
                 UARTN, g_cfg.uart_tx, g_cfg.uart_rx, (unsigned long)g_cfg.uart_baud, log);
        return e;
    }
    ESP_LOGI(TAG, "Wio-E5 ready: address 0x%04X, network 0x%04X, %.3f MHz SF%d", g_cfg.e52_addr, g_cfg.e52_panid,
             e52_chan_mhz(g_cfg.e52_channel), sf());
    return ESP_OK;
}
