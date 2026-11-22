/*
 * GPS receiver for the survey build (config "gps": true).
 *
 * A survey unit is a C6 + E52 + GPS with no RTU, so the GPS module's TX is
 * wired to the RS-485 receive pin (XIAO D3) and takes over that UART at
 * 9600 baud. Any NMEA receiver works; tested target is the u-blox NEO-6M.
 * GGA (position, altitude, satellites, HDOP), RMC (date, speed) and GSV
 * (satellites in view and signal strength, for fault finding) are read. Sentences with a wrong checksum are dropped.
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "gps.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "gps";
#define UARTN CONFIG_LMN_RS485_UART_NUM

static gps_fix_t s_fix;
static uint32_t s_fix_at_s, s_any_at_s;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t up_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

/* "ddmm.mmmm" / "dddmm.mmmm" -> degrees */
static double nmea_deg(const char *f, const char *hemi)
{
    double v = atof(f);
    int d = (int)(v / 100);
    double deg = d + (v - d * 100) / 60.0;
    return (*hemi == 'S' || *hemi == 'W') ? -deg : deg;
}

static bool checksum_ok(const char *s)
{
    if (*s != '$') return false;
    const char *star = strchr(s, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t x = 0;
    for (const char *p = s + 1; p < star; p++) x ^= (uint8_t)*p;
    return x == (uint8_t)strtol(star + 1, NULL, 16);
}

/* split in place on ',' (keeps empty fields); returns the field count */
static int split(char *s, char **f, int max)
{
    int n = 0;
    f[n++] = s;
    for (char *p = s; *p && n < max; p++) {
        if (*p == ',' || *p == '*') { *p = 0; f[n++] = p + 1; }
    }
    return n;
}

static void sentence(char *s)
{
    if (!checksum_ok(s)) return;
    char *f[24];
    int n = split(s, f, 24);
    if (strlen(f[0]) < 6) return;
    const char *type = f[0] + 3;                 /* skip "$GP" / "$GN" / "$GL" */
    gps_fix_t g;
    portENTER_CRITICAL(&s_mux);
    g = s_fix;
    portEXIT_CRITICAL(&s_mux);
    g.sentences++;
    bool got_fix = false;
    static uint8_t gsv_heard, gsv_snr;           /* running totals over one GSV group */

    if (strcmp(type, "GGA") == 0 && n >= 10) {
        /* GGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,... */
        int q = atoi(f[6]);
        g.sats = (uint8_t)atoi(f[7]);
        if (q > 0 && f[2][0] && f[4][0]) {
            g.lat = nmea_deg(f[2], f[3]);
            g.lon = nmea_deg(f[4], f[5]);
            g.hdop = (float)atof(f[8]);
            g.alt = (float)atof(f[9]);
            g.fix = true;
            got_fix = true;
        } else g.fix = false;
    } else if (strcmp(type, "GSV") == 0 && n >= 5 && s[2] == 'P') {
        /* GSV,messages,this,in view,{prn,elev,azimuth,snr} x up to 4. GPS only ($GP). */
        if (atoi(f[2]) <= 1) gsv_heard = gsv_snr = 0;
        for (int i = 7; i < n - 1; i += 4) {     /* last field is the checksum */
            int snr = atoi(f[i]);
            if (snr > 0) gsv_heard++;
            if (snr > gsv_snr) gsv_snr = (uint8_t)snr;
        }
        if (atoi(f[2]) >= atoi(f[1])) {
            g.view = (uint8_t)atoi(f[3]);
            g.heard = gsv_heard;
            g.snr = gsv_snr;
        }
    } else if (strcmp(type, "RMC") == 0 && n >= 10) {
        /* RMC,time,A/V,lat,N,lon,E,knots,course,ddmmyy,... */
        if (f[2][0] == 'A') g.spd_kmh = (float)(atof(f[7]) * 1.852);
        if (strlen(f[1]) >= 6 && strlen(f[9]) == 6)
            snprintf(g.utc, sizeof g.utc, "20%.2s-%.2s-%.2sT%.2s:%.2s:%.2sZ", f[9] + 4, f[9] + 2, f[9], f[1], f[1] + 2, f[1] + 4);
    }
    uint32_t now = up_s();
    portENTER_CRITICAL(&s_mux);
    s_fix = g;
    s_any_at_s = now;
    if (got_fix) s_fix_at_s = now;
    portEXIT_CRITICAL(&s_mux);
}

static void task(void *arg)
{
    static char line[100];
    size_t len = 0;
    uint8_t buf[64];
    for (;;) {
        int n = uart_read_bytes(UARTN, buf, sizeof buf, pdMS_TO_TICKS(200));
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '$') len = 0;
            if (c == '\r' || c == '\n') {
                if (len > 6) { line[len] = 0; sentence(line); }
                len = 0;
            } else if (len < sizeof line - 1) line[len++] = c;
        }
    }
}

/* false = never heard a valid sentence, or nothing for 5 s (module missing or miswired) */
bool gps_get(gps_fix_t *out)
{
    uint32_t now = up_s();
    portENTER_CRITICAL(&s_mux);
    *out = s_fix;
    uint32_t fix_at = s_fix_at_s, any_at = s_any_at_s;
    portEXIT_CRITICAL(&s_mux);
    bool alive = out->sentences && now - any_at <= 5;
    if (!alive || now - fix_at > 5) out->fix = false;      /* a stale position is not a fix */
    return alive;
}

esp_err_t gps_start(void)
{
    uart_config_t u = {
        .baud_rate = 9600, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t e = uart_driver_install(UARTN, 1024, 0, 0, NULL, 0);
    if (e == ESP_OK) e = uart_param_config(UARTN, &u);
    if (e == ESP_OK) e = uart_set_pin(UARTN, UART_PIN_NO_CHANGE, CONFIG_LMN_RS485_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "GPS on UART%d RX=GPIO%d 9600 8N1: %s", UARTN, CONFIG_LMN_RS485_RX_GPIO, esp_err_to_name(e));
    if (e == ESP_OK) xTaskCreate(task, "gps", 3072, NULL, 3, NULL);
    return e;
}
