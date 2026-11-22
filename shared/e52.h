#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#define E52_MAX_PAYLOAD 200

typedef struct {
    uint8_t  type;      /* 0xC1 unicast, 0xC2 multicast, 0xC3 broadcast, 0xC4 anycast */
    uint16_t panid;
    uint16_t src;
    uint16_t dst;
    uint8_t  len;
    uint8_t  data[E52_MAX_PAYLOAD];
} e52_frame_t;

typedef void (*e52_rx_cb_t)(const e52_frame_t *f);

typedef struct {
    uint32_t tx_frames, rx_frames, tx_bytes, rx_bytes;
    uint32_t at_ok, at_err, rx_junk, tx_fail;
} e52_stats_t;

esp_err_t e52_init(e52_rx_cb_t cb);
/* Send raw AT command (no CR/LF appended per E52 manual). resp may be NULL.
 * Returns ESP_OK if a reply arrived that does not contain ERR. */
esp_err_t e52_at(const char *cmd, char *resp, size_t resp_len, int timeout_ms);
/* dst = 0xFFFF => broadcast */
esp_err_t e52_send(uint16_t dst, const uint8_t *data, size_t len);
/* Push g_cfg radio settings to module (saved in module flash) and reset it */
esp_err_t e52_apply_config(char *log, size_t log_len);
esp_err_t e52_reset(void);
/* Re-send settings the module forgets on reset (HEAD/BACK/CSMA_RNG). */
void e52_runtime_setup(void);
void e52_get_stats(e52_stats_t *s);
/* module's SRC_ADDR, CHANNEL and PANID match g_cfg (detects a swapped/unprogrammed module) */
bool e52_matches_cfg(void);
uint32_t e52_airtime_ms(size_t len);

/* One entry of the module's route table (AT+ROUTER_READ=?). rssi is the last
 * frame heard from the next hop, in dBm. */
typedef struct { uint16_t dst, hop; int8_t score; int16_t rssi; } e52_route_t;
#define E52_MAX_ROUTES 16
/* Copy the route table; re-read from the module only if the cached copy is
 * older than max_age_ms. Returns the number of entries. */
int  e52_routes(e52_route_t *out, int max, uint32_t max_age_ms);
bool e52_route_to(uint16_t dst, e52_route_t *r, uint32_t max_age_ms);
