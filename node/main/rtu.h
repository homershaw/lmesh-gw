#pragma once
#include <stdint.h>
#include "esp_err.h"

#define RTU_ERR_TIMEOUT  (-1)
#define RTU_ERR_CRC      (-2)
#define RTU_ERR_BAD      (-3)    /* wrong unit/fc, short frame, bad request */

typedef struct { uint32_t reqs, ok, exceptions, timeouts, crc_err, bad; } rtu_stats_t;

esp_err_t rtu_init(void);
/* Send unit + PDU (CRC added). Returns response PDU length (fc + data, an
 * exception response has fc|0x80) copied to rsp, or RTU_ERR_*.
 * timeout_ms <= 0 uses the configured rtu_timeout_ms. Thread-safe. */
int rtu_txn(uint8_t unit, const uint8_t *pdu, int pl, uint8_t *rsp, int rmax, int timeout_ms);
void rtu_get_stats(rtu_stats_t *s);
