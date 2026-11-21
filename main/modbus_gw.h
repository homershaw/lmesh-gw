#pragma once
#include <stdint.h>
#include <stddef.h>

#define MB_GW_UNIT_LOCAL 255     /* gateway diagnostics registers */

/* Result codes (negative) */
#define MB_ERR_TIMEOUT   -1
#define MB_ERR_SEND      -2
#define MB_ERR_BUSY      -3
#define MB_ERR_PARAM     -4

void modbus_gw_start(void);
/* Map TCP unit id -> mesh node + RS-485 unit. Returns false if unmapped & no default */
void modbus_gw_resolve(uint8_t unit, uint16_t *node, uint8_t *dst_unit);
/* One Modbus transaction over the mesh. pdu = function code + data.
 * Returns response PDU length (>0) or MB_ERR_*. Thread-safe. */
int  modbus_gw_txn(uint16_t node, uint8_t dst_unit, const uint8_t *pdu, size_t plen,
                   uint8_t *rsp, size_t rmax, int timeout_ms);
void modbus_gw_stats(uint32_t *reqs, uint32_t *timeouts, uint32_t *clients, uint32_t *retries);
