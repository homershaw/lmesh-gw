#pragma once
/*
 * LMESH: compressed UDP/IPv4 over the E52 LoRa mesh.
 *
 * IPv4 addressing is implicit: node with E52 address A has IP 10.52.(A>>8).(A&0xFF).
 * The E52 frame header (HEAD=1) already carries src/dst addresses, so IP
 * addresses are never sent over the air. Each E52 payload (<=200 B) is:
 *
 *   b0      0x10 | type      (high nibble = protocol version 1)
 *   b1      seq              (request/response matching, fragment grouping)
 *   b2      frag             (index << 4) | total   ; 0x01 = unfragmented
 *   b3..b4  source port      (big endian)
 *   b5..b6  dest port        (big endian)
 *   b7..    payload segment  (<= 193 B)
 *
 * Datagrams up to 4 fragments (LMESH_MAX_DGRAM bytes) are reassembled.
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#define LMESH_VER          0x10
#define LMESH_T_UDP        0x01
#define LMESH_T_ANNOUNCE   0x02   /* payload: "name\0kind\0fw\0" */
#define LMESH_T_PING       0x03
#define LMESH_T_PONG       0x04

#define LMESH_HDR          7
#define LMESH_SEG          (200 - LMESH_HDR)     /* 193 */
#define LMESH_MAX_FRAGS    4
#define LMESH_MAX_DGRAM    (LMESH_SEG * LMESH_MAX_FRAGS)
#define LMESH_BCAST        0xFFFF

/* well-known ports */
#define LMESH_PORT_MODBUS  502    /* payload: unit + Modbus PDU (request & response) */
#define LMESH_PORT_MQTT_UP 1883   /* node -> gw: "subtopic\0payload"  */
#define LMESH_PORT_MQTT_DN 1884   /* gw -> node: "subtopic\0payload"  */

#define LMESH_MAX_NODES    64

typedef struct {
    uint16_t src;
    uint16_t sport, dport;
    uint8_t  seq;
    uint16_t len;
    const uint8_t *data;
} lmesh_dgram_t;

typedef struct {
    uint16_t addr;
    uint32_t first_seen_s, last_seen_s;
    uint32_t rx_pkts;
    int32_t  rtt_ms;          /* last ping, -1 = none */
    char name[24];
    char kind[16];
    char fw[16];
} lmesh_node_t;

typedef void (*lmesh_handler_t)(const lmesh_dgram_t *d);

void      lmesh_init(void);
esp_err_t lmesh_start(void);                               /* bring up E52, announce */
esp_err_t lmesh_bind(uint16_t port, lmesh_handler_t h);
/* default handler for datagrams to unbound ports (UDP bridge) */
void      lmesh_bind_default(lmesh_handler_t h);
uint8_t   lmesh_next_seq(void);
esp_err_t lmesh_send(uint16_t node, uint8_t type, uint8_t seq,
                     uint16_t sport, uint16_t dport, const uint8_t *data, size_t len);
static inline esp_err_t lmesh_send_udp(uint16_t node, uint8_t seq, uint16_t sport, uint16_t dport,
                                       const uint8_t *d, size_t len)
{ return lmesh_send(node, LMESH_T_UDP, seq, sport, dport, d, len); }

int32_t   lmesh_ping(uint16_t node, int timeout_ms);       /* rtt ms or -1 */
esp_err_t lmesh_discover(void);                            /* broadcast ping */

int       lmesh_nodes(lmesh_node_t *out, int max);
int       lmesh_node_count(void);
void      lmesh_ip_str(uint16_t addr, char *buf, size_t n);
bool      lmesh_ip_to_addr(uint32_t ip_host_order, uint16_t *addr);
uint32_t  lmesh_addr_to_ip(uint16_t addr);                 /* host order */
