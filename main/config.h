#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MB_MAP_MAX 32

typedef struct {
    uint8_t  unit;       /* Modbus unit id on TCP side */
    uint16_t node;       /* E52 mesh address */
    uint8_t  dst_unit;   /* unit id to use on the node's RS-485 (0 = same) */
} mb_map_t;

typedef struct {
    /* Wi-Fi */
    char sta_ssid[33];
    char sta_pass[65];
    char ap_ssid[33];
    char ap_pass[65];
    char hostname[32];
    char web_pass[65];       /* web UI login (user "admin"); empty = use ap_pass */

    /* MQTT */
    bool mqtt_en;
    char mqtt_uri[128];      /* mqtt://192.168.1.10:1883 */
    char mqtt_user[64];
    char mqtt_pass[64];
    char mqtt_base[64];      /* e.g. lmesh/<id> ; empty = lmesh/<last4 MAC> */

    /* E52 radio */
    uint8_t  e52_channel;    /* 52..77 => 902.125..927.125 MHz */
    uint8_t  e52_rate;       /* 0=62.5k 1=21.875k 2=7k */
    int8_t   e52_power;      /* -9..22 dBm */
    uint16_t e52_panid;
    uint16_t e52_addr;       /* gateway mesh address (router range 0x0000..0x7FFF) */
    uint16_t e52_csma;       /* ms */

    /* UART pins (reboot to apply) */
    int8_t uart_tx, uart_rx, uart_rst;
    uint32_t uart_baud;

    /* Modbus */
    uint16_t mb_port;
    uint16_t mb_timeout_ms;

    /* range survey */
    double gw_lat, gw_lon;   /* gateway position, degrees; 0,0 = not set */
    float  ant_gain_db;      /* gain of both antennas together, for the path-loss figure */
    uint8_t  mb_map_n;
    mb_map_t mb_map[MB_MAP_MAX];
} gw_cfg_t;

extern gw_cfg_t g_cfg;
extern char g_gw_id[8];      /* last 4 hex of STA MAC */

void cfg_defaults(gw_cfg_t *c);
void cfg_load(void);
bool cfg_save(void);
char *cfg_to_json(bool include_secrets);       /* caller frees */
bool cfg_from_json(const char *json);          /* merges into g_cfg */
float e52_chan_mhz(uint8_t ch);
