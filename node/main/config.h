#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define POLL_MAX 32

/* how raw registers become a value */
enum {
    PT_U16, PT_S16, PT_U32, PT_S32, PT_F32,      /* 32-bit: high word first (big endian) */
    PT_U32_SW, PT_S32_SW, PT_F32_SW,             /* 32-bit: low word first (word swapped) */
    PT_BIT,                                      /* FC1/FC2 coil / discrete input */
    PT_COUNT
};

typedef struct {
    char     name[24];   /* JSON key */
    uint8_t  unit;       /* Modbus slave id */
    uint8_t  fc;         /* 1,2,3,4 */
    uint16_t addr;       /* 0-based register / coil address */
    uint8_t  type;       /* PT_* */
    float    scale;      /* value = raw * scale + offset */
    float    offset;
} poll_item_t;

typedef struct {
    /* identity */
    char site[40];           /* location id (LSD), MQTT topic level */
    char name[24];           /* free text, e.g. "Wellhead RTU" */

    /* Wi-Fi AP + web login */
    char ap_ssid[33];        /* empty = LMESH-NODE-xxxx */
    char ap_pass[65];
    char web_pass[65];       /* empty = ap_pass */
    bool ext_ant;            /* XIAO: use U.FL antenna for Wi-Fi */

    /* optional I2C OLED status display */
    int8_t oled_sda, oled_scl;   /* -1 = display off */
    bool   oled_sh1106;          /* false = SSD1306 (0.96"), true = SH1106 (1.3") */

    /* E52 radio (field names shared with gateway: shared/e52.c uses them) */
    uint8_t  e52_channel;
    uint8_t  e52_rate;
    int8_t   e52_power;
    uint16_t e52_panid;
    uint16_t e52_addr;       /* this node; router range 0x0002..0x7FFF */
    uint16_t e52_csma;
    uint16_t gw_addr;        /* gateway mesh address (learned from its ANNOUNCE too) */
    int8_t   uart_tx, uart_rx, uart_rst;
    uint32_t uart_baud;

    /* RS-485 / Modbus RTU */
    uint32_t rs_baud;
    uint8_t  rs_parity;      /* 0 none, 1 even, 2 odd */
    uint8_t  rs_stop;        /* 1 or 2 */
    uint16_t rtu_timeout_ms;

    /* polling / publish */
    uint16_t poll_s;         /* 0 = polling off */
    uint8_t  poll_n;
    poll_item_t poll[POLL_MAX];
} node_cfg_t;

extern node_cfg_t g_cfg;
extern SemaphoreHandle_t g_cfg_lock;   /* hold while reading poll list / writing cfg */
extern char g_node_id[8];              /* last 4 hex of MAC */

void cfg_defaults(node_cfg_t *c);
void cfg_load(void);
bool cfg_save(void);
char *cfg_to_json(bool include_secrets);   /* caller frees */
bool cfg_from_json(const char *json);      /* merges into g_cfg */
float e52_chan_mhz(uint8_t ch);
const char *pt_name(uint8_t t);
int pt_regs(uint8_t t);                    /* registers (or bits) per item */
