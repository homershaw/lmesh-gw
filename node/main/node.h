#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NODE_FW_VERSION "0.3.0"

typedef struct {
    uint32_t mb_reqs, mb_dups, mb_rtu_err;       /* mesh Modbus pass-through */
    uint32_t polls, pubs, pub_fail, writes;
    uint32_t last_pub_s;                         /* uptime of last publish, 0 = never */
    uint32_t gw_seen_s;                          /* uptime of last gateway ANNOUNCE */
    uint16_t gw_addr;
} node_stats_t;

void node_start(void);              /* after lmesh_init, before lmesh_start */
void node_after_radio_up(void);     /* announce + start poller */
void node_poll_now(void);
void node_identity_changed(void);   /* site/name edited: re-announce */
void node_get_stats(node_stats_t *s);
int  node_value_count(void);
void node_value_text(int i, char *out, size_t n);   /* "name value" for the display */
char *node_values_json(void);       /* latest poll values, caller frees */
