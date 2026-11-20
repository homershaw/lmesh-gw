#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* wifi.c */
void wifi_start(void);
bool wifi_sta_connected(void);
void wifi_status(char *sta_ip, size_t n1, char *ap_ip, size_t n2, int *rssi);

/* mqtt_gw.c */
void mqtt_gw_start(void);
void mqtt_gw_on_network(bool up);
bool mqtt_gw_connected(void);
const char *mqtt_gw_base(void);
void mqtt_gw_stats(uint32_t *up, uint32_t *down);

/* web.c */
void web_start(void);

/* udp_bridge.c */
void udp_bridge_start(void);
#define UDP_BRIDGE_PORT 5252

#define GW_FW_VERSION "1.0.0"
