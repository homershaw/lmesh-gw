#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool   fix;            /* position below is current (GGA quality > 0, under 5 s old) */
    double lat, lon;       /* degrees, south and west negative */
    float  alt;            /* m above mean sea level */
    float  hdop;
    float  spd_kmh;
    uint8_t sats;
    char   utc[21];        /* "2026-11-22T17:03:10Z", empty until the first RMC with a date */
    uint32_t sentences;    /* valid sentences since boot */
} gps_fix_t;

esp_err_t gps_start(void);
bool gps_get(gps_fix_t *out);   /* false = receiver silent */
