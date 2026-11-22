/*
 * Range survey: turns the "pos" messages of a GPS survey node into
 * distance and path-loss points.
 *
 * For every position report the gateway adds what it knows itself (its own
 * RSSI and next hop for that node, its configured position) and works out:
 *
 *   dist      great-circle distance gateway <-> node
 *   loss      TX power + antenna gains - RSSI     (direct links only)
 *   fspl      free-space loss at that distance on the configured channel
 *   excess    loss - fspl: what terrain and obstructions cost
 *
 * RSSI is the node's reading of its next hop. If that hop is the gateway,
 * the link is direct and the loss figure is meaningful. On a relayed route
 * the point is kept (hop, RTT and delivery still matter) but has no loss.
 *
 * The last SURVEY_MAX points are kept in RAM for the web page; each one is
 * also published to MQTT as <base>/<site>/survey.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "gw.h"
#include "config.h"
#include "lmesh.h"
#include "e52.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define SURVEY_MAX 600

typedef struct {
    uint32_t t_s;            /* gateway uptime */
    uint16_t node, hop;      /* hop: the node's next hop towards the gateway */
    uint16_t n;              /* node's running counter: gaps = lost reports */
    double latd, lond;       /* degrees */
    float dist_m;            /* <0 = unknown (no fix or gateway position not set) */
    int16_t rssi_n, rssi_g;  /* at the node / at the gateway; 0 = unknown */
    int16_t rtt_ms;          /* node's ping to the gateway, -1 = lost */
    float loss, fspl;        /* dB; loss <0 = not a direct link */
    uint8_t sats;
    bool fix;
} pt_t;

static pt_t s_pt[SURVEY_MAX];
static int s_head, s_n;
static SemaphoreHandle_t s_lock;

static double rad(double d) { return d * 0.017453292519943295; }

static double haversine_m(double la1, double lo1, double la2, double lo2)
{
    double dla = rad(la2 - la1), dlo = rad(lo2 - lo1);
    double a = sin(dla / 2) * sin(dla / 2) + cos(rad(la1)) * cos(rad(la2)) * sin(dlo / 2) * sin(dlo / 2);
    return 6371000.0 * 2 * atan2(sqrt(a), sqrt(1 - a));
}

static double jd(cJSON *o, const char *k, double def) { cJSON *v = cJSON_GetObjectItem(o, k); return cJSON_IsNumber(v) ? v->valuedouble : def; }

/* Called for a node's "pos" uplink. Writes the enriched JSON to out
 * (for MQTT <site>/survey); returns its length, 0 if the message was unusable. */
int survey_on_pos(uint16_t src, const char *json, int len, char *out, size_t outmax)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    cJSON *o = cJSON_ParseWithLength(json, len);
    if (!o) return 0;
    pt_t p = { 0 };
    p.t_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
    p.node = src;
    p.n = (uint16_t)jd(o, "n", 0);
    p.fix = jd(o, "fix", 0) != 0;
    p.latd = jd(o, "lat", 0);
    p.lond = jd(o, "lon", 0);
    p.sats = (uint8_t)jd(o, "sats", 0);
    p.rssi_n = (int16_t)jd(o, "rssi", 0);
    p.rtt_ms = (int16_t)jd(o, "rtt", -1);
    cJSON *h = cJSON_GetObjectItem(o, "hop");
    p.hop = cJSON_IsString(h) ? (uint16_t)strtol(h->valuestring, NULL, 16) : 0xFFFF;
    char utc[24] = "";
    cJSON *u = cJSON_GetObjectItem(o, "utc");
    if (cJSON_IsString(u)) snprintf(utc, sizeof utc, "%s", u->valuestring);
    cJSON_Delete(o);

    e52_route_t r;
    uint16_t hop_g = 0xFFFF;
    if (e52_route_to(src, &r, 3000)) { p.rssi_g = r.rssi; hop_g = r.hop; }

    bool have_gw = g_cfg.gw_lat != 0 || g_cfg.gw_lon != 0;
    p.dist_m = (p.fix && have_gw) ? (float)haversine_m(g_cfg.gw_lat, g_cfg.gw_lon, p.latd, p.lond) : -1;
    p.loss = -1;
    p.fspl = -1;
    if (p.dist_m >= 1) {
        double mhz = e52_chan_mhz(g_cfg.e52_channel);
        p.fspl = (float)(32.44 + 20 * log10(mhz) + 20 * log10(p.dist_m / 1000.0));
    }
    bool direct = p.hop == g_cfg.e52_addr && p.rssi_n < 0;
    if (direct) p.loss = (float)(g_cfg.e52_power + g_cfg.ant_gain_db - p.rssi_n);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_pt[(s_head + s_n) % SURVEY_MAX] = p;
    if (s_n < SURVEY_MAX) s_n++; else s_head = (s_head + 1) % SURVEY_MAX;
    xSemaphoreGive(s_lock);

    int l = snprintf(out, outmax, "{\"n\":%u,\"fix\":%d,\"lat\":%.6f,\"lon\":%.6f,\"sats\":%u,\"utc\":\"%s\","
                     "\"rssi_node\":%d,\"rssi_gw\":%d,\"hop\":\"%04X\",\"hop_gw\":\"%04X\",\"direct\":%s,\"rtt\":%d",
                     p.n, p.fix, p.latd, p.lond, p.sats, utc, p.rssi_n, p.rssi_g, p.hop, hop_g, direct ? "true" : "false", p.rtt_ms);
    if (p.dist_m >= 0 && l < (int)outmax) l += snprintf(out + l, outmax - l, ",\"dist_m\":%.0f", p.dist_m);
    if (p.loss >= 0 && l < (int)outmax) l += snprintf(out + l, outmax - l, ",\"loss_db\":%.1f", p.loss);
    if (p.fspl >= 0 && l < (int)outmax) l += snprintf(out + l, outmax - l, ",\"fspl_db\":%.1f", p.fspl);
    if (p.loss >= 0 && p.fspl >= 0 && l < (int)outmax) l += snprintf(out + l, outmax - l, ",\"excess_db\":%.1f", p.loss - p.fspl);
    if (l < (int)outmax) l += snprintf(out + l, outmax - l, "}");
    return l < (int)outmax ? l : 0;
}

void survey_clear(void)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_head = s_n = 0;
    xSemaphoreGive(s_lock);
}

/* Points as rows for the web page and the CSV download; caller frees.
 * csv=false: JSON {"gw":[lat,lon],"cols":[...],"rows":[[...],...]} */
char *survey_dump(bool csv)
{
    static const char *cols = "t,node,n,fix,lat,lon,sats,dist_m,rssi_node,rssi_gw,hop,rtt_ms,loss_db,fspl_db";
    size_t cap = 200 + (size_t)s_n * 110, l = 0;
    char *b = malloc(cap);
    if (!b) return NULL;
    if (csv) l += snprintf(b, cap, "%s\n", cols);
    else l += snprintf(b, cap, "{\"gw\":[%.6f,%.6f],\"cols\":\"%s\",\"rows\":[", g_cfg.gw_lat, g_cfg.gw_lon, cols);
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_n && l + 120 < cap; i++) {
        const pt_t *p = &s_pt[(s_head + i) % SURVEY_MAX];
        l += snprintf(b + l, cap - l, csv ? "%s%lu,%04X,%u,%d,%.6f,%.6f,%u,%.0f,%d,%d,%04X,%d,%.1f,%.1f\n"
                                          : "%s[%lu,\"%04X\",%u,%d,%.6f,%.6f,%u,%.0f,%d,%d,\"%04X\",%d,%.1f,%.1f]",
                      csv ? "" : (i ? "," : ""), (unsigned long)p->t_s, p->node, p->n, p->fix, p->latd, p->lond, p->sats,
                      p->dist_m, p->rssi_n, p->rssi_g, p->hop, p->rtt_ms, p->loss, p->fspl);
    }
    if (s_lock) xSemaphoreGive(s_lock);
    if (!csv) l += snprintf(b + l, cap - l, "]}");
    return b;
}
