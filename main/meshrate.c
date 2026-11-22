/*
 * Change the air rate of the whole mesh from the gateway, with fallback.
 *
 *  1. Every field node gets a "radio" downlink: {"rate":N,"in_s":S}. S is
 *     chosen so that all nodes (relays included) switch at the same moment.
 *  2. The gateway switches too, then pings the nodes at the new rate.
 *  3. A node that hears the gateway at the new rate keeps it. A node that
 *     hears nothing for NODE_TRIAL_S goes back to the old rate by itself.
 *  4. If no node answers, the gateway goes back too, so the mesh re-forms
 *     at the old rate. Nothing is saved to flash until a link is proven.
 */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "gw.h"
#include "config.h"
#include "lmesh.h"
#include "e52.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#define NODE_TRIAL_S   180      /* must match the node firmware */
#define CONFIRM_S      150      /* gateway gives up before the nodes do */
#define MIN_NODE_FW    "0.3.0"  /* first node firmware that understands "radio" */

static const char *TAG = "meshrate";
static char s_log[1400];
static volatile int s_state;    /* 0 idle, 1 running, 2 done ok, 3 reverted or aborted */
static int s_target;
static bool s_force;

static uint32_t up_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

static void say(const char *fmt, ...)
{
    size_t l = strlen(s_log);
    if (l > sizeof s_log - 100) return;
    l += snprintf(s_log + l, sizeof s_log - l, "[%lus] ", (unsigned long)up_s());
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_log + l, sizeof s_log - l, fmt, ap);
    va_end(ap);
    ESP_LOGI(TAG, "%s", s_log + l);
    strlcat(s_log, "\n", sizeof s_log);
}

static int fw_cmp(const char *a, const char *b)
{
    int x[3] = { 0 }, y[3] = { 0 };
    sscanf(a, "%d.%d.%d", &x[0], &x[1], &x[2]);
    sscanf(b, "%d.%d.%d", &y[0], &y[1], &y[2]);
    for (int i = 0; i < 3; i++) if (x[i] != y[i]) return x[i] - y[i];
    return 0;
}

static void finish(int state)
{
    s_state = state;
    vTaskDelete(NULL);
}

static void task(void *arg)
{
    static lmesh_node_t all[LMESH_MAX_NODES];
    static uint16_t node[LMESH_MAX_NODES];
    static bool back[LMESH_MAX_NODES];
    int n = 0, skipped = 0, old = g_cfg.e52_rate, rate = s_target;
    int cnt = lmesh_nodes(all, LMESH_MAX_NODES);
    uint32_t now = up_s();

    for (int i = 0; i < cnt; i++) {
        if (strcmp(all[i].kind, "rtu") != 0 || now - all[i].last_seen_s > 900) continue;
        if (fw_cmp(all[i].fw, MIN_NODE_FW) < 0) {
            say("node %04X has firmware %s, needs %s or newer", all[i].addr, all[i].fw, MIN_NODE_FW);
            skipped++;
            continue;
        }
        node[n++] = all[i].addr;
    }
    if (skipped && !s_force) {
        say("ABORTED, nothing changed: update those nodes first, or tick the box to leave them behind");
        finish(3);
    }
    say("rate %d -> %d, %d node(s) to tell", old, rate, n);

    /* everyone switches at t_sw; leave time for 3 tries per node at the old rate */
    uint32_t t_sw = now + 15 + 12 * n;
    int acked = 0;
    for (int i = 0; i < n; i++) {
        uint8_t buf[64];
        int in_s = (int)(t_sw - up_s());
        if (in_s < 5) in_s = 5;
        int l = snprintf((char *)buf, sizeof buf, "radio%c{\"rate\":%d,\"in_s\":%d}", 0, rate, in_s);
        back[i] = false;
        if (gw_downlink(node[i], buf, l)) { acked++; say("node %04X acknowledged, switches in %d s", node[i], in_s); }
        else { say("node %04X did NOT acknowledge", node[i]); node[i] = 0; }
    }
    if (acked < n && !s_force) {
        for (int i = 0; i < n; i++) if (node[i]) {
            uint8_t buf[40];
            int l = snprintf((char *)buf, sizeof buf, "radio%c{\"cancel\":true}", 0);
            if (!gw_downlink(node[i], buf, l)) say("node %04X did not acknowledge the cancel; it will try the new rate and come back within %d s", node[i], NODE_TRIAL_S + 60);
        }
        say("ABORTED: not every node answered (tick the box to leave the silent ones behind)");
        finish(3);
    }

    while (up_s() < t_sw + 1) vTaskDelay(pdMS_TO_TICKS(500));
    g_cfg.e52_rate = rate;
    say("gateway switching to rate %d", rate);
    e52_apply_config(NULL, 0);

    int ok = 0;
    uint32_t t_end = up_s() + CONFIRM_S;
    vTaskDelay(pdMS_TO_TICKS(4000));                     /* nodes are resetting their modules too */
    while (ok < acked && up_s() < t_end) {
        lmesh_announce(0xFFFF);
        vTaskDelay(pdMS_TO_TICKS(1500));
        for (int i = 0; i < n; i++) {
            if (!node[i] || back[i]) continue;
            int32_t rtt = lmesh_ping(node[i], rate == 2 ? 9000 : 5000);
            if (rtt >= 0) { back[i] = true; ok++; say("node %04X is back at rate %d (%ld ms)", node[i], rate, (long)rtt); }
        }
        if (ok < acked) vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (acked == 0 || ok > 0) {
        bool saved = cfg_save();
        for (int i = 0; i < n; i++) if (node[i] && !back[i])
            say("node %04X did not come back; it returns to rate %d by itself and is then out of reach", node[i], old);
        say("DONE: mesh is at rate %d, %d of %d node(s) confirmed%s", rate, ok, acked, saved ? "" : " (SAVING THE SETTING FAILED)");
        finish(2);
    }
    g_cfg.e52_rate = old;
    e52_apply_config(NULL, 0);
    say("REVERTED: no node answered at rate %d. Gateway is back at rate %d; nodes follow within %d s", rate, old, NODE_TRIAL_S);
    finish(3);
}

/* returns NULL when started, else the reason it was not */
const char *meshrate_start(int rate, bool force)
{
    if (rate < 0 || rate > 2) return "rate must be 0, 1 or 2";
    if (s_state == 1) return "a rate change is already running";
    if (rate == g_cfg.e52_rate) return "the mesh is already at that rate";
    s_log[0] = 0;
    s_target = rate;
    s_force = force;
    s_state = 1;
    if (xTaskCreate(task, "meshrate", 6144, NULL, 4, NULL) != pdPASS) { s_state = 0; return "no memory"; }
    return NULL;
}

int meshrate_status(const char **log) { *log = s_log; return s_state; }
