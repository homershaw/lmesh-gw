/*
 * LMESH field-node agent.
 *
 *  mesh :502  [unit][PDU]  -> Modbus RTU on RS-485 -> reply [unit][PDU], same seq
 *  poller     every poll_s: read the poll list, publish JSON to the gateway
 *             (LMESH :1883 "data\0{json}" -> MQTT <base>/<site>/data)
 *  mesh :1884 downlink "write\0{json}" / "poll\0"  (MQTT <base>/<site>/set/...)
 *  ANNOUNCE   name, "rtu", fw, site -> gateway on boot, when the gateway
 *             announces itself, and every 10 min
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "node.h"
#include "config.h"
#include "rtu.h"
#include "lmesh.h"
#include "e52.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "node";

#define REANNOUNCE_S   600
#define STATUS_S       600
#define PUB_CHUNK      560          /* split data JSON so one message stays <= 3 radio frames */
#define NOTE_POLL      (1u << 0)
#define NOTE_ANNOUNCE  (1u << 1)
#define NOTE_WAKE      (1u << 2)

/* Trial change of the air rate, ordered by the gateway ("radio" downlink).
 * The new rate is kept only if the gateway is heard on it within TRIAL_S;
 * otherwise the node goes back, so a failed change cannot strand it.
 * The config is saved only after that proof. If the node resets mid-trial,
 * boot sees module rate != config and reprograms the old rate. */
#define TRIAL_S        180      /* must match the gateway (meshrate.c) */
static struct {
    volatile uint8_t state;     /* 0 none, 1 scheduled, 2 on trial */
    uint8_t new_rate, old_rate;
    uint32_t at_s, switched_s, deadline_s, next_ann_s;
} s_trial;

static node_stats_t s_st;
static TaskHandle_t s_poll_task;
static QueueHandle_t s_workq;
static volatile uint16_t s_gw;

typedef struct {
    uint8_t  kind;                  /* 0 = Modbus request, 1 = downlink */
    uint16_t src;
    uint8_t  seq;
    uint16_t sport;
    uint16_t len;
    uint8_t  data[LMESH_SEG * 2];
} work_t;

/* duplicate suppression for the gateway's retransmits (same src + seq) */
static struct {
    volatile bool busy;
    uint16_t src; uint8_t seq;
    int64_t done_us;
    uint16_t rlen;
    uint8_t rsp[260];
} s_last;

/* latest values for the web page */
typedef struct { bool ok; double v; uint32_t at_s; } val_t;
static val_t s_vals[POLL_MAX];

static uint32_t uptime_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

static void set_identity(void)
{
    xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
    lmesh_set_identity(g_cfg.name, "rtu", NODE_FW_VERSION, g_cfg.site, s_gw);
    xSemaphoreGive(g_cfg_lock);
}

/* ---------------- uplink ---------------- */

static bool publish(const char *sub, const char *payload)
{
    size_t sl = strlen(sub), pl = strlen(payload);
    if (sl + 1 + pl > LMESH_MAX_DGRAM) { ESP_LOGW(TAG, "publish %s: too large (%u)", sub, (unsigned)(sl + 1 + pl)); return false; }
    uint8_t *b = malloc(sl + 1 + pl);
    if (!b) return false;
    memcpy(b, sub, sl + 1);
    memcpy(b + sl + 1, payload, pl);
    esp_err_t e = lmesh_send_udp(s_gw, lmesh_next_seq(), LMESH_PORT_MQTT_UP, LMESH_PORT_MQTT_UP, b, sl + 1 + pl);
    free(b);
    if (e == ESP_OK) { s_st.pubs++; s_st.last_pub_s = uptime_s(); }
    else s_st.pub_fail++;
    return e == ESP_OK;
}

static void publish_status(void)
{
    e52_stats_t es; e52_get_stats(&es);
    rtu_stats_t rs; rtu_get_stats(&rs);
    char b[400], link[40] = "";
    e52_route_t rt;                      /* our route to the gateway: next hop and its RSSI */
    if (e52_route_to(s_gw, &rt, 5000))
        snprintf(link, sizeof link, ",\"rssi\":%d,\"hop\":\"%04X\"", rt.rssi, rt.hop);
    xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
    snprintf(b, sizeof b,
             "{\"site\":\"%s\",\"name\":\"%s\",\"fw\":\"%s\",\"up\":%lu,\"addr\":\"%04X\","
             "\"rtu\":{\"ok\":%lu,\"exc\":%lu,\"timeout\":%lu,\"crc\":%lu},"
             "\"radio\":{\"tx\":%lu,\"rx\":%lu,\"tx_fail\":%lu},\"pubs\":%lu%s}",
             g_cfg.site, g_cfg.name, NODE_FW_VERSION, (unsigned long)uptime_s(), g_cfg.e52_addr,
             (unsigned long)rs.ok, (unsigned long)rs.exceptions, (unsigned long)rs.timeouts, (unsigned long)rs.crc_err,
             (unsigned long)es.tx_frames, (unsigned long)es.rx_frames, (unsigned long)es.tx_fail, (unsigned long)s_st.pubs, link);
    xSemaphoreGive(g_cfg_lock);
    publish("status", b);
}

/* ---------------- polling ---------------- */

static bool decode(const poll_item_t *it, const uint8_t *r, int n, double *out)
{
    if (n < 3 || (r[0] & 0x80)) return false;
    if (it->fc <= 2) { *out = r[2] & 1; return true; }
    int regs = pt_regs(it->type);
    if (r[1] < regs * 2 || n < 2 + regs * 2) return false;
    uint16_t w0 = (r[2] << 8) | r[3], w1 = regs > 1 ? (r[4] << 8) | r[5] : 0;
    uint32_t u = 0;
    switch (it->type) {
    case PT_U16: *out = w0; break;
    case PT_S16: *out = (int16_t)w0; break;
    case PT_U32: case PT_S32: case PT_F32: u = ((uint32_t)w0 << 16) | w1; break;
    default: u = ((uint32_t)w1 << 16) | w0; break;            /* word swapped */
    }
    if (it->type == PT_U32 || it->type == PT_U32_SW) *out = u;
    else if (it->type == PT_S32 || it->type == PT_S32_SW) *out = (int32_t)u;
    else if (it->type == PT_F32 || it->type == PT_F32_SW) { float f; memcpy(&f, &u, 4); if (!isfinite(f)) return false; *out = f; }
    return true;
}

static void fmt_num(char *b, size_t n, double v)
{
    if (v == (double)(int64_t)v && fabs(v) < 1e15) snprintf(b, n, "%lld", (long long)v);
    else snprintf(b, n, "%.7g", v);
}

static void poll_once(void)
{
    static poll_item_t items[POLL_MAX];
    char site[40];
    xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
    int n = g_cfg.poll_n;
    memcpy(items, g_cfg.poll, n * sizeof items[0]);
    strcpy(site, g_cfg.site[0] ? g_cfg.site : g_node_id);
    xSemaphoreGive(g_cfg_lock);
    if (!n) return;
    s_st.polls++;

    char *b = malloc(PUB_CHUNK + 96);
    if (!b) return;
    uint32_t now = uptime_s();
    int head = snprintf(b, PUB_CHUNK, "{\"site\":\"%s\",\"t\":%lu", site, (unsigned long)now);
    int o = head, errs = 0;
    for (int i = 0; i < n; i++) {
        const poll_item_t *it = &items[i];
        uint8_t pdu[5] = { it->fc, it->addr >> 8, it->addr & 0xFF, 0, (uint8_t)pt_regs(it->type) };
        uint8_t r[16];
        int rl = rtu_txn(it->unit, pdu, 5, r, sizeof r, 0);
        double v = 0;
        bool ok = rl > 0 && decode(it, r, rl, &v);
        if (ok) v = v * it->scale + it->offset;
        else errs++;
        s_vals[i] = (val_t){ ok, v, now };

        char num[32];
        if (ok) fmt_num(num, sizeof num, v); else strcpy(num, "null");
        char kv[80];
        int kl = snprintf(kv, sizeof kv, ",\"%s\":%s", it->name, num);
        if (o + kl + 2 > PUB_CHUNK) {              /* flush a chunk, start a new one */
            strcpy(b + o, "}");
            publish("data", b);
            o = head;
        }
        memcpy(b + o, kv, kl + 1);
        o += kl;
    }
    if (errs) o += snprintf(b + o, 32, ",\"_err\":%d", errs);
    strcpy(b + o, "}");
    publish("data", b);
    free(b);
}

static void trial_step(uint32_t now)
{
    if (s_trial.state == 1 && now >= s_trial.at_s) {
        s_trial.old_rate = g_cfg.e52_rate;
        g_cfg.e52_rate = s_trial.new_rate;
        ESP_LOGW(TAG, "air rate %u -> %u on trial", s_trial.old_rate, s_trial.new_rate);
        e52_apply_config(NULL, 0);
        s_trial.switched_s = uptime_s();
        s_trial.deadline_s = s_trial.switched_s + TRIAL_S;
        s_trial.next_ann_s = s_trial.switched_s + 3;
        s_trial.state = 2;
    } else if (s_trial.state == 2) {
        if (lmesh_last_seen(s_gw) > s_trial.switched_s) {
            s_trial.state = 0;
            bool saved = cfg_save();
            ESP_LOGW(TAG, "gateway heard at rate %u: keeping it%s", g_cfg.e52_rate, saved ? "" : " (SAVE FAILED)");
            publish_status();
        } else if (now >= s_trial.deadline_s) {
            ESP_LOGW(TAG, "gateway not heard at rate %u: back to rate %u", g_cfg.e52_rate, s_trial.old_rate);
            g_cfg.e52_rate = s_trial.old_rate;
            e52_apply_config(NULL, 0);
            s_trial.state = 0;
            lmesh_announce(s_gw);
        } else if (now >= s_trial.next_ann_s) {
            lmesh_announce(s_gw);                       /* lets the gateway find us sooner */
            s_trial.next_ann_s = now + 20;
        }
    }
}

static void poll_task(void *a)
{
    uint32_t next_poll = uptime_s() + 5, next_ann = uptime_s() + REANNOUNCE_S, next_status = uptime_s() + 20;
    for (;;) {
        uint32_t now = uptime_s(), wake = next_ann < next_status ? next_ann : next_status;
        uint16_t ps = g_cfg.poll_s;
        if (ps && next_poll < wake) wake = next_poll;
        if (s_trial.state && wake > now + 2) wake = now + 2;
        uint32_t note = 0;
        xTaskNotifyWait(0, UINT32_MAX, &note, pdMS_TO_TICKS(wake > now ? (wake - now) * 1000 : 0));
        now = uptime_s();
        trial_step(now);
        now = uptime_s();
        if (note & NOTE_ANNOUNCE) {
            vTaskDelay(pdMS_TO_TICKS(200 + esp_random() % 2000));   /* don't all answer at once */
            set_identity();
            lmesh_announce(s_gw);
            next_ann = now + REANNOUNCE_S;
        }
        if (now >= next_ann) { set_identity(); lmesh_announce(s_gw); next_ann = now + REANNOUNCE_S; }
        if (now >= next_status) { publish_status(); next_status = now + STATUS_S; }
        if ((note & NOTE_POLL) || (ps && now >= next_poll)) {
            poll_once();
            next_poll = uptime_s() + (ps ? ps : 3600);
        }
    }
}

void node_poll_now(void) { if (s_poll_task) xTaskNotify(s_poll_task, NOTE_POLL, eSetBits); }
void node_identity_changed(void) { if (s_poll_task) xTaskNotify(s_poll_task, NOTE_ANNOUNCE, eSetBits); }

/* ---------------- mesh Modbus + downlink (worker) ---------------- */

static void do_modbus(const work_t *w)
{
    if (w->len < 2) return;
    uint8_t out[260];
    out[0] = w->data[0];
    int rl = rtu_txn(w->data[0], w->data + 1, w->len - 1, out + 1, sizeof out - 1, 0);
    if (rl < 0) {                                   /* no/garbled reply: Modbus "target failed to respond" */
        out[1] = w->data[1] | 0x80;
        out[2] = 0x0B;
        rl = 2;
        s_st.mb_rtu_err++;
    }
    s_last.rlen = rl + 1;
    memcpy(s_last.rsp, out, rl + 1);
    s_last.done_us = esp_timer_get_time();
    lmesh_send_udp(w->src, w->seq, LMESH_PORT_MODBUS, w->sport, out, rl + 1);
}

static void write_ack(bool ok, const char *err)
{
    char b[96];
    snprintf(b, sizeof b, "{\"ok\":%s%s%s%s}", ok ? "true" : "false", err ? ",\"err\":\"" : "", err ? err : "", err ? "\"" : "");
    publish("write/ack", b);
}

/* "write" {"name":"sp","value":12.5} or {"unit":1,"addr":100,"value":5[,"type":"f32"]} */
static void do_write(const char *json)
{
    cJSON *o = cJSON_Parse(json);
    if (!o) { write_ack(false, "bad json"); return; }
    cJSON *jv = cJSON_GetObjectItem(o, "value"), *jn = cJSON_GetObjectItem(o, "name");
    if (!cJSON_IsNumber(jv)) { cJSON_Delete(o); write_ack(false, "no value"); return; }
    poll_item_t it = { .unit = 1, .fc = 3, .type = PT_U16, .scale = 1 };
    bool found = !cJSON_IsString(jn);
    if (!found) {
        xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
        for (int i = 0; i < g_cfg.poll_n; i++)
            if (strcmp(g_cfg.poll[i].name, jn->valuestring) == 0) { it = g_cfg.poll[i]; found = true; break; }
        xSemaphoreGive(g_cfg_lock);
    } else {
        cJSON *u = cJSON_GetObjectItem(o, "unit"), *a = cJSON_GetObjectItem(o, "addr"), *t = cJSON_GetObjectItem(o, "type");
        if (!cJSON_IsNumber(a)) { cJSON_Delete(o); write_ack(false, "need name or addr"); return; }
        if (cJSON_IsNumber(u)) it.unit = u->valueint;
        it.addr = a->valueint;
        if (cJSON_IsString(t)) for (int k = 0; k < PT_COUNT; k++) if (!strcmp(t->valuestring, pt_name(k))) it.type = k;
    }
    double v = jv->valuedouble;
    cJSON_Delete(o);
    if (!found) { write_ack(false, "unknown name"); return; }
    if (it.fc == 2 || it.fc == 4) { write_ack(false, "read-only point"); return; }

    uint8_t pdu[12], r[8];
    int pl;
    if (it.fc == 1) {                                  /* coil: FC5 */
        pdu[0] = 5; pdu[1] = it.addr >> 8; pdu[2] = it.addr; pdu[3] = v != 0 ? 0xFF : 0; pdu[4] = 0; pl = 5;
    } else {
        double raw = (v - it.offset) / (it.scale ? it.scale : 1);
        uint32_t u;
        if (it.type == PT_F32 || it.type == PT_F32_SW) { float f = raw; memcpy(&u, &f, 4); }
        else u = (uint32_t)(int32_t)lround(raw);
        if (pt_regs(it.type) == 1) {                   /* FC6 */
            pdu[0] = 6; pdu[1] = it.addr >> 8; pdu[2] = it.addr; pdu[3] = u >> 8; pdu[4] = u; pl = 5;
        } else {                                       /* FC16, 2 registers */
            uint16_t hi = u >> 16, lo = u & 0xFFFF;
            if (it.type >= PT_U32_SW) { uint16_t t = hi; hi = lo; lo = t; }
            pdu[0] = 16; pdu[1] = it.addr >> 8; pdu[2] = it.addr; pdu[3] = 0; pdu[4] = 2; pdu[5] = 4;
            pdu[6] = hi >> 8; pdu[7] = hi; pdu[8] = lo >> 8; pdu[9] = lo; pl = 10;
        }
    }
    int rl = rtu_txn(it.unit, pdu, pl, r, sizeof r, 0);
    s_st.writes++;
    if (rl < 0) write_ack(false, rl == RTU_ERR_TIMEOUT ? "rtu timeout" : "rtu error");
    else if (r[0] & 0x80) { char e[24]; snprintf(e, sizeof e, "exception %u", rl > 1 ? r[1] : 0); write_ack(false, e); }
    else { write_ack(true, NULL); node_poll_now(); }
}

/* "config" {"site":"..","name":"..","poll_s":60,"poll":[...]} from the gateway
 * page or MQTT set/config. Radio, RS-485 and Wi-Fi settings stay local-only. */
static void do_config(const char *json)
{
    static const char *allowed[] = { "site", "name", "poll_s", "poll" };
    cJSON *in = cJSON_Parse(json), *o = cJSON_CreateObject();
    int n = 0;
    for (int i = 0; in && i < 4; i++) {
        cJSON *v = cJSON_DetachItemFromObject(in, allowed[i]);
        if (v) { cJSON_AddItemToObject(o, allowed[i], v); n++; }
    }
    char *s = n ? cJSON_PrintUnformatted(o) : NULL;
    bool ok = s && cfg_from_json(s) && cfg_save();
    free(s);
    cJSON_Delete(in);
    cJSON_Delete(o);
    ESP_LOGI(TAG, "remote config (%d keys): %s", n, ok ? "saved" : "rejected");
    char b[96];
    xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
    snprintf(b, sizeof b, "{\"ok\":%s,\"site\":\"%s\"}", ok ? "true" : "false", g_cfg.site);
    xSemaphoreGive(g_cfg_lock);
    if (ok) { set_identity(); lmesh_announce(s_gw); node_poll_now(); }   /* new topic + schedule now */
    publish("config/ack", b);
}

/* "radio" {"rate":2,"in_s":30} or {"cancel":true}, from the gateway's mesh-rate change */
static void do_radio(const char *json)
{
    cJSON *o = cJSON_Parse(json);
    cJSON *r = o ? cJSON_GetObjectItem(o, "rate") : NULL, *in = o ? cJSON_GetObjectItem(o, "in_s") : NULL;
    const char *err = NULL;
    if (o && cJSON_IsTrue(cJSON_GetObjectItem(o, "cancel"))) {
        if (s_trial.state == 1) s_trial.state = 0;
    } else if (!cJSON_IsNumber(r) || r->valueint < 0 || r->valueint > 2) err = "bad rate";
    else if (s_trial.state == 2) err = "trial running";
    else if (r->valueint == g_cfg.e52_rate) s_trial.state = 0;
    else {
        int d = cJSON_IsNumber(in) ? in->valueint : 20;
        if (d < 3) d = 3;
        if (d > 900) d = 900;
        s_trial.state = 0;
        s_trial.new_rate = (uint8_t)r->valueint;
        s_trial.at_s = uptime_s() + d;
        s_trial.state = 1;
        ESP_LOGW(TAG, "air rate change to %u in %d s", s_trial.new_rate, d);
        if (s_poll_task) xTaskNotify(s_poll_task, NOTE_WAKE, eSetBits);
    }
    cJSON_Delete(o);
    char b[80];
    snprintf(b, sizeof b, "{\"ok\":%s%s%s%s}", err ? "false" : "true", err ? ",\"err\":\"" : "", err ? err : "", err ? "\"" : "");
    publish("radio/ack", b);
}

static void do_downlink(const work_t *w)
{
    const uint8_t *z = memchr(w->data, 0, w->len);
    if (!z) return;
    const char *sub = (const char *)w->data;
    char payload[LMESH_SEG * 2 + 1];
    int pl = w->len - (z - w->data) - 1;
    memcpy(payload, z + 1, pl);
    payload[pl] = 0;
    ESP_LOGI(TAG, "downlink %s: %s", sub, payload);
    if (!strcmp(sub, "write")) do_write(payload);
    else if (!strcmp(sub, "poll")) node_poll_now();
    else if (!strcmp(sub, "status")) publish_status();
    else if (!strcmp(sub, "config")) do_config(payload);
    else if (!strcmp(sub, "radio")) do_radio(payload);
    else ESP_LOGW(TAG, "unknown downlink '%s'", sub);
}

static void worker(void *a)
{
    static work_t w;
    for (;;) {
        xQueueReceive(s_workq, &w, portMAX_DELAY);
        if (w.kind == 0) { do_modbus(&w); s_last.busy = false; }
        else do_downlink(&w);
    }
}

/* lmesh task context: keep short */
static void on_modbus(const lmesh_dgram_t *d)
{
    if (d->len < 2 || d->len > sizeof ((work_t *)0)->data) return;
    s_st.mb_reqs++;
    if (s_last.src == d->src && s_last.seq == d->seq) {
        s_st.mb_dups++;
        if (s_last.busy) return;                       /* still on the RS-485 bus: reply will go out */
        if (esp_timer_get_time() - s_last.done_us < 15000000LL) {   /* our reply was lost: resend it */
            lmesh_send_udp(d->src, d->seq, LMESH_PORT_MODBUS, d->sport, s_last.rsp, s_last.rlen);
            return;
        }
    }
    static work_t w;
    w.kind = 0; w.src = d->src; w.seq = d->seq; w.sport = d->sport; w.len = d->len;
    memcpy(w.data, d->data, d->len);
    s_last.src = d->src; s_last.seq = d->seq; s_last.busy = true;
    if (xQueueSend(s_workq, &w, 0) != pdTRUE) s_last.busy = false;
}

static void on_downlink(const lmesh_dgram_t *d)
{
    static uint16_t last_src; static int last_seq = -1; static int64_t last_us;
    if (d->len > sizeof ((work_t *)0)->data) return;
    lmesh_send_udp(d->src, d->seq, LMESH_PORT_MQTT_DN, d->sport, (const uint8_t *)"", 0);   /* ack */
    int64_t now = esp_timer_get_time();
    if (d->src == last_src && d->seq == last_seq && now - last_us < 60000000LL) return;   /* gateway resend */
    last_src = d->src; last_seq = d->seq; last_us = now;
    static work_t w;
    w.kind = 1; w.src = d->src; w.seq = d->seq; w.sport = d->sport; w.len = d->len;
    memcpy(w.data, d->data, d->len);
    xQueueSend(s_workq, &w, 0);
}

static void on_announce(uint16_t src, const char *kind)
{
    if (strcmp(kind, "gateway") != 0) return;
    s_st.gw_seen_s = uptime_s();
    if (s_gw != src) ESP_LOGI(TAG, "gateway is 0x%04X", src);
    s_gw = src;
    node_identity_changed();                           /* answer with our own ANNOUNCE */
}

/* ---------------- web helpers ---------------- */

void node_get_stats(node_stats_t *s) { *s = s_st; s->gw_addr = s_gw; }

int node_value_count(void) { return g_cfg.poll_n; }

void node_value_text(int i, char *out, size_t n)
{
    out[0] = 0;
    xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
    if (i >= 0 && i < g_cfg.poll_n) {
        const char *nm = g_cfg.poll[i].name;
        if (!s_vals[i].at_s) snprintf(out, n, "%.6s -", nm);
        else if (!s_vals[i].ok) snprintf(out, n, "%.6s ERR", nm);
        else snprintf(out, n, "%.5s %.4g", nm, s_vals[i].v);
    }
    xSemaphoreGive(g_cfg_lock);
}

char *node_values_json(void)
{
    cJSON *a = cJSON_CreateArray();
    uint32_t now = uptime_s();
    xSemaphoreTake(g_cfg_lock, portMAX_DELAY);
    for (int i = 0; i < g_cfg.poll_n; i++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "name", g_cfg.poll[i].name);
        if (s_vals[i].at_s) {
            if (s_vals[i].ok) cJSON_AddNumberToObject(e, "v", s_vals[i].v); else cJSON_AddNullToObject(e, "v");
            cJSON_AddNumberToObject(e, "age", now - s_vals[i].at_s);
        }
        cJSON_AddItemToArray(a, e);
    }
    xSemaphoreGive(g_cfg_lock);
    char *s = cJSON_PrintUnformatted(a);
    cJSON_Delete(a);
    return s;
}

/* ---------------- start ---------------- */

void node_start(void)
{
    s_gw = g_cfg.gw_addr;
    s_workq = xQueueCreate(4, sizeof(work_t));
    lmesh_bind(LMESH_PORT_MODBUS, on_modbus);
    lmesh_bind(LMESH_PORT_MQTT_DN, on_downlink);
    lmesh_on_announce(on_announce);
    set_identity();
    xTaskCreate(worker, "node_wk", 6144, NULL, 8, NULL);
}

void node_after_radio_up(void)
{
    xTaskCreate(poll_task, "node_poll", 6144, NULL, 5, &s_poll_task);
    node_identity_changed();     /* boot ANNOUNCE may have gone out before the E52 was programmed */
}
