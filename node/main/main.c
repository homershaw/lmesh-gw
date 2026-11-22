/*
 * LMESH field node - Seeed XIAO ESP32-C6 + Ebyte E52-900NW22S + RS-485
 *  - mesh Modbus pass-through to the RTU (gateway Modbus TCP -> here -> RS-485)
 *  - polls the RTU and pushes JSON to the gateway's MQTT bridge, keyed by site id
 *  - Wi-Fi AP "LMESH-NODE-xxxx" with a local web page for the site
 */
#include "config.h"
#include "lmesh.h"
#include "e52.h"
#include "rtu.h"
#include "node.h"
#include "ota.h"
#include "gps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "main";

void wifi_start(void);
void web_start(void);
void display_start(void);
void fwup_start(void);

/* A new node firmware is kept once it has reached the gateway over the mesh:
 * that is the path the next update would have to come in by. On a bench with
 * no gateway, "Keep it" on the node's own page confirms it by hand. */
static bool ota_ok(void)
{
    node_stats_t ns;
    node_get_stats(&ns);
    return lmesh_ping(ns.gw_addr, 6000) >= 0;
}

/* program the E52 only when radio settings changed (writes module flash) */
static uint32_t radio_hash(void)
{
    const node_cfg_t *c = &g_cfg;
    uint32_t h = 2166136261u;
    uint32_t v[] = { c->e52_channel, c->e52_rate, (uint32_t)(int32_t)c->e52_power, c->e52_panid, c->e52_addr, c->e52_csma };
    for (unsigned i = 0; i < sizeof v / sizeof v[0]; i++) { h ^= v[i]; h *= 16777619u; }
    return h;
}

static void sync_radio_config(void)
{
    nvs_handle_t nh;
    uint32_t stored = 0, now = radio_hash();
    if (nvs_open("lmnode", NVS_READWRITE, &nh) != ESP_OK) return;
    nvs_get_u32(nh, "e52hash", &stored);
    bool module_ok = e52_matches_cfg();
    if (stored != now || !module_ok) {
        if (stored == now) ESP_LOGW(TAG, "E52 settings differ from config (new or swapped module?) -> reprogramming");
        ESP_LOGI(TAG, "radio settings changed -> programming E52 (%.3f MHz, rate %u, %d dBm, pan 0x%04X, addr 0x%04X)",
                 e52_chan_mhz(g_cfg.e52_channel), g_cfg.e52_rate, g_cfg.e52_power, g_cfg.e52_panid, g_cfg.e52_addr);
        if (e52_apply_config(NULL, 0) == ESP_OK) {
            nvs_set_u32(nh, "e52hash", now);
            nvs_commit(nh);
        }
    }
    nvs_close(nh);
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    cfg_load();
    wifi_start();
    rtu_init();
    if (g_cfg.gps) gps_start();

    lmesh_init();
    node_start();
    web_start();
    fwup_start();
    display_start();

    if (lmesh_start() == ESP_OK) {
        sync_radio_config();
        node_after_radio_up();
    } else {
        ESP_LOGE(TAG, "E52 not responding - check wiring (TX GPIO%d -> E52 RXD, RX GPIO%d <- E52 TXD)",
                 g_cfg.uart_tx, g_cfg.uart_rx);
    }
    ota_watch(ota_ok, 600);
    ESP_LOGI(TAG, "LMESH node %s up: site '%s', mesh 0x%04X, gateway 0x%04X",
             NODE_FW_VERSION, g_cfg.site, g_cfg.e52_addr, g_cfg.gw_addr);
}
