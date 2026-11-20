/*
 * LoRa Mesh Border Gateway - ESP32-S31 + Ebyte E52-900NW22S
 *  - Modbus TCP server :502  -> Modbus over LMESH UDP to end nodes
 *  - MQTT client bridge       -> broker (riotdash etc.)
 *  - UDP bridge :5252         -> any mesh node IPv4 10.52.x.y
 *  - Web UI :80 (AP "LMESH-GW-xxxx" @192.168.4.1 always on + STA)
 */
#include "gw.h"
#include "config.h"
#include "lmesh.h"
#include "e52.h"
#include "modbus_gw.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "main";

static uint32_t radio_hash(void)
{
    const gw_cfg_t *c = &g_cfg;
    uint32_t h = 2166136261u;
    uint32_t v[] = { c->e52_channel, c->e52_rate, (uint32_t)(int32_t)c->e52_power, c->e52_panid, c->e52_addr, c->e52_csma };
    for (unsigned i = 0; i < sizeof v / sizeof v[0]; i++) { h ^= v[i]; h *= 16777619u; }
    return h;
}

static void sync_radio_config(void)
{
    nvs_handle_t nh;
    uint32_t stored = 0, now = radio_hash();
    if (nvs_open("lmgw", NVS_READWRITE, &nh) != ESP_OK) return;
    nvs_get_u32(nh, "e52hash", &stored);
    if (stored != now) {
        ESP_LOGI(TAG, "radio settings changed -> programming E52 (ch %u = %.3f MHz, rate %u, %d dBm, pan 0x%04X, addr 0x%04X)",
                 g_cfg.e52_channel, e52_chan_mhz(g_cfg.e52_channel), g_cfg.e52_rate, g_cfg.e52_power,
                 g_cfg.e52_panid, g_cfg.e52_addr);
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
    wifi_start();               /* also sets g_gw_id */

    lmesh_init();
    modbus_gw_start();          /* binds mesh port 502 */
    mqtt_gw_start();            /* binds mesh port 1883 */
    udp_bridge_start();         /* default handler */
    web_start();

    if (lmesh_start() == ESP_OK) sync_radio_config();
    else ESP_LOGE(TAG, "E52 not responding - web UI still available for pin/baud config");

    ESP_LOGI(TAG, "LMESH gateway %s up, id %s", GW_FW_VERSION, g_gw_id);
}
