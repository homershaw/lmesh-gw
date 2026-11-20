# LMESH Gateway — ESP32-S31 + Ebyte E52-900NW22S

Border gateway for a LoRa mesh (902–928 MHz). It carries UDP/IPv4 datagrams across the mesh, and the gateway ends TCP (Modbus TCP, MQTT, HTTP) itself, so no TCP traffic goes over the air.

| Service | Where |
|---|---|
| Web UI (status, nodes, Modbus test, E52 config, Wi-Fi/MQTT, unit map) | `http://192.168.4.1` on AP `LMESH-GW-xxxx` (pw `lmesh1234`), or the STA IP |
| Modbus TCP server | port 502 → mesh node (unit map, or unit N → node N) |
| Gateway diagnostics | Modbus unit 255, FC3/4, regs 0–15 |
| MQTT client bridge | `<base>/<NODE>/…` up, `<base>/<NODE>/set/…` down |
| UDP bridge | port 5252, `[node IPv4][port][payload]` |

## Build (tonight)

ESP32-S31 builds only on **ESP-IDF v6.1 / master**.

```bash
cd lmesh-gw
idf.py set-target esp32s31
idf.py menuconfig        # "LoRa Mesh Gateway": set E52 UART pins for your Coreboard
idf.py build flash monitor
```

The component manager downloads `espressif/mqtt` and `espressif/cjson` on IDF 6.x (they were moved out of IDF). On IDF 5.x it uses the built-in versions.

## Wiring E52-900NW22S

| E52 pin | Signal | ESP32-S31 |
|---|---|---|
| 28 VDD | 3.3 V (separate LDO is better; TX peaks ~110 mA at 22 dBm) | 3V3 |
| 13/15/27 | GND | GND |
| 9 PA2 UART_TXD | module TX | `LMGW_E52_RX_GPIO` (default 18) |
| 10 PA3 UART_RXD | module RX | `LMGW_E52_TX_GPIO` (default 17) |
| 17 NRST | reset (optional) | `LMGW_E52_RST_GPIO` (default -1) |
| 14 ANT | 915 MHz antenna — **don't transmit without it** | — |

**GPIO 17/18 are placeholders. Check them against the S31 Coreboard pinout before flashing.** You can also change pins later on the Radio page (reboot to apply). A bad pin setting won't boot-loop the board, because the web UI stays up.

## Radio settings

* Channel n = 850.125 + n MHz. The UI only offers 52–77 (902.125–927.125 MHz). Default is 65 = 915.125 MHz.
* Air rate 2 = 7 kbps (default), 1 = 21.875 kbps, 0 = 62.5 kbps. The E52 sets bandwidth and SF internally for each rate. Its AT command set has no separate 500 kHz BW setting.
* Max 200 B per packet. LMESH uses 7 B of header, which leaves 193 B per fragment, with up to 4 fragments (772 B).
* On boot the gateway sends the radio settings to the module only if they changed (`AT+HEAD=1`, `BACK=0`, `CHANNEL`, `POWER`, `PANID`, `SRC_ADDR`, `TYPE=0`, `CSMA_RNG`, `RATE`, `OPTION`), then resets it. "Save + apply to E52" does the same on demand and shows each reply.
* All nodes need the same PANID, channel and rate. Routers use addresses 0x0001–0x7FFF and battery or terminal nodes use 0x8000–0xFFFE. Node IPv4 = `10.52.<hi>.<lo>` of the address.

## LMESH frame (what an end node must implement)

Every E52 payload:

```
b0      0x10 | type     1=UDP 2=ANNOUNCE 3=PING 4=PONG
b1      seq             reply with the same seq
b2      frag            (index<<4)|total, 0x01 = single
b3..4   src port  BE
b5..6   dst port  BE
b7..    payload (<=193)
```

End-node agent duties (any MCU + E52 with `AT+HEAD=1`, `AT+TYPE` set, unicast to the gateway address):

1. **On boot**: send ANNOUNCE to the gateway with payload `name\0kind\0fw\0`.
2. **PING** → reply PONG with the same seq.
3. **UDP dport 502**: payload `[unit][PDU]`. Run it as Modbus RTU on the RS-485/UART (add CRC), then reply UDP sport 502 → dport 502 with `[unit][response PDU]` and **the same seq**. Fragment if the reply is over 193 B (FC3 with more than 95 registers).
4. **MQTT up**: send UDP dport 1883 with `subtopic\0payload`. The gateway publishes it to `<base>/<NODE4HEX>/subtopic`.
5. **MQTT down**: the gateway sends UDP dport 1884 with `subtopic\0payload` from `<base>/<NODE>/set/subtopic`.
6. **UDP bridge replies**: answer to whatever source port the request came from (0xC000+slot).

Avoid sending anything that starts with `AT+`. The module treats it as a command. LMESH frames always start with 0x1x, so normal traffic can't hit this.

## Timing

At 7 kbps, a Modbus read request (~15 B) takes about 60–100 ms per hop of airtime, plus CSMA. Set your SCADA poll timeout to at least the gateway's `mb_timeout_ms` (default 6000), and poll slowly (≥10 s per device). The gateway sends one transaction per node at a time. Unanswered requests come back as Modbus exception 0x0B.

## MQTT topics (base default `lmesh/<last4MAC>`)

* `<base>/gw/status` — `online`/`offline` (retained, LWT)
* `<base>/gw/stats` — JSON every 60 s
* `<base>/gw/nodes` — JSON node table every 60 s (retained)
* `<base>/<NODE>/<subtopic>` — uplink from node
* `<base>/<NODE>/set/<subtopic>` — downlink to node

## Known limits / to verify on hardware

* The E52 query-reply format (`AT+INFO=?` etc.) isn't documented. The parser treats any text up to a 20 ms gap as the reply.
* The web Modbus test and Ping run in the HTTP server task, so the UI pauses until the reply or timeout.
* No RSSI per packet: the E52 doesn't send it to the UART.
* The web UI has no login yet. Keep the AP password set, and don't expose port 80 to the internet.
