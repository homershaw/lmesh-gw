# LMESH Gateway — ESP32-S31 + Ebyte E52-900NW22S

Border gateway for a LoRa mesh (902–928 MHz). It carries UDP/IPv4 datagrams across the mesh, and the gateway ends TCP (Modbus TCP, MQTT, HTTP) itself, so no TCP traffic goes over the air.

| Service | Where |
|---|---|
| Web UI (status, nodes, Modbus test, E52 config, Wi-Fi/MQTT, unit map) | `http://192.168.4.1` on AP `LMESH-GW-xxxx` (pw `lmesh1234`), or the STA IP. Login: user `admin`, password = web password if set, else the AP password |
| Modbus TCP server | port 502 → mesh node (unit map, or unit N → node N) |
| Gateway diagnostics | Modbus unit 255, FC3/4, regs 0–15 |
| MQTT client bridge | `<base>/<NODE>/…` up, `<base>/<NODE>/set/…` down |
| UDP bridge | port 5252, `[node IPv4][port][payload]` |

## Repo layout

* `main/` holds the gateway firmware (ESP32-S31).
* `node/` holds the field node firmware (XIAO ESP32-C6 + E52 + RS-485). See [node/README.md](node/README.md).
* `shared/` holds the E52 driver and the LMESH layer, which both firmwares use.
* `tools/` holds the PC end-node simulator, benchmarks, and a small test MQTT broker.

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
* Air rate 2 = 7 kbps (longest range), 1 = 21.875 kbps (default), 0 = 62.5 kbps. The E52 sets bandwidth and SF internally for each rate. Its AT command set has no separate 500 kHz BW setting.
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

1. **On boot**: send ANNOUNCE to the gateway with payload `name\0kind\0fw\0site\0` (site optional). Re-send it when the gateway announces itself (kind `gateway`) and every 10 min, so a rebooted gateway relearns site IDs.
2. **PING** → reply PONG with the same seq.
3. **UDP dport 502**: payload `[unit][PDU]`. Run it as Modbus RTU on the RS-485/UART (add CRC), then reply UDP sport 502 → dport 502 with `[unit][response PDU]` and **the same seq**. Fragment if the reply is over 193 B (FC3 with more than 95 registers).
4. **MQTT up**: send UDP dport 1883 with `subtopic\0payload`. The gateway publishes it to `<base>/<NODE4HEX>/subtopic`.
5. **MQTT down**: the gateway sends UDP dport 1884 with `subtopic\0payload` from `<base>/<NODE>/set/subtopic`.
6. **UDP bridge replies**: answer to whatever source port the request came from (0xC000+slot).

Avoid sending anything that starts with `AT+`. The module treats it as a command. LMESH frames always start with 0x1x, so normal traffic can't hit this.

## Timing

Measured on the bench (one hop, Modbus FC3 read of 4 registers, Modbus TCP round trip): 7 kbps median ~1.2 s (max ~2.7 s); 21.875 kbps median ~375 ms (max ~620 ms); 62.5 kbps median ~290 ms. Each step up in rate costs roughly 4–5 dB of link budget (range). Set your SCADA poll timeout to at least the gateway's `mb_timeout_ms` (default 6000), and poll slowly (≥10 s per device). The gateway sends one transaction per node at a time. If a reply doesn't arrive within about 3× a normal round trip (≈1.5 s at 21.875 kbps), it resends the same request with the same seq; nodes must therefore handle a repeated request (Modbus reads/writes are safe to repeat). Requests still unanswered at `mb_timeout_ms` come back as Modbus exception 0x0B.

CSMA backoff (`e52_csma`) defaults to 40 ms (factory 127). On the bench this cut 1-hop ping RTT at 21.875 kbps from a median of 371 ms to 224 ms. The E52 does not keep `CSMA_RNG` across a reset, so the gateway sends it on every boot and after every config apply. End nodes should do the same.

## MQTT topics (base default `lmesh/<last4MAC>`)

* `<base>/gw/status` — `online`/`offline` (retained, LWT)
* `<base>/gw/stats` — JSON every 60 s
* `<base>/gw/nodes` — JSON node table every 60 s (retained)
* `<base>/<SITE or NODE>/<subtopic>` — uplink from node. `<SITE>` is the site ID the node sent in ANNOUNCE (4th field). If the node sent none, or another live node already holds that ID, the 4-hex node number is used instead.
* `<base>/<SITE or NODE>/set/<subtopic>` — downlink to node

## Known limits / to verify on hardware

* The E52 query-reply format (`AT+INFO=?` etc.) isn't documented. The parser treats any text up to a 20 ms gap as the reply.
* The web Modbus test and Ping run in the HTTP server task, so the UI pauses until the reply or timeout.
* No RSSI per packet: the E52 doesn't send it to the UART.
* The web UI uses HTTP Basic login (user `admin`). That is plain HTTP, so the password is only as private as the network: keep the AP on WPA2 and don't expose port 80 to the internet. Modbus TCP (502) has no authentication; that's the protocol.
* With no Station SSID set, the gateway runs AP-only (no idle STA interface). Setting an SSID switches to AP+STA after a reboot.
* The AP's DHCP leases live in RAM. After a gateway reboot, clients get fresh addresses; a client that kept its old one can briefly clash until it renews.
