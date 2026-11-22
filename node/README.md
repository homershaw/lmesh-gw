# LMESH field node (XIAO ESP32-C6)

Masthead unit: Seeed XIAO ESP32-C6, Ebyte E52-900NW22S, and an auto-direction TTL↔RS-485 board down to the RTU.

* **Modbus pass-through.** Modbus TCP on the gateway → mesh → this node → Modbus RTU on RS-485, so SCADA can poll the RTU as before.
* **Poll and push.** The node reads a poll list from the RTU on a timer and publishes one JSON message per poll. The gateway forwards it to MQTT as `<base>/<site>/data`.
* **Site page.** A Wi-Fi AP `LMESH-NODE-xxxx` serves a page at `http://192.168.4.1`. Login is user `admin` with the AP password (`lmesh1234` by default).
* **Relay.** The node routes mesh traffic for other units, because the E52 runs as a router.

## Wiring

| XIAO | GPIO | to |
|---|---|---|
| D2 | 2 | RS-485 board TXD (ESP transmits) |
| D3 | 21 | RS-485 board RXD (ESP receives) |
| D4 | 22 | E52 RXD |
| D5 | 23 | E52 TXD |
| D6 | 16 | E52 reset (optional; set `uart_rst` to 16 if wired) |
| D9 | 20 | OLED SDA (optional display) |
| D10 | 18 | OLED SCL |
| 3V3 / GND | | RS-485 board VCC / GND, OLED VCC / GND |

* If the RTU never answers, swap D2 and D3. The TXD LED on the RS-485 board should blink on every poll.
* Power the E52 from a supply that can deliver at least 200 mA at 3.3 V, not the XIAO's 3V3 pin. It draws over 100 mA in bursts at 22 dBm.
* The display is optional: a 128×64 I2C OLED (SSD1306, or SH1106 if selected under Settings) at address 0x3C or 0x3D. It shows site, mesh address, RSSI and next hop towards the gateway, RTU counters and the latest poll values. It can be plugged in while running.
* The console runs over USB (USB-Serial-JTAG), so UART0 is free for RS-485.
* To use the XIAO's U.FL connector for Wi-Fi, go to Settings → Wi-Fi antenna → External, then reboot.

## Build / flash

```
cd node
idf.py set-target esp32c6     # first time only
idf.py -p COMx build flash monitor
```

The E52 and LMESH code is shared with the gateway (`../shared`).

## First setup

1. Join `LMESH-NODE-xxxx`, open `http://192.168.4.1`, and log in as `admin` / `lmesh1234`.
2. **Settings:** set the site ID (or click ✎ in the Site column of the gateway's Nodes page; it is sent over the mesh) (LSD, e.g. `01-02-034-05W5`), the RS-485 baud/parity, and a new AP password.
3. **Radio:** check that channel, rate, PAN ID and CSMA match the gateway (the defaults do). The node address defaults to a value derived from the MAC (0x0100–0x7FFE); change it if you keep an address plan. Then click **Save & apply to E52**.
4. **RS-485 test:** read a few registers to confirm wiring and settings.
5. **Poll list:** add points (name, unit, function, address, type, scale), set the interval, and save.

## MQTT (through the gateway)

| Topic | Direction | Payload |
|---|---|---|
| `<base>/<site>/data` | up, every poll | `{"site":"01-02-034-05W5","t":1234,"tank_level":42.7,"pump":1}` (a failed read gives `null` plus `"_err":n`) |
| `<base>/<site>/status` | up, every 10 min | fw, uptime, mesh address, RTU and radio counters, plus `rssi` (dBm) and `hop` (next node) of its route to the gateway |
| `<base>/<site>/set/write` | down | `{"name":"pump_sp","value":12.5}`, or raw `{"unit":1,"addr":100,"value":5,"type":"u16"}` |
| `<base>/<site>/write/ack` | up | `{"ok":true}` or `{"ok":false,"err":"rtu timeout"}` |
| `<base>/<site>/set/poll` | down | (any payload) triggers an immediate poll |
| `<base>/<site>/set/config` | down | `{"site":"…","name":"…","poll_s":60,"poll":[…]}`. Only these keys are accepted; radio, RS-485 and Wi-Fi settings can only be changed locally. |
| `<base>/<site>/config/ack` | up | `{"ok":true,"site":"…"}` |

Notes on topics and delivery:

* **Where `<site>` comes from.** The node includes its site ID in ANNOUNCE, which it sends on boot, whenever the gateway announces itself, and every 10 minutes. The gateway uses that ID in place of the 4-hex node number. If two live nodes claim the same site ID, the second one is shown as a duplicate on the gateway's Nodes page and keeps its node number.
* **Large polls.** A poll that produces more than about 560 bytes of JSON goes out as several `data` messages. Each one is complete JSON with `site` and `t`.
* **Writes.** These use FC5 for coils, FC6 for 16-bit values and FC16 for 32-bit values. The value is converted back through the point's scale and offset. Points read with FC2 or FC4 are read-only.
* **Delivery is best effort, like MQTT QoS 0.** A lost uplink is replaced by the next poll.

## Firmware updates

The flash holds two firmware slots (`partitions.csv`). After one USB flash of a build with this layout, later updates need no cable:

* **Over the node's Wi-Fi:** join `LMESH-NODE-xxxx`, open `http://192.168.4.1`, Settings → Firmware, choose `node/build/lmesh_node.bin`, **Upload and restart**. The file is checked (right project, right chip, intact) before anything changes.
* **On trial:** the new firmware must reach the gateway over the mesh within 10 minutes. If it cannot, the node restarts into the previous firmware by itself. On a bench with no gateway, use **Keep it** on the node's page.

The gateway works the same way from its own page (`build/lmesh_gw.bin`); it keeps a new firmware once someone logs in to its web page.

### Over the mesh (no site visit)

On the gateway page, in the **Node firmware over the mesh** card: store `node/build/lmesh_node.bin` on the gateway once, pick a node, **Send to node**. The gateway feeds the image to the node one radio frame at a time (`shared/fwproto.h`).

* Measured: 1.07 MB in 46.5 minutes at 21.9 kbps over one hop, about 390 bytes/s. Expect two to three times longer at 7 kbps.
* Every packet carries a checksum and the whole image is checked again at the end, before the node installs anything.
* An interrupted transfer resumes when started again, as long as the node has not restarted.
* The node then runs the new firmware on trial, exactly as with a Wi-Fi upload, and returns to the old one if it cannot reach the gateway.
* Needs node firmware 0.6.1 or newer on the node being updated.
