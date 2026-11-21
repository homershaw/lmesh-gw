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
| 3V3 / GND | | RS-485 board VCC / GND |

* If the RTU never answers, swap D2 and D3. The TXD LED on the RS-485 board should blink on every poll.
* Power the E52 from a supply that can deliver at least 200 mA at 3.3 V, not the XIAO's 3V3 pin. It draws over 100 mA in bursts at 22 dBm.
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
| `<base>/<site>/status` | up, every 10 min | fw, uptime, mesh address, RTU and radio counters |
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
