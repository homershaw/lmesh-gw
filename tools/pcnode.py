"""
LMESH end-node simulator on a PC-attached E52 (CH340 USB board).

  python pcnode.py COM4 --program      # one-time: put module on gateway's mesh settings
  python pcnode.py COM4                # run node agent (announce, ping/pong, Modbus sim)

Modbus sim: FC3/FC4 return register i = 1000*unit + addr + i ; FC6/FC16 store into a table.
"""
import argparse, struct, sys, time, threading, serial

LMESH_VER, T_UDP, T_ANN, T_PING, T_PONG = 0x10, 1, 2, 3, 4
HDR, SEG = 7, 193

ap = argparse.ArgumentParser()
ap.add_argument('port')
ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x0002)
ap.add_argument('--gw', type=lambda s: int(s, 0), default=0x0001)
ap.add_argument('--channel', type=int, default=65)
ap.add_argument('--rate', type=int, default=2)
ap.add_argument('--panid', type=lambda s: int(s, 0), default=0x1234)
ap.add_argument('--power', type=int, default=22)
ap.add_argument('--csma', type=int, default=None, help='set AT+CSMA_RNG (ms) at startup')
ap.add_argument('--program', action='store_true', help='write mesh settings to module flash, then exit')
ap.add_argument('--name', default='pcnode')
a = ap.parse_args()

s = serial.Serial(a.port, a.baud, timeout=0.02)
lock = threading.Lock()
t0 = time.time()


def log(*x):
    print(f'[{time.time() - t0:8.3f}]', *x, flush=True)


def at(cmd, wait=0.8):
    """Send AT command, return text reply (only safe before rx thread starts)."""
    s.reset_input_buffer()
    s.write(cmd.encode())
    end, out = time.time() + wait, b''
    while time.time() < end:
        out += s.read(256)
        if out.endswith(b'\r\n') and time.time() > end - wait + 0.05:
            break
    return out.decode(errors='replace').strip()


if a.program:
    cmds = ['AT+HEAD=1', 'AT+BACK=0', 'AT+SRC_PORT=1,1', f'AT+CHANNEL={a.channel},1',
            f'AT+POWER={a.power},1', f'AT+PANID={a.panid},1', f'AT+SRC_ADDR={a.addr},1',
            'AT+TYPE=0', 'AT+CSMA_RNG=40', f'AT+RATE={a.rate}', 'AT+OPTION=1,1',
            f'AT+DST_ADDR={a.gw},1']
    for c in cmds:
        r = at(c)
        if not r:
            r = at(c)
        print(f'{c:24s} -> {r}')
        time.sleep(0.05)
    print('AT+RESET ->', at('AT+RESET', 1.5))
    time.sleep(1.5)
    print(at('AT+HEAD=1'), '|', at('AT+BACK=0'))
    print(at('AT+INFO=?', 1.2))
    sys.exit(0)

# ---------------- node agent ----------------
regs = {}
seq_ctr = [0]
cur_dst = [None]
pending_at = []


def next_seq():
    seq_ctr[0] = (seq_ctr[0] + 1) & 0xFF
    return seq_ctr[0]


def raw_send(dst, frame):
    with lock:
        if cur_dst[0] != dst:
            s.write(f'AT+DST_ADDR={dst},0'.encode())
            time.sleep(0.06)            # reply is consumed/ignored by rx thread
            cur_dst[0] = dst
        s.write(frame)
        # 7 kbps airtime pacing
        time.sleep(((len(frame) + 16) * 8) / {0: 62500, 1: 21875, 2: 7000}[a.rate] + 0.04)


def lsend(dst, typ, seq, sport, dport, data=b''):
    total = max(1, -(-len(data) // SEG))
    for i in range(total):
        seg = data[i * SEG:(i + 1) * SEG]
        raw_send(dst, bytes([LMESH_VER | typ, seq, (i << 4) | total]) + struct.pack('>HH', sport, dport) + seg)


def modbus(unit, pdu):
    fc = pdu[0]
    try:
        if fc in (3, 4):
            addr, n = struct.unpack('>HH', pdu[1:5])
            if not 1 <= n <= 125:
                return bytes([fc | 0x80, 3])
            vals = [regs.get(addr + i, (1000 * unit + addr + i) & 0xFFFF) for i in range(n)]
            return bytes([fc, 2 * n]) + struct.pack(f'>{n}H', *vals)
        if fc == 6:
            addr, v = struct.unpack('>HH', pdu[1:5])
            regs[addr] = v
            return pdu[:5]
        if fc == 16:
            addr, n, bc = struct.unpack('>HHB', pdu[1:6])
            for i in range(n):
                regs[addr + i] = struct.unpack('>H', pdu[6 + 2 * i:8 + 2 * i])[0]
            return pdu[:5]
        return bytes([fc | 0x80, 1])
    except struct.error:
        return bytes([fc | 0x80, 3])


reasm = {}


def on_frame(src, data):
    if len(data) < HDR or (data[0] & 0xF0) != LMESH_VER:
        log(f'non-LMESH from 0x{src:04X}: {data!r}')
        return
    typ, seq, frag = data[0] & 0x0F, data[1], data[2]
    sport, dport = struct.unpack('>HH', data[3:7])
    pl = data[7:]
    idx, total = frag >> 4, frag & 0x0F
    if total > 1:
        r = reasm.setdefault((src, seq), {})
        r[idx] = pl
        if len(r) < total:
            return
        pl = b''.join(r[i] for i in range(total))
        del reasm[(src, seq)]
    if typ == T_PING:
        log(f'PING from 0x{src:04X} seq {seq} -> PONG')
        lsend(src, T_PONG, seq, 0, 0)
    elif typ == T_ANN:
        log(f'ANNOUNCE from 0x{src:04X}: {pl.split(b"\\0")[:3]}')
    elif typ == T_UDP and dport == 502 and len(pl) >= 2:
        unit, pdu = pl[0], pl[1:]
        rsp = modbus(unit, pdu)
        log(f'Modbus from 0x{src:04X} seq {seq} unit {unit} pdu {pdu.hex()} -> {rsp.hex()}')
        lsend(src, T_UDP, seq, 502, 502, bytes([unit]) + rsp)
    elif typ == T_UDP and dport == 1884:
        log(f'MQTT down: {pl!r}')
    elif typ == T_UDP:
        log(f'UDP 0x{src:04X} {sport}->{dport}: {pl!r}  (echoing)')
        lsend(src, T_UDP, seq, dport, sport, pl)
    else:
        log(f'type {typ} from 0x{src:04X}')


def rx_loop():
    buf = b''
    last = time.time()
    while True:
        b = s.read(512)
        now = time.time()
        if b:
            buf += b
            last = now
        elif buf and now - last > 0.03:
            # flush leftover text
            txt = buf.decode(errors='replace').strip()
            if txt:
                log('module:', txt)
            buf = b''
        while buf:
            if 0xC1 <= buf[0] <= 0xC4:
                if len(buf) < 8:
                    break
                ln = buf[1]
                if len(buf) < 8 + ln:
                    break
                _, _, pan, src, dst = struct.unpack('<BBHHH', buf[:8])
                data = buf[8:8 + ln]
                buf = buf[8 + ln:]
                try:
                    on_frame(src, data)
                except Exception as e:
                    log('handler error', e)
            else:
                # text (AT reply) up to CRLF
                i = buf.find(b'\r\n')
                j = next((k for k in range(len(buf)) if 0xC1 <= buf[k] <= 0xC4), None)
                if j is not None and (i < 0 or j < i):
                    if j:
                        log('module:', buf[:j].decode(errors='replace').strip())
                    buf = buf[j:]
                    continue
                if i < 0:
                    break
                line = buf[:i].decode(errors='replace').strip()
                buf = buf[i + 2:]
                if line and not line.startswith('AT+DST_ADDR='):
                    log('module:', line)


if a.csma is not None:
    log('CSMA:', at(f'AT+CSMA_RNG={a.csma}'), '|', at('AT+CSMA_RNG=?'))
log(f'node 0x{a.addr:04X} on {a.port}; gateway 0x{a.gw:04X}')
threading.Thread(target=rx_loop, daemon=True).start()
lsend(a.gw, T_ANN, next_seq(), 0, 0, f'{a.name}\0sim\x000.1\0'.encode())
log('ANNOUNCE sent to gateway')
lsend(a.gw, T_UDP, next_seq(), 0, 1883, b'hello\0node up')
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
