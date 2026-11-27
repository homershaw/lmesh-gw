"""
LMESH end-node simulator on a PC-attached Wio-E5 (factory AT firmware, test
mode), speaking the star air frame of shared/wio.c: ack for every unicast
frame, up to 3 repeats, duplicates dropped.

  python wionode.py COM6                          # node 0x0002: channel 73, rate 1, network 0x2001
  python wionode.py COM6 --ping 10                # also ping the gateway every 10 s
  python wionode.py COM6 --up 30                  # also publish an MQTT message every 30 s

Same agent as pcnode.py: ANNOUNCE at start (and in answer to a gateway's),
PONG to PING, Modbus sim (FC3/FC4 register i = 1000*unit + addr + i; FC6/FC16
store), MQTT downlinks acked like node.c (empty UDP to 1884, same seq; resends
ignored), echo of other UDP.
The board must already run at --baud (AT+UART=BR,115200 once).
"""
import argparse, random, struct, sys, time, serial
from datetime import datetime

LMESH_VER, T_UDP, T_ANN, T_PING, T_PONG = 0x10, 1, 2, 3, 4
LHDR, SEG = 7, 193
MAGIC, K_DATA, K_BCAST, K_ACK, HDR = 0xB0, 1, 2, 3, 8
TRIES, AT_TX_MS = 4, 100
SFS = [5, 7, 9, 10, 11, 12]

ap = argparse.ArgumentParser()
ap.add_argument('port')
ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x0002)
ap.add_argument('--gw', type=lambda s: int(s, 0), default=0x0001)
ap.add_argument('--channel', type=int, default=73, help='E52 numbering: 850.125 + n MHz')
ap.add_argument('--rate', type=int, default=1, help='0..5 = SF5, SF7, SF9, SF10, SF11, SF12')
ap.add_argument('--panid', type=lambda s: int(s, 0), default=0x2001, help='network id')
ap.add_argument('--power', type=int, default=10, help='dBm (bench: keep low)')
ap.add_argument('--name', default='wionode')
ap.add_argument('--site', default='')
ap.add_argument('--ping', type=float, default=0, help='ping the gateway every N s')
ap.add_argument('--up', type=float, default=0, help='MQTT-up message every N s')
ap.add_argument('--log', help='also append every line, with wall-clock time, to this file (keeps Windows awake)')
ap.add_argument('--beep', action='store_true', help='range walk: high beep per PONG, low beep per lost ping')
a = ap.parse_args()

SF = SFS[min(a.rate, 5)]
MHZ = 850.125 + a.channel
t0 = time.monotonic()
logf = open(a.log, 'a', encoding='utf-8') if a.log else None
if logf and sys.platform == 'win32':
    import ctypes
    ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)   # ES_CONTINUOUS | ES_SYSTEM_REQUIRED: no idle sleep


def log(*x):
    msg = ' '.join(str(v) for v in x)
    print(f'[{time.monotonic() - t0:8.3f}] {msg}', flush=True)
    if logf:
        logf.write(f'{datetime.now().isoformat(timespec="milliseconds")} {msg}\n')
        logf.flush()


beeps = []                  # played by the main loop when the radio is idle


def beep():
    if a.beep and beeps and sys.platform == 'win32':
        import winsound
        winsound.Beep(*beeps.pop(0))


def air_ms(pl):
    """LoRa time on air: 500 kHz, CR 4/5, 12-symbol preamble, explicit header, CRC (as wio.c)."""
    ts = (1 << SF) / 500.0
    num, den = 8 * pl - 4 * SF + 28 + 16, 4 * SF
    n = -(-num // den) if num > 0 else 0
    return (12 + 4.25 + (2 if SF < 7 else 0) + 8 + n * 5) * ts


ACK_WAIT = (AT_TX_MS + air_ms(HDR) * 2 + 250) / 1000

# ---------------- module link ----------------
s = serial.Serial(a.port, a.baud, timeout=0)
buf = b''
rssi_snr = (0, 0)


def readline(ms):
    """Next non-empty line within ms (one look even at 0), else None."""
    global buf
    end = time.monotonic() + ms / 1000
    while True:
        i = buf.find(b'\n')
        if i >= 0:
            ln, buf = buf[:i].decode(errors='replace').strip(), buf[i + 1:]
            if ln:
                return ln
            continue
        n = s.in_waiting
        if n:
            buf += s.read(n)
            continue
        if time.monotonic() >= end:
            return None
        time.sleep(0.001)


def wait_for(want, ms):
    """Read until a line contains want; packet reports seen meanwhile are handled."""
    end = time.monotonic() + ms / 1000
    while True:
        left = (end - time.monotonic()) * 1000
        if left <= 0:
            return None
        ln = readline(left)
        if ln is None:
            return None
        if on_line(ln):
            continue
        if want in ln:
            return ln


def cmd(c, ms=1000):
    s.write((c + '\r\n').encode())
    return wait_for('+', ms)


def rx_on():
    s.write(b'AT+TEST=RXLRPKT\r\n')
    return wait_for('+TEST: RXLRPKT', 500) is not None


def tx(frame):
    s.write(('AT+TEST=TXLRPKT,"%s"\r\n' % frame.hex().upper()).encode())
    ok = wait_for('TX DONE', air_ms(len(frame)) + AT_TX_MS + 600) is not None
    if not ok:
        log(f'no TX DONE for a {len(frame)}-byte frame')
    if not rx_on():
        log('module did not go back to receive')
        ok = False
    if not ok:
        need_setup[0] = True
    return ok


need_setup = [False]


def setup():
    """Test mode, radio settings, receive. Reopens the port if the board does not
    answer (unplugged and plugged back, or reset)."""
    global s, buf
    try:
        alive = cmd('AT', 500) or cmd('AT', 500)
    except serial.SerialException:          # handle went stale with the unplug
        alive = None
    if not alive:
        try:
            s.close()
        except serial.SerialException:
            pass
        try:
            s = serial.Serial(a.port, a.baud, timeout=0)
        except serial.SerialException as e:
            log(f'{a.port}: {e}')
            return False
        buf = b''
        if not (cmd('AT') or cmd('AT')):
            log(f'{a.port}: no answer at {a.baud} baud')
            return False
    for c in ('AT+MODE=TEST', f'AT+TEST=RFCFG,{MHZ:.3f},SF{SF},500,12,15,{a.power},ON,OFF,OFF'):
        log(c, '->', cmd(c))
    return rx_on()


# ---------------- star frames ----------------
ackq, txq = [], []          # acks owed (dst, seq); LMESH frames waiting (dst, bytes)
out = None                  # frame in flight: dict(dst, seq, f, tries, sent, deadline, resend_at, acked)
last_seq = {}               # src -> last data seq seen (duplicate drop)
my_seq = random.randrange(256)


def hdr(kind, dst, seq):
    return bytes([MAGIC | kind]) + struct.pack('<HHHB', a.panid, dst, a.addr, seq)


def on_line(ln):
    global rssi_snr
    if ln.startswith('+TEST: LEN:'):
        try:
            p = dict(x.strip().split(':') for x in ln[7:].split(','))
            rssi_snr = (int(p['RSSI']), int(p['SNR']))
        except (ValueError, KeyError):
            pass
        return True
    if ln.startswith('+TEST: RX "'):
        try:
            on_packet(bytes.fromhex(ln[11:].rstrip('"')))
        except ValueError:
            log('bad RX line:', ln[:80])
        return True
    return False


def on_packet(f):
    if len(f) < HDR or (f[0] & 0xF0) != MAGIC:
        log(f'not a star frame ({len(f)} B), ignored')
        return
    kind, seq = f[0] & 0x0F, f[7]
    net, dst, src = struct.unpack('<HHH', f[1:7])
    if net != a.panid or (dst != a.addr and dst != 0xFFFF):
        return
    rs = f'RSSI {rssi_snr[0]} SNR {rssi_snr[1]}'
    if kind == K_ACK:
        if out and src == out['dst'] and seq == out['seq'] and not out['acked']:
            out['acked'] = True
            log(f'  ack from 0x{src:04X} seq {seq}: try {out["tries"]}, '
                f'{(time.monotonic() - out["sent"]) * 1000:.0f} ms after first send  ({rs})')
        return
    if kind == K_DATA:
        ackq.append((src, seq))
        if last_seq.get(src) == seq:
            log(f'  repeat of seq {seq} from 0x{src:04X} (our ack was lost) - acked again, dropped')
            return
        last_seq[src] = seq
    elif kind != K_BCAST:
        return
    log(f'<- 0x{src:04X} {"bcast" if kind == K_BCAST else "data"} seq {seq}, {len(f) - HDR} B ({rs})')
    lmesh_rx(src, f[HDR:])


def radio_step():
    """One pass of what wio.c's radio task does."""
    global out, my_seq
    now = time.monotonic()
    if ackq or (out and out['acked']) or (not out and txq):
        wait = 0
    elif out:
        due = out['deadline'] if now < out['deadline'] else out['resend_at']
        wait = max(0, min(10, (due - now) * 1000))
    else:
        wait = 10
    while (ln := readline(wait)) is not None:
        on_line(ln)
        wait = 0
    now = time.monotonic()
    if ackq:
        dst, seq = ackq.pop(0)
        tx(hdr(K_ACK, dst, seq))
    elif out:
        if out['acked']:
            out = None
        elif now >= out['deadline']:
            if out['tries'] >= TRIES:
                log(f'  no ack from 0x{out["dst"]:04X} seq {out["seq"]} after {TRIES} tries')
                out = None
            elif now >= out['resend_at']:
                out['tries'] += 1
                log(f'  repeat {out["tries"] - 1} of seq {out["seq"]}')
                send_out()
    elif txq:
        dst, data = txq.pop(0)
        my_seq = (my_seq + 1) & 0xFF
        kind = K_BCAST if dst == 0xFFFF else K_DATA
        f = hdr(kind, dst, my_seq) + data
        if kind == K_BCAST:
            tx(f)
        else:
            out = dict(dst=dst, seq=my_seq, f=f, tries=1, sent=time.monotonic(), acked=False)
            send_out()


def send_out():
    tx(out['f'])
    out['deadline'] = time.monotonic() + ACK_WAIT
    out['resend_at'] = out['deadline'] + random.uniform(0, air_ms(len(out['f'])) + AT_TX_MS) / 1000


# ---------------- LMESH agent (as pcnode.py) ----------------
regs, reasm, pings, dn_last = {}, {}, {}, {}
tally = [0, 0]              # pings sent, PONGs back
PING_LOST_S = 8
lseq = [0]


def next_lseq():
    lseq[0] = (lseq[0] + 1) & 0xFF
    return lseq[0]


def lsend(dst, typ, seq, sport, dport, data=b''):
    total = max(1, -(-len(data) // SEG))
    for i in range(total):
        txq.append((dst, bytes([LMESH_VER | typ, seq, (i << 4) | total]) +
                    struct.pack('>HH', sport, dport) + data[i * SEG:(i + 1) * SEG]))


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


def lmesh_rx(src, data):
    if len(data) < LHDR or (data[0] & 0xF0) != LMESH_VER:
        log(f'   non-LMESH payload from 0x{src:04X}: {data!r}')
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
        log(f'   PING from 0x{src:04X} seq {seq} -> PONG')
        lsend(src, T_PONG, seq, 0, 0)
    elif typ == T_PONG:
        t = pings.pop(seq, None)
        if t:
            tally[1] += 1
            beeps.append((1800, 60))
        log(f'   PONG from 0x{src:04X} seq {seq}' + (f': round trip {(time.monotonic() - t) * 1000:.0f} ms' if t else '') +
            f'  [{tally[1]}/{tally[0]} pings ok]')
    elif typ == T_ANN:
        f = [x.decode(errors="replace") for x in pl.split(b"\0")[:4]]
        log(f'   ANNOUNCE from 0x{src:04X}: {f}')
        if len(f) > 1 and f[1] == 'gateway':            # as node.c: answer with our own
            lsend(src, T_ANN, next_lseq(), 0, 0, ann.encode())
    elif typ == T_UDP and dport == 502 and len(pl) >= 2:
        unit, pdu = pl[0], pl[1:]
        rsp = modbus(unit, pdu)
        log(f'   Modbus unit {unit} pdu {pdu.hex()} -> {rsp.hex()}')
        lsend(src, T_UDP, seq, 502, 502, bytes([unit]) + rsp)
    elif typ == T_UDP and dport == 1884:
        lsend(src, T_UDP, seq, 1884, sport)             # ack: empty datagram, same seq (as node.c)
        last = dn_last.get(src)
        if last and last[0] == seq and time.monotonic() - last[1] < 60:
            log(f'   MQTT down seq {seq} again (gateway resend) - acked, ignored')
            return
        dn_last[src] = (seq, time.monotonic())
        log(f'   MQTT down seq {seq}: {pl!r}')
    elif typ == T_UDP:
        log(f'   UDP {sport}->{dport}: {pl!r}  (echoing)')
        lsend(src, T_UDP, seq, dport, sport, pl)
    else:
        log(f'   LMESH type {typ} from 0x{src:04X}')


# ---------------- start ----------------
if not setup():
    raise SystemExit(f'{a.port}: board not ready')
log(f'node 0x{a.addr:04X} on {a.port}: {MHZ:.3f} MHz SF{SF} 500 kHz, network 0x{a.panid:04X}, '
    f'gateway 0x{a.gw:04X}; ack wait {ACK_WAIT * 1000:.0f} ms')

ann = f'{a.name}\0sim\0wio-0.1\0' + (f'{a.site}\0' if a.site else '')
lsend(a.gw, T_ANN, next_lseq(), 0, 0, ann.encode())
log('-> ANNOUNCE to gateway')
lsend(a.gw, T_UDP, next_lseq(), 0, 1883, b'hello\0node up')
next_ping = time.monotonic() + (a.ping or 0)
next_up, n_up = time.monotonic() + (a.up or 0), 0
try:
    while True:
        try:
            if need_setup[0]:
                log('board not answering - setting it up again')
                if not setup():
                    time.sleep(2)
                    continue
                need_setup[0] = False
                log('board back')
            radio_step()
        except serial.SerialException as e:
            log('serial error:', e)
            need_setup[0] = True
            time.sleep(1)
            continue
        now = time.monotonic()
        for q, t in list(pings.items()):
            if now - t > PING_LOST_S:
                del pings[q]
                beeps.append((400, 250))
                log(f'   PING seq {q}: no PONG  [{tally[1]}/{tally[0]} pings ok]')
        if beeps and not ackq and not out:
            beep()
        if a.ping and now >= next_ping and not out and not txq:
            q = next_lseq()
            pings[q] = now
            tally[0] += 1
            log(f'-> PING gateway seq {q}')
            lsend(a.gw, T_PING, q, 0, 0)
            next_ping = now + a.ping
        if a.up and now >= next_up:
            n_up += 1
            lsend(a.gw, T_UDP, next_lseq(), 0, 1883, f'count\0{n_up}'.encode())
            log(f'-> MQTT up count={n_up}')
            next_up = now + a.up
except KeyboardInterrupt:
    pass
