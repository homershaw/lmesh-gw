"""
Minimal MQTT 3.1.1 broker for bench tests (QoS 0/1, retained ignored, no auth).
Prints every publish and forwards it to matching subscribers.

  python minibroker.py [port]            # default 1883
  python minibroker.py pub TOPIC PAYLOAD # publish one message to a running broker

Point the gateway at it: MQTT URI mqtt://<this PC's IP>:1883, MQTT enabled.
"""
import socket, struct, sys, threading, time

def rd_len(sock):
    mult, val = 1, 0
    while True:
        b = sock.recv(1)
        if not b:
            raise ConnectionError
        val += (b[0] & 0x7F) * mult
        if not b[0] & 0x80:
            return val
        mult *= 128

def enc_len(n):
    out = b''
    while True:
        d, n = n % 128, n // 128
        out += bytes([d | (0x80 if n else 0)])
        if not n:
            return out

def recv_n(sock, n):
    b = b''
    while len(b) < n:
        c = sock.recv(n - len(b))
        if not c:
            raise ConnectionError
        b += c
    return b

def match(flt, topic):
    f, t = flt.split('/'), topic.split('/')
    for i, p in enumerate(f):
        if p == '#':
            return True
        if i >= len(t) or (p != '+' and p != t[i]):
            return False
    return len(f) == len(t)

def publish_pkt(topic, payload):
    tb = topic.encode()
    body = struct.pack('>H', len(tb)) + tb + payload
    return bytes([0x30]) + enc_len(len(body)) + body

subs = {}       # sock -> [filters]
lock = threading.Lock()

def log(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)

def client(sock, addr):
    name = f'{addr[0]}:{addr[1]}'
    try:
        while True:
            h = recv_n(sock, 1)[0]
            body = recv_n(sock, rd_len(sock))
            t = h >> 4
            if t == 1:                       # CONNECT
                cid_len = struct.unpack('>H', body[10:12])[0]
                name = body[12:12 + cid_len].decode(errors='replace') or name
                log(f'CONNECT {name} from {addr[0]}')
                sock.sendall(b'\x20\x02\x00\x00')
            elif t == 3:                     # PUBLISH
                qos = (h >> 1) & 3
                tl = struct.unpack('>H', body[:2])[0]
                topic = body[2:2 + tl].decode(errors='replace')
                p = 2 + tl
                if qos:
                    pid = body[p:p + 2]; p += 2
                    sock.sendall(b'\x40\x02' + pid)
                payload = body[p:]
                log(f'PUB {topic} = {payload.decode(errors="replace")}' + (' [retain]' if h & 1 else ''))
                pkt = publish_pkt(topic, payload)
                with lock:
                    for s, fl in subs.items():
                        if s is not sock and any(match(f, topic) for f in fl):
                            try: s.sendall(pkt)
                            except OSError: pass
            elif t == 8:                     # SUBSCRIBE
                pid, p, rc = body[:2], 2, b''
                while p < len(body):
                    fl = struct.unpack('>H', body[p:p + 2])[0]
                    f = body[p + 2:p + 2 + fl].decode(); p += 2 + fl + 1
                    with lock: subs.setdefault(sock, []).append(f)
                    rc += b'\x00'
                    log(f'SUB {name}: {f}')
                sock.sendall(bytes([0x90]) + enc_len(2 + len(rc)) + pid + rc)
            elif t == 12:                    # PINGREQ
                sock.sendall(b'\xd0\x00')
            elif t == 14:                    # DISCONNECT
                break
    except (ConnectionError, OSError):
        pass
    with lock: subs.pop(sock, None)
    sock.close()
    log(f'gone {name}')

if len(sys.argv) > 1 and sys.argv[1] == 'pub':
    s = socket.create_connection(('127.0.0.1', 1883))
    cid = b'pubtool'
    var = b'\x00\x04MQTT\x04\x02\x00\x3c' + struct.pack('>H', len(cid)) + cid
    s.sendall(bytes([0x10]) + enc_len(len(var)) + var); s.recv(4)
    s.sendall(publish_pkt(sys.argv[2], sys.argv[3].encode())); time.sleep(0.3)
    s.sendall(b'\xe0\x00'); s.close()
    sys.exit(0)

port = int(sys.argv[1]) if len(sys.argv) > 1 else 1883
srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('0.0.0.0', port)); srv.listen(8)
log(f'mini broker on :{port}')
while True:
    c, a = srv.accept()
    threading.Thread(target=client, args=(c, a), daemon=True).start()
