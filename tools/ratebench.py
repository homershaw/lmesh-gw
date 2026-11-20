"""Benchmark Modbus/ping latency through the gateway at several E52 air rates.
   python ratebench.py COM4 2 1 0
"""
import json, os, socket, struct, subprocess, sys, time, urllib.request

PORT = sys.argv[1]
RATES = [int(x) for x in sys.argv[2:]] or [2, 1, 0]
GW = '192.168.4.1'
PY = sys.executable
HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = {0: '62.5k', 1: '21.875k', 2: '7k'}


def api(path, body=None, timeout=30):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f'http://{GW}{path}', data=data, method='POST' if data else 'GET',
                                 headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=timeout).read())


def modbus_reads(n=10, unit=2, count=4):
    out = []
    s = socket.create_connection((GW, 502), timeout=15)
    for tid in range(1, n + 1):
        t = time.time()
        s.sendall(struct.pack('>HHHBBHH', tid, 0, 6, unit, 3, 0, count))
        hdr = b''
        while len(hdr) < 7:
            hdr += s.recv(7 - len(hdr))
        ln = struct.unpack('>H', hdr[4:6])[0]
        body = b''
        while len(body) < ln - 1:
            body += s.recv(ln - 1 - len(body))
        dt = (time.time() - t) * 1000
        out.append(None if body[0] & 0x80 else dt)
        time.sleep(0.3)
    s.close()
    return out


def pings(n=5):
    out = []
    for _ in range(n):
        r = api('/api/ping', {'node': 2}, timeout=20)
        out.append(r['rtt'] if r['rtt'] >= 0 else None)
        time.sleep(0.3)
    return out


def stats(xs):
    ok = [x for x in xs if x is not None]
    if not ok:
        return f'0/{len(xs)} ok'
    ok.sort()
    return (f'{len(ok)}/{len(xs)} ok  min {ok[0]:.0f}  med {ok[len(ok) // 2]:.0f}  '
            f'avg {sum(ok) / len(ok):.0f}  max {ok[-1]:.0f} ms')


results = {}
for rate in RATES:
    print(f'\n===== rate {rate} ({NAMES[rate]}) =====', flush=True)
    api('/api/config', {'e52_rate': rate})
    r = api('/api/e52/apply', {}, timeout=60)
    print('gateway apply ok:', r['ok'])
    if not r['ok']:
        print(r['log'])
    p = subprocess.run([PY, '-I', os.path.join(HERE, 'pcnode.py'), PORT, '--program', '--rate', str(rate)],
                       capture_output=True, text=True)
    print('node:', [l for l in p.stdout.splitlines() if 'RATE' in l or 'FREQ' in l])
    log = open(os.path.join(HERE, f'node_rate{rate}.log'), 'w')
    node = subprocess.Popen([PY, '-I', '-u', os.path.join(HERE, 'pcnode.py'), PORT, '--rate', str(rate)],
                            stdout=log, stderr=subprocess.STDOUT)
    try:
        time.sleep(4)                      # let the node announce / mesh settle
        pings(1)                           # warm-up (route discovery)
        mb = modbus_reads()
        pg = pings()
        results[rate] = (stats(mb), stats(pg))
        print('modbus:', results[rate][0])
        print('ping  :', results[rate][1], flush=True)
    finally:
        node.terminate()
        node.wait()
        log.close()

print('\n===== summary =====')
for rate, (m, p) in results.items():
    print(f'rate {rate} {NAMES[rate]:>8}:  modbus {m}\n{"":19}ping   {p}')
