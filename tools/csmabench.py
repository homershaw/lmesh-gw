"""Ping RTT vs E52 CSMA_RNG.  python csmabench.py COM4 127 60 20"""
import base64, json, os, subprocess, sys, time, urllib.request

PORT, VALS = sys.argv[1], [int(x) for x in sys.argv[2:]]
PY, HERE = sys.executable, os.path.dirname(os.path.abspath(__file__))
AUTH = 'Basic ' + base64.b64encode(os.environ.get('LMGW_AUTH', 'admin:lmesh1234').encode()).decode()


def api(path, body, timeout=30):
    req = urllib.request.Request('http://192.168.4.1' + path, json.dumps(body).encode(),
                                 {'Content-Type': 'application/json', 'Authorization': AUTH})
    return json.loads(urllib.request.urlopen(req, timeout=timeout).read())


summary = []
for v in VALS:
    print(f'\n== CSMA_RNG {v} ms', flush=True)
    print('gw:', api('/api/e52/at', {'cmd': f'AT+CSMA_RNG={v}'})['resp'].strip(),
          '|', api('/api/e52/at', {'cmd': 'AT+CSMA_RNG=?'})['resp'].strip())
    log = open(os.path.join(HERE, f'node_csma{v}.log'), 'w')
    node = subprocess.Popen([PY, '-I', '-u', os.path.join(HERE, 'pcnode.py'), PORT, '--rate', '1', '--csma', str(v)],
                            stdout=log, stderr=subprocess.STDOUT)
    try:
        time.sleep(4)
        api('/api/ping', {'node': 2}, 20)          # warm-up
        rtts = []
        for _ in range(20):
            rtts.append(api('/api/ping', {'node': 2}, 20)['rtt'])
            time.sleep(0.2)
    finally:
        node.terminate(); node.wait(); log.close()
    ok = sorted(r for r in rtts if r >= 0)
    line = (f'CSMA {v:3d}: {len(ok)}/{len(rtts)} ok  min {ok[0]}  med {ok[len(ok)//2]}  '
            f'avg {sum(ok)//len(ok)}  p90 {ok[int(len(ok)*0.9)-1]}  max {ok[-1]} ms') if ok else f'CSMA {v}: all lost'
    print(line, flush=True)
    summary.append(line)

print('\n' + '\n'.join(summary))
