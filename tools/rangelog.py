"""
Summarise a range-walk log written by wionode.py --log.

  python rangelog.py range.log                          # whole log, one row per minute
  python rangelog.py range.log --every 30               # 30 s rows
  python rangelog.py range.log --from 17:43:30 --to 17:48:30 --every 0   # one window, no rows

Signal figures are the gateway's frames (acks and PONGs) as heard at the node.
A ping counts as answered when its PONG came back; "first try" is the share of
the node's frames the gateway acked on the first transmission.
"""
import argparse, re, statistics

ap = argparse.ArgumentParser()
ap.add_argument('log')
ap.add_argument('--from', dest='t_from', default='00:00:00')
ap.add_argument('--to', dest='t_to', default='99:99:99')
ap.add_argument('--every', type=int, default=60, help='row length in seconds, 0 = summary only')
a = ap.parse_args()

R_SIG = re.compile(r'RSSI (-?\d+) SNR (-?\d+)')
R_TRY = re.compile(r'ack from 0x[0-9A-F]+ seq \d+: try (\d+)')
R_RTT = re.compile(r'PONG from .*round trip (\d+) ms')
R_POW = re.compile(r'POW:(-?\d+)dBm')


def secs(hms):
    h, m, s = hms.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


class Bin:
    def __init__(self):
        self.pings = self.ok = self.lost = 0
        self.tries, self.rssi, self.snr, self.rtt = [], [], [], []

    def add(self, line):
        if '-> PING gateway' in line:
            self.pings += 1
        elif 'no PONG' in line:
            self.lost += 1
        m = R_RTT.search(line)
        if m:
            self.ok += 1
            self.rtt.append(int(m.group(1)))
        m = R_TRY.search(line)
        if m:
            self.tries.append(int(m.group(1)))
        if 'no ack from' in line:
            self.tries.append(99)
        if ('ack from' in line or '<- 0x' in line) and (m := R_SIG.search(line)):
            self.rssi.append(int(m.group(1)))
            self.snr.append(int(m.group(2)))

    def row(self, label):
        done = self.ok + self.lost
        pct = f'{100 * self.ok / done:5.1f}%' if done else '    - '
        first = f'{100 * self.tries.count(1) / len(self.tries):5.1f}%' if self.tries else '    - '
        sig = (f'{min(self.rssi):5d} {statistics.median(self.rssi):6.0f} {max(self.rssi):5d}  {statistics.median(self.snr):4.0f}'
               if self.rssi else '    -      -     -     -')
        rtt = f'{statistics.median(self.rtt):5.0f}' if self.rtt else '    -'
        return f'{label:19s} {self.pings:5d} {self.ok:5d} {pct}  {first}  {sig}  {rtt}'


HEAD = f'{"time":19s} {"pings":>5s} {"ok":>5s} {"ok %":>6s}  {"1st try":>6s}  {"RSSI min  med   max":>18s}  {"SNR":>4s}  {"RTT":>5s}'
lo, hi = secs(a.t_from), secs(a.t_to)
total, rows, cur, cur_start, powers = Bin(), [], None, None, []
for line in open(a.log, encoding='utf-8', errors='replace'):
    if len(line) < 24 or line[10] != 'T':
        continue
    t = secs(line[11:23])
    if not lo <= t <= hi:
        continue
    if 'AT+TEST=RFCFG' in line and (m := R_POW.search(line)):
        powers.append((line[11:19], int(m.group(1))))
    total.add(line)
    if a.every:
        start = t - t % a.every
        if cur is None or start != cur_start:
            if cur:
                rows.append((cur_start, cur))
            cur, cur_start = Bin(), start
        cur.add(line)
if cur:
    rows.append((cur_start, cur))

print(HEAD)
for start, b in rows:
    if b.pings or b.ok or b.lost or b.rssi:
        h, m, s = int(start // 3600), int(start % 3600 // 60), int(start % 60)
        print(b.row(f'{h:02d}:{m:02d}:{s:02d}'))
print(total.row('TOTAL'))
if powers:
    print('node TX power set:', ', '.join(f'{p} dBm at {t}' for t, p in powers))
