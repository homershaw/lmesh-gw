import socket, struct, sys, time
host = sys.argv[1] if len(sys.argv) > 1 else '192.168.4.1'
unit = int(sys.argv[2]) if len(sys.argv) > 2 else 2
n = int(sys.argv[3]) if len(sys.argv) > 3 else 5
count = int(sys.argv[4]) if len(sys.argv) > 4 else 4
s = socket.create_connection((host, 502), timeout=15)
times = []
for tid in range(1, n + 1):
    req = struct.pack('>HHHBBHH', tid, 0, 6, unit, 3, 0, count)
    t = time.time(); s.sendall(req)
    hdr = s.recv(7)
    _, _, ln, u = struct.unpack('>HHHB', hdr)
    body = b''
    while len(body) < ln - 1: body += s.recv(ln - 1 - len(body))
    dt = (time.time() - t) * 1000
    if body[0] & 0x80: print(f'#{tid} exception 0x{body[1]:02X} after {dt:.0f} ms'); continue
    vals = struct.unpack(f'>{body[1] // 2}H', body[2:])
    times.append(dt); print(f'#{tid} unit {u} regs {list(vals)}  {dt:.0f} ms')
if times: print(f'ok {len(times)}/{n}  min {min(times):.0f}  avg {sum(times)/len(times):.0f}  max {max(times):.0f} ms')
