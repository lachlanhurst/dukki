"""Record robotd's trunk gyro and gravity for N seconds (default 20) to /tmp/flip_test.json, then summarise
per second: peak |gyro| per axis, gravity direction and tilt. For the hand flip test of the IMU range."""
import json, math, socket, sys, time
N = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
s = socket.socket(socket.AF_UNIX); s.connect("/run/robotd.sock"); s.settimeout(2)
s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.subscribe", "params": {}}) + "\n").encode())
buf, frames, t0 = b"", [], time.monotonic()
print("recording", N, "s", flush=True)
while time.monotonic() - t0 < N:
    try: buf += s.recv(65536)
    except socket.timeout: continue
    *ls, buf = buf.split(b"\n")
    for l in ls:
        d = json.loads(l)
        if d.get("method") == "robot.state":
            p = d["params"]; frames.append((time.monotonic() - t0, p["imu"]["gyro"], p["safety"]["gravity"], p["imu"].get("quat")))
json.dump(frames, open("/tmp/flip_test.json", "w"))
for sec in range(int(N)):
    f = [x for x in frames if sec <= x[0] < sec + 1]
    if not f: continue
    pk = [max(abs(x[1][i]) for x in f) for i in range(3)]
    g = f[-1][2]; tilt = math.degrees(math.acos(max(-1, min(1, -g[2]))))
    print(f"{sec:3d}s  peak gyro x {pk[0]:5.1f} y {pk[1]:5.1f} z {pk[2]:5.1f} rad/s   gravity at end ({g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f}) tilt {tilt:5.1f} deg")
