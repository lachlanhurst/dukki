"""Record robotd's state and the head ToF while someone drives the robot with the pad. Runs on the robot.

    python3 speed_record.py [seconds=90] [out=/var/tmp/speed.json]

Every message also goes to <out>.part as a JSON line, flushed every 10 s, so a power cut keeps what
was recorded.

Writes {"state": [[t, robot.state params], ...], "tof": [[t, at_us, distance_mm, status], ...]} with t in
seconds from the start, on the Mac's side analysed by speed_summary.py: walking at a wall, the ToF zones
that see the wall close at the robot's speed.
"""
import json, select, signal, socket, sys, time

RUN = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/var/tmp/speed.json"


def subscribe(path, method):
    s = socket.socket(socket.AF_UNIX)
    s.connect(path)
    s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": {}}) + "\n").encode())
    s.setblocking(False)
    return s


socks = {subscribe("/run/robotd.sock", "robot.subscribe"): "state", subscribe("/run/tofd/tof.sock", "tof.stream"): "tof"}
bufs = {s: b"" for s in socks}
rec = {"state": [], "tof": []}
stop = False


def finish(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, finish)
signal.signal(signal.SIGTERM, finish)
part = open(OUT + ".part", "w")  # one JSON line per message, flushed every 10 s
t0 = time.monotonic()
print(f"recording {RUN:.0f} s to {OUT}", flush=True)
next_note = 10.0
while not stop and time.monotonic() - t0 < RUN:
    ready, _, _ = select.select(list(socks), [], [], 0.5)
    now = time.monotonic() - t0
    for s in ready:
        chunk = s.recv(1 << 20)
        if not chunk:
            sys.exit(f"{socks[s]} stream closed")
        bufs[s] += chunk
        *lines, bufs[s] = bufs[s].split(b"\n")
        for line in lines:
            if not line:
                continue
            d = json.loads(line)
            if d.get("method") == "robot.state":
                rec["state"].append([round(now, 4), d["params"]])
                part.write(json.dumps(["state", round(now, 4), d["params"]]) + "\n")
            elif d.get("method") == "tof.frame":
                p = d["params"]
                rec["tof"].append([round(now, 4), p["at_us"], p["distance_mm"], p["status"]])
                part.write(json.dumps(["tof", round(now, 4), p["at_us"], p["distance_mm"], p["status"]]) + "\n")
    if now >= next_note:
        # Saved as it goes, so a power cut or reboot keeps what was recorded (use a path on disk,
        # /var/tmp, not /tmp, which a reboot clears).
        part.flush()
        print(f"{now:.0f} s: {len(rec['state'])} states, {len(rec['tof'])} ToF frames", flush=True)
        next_note += 10.0
json.dump(rec, open(OUT, "w"))
print(f"wrote {OUT}: {len(rec['state'])} states, {len(rec['tof'])} ToF frames", flush=True)
