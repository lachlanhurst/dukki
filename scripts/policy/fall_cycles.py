"""Fall and get-up cycles, run on the robot: from a standing robot under the policy, N times: policy off and
torque off (it collapses limp), wait, policy on (robotd ramps to home where it lies, then the stand policy gets it
up), stand. The whole state stream to /tmp/fall_cycles.json, with the command times."""
import json, socket, sys, time
N = int(sys.argv[1]) if len(sys.argv) > 1 else 3
LIE, STAND = 3.0, 10.0
def call(method, params=None):
    s = socket.socket(socket.AF_UNIX); s.connect("/run/robotd.sock"); s.settimeout(3)
    s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}) + "\n").encode())
    buf = b""
    while b"\n" not in buf: buf += s.recv(65536)
    s.close(); return json.loads(buf.split(b"\n")[0]).get("result")
sub = socket.socket(socket.AF_UNIX); sub.connect("/run/robotd.sock"); sub.settimeout(0.5)
sub.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.subscribe", "params": {}}) + "\n").encode())
frames, events, buf, t0 = [], [], b"", time.monotonic()
def pump(secs):
    global buf
    end = time.monotonic() + secs
    while time.monotonic() < end:
        try: buf += sub.recv(65536)
        except socket.timeout: continue
        *ls, buf = buf.split(b"\n")
        for l in ls:
            d = json.loads(l)
            if d.get("method") == "robot.state": frames.append((time.monotonic() - t0, d["params"]))
def event(name, r):
    events.append((time.monotonic() - t0, name)); print(f"{time.monotonic() - t0:6.2f} {name}: {r}", flush=True)
pump(1.0)
for k in range(N):
    event("disable", call("robot.enable", {"on": False}))
    event("relax", call("robot.relax"))
    pump(LIE)
    event("enable", call("robot.enable", {"on": True}))
    pump(STAND)
json.dump({"events": events, "frames": frames}, open("/tmp/fall_cycles.json", "w"))
print("done; the policy is left running")
