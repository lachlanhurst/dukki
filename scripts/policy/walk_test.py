"""Walk test: robot.enable, stand, walk forward at vx, stand, disable, relax; recording the stream. Runs on the robot.

    scp scripts/policy/walk_test.py root@microduck.local:/tmp/
    ssh root@microduck.local python3 /tmp/walk_test.py [vx m/s, default 0.15] [walk seconds, default 6]

robot.move is a notification the deadman (500 ms) expects refreshed, so it goes out at 50 Hz the whole
time: zero while standing, a 1 s ramp up to vx, vx, a 1 s ramp down. Ctrl-C disables at once.
"""
import json, math, socket, sys, time

VX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
WALK = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
STAND_BEFORE, RAMP, STAND_AFTER = 4.0, 1.0, 4.0
SOCK = "/run/robotd.sock"


def call(method, params=None):
    s = socket.socket(socket.AF_UNIX); s.connect(SOCK); s.settimeout(3)
    s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}) + "\n").encode())
    buf = b""
    while b"\n" not in buf: buf += s.recv(65536)
    s.close(); return json.loads(buf.split(b"\n")[0])


sub = socket.socket(socket.AF_UNIX); sub.connect(SOCK); sub.settimeout(0.005)
sub.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.subscribe", "params": {}}) + "\n").encode())
tell = socket.socket(socket.AF_UNIX); tell.connect(SOCK)
frames, buf = [], b""


def pump():
    global buf
    try: buf += sub.recv(262144)
    except (socket.timeout, BlockingIOError): return
    *lines, buf = buf.split(b"\n")
    for l in lines:
        if not l.strip(): continue
        d = json.loads(l)
        if d.get("method") == "robot.state": frames.append((time.time(), d["params"]))


def move(vx):
    tell.sendall((json.dumps({"jsonrpc": "2.0", "method": "robot.move", "params": {"vx": vx, "vy": 0.0, "vyaw": 0.0}}) + "\n").encode())


def command(t):
    """vx at t seconds after enable (2.5 s of that is robotd's ramp to home)."""
    t -= 2.5 + STAND_BEFORE
    if t < 0: return 0.0
    if t < RAMP: return VX * t / RAMP
    if t < RAMP + WALK: return VX
    if t < 2 * RAMP + WALK: return VX * (2 * RAMP + WALK - t) / RAMP
    return 0.0


total = 2.5 + STAND_BEFORE + 2 * RAMP + WALK + STAND_AFTER
end = time.time() + 1.0
while time.time() < end: pump()
t0 = time.time()
print(f"enable: {call('robot.enable', {'on': True}).get('result')}; vx {VX} m/s for {WALK:.0f} s", flush=True)
cmds = []
try:
    nxt = t0
    while time.time() < t0 + total:
        vx = command(time.time() - t0); move(vx); cmds.append((time.time() - t0, vx))
        nxt += 0.02
        while time.time() < nxt: pump()
finally:
    move(0.0)
    print("disable:", call("robot.enable", {"on": False}).get("result"), flush=True)
    end = time.time() + 1.0
    while time.time() < end: pump()
    print("relax:", call("robot.relax").get("result"), flush=True)
    end = time.time() + 1.0
    while time.time() < end: pump()
    json.dump({"vx": VX, "walk": WALK, "commands": cmds, "frames": [(t - t0, p) for t, p in frames]},
              open("/tmp/walk_test.json", "w"))

run = [(t - t0, p) for t, p in frames if t0 + 3.0 <= t <= t0 + total]
if not run: sys.exit("no frames")
print(f"{len(run)} frames under the policy; fallen {sum(p['safety']['fallen'] for _, p in run)}")
print(f"{'phase':<10}{'s':>6}{'tilt mean':>10}{'tilt max':>9}{'pitch fwd+':>11}{'roll':>7}{'applied vx':>10}")
phases = [("stand", 3.0, 2.5 + STAND_BEFORE), ("walk", 2.5 + STAND_BEFORE + RAMP, 2.5 + STAND_BEFORE + RAMP + WALK),
          ("stand", 2.5 + STAND_BEFORE + 2 * RAMP + WALK, total)]
for name, a, b in phases:
    ps = [p for t, p in run if a <= t <= b]
    if not ps: continue
    g = [p["safety"]["gravity"] for p in ps]
    tilt = [math.degrees(math.acos(max(-1, min(1, -x[2])))) for x in g]
    vx = sum(p["move"]["applied"][0] for p in ps) / len(ps)
    od = [p["odom"]["position"] for p in (ps[0], ps[-1])]
    print(f"{name:<10}{b - a:6.1f}{sum(tilt)/len(tilt):10.1f}{max(tilt):9.1f}{sum(x[0] for x in g)/len(g):+11.3f}"
          f"{sum(x[1] for x in g)/len(g):+7.3f}{vx:+10.3f}  odom dx {od[1][0]-od[0][0]:+.2f} dy {od[1][1]-od[0][1]:+.2f}")
lims = {}
for _, p in run:
    for l in p["move"].get("limited_by") or []: lims[l] = lims.get(l, 0) + 1
print("limits (frames):", lims or "none")
