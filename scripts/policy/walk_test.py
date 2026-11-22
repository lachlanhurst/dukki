"""Walk test: robot.enable, stand, walk at (vx, vy, vyaw), stand, disable, relax; recording the stream.
Runs on the robot.

    scp scripts/policy/walk_test.py root@microduck.local:/tmp/
    ssh root@microduck.local python3 /tmp/walk_test.py [vx m/s] [walk seconds] [vy m/s] [vyaw rad/s]

Defaults: vx 0.15, 6 s, vy 0, vyaw 0. robot.move is a notification the deadman (500 ms) expects
refreshed, so it goes out at 50 Hz the whole time: zero while standing, a 1 s ramp up to the command,
the command, a 1 s ramp down. Ctrl-C disables at once. The summary gives each phase's tilt and lean,
and the heading turned and mean turn rate from the gyro (robotd's odometry is not right for Dukki).
"""
import json, math, socket, sys, time

VX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
WALK = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
VY = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
VYAW = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
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


def move(k):
    tell.sendall((json.dumps({"jsonrpc": "2.0", "method": "robot.move",
                              "params": {"vx": VX * k, "vy": VY * k, "vyaw": VYAW * k}}) + "\n").encode())


def share(t):
    """How much of the command applies at t seconds after enable (2.5 s of that is robotd's ramp to home)."""
    t -= 2.5 + STAND_BEFORE
    if t < 0: return 0.0
    if t < RAMP: return t / RAMP
    if t < RAMP + WALK: return 1.0
    if t < 2 * RAMP + WALK: return (2 * RAMP + WALK - t) / RAMP
    return 0.0


total = 2.5 + STAND_BEFORE + 2 * RAMP + WALK + STAND_AFTER
end = time.time() + 1.0
while time.time() < end: pump()
t0 = time.time()
print(f"enable: {call('robot.enable', {'on': True}).get('result')}; vx {VX} vy {VY} m/s, vyaw {VYAW} rad/s "
      f"for {WALK:.0f} s", flush=True)
cmds = []
try:
    nxt = t0
    while time.time() < t0 + total:
        k = share(time.time() - t0); move(k); cmds.append((time.time() - t0, k))
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
    json.dump({"vx": VX, "vy": VY, "vyaw": VYAW, "walk": WALK, "commands": cmds,
               "frames": [(t - t0, p) for t, p in frames]}, open("/tmp/walk_test.json", "w"))

run = [(t - t0, p) for t, p in frames if t0 + 3.0 <= t <= t0 + total]
if not run: sys.exit("no frames")
print(f"{len(run)} frames under the policy; fallen {sum(p['safety']['fallen'] for _, p in run)}")
print(f"{'phase':<10}{'s':>6}{'tilt mean':>10}{'tilt max':>9}{'pitch fwd+':>11}{'roll':>7}{'turned deg':>11}{'rate r/s':>9}")
phases = [("stand", 3.0, 2.5 + STAND_BEFORE), ("walk", 2.5 + STAND_BEFORE + RAMP, 2.5 + STAND_BEFORE + RAMP + WALK),
          ("stand", 2.5 + STAND_BEFORE + 2 * RAMP + WALK, total)]
for name, a, b in phases:
    ps = [(t, p) for t, p in run if a <= t <= b]
    if len(ps) < 2: continue
    g = [p["safety"]["gravity"] for _, p in ps]
    tilt = [math.degrees(math.acos(max(-1, min(1, -x[2])))) for x in g]
    # Heading rate about world vertical: the gyro projected on gravity's up direction.
    turned = sum(-sum(gi * ui for gi, ui in zip(p["imu"]["gyro"], p["safety"]["gravity"])) * 0.02 for _, p in ps)
    print(f"{name:<10}{b - a:6.1f}{sum(tilt)/len(tilt):10.1f}{max(tilt):9.1f}{sum(x[0] for x in g)/len(g):+11.3f}"
          f"{sum(x[1] for x in g)/len(g):+7.3f}{math.degrees(turned):+11.1f}{turned / (b - a):+9.2f}")
lims = {}
for _, p in run:
    for l in p["move"].get("limited_by") or []: lims[l] = lims.get(l, 0) + 1
print("limits (frames):", lims or "none")
