"""A stand test, run on the robot: robot.enable (the walk policy at zero velocity, after robotd's
2.5 s ramp to home), N seconds (default 15), disable, relax; the state stream recorded throughout
to /tmp/stand_test.json, and a summary printed.

    scp scripts/policy/stand_test.py root@microduck.local:/tmp/ && ssh root@microduck.local python3 /tmp/stand_test.py 60
"""
import json, math, socket, sys, time
NAMES = ["left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee","left_ankle","neck_pitch","head_pitch",
         "head_yaw","head_roll","mouth","right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee","right_ankle"]
RUN = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
def call(method, params=None):
    s = socket.socket(socket.AF_UNIX); s.connect("/run/robotd.sock"); s.settimeout(3)
    s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}) + "\n").encode())
    buf = b""
    while b"\n" not in buf: buf += s.recv(65536)
    s.close(); return json.loads(buf.split(b"\n")[0])
sub = socket.socket(socket.AF_UNIX); sub.connect("/run/robotd.sock"); sub.settimeout(1)
sub.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.subscribe", "params": {}}) + "\n").encode())
frames, buf = [], b""
def pump(until):
    global buf
    while time.time() < until:
        try: buf += sub.recv(65536)
        except socket.timeout: continue
        *lines, buf = buf.split(b"\n")
        for l in lines:
            d = json.loads(l)
            if d.get("method") == "robot.state": frames.append((time.time(), d["params"]))
pump(time.time() + 1.0)
t0 = time.time(); print("enable:", call("robot.enable", {"on": True}).get("result"), flush=True)
pump(t0 + 2.5 + RUN)
print("disable:", call("robot.enable", {"on": False}).get("result"), flush=True)
pump(time.time() + 1.0)
print("relax:", call("robot.relax").get("result"), flush=True)
pump(time.time() + 1.0)
json.dump([(t - t0, p) for t, p in frames], open("/tmp/stand_test.json", "w"))
run = [p for t, p in frames if t0 + 3.0 <= t <= t0 + 2.5 + RUN]
if not run: sys.exit("no frames")
pol = {}
for p in run: pol[p.get("policy")] = pol.get(p.get("policy"), 0) + 1
tilt = [math.degrees(math.acos(max(-1, min(1, -p["safety"]["gravity"][2])))) for p in run]
gx = [p["safety"]["gravity"][0] for p in run]; gy = [p["safety"]["gravity"][1] for p in run]
print(f"{len(run)} frames under the policy; policy field {pol}; fallen {sum(p['safety']['fallen'] for p in run)}; gain {run[-1]['safety'].get('gain')}")
print(f"tilt from upright: mean {sum(tilt)/len(tilt):.1f} deg, max {max(tilt):.1f}; gravity x {sum(gx)/len(gx):+.3f} (fwd +), y {sum(gy)/len(gy):+.3f}")
print(f"{'joint':<16}{'home':>7}{'mean q':>8}{'q range':>9}{'|q-tgt|':>8}{'load mA':>8}{'peak':>6}")
home = [0,-5.0,-26.2,-0.3,26.0,20,20,0,0,0,0,5.0,26.2,0.3,-26.0]
for j, n in enumerate(NAMES):
    q = [math.degrees(p["joints"][j]) for p in run]; e = [abs(math.degrees(p["joints"][j] - p["targets"][j])) for p in run]
    c = [p.get("currents_ma", [0]*15)[j] for p in run]
    print(f"{n:<16}{home[j]:7.1f}{sum(q)/len(q):8.1f}{max(q)-min(q):9.1f}{sum(e)/len(e):8.2f}{sum(c)/len(c):8.0f}{max(c):6.0f}")
