"""Record robot.state while the robot is driven by hand (the pad): one frame per line to
/tmp/pad_session.jsonl, with the recorder's monotonic time. Runs on the robot.

    scp scripts/policy/record.py root@microduck.local:/tmp/
    ssh root@microduck.local 'nohup python3 /tmp/record.py 900 >/tmp/record.log 2>&1 & echo $! > /tmp/record.pid'
    ssh root@microduck.local 'kill $(cat /tmp/record.pid)'      # to stop before the time is up

Stops after the given seconds (default 900) or when killed; every line is flushed as it comes.
"""
import json, socket, sys, time

N = float(sys.argv[1]) if len(sys.argv) > 1 else 900
out = open("/tmp/pad_session.jsonl", "w", buffering=1)
s = socket.socket(socket.AF_UNIX); s.connect("/run/robotd.sock"); s.settimeout(1.0)
s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.subscribe", "params": {}}) + "\n").encode())
buf, end = b"", time.monotonic() + N
while time.monotonic() < end:
    try: buf += s.recv(262144)
    except socket.timeout: continue
    *lines, buf = buf.split(b"\n")
    for l in lines:
        if not l.strip(): continue
        d = json.loads(l)
        if d.get("method") == "robot.state":
            out.write(json.dumps({"t": time.monotonic(), "s": d["params"]}) + "\n")
