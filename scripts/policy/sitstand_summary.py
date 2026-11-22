"""Each sit and rise in a record.py recording: how long, the trunk's pitch and roll range (forward
and right positive), the peak body rate, frames robotd called fallen or limp, and how long the body
kept moving (gyro above 0.3 rad/s). A rise is judged over its first 3 s, through the handover to the
walk policy at policy.sitstand_rise_s.

    scp root@microduck.local:/tmp/pad_session.jsonl .
    python3 scripts/policy/sitstand_summary.py pad_session.jsonl
"""
import json, math, sys

F = [json.loads(l) for l in open(sys.argv[1] if len(sys.argv) > 1 else "pad_session.jsonl")]
t0 = F[0]["t"]
segs, prev = [], None
for f in F:
    if f["s"]["policy"] != prev:
        segs.append((f["s"]["policy"], f["t"] - t0)); prev = f["s"]["policy"]
print(f"{(F[-1]['t'] - t0):.0f} s: " + " ".join(f"{p}@{t:.1f}" for p, t in segs))


def angle(f, i):
    g = f["s"]["safety"]["gravity"]
    return math.degrees(math.atan2(g[i], -g[2]))


for i, (p, t) in enumerate(segs):
    if p not in ("sit", "rise"): continue
    end = segs[i + 1][1] if i + 1 < len(segs) else F[-1]["t"] - t0
    fr = [f for f in F if t <= f["t"] - t0 <= (t + 3.0 if p == "rise" else end)]
    pitch, roll = [angle(f, 0) for f in fr], [angle(f, 1) for f in fr]
    rate = [math.hypot(*f["s"]["imu"]["gyro"]) for f in fr]
    moving = max([f["t"] - t0 for f, r in zip(fr, rate) if r > 0.3], default=t) - t
    print(f"{p:<5}{t:7.1f} for {end - t:5.1f} s  pitch {min(pitch):+4.0f} to {max(pitch):+4.0f}  "
          f"roll {min(roll):+4.0f} to {max(roll):+4.0f}  peak rate {max(rate):4.1f} rad/s  "
          f"fallen {sum(f['s']['safety']['fallen'] for f in fr):3d}  limp {sum(f['s']['safety']['limp'] for f in fr):3d}  "
          f"moving {moving:.1f} s")
