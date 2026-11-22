"""Summarise a stand_test.py or walk_test.py recording: per 5 s, the policy's action change per tick,
the gyro and the worst tilt, how many seconds were restless, and robotd's loop timing.

    uv run --with numpy --with onnxruntime python scripts/policy/stand_summary.py \\
        RECORDING.json POLICY.onnx LEGS_ALPHA HEAD_ALPHA

LEGS_ALPHA and HEAD_ALPHA are robotd's legs_lowpass and head_lowpass for the run: the policy's raw
actions are rebuilt from the filtered targets with them (obs_replay.py), and the replay check
reports zero error only when they are right.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

recording, policy, legs, head = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
here = Path(__file__).parent
env = dict(os.environ, LEGS_ALPHA=legs, HEAD_ALPHA=head)
replay = subprocess.run([sys.executable, str(here / "obs_replay.py"), recording, policy], env=env,
                        capture_output=True, text=True)
print((replay.stdout.splitlines() or [replay.stderr.strip()[-200:]])[0][:110])

R = np.load(recording.replace(".json", ".obs.npz"))
t = R["t"]
seconds = []
for a in range(3, int(t.max())):
    m = R["ok"] & (t >= a) & (t < a + 1)
    if m.sum() < 3:
        continue
    o, act = R["obs"][m], R["act"][m]
    g = o[:, 3:6]
    seconds.append((a, np.abs(np.diff(act, axis=0)).mean(), np.sqrt((o[:, 0:3] ** 2).mean()),
                    np.degrees(np.arccos(np.clip(-g[:, 2], -1, 1))).max()))
for i in range(0, len(seconds), 5):
    blk = seconds[i:i + 5]
    print(f"{blk[0][0]:2d}-{blk[-1][0]:2d}s  action change {np.mean([b[1] for b in blk]):.3f}  "
          f"gyro {np.mean([b[2] for b in blk]):.2f}  max tilt {max(b[3] for b in blk):5.1f}")
print(f"restless seconds (action change > 0.03): {sum(1 for b in seconds if b[1] > 0.03)} of {len(seconds)}; "
      f"worst tilt {max(b[3] for b in seconds):.1f}")

data = json.load(open(recording))
frames = data["frames"] if isinstance(data, dict) else data
timing = [p["loop"]["timing"] for _, p in frames if p.get("policy") == "walk" and p["loop"].get("timing")]
for key in ("servo_age_us", "imu_age_us", "round_trip_us", "compute_us", "command_age_us"):
    if timing:
        v = np.array([x[key] for x in timing]) / 1000
        print(f"  {key:15} median {np.median(v):6.2f} ms  p5 {np.percentile(v, 5):6.2f}  "
              f"p95 {np.percentile(v, 95):6.2f}  max {v.max():6.2f}")
