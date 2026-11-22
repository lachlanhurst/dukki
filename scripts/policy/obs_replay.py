"""Rebuild the walk policy's 61-D observations from a robotd state recording, and check them by
running the ONNX on them: if they are what robotd fed it, its outputs match the actions recovered
from the recorded (filtered) targets.

    LEGS_ALPHA=0.7 HEAD_ALPHA=0.5 uv run --with onnxruntime --with numpy python scripts/policy/obs_replay.py RECORDING.json POLICY.onnx

Writes <recording>.obs.npz: t, obs (N x 61), act (N x 14, recovered), onnx (N x 14), policy labels.
"""
import json, sys
import numpy as np
import onnxruntime as ort

DEFAULT = np.array([0.0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0.0, 0.0, 0.0,
                    0.0, 0.0873, 0.4579, 0.0049, -0.4530])
MOUTH = 9
POL = [j for j in range(15) if j != MOUTH]
import os
ALPHA = np.array([float(os.environ.get("HEAD_ALPHA", 0.5)) if 5 <= j < 9 else float(os.environ.get("LEGS_ALPHA", 0.7)) for j in POL])  # robotd head_lowpass, legs_lowpass

rec = json.load(open(sys.argv[1]))
frames = rec["frames"] if isinstance(rec, dict) else rec
sess = ort.InferenceSession(sys.argv[2])
inp = sess.get_inputs()[0].name

t = np.array([f[0] for f in frames])
pol = [f[1]["policy"] for f in frames]
q = np.array([f[1]["joints"] for f in frames])[:, POL]
qd = np.array([f[1]["velocities"] for f in frames])[:, POL]
tg = np.array([f[1]["targets"] for f in frames])[:, POL]
gyro = np.array([f[1]["imu"]["gyro"] for f in frames])
grav = np.array([f[1]["safety"]["gravity"] for f in frames])
cmd = np.array([f[1]["move"]["applied"] for f in frames])
head = np.array([f[1].get("head", [0, 0, 0, 0]) for f in frames])
d = DEFAULT[POL]

# Recovered raw action: filtered_t = a(d + act_t) + (1 - a) filtered_{t-1}; on the first walk tick
# after another controller there is no filter state, so only consecutive walk frames count.
act = np.full_like(tg, np.nan)
for i in range(1, len(frames)):
    if pol[i] == "walk" and pol[i - 1] == "walk":
        act[i] = (tg[i] - (1 - ALPHA) * tg[i - 1]) / ALPHA - d

obs = np.zeros((len(frames), 61), np.float32)
obs[:, 0:3] = gyro
obs[:, 3:6] = grav
obs[:, 6:20] = q - d
obs[:, 20:34] = qd
obs[1:, 34:48] = np.nan_to_num(act[:-1])
obs[:, 48:51] = cmd
obs[:, 51:55] = head
ok = np.array([i > 1 and pol[i] == "walk" and pol[i - 1] == "walk" and pol[i - 2] == "walk" for i in range(len(frames))])
out = np.full_like(act, np.nan)
out[ok] = np.concatenate([sess.run(None, {inp: obs[i:i + 1]})[0] for i in np.flatnonzero(ok)])
np.savez(sys.argv[1].replace(".json", ".obs.npz"), t=t, obs=obs, act=act, onnx=out, ok=ok, policy=np.array(pol))

e = np.abs(out[ok] - act[ok])
print(f"{ok.sum()} walk frames; |onnx - recovered action|: median {np.median(e):.4f} rad, mean {e.mean():.4f}, "
      f"95% {np.percentile(e, 95):.4f}; |action| mean {np.abs(act[ok]).mean():.4f}")
names = ["l_hyaw", "l_hroll", "l_hpitch", "l_knee", "l_ankle", "neck", "hpitch", "hyaw", "hroll",
         "r_hyaw", "r_hroll", "r_hpitch", "r_knee", "r_ankle"]
print("per joint median err:", " ".join(f"{n} {np.median(e[:, k]):.3f}" for k, n in enumerate(names)))
