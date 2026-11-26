#!/usr/bin/env python3
"""How still the robot stands after it stops, from a `speed_record.py` recording (section 13).

    python3 stop_summary.py rec.json [rec2.json ...]

A stop is the pad's applied command falling below robotd's 0.05 standing threshold. Per recording:

- falls (limp-fall sequences) and the policy timeline;
- half-second windows standing still, split into the first second after a stop and later: the trunk's
  twist rate (yaw) RMS, how often it is above 0.6 rad/s (shaking), and the leg targets' change per tick;
- stands of 5 s or more: the twist RMS 1 to 5 s after each stop, which separates stops that settle
  from ones that keep shaking;
- standing from 2 s after a stop: the fore-aft (pitch) rate per 1 s window, how many seconds sway above
  0.2 rad/s, and which leg targets swing most (the 2 Hz fore-aft sway at legs 0.85 on a hard floor);
- the stops grouped by what came before them (walking forwards, backwards, turning).
"""
import json
import sys

import numpy as np

NAMES = ["l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle", "neck_pitch", "head_pitch", "head_yaw",
         "head_roll", "mouth", "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle"]
LEGS = [0, 1, 2, 3, 4, 10, 11, 12, 13, 14]
DT = 0.02
THRESHOLD = 0.05  # robotd's standing threshold on the command's magnitude
SHAKE = 0.6       # rad/s of twist RMS over half a second
SWAY = 0.2        # rad/s of pitch RMS over a second


def rms(x, axis=None):
    return np.sqrt(np.mean(np.square(x), axis=axis))


def summarise(path):
    rec = json.load(open(path))
    st = [s for s in rec["state"] if "gyro" in s[1].get("imu", {})]
    t = np.array([s[0] for s in st])
    p = [s[1] for s in st]
    g = np.array([x["imu"]["gyro"] for x in p])
    tg = np.array([x["targets"] for x in p])
    cmd = np.array([x["move"]["applied"] for x in p])
    pol = [str(x["policy"]) for x in p]
    moving = np.linalg.norm(cmd, axis=1) >= THRESHOLD
    standing = np.array([not m and q in ("walk", "stand") for m, q in zip(moving, pol)])
    stops = [i for i in range(1, len(t)) if moving[i - 1] and not moving[i]]
    falls = sum(1 for i in range(1, len(pol)) if pol[i] == "limp_fall" and pol[i - 1] != "limp_fall")
    print(f"== {path}: {t[-1]:.0f} s, {len(stops)} stops, {falls} falls")

    tl, a = [], 0
    for i in range(1, len(pol) + 1):
        if i == len(pol) or pol[i] != pol[a]:
            tl.append(f"{pol[a]} {t[a]:.0f}-{t[i - 1]:.0f}")
            a = i
    print("   timeline:", "; ".join(tl))

    since = np.full(len(t), np.inf)
    last = None
    for i in range(len(t)):
        if moving[i]:
            last = None
        elif last is None and i in set(stops):
            last = t[i]
        if last is not None:
            since[i] = t[i] - last
    dleg = np.r_[np.zeros(1), np.abs(np.diff(tg[:, LEGS], axis=0)).max(1)]
    rows = []
    for t0 in np.arange(t[0], t[-1] - 0.5, 0.5):
        m = (t >= t0) & (t < t0 + 0.5)
        if not standing[m].all() or len(set(np.array(pol)[m])) != 1:
            continue
        rows.append((since[m][0], rms(g[m, 2]), dleg[m].mean()))
    for lo, hi, label in ((0, 1, "first 1 s after a stop"), (1, np.inf, "from 1 s on")):
        r = np.array([x[1:] for x in rows if lo <= x[0] < hi])
        if len(r):
            print(f"   {label:24} {len(r):3d} half-seconds: twist RMS median {np.median(r[:, 0]):.2f} max {r[:, 0].max():.2f}"
                  f" rad/s, shaking {np.mean(r[:, 0] > SHAKE) * 100:3.0f}%, leg target change/tick median {np.median(r[:, 1]):.3f} rad")

    long_stands, by_dir = [], {}
    for i in stops:
        j = i
        while j < len(t) and standing[j]:
            j += 1
        if j == i:
            continue
        c = cmd[max(0, i - 25):i].mean(0)
        kind = "forwards" if c[0] > THRESHOLD else "backwards" if c[0] < -THRESHOLD else "turning or sideways"
        if t[j - 1] - t[i] >= 5:
            w = (t >= t[i] + 1) & (t < t[i] + 5)
            long_stands.append(rms(g[w, 2]))
        if t[j - 1] - t[i] >= 3:
            w = (t >= t[i] + 1) & (t < t[i] + 3)
            by_dir.setdefault(kind, []).append(rms(g[w, 2]))
    if long_stands:
        print(f"   stands of 5 s or more ({len(long_stands)}): twist RMS 1-5 s after the stop {np.round(sorted(long_stands), 2).tolist()}")
    for kind, v in by_dir.items():
        print(f"   after {kind:20} {len(v):2d} stands: twist RMS 1-3 s after, median {np.median(v):.2f} max {max(v):.2f}")

    still = standing & (since >= 2)
    segs, cur = [], []
    for i in np.flatnonzero(still):
        if cur and i != cur[-1] + 1:
            segs.append(cur)
            cur = []
        cur.append(i)
    if cur:
        segs.append(cur)
    per = [rms(g[s[k * 50:(k + 1) * 50], 1]) for s in segs for k in range(len(s) // 50)]
    swings = [np.degrees(tg[s][:, LEGS] - tg[s][:, LEGS].mean(0)) for s in segs if len(s) >= 50]
    if per:
        sd = np.concatenate(swings).std(0)
        top = ", ".join(f"{NAMES[LEGS[k]]} {sd[k]:.1f}" for k in np.argsort(-sd)[:4])
        print(f"   from 2 s after a stop ({still.sum() * DT:.0f} s): fore-aft rate per 1 s median {np.median(per):.2f} rad/s,"
              f" swaying {np.mean(np.array(per) > SWAY) * 100:.0f}% of seconds; biggest leg target swings (sd deg): {top}")


if __name__ == "__main__":
    for path in sys.argv[1:]:
        summarise(path)
