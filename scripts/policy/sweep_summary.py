#!/usr/bin/env python3
"""Left against right from a joint_sweep recording (microduck `duck-control/examples/joint_sweep`).

For each side and frequency, over the full-amplitude cycles (the first and last, faded, left out):

- how the joint follows its target: amplitude ratio and lag, and the RMS tracking error;
- what holds it back, from the servo's own torque fitted as  tau = Fc sign(w) + b w + J a + c:
  Coulomb friction Fc, viscous b, inertia J (the swinging leg's), offset c;
- the output-shaft encoder's swing against the rotor's: play shows as the output turning less than
  the rotor (the encoder is only trustworthy at rest, section 17 of j288-testing.md, so this is a
  comparison between the two sides more than a measurement).

    python3 scripts/policy/sweep_summary.py sweep_hip_yaw.json
"""
from __future__ import annotations

import json
import math
import sys

import numpy as np

NAMES = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch",
         "head_pitch", "head_yaw", "head_roll", "mouth", "right_hip_yaw", "right_hip_roll",
         "right_hip_pitch", "right_knee", "right_ankle"]
COUNTS = 8192


def sine_fit(t: np.ndarray, y: np.ndarray, f: float) -> tuple[float, float]:
    """Amplitude and phase (rad) of y's component at f."""
    w = 2 * math.pi * f
    A = np.stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)], 1)
    (s, c, _), *_ = np.linalg.lstsq(A, y, rcond=None)
    return math.hypot(s, c), math.atan2(c, s)


def main() -> None:
    rec = json.load(open(sys.argv[1]))
    joint = rec["joint"]
    print(f"{joint}: ±{rec['amp_deg']}°, kp {rec['kp']:.2f} kd {rec['kd']:.3f}")
    print(f"{'side':6}{'Hz':>5}{'follows':>9}{'lag°':>7}{'err rms°':>10}{'Fc N.m':>9}{'b N.m.s':>9}"
          f"{'J g.m²':>9}{'c N.m':>8}{'fit R²':>8}{'output/rotor':>13}")
    for f in rec["freqs"]:
        for side in ("left", "right"):
            # The tool labels with Rust's float formatting: 2.0 prints as "2".
            label = f"{side} {int(f) if float(f).is_integer() else f} Hz"
            fr = [x for x in rec["frames"] if x["phase"] == label]
            if not fr:
                continue
            j = NAMES.index(f"{side}_{joint}")
            t = np.array([x["t"] for x in fr])
            t = t - t[0]
            keep = (t * f >= 1.0) & (t * f <= rec["cycles"] + 1.0)
            t = t[keep]
            q = np.array([x["positions"][j] for x in fr])[keep]
            tg = np.array([x["targets"][j] for x in fr])[keep]
            tau = np.array([x["torques"][j] for x in fr])[keep]
            enc = np.unwrap(np.array([x["encoders"][j] for x in fr], float)[keep] * 2 * math.pi / COUNTS)
            a_t, p_t = sine_fit(t, tg, f)
            a_q, p_q = sine_fit(t, q, f)
            a_e, _ = sine_fit(t, enc, f)
            # Speed and acceleration from the position's own fit plus harmonics: smoother than
            # differencing a 50 Hz record.
            w = 2 * math.pi * f
            H = np.concatenate([np.stack([np.sin(k * w * t), np.cos(k * w * t)], 1) for k in (1, 2, 3)] +
                               [np.ones((len(t), 1))], 1)
            coef, *_ = np.linalg.lstsq(H, q, rcond=None)
            dH = np.concatenate([np.stack([k * w * np.cos(k * w * t), -k * w * np.sin(k * w * t)], 1) for k in (1, 2, 3)] +
                                [np.zeros((len(t), 1))], 1)
            ddH = np.concatenate([np.stack([-(k * w) ** 2 * np.sin(k * w * t), -(k * w) ** 2 * np.cos(k * w * t)], 1)
                                  for k in (1, 2, 3)] + [np.zeros((len(t), 1))], 1)
            vel, acc = dH @ coef, ddH @ coef
            X = np.stack([np.sign(vel), vel, acc, np.ones_like(t)], 1)
            (fc, b, J, c), *_ = np.linalg.lstsq(X, tau, rcond=None)
            r2 = 1 - np.var(tau - X @ np.array([fc, b, J, c])) / max(np.var(tau), 1e-12)
            lag = math.degrees((p_t - p_q + math.pi) % (2 * math.pi) - math.pi)
            err = math.degrees(float(np.sqrt(np.mean((tg - q) ** 2))))
            print(f"{side:6}{f:5.1f}{a_q / a_t:9.2f}{lag:7.1f}{err:10.2f}{fc:9.3f}{b:9.4f}{J * 1000:9.2f}{c:8.3f}"
                  f"{r2:8.2f}{a_e / a_q:13.2f}")


if __name__ == "__main__":
    main()
