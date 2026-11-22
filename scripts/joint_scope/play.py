"""Gear play in each J288, from the gap between its rotor position and its output encoder.

Reads joint_scope's stream (microduck `duck-control/examples/joint_scope`, every servo limp). The
J288's multi-turn position comes from its rotor encoder, through the gearbox; the 13-bit encoder
is on the output shaft. Rock a joint by hand and the output leads the rotor by the play, one way
then the other, so `encoder - position` steps between two values as the direction reverses.

Per joint, over samples moving slowly enough to trust the encoder, a least-squares fit of

    encoder - position = a + b * angle + (play / 2) * sign(velocity)

gives the play, and `b` any steady drift with angle (a gearbox ratio that is slightly off), kept
apart so it is not counted as play. A still recording first gives each encoder's noise floor.

This is the play inside the servo only. Play between the output shaft and the limb (the horn,
bearings, linkages) moves neither encoder.

    uv run --with numpy python scripts/joint_scope/play.py --still 3 --seconds 90 --out scratch/play
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from joint_scope import COUNTS, JOINT_NAMES, TWO_PI  # noqa: E402

MOVING = (0.03, 0.6)  # rad/s: slow enough to trust the encoder, fast enough to know the direction


def record(host: str, port: int, seconds: float) -> list[dict]:
    frames, buf = [], b""
    with socket.create_connection((host, port), timeout=3) as sock:
        sock.settimeout(1.0)
        while b"\n" not in buf:
            buf += sock.recv(65536)
        buf = buf.split(b"\n", 1)[1]
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            buf += sock.recv(65536)
            *lines, buf = buf.split(b"\n")
            frames += [json.loads(line) for line in lines if line.strip()]
    return frames


def gap(frames: list[dict], j: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(encoder - position, unwrapped near zero; position; velocity) for joint j."""
    pos = np.array([f["servos"][j]["pos"] for f in frames])
    enc = np.array([f["servos"][j]["enc"] for f in frames]) * TWO_PI / COUNTS
    vel = np.array([f["servos"][j]["vel"] for f in frames])
    d = np.angle(np.exp(1j * (enc - pos)))
    d = np.angle(np.exp(1j * (d - np.angle(np.exp(1j * d).mean())))) + np.angle(np.exp(1j * d).mean())
    return d, pos, vel


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="microduck.local")
    p.add_argument("--port", type=int, default=7788)
    p.add_argument("--still", type=float, default=3.0, help="seconds of the robot untouched first")
    p.add_argument("--seconds", type=float, default=90.0, help="seconds of rocking the joints")
    p.add_argument("--out", type=Path, default=Path("scratch/play"))
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    print(f"recording {args.still:.0f} s still: hands off", flush=True)
    still = record(args.host, args.port, args.still)
    print(f"now rock every joint, slowly, a few times each way, for {args.seconds:.0f} s", flush=True)
    rock = record(args.host, args.port, args.seconds)
    (args.out / "frames.json").write_text(json.dumps({"still": still, "rock": rock}))

    rows = []
    print(f"{'joint':<16}{'noise deg':>10}{'samples':>9}{'play deg':>10}{'+-':>6}{'drift deg/rad':>14}{'travel deg':>11}")
    for j, name in enumerate(JOINT_NAMES):
        if rock[0]["servos"][j].get("seg") is None:
            continue
        ds, _, _ = gap(still, j)
        noise = math.degrees(np.percentile(ds, 97.5) - np.percentile(ds, 2.5))
        d, pos, vel = gap(rock, j)
        m = (np.abs(vel) > MOVING[0]) & (np.abs(vel) < MOVING[1])
        travel = math.degrees(pos.max() - pos.min())
        if m.sum() < 40 or travel < 4.0:
            rows.append({"joint": name, "noise_deg": noise, "samples": int(m.sum()), "travel_deg": travel})
            print(f"{name:<16}{noise:10.2f}{int(m.sum()):9d}{'-':>10}{'':>6}{'':>14}{travel:11.1f}   not rocked enough")
            continue
        x = np.column_stack([np.ones(m.sum()), pos[m] - pos[m].mean(), np.sign(vel[m])])
        coef, res, *_ = np.linalg.lstsq(x, d[m], rcond=None)
        resid = d[m] - x @ coef
        cov = np.linalg.inv(x.T @ x) * resid.var()
        play, play_sd = 2 * coef[2], 2 * math.sqrt(cov[2, 2])
        rows.append({"joint": name, "noise_deg": noise, "samples": int(m.sum()), "play_deg": math.degrees(play),
                     "play_sd_deg": math.degrees(play_sd), "drift_deg_per_rad": math.degrees(coef[1]), "travel_deg": travel})
        print(f"{name:<16}{noise:10.2f}{int(m.sum()):9d}{math.degrees(play):10.2f}{math.degrees(play_sd):6.2f}"
              f"{math.degrees(coef[1]):14.2f}{travel:11.1f}")
    (args.out / "play.json").write_text(json.dumps(rows, indent=1))
    print(f"wrote {args.out / 'play.json'}")


if __name__ == "__main__":
    main()
