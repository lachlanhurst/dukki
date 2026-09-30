"""Convert explore.py logs to BAM's raw log format, for the J288 actuator.

BAM logs are one JSON per recording: testbench metadata plus a list of entries.
This writes the fields the J288 actuator in the bam fork (bam.unitree.j288)
replays and compares against:

    position        rotor-derived position [rad] (what the firmware PD reads)
    speed           true speed, differentiated from position [rad/s]
    reported_speed  the servo's own (filtered) speed [rad/s]
    torque          the servo's reported motor torque [N.m]
    goal_position, goal_speed, goal_torque, kp, kd, torque_enable
    input_volts, winding_temp

Resample the output with BAM before fitting, e.g.
    python -m bam.process --raw <out> --logdir <processed> --dt 0.001

The explore.py bench arm has no known zero angle (it was never hung straight
down), so by default its inertia goes in as ``mass_inertia`` and gravity is left
out; the 6 g arm's gravity torque is under 0.003 N.m.

    uv run --with numpy scripts/j288/to_bam.py scratch/j288_explore/steps_*.json \
        --out scratch/j288_bam/raw --mass-inertia 2.0e-5
"""

import argparse
import json
import os

import numpy as np


def convert(path: str, args) -> dict:
    data = json.load(open(path))
    entries = data["entries"]
    if args.phases:
        entries = [e for e in entries if e["phase"] in args.phases]
    if len(entries) < 3:
        raise ValueError(f"{path}: fewer than 3 entries after filtering")

    t = np.array([e["timestamp"] for e in entries])
    t = t - t[0]
    q = np.array([e["position"] for e in entries])
    speed = np.gradient(q, t)
    foc = [e for e in entries if e["cmd_mode"] == 1]

    out_entries = []
    for e, ti, wi in zip(entries, t, speed):
        enabled = e["cmd_mode"] == 1
        out_entries.append(
            {
                "timestamp": float(ti),
                "position": e["position"],
                "speed": float(wi),
                "reported_speed": e["speed"],
                "torque": e["torque"],
                # With kp = 0 the target position has no effect; keep it at the
                # current position so plots stay readable.
                "goal_position": e["cmd_position"] if e["kp"] > 0 else e["position"],
                "goal_speed": e["cmd_speed"],
                "goal_torque": e["cmd_torque"],
                "kp": e["kp"],
                "kd": e["kd"],
                "torque_enable": enabled,
                "input_volts": e["input_volts"],
                "winding_temp": e["winding_temp"],
            }
        )

    return {
        "motor": "unitree_j288",
        "trajectory": data.get("test", "explore"),
        "source": os.path.basename(path),
        "note": data.get("note", ""),
        "mass": args.mass,
        "arm_mass": args.arm_mass,
        "length": args.length,
        "mass_inertia": args.mass_inertia,
        "kp": foc[0]["kp"] if foc else 0.0,
        "kd": foc[0]["kd"] if foc else 0.0,
        "vin": float(np.mean([e["input_volts"] for e in entries])),
        "entries": out_entries,
    }


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logs", nargs="+", help="explore.py JSON logs")
    p.add_argument("--out", required=True, help="output directory")
    p.add_argument("--phases", nargs="*", help="keep only these phases (default all)")
    p.add_argument("--mass", type=float, default=0.0, help="point mass [kg]")
    p.add_argument("--arm-mass", type=float, default=0.0, help="uniform arm mass [kg]")
    p.add_argument("--length", type=float, default=0.1, help="pivot to mass centre [m]")
    p.add_argument(
        "--mass-inertia", type=float, default=0.0,
        help="extra inertia about the pivot, not subject to gravity [kg.m^2]",
    )
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)
    for path in args.logs:
        log = convert(path, args)
        name = os.path.splitext(os.path.basename(path))[0]
        suffix = "_" + "-".join(args.phases) if args.phases else ""
        out = os.path.join(args.out, f"{name}{suffix}.json")
        json.dump(log, open(out, "w"))
        print(f"{out}: {len(log['entries'])} entries, {log['entries'][-1]['timestamp']:.1f} s")


if __name__ == "__main__":
    main()
