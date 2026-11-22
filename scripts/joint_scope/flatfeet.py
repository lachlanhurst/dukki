"""Check the leg calibration with both soles flat on a table: each sole's down, carried through its
leg's joints, must agree with the other sole's and with the trunk IMU's down.

Reads the same stream as joint_scope.py (microduck `duck-control/examples/joint_scope`, every
servo limp). Hold the robot standing with both soles pressed flat on a flat table, in a few
different crouches and leans, and take a reading in each. Commands on stdin:

    r   a reading (two seconds, the robot held still).
    w   the summary over the readings so far.
    q   quit.

Per reading, each foot's down in the trunk frame (MJCF kinematics at the measured joint angles)
against the IMU's (its SFLP quaternion through the robot's mount), as pitch (about the trunk's y,
forward positive) and roll (about its x). The left-right difference needs no IMU. Read as
calibration: a leg's pitch error is the sum of its hip pitch, knee and ankle zero errors, its roll
error its hip roll's (there is no ankle roll). Each is also restated as the change to that leg's
ankle zero (pitch) or hip roll zero (roll) that would flatten that foot. Nothing is written but
--out/readings.json; the zeros file is only read.

    uv run --with mujoco --with numpy python scripts/joint_scope/flatfeet.py \\
        --zeros hardware/calibration/bridge_zero.toml --mount hardware/calibration/bridge_imu_mount.toml
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tomllib
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from joint_scope import COUNTS, JOINT_NAMES, TWO_PI, wrap  # noqa: E402
from seatings import Stream, joint_angles, sensor_down, summarise  # noqa: E402

READ_SECONDS = 2.0
STILL = 0.15  # rad/s: held by hand, so looser than the jig's


HOME = {"left_hip_roll": -0.0873, "left_hip_pitch": -0.4579, "left_knee": -0.0049, "left_ankle": 0.4530,
        "right_hip_roll": 0.0873, "right_hip_pitch": 0.4579, "right_knee": 0.0049, "right_ankle": -0.4530}


class Legs:
    """Each sole's down in the trunk frame, from the MJCF's kinematics.

    The sole is the bottom face of each foot's collision mesh, not the `*_foot` sites: those are
    rolled about 5 degrees from the sole (they follow the hip roll's home angle), while the soles
    are flat to 0.3 degrees with the trunk level in the home pose."""

    def __init__(self, mjcf: Path):
        import mujoco

        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(mjcf))
        self.d = mujoco.MjData(self.m)
        self._pose(HOME)
        self.sole = {}  # sole normal (outward, down) in each collision geom's frame
        for side in ("left", "right"):
            g = self.m.geom(f"{side}_foot_collision").id
            mesh = self.m.geom_dataid[g]
            v = self.m.mesh_vert[self.m.mesh_vertadr[mesh]:self.m.mesh_vertadr[mesh] + self.m.mesh_vertnum[mesh]]
            rot = self.d.geom_xmat[g].reshape(3, 3)
            w = v @ rot.T
            n = np.array([0.0, 0.0, 1.0])
            for _ in range(5):  # plane through the lowest vertices, refitted along its own normal
                h = w @ n
                low = w[h < h.min() + 0.0015]
                _, _, vt = np.linalg.svd(low - low.mean(0))
                n = vt[2] * np.sign(vt[2][2])
            self.sole[side] = rot.T @ -n

    def _pose(self, angles: dict[str, float]) -> None:
        for name, q in angles.items():
            j = self.mj.mj_name2id(self.m, self.mj.mjtObj.mjOBJ_JOINT, name)
            if j >= 0:
                self.d.qpos[self.m.jnt_qposadr[j]] = q
        self.mj.mj_kinematics(self.m, self.d)

    def downs(self, angles: dict[str, float]) -> dict[str, np.ndarray]:
        self._pose(angles)
        trunk = self.d.xmat[self.m.body("trunk_base").id].reshape(3, 3)
        out = {}
        for side in ("left", "right"):
            geom = self.d.geom_xmat[self.m.geom(f"{side}_foot_collision").id].reshape(3, 3)
            out[side] = trunk.T @ geom @ self.sole[side]
        return out


def pitch_roll(down: np.ndarray) -> tuple[float, float]:
    """Tilt of a down vector in the trunk frame, deg: pitch about y (forward positive), roll about x."""
    return math.degrees(math.atan2(down[0], -down[2])), math.degrees(math.atan2(down[1], -down[2]))


def flattening(legs: Legs, angles: dict[str, float], side: str, joint: str, axis: int, target: float) -> float:
    """Change to `joint`'s reading, deg, that brings `side`'s foot to `target` on `axis` (0 pitch, 1 roll)."""
    def f(delta: float) -> float:
        a = dict(angles)
        a[joint] += math.radians(delta)
        return pitch_roll(legs.downs(a)[side])[axis] - target

    delta = 0.0
    for _ in range(8):  # secant on a near-linear function
        f0, f1 = f(delta), f(delta + 0.5)
        if abs(f1 - f0) < 1e-9:
            break
        delta -= f0 * 0.5 / (f1 - f0)
    return delta


def analyse(legs: Legs, angles: dict[str, float], imu_down: np.ndarray) -> dict:
    downs = legs.downs(angles)
    imu = pitch_roll(imu_down)
    r = {"imu_pitch": imu[0], "imu_roll": imu[1]}
    for side in ("left", "right"):
        p, rl = pitch_roll(downs[side])
        r[f"{side}_pitch_err"] = p - imu[0]
        r[f"{side}_roll_err"] = rl - imu[1]
        # Reading change that flattens this foot. The zero moves the opposite way: a reading
        # that must grow by d means the zero count is d too high.
        r[f"{side}_ankle_fix"] = flattening(legs, angles, side, f"{side}_ankle", 0, imu[0])
        r[f"{side}_hip_roll_fix"] = flattening(legs, angles, side, f"{side}_hip_roll", 1, imu[1])
    r["lr_pitch"] = r["left_pitch_err"] - r["right_pitch_err"]
    r["lr_roll"] = r["left_roll_err"] - r["right_roll_err"]
    return r


def show(tag: str, r: dict) -> str:
    return (f"{tag}: trunk pitch {r['imu_pitch']:+5.1f} roll {r['imu_roll']:+5.1f} | foot minus IMU, deg: "
            f"left pitch {r['left_pitch_err']:+5.2f} roll {r['left_roll_err']:+5.2f}, "
            f"right pitch {r['right_pitch_err']:+5.2f} roll {r['right_roll_err']:+5.2f} | "
            f"left minus right: pitch {r['lr_pitch']:+5.2f} roll {r['lr_roll']:+5.2f}")


def summary(rows: list[dict]) -> str:
    if not rows:
        return "no readings yet"
    keys = [("left_pitch_err", "left foot pitch vs IMU"), ("right_pitch_err", "right foot pitch vs IMU"),
            ("lr_pitch", "left minus right pitch"), ("left_roll_err", "left foot roll vs IMU"),
            ("right_roll_err", "right foot roll vs IMU"), ("lr_roll", "left minus right roll"),
            ("left_ankle_fix", "left ankle reading change to flatten"),
            ("right_ankle_fix", "right ankle reading change to flatten"),
            ("left_hip_roll_fix", "left hip roll reading change to flatten"),
            ("right_hip_roll_fix", "right hip roll reading change to flatten")]
    out = [f"over {len(rows)} readings (mean, spread min..max), deg:"]
    for k, label in keys:
        v = np.array([r[k] for r in rows])
        out.append(f"  {label:<42}{v.mean():+6.2f}   {v.min():+6.2f} .. {v.max():+6.2f}")
    return "\n".join(out)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="microduck.local")
    p.add_argument("--port", type=int, default=7788)
    p.add_argument("--zeros", type=Path, required=True, help="the current [bus.bridge_zero] file (read only)")
    p.add_argument("--mount", type=Path, required=True, help="the current [bus.bridge_imu_mount] file (read only)")
    p.add_argument("--travel", type=Path, default=Path("hardware/calibration/joint_travel.json"))
    p.add_argument("--mjcf", type=Path, default=Path("../microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml"))
    p.add_argument("--out", type=Path, default=Path("scratch/flatfeet"))
    args = p.parse_args()

    import mujoco

    table = tomllib.loads(args.zeros.read_text())["bus"]["bridge_zero"]
    zeros = [int(table[n]) for n in JOINT_NAMES]
    travel = {t["name"]: t for t in json.loads(args.travel.read_text())["joints"]}
    centres = []
    for name, z in zip(JOINT_NAMES, zeros):
        t = travel[name]
        shift = math.degrees(wrap((t["zero_counts"] - z) * TWO_PI / COUNTS))
        centres.append(math.radians((t["range_deg"][0] + t["range_deg"][1]) / 2 + shift))
    mq = tomllib.loads(args.mount.read_text())["bus"]["bridge_imu_mount"]
    mount = np.empty(9)
    mujoco.mju_quat2Mat(mount, np.array([mq["w"], mq["x"], mq["y"], mq["z"]]))
    mount = mount.reshape(3, 3)

    legs = Legs(args.mjcf)
    args.out.mkdir(parents=True, exist_ok=True)
    stream = Stream(args.host, args.port)
    rows: list[dict] = []
    print("connected; r = a reading with both soles flat, w = summary, q = quit", flush=True)
    for line in sys.stdin:
        cmd = line.strip()
        if cmd == "q":
            break
        if cmd == "r":
            s = summarise(stream.read(READ_SECONDS))
            if s["fastest"][1] > STILL:
                print(f"refused: {s['fastest'][0]} moving at {s['fastest'][1]:.2f} rad/s", flush=True)
                continue
            angles = dict(zip(JOINT_NAMES, joint_angles(s["counts"], zeros, centres)))
            imu = mount @ np.array(s["down"])
            r = analyse(legs, angles, imu / np.linalg.norm(imu))
            r["angles_deg"] = {n: round(math.degrees(a), 2) for n, a in angles.items()}
            rows.append(r)
            print(show(f"reading {len(rows)}", r), flush=True)
            (args.out / "readings.json").write_text(json.dumps(rows, indent=1))
            continue
        if cmd == "w":
            print(summary(rows), flush=True)
    print(summary(rows), flush=True)


if __name__ == "__main__":
    main()
