"""Calibrate from several seatings in the jig: joint zeros averaged, and the trunk IMU's mounting.

Reads the same stream as joint_scope.py (microduck `duck-control/examples/joint_scope`, every
servo limp). Commands on stdin, each reading two seconds with the robot at rest:

    s   a seating in the calibration jig: every joint's output encoder, and which way is down
        for the trunk IMU (from its SFLP game-rotation quaternion, what robotd reads). Re-seat
        the robot between seatings; the spread between them is how well the jig locates it.
    u   the robot upright, both soles flat on a table: the encoders give the trunk's attitude
        through the legs, so this need not be any particular pose.
    w   write: the zeros averaged over the seatings (the mouth kept from --zeros, since the jig
        does not set it), each joint's spread, and the IMU mounting solved from the two kinds of
        reading. Nothing is written outside --out.
    q   quit.

Down in the trunk frame is exactly -x in the jig (robot on its back), and with the soles flat
it is the soles' down carried through the leg joints. The mounting is the rotation taking the
sensor's down to the trunk's in both, by least squares (Kabsch), as a scalar-first quaternion
for robotd's `bus.bridge_imu_mount`.

    uv run --with mujoco --with numpy python scripts/joint_scope/seatings.py \\
        --zeros hardware/calibration/bridge_zero.toml --out scratch/imu_cal
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import sys
import time
import tomllib
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from joint_scope import COUNTS, JOINT_NAMES, MOUTH, TWO_PI, wrap  # noqa: E402

READ_SECONDS = 2.0
REST = 0.05  # rad/s: a reading is refused if any joint moves faster


def half(u: int) -> float:
    return struct.unpack("<e", struct.pack("<H", u))[0]


def sensor_down(block: list[int]) -> np.ndarray:
    """World down in the IMU's frame, from the SFLP game-rotation quaternion (sensor to world)."""
    x, y, z = (half(block[6 + 2 * i] | block[7 + 2 * i] << 8) for i in range(3))
    w = math.sqrt(max(0.0, 1.0 - x * x - y * y - z * z))
    third_row = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
    return -third_row


class Stream:
    """A fresh connection per reading: joint_scope drops a client that stops reading, and one
    left idle between commands would otherwise be read from the stale buffer it left behind."""

    def __init__(self, host: str, port: int):
        self.addr = (host, port)

    def read(self, seconds: float) -> list[dict]:
        frames, buf = [], b""
        with socket.create_connection(self.addr, timeout=3) as sock:
            sock.settimeout(1.0)
            while b"\n" not in buf:  # the first line may be cut: start at the second
                buf += sock.recv(65536)
            buf = buf.split(b"\n", 1)[1]
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                buf += sock.recv(65536)
                *lines, buf = buf.split(b"\n")
                frames += [json.loads(line) for line in lines if line.strip()]
        return frames


def summarise(frames: list[dict]) -> dict:
    """Mean encoder counts per joint (circular), mean IMU down, and the fastest joint."""
    enc = np.array([[s["enc"] for s in f["servos"]] for f in frames], dtype=float) * TWO_PI / COUNTS
    vel = np.array([[abs(s["vel"]) for s in f["servos"]] for f in frames])
    mean = np.arctan2(np.sin(enc).sum(0), np.cos(enc).sum(0))
    down = np.mean([sensor_down(f["imu"]["block"]) for f in frames], axis=0)
    return {
        "counts": [int(round(m * COUNTS / TWO_PI)) % COUNTS for m in mean],
        "spread_deg": [float(np.degrees(np.max(np.abs(np.angle(np.exp(1j * (enc[:, i] - mean[i]))))))) for i in range(len(mean))],
        "down": (down / np.linalg.norm(down)).tolist(),
        "fastest": [JOINT_NAMES[int(vel.max(0).argmax())], float(vel.max())],
        "frames": len(frames),
    }


def circmean_counts(values: list[int]) -> int:
    a = np.array(values) * TWO_PI / COUNTS
    return int(round(math.atan2(np.sin(a).sum(), np.cos(a).sum()) * COUNTS / TWO_PI)) % COUNTS


def joint_angles(counts: list[int], zeros: list[int], centres: list[float]) -> list[float]:
    """Joint angles (rad) from encoder counts, each in the turn centred on its travel."""
    out = []
    for c, z, centre in zip(counts, zeros, centres):
        a = (c - z) * TWO_PI / COUNTS
        out.append(a - TWO_PI * round((a - centre) / TWO_PI))
    return out


def upright_down(mjcf: Path, angles: list[float]) -> tuple[np.ndarray, float]:
    """Down in the trunk frame with both soles flat, and the angle between the two soles' answers."""
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(mjcf))
    d = mujoco.MjData(m)
    for name, q in zip(JOINT_NAMES, angles):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        if j >= 0:
            d.qpos[m.jnt_qposadr[j]] = q
    mujoco.mj_kinematics(m, d)
    trunk = d.xmat[m.body("trunk_base").id].reshape(3, 3)
    downs = []
    for site in ("left_foot", "right_foot"):
        foot = d.site_xmat[m.site(site).id].reshape(3, 3)
        downs.append(trunk.T @ foot @ np.array([0.0, 0.0, -1.0]))
    a, b = downs
    return (a + b) / np.linalg.norm(a + b), float(np.degrees(np.arccos(np.clip(a @ b, -1, 1))))


def kabsch(sensor: list[np.ndarray], trunk: list[np.ndarray]) -> np.ndarray:
    """Rotation R (sensor to trunk) minimising sum |R s - t|^2."""
    h = sum(np.outer(s, t) for s, t in zip(sensor, trunk))
    u, _, vt = np.linalg.svd(h)
    dsign = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1.0, 1.0, dsign]) @ u.T


def quat(r: np.ndarray) -> list[float]:
    import mujoco

    q = np.empty(4)
    mujoco.mju_mat2Quat(q, r.flatten())
    return (q * np.sign(q[0] or 1.0)).tolist()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="microduck.local")
    p.add_argument("--port", type=int, default=7788)
    p.add_argument("--zeros", type=Path, required=True, help="the current [bus.bridge_zero] file (read only)")
    p.add_argument("--travel", type=Path, default=Path("hardware/calibration/joint_travel.json"))
    p.add_argument("--mjcf", type=Path, default=Path("../microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml"))
    p.add_argument("--out", type=Path, default=Path("scratch/imu_cal"))
    p.add_argument("--resume", action="store_true", help="carry on from the readings already in --out")
    args = p.parse_args()

    table = tomllib.loads(args.zeros.read_text())["bus"]["bridge_zero"]
    zeros = [int(table[n]) for n in JOINT_NAMES]
    travel = {t["name"]: t for t in json.loads(args.travel.read_text())["joints"]}
    centres = []
    for name, z in zip(JOINT_NAMES, zeros):
        t = travel[name]
        shift = math.degrees(wrap((t["zero_counts"] - z) * TWO_PI / COUNTS))
        centres.append(math.radians((t["range_deg"][0] + t["range_deg"][1]) / 2 + shift))
    args.out.mkdir(parents=True, exist_ok=True)
    stream = Stream(args.host, args.port)
    seatings: list[dict] = []
    uprights: list[dict] = []
    if args.resume and (args.out / "readings.json").exists():
        saved = json.loads((args.out / "readings.json").read_text())
        seatings, uprights = saved["seatings"], saved["uprights"]
        print(f"resumed {len(seatings)} seatings and {len(uprights)} upright readings", flush=True)
    print("connected; s = a jig seating, u = upright on flat soles, w = write, q = quit", flush=True)

    for line in sys.stdin:
        cmd = line.strip()
        if cmd == "q":
            break
        if cmd in ("s", "u"):
            r = summarise(stream.read(READ_SECONDS))
            if r["fastest"][1] > REST:
                print(f"refused: {r['fastest'][0]} moving at {r['fastest'][1]:.2f} rad/s", flush=True)
                continue
            angles = joint_angles(r["counts"], zeros, centres)
            r["angles_deg"] = [round(math.degrees(a), 2) for a in angles]
            if cmd == "s":
                seatings.append(r)
                tag = f"seating {len(seatings)}"
                t = np.array([-1.0, 0.0, 0.0])
            else:
                r["trunk_down"], r["soles_disagree_deg"] = upright_down(args.mjcf, angles)
                r["trunk_down"] = r["trunk_down"].tolist()
                uprights.append(r)
                tag = f"upright {len(uprights)} (soles agree to {r['soles_disagree_deg']:.2f} deg)"
            print(f"{tag}: {r['frames']} frames, IMU down {np.round(r['down'], 4).tolist()}", flush=True)
            print("  joints vs the current zeros, deg: " + "  ".join(
                f"{n} {a:+.1f}" for n, a in zip(JOINT_NAMES, r["angles_deg"])), flush=True)
            (args.out / "readings.json").write_text(json.dumps({"seatings": seatings, "uprights": uprights}, indent=1))
            continue
        if cmd == "w":
            out = []
            if seatings:
                new = [zeros[i] if i == MOUTH else circmean_counts([s["counts"][i] for s in seatings])
                       for i in range(len(JOINT_NAMES))]
                out.append(f"zeros from {len(seatings)} seatings (counts; change from the current file; spread between seatings):")
                for i, name in enumerate(JOINT_NAMES):
                    if i == MOUTH:
                        out.append(f"  {name:<16} {new[i]:>5}   kept from {args.zeros}")
                        continue
                    vals = [math.degrees(wrap((s["counts"][i] - new[i]) * TWO_PI / COUNTS)) for s in seatings]
                    change = math.degrees(wrap((new[i] - zeros[i]) * TWO_PI / COUNTS))
                    out.append(f"  {name:<16} {new[i]:>5}   {change:+6.2f} deg   spread {max(vals) - min(vals):5.2f} deg")
                body = ["# Joint zeros averaged over several seatings in the calibration jig by",
                        f"# scripts/joint_scope/seatings.py, {time.strftime('%d/%m/%Y %H:%M')}; mouth from {args.zeros}.",
                        "[bus.bridge_zero]"] + [f"{n} = {z}" for n, z in zip(JOINT_NAMES, new)]
                (args.out / "bridge_zero.toml").write_text("\n".join(body) + "\n")
                out.append(f"  wrote {args.out / 'bridge_zero.toml'}")
            if seatings and uprights:
                s_vecs = [np.mean([s["down"] for s in seatings], axis=0), np.mean([u["down"] for u in uprights], axis=0)]
                s_vecs = [v / np.linalg.norm(v) for v in s_vecs]
                t_vecs = [np.array([-1.0, 0.0, 0.0]), np.mean([u["trunk_down"] for u in uprights], axis=0)]
                t_vecs[1] /= np.linalg.norm(t_vecs[1])
                r = kabsch(s_vecs, t_vecs)
                q = quat(r)
                cad = np.array([0.5, -0.5, -0.5, 0.5])
                off = math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(q, cad))))))
                ang = lambda a, b: math.degrees(math.acos(np.clip(a @ b, -1, 1)))  # noqa: E731
                out.append(f"IMU mounting (sensor to trunk, w x y z): {np.round(q, 5).tolist()}, {off:.2f} deg from the CAD's")
                out.append(f"  angle between the readings: sensor {ang(*s_vecs):.2f} deg, trunk {ang(*t_vecs):.2f} deg")
                for k, (sv, tv) in enumerate(zip(s_vecs, t_vecs)):
                    out.append(f"  residual {'jig' if k == 0 else 'upright'}: {ang(r @ sv, tv):.2f} deg")
                for s in seatings:
                    out.append(f"  seating down after mounting: {np.round(r @ np.array(s['down']), 4).tolist()}")
                (args.out / "imu_mount.json").write_text(json.dumps({"mount_wxyz": q, "from_cad_deg": off}, indent=1))
                out.append(f"  wrote {args.out / 'imu_mount.json'}")
            print("\n".join(out) or "nothing to write yet", flush=True)


if __name__ == "__main__":
    main()
