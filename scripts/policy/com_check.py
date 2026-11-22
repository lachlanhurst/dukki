"""Where is the robot's centre of mass, fore and aft, against the model? From the joint torques it
holds while standing still.

Standing still on flat feet, everything above the ankles is in equilibrium under gravity and the two
ankles' torques, so about the line through the two ankle axes the sum of the ankle torques equals the
moment of that weight: M g times the centre of mass's distance ahead of the ankles. The same holds
at the knees and at the hip pitches for what is above them. Each J288 applies kp (target - position)
when still (kd acts on speed, zero here), so the recorded targets and positions give the robot's
torques; the model, posed exactly as the robot stood, gives the torques its mass distribution needs.
The difference, divided by the weight above, is how far the real centre of mass sits ahead of the
model's (negative: behind).

Two sources of error: friction can hold part of a load (each joint's breakaway torque, about 0.02 to
0.1 N.m, either way; averaging stances that settled from different directions helps), and the
trunk's tilt, taken from the IMU, which disagrees with the legs by about 4 degrees
(docs/policy-testing.md section 5.2): the result is also given with the trunk pitched 4 degrees
either way.

    cd microduck_rl && uv run python ../dukki/scripts/policy/com_check.py --kp 0.975 RECORDING.json ...

Recordings are stand_test.py's or walk_test.py's; still stretches (no joint moving, gyro quiet, at
least 1.5 s) are found in each, and each counts once.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

from mjlab_microduck.robot.dukki_constants import get_dukki_walk_spec  # noqa: E402

NAMES = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch",
         "head_pitch", "head_yaw", "head_roll", "mouth", "right_hip_yaw", "right_hip_roll",
         "right_hip_pitch", "right_knee", "right_ankle"]
G = 9.81


def still_stretches(frames, min_s=1.5):
    """(start, end) indices of stretches where the policy runs, nothing moves and the gyro is quiet."""
    ok = []
    for i, (_, p) in enumerate(frames):
        quiet = (p.get("policy") == "walk" and not p["safety"]["fallen"]
                 and max(abs(v) for v in p["imu"]["gyro"]) < 0.05
                 and i > 0 and max(abs(a - b) for a, b in zip(p["joints"], frames[i - 1][1]["joints"])) < 0.002)
        ok.append(quiet)
    out, start = [], None
    for i, q in enumerate(ok + [False]):
        if q and start is None:
            start = i
        elif not q and start is not None:
            if frames[i - 1][0] - frames[start][0] >= min_s:
                out.append((start, i))
            start = None
    return out


def tilted(down, deg):
    """`down` turned by `deg` about the trunk's y axis: the trunk pitched that much further forward."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    x, y, z = down
    return np.array([c * x - s * z, y, s * x + c * z])


def rotation_taking(a, b):
    """The smallest rotation matrix taking unit vector a to unit vector b."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * (1 / (1 + c))


class Model:
    def __init__(self):
        self.m = get_dukki_walk_spec().compile()
        self.d = mujoco.MjData(self.m)
        free = [j for j in range(self.m.njnt) if self.m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        self.root_q = self.m.jnt_qposadr[free[0]]
        self.trunk = self.m.jnt_bodyid[free[0]]

    def pose(self, joints, trunk_down):
        """Pose the model: joint angles, and the trunk tilted so its frame's `trunk_down` is world down."""
        d, m = self.d, self.m
        d.qpos[:] = m.qpos0
        for name, q in zip(NAMES, joints):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j >= 0:
                d.qpos[m.jnt_qposadr[j]] = q
        r = rotation_taking(np.asarray(trunk_down), np.array([0.0, 0.0, -1.0]))
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, r.flatten())
        d.qpos[self.root_q + 3:self.root_q + 7] = quat
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)

    def subtree(self, body):
        out, stack = set(), [body]
        while stack:
            b = stack.pop()
            out.add(b)
            stack += [c for c in range(self.m.nbody) if self.m.body_parentid[c] == b and c != b]
        return out

    def moment(self, joint_names, torques):
        """For a left/right joint pair: (the model's gravity moment about the line through the two
        joints, the robot's from `torques`, the mass above them)."""
        m, d = self.m, self.d
        ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joint_names]
        below = set().union(*(self.subtree(m.jnt_bodyid[j]) for j in ids))
        anchors = [d.xanchor[j].copy() for j in ids]
        u = anchors[0] - anchors[1]
        u /= np.linalg.norm(u)  # left minus right: about +y
        p = anchors[1]
        model, mass = 0.0, 0.0
        for b in range(1, m.nbody):
            if b in below:
                continue
            w = np.array([0.0, 0.0, -m.body_mass[b] * G])
            model += float(np.cross(d.xipos[b] - p, w) @ u)
            mass += m.body_mass[b]
        robot = sum(t * float(d.xaxis[j] @ u) for j, t in zip(ids, torques))
        return model, robot, mass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--kp", type=float, default=0.975, help="J288 kp the robot ran, N.m/rad (1.0 rounds to 0.975)")
    args = ap.parse_args()
    model = Model()
    pairs = {"ankles": ("left_ankle", "right_ankle"), "knees": ("left_knee", "right_knee"),
             "hip pitches": ("left_hip_pitch", "right_hip_pitch")}
    rows = {k: [] for k in pairs}
    for path in args.recordings:
        data = json.load(open(path))
        frames = data["frames"] if isinstance(data, dict) else data
        for a, b in still_stretches(frames):
            ps = [p for _, p in frames[a:b]]
            q = np.mean([p["joints"] for p in ps], axis=0)
            tgt = np.mean([p["targets"] for p in ps], axis=0)
            grav = np.mean([p["safety"]["gravity"] for p in ps], axis=0)
            tau = args.kp * (tgt - q)
            for source, down in (("imu", grav), ("imu+4", tilted(grav, 4.0)), ("imu-4", tilted(grav, -4.0))):
                model.pose(q, down)
                for name, joints in pairs.items():
                    ids = [NAMES.index(n) for n in joints]
                    mdl, rob, mass = model.moment(joints, [tau[i] for i in ids])
                    rows[name].append((Path(path).stem, frames[a][0], source, mdl, rob, mass))
    for name, rs in rows.items():
        print(f"\n{name}: robot's torque moment against the model's, and the centre of mass shift it implies")
        for source in ("imu", "imu+4", "imu-4"):
            sel = [r for r in rs if r[2] == source]
            shifts = [(r[4] - r[3]) / (r[5] * G) * 1000 for r in sel]
            if source == "imu":
                for r, sh in zip(sel, shifts):
                    print(f"  {r[0][:28]:28} {r[1]:5.1f}s  model {r[3]:+.3f}  robot {r[4]:+.3f} N.m  -> {sh:+6.1f} mm")
            print(f"  trunk tilt {source:6}: mean over {len(sel)} stances {np.mean(shifts):+.1f} mm "
                  f"(sd {np.std(shifts):.1f}), mass above {sel[0][5]:.3f} kg")


if __name__ == "__main__":
    main()
