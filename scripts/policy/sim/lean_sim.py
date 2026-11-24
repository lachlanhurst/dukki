#!/usr/bin/env python3
"""The ankle-lean test in simulation, and the robot's lean recordings against the model.

The test: the robot stands on a hard, flat table in the home pose with no policy, every servo
holding its target, and both ankles turn slowly together so the whole robot above the ankles leans
backwards (or forwards) over its flat feet, until it tips over the heel (or toe) edge of its soles.
The robot side is microduck `duck-control/examples/lean_test.rs`; its recordings are JSON.

Tipping happens when the centre of mass passes over the sole's edge, so a robot whose centre of
mass sits further back than the model's tips backwards sooner and forwards later. Two ways to
compare:

* simulate: the same ramp on the model, with the whole robot's centre of mass shifted fore and
  aft by each of --com-mm, printing the lean at which it tips each way.
* --recording: for each robot recording, the pose the robot was in as it started to tip (joint
  angles from the encoders) is posed on the model, and the model's centre of mass is measured
  against the soles' edges. At the tip the real centre of mass is over the edge, so the model's
  distance from the edge is how far the real one is from the model's. The trunk's tilt is taken
  two ways: from the legs (both soles flat on the table) and from the trunk IMU; they disagree
  by about 4 degrees on this robot (docs/policy-testing.md section 5.2), which is about 8 mm here.

Each recording's result is taken against the simulation's own tip at the same gains and rate, so the
onset detection's lead (about 1.5 mm before the edge) cancels. A backward tip measures c - s and a
forward one c + s, where c is how far the real centre of mass is ahead of the model's and s how much
shorter each end of the real sole is in effect (rounded rims, the sole's give); with both, c and s.

The soles are flat boxes in this simulation, not the meshes: see `Model`. The servos hold at
lean_test's kp (4 N.m/rad by default): at the policy's 1 N.m/rad the legs are too soft, and in
simulation the robot buckles before its centre of mass reaches the heel edge, by how much depending
on joint friction.

    cd microduck_rl
    uv run python ../dukki/scripts/policy/sim/lean_sim.py                      # simulate
    uv run python ../dukki/scripts/policy/sim/lean_sim.py --com-mm -6,-3,0,3   # shifted
    uv run python ../dukki/scripts/policy/sim/lean_sim.py --recording lean_back_1.json lean_fwd_1.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np
import mujoco

import mjlab_microduck.tasks  # noqa: F401  (plugin registration before the robot modules)
from mjlab_microduck.robot.dukki_constants import get_dukki_walk_backlash_spec, get_dukki_walk_spec
from mjlab_microduck.robot.microduck_constants import HOME_FRAME

# robotd's joint order (duck-ipc-proto JOINT_NAMES): the recordings' arrays are in it.
NAMES = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch",
         "head_pitch", "head_yaw", "head_roll", "mouth", "right_hip_yaw", "right_hip_roll",
         "right_hip_pitch", "right_knee", "right_ankle"]
# Lean direction per ankle: a positive delta (deg) lowers left_ankle and raises right_ankle by it,
# which leans the robot forwards (checked by `lean_signs`, which the simulation asserts).
# duck-control examples/lean_test.rs uses the same signs.
ANKLE_SIGN = {"left_ankle": -1.0, "right_ankle": 1.0}
TIP_RATE = 1.0     # rad/s about the trunk's y for TIP_TICKS ticks: tipping (lean_test's; a roll onto the toes stays under 0.3)
TIP_TICKS = 3
ONSET_RATE = 0.05  # rad/s: the last tick below this before the tip is where it started to go
POSE_TICKS = 10    # the pose it tipped from: joints and gravity averaged over the ticks up to the onset
CONTROL_DT = 0.01
SOLE_FACE_M = 0.0005  # the sole's face: within 0.5 mm of its plane (the rounded rims rise beyond)


def home_angles() -> dict[str, float]:
    import re
    out = {}
    for pat, v in HOME_FRAME.joint_pos.items():
        for n in NAMES:
            if re.fullmatch(pat, n):
                out[n] = v
    return out


class Model:
    """The walk model (Fusion inertials, unmodelled mass, Dukki's joint stops), on a floor."""

    def __init__(self, com_mm: float = 0.0, kp: float = 4.0, kd: float = 0.05, friction: float = 0.02,
                 max_torque: float = 1.5, flat_soles: bool = True, heel_mm: float = 0.0, backlash: bool = False):
        # backlash: the walk model with a passive ±1° hinge after every servo (training's gear-play
        # variant); the servos' positions are then the motor side, as the J288's rotor-derived ones.
        # heel_mm: the flat soles' heel end that much further forward (a heel that bears no weight).
        self.heel_mm = heel_mm
        spec = get_dukki_walk_backlash_spec() if backlash else get_dukki_walk_spec()
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.1], friction=[1.0, 0.005, 0.0001])
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.qadr = {n: self.m.jnt_qposadr[self.m.joint(n).id] for n in NAMES if n != "mouth"}
        self.home = home_angles()
        self._find_soles()
        if flat_soles:
            # The sole meshes are a shallow dome (the bottom face rises 0.3 mm over the 15 mm
            # behind the middle and 0.2 mm over the 20 mm ahead), and MuJoCo's plane-mesh contacts
            # sit only at the deepest vertices, so on the mesh the model rocks on a point under its
            # centre of mass instead of standing on the sole's face. A real sole under 11 N on a
            # desk flattens. Each foot gets a 2 mm box under the sole face's footprint instead.
            for side, body in (("left", "ankle_left"), ("right", "ankle_right")):
                g, _, _ = self.sole[side]
                pos, quat, half = self._sole_box(side, body)
                spec.geom(f"{side}_foot_collision").contype = 0
                spec.geom(f"{side}_foot_collision").conaffinity = 0
                spec.body(body).add_geom(name=f"{side}_sole_box", type=mujoco.mjtGeom.mjGEOM_BOX, size=half,
                                         pos=pos, quat=quat, contype=1, conaffinity=1, condim=3,
                                         friction=[1.0, 0.005, 0.0001], group=3)
            self.m = spec.compile()
            self.d = mujoco.MjData(self.m)
            self._find_soles()
        m = self.m
        self.trunk = m.body("trunk_base").id
        self.total = float(m.body_mass.sum())
        # Whole-robot centre of mass com_mm forwards (trunk +x), by moving the trunk's own.
        m.body_ipos[self.trunk, 0] += com_mm / 1000 * self.total / m.body_mass[self.trunk]
        for a in range(m.nu):
            m.actuator_gainprm[a, 0] = kp
            m.actuator_biasprm[a, 1] = -kp
            m.actuator_biasprm[a, 2] = -kd
            m.actuator_forcerange[a] = (-max_torque, max_torque)
        for j in range(1, m.njnt):
            m.dof_damping[m.jnt_dofadr[j]] = 0.0
            m.dof_frictionloss[m.jnt_dofadr[j]] = friction
        self.act = {m.actuator(a).name: a for a in range(m.nu)}

    def _find_soles(self) -> None:
        """Each sole: the bottom face of its collision mesh (vertices within SOLE_FACE_M of the plane
        through the lowest ones), in the mesh's frame, and the face's outward normal there."""
        m = self.m
        self._pose_trunk_frame(self.home)
        self.sole = {}
        for side in ("left", "right"):
            g = m.geom(f"{side}_foot_collision").id
            mesh = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]].astype(float)
            w = v @ self.d.geom_xmat[g].reshape(3, 3).T
            n = np.array([0.0, 0.0, 1.0])
            for _ in range(5):  # plane through the lowest vertices, refitted along its own normal
                h = w @ n
                low = w[h < h.min() + 0.0015]
                _, _, vt = np.linalg.svd(low - low.mean(0))
                n = vt[2] * np.sign(vt[2][2])
            h = w @ n
            self.sole[side] = (g, v[h < h.min() + SOLE_FACE_M], -(self.d.geom_xmat[g].reshape(3, 3).T @ n))

    def _sole_box(self, side: str, body: str):
        """A box under the sole face's footprint, in `body`'s frame: (pos, quat, half sizes)."""
        m, d = self.m, self.d
        self._pose_trunk_frame(self.home)
        g, v, n_geom = self.sole[side]
        Rg = d.geom_xmat[g].reshape(3, 3)
        pts = v @ Rg.T + d.geom_xpos[g]            # trunk frame
        down = Rg @ n_geom                          # trunk frame
        x = np.array([1.0, 0.0, 0.0]) - down * down[0]
        x /= np.linalg.norm(x)
        y = np.cross(down, x)                       # with z = -down: x, y, -down right-handed
        z = -down
        Rb_t = np.stack([x, y, z], 1)              # box axes in the trunk frame
        local = pts @ Rb_t
        lo, hi = local.min(0), local.max(0)
        lo[0] += self.heel_mm / 1000
        half = np.array([(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, 0.001])
        c = Rb_t @ np.array([(hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2, lo[2] + half[2]])
        Rbody = d.xmat[m.body(body).id].reshape(3, 3)
        pos = Rbody.T @ (c - d.xpos[m.body(body).id])
        R = Rbody.T @ Rb_t
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R.flatten())
        return pos, quat, half

    # ── kinematics with the trunk at the origin, unrotated ───────────────────────────────────
    def _pose_trunk_frame(self, angles: dict[str, float]) -> None:
        d = self.d
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0
        for n, q in angles.items():
            if n in self.qadr:
                d.qpos[self.qadr[n]] = q
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)

    def _sole_points(self, side: str) -> np.ndarray:
        """The sole face's vertices where the model is posed."""
        g, v, _ = self.sole[side]
        return v @ self.d.geom_xmat[g].reshape(3, 3).T + self.d.geom_xpos[g]

    def _sole_down(self, side: str) -> np.ndarray:
        g, _, n = self.sole[side]
        return self.d.geom_xmat[g].reshape(3, 3) @ n

    def stance(self, angles: dict[str, float], trunk_down: np.ndarray | None = None) -> dict:
        """The model posed at `angles`, its trunk tilted so that `trunk_down` (trunk frame) is
        down, or, without it, so that the two soles are flat (their mean). Distances in mm along
        the trunk's heading: centre of mass ahead of the rearmost sole point and behind the
        foremost, and the trunk's forward lean, deg."""
        self._pose_trunk_frame(angles)
        if trunk_down is None:
            trunk_down = self._sole_down("left") + self._sole_down("right")
        g = np.asarray(trunk_down, float)
        g = g / np.linalg.norm(g)
        R = rotation_taking(g, np.array([0.0, 0.0, -1.0]))
        com = R @ (self.d.xipos * self.m.body_mass[:, None]).sum(0) / self.total
        pts = np.concatenate([self._sole_points("left"), self._sole_points("right")]) @ R.T
        fwd = R[:, 0].copy()
        fwd[2] = 0.0
        fwd /= np.linalg.norm(fwd)
        along = pts[:, :2] @ fwd[:2]
        c = com[:2] @ fwd[:2]
        return {"ahead_of_heel_mm": 1000 * (c - along.min()), "behind_toe_mm": 1000 * (along.max() - c),
                "lean_deg": math.degrees(math.asin(np.clip(g[0], -1, 1))),
                "com_height_mm": 1000 * (com[2] - pts[:, 2].min())}

    def lean_for(self, delta_deg: float) -> dict[str, float]:
        a = dict(self.home)
        for n, s in ANKLE_SIGN.items():
            a[n] += math.radians(s * delta_deg)
        return a

    # ── dynamics ─────────────────────────────────────────────────────────────────────────────
    def place(self, angles: dict[str, float]) -> None:
        """Standing at `angles`, soles flat on the floor, at rest."""
        self._pose_trunk_frame(angles)
        g = self._sole_down("left") + self._sole_down("right")
        R = rotation_taking(g / np.linalg.norm(g), np.array([0.0, 0.0, -1.0]))
        pts = np.concatenate([self._sole_points("left"), self._sole_points("right")]) @ R.T
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, R.flatten())
        d = self.d
        d.qpos[3:7] = q
        d.qpos[0:3] = [0.0, 0.0, -pts[:, 2].min() + 0.0005]
        d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, d)

    def set_targets(self, angles: dict[str, float]) -> None:
        for n, q in angles.items():
            if n in self.act:
                self.d.ctrl[self.act[n]] = q

    def trunk_rate_y(self) -> float:
        """Angular velocity about the trunk's y, rad/s (the free joint's is in the body frame)."""
        return float(self.d.qvel[4])

    def gravity_b(self) -> np.ndarray:
        R = self.d.xmat[self.trunk].reshape(3, 3)
        return R.T @ np.array([0.0, 0.0, -1.0])

    def joints(self) -> dict[str, float]:
        return {n: float(self.d.qpos[a]) for n, a in self.qadr.items()}


def rotation_taking(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The smallest rotation taking unit vector a to unit vector b."""
    v = np.cross(a, b)
    c = float(a @ b)
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def lean_deg_of(gravity_b: np.ndarray) -> float:
    g = gravity_b / np.linalg.norm(gravity_b)
    return math.degrees(math.asin(np.clip(g[0], -1, 1)))


def lean_signs(model: Model) -> None:
    """+delta on the ankles (ANKLE_SIGN) must lean the trunk forwards with the soles flat."""
    a, b = model.stance(model.lean_for(-2.0))["lean_deg"], model.stance(model.lean_for(2.0))["lean_deg"]
    assert b > a, f"ANKLE_SIGN leans backwards: {a:+.2f} -> {b:+.2f} deg"


def find_tip(rates: list[float]) -> tuple[int, int] | None:
    """(onset, detection) tick indices from the trunk's pitch rates, or None."""
    run = 0
    for i, r in enumerate(rates):
        run = run + 1 if abs(r) > TIP_RATE else 0
        if run >= TIP_TICKS:
            k = i
            while k > 0 and abs(rates[k]) >= ONSET_RATE:
                k -= 1
            return k, i
    return None


def simulate(model: Model, direction: float, rate_deg_s: float, settle_s: float, max_deg: float) -> dict:
    """Ramp the lean at rate_deg_s in `direction` (+1 forwards, -1 backwards) until it tips."""
    m, d = model.m, model.d
    model.place(model.home)
    model.set_targets(model.home)
    steps = round(CONTROL_DT / m.opt.timestep)
    rows = []
    t = 0.0
    n_settle = round(settle_s / CONTROL_DT)
    while True:
        delta = direction * max(0.0, t - settle_s) * rate_deg_s
        if abs(delta) > max_deg:
            break
        model.set_targets(model.lean_for(delta))
        for _ in range(steps):
            mujoco.mj_step(m, d)
        t += CONTROL_DT
        rows.append({"t": t, "delta": delta, "rate": model.trunk_rate_y(), "lean": lean_deg_of(model.gravity_b()),
                     "joints": model.joints(), "gravity": model.gravity_b().tolist()})
        if len(rows) <= n_settle:
            continue
        ramp = rows[n_settle:]
        tip = find_tip([r["rate"] for r in ramp])
        if tip is not None:
            on = ramp[tip[0]]
            window = ramp[max(0, tip[0] - POSE_TICKS + 1):tip[0] + 1]
            pose = {n: float(np.mean([r["joints"][n] for r in window])) for n in model.qadr}
            st = model.stance(pose)
            st_imu = model.stance(pose, np.mean([r["gravity"] for r in window], axis=0))
            return {"tipped": True, "rows": rows, "n_settle": n_settle, "delta": on["delta"], "lean": on["lean"],
                    "ankle_deg": ANKLE_SIGN["left_ankle"] * math.degrees(on["joints"]["left_ankle"] - model.home["left_ankle"]),
                    "settled_lean": rows[n_settle - 1]["lean"], **st, "imu": st_imu}
    return {"tipped": False, "delta": rows[-1]["delta"], "lean": rows[-1]["lean"],
            "settled_lean": rows[n_settle - 1]["lean"]}


def save_recording(path: str, r: dict, kp: float, kd: float, rate: float) -> None:
    """A simulated run in lean_test's recording format, to check the analysis against a known shift."""
    frames = []
    for i, row in enumerate(r["rows"]):
        frames.append({"t": row["t"], "phase": "settle" if i < r["n_settle"] else "ramp", "delta_deg": row["delta"],
                       "positions": [row["joints"].get(n, 0.0) for n in NAMES], "gravity": row["gravity"],
                       "gyro": [0.0, row["rate"], 0.0], "currents_ma": [0.0] * len(NAMES)})
    json.dump({"source": "lean_sim", "kp": kp, "kd": kd, "rate_deg_s": rate, "frames": frames}, open(path, "w"))


def from_recording(model: Model, path: str, baseline: dict) -> dict | None:
    """One robot recording against the model; `baseline` is the simulation's own tips (direction ->
    simulate()'s result) at the recording's gains and rate, which carry the same onset detection."""
    rec = json.load(open(path))
    # The tool stops the ramp on the tick it sees the tip, so the tip's last ticks are in "after".
    ramp = [f for f in rec["frames"] if f["phase"] in ("ramp", "after")]
    if not ramp:
        print(f"{path}: no ramp")
        return None
    tip = find_tip([f["gyro"][1] for f in ramp])
    if tip is None:
        print(f"{path}: never tipped (the ankles reached {ramp[-1]['delta_deg']:+.1f} deg)")
        return None
    on = ramp[tip[0]]
    window = ramp[max(0, tip[0] - POSE_TICKS + 1):tip[0] + 1]
    angles = {n: float(np.mean([f["positions"][j] for f in window])) for j, n in enumerate(NAMES)}
    imu_down = np.mean([f["gravity"] for f in window], axis=0)
    legs = model.stance(angles)
    imu = model.stance(angles, imu_down)
    back = on["delta_deg"] < 0
    sim = baseline["back" if back else "fwd"]
    ankles = [ANKLE_SIGN[n] * math.degrees(angles[n] - model.home[n]) for n in ("left_ankle", "right_ankle")]
    lean = lean_deg_of(imu_down)
    print(f"{path}: tipped {'backwards' if back else 'forwards'} {on['t']:.1f} s in, ankles commanded "
          f"{on['delta_deg']:+.2f} deg, read {ankles[0]:+.2f} / {ankles[1]:+.2f} (simulation: {sim['ankle_deg']:+.2f})")
    print(f"   trunk lean: IMU {lean:+.2f} deg (simulation {sim['lean']:+.2f}), legs with the soles flat {legs['lean_deg']:+.2f}")
    out = {"back": back}
    for tag, st in (("legs", legs), ("imu", imu)):
        # The simulation's own tip, its trunk tilt taken the same way (at the onset its soles have
        # begun to turn, so the IMU's tilt and the flat soles' already differ a little there).
        sim = baseline["back" if back else "fwd"]
        sim = sim["imu"] if tag == "imu" else sim
        if back:
            # The real centre of mass was over the heel edge; the model's is this far ahead of it,
            # beyond what the simulation's own onset leaves.
            shift = -(st["ahead_of_heel_mm"] - sim["ahead_of_heel_mm"])
            where = f"{st['ahead_of_heel_mm']:+5.1f} mm ahead of the heel edge (simulation's own tip {sim['ahead_of_heel_mm']:+.1f})"
        else:
            shift = st["behind_toe_mm"] - sim["behind_toe_mm"]
            where = f"{st['behind_toe_mm']:+5.1f} mm behind the toe edge (simulation's own tip {sim['behind_toe_mm']:+.1f})"
        print(f"   tilt from the {tag:4}: model posed so has its centre of mass {where}: robot's {shift:+.1f} mm ahead of the model's")
        out[tag] = shift
    peak = max(abs(c) for f in ramp for c in f["currents_ma"])
    print(f"   largest servo current during the ramp {peak:.0f} mA")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recording", nargs="*", help="lean_test recordings to compare with the model (their kp, kd and rate are used)")
    ap.add_argument("--com-mm", default="0", help="whole-robot centre of mass shifts to simulate, mm forwards, comma separated")
    ap.add_argument("--kp", type=float, default=4.0, help="N.m/rad, as lean_test's --kp")
    ap.add_argument("--kd", type=float, default=0.05)
    ap.add_argument("--friction", default="0.02", help="joint friction loss, N.m, comma separated to sweep")
    ap.add_argument("--rate", type=float, default=1.0, help="lean rate, deg/s")
    ap.add_argument("--settle", type=float, default=2.0, help="seconds standing at home before the ramp")
    ap.add_argument("--max-deg", type=float, default=30.0)
    ap.add_argument("--heel-mm", type=float, default=0.0, help="shorten the soles at the heel, mm")
    ap.add_argument("--backlash", action="store_true", help="the gear-play model (±1° after every servo)")
    ap.add_argument("--save", help="write each simulated run as a recording, PREFIX_<com>_<back|fwd>.json")
    args = ap.parse_args()

    if args.recording:
        first = json.load(open(args.recording[0]))
        kp, kd, rate = first["kp"], first["kd"], first["rate_deg_s"]
        model = Model(kp=kp, kd=kd)
        lean_signs(model)
        baseline = {w: simulate(Model(kp=kp, kd=kd), s, rate, args.settle, args.max_deg)
                    for w, s in (("back", -1.0), ("fwd", 1.0))}
        print(f"simulation at kp {kp} kd {kd}, {rate} deg/s: tips back at ankles {baseline['back']['ankle_deg']:+.2f} deg, "
              f"IMU {baseline['back']['lean']:+.2f}; forwards at {baseline['fwd']['ankle_deg']:+.2f}, IMU {baseline['fwd']['lean']:+.2f}")
        results = [r for r in (from_recording(model, p, baseline) for p in args.recording) if r]
        for tag in ("legs", "imu"):
            back = [r[tag] for r in results if r["back"]]
            fwd = [r[tag] for r in results if not r["back"]]
            if back and fwd:
                # Backwards gives c - s, forwards c + s (s: how much shorter each end of the real
                # sole is than the model's, in effect), so the mean of the two is c alone.
                cb, cf = float(np.mean(back)), float(np.mean(fwd))
                print(f"both ways, tilt from the {tag}: robot's centre of mass {(cb + cf) / 2:+.1f} mm ahead of the model's, "
                      f"each sole end {(cf - cb) / 2:+.1f} mm shorter in effect ({len(back)} back, {len(fwd)} forwards)")
        return

    model = Model(kp=args.kp, kd=args.kd, heel_mm=args.heel_mm, backlash=args.backlash)
    lean_signs(model)
    st = model.stance(model.home)
    print(f"model at home, soles flat: centre of mass {st['ahead_of_heel_mm']:.1f} mm ahead of the heel edge, "
          f"{st['behind_toe_mm']:.1f} mm behind the toe edge, {st['com_height_mm']:.0f} mm up; trunk lean {st['lean_deg']:+.2f} deg")
    print(f"kp {args.kp} kd {args.kd}, lean {args.rate} deg/s after {args.settle} s at home")
    for fr in [float(x) for x in args.friction.split(",")]:
        for com in [float(x) for x in args.com_mm.split(",")]:
            for direction, word in ((-1.0, "back"), (1.0, "fwd ")):
                model = Model(com_mm=com, kp=args.kp, kd=args.kd, friction=fr, heel_mm=args.heel_mm,
                              backlash=args.backlash)
                r = simulate(model, direction, args.rate, args.settle, args.max_deg)
                head = f"friction {fr:.3f} com {com:+5.1f} mm {word}:"
                if not r["tipped"]:
                    print(f"{head} no tip by {r['delta']:+.1f} deg (settled lean {r['settled_lean']:+.2f})")
                    continue
                if args.save:
                    save_recording(f"{args.save}_{com:+.1f}_{word.strip()}.json", r, args.kp, args.kd, args.rate)
                edge = r["ahead_of_heel_mm"] if direction < 0 else r["behind_toe_mm"]
                print(f"{head} tips at ankle command {r['delta']:+6.2f} deg, ankle reads {r['ankle_deg']:+6.2f}, "
                      f"IMU lean {r['lean']:+6.2f} deg (settled at home {r['settled_lean']:+5.2f}); "
                      f"model edge check {edge:+.1f} mm")


if __name__ == "__main__":
    main()
