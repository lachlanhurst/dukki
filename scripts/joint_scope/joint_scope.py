"""Live view of the real robot's joints on the MuJoCo model, with every servo limp.

Pairs with `joint_scope` on the robot (microduck `duck-control/examples/joint_scope.rs`),
which puts every J288 in FOC with zero gains and streams what each one reports. Move a joint
of the real robot by hand and the matching joint of the model moves with it. Three uses:

    IDs         The joint being moved is picked out by name, servo ID and bus segment, and
                highlighted on the model. If the model moves a different joint, that servo's
                ID is wrong.
    Directions  If the model's joint turns the other way to the real one, `f` flips that
                joint's sign. The sign is what the firmware will need to map servo angles to
                the model's.
    Ranges      Each joint's travel is recorded as it is moved, so easing a joint to where it
                first touches something records that end. `r` clears a joint's record after a
                bump.

Angles. Each servo reports a multi-turn position, smooth but relative to where it powered up,
and a 13-bit absolute output encoder, only trustworthy at rest. Once a servo has been at rest,
the difference between the two anchors it: position plus that anchor is an absolute angle in
the servo's own frame (rad, unwrapped from the first reading) that follows motion smoothly. The
anchor is checked at every later rest and any disagreement shown, which would mean the
multi-turn position drifts against the encoder.

Joint zeros are not known until the calibration jig, so the model is shown with the pose at
connection (or at the last `z`) as the model's zero. Lay the robot out straight by eye first
and the model will look about right. The saved ranges are in encoder counts too, so they stay
valid once the jig gives the true zeros.

Calibration. With the robot on its back in the calibration jig (every joint at the model's zero,
servos limp, nothing touching it), `k` reads every joint's output encoder over a second at rest:
those counts are the zeros. The mouth is not set by the jig: hold it closed against its stop and
`m` reads it, and its zero is put so the closed stop reads MOUTH_CLOSED_DEG less STOP_MARGIN_DEG
(-8 degrees), which keeps robotd's closed target (-5) off the stop. Once both are read, the
`[bus.bridge_zero]` table for robotd.toml is written to `--zero-out`, and the measured travel
(`--travel`) restated against the new zeros, as duck-control's `JOINT_TRAVEL_DEG`, beside it.
A capture is refused if any joint moves during it.

Commands, typed into this terminal (a joint is the one being moved unless named or given by ID):

    z           take the current pose as every joint's zero (robot laid out straight)
    f [joint]   flip a joint's direction
    c [joint]   mark a joint's ID and direction as confirmed
    r [joint]   clear a joint's recorded range
    k           capture every joint's zero, robot in the calibration jig
    m           capture the mouth's zero, mouth held closed
    p           print the table
    s           save now (also saved every 10 s and on quit)
    q           quit

Run on the Mac, with joint_scope running on the robot:

    uv run --with mujoco mjpython scripts/joint_scope/joint_scope.py \\
        --mjcf ../microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml

`--demo` runs without a robot, on made-up data; `--no-viewer` without the window.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# The firmware's joint order: servo ID j drives JOINT_NAMES[j] (duck-ipc-proto).
JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
SEGMENTS = "ABCD"
COUNTS = 8192
TWO_PI = 2.0 * math.pi

# A servo counts as at rest below this speed, for anchoring.
REST_SPEED = 0.03  # rad/s
# Rest readings averaged into the first anchor.
ANCHOR_SAMPLES = 10
# Speed that marks the joint being moved.
MOVING_SPEED = 0.15  # rad/s
# How long a joint stays the one being moved after it stops.
ACTIVE_HOLD = 3.0  # s
HIGHLIGHT = np.array([1.0, 0.55, 0.1, 1.0], dtype=np.float32)

# A zero capture: this many frames (a second at 50 Hz), every joint slower than this throughout.
CAPTURE_FRAMES = 50
CAPTURE_REST = 0.05  # rad/s
MOUTH = JOINT_NAMES.index("mouth")
# duck-control's model::MOUTH_CLOSED and model::STOP_MARGIN_DEG.
MOUTH_CLOSED_DEG = -5.0
STOP_MARGIN_DEG = 3.0


def wrap(a: float) -> float:
    """To (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def deg(a: float | None) -> str:
    return "    -  " if a is None else f"{math.degrees(a):+7.1f}"


@dataclass
class Joint:
    id: int
    name: str
    sign: int = 1
    confirmed: bool = False
    segment: int | None = None
    mode: int | None = None
    temp: int | None = None
    error: int = 0
    # Absolute angle = pos + anchor, rad, servo frame.
    anchor: float | None = None
    anchor_rest: list[float] = field(default_factory=list)
    # Latest disagreement between the anchor and a rest reading, rad.
    anchor_err: float = 0.0
    abs_angle: float | None = None
    vel: float = 0.0
    zero: float | None = None
    lo: float | None = None
    hi: float | None = None
    last_moving: float = -1e9

    def angle(self) -> float | None:
        """Joint angle in the model's frame, relative to the eyeballed zero."""
        if self.abs_angle is None or self.zero is None:
            return None
        return self.sign * (self.abs_angle - self.zero)

    def range(self) -> tuple[float | None, float | None]:
        """Recorded travel in the model's frame, low then high."""
        if self.lo is None or self.zero is None:
            return None, None
        a, b = self.sign * (self.lo - self.zero), self.sign * (self.hi - self.zero)
        return min(a, b), max(a, b)


class Scope:
    """Everything but the window: anchors, zeros, ranges and the joint being moved."""

    def __init__(self, mjcf_ranges: dict[str, tuple[float, float]]):
        self.joints = [Joint(i, n) for i, n in enumerate(JOINT_NAMES)]
        self.mjcf_ranges = mjcf_ranges
        self.active: int | None = None
        self.limp: bool | None = None
        self.pack_mv = 0
        self.t = 0.0
        self.frames = 0
        self.zeroed = False
        # Zero capture: what is being captured ("jig" or "mouth") and the frames so far.
        self.capturing: str | None = None
        self.captured: list[dict] = []
        self.jig_counts: dict[int, int] | None = None
        self.mouth_closed: int | None = None
        # Lines for the terminal, from captures that finish between commands.
        self.messages: list[str] = []
        # Set by main: where a finished calibration goes, and the measured travel to restate.
        self.zero_out: Path | None = None
        self.travel: dict | None = None

    def update(self, frame: dict) -> None:
        if self.capturing is not None:
            self.captured.append(frame)
            if len(self.captured) >= CAPTURE_FRAMES:
                self.messages.append(self.finish_capture())
        self.frames += 1
        self.t = frame["t"]
        self.limp = frame.get("limp")
        self.pack_mv = frame.get("pack_mv", 0)
        now = time.monotonic()
        for s in frame["servos"]:
            j = self.joints[s["id"]]
            j.segment = s.get("seg")
            if j.segment is None:
                continue
            j.mode, j.temp, j.error = s["mode"], s["temp"], s["err"]
            pos, vel = s["pos"], s["vel"]
            j.vel = vel
            enc = s["enc"] * TWO_PI / COUNTS
            if abs(vel) < REST_SPEED:
                # enc = pos + anchor (mod 2 pi) at rest.
                est = enc - pos
                if j.anchor is None:
                    j.anchor_rest.append(est)
                    if len(j.anchor_rest) >= ANCHOR_SAMPLES:
                        ref = j.anchor_rest[0]
                        j.anchor = ref + float(np.median([wrap(e - ref) for e in j.anchor_rest]))
                        # So that the first absolute angle lies in [0, 2 pi), as the encoder does.
                        j.anchor -= TWO_PI * math.floor((pos + j.anchor) / TWO_PI)
                else:
                    j.anchor_err = wrap(est - j.anchor)
            if j.anchor is None:
                continue
            j.abs_angle = pos + j.anchor
            if j.zero is None:
                j.zero = j.abs_angle
            j.lo = j.abs_angle if j.lo is None else min(j.lo, j.abs_angle)
            j.hi = j.abs_angle if j.hi is None else max(j.hi, j.abs_angle)
            if abs(vel) > MOVING_SPEED:
                j.last_moving = now
        moving = [j for j in self.joints if now - j.last_moving < ACTIVE_HOLD]
        if moving:
            fastest = max(moving, key=lambda j: (now - j.last_moving < 0.3, abs(j.vel)))
            if self.active is None or now - self.joints[self.active].last_moving > 0.3:
                self.active = fastest.id
        elif self.active is not None and now - self.joints[self.active].last_moving >= ACTIVE_HOLD:
            self.active = None

    def pick(self, word: str | None) -> Joint | None:
        if word is None:
            return None if self.active is None else self.joints[self.active]
        if word.isdigit() and int(word) < len(self.joints):
            return self.joints[int(word)]
        hits = [j for j in self.joints if j.name == word] or [j for j in self.joints if word in j.name]
        return hits[0] if len(hits) == 1 else None

    def command(self, text: str) -> str:
        words = text.split()
        if not words:
            return ""
        verb, arg = words[0], (words[1] if len(words) > 1 else None)
        if verb in ("k", "m"):
            self.capturing = "jig" if verb == "k" else "mouth"
            self.captured = []
            return f"capturing the {'jig zeros' if verb == 'k' else 'closed mouth'}: hold still for a second"
        if verb == "z":
            for j in self.joints:
                if j.abs_angle is not None:
                    j.zero = j.abs_angle
            self.zeroed = True
            return "every joint zeroed at the current pose"
        if verb in ("f", "c", "r"):
            j = self.pick(arg)
            if j is None:
                return "no joint: move one, or name it (name, part of a name, or servo ID)"
            if verb == "f":
                j.sign = -j.sign
                return f"{j.name}: direction flipped, sign now {j.sign:+d}"
            if verb == "c":
                j.confirmed = not j.confirmed
                return f"{j.name}: {'confirmed' if j.confirmed else 'unconfirmed'}"
            j.lo = j.hi = j.abs_angle
            return f"{j.name}: range cleared"
        return f"unknown command {verb!r}"

    def finish_capture(self) -> str:
        what, frames = self.capturing, self.captured
        self.capturing, self.captured = None, []
        ids = [MOUTH] if what == "mouth" else [i for i in range(len(JOINT_NAMES)) if i != MOUTH]
        counts, lines, problems = {}, [], []
        for i in ids:
            servos = [f["servos"][i] for f in frames]
            if any(s.get("seg") is None for s in servos):
                problems.append(f"{JOINT_NAMES[i]} absent")
                continue
            moving = max(abs(s["vel"]) for s in servos)
            if moving > CAPTURE_REST:
                problems.append(f"{JOINT_NAMES[i]} moved ({moving:.2f} rad/s)")
            a = np.array([s["enc"] for s in servos]) * TWO_PI / COUNTS
            mean = math.atan2(np.sin(a).sum(), np.cos(a).sum())
            spread = max(abs(wrap(x - mean)) for x in a)
            counts[i] = int(round(mean * COUNTS / TWO_PI)) % COUNTS
            lines.append(f"  {JOINT_NAMES[i]:<16} {counts[i]:>5}   spread {math.degrees(spread):.2f} deg")
        if problems:
            return "capture refused, try again: " + ", ".join(problems)
        if what == "mouth":
            self.mouth_closed = counts[MOUTH]
        else:
            self.jig_counts = counts
        out = [f"captured the {'jig zeros' if what == 'jig' else 'closed mouth'}:"] + lines
        zeros = self.zeros()
        if zeros is None:
            out.append("  now " + ("hold the mouth closed and type m" if what == "jig" else "put the robot in the jig and type k"))
        else:
            out.append(self.write_calibration(zeros))
        return "\n".join(out)

    def zeros(self) -> list[int] | None:
        """Every joint's zero in counts, once the jig and the mouth are both captured."""
        if self.jig_counts is None or self.mouth_closed is None:
            return None
        # The closed stop reads MOUTH_CLOSED_DEG - STOP_MARGIN_DEG: angle = encoder - zero.
        mouth = self.mouth_closed + round((STOP_MARGIN_DEG - MOUTH_CLOSED_DEG) / 360.0 * COUNTS)
        return [mouth % COUNTS if i == MOUTH else self.jig_counts[i] for i in range(len(JOINT_NAMES))]

    def write_calibration(self, zeros: list[int]) -> str:
        if self.zero_out is None:
            return "  (no --zero-out, nothing written)"
        self.zero_out.parent.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%d/%m/%Y %H:%M")
        body = [f"# Joint zeros captured {stamp} by scripts/joint_scope (robot in the calibration jig,",
                "# mouth against its closed stop). For /etc/robot/robotd.toml.",
                "[bus.bridge_zero]"]
        body += [f"{name} = {z}" for name, z in zip(JOINT_NAMES, zeros)]
        self.zero_out.write_text("\n".join(body) + "\n")
        out = [f"  wrote {self.zero_out}"]
        if self.travel is not None:
            rows = [f"// JOINT_TRAVEL_DEG against the zeros captured {stamp}"]
            for name, z in zip(JOINT_NAMES, zeros):
                j = next(t for t in self.travel["joints"] if t["name"] == name)
                shift = math.degrees(wrap((j["zero_counts"] - z) * TWO_PI / COUNTS))
                lo, hi = (v + shift for v in j["range_deg"])
                rows.append(f"    ({lo:.1f}, {hi:.1f}), // {name}, {shift:+.1f} from the eyeballed zero")
            path = self.zero_out.with_name(self.zero_out.stem + "_travel.rs.txt")
            path.write_text("\n".join(rows) + "\n")
            out.append(f"  wrote {path}")
            out += rows[1:]
        return "\n".join(out)

    def table(self) -> str:
        rows = [f"{'ID':>2} {'joint':<16}{'seg':>4}{'mode':>5}{'sign':>5}{'angle':>8}"
                f"{'min':>8}{'max':>8}   {'model range':>15} {'anchor':>7} {'temp':>4}  ok"]
        for j in self.joints:
            lo, hi = j.range()
            mr = self.mjcf_ranges.get(j.name)
            mrs = f"{deg(mr[0])} {deg(mr[1])}" if mr else "      (none)   "
            seg = "-" if j.segment is None else SEGMENTS[j.segment % 4]
            rows.append(
                f"{j.id:>2} {j.name:<16}{seg:>4}{'' if j.mode is None else j.mode:>5}{j.sign:>+5d}"
                f"{deg(j.angle())}{deg(lo)}{deg(hi)}   {mrs} {math.degrees(j.anchor_err):+7.2f}"
                f" {'' if j.temp is None else j.temp:>4}  {'yes' if j.confirmed else ''}"
            )
        return "\n".join(rows)

    def overlay(self) -> list[tuple[str, str]]:
        """(left, right) text blocks for the viewer."""
        if self.active is None:
            head = ("Move a joint", "")
        else:
            j = self.joints[self.active]
            lo, hi = j.range()
            head = (
                f"{j.name}\nservo ID {j.id}, segment {SEGMENTS[(j.segment or 0) % 4]}\nsign {j.sign:+d}"
                f"{'  confirmed' if j.confirmed else ''}",
                f"{deg(j.angle())} deg\nrange {deg(lo)} to {deg(hi)}\n",
            )
        left, right = [], []
        for j in self.joints:
            lo, hi = j.range()
            mark = ">" if j.id == self.active else " "
            left.append(f"{mark}{j.id:>2} {j.name}{'  ok' if j.confirmed else ''}")
            if j.segment is None:
                right.append("absent")
            elif j.anchor is None:
                right.append("hold still to anchor")
            else:
                right.append(f"{deg(j.angle())}   [{deg(lo)} {deg(hi)}]")
        status = "limp" if self.limp else "BRAKED" if self.limp is False else "no data"
        pack = f", pack {self.pack_mv / 1000:.1f} V" if self.pack_mv else ""
        left.append(f"\n{status}{pack}")
        right.append("\n" + ("" if self.zeroed else "zero = pose at start (z to re-zero)"))
        return [head, ("\n".join(left), "\n".join(right))]

    def to_json(self) -> dict:
        out = []
        for j in self.joints:
            lo, hi = j.range()
            mr = self.mjcf_ranges.get(j.name)
            counts = (lambda a: None if a is None else int(round(a * COUNTS / TWO_PI)) % COUNTS)
            out.append({
                "id": j.id,
                "name": j.name,
                "segment": None if j.segment is None else SEGMENTS[j.segment % 4],
                "sign": j.sign,
                "confirmed": j.confirmed,
                # Encoder counts (0 to 8191) of the eyeballed zero and of the two ends of the
                # recorded travel, in the servo's own frame: valid against the jig's zeros.
                "zero_counts": counts(j.zero),
                "travel_counts": None if j.lo is None else [counts(j.lo), counts(j.hi)],
                "travel_span_deg": None if j.lo is None else round(math.degrees(j.hi - j.lo), 2),
                # The same travel in the model's frame, against the eyeballed zero.
                "range_deg": None if lo is None else [round(math.degrees(lo), 2), round(math.degrees(hi), 2)],
                "model_range_deg": None if mr is None else [round(math.degrees(mr[0]), 2), round(math.degrees(mr[1]), 2)],
                "anchor_disagreement_deg": round(math.degrees(j.anchor_err), 3),
            })
        return {
            "saved": time.strftime("%d/%m/%Y %H:%M:%S"),
            "zero": "eyeballed: the pose at 'z'" if self.zeroed else "eyeballed: the pose at connection",
            "joints": out,
        }


def load_model(path: Path):
    import mujoco

    spec = mujoco.MjSpec.from_file(str(path))
    # A light grey background: the shells are dark grey and vanish against MuJoCo's black.
    spec.add_texture(name="joint_scope_sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT, rgb1=[0.8, 0.8, 0.8],
                     rgb2=[0.8, 0.8, 0.8], width=64, height=64)
    model = spec.compile()
    model.vis.headlight.ambient[:] = 0.35
    model.vis.headlight.diffuse[:] = 0.7
    data = mujoco.MjData(model)
    qadr, ranges, bodies = {}, {}, {}
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            continue
        qadr[name] = model.jnt_qposadr[jid]
        if model.jnt_limited[jid]:
            ranges[name] = tuple(float(x) for x in model.jnt_range[jid])
        bodies[name] = model.jnt_bodyid[jid]
    # Hold the trunk up, level, clear of anything; nothing is simulated, only posed.
    free = [i for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE]
    if free:
        a = model.jnt_qposadr[free[0]]
        data.qpos[a:a + 7] = [0, 0, 0.3, 1, 0, 0, 0]
    mujoco.mj_kinematics(model, data)
    return model, data, qadr, ranges, bodies


def stream(host: str, port: int, frames: "queue.Queue[dict]", stop: threading.Event) -> None:
    """Newline-delimited JSON from the robot, reconnecting until stopped."""
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=3) as sock:
                sock.settimeout(1.0)
                print(f"connected to {host}:{port}")
                buf = b""
                while not stop.is_set():
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("robot closed the stream")
                    buf += chunk
                    *lines, buf = buf.split(b"\n")
                    for line in lines:
                        if line.strip():
                            frames.put(json.loads(line))
        except (OSError, ConnectionError, json.JSONDecodeError) as e:
            if not stop.is_set():
                print(f"stream: {e}; retrying")
                time.sleep(1.0)


def demo(frames: "queue.Queue[dict]", stop: threading.Event) -> None:
    """Made-up frames: one joint at a time swung through a range, the others still."""
    rng = np.random.default_rng(1)
    enc0 = rng.integers(0, COUNTS, len(JOINT_NAMES))
    power_up = rng.uniform(-0.2, 0.2, len(JOINT_NAMES))  # where the multi-turn count started
    t0 = time.monotonic()
    pos = np.zeros(len(JOINT_NAMES))
    prev = pos.copy()
    while not stop.is_set():
        t = time.monotonic() - t0
        k = int(t // 6) % len(JOINT_NAMES)
        phase = (t % 6) - 1.5
        pos[:] = 0.0
        if 0 < phase < 4:
            pos[k] = 0.6 * math.sin(math.pi * phase / 2)
        vel = (pos - prev) * 50
        prev = pos.copy()
        servos = []
        for i in range(len(JOINT_NAMES)):
            enc = int(enc0[i] + round(pos[i] * COUNTS / TWO_PI)) % COUNTS
            servos.append({"id": i, "seg": [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2][i],
                           "mode": 1, "pos": float(pos[i] + power_up[i]), "vel": float(vel[i]),
                           "tq": 0.0, "enc": enc, "temp": 30, "err": 0, "age": 400})
        frames.put({"t": t, "limp": True, "flags": 2, "pack_mv": 24000, "servos": servos})
        time.sleep(0.02)


def commands(lines: "queue.Queue[str]", stop: threading.Event) -> None:
    for line in sys.stdin:
        lines.put(line.strip())
        if line.strip() == "q":
            break
    stop.set()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mjcf", type=Path,
                   default=Path("../microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml"))
    p.add_argument("--host", default="microduck.local")
    p.add_argument("--port", type=int, default=7788)
    p.add_argument("--out", type=Path, default=Path("scratch/joint_scope/joint_scope.json"))
    p.add_argument("--zero-out", type=Path, default=Path("scratch/joint_scope/bridge_zero.toml"),
                   help="where k and m write the [bus.bridge_zero] table")
    p.add_argument("--travel", type=Path, default=Path("hardware/calibration/joint_travel.json"),
                   help="measured travel to restate against the captured zeros")
    p.add_argument("--demo", action="store_true", help="made-up data, no robot")
    p.add_argument("--no-viewer", action="store_true", help="terminal only")
    p.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0: run until q)")
    args = p.parse_args()

    model, data, qadr, ranges, bodies = load_model(args.mjcf)
    scope = Scope(ranges)
    scope.zero_out = args.zero_out
    if args.travel.exists():
        scope.travel = json.loads(args.travel.read_text())
    frames: queue.Queue[dict] = queue.Queue()
    lines: queue.Queue[str] = queue.Queue()
    stop = threading.Event()
    source = demo if args.demo else stream
    src_args = (frames, stop) if args.demo else (args.host, args.port, frames, stop)
    threading.Thread(target=source, args=src_args, daemon=True).start()
    threading.Thread(target=commands, args=(lines, stop), daemon=True).start()
    args.out.parent.mkdir(parents=True, exist_ok=True)

    def save() -> None:
        args.out.write_text(json.dumps(scope.to_json(), indent=1) + "\n")

    import mujoco

    rgba0 = model.geom_rgba.copy()
    def subtree(root: int) -> set[int]:
        out = {root}
        for b in range(root + 1, model.nbody):
            if model.body_parentid[b] in out:
                out.add(b)
        return out

    # Everything a joint moves: its body and every body below it.
    body_geoms = {n: np.flatnonzero(np.isin(model.geom_bodyid, list(subtree(b)))) for n, b in bodies.items()}

    def pose() -> None:
        for j in scope.joints:
            if j.name in qadr:
                a = j.angle()
                data.qpos[qadr[j.name]] = 0.0 if a is None else a
        mujoco.mj_kinematics(model, data)
        model.geom_rgba[:] = rgba0
        if scope.active is not None and JOINT_NAMES[scope.active] in body_geoms:
            model.geom_rgba[body_geoms[JOINT_NAMES[scope.active]]] = HIGHLIGHT

    def step() -> bool:
        while True:
            try:
                scope.update(frames.get_nowait())
            except queue.Empty:
                break
        while scope.messages:
            print(scope.messages.pop(0))
        while True:
            try:
                text = lines.get_nowait()
            except queue.Empty:
                break
            if text == "q":
                return False
            if text == "p":
                print(scope.table())
            elif text == "s":
                save()
                print(f"saved {args.out}")
            elif text:
                print(scope.command(text))
        return not stop.is_set()

    print(__doc__.split("Commands,")[1].split("Run on the Mac")[0].strip())
    start = time.monotonic()
    saved = time.monotonic()
    viewer = None
    if not args.no_viewer:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.6, -15, 140
        viewer.cam.lookat[:] = data.xpos[1:].mean(axis=0)
    try:
        while step() and (viewer is None or viewer.is_running()):
            if args.seconds and time.monotonic() - start > args.seconds:
                break
            if viewer is not None:
                with viewer.lock():
                    pose()
                # set_texts takes the viewer's lock itself, so it must not be called under it.
                (h1, h2), (t1, t2) = scope.overlay()
                viewer.set_texts([
                    (mujoco.mjtFontScale.mjFONTSCALE_200, mujoco.mjtGridPos.mjGRID_TOPLEFT, h1, h2),
                    (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, t1, t2),
                ])
                viewer.sync()
            if time.monotonic() - saved > 10 and scope.frames:
                save()
                saved = time.monotonic()
            time.sleep(1 / 60)
    finally:
        stop.set()
        if scope.frames:
            save()
            print(f"saved {args.out}")
        if viewer is not None:
            viewer.close()
    print(scope.table())


if __name__ == "__main__":
    main()
