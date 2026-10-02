# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Exploratory J288 bench tests, run before building the identification rig.

Each test logs every command/reply pair to a JSON file in --logdir. Every
frame is sent with timeout protection on, so the servo stops by itself about
1 s after this script dies, and the script sends stop on exit or Ctrl-C.

Tests:
    hand      Interactive. Turn the output by hand through three phases:
              mode 0, mode 1 with zero torque, and mode 1 damping.
    ramp      Slow torque ramp in each direction to find breakaway torque.
    steps     Short torque steps, braked by damping, for acceleration and the
              speed envelope.
    position  Position steps at Unitree's example gains.
    stall     Interactive. Arm pressed onto a scale at increasing torque; the
              operator types in each scale reading.

    uv run --with pyserial --with-editable ../bam scripts/j288/explore.py hand --port /dev/cu.usbmodemXXXX
"""

import argparse
import datetime
import json
import os
import threading
import time

import numpy as np

from bam.unitree.digital_servo import MODE_FOC, MODE_STOP, DigitalServo


class Session:
    """Runs a servo at a fixed loop rate, logs everything, and enforces a
    speed limit (switches to damping to brake, then stops)."""

    def __init__(self, args, test: str):
        self.servo = DigitalServo(args.port, args.id)
        self.dt = 1.0 / args.rate
        self.speed_limit = args.speed_limit
        self.logdir = args.logdir
        self.test = test
        self.entries = []
        self.meta = {}
        self.t0 = time.perf_counter()
        self.state = self.recover()
        if self.state["error"]:
            raise RuntimeError(f"servo reports fault {self.state['error']:#x}")
        self._q_prev, self._t_prev = self.state["position"], self.now()
        self.speed_est = 0.0

    def recover(self):
        """Clear a latched timeout (see DigitalServo.recover)."""
        return self.servo.recover()

    def now(self) -> float:
        return time.perf_counter() - self.t0

    def step(
        self, phase: str, mode: int, torque=0.0, speed=0.0, position=0.0, kp=0.0, kd=0.0
    ):
        """Send one command, log it with its reply, and hold the loop rate."""
        t_send = self.now()
        state = self.servo.command(mode, torque, speed, position, kp, kd)
        t_recv = self.now()
        if state is not None:
            self.state = state
            # The servo's own speed lags ~15 ms behind the motion (first-order
            # filter), so the safety check uses a lightly filtered difference
            # of position instead.
            t = (t_send + t_recv) / 2.0
            if t > self._t_prev:
                raw = (state["position"] - self._q_prev) / (t - self._t_prev)
                self.speed_est += 0.5 * (raw - self.speed_est)
            self._q_prev, self._t_prev = state["position"], t
            self.entries.append(
                {
                    "timestamp": (t_send + t_recv) / 2.0,
                    "phase": phase,
                    "cmd_mode": mode,
                    "cmd_torque": torque,
                    "cmd_speed": speed,
                    "cmd_position": position,
                    "kp": kp,
                    "kd": kd,
                    **{k: float(v) for k, v in state.items()},
                }
            )
            if state["error"]:
                raise RuntimeError(f"servo fault {int(state['error']):#x}")
            if (
                abs(self.speed_est) > self.speed_limit
                and mode == MODE_FOC
                and kp == 0.0
                and kd == 0.0
            ):
                raise SpeedLimit(self.speed_est)
        while self.now() - t_send < self.dt:
            pass
        return state

    def hold(self, phase: str, duration: float, **cmd):
        end = self.now() + duration
        while self.now() < end:
            self.step(phase, **cmd)

    def brake(self, kd=0.02, duration=1.0):
        """Damping until nearly still, then stop."""
        end = self.now() + duration
        while self.now() < end and abs(self.speed_est) > 0.05:
            self.step("brake", MODE_FOC, kd=kd)
        self.hold("brake", 0.2, mode=MODE_FOC, kd=kd)
        self.step("stop", MODE_STOP)

    def save(self, meta: dict):
        os.makedirs(self.logdir, exist_ok=True)
        date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
        filename = f"{self.logdir}/{self.test}_{date}.json"
        data = {
            "test": self.test,
            "rate": 1.0 / self.dt,
            **meta,
            **self.meta,
            "entries": self.entries,
        }
        with open(filename, "w") as f:
            json.dump(data, f)
        print(f"saved {len(self.entries)} entries to {filename}")
        return filename

    def close(self):
        for _ in range(3):
            self.servo.stop()
        self.servo.ser.close()


class SpeedLimit(Exception):
    pass


def countdown(s: Session, message: str, seconds: int = 3):
    """Count down while sending stop frames, so the timeout never trips."""
    print(message)
    for i in range(seconds, 0, -1):
        print(f"  {i}...", flush=True)
        s.hold("idle", 1.0, mode=MODE_STOP)


def summarise(entries, key="phase"):
    phases = []
    for e in entries:
        if not phases or phases[-1][0] != e[key]:
            phases.append((e[key], []))
        phases[-1][1].append(e)
    for name, es in phases:
        speed = np.array([e["speed"] for e in es])
        torque = np.array([e["torque"] for e in es])
        pos = np.array([e["position"] for e in es])
        print(
            f"  {name:22s} n={len(es):6d}  max|speed| {np.abs(speed).max():6.2f} rad/s  "
            f"torque min/max {torque.min():+.3f}/{torque.max():+.3f} N.m  "
            f"travel {pos.max() - pos.min():6.2f} rad"
        )


def test_hand(s: Session, args):
    phases = [
        ("mode0_stop", dict(mode=MODE_STOP), "mode 0 (stop)"),
        ("mode1_zero", dict(mode=MODE_FOC), "mode 1, zero torque, kp = kd = 0"),
        (
            "mode1_damping",
            dict(mode=MODE_FOC, kd=args.kd),
            f"mode 1, damping kd = {args.kd} N.m.s/rad",
        ),
    ]
    print(
        f"Turn the arm by hand, both directions, slow then fast, for {args.duration:.0f} s per phase."
    )
    print("Note how each phase feels: free, notchy, braked, viscous.\n")
    for name, cmd, label in phases:
        countdown(s, f"Next: {label}. Hands on the arm.")
        print(f"  GO: {label}", flush=True)
        s.hold(name, args.duration, **cmd)
        s.step("stop", MODE_STOP)
        print("  done, hands off.\n")
        s.hold("idle", 1.0, mode=MODE_STOP)


def test_ramp(s: Session, args):
    """Torque ramps from zero at args.ramp_rate N.m/s until the arm moves,
    then brake. Once per direction, starting from rest."""
    for sign in (+1, -1):
        name = "ramp_pos" if sign > 0 else "ramp_neg"
        s.hold("settle", 1.0, mode=MODE_STOP)
        start = s.now()
        moved_at = None
        while True:
            torque = sign * args.ramp_rate * (s.now() - start)
            if abs(torque) > args.max_torque:
                print(f"  {name}: reached {args.max_torque} N.m without moving")
                break
            try:
                state = s.step(name, MODE_FOC, torque=torque)
            except SpeedLimit as e:
                print(f"  {name}: speed limit at {e.args[0]:+.2f} rad/s")
                break
            if moved_at is None and abs(state["speed"]) > args.move_speed:
                moved_at = s.now()
                print(
                    f"  {name}: moving at commanded {torque:+.4f} N.m (reported {state['torque']:+.4f})"
                )
            if moved_at is not None and s.now() - moved_at > args.after_move:
                break
        s.brake()


def test_steps(s: Session, args):
    for torque in args.torques:
        for sign in (+1, -1):
            name = f"step_{sign * torque:+.2f}"
            s.hold("settle", 1.0, mode=MODE_STOP)
            try:
                s.hold(name, args.step_time, mode=MODE_FOC, torque=sign * torque)
            except SpeedLimit as e:
                print(f"  {name}: speed limit at {e.args[0]:+.2f} rad/s")
            s.brake()


def test_position(s: Session, args):
    q0 = s.state["position"]
    kp, kd = args.kp, args.kd
    # Hold where we are, then step out and back.
    s.hold("hold", 1.0, mode=MODE_FOC, position=q0, kp=kp, kd=kd)
    for delta in args.deltas:
        s.hold(
            f"to_{delta:+.2f}",
            args.step_time,
            mode=MODE_FOC,
            position=q0 + delta,
            kp=kp,
            kd=kd,
        )
        s.hold("back", args.step_time, mode=MODE_FOC, position=q0, kp=kp, kd=kd)
    s.step("stop", MODE_STOP)


def test_stall(s: Session, args):
    """Arm pressed onto a scale. Hold increasing torques into the scale; the
    operator reads the scale at each level and types it in."""
    lever, sign = args.lever, args.sign
    print(f"Lever {lever * 1000:.0f} mm, torque sign {sign:+d} pushes into the scale.")

    # Approach: a small torque towards the scale until the arm stops moving,
    # with a travel limit in case the sign is wrong or the scale is missing.
    s.recover()
    q0 = s.state["position"]
    still_since = None
    print("Approaching the scale...")
    while True:
        s.step("approach", MODE_FOC, torque=sign * args.approach_torque)
        travel = s.state["position"] - q0
        if abs(travel) > args.approach_travel or travel * sign < -0.02:
            s.brake(kd=0.05)
            raise RuntimeError(
                f"arm moved {travel:+.3f} rad without meeting the scale; "
                "check --sign and the scale position"
            )
        if abs(s.speed_est) < 0.02:
            still_since = still_since or s.now()
            if s.now() - still_since > 0.3:
                break
        else:
            still_since = None
    s.hold("contact", 0.5, mode=MODE_FOC, torque=sign * args.approach_torque)
    s.step("stop", MODE_STOP)
    print(f"  contact after {abs(s.state['position'] - q0) * 1000:.0f} mrad of travel.")
    s.hold("idle", 0.2, mode=MODE_STOP)
    input("Tare the scale now (arm touching it, servo relaxed), then Enter: ")

    # Stored up front so the readings are saved even if the test is interrupted.
    results = []
    s.meta["stall"] = {"lever": lever, "sign": sign, "results": results}
    for level in args.levels:
        answer = input(f"\nEnter to apply {level:.2f} N.m (q to quit): ")
        if answer.strip().lower() == "q":
            break
        s.recover()
        q_start = s.state["position"]
        name = f"stall_{level:.2f}"

        # Hold the torque while the operator reads the scale; the reading is
        # typed in on another thread so the control loop keeps running.
        reading = []
        reader = threading.Thread(
            target=lambda: reading.append(
                input("  holding: type the steady scale reading in grams, then Enter: ")
            ),
            daemon=True,
        )
        reader.start()
        t_start = s.now()
        stop_reason = None
        while reader.is_alive():
            t = s.now() - t_start
            torque = sign * level * min(1.0, t / 0.3)  # 0.3 s ramp up
            s.step(name, MODE_FOC, torque=torque)
            if abs(s.state["position"] - q_start) > args.max_travel:
                stop_reason = "arm moved too far (slipped or bent?)"
            elif s.state["winding_temp"] > args.max_winding:
                stop_reason = f"winding above {args.max_winding} °C"
            elif t > args.hold:
                stop_reason = f"held {args.hold:.0f} s"
            if stop_reason:
                break
        s.brake(kd=0.05)
        s.hold("idle", 0.2, mode=MODE_STOP)
        if stop_reason:
            print(f"\n  released: {stop_reason}. Type the reading you saw, then Enter.")
        reader.join()

        es = [e for e in s.entries if e["phase"] == name]
        held = [e for e in es if e["timestamp"] - es[0]["timestamp"] > 0.5]
        rep = float(np.mean([e["torque"] for e in held])) if held else float("nan")
        winding = max(e["winding_temp"] for e in es)
        volts = min(e["input_volts"] for e in es)
        travel = max(abs(e["position"] - q_start) for e in es)
        text = reading[0].strip() if reading else ""
        grams = float(text) if text else None
        measured = grams / 1000 * 9.81 * lever if grams is not None else None
        print(
            f"  reported {rep:+.3f} N.m, winding max {winding:.0f} °C, "
            f"supply min {volts:.1f} V, deflection {travel * 1000:.1f} mrad"
        )
        if measured is not None:
            print(
                f"  output torque from scale {measured:.3f} N.m "
                f"({measured / level * 100:.0f}% of command)"
            )
        results.append(
            {
                "cmd": level,
                "scale_g": grams,
                "measured": measured,
                "reported": rep,
                "winding": winding,
                "volts": volts,
                "travel": travel,
                "stop_reason": stop_reason,
            }
        )
        if stop_reason and not stop_reason.startswith("held"):
            print("  stopping the test.")
            break

    print("\n  cmd N.m   scale g   from scale N.m   reported N.m")
    for r in results:
        g = f"{r['scale_g']:8.0f}" if r.get("scale_g") is not None else "       -"
        m = (
            f"{r['measured']:14.3f}"
            if r.get("measured") is not None
            else "             -"
        )
        print(f"  {r['cmd']:7.2f}  {g}  {m}   {r['reported']:+11.3f}")


def analyse_backlash(entries, plateau_above: float, plot_path: str | None):
    """Play from the torque loop, split into inside the servo and outside it.

    With the output clamped, rotor position against reported motor torque is a
    loop with flat plateaus once the play is taken up (the first J288 run,
    02/10/2026, took it up in two steps, at about 0.025 and 0.07 N.m). The play
    is the gap between the plateaus (|torque| > ``plateau_above``), measured
    three ways:

    * rotor-derived position: everything between the motor and the clamp;
    * output encoder (on the output shaft, reliable at rest, as the plateaus
      are): motion of the output shaft itself, i.e. the horn, spline or clamp
      giving;
    * rotor minus output encoder: the play inside the servo (the gearbox).
    """
    es = [e for e in entries if e["phase"].startswith("cycle")]
    if not es:
        return None
    tau = np.array([e["torque"] for e in es])
    q = np.array([e["position"] for e in es])
    enc = np.unwrap(np.array([e["output_encoder"] for e in es]))
    hi, lo = tau > plateau_above, tau < -plateau_above
    if hi.sum() < 50 or lo.sum() < 50:
        print(f"  not enough samples beyond {plateau_above} N.m")
        return None

    def gap(x):
        return float(np.median(x[hi]) - np.median(x[lo]))

    total = gap(q)
    result = {"play_total_rad": total}
    print(f"  rotor travel between plateaus {total * 1000:.1f} mrad ({np.degrees(total):.2f} deg)")

    # Zero-current rests: the encoder reads true, so rotor minus encoder
    # between the two ends is the play inside the servo.
    rests = {}
    for e in entries:
        for side in ("rest_pos", "rest_neg"):
            if e["phase"].startswith(side):
                rests.setdefault((side, e["phase"][len(side):]), []).append(e)
    pairs = sorted({c for _, c in rests})
    splits = []
    for c in pairs:
        if ("rest_pos", c) not in rests or ("rest_neg", c) not in rests:
            continue
        # Skip the first 50 ms of each rest, while the current dies away.
        pos, neg = (rests[(side, c)][50:] for side in ("rest_pos", "rest_neg"))
        enc_unwrap = lambda es: np.unwrap([x["output_encoder"] for x in es])
        rq = np.median([x["position"] for x in pos]) - np.median([x["position"] for x in neg])
        re = np.median(enc_unwrap(pos)) - np.median(enc_unwrap(neg))
        drift = max(np.ptp([x["position"] for x in r]) for r in (pos, neg))
        splits.append((rq, re, drift))
        print(
            f"  cycle {c} at rest: rotor {rq * 1000:5.1f}, output shaft {re * 1000:+5.1f}, "
            f"inside the servo {(rq - re) * 1000:5.1f} mrad (rotor drift in rest {drift * 1000:.2f})"
        )
    if splits:
        a = np.array(splits)
        inside, shaft = float(np.median(a[:, 0] - a[:, 1])), float(np.median(a[:, 1]))
        result.update(
            play_at_rest_rad=float(np.median(a[:, 0])),
            output_shaft_motion_rad=shaft,
            play_inside_servo_rad=inside,
        )
        print(
            f"  inside the servo {inside * 1000:.1f} mrad ({np.degrees(inside):.2f} deg, "
            f"half {inside * 500:.1f} mrad); output shaft {shaft * 1000:+.1f} mrad (horn, spline or clamp)"
        )
    if plot_path:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.plot(tau, (q - np.median(q[lo])) * 1000, lw=0.6, label="rotor-derived position")
        ax.plot(tau, (enc - np.median(enc[lo])) * 1000, lw=0.4, alpha=0.5,
                label="output encoder (disturbed by current)")
        ax.set_xlabel("reported motor torque [N.m]")
        ax.set_ylabel("position from the negative plateau [mrad, output side]")
        title = f"J288 play, output clamped: {total * 1000:.1f} mrad total"
        if "play_inside_servo_rad" in result:
            title += f", {result['play_inside_servo_rad'] * 1000:.1f} inside the servo (at rest)"
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_path, dpi=140)
        print(f"  plot {plot_path}")
    return result


def test_backlash(s: Session, args):
    """Output clamped: slow torque ramps through zero to measure gear play.

    Torque mode (kp = kd = 0). Each cycle ramps 0 -> +A, holds, then switches
    to mode 0 for args.rest s ("rest_pos"), then 0 -> -A, holds, mode 0
    ("rest_neg"); A = args.amplitude N.m, each ramp args.period / 4 s. The
    output encoder is disturbed by the winding current (it drifts with torque
    while the shaft is clamped), so it is only read in the zero-current rests,
    where friction (about 0.025 N.m to break away) keeps the rotor on its side
    of the play. Stops and brakes if the rotor moves more than args.max_travel
    rad from where it started (the clamp slipped, or only stops one direction).
    """
    s.hold("settle", 1.0, mode=MODE_STOP)
    q_start = s.state["position"]
    ramp = args.period / 4
    print(
        f"  ramps to +-{args.amplitude} N.m over {ramp:.1f} s, hold {args.hold_end} s, "
        f"zero-current rest {args.rest} s, {args.cycles} cycles, abort beyond {args.max_travel} rad"
    )
    stop_reason = None

    def checked(phase, **cmd):
        state = s.step(phase, **cmd)
        if state is not None and abs(state["position"] - q_start) > args.max_travel:
            raise RuntimeError(f"rotor moved {state['position'] - q_start:+.3f} rad")
        if state is not None and state["winding_temp"] > args.max_winding:
            raise RuntimeError(f"winding {state['winding_temp']} °C")

    try:
        for cycle in range(args.cycles):
            for sign, rest in ((+1, "rest_pos"), (-1, "rest_neg")):
                t0 = s.now()
                while (t := s.now() - t0) < ramp:
                    checked(f"cycle{cycle}", mode=MODE_FOC, torque=sign * args.amplitude * t / ramp)
                t0 = s.now()
                while s.now() - t0 < args.hold_end:
                    checked(f"cycle{cycle}", mode=MODE_FOC, torque=sign * args.amplitude)
                t0 = s.now()
                while s.now() - t0 < args.rest:
                    checked(f"{rest}{cycle}", mode=MODE_STOP)
    except SpeedLimit as e:
        stop_reason = f"speed limit at {e.args[0]:+.2f} rad/s"
    except RuntimeError as e:
        if "fault" in str(e):
            raise
        stop_reason = str(e)
    if stop_reason:
        print(f"  STOPPED: {stop_reason}")
        s.brake()
    else:
        s.step("stop", MODE_STOP)
    s.meta["stop_reason"] = stop_reason
    plot = os.path.join(
        args.logdir,
        f"backlash_{datetime.datetime.now().strftime('%Y-%m-%d_%Hh%Mm%S')}.png",
    )
    os.makedirs(args.logdir, exist_ok=True)
    s.meta["backlash"] = analyse_backlash(s.entries, args.plateau_above, plot)


TESTS = {
    "hand": test_hand,
    "ramp": test_ramp,
    "steps": test_steps,
    "position": test_position,
    "stall": test_stall,
    "backlash": test_backlash,
}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("test", choices=TESTS)
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--logdir", default="scratch/j288_explore")
    parser.add_argument("--rate", type=float, default=1000.0, help="loop rate [Hz]")
    parser.add_argument(
        "--speed-limit",
        type=float,
        default=10.0,
        help="brake above this in torque tests [rad/s]",
    )
    parser.add_argument(
        "--note", default="", help="free text stored in the log (arm, supply, ...)"
    )
    # hand
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--kd", type=float, default=0.05)
    # ramp
    parser.add_argument("--ramp-rate", type=float, default=0.01, help="[N.m/s]")
    parser.add_argument("--max-torque", type=float, default=0.2)
    parser.add_argument(
        "--move-speed", type=float, default=0.3, help="breakaway threshold [rad/s]"
    )
    parser.add_argument(
        "--after-move",
        type=float,
        default=0.5,
        help="keep ramping this long after moving [s]",
    )
    # steps
    parser.add_argument("--torques", type=float, nargs="+", default=[0.1, 0.2, 0.3])
    parser.add_argument("--step-time", type=float, default=1.0)
    # stall
    parser.add_argument(
        "--lever", type=float, default=0.1, help="pivot to scale contact [m]"
    )
    parser.add_argument(
        "--sign",
        type=int,
        choices=(-1, 1),
        default=1,
        help="torque sign into the scale",
    )
    parser.add_argument("--approach-torque", type=float, default=0.06, help="[N.m]")
    parser.add_argument(
        "--approach-travel", type=float, default=0.8, help="give up after [rad]"
    )
    parser.add_argument(
        "--levels",
        type=float,
        nargs="+",
        default=[0.1, 0.2, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 2.0],
    )
    parser.add_argument(
        "--hold", type=float, default=10.0, help="longest hold per level [s]"
    )
    parser.add_argument(
        "--max-travel",
        type=float,
        default=0.15,
        help="stop if the arm moves this far [rad]",
    )
    parser.add_argument("--max-winding", type=float, default=70.0, help="[°C]")
    # backlash
    parser.add_argument("--amplitude", type=float, default=0.15, help="torque triangle [N.m]")
    parser.add_argument("--period", type=float, default=10.0, help="per triangle [s]")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--hold-end", type=float, default=0.5, help="hold at +-A [s]")
    parser.add_argument("--rest", type=float, default=0.3, help="zero-current rest [s]")
    parser.add_argument(
        "--plateau-above", type=float, default=0.10, help="plateaus beyond [N.m]"
    )
    # position
    parser.add_argument("--kp", type=float, default=0.5)
    parser.add_argument("--deltas", type=float, nargs="+", default=[0.5, -0.5, 1.0])
    args = parser.parse_args()

    s = Session(args, args.test)
    print(
        f"servo {args.id}: {s.state['input_volts']:.1f} V, housing {s.state['temp']} °C, "
        f"winding {s.state['winding_temp']} °C, position {s.state['position']:.3f} rad"
    )
    try:
        TESTS[args.test](s, args)
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        s.close()
        if s.entries:
            s.save({"note": args.note, "args": vars(args)})
            summarise(s.entries)


if __name__ == "__main__":
    main()
