# j288

Bench tools for a single Unitree J288 servo on a USB to single-bus adapter:

- `explore.py`: the exploratory bench tests used to characterise the servo before the BAM identification rig. Every test logs each command and reply pair to JSON.
- `to_bam.py`: converts explore.py logs to BAM's log format (below).

The serial driver is `bam/unitree/digital_servo.py` in the bam fork (`../bam`, branch `unitree-j288`), shared with the J288 recorder there. It builds the 20-byte command frame, parses the 26-byte reply, converts between the protocol's rotor-side units and output-side SI units (gear ratio 70070/243 = 288.354), and clears faults and latched timeouts. The same protocol serves the S288.

Results from the tests run so far are in `docs/j288-testing.md`.

These are exploration tools. Identification recordings for the BAM model are made with the recorder in the bam fork, which calibrates the pendulum's hanging angle and writes BAM-format logs directly: `bam.unitree.calibrate`, `bam.unitree.record` (one recording) and `bam.unitree.all_record` (a session for one setup). Their module docstrings give the command lines.

## Requirements

- [uv](https://docs.astral.sh/uv/). The commands below pull `pyserial` on the fly and install the bam fork from `../bam` into a throwaway environment, so there is nothing to install. The fork must be checked out next to this repository.
- A USB to single-bus adapter that runs 6 Mbps. Unitree's module enumerates on macOS as two CDC ports, `/dev/cu.usbmodem<serial>1` and `...3`. Use the first; the second does not answer.
- The servo powered through the adapter's XT30 input, with its ID known (factory default 0, see `docs/motor-setup.md`).

Find the port:

```
ls /dev/cu.usbmodem*
```

Run everything from the repository root, so logs land in `scratch/j288_explore/` (ignored by git).

## Probe (no motion)

Sends stop frames and reports how many valid replies came back, the round-trip time and the servo's state.

```
uv run --with pyserial --with-editable ../bam python -m bam.unitree.digital_servo --port /dev/cu.usbmodem3744CBC819741 --id 0
```

Expect all replies valid and a round trip around 0.2 ms. The 3 mrad or so between `position` and `output_encoder` at rest is normal.

## Bench tests

Common options: `--port` (required), `--id` (default 0), `--rate` (loop rate, default 1000 Hz), `--note` (free text saved in the log; record the arm, load and supply here), `--logdir`.

Before any test that moves the output, clamp the servo down and keep the arm's sweep clear.

### hand: what the modes feel like

Interactive. Three 10 s phases, each after a 3 s countdown: mode 0 (stop), mode 1 with zero torque, and mode 1 damping. Turn the arm by hand both ways during each phase.

```
uv run --with pyserial --with-editable ../bam scripts/j288/explore.py hand --port /dev/cu.usbmodem3744CBC819741 --note "100 mm arm, 4S"
```

`--duration` sets the phase length and `--kd` the damping gain (default 0.05 N·m·s/rad).

### ramp: breakaway torque

Ramps torque from zero at 0.01 N·m/s in each direction until the arm moves, then brakes.

```
uv run --with pyserial --with-editable ../bam scripts/j288/explore.py ramp --port /dev/cu.usbmodem3744CBC819741 --note "100 mm arm, 4S"
```

`--ramp-rate`, `--max-torque` (give up above this), `--move-speed` (breakaway threshold).

### steps: acceleration and armature

Torque steps of 0.1, 0.2 and 0.3 N·m for up to 1 s each way, braked when the speed passes `--speed-limit` (default 10 rad/s). Unloaded, every step reaches the limit and the arm does full turns.

```
uv run --with pyserial --with-editable ../bam scripts/j288/explore.py steps --port /dev/cu.usbmodem3744CBC819741 --note "100 mm arm, 4S"
uv run --with pyserial --with-editable ../bam scripts/j288/explore.py steps --port /dev/cu.usbmodem3744CBC819741 --torques 0.05 0.1 --step-time 0.5
```

### position: position loop

Holds the current position, then steps by `--deltas` (default +0.5, -0.5, +1.0 rad) and back at `--kp` (default 0.5 N·m/rad) and `--kd`.

```
uv run --with pyserial --with-editable ../bam scripts/j288/explore.py position --port /dev/cu.usbmodem3744CBC819741 --kp 0.5 --kd 0.05
```

### stall: output torque against a scale

Interactive. The arm presses on a kitchen scale; the script holds a series of torques and you type in the scale reading at each.

1. Put the scale under the arm and measure from the output axis to the contact point. That is `--lever`, in metres.
2. Work out which torque sign drives the arm into the scale and pass it as `--sign`. On the bench servo, positive was into the scale.
3. Run the test. It first drives the arm gently into the scale at 0.06 N·m until it stops, then asks you to tare.
4. At each level press Enter. The torque ramps up over 0.3 s and holds until you type the steady scale reading and press Enter. Read it while the torque is on: the reading after release is only the friction holding the arm down.

```
uv run --with pyserial --with-editable ../bam scripts/j288/explore.py stall --port /dev/cu.usbmodem3744CBC819741 --sign +1 --lever 0.100 --note "100 mm arm on kitchen scale, 4S"
uv run --with pyserial --with-editable ../bam scripts/j288/explore.py stall --port /dev/cu.usbmodem3744CBC819741 --sign +1 --lever 0.100 --levels 0.2 0.5 0.8
uv run --with pyserial --with-editable ../bam scripts/j288/explore.py stall --port /dev/cu.usbmodem3744CBC819741 --sign +1 --lever 0.100 --levels 1.1 --note "clamp point from cold"
```

A level ends early if the hold passes `--hold` (default 10 s; the test moves on), the winding passes `--max-winding` (default 70 °C; the test stops) or the arm moves more than `--max-travel` (default 0.15 rad; the test stops). Type `q` at a prompt to finish. Readings are saved as they are entered, so Ctrl-C loses nothing.

The windings heat at about 1 °C/s at 0.5 N·m and 4.5 °C/s at the 1.0 N·m clamp. For a reading at the clamp, let the servo cool to about 40 °C and run that level alone, as in the third example.

### backlash: gear play with the output clamped

Clamp the output so it cannot turn either way, as close to the horn as possible; anything that flexes or slips between the horn and the clamp is measured as play. The servo can sit at any angle.

The test runs in torque mode (kp = kd = 0). Each cycle ramps the torque to ± `--amplitude` (default 0.15 N·m) over a quarter `--period` (default 10 s), holds it for `--hold-end` (0.5 s), switches to mode 0 for `--rest` (0.3 s), then does the same on the negative side. It runs `--cycles` (3) cycles, about 22 s. It brakes and stops if the rotor moves more than `--max-travel` from where it started (pass 0.1 rad: the clamp slipped or only stops one way) or the winding passes `--max-winding`.

```
uv run --with pyserial --with matplotlib --with-editable ../bam scripts/j288/explore.py backlash --port /dev/cu.usbmodem3744CBC819741 --max-travel 0.1 --note "output clamped near horn, 6S"
```

It prints the rotor travel between the two torque plateaus (everything from the motor to the clamp) and, from the zero-current rests, the split between play inside the servo (rotor-derived position minus output encoder) and motion of the output shaft (horn, spline or clamp). The output encoder is read only in the rests: under current it drifts with torque while the shaft is clamped (`j288-testing.md` section 17). A plot of the loop is saved next to the log.

## Converting logs for BAM

`to_bam.py` turns explore.py logs into BAM's log format for the J288 actuator in the bam fork (`J288Actuator` in `bam/unitree/actuator.py`). Each entry gets the rotor-derived `position`, the true `speed` (differentiated from position), the servo's `reported_speed` and `torque`, and the command it was sent (`goal_position`, `goal_speed`, `goal_torque`, `kp`, `kd`, `torque_enable`).

```
uv run --with numpy scripts/j288/to_bam.py scratch/j288_explore/steps_*.json --out scratch/j288_bam/raw --mass-inertia 2.0e-5
```

The bench arm was never hung straight down, so its angle has no known zero; by default its inertia goes in as `--mass-inertia` and gravity is left out (the 6 g arm's gravity torque is under 0.003 N·m). For a pendulum with a known zero, pass `--mass`, `--arm-mass` and `--length` instead. `--phases` keeps only some phases of a log.

Then, from the bam fork, resample and fit or evaluate (see `bam/unitree/fit_control.py`):

```
python -m bam.process --raw ../dukki/scratch/j288_bam/raw --logdir ../dukki/scratch/j288_bam/proc --dt 0.001
python -m bam.unitree.fit_control fit --logdir ../dukki/scratch/j288_bam/proc --output j288_control.json
python -m bam.unitree.fit_control eval --params j288_control.json --logdir ../dukki/scratch/j288_bam/proc
```

Only logs where nothing but the servo and gravity act on the arm (steps, position, ramp) can be simulated. Logs with the arm held or pushed (hand, stall) still work for the control-law fit, which uses only the servo's own readings.

## Safety behaviour

- Every frame is sent with the servo's timeout protection on, so the servo stops by itself about 1 s after the script dies. The script sends stop frames on exit and on Ctrl-C.
- A servo left without frames for about 1 s latches a timeout and ignores enable commands until it receives a frame with the timeout bit clear. Each test start and each stall level clears it first, which can take up to ten frames.
- The torque tests brake (damping, then stop) when the speed passes `--speed-limit`. The check uses a difference of position, not the servo's reported speed, which lags the motion by about 15 ms.
- Any servo fault flag stops the test.

## Log format

`scratch/j288_explore/<test>_<YYYY-MM-DD_HHhMMmSS>.json`:

- Top level: `test`, `rate`, `note`, `args` (every command-line option), and for stall tests `stall` with `lever`, `sign` and a `results` list (`cmd`, `scale_g`, `measured`, `reported`, `winding`, `volts`, `travel`, `stop_reason`).
- `entries`: one per reply, with `timestamp` (s, midpoint of send and receive), `phase`, the command (`cmd_mode`, `cmd_torque`, `cmd_speed`, `cmd_position`, `kp`, `kd`) and the parsed reply in output-side units: `position` (rotor-derived, multi-turn, rad), `output_encoder` (0 to 2π rad), `speed` (rad/s), `torque` (N·m), `temp` (housing °C), `winding_temp` (°C), `input_volts` (0.5 V steps), `error`, `warning`, `mode`, `timeout`.

Things to know when reading the logs:

- Each reply reports the state from before its own command was applied, a one-frame lag.
- `speed` is filtered inside the servo (first order, about 15 ms). Differentiate `position` for the true speed.
- `torque` is the motor's current-based estimate, before gearbox losses. About half of it reaches the output.
- `output_encoder` is trustworthy at rest but jumps by tens of mrad while the output moves.
