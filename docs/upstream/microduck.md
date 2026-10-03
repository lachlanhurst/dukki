# Upstream changes: microduck (firmware)

High level record of what changed in the `microduck` firmware repository between the commits we
have reviewed, and what it means for the Unitree J288 rework. One entry per review. The newest
entry is at the top. Sibling files in this folder will track `microduck_rl` and any other source
repositories once a baseline commit has been recorded for them.

Last reviewed commit: `67a19637ee1018088fb6c72c48a9e287949ff7de` (14/09/2026).

How to add an entry: note the previous "last reviewed" hash, run `git log --oneline OLD..NEW` in
`../microduck`, read the commit bodies for anything touching `duck-control`, `robotd`,
`robotd-params`, `kinematics`, `odometry`, `deploy/robotd.toml` and `duck-ipc-proto`, then write up
what matters here. Skip things that only affect the console, Hugging Face Spaces or CI unless they
change a robot-side interface.

---

## bc41fb5 (03/09/2026) to 67a1963 (14/09/2026)

Releases 0.11.0 (07/09/2026) and 0.12.0 (10/09/2026) fall in this range. Roughly 230 commits and
176 files changed. Most of the volume is remote access over WebRTC, Hugging Face Spaces, and a
MuJoCo simulation harness. The parts that affect the servo rework are listed first.

### 1. Servo bus and the RobotIo seam

These are the changes to the code we will be replacing or reimplementing for the J288.

**The RobotIo trait gained a `reboot(id)` method.** File `duck-control/src/io.rs`. It sends the
Dynamixel Protocol 2 REBOOT instruction to clear a latched hardware error (overload, overheating,
electrical shock). `Safety::reboot_motors(ids)` wraps it and forgets the gain cache so gains are
rewritten on the next tick. Exposed as the `robot.rebootMotors` IPC intent, the
`robotctl robot reboot-motors [ID ...]` command, and D-pad right on the gamepad. `FakeIo` records
reboots in a `reboots` vector for tests.

Implication: a Unitree backend must implement `reboot`. The published J288 protocol has no reboot
frame. Options are to return an error, treat it as a no-op with a log line, or have the bridge MCU
power-cycle the servo rail. This needs a decision before the backend is written.

**Factory-fresh servo adoption.** File `duck-control/src/bus.rs`, plus `FACTORY_ID = 1` and
`FACTORY_BAUD_RATE = 57_600` in `model.rs`. On startup `open_bus` now pings all fifteen joint IDs.
If exactly one is silent it looks for ID 1 at 1 Mbps, then reopens the port at 57 600 baud, writes
the missing ID and the bus baud rate, reopens at 1 Mbps, runs the register check on the new servo
and reboots it. Two or more missing servos are left alone. `robotd init` takes the same path. New
constants `REBOOT_SETTLE` (500 ms) and `EEPROM_SETTLE` (20 ms). New public functions
`missing_servos`, `adopt_replacement`, `replacement_target`.

Implication: this is entirely Dynamixel specific and cannot be ported. The J288 protocol has no
ID-set command, so ID assignment stays a bench procedure with the Windows GUI as documented in
`docs/motor-setup.md`. Our `open_bus` equivalent should keep the ping census (useful for
diagnostics) and drop the adoption step. `check_registers` and `EXPECTED_REGISTERS` are also
Dynamixel EEPROM specific and have no J288 equivalent.

**Non-finite command targets are dropped.** File `robotd/src/main.rs`. A JSON value like 1e400
parses as infinity and used to poison the twist, head and body EMA filters permanently. Now
rejected before the filters. Backend independent, but worth keeping in mind for any new intent we
add.

**Bus read warnings are rate limited.** An isolated dropped `sync_read` now logs once a minute with
an `also=N` count, and the five-minute loop summary gains a `bus_drops` field. Continuous runs of
errors still log every tenth. Our backend's read errors will flow through the same path.

**Endpoint lock before the bus opens.** Both the daemon and standalone `robotd init` take an
advisory lock on `<socket>.lock` (normally `/run/robotd.sock.lock`) before publishing identity or
opening the serial port. A second claimant is refused. The lock file is never unlinked. Documented
in `docs/design/robotd-design.md` section 1.1. Any bench tool we write that opens the bus port on a
robot should either take the same lock or be run with the daemon stopped.

### 2. Simulation backend: RemoteIo over TCP

New file `duck-control/src/sim.rs`. A third `RobotIo` implementation that talks to a MuJoCo body
served by `duck-body` in the `microduck_rl` repository. Started with `robotd --sim`, `tofd --sim`
and `mediad --sim-camera`, all wrapped by `scripts/duck-sim`. Design in
`docs/design/simulation.md`, usage in `docs/robot/simulation.md`.

The wire protocol is newline-delimited JSON over TCP with `TCP_NODELAY`, protocol version 1,
200 ms request timeout, reconnect on the next tick after a broken link. Requests are tagged by
`op`: `hello`, `read`, `write`, `gain`, `torque`, `slow`. The simulator reports in the robot's own
units (radians, rad/s, mA, IMU already in the trunk frame). The daemon holds 50 Hz against it.

Why this matters for us:

- It is the cleanest example of writing a new `RobotIo` backend, and a template for the J288
  bridge backend.
- It lets the whole daemon stack run without servos, so control loop changes for the rework can
  be tested before hardware arrives.
- The `gain` op carries `kp: u16`, a Dynamixel register value. The J288 frame takes kp and kd as
  floats with different scaling. The gain path through `Safety` and `RobotIo::set_gain` is one
  place the trait's Dynamixel assumptions leak, and it will need attention in both the real
  backend and any sim mapping.
- `RobotIo::slow_read` returns supply volts and per-joint temperatures. The J288 reply frame
  carries temperature and fault codes per servo, so this maps reasonably well.

### 3. Policies and runtime defaults

**Recurrent LSTM ONNX policies are supported.** File `duck-control/src/policy.rs`, docs in
`docs/recurrent-policies.md`. Explicit-state LSTM exports from mjlab/rsl_rl with inputs
`obs [1,61]`, `h_in`, `c_in` and outputs `actions [1,14]`, `h_out`, `c_out`. Feed-forward
policies still work. Observation width (61) and action width (14) are unchanged. Recurrent
policies need `model_api: 2` in the manifest. Memory resets on activation, network switch, pause
or fall recovery. A new example `policy-rehearsal` measures inference latency on the board without
opening the bus.

**velstand is the default walk.** Walk-mode defaults are now `walk = velstand.onnx` and
`stand = none`. The official policy set pin moved from v1 to v5 and is now a minimum rather than
a floor: the post-install hook moves an older official set up to it. Consequences for the shipped
`deploy/robotd.toml` defaults:

| Key | Old default | New default |
| --- | --- | --- |
| `[policy] walk` | alpha_walking.onnx | velstand.onnx |
| `[policy] stand` | alpha_stand.onnx | none |
| `[policy] voltage_adapt` | false | true |
| `[safety] limp_fall` | true | false |

`voltage_adapt` scales actions by nominal over measured supply voltage because the XL330's
effective kP tracks its supply. The J288 runs from 25.2 V with its own regulation and this
assumption should be checked. It may need to default off, or the nominal voltage changed, in our
config.

### 4. Kinematics and odometry

**Whole-skeleton forward kinematics.** `kinematics::Model::body_poses` returns every body's pose
in the trunk frame, served per tick in `robot.state.skeleton` and statically in
`robot.model.skeleton`. Uses the MJCF body tree, so our reworked MJCF must keep a clean parent
ordering.

**A `head_imu` site.** `HeadFk::head_imu_in_trunk` reads an optional `head_imu` site from the
MJCF for the BMI088 on the head module. The sensor is mounted tilted, so the site frame is the
sensor's own axes. If our model omits the site the code returns `None` and the head IMU frame is
skipped.

**Contact odometry anchor sets are generated from the sole mesh.** New file
`odometry/src/anchors.rs` is generated by `microduck_rl/scripts/odom_anchor_points.py`, which
drops vertical rays onto the alpha sole collision mesh. Three sets: `V15` (the old hardcoded
corners), `ALPHA4` and `ALPHA16`. `ALPHA16` is now the default and measured 9.1 mm final XY error
against 17.3 mm for V15 over 3.2 m in the twin. If the rework changes the foot or sole geometry,
`anchors.rs` must be regenerated from the new mesh.

**Bug fix in the hand tracker.** `kinematics/src/hand.rs` no longer panics on an empty ToF band
when `min_zones = 0`.

### 5. Config file: renamed and new sections in robotd.toml

`robotctl configure` renames old sections on save, and old keys load through serde aliases, but a
hand-edited file or our own fork of `deploy/robotd.toml` should use the new names.

| Old | New | Notes |
| --- | --- | --- |
| `[detect]` | `[duck_detector]` | Read by mediad. Model now comes from the Hub, not the release |
| `[imu_head]` | `[pad_imu_head_control]` | Gamepad IMU steering the head. Read by padd |
| `[media] camera = true/false` | `[media] source = "camera"/"test"` | Old key is ignored with a warning, not aliased |
| (none) | `[head_imu] enabled = false` | BMI088 on the head module, read by tofd. Off by default, costs 3.5 to 4.5 percent of a core at 100 Hz |
| (none) | `[media.intrinsics]` | Per-robot camera calibration, served scaled in `media.video` |
| `[theremin] enabled` | unchanged | Default flipped to false |

`robotctl configure` now knows per key whether a change needs a restart, a `robotd` reload
(`[policy]` except `mode` and `enabled`) or nothing (`[pad]` and `[pad_imu_head_control]`, which
padd re-reads within a second). Adding config keys for the Unitree backend means registering them
in `robotd-params/src/registry.rs` with the right apply action, or a test in `robotctl` fails.

### 6. IPC protocol additions

All additive. `duck-ipc-proto/src/lib.rs` grew by about 550 lines.

- API v24: `t_ns` (CLOCK_MONOTONIC) on `robot.state` and `tof.frame`, trunk IMU gyro and quat in
  `robot.state.imu`, camera and ToF poses in `robot.state.frames`, static geometry in
  `robot.model`.
- API v25: skeleton poses as above.
- `robot.rebootMotors { ids }` intent.
- `head_imu.stream` from tofd (renamed from `imu.stream`). Frames carry gyro, accel, Madgwick
  quaternion and temperature.
- `media.frame`: one raw camera frame on a local socket, also served by `robotctl` and HTTP, with
  the mount angle so the PNG comes out upright.
- `detector.check` and `detector.install` on updaterd, mirroring the policy set commands.
- Camera intrinsics moved from `robot.model` to `media.video`. The embedded per-revision
  calibration asset under `robotd/assets/alpha` was removed.

### 7. Gamepad behaviour

Documented in `docs/robot/cheatsheet.md`. Relevant when testing on the reworked robot.

- Start on a fresh boot now runs `robot.init` (torque on, 2 s ramp to home, hold). The second
  Start enables the policy. After that it toggles.
- Select short press sends `robot.relax` (torque off) on release. Select held 2 s sits, cuts
  torque and powers off.
- D-pad right reboots every servo.
- Y with `[pad_imu_head_control]` enabled and an IMU-equipped pad (Pro Controller clones) steers
  the head from the pad's tilt. New `pad-imu` crate holds the attitude filter.
- With two pads connected only the driving pad's buttons act.
- Classic Bluetooth pads now go through the kernel's hidp rather than bluetoothd
  (`UserspaceHID=false`), saving about 16 percent of a core on the board.

### 8. New crates, tools and infrastructure

Lower relevance to the servo rework but useful to know exist.

- `duck-ether`: a simulated lossy radio between ducks in containers, used to reproduce the
  chorale bug.
- `pad-imu`: gamepad attitude filter shared by padd and `robotctl monitor`.
- `spaces/`: Hugging Face Spaces (`hello`, `vision-demo`, `policy-shop`) and a shared client. The
  robot can push H.264 to a Space over WebRTC.
- WebRTC is now the default remote transport. The console offers its own relay, and a TURN
  endpoint lives on a Space. CI checks the relay daily.
- `duckctl ssh`, `duckctl scp` and `duckctl logs` (a daemon's last words over BLE).
- The duck detector model is seeded from the Hub (`pollen-robotics/microduck-duck-detector`,
  pin `duck-v1`) instead of being vendored. 14 MB of weights left the repository.
- Hugging Face login moved to the `hf-robot-account` crate on crates.io.
- `bmi088` crate pinned to tag v0.1.2 for the head IMU.
- `scripts/setup-quiet-boot.sh` takes Armbian's apt update check off the boot path.
- Workspace version is 0.12.0. New workspace dependency `url = "2"`.

### 9. Actions for the Unitree rework arising from this range

1. Decide how `RobotIo::reboot` behaves on the J288 backend.
2. Drop the factory-adoption and EEPROM register check paths from our `open_bus`; keep the ping
   census.
3. Review the `set_gain(kp: u16)` contract against the J288 kp/kd float fields, including the
   RemoteIo `gain` op if we want the sim to match.
4. Check whether `voltage_adapt = true` is right for a 25.2 V bus, and set the default in our
   config accordingly.
5. Add a `head_imu` site to the reworked MJCF if the head module and BMI088 are retained.
6. Regenerate `odometry/src/anchors.rs` from the new sole mesh if foot geometry changes.
7. Register any new config keys in `robotd-params` with their apply action.
8. Record a baseline commit for `microduck_rl` so its changes (in particular `duck-body` and
   `odom_anchor_points.py`) can be tracked the same way.
9. Add configurable I2C addresses with chip-ID detection to `bmi088-rs`, and use it in tofd's
   `open_imu` (done locally 03/10/2026, `../bmi088-rs` branch `configurable-addresses`); offer
   both upstream. Our BMI088 module can only strap 0x18 with 0x68, not the HAT's 0x19 with 0x68.
   See `docs/hardware.md` section 6.2.
