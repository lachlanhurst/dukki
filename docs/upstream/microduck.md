# Upstream changes: microduck (firmware)

High level record of what changed in the `microduck` firmware repository between the commits we
have reviewed, and what it means for the Unitree J288 rework. One entry per review. The newest
entry is at the top. Sibling files in this folder will track `microduck_rl` and any other source
repositories once a baseline commit has been recorded for them.

Last reviewed commit: `dec725c67ffbbdde7f1de30f8c936b1bf9c1d70e` (07/10/2026). Our copy in `../microduck` tracks upstream on `main`; our work goes on the `dukki` branch (renamed from `unitree` 03/10/2026), which has this merged in (merge `3431b48`, 10/10/2026).

How to add an entry: note the previous "last reviewed" hash, run `git log --oneline OLD..NEW` in
`../microduck`, read the commit bodies for anything touching `duck-control`, `robotd`,
`robotd-params`, `kinematics`, `odometry`, `deploy/robotd.toml` and `duck-ipc-proto`, then write up
what matters here. Skip things that only affect the console, Hugging Face Spaces or CI unless they
change a robot-side interface.

---

## ac7531a (22/09/2026) to dec725c (07/10/2026)

Releases 0.15.0 (23/09/2026), 0.15.1 (01/10/2026), 0.15.2 (05/10/2026), 0.15.3 and 0.16.0
(06/10/2026) and 0.16.1 (07/10/2026). About 92 non-merge commits and 144 files changed. The first
31 commits (to `9136aa4`, release 0.15.1) were merged into `dukki` before this entry was written
and are covered here too. Most of the volume is support for Pollen's "beta" board, a declared
board type that many defaults now hang off, a posture-aware start, and new behaviour defaults
(pickup detection, limp fall, an idle head). `RobotIo`, `kinematics` and `odometry` are unchanged
apart from two additive `RobotIo` methods, so `BridgeIo` needed no changes in the merge.

### 1. Merge notes

Five files conflicted when merging into `dukki`:

- `Cargo.toml`: workspace members keep `duck-bridge-proto` and `express` and gain upstream's
  `nfc`.
- `mediad`: upstream moved per-sensor facts (mode, exposure units, field of view, family
  calibration) into profiles in `mediad/src/sensor.rs`. Our `--full-frame` IMX219 crop is now the
  profile `sensor::IMX219_FULL_FRAME`, the IMX219 with the 3280×1845 ISP crop as its mode.
  `pin_full_frame` takes the expected sensor and refuses anything but an IMX219. Not yet
  re-tested on the board.
- `robotd/src/main.rs`: upstream's `WaitingVoice` plays one-shot sounds while the bus is not up.
  Our bridge path returned before that point, so it now uses `WaitingVoice` around
  `open_bridge_waiting` too.

Tests after the merge (macOS, `cargo test --workspace --exclude nfc`): 1589 passed, 1 failed. The
failure is ours and predates the merge: `express/systemd/sysusers.d/expressd.conf` is not
packaged in `.github/workflows/dev.yml`, which an xtask test checks. Upstream's `nfcd` does not
build on macOS (`nfc::spi::Spidev` is Linux-only but imported unconditionally); it builds for the
board.

### 2. Board declaration

New `[board] version = "zero3" | "beta"` in `robotd.toml`, unset meaning `zero3`
(`robotd-params/src/board.rs`, commits `4ba09e0`, `2c5943e`). It sets the updater's hardware
revision (zero3 1, beta 2), the camera sensor mediad expects and its mount angle (zero3 IMX219 at
90°, beta GC2093 at 0°), the body IMU mount, which bus tofd uses for the ToF, and which daemon
reads the head IMU. xtask derives `min_hw_rev` from the board and the `--min-hw-rev` flag is gone.
`robotctl health` shows "(not declared)" and offers to write the board when the device tree names
one it knows (`radxa,zero3*`, `pollen,microduck*`, `seeed,microduck*`).

Implication: leave it unset (or `zero3`) on the Radxa CM4. `beta` would make mediad demand a
GC2093, move tofd to `/dev/spidev3.0` and hand the head IMU to robotd's LSM6DSV16X driver, so our
BMI088 would never be read. The CM4's device tree is not recognised, so robotctl will not offer to
rewrite it. tofd now reads `robotd.toml` at start but on zero3 still sweeps `/dev/i2c-pihat` and
`/dev/i2c-3`, so the BMI088 on I2C8 still needs `--bus` as before. `[head_imu] enabled` became
`Option<bool>`; unset on zero3 is off, unchanged. `install.sh` gained `DUCK_BOARD`, which rewrites
any `version = "..."` line in `robotd.toml`, so avoid another key called `version`.

### 3. New behaviour defaults

- Pickup detection, on by default (`3c88c47`, `9c8a03a`, v2 model `620affc`, state field
  `safety.picked_up` in `25298b1`). A small model over joint velocities and commanded targets
  pauses the policy and holds a fixed `PAUSE_POSE` while the robot is held. Trained on the stock
  robot with XL330 dynamics, and the pose is the stock stance.
- `safety.limp_fall`, on by default (`e8468c4`). Only the default changed. Thresholds were tuned
  on the XL330 robot; it drops to `gain_limp`, which `BridgeIo` scales through
  `bridge_kp_per_gain`, then poses back to `DEFAULT_POSITION`.
- Idle head (`robotd/src/idle_head.rs`, `57fa6c8`, `f166580`, `2fcd152`). After 2 s idle the head
  glances, drifts and dozes, up to ±0.6 rad yaw, 0.2 rad pitch and 0.3 rad roll, plus a 1 mm body
  height "breath" under a stand policy. No config switch.

Implication: set `[pickup] enabled = false` and `[safety] limp_fall = false` on Dukki until each is
tested or retrained on the J288 model in `microduck_rl`. Check the J288 head joints tolerate the
idle head commands, or patch it out on `dukki`.

### 4. Bring-up, posture and shutdown

- Shutdown eases into a recorded `REST_POSITION` (`duck-control/src/model.rs`) over 1 s, holds
  1 s, then cuts torque (`da4973d`). The pose was recorded on a stock robot sitting untorqued.
- Seated-boot detection was removed (`759feaa`). Instead a posture check runs on every enable
  (`robotd/src/posture.rs`, `eecefdd`, `8e9db5d`) using `kinematics::Model::alpha()` foot sites and
  the IMU attitude: seated starts with a sitstand rise, anything else goes to the gait. It never
  refuses.
- A robot seated at torque-on ramps to a fixed `SEAT_POSITION` rather than home, re-measured at
  every start, settles before rising, and seeds the policy's action history from the held pose
  (`8508326`, `8193f55`, `b0cf8e5`). One move at a time, none from the seat (`f93ecf7`).

Implication: `REST_POSITION`, `SEAT_POSITION` and the posture check's 50% and 80% height
thresholds all assume the stock leg geometry and the stock sitstand keyframe. They need Dukki
values once Dukki has a sitstand policy, and `REST_POSITION` should be re-recorded on Dukki
before relying on the shutdown ease.

### 5. State stream and the servo interface

- `robot.state` now carries joint velocities and load (present current) every tick (`b1c2475`,
  `dd327fd`, `a2d37e3`). `RobotIo` gained `measures_velocity` and `measures_load`, both default
  true, so a backend with nothing to report sends nothing rather than zeros. New
  `[control] publish_velocity_and_load`, default true.
- `BusHealth::missing` names silent servos (`76324a5`), `ImuHealth::missing` and a ping of IMU id
  200 (`04ae560`, `07ea87e`). `JOINT_IDS` moved into `duck-ipc-proto`.
- New `robot.rest` and `robot.flashlight` routes. API version 34 to 41 across the range.
- Adopted knees get their `homing_offset = -512` written (`4656690`), Dynamixel only.
- `[control]` velocity limits (max vx, vy, vtheta and friends) are editable in
  `robotctl configure` (`bbf5c13`).

Implication: `BridgeIo` already fills `velocities` from the bridge and `currents_ma` from J288
torque divided by `TORQUE_PER_AMP`, so the defaults are right for it. The missing-servo and IMU
health fields, the IMU mount and the knee offset live only on `DynamixelIo`, so bridge health
never sets them. Worth adding missing-servo reporting to `BridgeIo` later.

### 6. Beta board hardware

LSM6DSV16X head IMU read by robotd at 60 Hz (`a41a674`, `6306620`, `8527433`, `2f3c289`), a GC2093
camera (`102437a`, `3565b6b`, `d83c6eb`), VL53L8CX ToF over SPI (`8fde3c0`), ST25R100 NFC in a new
`nfcd` with pad pairing by tag (`0e78feb`, `fc204c4`, `1297f93`), and face LEDs driven through
sysfs by robotd, mediad and configd (`c2238ab`, `0ee179a`). The units gained
`ReadWritePaths=-/sys/devices/platform/face-leds`; the `-` makes that harmless on our board.

Implication: none of this applies to Dukki. NFC is left in the tree rather than removed, since it
is wired into `Cargo.toml`, `deploy/updater.toml`, both release workflows and `install.sh`, and
removing it would conflict on every merge. Do not install or enable `nfcd` on the board.

### 7. Gamepad (padd)

Full button remap (`fe2ed0d`, `a48dd1e`). Select held 2 s reboots every servo as it cuts torque
(`148651a`); released between 2 and 4 s rests the robot, held to 4 s powers it off (`06bbc68`). A
driving pad that drops out sits the robot down (`11b99db`). `padd` and `mediad` no longer pull
robotd back up after `systemctl stop robotd` (`e463144`).

Implication: on the bridge the reboot is `BridgeIo::reboot`, which clears J288 faults. Try the
Select hold on the bench before relying on it.

### 8. Updater, toolchain and release

- Releases must contain every packaged daemon binary (`f8dbb50`, `3ef856d`), now including
  `bin/nfcd` (`21f9ab9`). Per-component artefact budgets are enforced.
- Rust floor moved to 1.99 (`82bbad2`) and then back to 1.89 with a ceiling of 1.94.1 to stay
  under Yocto's toolchain (`9cd5b3e`).
- Policy-set floor moved from v5 to v7 (`a72beaf`); the post-install hook moves a board below v7
  to Pollen's v7 set.
- rustls 0.23.45 for GHSA-2mjx-qc3c-rqvc (`93f842c`).
- ALSA falls back to `default` when the configured card is absent (`db73d80`).

Implication: hand installs with `push-daemons.sh` are unaffected. A release packaged for updaterd
must build `nfcd`, and `expressd` is not in the required list, so updates will not carry it until
it is added. Keep J288 policies out of the default slots so the v7 floor cannot shadow them.

### 9. Actions for the Unitree rework arising from this range

1. Set `[pickup] enabled = false` and `[safety] limp_fall = false` in the board's `robotd.toml`.
2. Leave `[board] version` unset; never `beta`.
3. Decide on the idle head: test it on the J288 head or disable it on `dukki`.
4. Re-test `mediad --full-frame` on the board after the port to sensor profiles.
5. Record Dukki values for `REST_POSITION`, and later `SEAT_POSITION` and the posture thresholds.
6. Bench-test the padd Select hold (servo reboot, rest, power-off) through the bridge.
7. Package `expressd.conf` and `expressd` in the release workflows and updater required files,
   which also fixes the failing xtask test.
8. Optionally report missing servos from `BridgeIo` in `BusHealth::missing`.

---

## 67a1963 (14/09/2026) to ac7531a (22/09/2026)

Releases 0.13.0 (15/09/2026), 0.14.0 and 0.14.1 (17/09/2026), 0.14.2 and 0.14.3 (21/09/2026) and
0.14.4 (22/09/2026). About 157 commits and 116 files changed. Most of the volume is the phone app
path over BLE, a browser "policy playground" Space replacing the policy shop, and the simulated
duck growing into a full robot with remote access. Three changes touch the servo bus directly and
are listed first.

### 1. Servo bus

**Every sync read is now a fast sync read.** Files `duck-control/src/bus.rs` and
`robotd-params/src/lib.rs`. Dynamixel Protocol 2.0 instruction 0x8A: the sixteen devices append
their blocks to one status packet from the broadcast ID instead of each sending its own, so the
bus turns around once per tick rather than sixteen times. `rustypot` floor moved from 1.6.0 to
1.8.0 for it. `DynamixelIo::open` gained a `fast_sync_read: bool` argument, kept in the struct so
the motor-swap reopen at 57 600 baud does not lose it. New config key `[bus] fast_sync_read`,
default true, registered in `robotctl configure` beside the port. Requires XL330 firmware v46 or
newer and an `imu_to_dxl` board that implements the instruction. A device that does not implement
it does not answer at all, so every read times out and the release rolls itself back.

Implication: this is another Dynamixel-specific optimisation with no J288 equivalent. The
J288 protocol is one 20-byte command and 26-byte reply per servo, so the bridge MCU already owns
the "one transaction per tick" problem on its three bus segments. Our `BridgeIo` ignores the flag,
but `open_bus` now passes the whole `[bus]` section rather than a port string, so the signature we
implement against has changed. Keep the key in the file for compatibility and document it as
inert on the Unitree build.

**The shutdown mask's meaning was corrected, and it matters for our power design.** File
`duck-control/src/model.rs`. `shutdown = 52` is overload, electrical shock and overheating with
the input-voltage bit clear. The factory default is 53. The clear bit is what stops fifteen XL330s
latching torque off when a charged 2S pack sits above the servo's unwritten 7.0 V `Max Voltage
Limit`. The docs previously said the opposite. A test now pins the mask by named bits. The
comments on `BATTERY_FULL_V` and `BATTERY_EMPTY_V` also now say the span is a statement about
the servo rail being the pack, not a regulated supply.

Implication: the J288 fault table has its own over-voltage codes (transient 0x02, sustained
0x04) and no writable mask, so this register does not carry over. The battery span constants do,
and `docs/hardware.md` section 10 already plans 25.2 and 21.0 V for 6S. The upstream comment
now says explicitly that anything regulating the servo rail must move these constants, which
matches our plan.

**The safety layer now journals what it refuses or rewrites.** File `duck-control/src/safety.rs`,
about 300 lines added. Non-finite targets, out-of-range clamps and deadman trips each log the
first occurrence and then every fiftieth, with a per-limit run counter. The deadman only reports
once a fresh intent has ever arrived, so an idle bench robot does not fill the journal. The fall
verdict logs on every flip, unrated. `Safety::gate` now takes `&mut self`. Backend independent
and useful for bring-up on the J288 build: a policy or bridge fault that pushes a joint past
travel now leaves evidence.

### 2. robotd control loop

- **Mode switches wait for what they need.** A `robot.setMode` arriving during a policy load, a
  home ramp or a shutdown is queued or refused rather than acted on mid-motion, and a waiting
  switch is dropped if the robot goes limp before it fires. The limp path logs what it does
  instead of promising a ramp. Three PRs (#195, #228, #319) and one revert; `robotd/src/main.rs`
  grew by about 700 lines across the range.
- **New `soc.rs` module.** Reads thermal zones and the cpufreq ceiling from sysfs once a second.
  `robot.health` now carries `CpuThrottle` (level, max level, current and max kHz) and
  `robotctl health` prints "throttled to 408 of 1800 MHz". Relevant to us given the RK3576 in the
  head: the zone names and cpufreq policy paths will differ and should be checked on our board.
- **robotd publishes `homed` and `sitting`** on `robot.policies`, both `Option<bool>`, so a client
  can wait for the home pose and choose between `robot.init` and `robot.do sit_toggle`.

### 3. IPC protocol

`API_VERSION` moved from 28 to 34. All additive.

| Version | Addition |
| --- | --- |
| 29 | `simulated: bool` on `system.info` and in the rendezvous registration |
| 30 | `homed` on `robot.policies` |
| 31 | `sitting` on `robot.policies` |
| 32 | `degraded: bool` and `reason` on the updater status, so a bench board with servo power off is not reported as unhealthy |
| 33 | `cpu_throttle` on `robot.health` |
| 34 | `description` and `preview` on policy search results |

### 4. BLE and the phone app

The phone app now exists (design in the new `docs/design/mobile-app.md`) and drove most of the
btd changes.

- `robot.init`, `robot.enable` and `robot.rebootMotors` are now routed over BLE. `robot.relax`,
  `robot.move` and `robot.head` stay refused on the radio. The stated reasoning: standing up and
  rebooting a latched servo are controlled actions with a recovery path, relaxing is a fall.
- New `duck-ble` crate holds the GATT UUIDs, framing and advertisement decoding, extracted from
  btd so `duckctl` and the phone app share one wire contract without linking a daemon.
- btd stays discoverable while a central is connected, notifications may carry 512 bytes, and the
  chorale beacon is a broadcast rather than a peripheral.
- A known fault is documented in `docs/project/pad-minimal-pairing.md`: a bonded gamepad cannot
  reconnect while a phone is connected to btd. No fix; the page notes the chip is one they are
  considering replacing.
- `duckctl reboot-motors` wraps the new route.

### 5. Simulation

- A simulated duck now registers with the rendezvous service like a real one, flagged
  `simulated`. `configd --simulated <serial>` is the one place it is declared.
- The mediad pipeline builds off Linux behind a `gstreamer` feature, so a MuJoCo duck on a laptop
  has video and a console.
- The updater's policy library path is a config key (`policy_library`) so the twin can install
  community policies.
- `scripts/duck-sim` finds the RL repo beside this one and checks the streaming elements rather
  than the build ones. `spaces/vision-demo/fake_duck.py` was deleted since the twin is the robot's
  own code.

### 6. Policies

- The policy shop Space is replaced by `spaces/policy-playground`, a browser page that lists the
  tricks a duck knows, installs new ones and can stand the duck up.
- `robotctl policy search` shows each policy's one-line `description` from its manifest and links
  a preview clip found by path (`media/preview.mp4` first). Documented in
  `docs/policy-manifest.md`. No manifest schema change.
- No change to the observation or action contract, policy loading or the velstand defaults.

### 7. Updater

- Boot recovery polls robotd for its health timeout before reverting, rather than reverting a
  robot that is still starting.
- A `select` whose apply action fails after the symlink swap now rolls back and journals it.
- Rollback journal entries record where the board came from, not what failed.
- Status is three-way: healthy commits, degraded commits, anything else reverts.

### 8. Other

- New `uyvy` crate: UYVY to RGB sampling and the camera mount turn, moved out of `duck-detect` so
  `robotctl monitor` can draw a camera frame (key `c`) without linking a model runtime.
- mediad's auto-exposure writes digital gain back to 1x, and logs the ICE candidate pair used.
- tofd only subscribes to the stream that was asked for. No change to the head IMU reader.
- padd drops in-flight button holds when the pad disconnects but keeps a shutdown that was
  already sent.
- Workspace version 0.14.4. New workspace members `duck-ble` and `uyvy`.

### 9. Actions for the Unitree rework arising from this range

1. `open_bus` and `DynamixelIo::open` now take the `[bus]` section. Implement `BridgeIo::open`
   against that shape and treat `fast_sync_read` as inert.
2. Check `soc.rs` thermal zone and cpufreq paths on the RK3576 board. The module picks the
   hottest zone and worst cluster generically, but confirm the names it matches.
3. Nothing in this range changes the head IMU path, the RemoteIo sim protocol, kinematics or
   odometry. Items 1 to 9 from the previous entry still stand.

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
