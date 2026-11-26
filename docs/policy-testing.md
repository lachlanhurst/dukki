# Policy testing on the robot

The first runs of a trained policy on the assembled Dukki, 21 and 22/11/2026: what was tested, what was measured, and what each finding means for the robot's settings and for training. The walk policy at zero command is also what the robot stands with, so most of the tests are stands.

The robot ends this round on the walk policy gearplay 10000 (run `2026-11-22_*_dukki-walk-gearplay-kp1-kdr`, microduck_rl `dukki-j288`), trained with gear play, high joint friction and kd randomised (section 8), with robotd's leg filter at 0.7 and head filter at 0.5, the IMU median off and kd 0.07 (section 2). It stands still, walks forwards nearly straight and backwards without falling, turns in place at 1 rad/s and recovers from most hand pushes. It sits and stands on the sit-stand policy 9248, from the PS4 controller (section 9). Why the robot is more restless than the simulation, and why its falls lean backwards, are still open (section 13). Getting up after a fall works on the stand-up policy 9250 in the `stand` slot, cleanly on a hard floor; on carpet half its rises shake or fall backwards once (section 11, 23/11/2026).

## 1. Setup and tools

The robot runs robotd from microduck `dukki`; `scripts/policy/robotd.toml` is a copy of its settings as installed, which since the upstream 0.23.0 merge is robotd's user layer, `/var/lib/robot/robotd.toml`, over the template in `/etc/robot/robotd.toml` (`compute-setup.md` section 5.8). Before that it was `/etc/robot/robotd.toml`. Policies are exported to ONNX on the training host with microduck_rl's `scripts/export.py` and copied to `/opt/robot/policies/dukki/` on the robot.

Robot tests, `scripts/policy/`, copied to the robot's `/tmp` and run there over ssh:

- `stand_test.py N` enables the walk policy at zero velocity (after robotd's 2.5 s ramp to home), runs N seconds, then disables and relaxes, recording the whole state stream to `/tmp/stand_test.json`. A pushed stand is 60 s with hand pushes from front, back and sides for the first 40 to 45 s and hands off after; an unpushed stand is 60 s hands off.
- `walk_test.py [vx] [seconds] [vy] [vyaw]` sends `robot.move` at 50 Hz (robotd's deadman is 500 ms): 4 s standing, a 1 s ramp to the command, the command for the given time, a 1 s ramp down, 4 s standing. It reports the heading turned, from the gyro (robotd's odometry is wrong for Dukki). The standard test is 0.15 m/s for 6 s on the floor or a mat, about 0.7 m.
- `obs_replay.py` rebuilds the policy's 61 inputs from a recording and runs the ONNX on them. Its actions match the ones recovered from robotd's targets exactly (error 0.0000 rad) only when the inputs and the filter values are right, which makes it a check on both.
- `com_check.py` compares the torques the joints hold in still stances with what the model, posed the same way, needs: the centre of mass fore and aft against the model's (section 8). It runs from a microduck_rl checkout.
- `record.py [seconds]` records the state stream to `/tmp/pad_session.jsonl` while someone drives the robot from the pad; `sitstand_summary.py` (run on the Mac against the copy) lists each sit and rise with its tilt, peak body rate and any fall.
- `fall_cycles.py [N]` drops a standing robot limp (policy and torque off), waits 3 s, and turns the policy back on to get up, N times, recording to `/tmp/fall_cycles.json`. `flip_record.py [seconds]` records the trunk gyro and gravity while the robot is turned by hand and prints per-second peaks (the IMU range check, section 11.3).
- `stand_summary.py` prints, per 5 s, the action change per tick, the gyro and the worst tilt, the restless seconds (action change above 0.03 rad a tick) and robotd's loop timing. `timeline.py` is a per-second view; `obs_compare.py` compares the robot's inputs with the simulation's.

Simulation tests, `scripts/policy/sim/`, run from a microduck_rl checkout on the training host with microduck_rl's `scripts/dukki_eval.py` setup (training randomisation kept, pushes off):

- `walk_profile.py` runs the robot's walk test on 32 simulated robots and counts falls and which way they go, with switches for the task (the backlash model for gear play), a turn command, joint friction, kd, robotd's filters, an IMU pitch error and an ankle calibration error.
- `sitstand_sweep.sh` runs microduck_rl's `scripts/dukki_sitstand_eval.py` on a sit-stand checkpoint with and without gear play at set joint frictions (section 9).
- `stand_sweep.py` stands and walks 8 robots each with one change from training (delays, noise, friction, gains, calibration offsets, gear play) and reports how much the policy moves.
- `obs_dump.py`, `stand_specific.py` and `inair.py` were one-off comparisons (sections 5.1, 5.3 and 5.4).
- `standup_from_robot.py` places simulated robots in a pose recorded on the robot (`stuck_faceup_*.json`) and runs a stand-up checkpoint from it; `stuck-sweep.sh` runs it robot-like. `collision_check.py` lists what collides and what sinks into the floor in a pose, `lying_check.py` how the model lies face up and face down at home pose, and `expert-check.sh` runs the stand-up's behaviour-cloning teacher on its own (section 11). `lean_sim.py` is the ankle-lean test in desktop MuJoCo and compares `lean_test` recordings with the model (it needs no GPU and runs on the Mac); `lean_warp.py` runs it in training's simulator (section 12.1). `walk_profile.py` also takes a fixed IMU roll and hip yaw zero errors (section 12.2). `sweep_warp.py` runs microduck's `joint_sweep` in training's simulator with the trunk held in the air, each robot at a set multiple of the bench friction, and writes `joint_sweep`'s format for `sweep_summary.py` (section 12.2); it needs no GPU.

## 2. The robot's settings

| Setting | Value | Why |
|---|---|---|
| Walk policy | gearplay 10000 | Walks straight and backwards where 9250 circled and fell; pushes as 9250 (section 8). 9250 before it (section 4) |
| kp | 1.0 N·m/rad (`bus.bridge_kp_per_gain = 0.005` × gain 200) | What the Dukki policies train at; robotd's default gave 2.0 |
| kd | 0.07 N·m·s/rad (`bus.bridge_kd`) | Standing went from restless to still (section 5.7); inside the 0.05 to 0.08 gearplay trained with |
| IMU median | off (`bus.bridge_imu_median = false`) | It made the IMU about 20 ms later than training allows; section 6 |
| Leg filter | 0.7 (`policy.legs_lowpass`) | 0.85 let a self-sustaining jitter take hold; 0.5 and below lose walks to friction (section 5.6) |
| Head filter | 0.5 (`policy.head_lowpass`) | 0.6 made no difference |
| Action scale | 1.0; `voltage_adapt` off; `idle_head` off | As trained; voltage scaling cut the actions to 78% on 6S |
| Sit-stand policy | 9248, handing over to the walk policy 1.5 s into the rise (`policy.sitstand_rise_s`) | 7749 fell over backwards rising; section 9 |
| Other policy slots | `"none"` | The defaults are Pollen's alpha networks |
| Pad | `[pad_drive]`: forwards 0.3, backwards 0.2, sideways 0.2 m/s, turning 1.0 rad/s | Inside the walk policy's trained range; `compute-setup.md` section 5.8 |
| Limp-fall | on | Bench safety net |

## 3. Matching the robot to training, 21/11/2026

The first stands found several settings where robotd differed from the training environment: kp was 2.0 rather than 1.0, the action scale 0.9, battery-voltage scaling on (`7.4 / clamp(9.5)` scaled every action to 78% on the 6S pack), the idle look-around on (its head roll is past the trained range), and the battery constants were 2S. All were fixed (microduck `dukki` 27cdade and the robotd.toml above). Every policy input was then checked against the training layout by rebuilding it from the recording (`obs_replay.py`), and kp and kd against the servos' own torque reports: reported torque over position error gives 0.8 to 1.0 N·m/rad at a one-tick lag, and of the kd values tried, 0.05 explains the torque best. No unit errors.

Unfiltered, the walk policy stuck, jerked and ran away standing; the user caught it several times and once powered it off. robotd's target low-pass (legs 0.7, head 0.5), which Pollen uses at runtime on policies trained without it, steadied it. Pollen trained with the filter once (their microduck_rl 9cc8fc1a) and reverted it the next day; a Dukki walk policy trained with the leg filter (microduck_rl 03da4392) learned to stand still for every slow command, so the policies train unfiltered and the robot filters.

Not the cause of the runaway: servo gear play (0.2 to 0.8° per joint, measured with `scripts/joint_scope/play.py`, none outside the servos), loop timing (section 6), IMU mounting, or sole compliance (the gyro spikes were the user sliding the robot). Joint friction is: each joint broke free at 1.5 to 4.7 times the BAM bench fit's torque, where training randomised friction by only ±10%.

## 4. Choosing the walk policy

Every candidate was judged on the robot by a pushed 60 s stand. Simulation's metrics favoured the calmer policies each time, and each time the robot disagreed.

| Policy (microduck_rl run, checkpoint) | Trained with | On the robot |
|---|---|---|
| 8500 (`dukki-walk-headroll15`) | friction ±10% | Ran away unfiltered |
| 9250 (`dukki-walk-friction`, 43081b3a) | per-joint friction 1 to 3 times the bench fit | With the filters and the IMU median on: 0 falls in three pushed stands, worst tilt 15 to 16° |
| 9500 (`dukki-walk-robust`, 827a9112) | + encoder offsets ±2°, kp spread ±15% | Calmer standing, fell backwards under pushes |
| 10249 (`dukki-walk-lownoise`) | + action std 0.05, no entropy bonus | Fell backwards under pushes |
| 10000 (`dukki-walk-lownoise-nospread`, dbbbb194) | low noise, no offsets or kp spread | Calmest quiet stand, fell backwards under pushes (worst 93°) |
| 10750 (`dukki-walk-slowwalk2`, 371d60d9) | friction 1 to 5 times, fine speed reward, slow-walk commands, action-rate penalty halved | Three backward falls in a pushed stand, three times as twitchy (section 7) |
| 10000 (`dukki-walk-gearplay`) | gear play, friction 1 to 3 → 1 to 4 times, kd 0.05 to 0.08, fine speed and yaw rewards, slow-walk and turn-in-place commands | Walks straight and backwards, turns, pushes as 9250; runs on the robot (section 8) |

Calmer training cost push recovery on the robot every time. Push recovery decides; a calm quiet stand only breaks ties.

## 5. Why the robot is restless and the simulation is not

Standing on the desk, 9250 kept making corrections the simulation never does. From a recording on 22/11 against 8 simulated stands:

| | Robot | Simulation |
|---|---|---|
| Action change per tick (legs) | 0.085 to 0.09 | 0.007 (0.000 without sensor noise) |
| Joint angle spread | 0.03 to 0.15 rad | 0.003 rad |
| Gyro spread | 0.5 to 1.1 rad/s | 0.018 rad/s |

No input group drives it alone: holding any one of gyro, gravity, joint positions, joint velocities or the last action at its mean in a replay cuts the action change by only 10 to 30%. The whole loop of policy, servos and body keeps itself moving on the robot and settles in simulation.

### 5.1 Simulation sweep

9250 in simulation, one change from training at a time (`stand_sweep.py`):

| Change | Stand: action change, gyro spread | Walk 0.15 m/s |
|---|---|---|
| None | 0.007, 0.018 | normal |
| Servo command delay 15 to 30 ms (training: 5 to 10) | unchanged | slightly more sway |
| Every sensor one tick late | unchanged | normal |
| kp 0.6, kd 0, friction 3 to 5 times, foot friction 0.4, gear play, the servo's own speed reading | unchanged | kd 0: falls over and over; friction 3 to 5 and kp 0.6: more sway |
| Sensor noise ×4 | 0.030, 0.076 | normal |
| Encoder offsets ±5° / ±8° | 0.016, 0.084 / 0.044, 0.28 | normal |
| Command delay 40 to 50 ms | unchanged | 6 falls |
| Gyro and gravity 20 / 40 / 60 ms late | unchanged / unchanged / 1 fall | normal / more sway / 9 falls, action change 0.167 |

Only large encoder offsets make the simulated stand restless, and ±8° gives half the robot's motion. Nothing reproduces it.

### 5.2 Calibration

`scripts/joint_scope/flatfeet.py` checks the leg calibration with the robot limp and both soles pressed flat on a table: each sole's down, carried through its leg's joints with the MJCF's kinematics, must agree with the other's and with the IMU's. The soles are the bottom faces of the foot collision meshes, flat to 0.3° in the home pose; the MJCF's `*_foot` sites are rolled about 5° from them and must not be used. Over five readings in different crouches and leans:

| | Mean ± spread |
|---|---|
| Left against right, pitch | −0.3 ± 2.2° |
| Left against right, roll | +1.3 ± 0.9° |
| Both feet against the IMU, pitch | −3.7 ± 2.1° |
| Both feet against the IMU, roll | +3.8 ± 1.4° |

The legs agree with each other to within the readings' spread. The IMU and the legs disagree by about 5° in the same way for both feet: either the IMU mount (section 12 of `hardware.md`) or a common error in the jig's zeros. Set exactly in simulation (left hip roll ±3.4°, IMU pitch ±5°, combined), these errors do not disturb the stand or the walk, and training already randomises the IMU mount by up to 6°.

### 5.3 Rotor position against the output encoder

The policy and the servo's own position loop both use the rotor-derived multi-turn position; the output encoder is read only at power-up, to seed it, and by robotd to pick the joint's turn. Read limp, at zero current:

| | Encoder minus rotor |
|---|---|
| Straight after power-up | within ±0.75° on every joint |
| After 25 minutes of stands | up to 3.1° (left hip roll −3.1°, right knee +2.8°, neck pitch +2.4°) |
| After one walk | changed by up to ±2.5° either way (left hip roll −3.1° to −0.9°, right knee +2.8° to +0.3°) |

It wanders rather than growing, and does not track the restlessness. Which of the two is right after use is still open: pressing joints against their hard stops at power-up and after a session would tell. Unitree's own deployment example feeds the J288's policy the output encoder (section 10); `j288-testing.md` section 17 found the output encoder shifts by 0.6 to 1.4° with held torque, which the rotor does not.

### 5.4 In the air, and on other surfaces

Held by the trunk with the feet off the table (pick-up detection off for the test), the policy was dead still for 15 s: action change 0.001, no joint moved more than 0.3°, the targets 1 to 3.7° from the joints and the joints not breaking free. The restlessness needs the feet on the ground.

After a power cycle the robot stood dead still on the bare desk and on a grippy mat, braced as the simulation braces: both hip rolls pushing their feet towards the centreline 8 to 12° past where they sit (friction under the soles holds them), the knees 6 to 9° off target under the weight. Taps on the back of the head swayed it up to 10° and it was still again within 1 to 2 s on both surfaces. The surface made no difference.

### 5.5 Which way it falls

Every fall of every policy with the IMU median on was backwards: under pushes, in walks and in stands. The simulation's falls have no preferred direction. After the median came off (section 6), the first fall was forwards.

### 5.6 The leg filter decides the standing state

Unpushed 60 s stands, 9250, back to back in one session so the drift, temperatures and battery were the same:

| Legs / head | Restless seconds | Falls |
|---|---|---|
| 0.85 / 0.6 | about 48 of 60 | 0 |
| 0.85 / 0.5 | 56 of 59 | 1 |
| 0.7 / 0.5 | 3 of 60 (settling after the ramp) | 0 |

With the morning's three still stands at 0.7, the leg filter decides whether the robot stands still or keeps itself moving. 0.85 passes 10 to 20% more of the 5 to 11 Hz motion than 0.7, and that is enough: the robot's balance loop has little margin at those frequencies, and the simulation's has plenty (it stands still even unfiltered).

The filter costs walking under friction. 9250 on the walk test in simulation, head at 0.5:

| Leg filter | Falls of 32 at 1 to 3 times friction | at 3 to 5 times |
|---|---|---|
| 1.0 (none) | 0 | 7 |
| 0.85 | 0 | 4 |
| 0.7 | 0 | 8 |
| 0.5 | 0 | 23 |
| 0.35 | 1 | 28 |

### 5.7 kd

kd 0.07 in place of the trained 0.05, with the IMU median off and legs 0.7:

| | kd 0.05 | kd 0.07 |
|---|---|---|
| Unpushed stand | restless for the first 20 s, then still | still all 60 s (0 restless seconds) |
| Pushed stand | 1 fall (forwards, at the end, after hard pushes) | 2 falls (backwards, early), then a second run with 0 falls, worst 16.6° |
| Walk test | finished, worst tilt 14.9°, restless before and after | finished, worst 10.8°, still before and straight after |

In simulation more kd does nothing at normal friction and costs walks at high friction (falls of 32 at 3 to 5 times: kd 0.05 15, 0.07 19, 0.10 23, with robotd's filters), and the legs move more slowly (joint speed 1.95, 1.53 and 1.16 rad/s walking). On the robot 0.07 is the best setting found.

### 5.8 Body motion ahead of the joints

With both feet planted, the trunk's pitch rate is the sum of each leg's pitch joint rates. In the restless stands the gyro's pitch rate leads that sum, taken from the rotor positions, by about 34 ms (correlation 0.6 to 0.7, five recordings). A rigid robot cannot do that. Something lets the body move before the servos register it: gear play under load, the frame flexing, or the soles. The simulation's backlash variant did not reproduce the restlessness, so its play is not that something. Measuring where the give is (push the trunk with the servos holding home and compare gyro, rotor and output encoder) is the next step (section 13). Section 8 found that the backlash model, with more friction, does reproduce the backward falls, and that training with it fixed walking backwards on the robot.

## 6. Timing and the IMU median

robotd's tick, from its own timestamps over about 10,000 ticks on 22/11: 20.0 ms, 1% at or under 19.1 ms and 1% at or over 20.9 ms, the longest 21.3 ms, no missed ticks and no repeated frames.

Since microduck `dukki` 532a4e29 robotd publishes each tick's latencies in `robot.state`'s `loop.timing`. With the policy running:

| | Median | 95% under | Worst |
|---|---|---|---|
| Oldest servo reading when the bridge built the sample | 1.1 ms | 1.4 ms | 2.7 ms |
| IMU block's age then | 5.0 ms | 8.0 ms | 10.6 ms |
| Sample request to answer | 3.5 ms | 4.6 ms | 7.0 ms |
| robotd, sample in hand to command written (inference included) | 1.0 ms | 1.2 ms | 2.1 ms |

With the bridge's 2.8 ms from command to servo (`hardware.md` section 5.5) a servo acts on a reading about 6 to 8 ms old, inside the 5 to 10 ms training models. The joint velocities, the bridge's position difference over about 15 ms, lag the true rate by 5.6 ms; training's difference over one 20 ms tick lags by 10.

The IMU did not fit. robotd ran the gyro and gravity through a median of the last three ticks, Pollen's spike rejection, which holds a smoothly changing reading back by about one tick. With the block's age and the chip's own filtering the policy saw body motion about 25 to 30 ms late, past the 0 to 20 ms training randomises, and the policy is sensitive to IMU lag (section 5.1: 40 ms sways, 60 ms falls). Four recordings with the median off showed no wild blocks for it to reject: the bridge checksums every frame and robotd counts stale blocks. The jumps it would have clipped are real fast motion, during the jitter and at foot strikes. About 9% of the gyro's motion is above 15 Hz without the median (3% with it), vibration faster than the loop aliasing into it.

`bus.bridge_imu_median = false` (microduck 532a4e29) turns it off:

| Legs 0.85, unpushed | Median on | Median off |
|---|---|---|
| Restless seconds | 59 of 60 | 29 of 60, including 10 s after a tap |
| Falls | 1 | 0 |

The walk test with the median off, legs 0.7, kd 0.05, was the first to finish: 6 s at 0.15 m/s and the slow-down, worst tilt 14.9°, the trunk staying slightly forward throughout where every earlier walk drifted back and fell backwards. In a pushed stand it recovered from harder pushes than before; its one fall, at the end, was forwards.

With the median off the robot's IMU delay is about 5 to 15 ms, inside training's range, so this needs no retraining. Averaging the IMU's 120 Hz samples over each tick on the bridge would remove the aliasing for about 5 to 8 ms of lag, if it turns out to matter.

## 7. Walking and friction

Walk tests on the robot, 9250, 0.15 m/s:

| Settings | Result |
|---|---|
| Legs 0.7, median on, late in a long session (already restless standing) | Fell backwards about 2 s into the walk |
| Legs 0.7, median on, fresh after a power cycle | Walked the 6 s, fell backwards in the slow-down |
| Legs 0.85, median on | Fell backwards 3.5 s in |
| Legs 0.7, median off, kd 0.05 | Finished, worst tilt 14.9° |
| Legs 0.7, median off, kd 0.07 | Finished, worst tilt 10.8°, about 0.7 m in a straight line |

0.7 m over the 7 s of commanded walking is about 0.1 m/s for the 0.15 asked, as in simulation (0.103 m/s). robotd's odometry reported 0.87 m backwards for that walk and is wrong for Dukki.

In simulation 9250 falls in the walk test only when the joint friction is above what it trained with: none of 32 at 1 to 3 times the bench fit, 4 to 8 at 3 to 5 times, 23 at 4 to 6 times, mostly at the end of the walk and in the slow-down, as on the robot. An IMU pitch error of ±4° changes nothing.

Fine-tunes from 9250 to handle the higher friction (switches in microduck_rl `dukki-j288` 371d60d9, `tasks/dukki.py` item 8; runs on the training host):

| Run | Changes | Result in simulation |
|---|---|---|
| `dukki-walk-friction5` | friction 1 to 5 times from the start | Stopped walking at every command by 250 iterations |
| `dukki-walk-slowwalk` | friction 1 to 5 times (meant to ramp from 1 to 3 over 1000 iterations; see below), a fine speed reward (weight 1), 20% slow-walk commands | Stopped walking by 9750 |
| `dukki-walk-slowwalk2` | friction 1 to 5 times (the same), fine speed reward weight 2, 50% slow-walk commands at 0.1 to 0.3 m/s, action-rate penalty −0.5 in place of −1.0 | Walks slowly (0.13 to 0.16 m/s for 0.15) and backwards, no falls of 32 at 3 to 5 times friction, 1 at 4 to 6 times; stopped turning in place and veers 0.1 to 0.3 rad/s |

Why the first two stopped: the stock speed reward scores the instantaneous speed with a std of 0.32 m/s, and a stepping body's speed swings within each stride by about as much as a slow command, so at 0.15 m/s standing still keeps 80% of the reward and so does 9250's walk. Higher friction makes stepping dearer through the action-rate penalty, the largest cost of walking, and the policy stops. The fine reward scores the speed error averaged over 1 s with std 0.1, so a slow walk earns well above standing; Pollen fixed their turning dead zone the same way, with a fine yaw term added beside the stock one (their microduck_rl 4048a3ea). Even then it stopped until the action-rate penalty was halved.

The friction ramp in the slowwalk runs did not ramp. The env resets, and so runs the curriculum, before the runner loads the checkpoint and restores the step counter; the ramp took its start from the counter at 0, saw it jump to the checkpoint's step, and sat at its end range from the first iteration (the logs show `Curriculum/randomize_joint_friction_ramp` at 5.0 throughout). All three runs therefore trained at 1 to 5 times friction from the start. Fixed in microduck_rl: a jump in the counter between calls now restarts the ramp.

The original friction run was already drifting to standing at 1 to 3 times friction: 9250 walks 0.094 m/s at the 0.15 command, its 10000 0.046 and 10750 0.013. 9250 is a point on that slope.

slowwalk2's 10750 fell three times, backwards, in a pushed stand on the robot, with an action change of 0.2 to 0.56 a tick against 9250's 0.05 to 0.18. In simulation it is 30% twitchier than 9250 and spends 21% of its walking time with a joint above 18.6 rad/s, the fastest speed in the J288 bench identification, against 17%. Halving the action-rate penalty cost more on the robot than friction did. Keep it at −1.0.

What carried over from Pollen's microduck_rl (`develop` and branches to 10/11/2026): the fine yaw term and a turn-in-place command bucket for dead zones; their policies also walk slowly (0.2 m/s commanded, 0.09 achieved) and one would not start from standing at 0.2; their servo model's command delay (15 to 30 ms) is part of what the policy expects, and a test harness without it stood still; and they train on rough terrain with gear play, where these runs used flat ground without it.

## 8. Gear play, and the policy the robot runs now

The wrap-up tests of 9250 on the robot, with the current settings, 22/11/2026:

| Test | 9250 |
|---|---|
| Turn in place 0.5 rad/s | Did not move at all |
| Turn in place 1.0 rad/s | On the spot, 264° in 6 s (0.77 rad/s; simulation 0.96) |
| Forwards 0.3 m/s | Steady, worst tilt 12°, veered left about 0.28 rad/s |
| Sideways 0.2 m/s | Steady, turned left about 0.25 rad/s |
| Backwards 0.2 m/s | Fell backwards within 1 to 2 s, twice |
| Pushed stands at kd 0.07 | 2, 0 and 2 falls, all backwards |

On the polyurethane soles, fitted in place of the TPU ones the same evening, 9250 walked forwards at 0.15 m/s in a circle to the left, 177 to 265° in 6 s over three walks, its hip yaws lopsided by 6 to 8° (sum of left and right) where the TPU walks were balanced. The grippy soles hold each foot, so every twist at a hip turns the body; the TPU let the feet slip straight. A pushed stand on them fell once, backwards.

Two suspects for the backward falls did not hold up:

- The centre of mass. `scripts/policy/com_check.py` takes the torques the ankles hold in 27 still stances (kp times target minus position) and compares them with what the model, posed as the robot stood, needs. The real centre of mass is 5.1 mm (spread 2.7) behind the model's with the trunk's tilt from the IMU, or about 3 mm ahead with the tilt the legs give (section 5.2). Training randomises the trunk's centre of mass by ±15 mm, but that is the trunk body's own (310 g of 1,154 g), with ±10 mm on each head body: the whole robot's moves fore and aft by 2.7 mm typically (standard deviation) and 7.2 mm at most. 5.1 mm behind is further back than 98% of trained robots, so this suspect was dismissed too quickly (corrected 23/11/2026). The model's centre of mass at home is 10.6 mm ahead of the end of the soles' flat face and 30 mm behind the toe edge, so the backward margin is the small one. The ankle-lean test (section 12.1) then found the centre of mass where the model has it, to about 0.5 mm, and training's gear-play model leaning and tipping as the robot does.
- The ankles' zeros. A mirrored ±4° error on both ankles, about the IMU-against-legs disagreement, in simulation: no falls forwards or backwards.

Gear play and high friction together do reproduce it. 9250 in the walk test (`sim/walk_profile.py`) at the robot's settings (kd 0.07, robotd's filters), falls of 32:

| Model | Friction | Standing | Forwards 0.15 | Forwards 0.3 | Backwards 0.2 |
|---|---|---|---|---|---|
| No play | 1 to 3 times | 0 | 0 | 0 | 0 |
| No play | 3 to 5 times | 0 | 4 to 8 | | 4 |
| Backlash model (±1° hinge after each servo) | 1 to 3 times | | | | 0 |
| Backlash model | 3 to 5 times | 9 to 13 | 18 to 22 | 32 | 21 |

With both, most falls lean backwards and many come in the first second or two, as the robot settles from the start pose, like the robot after homing. Both are on the robot: friction 1.5 to 4.7 times the bench fit, and more give than the 0.2 to 0.8° inside each servo (section 5.8). The policies had trained without play and at 1 to 3 times friction. The backlash model's ±1° (2° in all) was set for the XL330 and is more than the J288's own play; it may stand in for the other give.

Two fine-tunes from 9250 on the backlash task, `Mjlab-Velocity-Flat-Backlash-Dukki`, both with friction ramping 1 to 3 → 1 to 4 times over 1000 iterations (the ramp fixed; section 7) and each joint's kd drawn in 0.05 to 0.08 per episode (`DUKKI_KD_RANGE`):

| Run | Also | Result |
|---|---|---|
| `dukki-walk-backlash` | nothing else | Falls in the gear-play test fell 60 to 80% (11249: 4, 6, 6, 4 of 32), but it stopped walking slowly by 9750 and stopped walking at all by 11249 |
| `dukki-walk-gearplay` | fine speed reward (weight 2, std 0.1 m/s), fine yaw-rate reward (`DUKKI_FINE_ANG_VEL`, weight 1.5, std 0.3 rad/s), 30% slow straight walking at 0.1 to 0.3 m/s, 20% turning in place from 0.2 rad/s (`DUKKI_TURN_IN_PLACE`) | Kept walking at every speed; 10000 walks 0.10 m/s at the 0.15 command, 0.29 at 0.3, 0.14 backwards at 0.2, and turns 1.00 rad/s at 1.0. Turning in place stopped from 10250 as friction neared 4 times, and slow walking faded by 11000 |

Gearplay 10000 in the gear-play test: 5, 10, 25 and 22 falls of 32 (standing, forwards 0.15, 0.3, backwards), against 9250's 9 to 12, 18 to 22, 32 and 21. On the robot it did better than that suggests:

| On the robot, polyurethane soles | 9250 | Gearplay 10000 |
|---|---|---|
| Forwards 0.15 m/s | Circles left, 177 to 265° in 6 s | Nearly straight, 26° right; hip yaw sum −1.7° |
| Backwards 0.2 m/s | Falls within 1 to 2 s | Walks, trunk level, no falls, still at once after |
| Turn in place 1.0 rad/s | 264° in 6 s (0.77 rad/s, TPU soles) | 387° (1.13 rad/s), worst tilt 6.3° |
| Forwards 0.3 m/s | Veered left 0.28 rad/s (TPU soles) | Steady, worst tilt 13.6°, 31° right over the walk |
| Pushed stand, kd 0.07 | 1 fall (2, 0, 2 on TPU) | 1 fall, backwards, early, then recovered every push |
| Action change walking | about 0.14 a tick | about 0.10 |

The robot runs gearplay 10000 since (`scripts/policy/robotd.toml`; the 9250 configuration is on the robot as `/etc/robot/robotd.toml.9250`). robotd clamps more of its targets at the stops than 9250's, mostly neck pitch (its upper stop, +34° real) and left hip roll; training clips its targets the same way.

The trade-off the two runs show: with sticky, gappy joints the policy can be steady by moving less, or keep moving and stay exposed. The anti-standing rewards held it in the middle until friction neared 4 times. Turning in place is the first thing it drops: at a 1.0 rad/s command a tight yaw reward pays standing still and turning badly about the same.

The walking wrap-up of gearplay 10000, polyurethane soles, the same evening:

| Command | On the robot | Simulation, 1 to 3 times friction |
|---|---|---|
| Turn in place 0.5 and 0.6 rad/s | No motion at all | 0.01 rad/s at 0.5 |
| Turn in place 0.7 rad/s | 0.60 rad/s | |
| Turn in place 0.75 rad/s | 0.78 rad/s | 0.22 rad/s |
| Turn in place 1.0 rad/s | 1.13 rad/s | 0.75 rad/s |
| Forwards 0.15 m/s turning 0.3 rad/s | 0.21 rad/s, 73° over the walk | |
| Forwards 0.15 m/s turning 0.6 rad/s | 0.72 rad/s, 248° | |
| Sideways 0.2 m/s | Steady, 29° left over the walk (9250: 58°) | |
| Pushed stand | 1 fall, backwards, at 12.5 s, then recovered every push | |

Turning in place has a dead zone below about 0.6 to 0.7 rad/s, the policy's own (the simulation shows it too, and a little wider); walking, it turns at any rate. 0.7 rad/s is slow enough for turning on the spot, so the dead zone stays, and `[pad_drive]` in `robotd.toml` maps the right stick to ±1.0 rad/s. The pushes the robot does not recover from are hard sideways ones: they spin it 30 to 90° and it then falls backwards. Left for later.

## 9. Sit-stand

Dukki's sit-stand policy runs in robotd's `sitstand` slot, which takes a posture flag: sit, or stand. Since 22/11/2026 it is driven from a PS4 controller through `padd` (Cross toggles sit and stand; `compute-setup.md` section 5.8). robotd runs the rise on the sit-stand network for `policy.sitstand_rise_s` and then hands over to the walk policy. The default 1.0 s is for Pollen's alpha network; Dukki's runs at 1.5 s (microduck `dukki` da3e241a made the time a setting). Recorded with `record.py` and summarised with `sitstand_summary.py`.

| Policy, robotd | Sits | Rises |
|---|---|---|
| 7749 (run `*_dukki-sitstand-stops-kp1-kd0p05`), legs 0.7 | 4 of 4, smooth, settled in about 2 s | 0 of 3: leant forward 10° and started up, then about 0.3 s in its targets swung to ±3 to 5 rad and it threw itself over backwards, 100 to 120° of tilt at about 8 rad/s |
| 7749, leg filter off | 1 of 1 | 1 of 1, but violently: about 10 rad/s, too fast for the trunk IMU's gravity estimate (it read the robot as upside down, and robotd called a fall that did not happen) |
| 9248 (below), legs 0.7 | 8 of 8, pitch within +2 to +18°, settled in 1 to 4 s | 8 of 8, no falls; pitch −19 to +9° on the first, +1 to +14° after; peak body rate 5 to 8 rad/s |

7749 was trained before the walk's friction and gear-play findings. In simulation, at the robot's kd 0.07, pushes off, its rise from seated stands or fails on joint friction, with or without gear play (`sim/sitstand_sweep.sh`; reached of about 700):

| Model, friction | 7749 sit from standing | 7749 stand from seated | 9248 sit from standing | 9248 stand from seated |
|---|---|---|---|---|
| No play, 1 times | 99.9% | 94% | 36% | 99% |
| No play, 3 to 5 times | 57% | 5.5% | 19% | 6.6% |
| Backlash model, 1 times | 99.5% | 98% | 78% | 100% |
| Backlash model, 2 times | 92% | 91% | 55% | 99.7% |
| Backlash model, 3 times | 61% | 12% | 25% | 88% |

The failures above 3 times end with the head on the ground, like the robot's. 9248 is 7749 fine-tuned the way the walk was: `Mjlab-SitStand-Flat-Backlash-Dukki` (microduck_rl 34b9e7b4), friction ramping 1 to 3 → 1 to 4 times over 1000 iterations, kd 0.05 to 0.08, action noise 0.05 and no entropy, 1500 iterations (run `2026-11-22_18-31-58_dukki-sitstand-gearplay-kp1-kdr`). It rises at friction where 7749 cannot, and on the robot it is the first sit-stand that works. Its sitting from standing scores worse in simulation (25 to 78% against 61 to 99.9%), but on the robot it sat every time.

The leg filter is part of it: unfiltered, 7749 rose instead of falling backwards, and the 0.7 filter's lag is something the fine-tune's extra friction may have stood in for. The robot keeps the filter at 0.7 and runs 9248 (`scripts/policy/robotd.toml`; 7749 is still installed).

9248 is a working fallback, not the finished article: a peak of 5 to 8 rad/s is a quick, jerky rise. The next step is a softer one.

## 10. Unitree's reinforcement-learning deployment example

Unitree published a deployment example for a small biped with 14 J288 or S288 servos and the LSM6DSV16X (support site, "Multi-Motor Reinforcement Learning Deployment Example", 16/10/2026). Its policy contract is the same 61 inputs as robotd's, and its code says it was ported from robotd (it carries "the original robotd control.rs target filter", off by default).

- For the J288 the policy's joint positions are the output encoder, the joint velocities the servo's own speed reading; the rotor position is a diagnostic option. Targets still go through the rotor position, aligned to the output encoder once, at the first command. For the S288 it is the reverse: rotor positions, and the output encoder for calibration, diagnostics and protection only.
- The S288 policies were trained with BAM's M1 model (gain profile `s288_m1_bam`, contract `rigid_rotor_pd_m1`). The J288 training code is not public.
- Calibration holds each joint by hand at the model's zero and reads the output encoder, with a direction test per servo. The IMU's gyro bias is averaged at startup; its mount is configured, not measured.
- Deployment limits target changes to 7 rad/s, ramps into the policy over 8 to 12 s, and offers a friction feed-forward (a torque in the direction of intended motion, a tanh step of up to 0.2 N·m, off until identified), which cancels friction on the robot rather than training around it.

## 11. Getting up after a fall, 23/11/2026

The stand-up policy (microduck_rl `Mjlab-StandUp-Flat-Backlash-Dukki-Distill`) runs in robotd's `stand` slot, which robotd also uses for standing still whenever the speed command is near zero, and which `limp_fall` hands a fallen robot to once it has landed and been posed. `standing_gain_ratio = 1.0` keeps it at the gain it trained at (`scripts/policy/robotd.toml`). Tests are `stand_test.py N keep` with the robot lying down (robotd ramps the joints to home where it lies, then the policy has it; `keep` leaves the policy running at the end, because disabling ramps to robotd's home pose, which does not hold Dukki up), and `fall_cycles.py`, which drops a standing robot limp and lets it get up again.

### 11.1 The stand-up with gear play, 7499

Stand-up stops 6000 fine-tuned with the walk's recipe (gear play, friction 1 to 3 times ramping to 1 to 4, kd 0.05 to 0.08, no calibration spread), run `2026-11-23_10-09-23_dukki-standup-gearplay-kp1-kdr`. In robot-like simulation 7499 got up 94.5, 79 and 95% of the time from face down, face up and sitting with each joint at 1 to 4 times friction (6000: 93, 68 and 84%). On the robot face down worked 2 of 2. Face up failed 2 of 2 the same way: one roll attempt, then dead still on its back, rolled about 33° onto its left side, with the left hip roll at its stop and the head turned 70°. The inputs were right: the recorded observations replayed through the ONNX matched robotd's actions to 0.0005 rad.

### 11.2 The thighs went through the floor

The stand-up and sit-stand models (the groundcontact MJCF and its backlash twin) collided only the parts Pollen's Onshape export gave collision geoms: the soles, `hip_l`, the lower legs, the head shells and jaw, and a box for the battery. The trunk shells, thighs, the feet above the soles, the hip and neck brackets and the J288 housings outside the head passed through the floor. In the robot's stuck pose the simulated left thigh sank 25 mm into the floor and the robot rolled on to 70° and got up; on the robot that thigh props it at 34°. microduck_rl now gives those parts collision twins of their meshes (`dukki_constants.add_body_contact`, `DUKKI_BODY_CONTACT=0` reverts), with three body pairs excluded where the hulls overlap in ordinary poses. With it the simulated robot holds the robot's stuck pose (gravity −0.83, +0.55, +0.03 against the robot's −0.83, +0.56, 0.00) and 7499 gets up from it 0 to 9% of the time, many frozen like the robot. From a plain face-up start 7499 fell from 79% to 16% (1 to 4 times friction). Face down at home pose matches the robot; face up the simulated robot lies flat where the real one reads about 13° more upright, not yet explained. Tools: `sim/standup_from_robot.py` (any recorded pose in simulation), `sim/collision_check.py`, `sim/lying_check.py`, `sim/stuck-sweep.sh`.

### 11.3 The expert was the obstacle

The stand-up learns partly by copying Pollen's `alpha_stand` on frames tilted more than 20°, added on 23/10/2026 to get past a head-propped trap. `alpha_stand` learnt its face-up roll on the same thigh-less model: on the corrected one it gets up 10.5% of the time from face up (gear play, 1 to 4 times; 60.5% on the old model), face down unchanged at 95% (`sim/expert-check.sh`). A fine-tune of 7499 on the corrected model with the copying still on reached only 27% face up (8500). With the copying off (`DUKKI_STAND_EXPERT_COEF=0`) and half the starts face up (`DUKKI_STANDUP_MIX=0.10,0.15,0.25,0.50`), 250 iterations later it got up from every start, run `2026-11-23_16-32-59_dukki-standup-nobc-kp1-kdr`:

| Checkpoint, robot-like, gear play | Face down, 1 to 4 times | Face up, 1 to 4 times | Sitting, 1 to 4 times | Stays up at 3 times |
|---|---|---|---|---|
| 8500 (expert on) | 95 / 93% | 27 / 26% | 91 / 87% | 57 to 74% |
| 9250 (expert off) | 100 / 98% | 100 / 99% | 100 / 99% | 89.5 to 90% |

Got up / stays up. 9250 was best of the run (9750 and 9999 no better) and is in the `stand` slot. Its face-up roll whips the head, spins the trunk onto its front and stands in about 0.5 s, peaking at 14 to 17 rad/s, past the trunk gyro's old ±500 dps range (about 10 rad/s), so the gyro went to ±2000 dps (`hardware.md` section 6.1, bridge protocol 3).

### 11.4 9250 on the robot

| Surface | Get-ups | Clean | Shook or fell backwards after rising |
|---|---|---|---|
| Hard floor | 3 (face up, side, face down) | 3 | 0 |
| Carpet | 8 | 4 | 4 |

Every rise worked: face down in 0.28 to 0.36 s, face up in 0.56 to 0.68 s, from its side in 0.58 s, peaking at 7 to 15 rad/s. The gravity estimate followed the gyro through the rolls (frame-to-frame error 0.01 to 0.09 at 10 to 14 rad/s) and robotd called no false falls. A clean rise is dead still within a second (leg targets change 0.0004 rad a tick, under a third of what 7499 drew in the hip rolls). On carpet half the rises were followed by a shake, the trunk twisting at 3 to 9 rad/s each way and the leg targets changing 0.2 to 0.3 rad a tick; two of those fell backwards about half a second after rising, and `limp_fall` and a second rise recovered them. The carpet's pile lets the feet sink and twist, which training's hard floor never does. Twice `limp_fall` cut a recovery short: once at 26° of a slow backward lean the policy might have caught (training keeps control to 70°), and once in the middle of a roll, when the fall verdict cleared for a moment. Stuck on its side once was the robot against a wall.

## 12. Where the weight sits, and left against right, 23 to 24/11/2026

### 12.1 The ankle-lean test

No policy: the servos hold home at kp 4 N·m/rad and both ankles turn at 1°/s until the robot tips over its heels, then over its toes (microduck `duck-control/examples/lean_test.rs`, robotd stopped; `sim/lean_sim.py` and `sim/lean_warp.py` run it in desktop MuJoCo and in training's MuJoCo Warp; recordings in `scratch/lean/`). At the policy's 1 to 2 N·m/rad the legs give and the model buckles before the soles' edge, so the test is stiffer on purpose. Two runs each way, the robot turned round between them. Ankle readings, deg:

| | Settled lean (IMU) | Tips backwards at | Tips forwards at |
|---|---|---|---|
| Robot | −1.2 to +0.3 | −0.5 to −0.9 | about +9, after a 4° roll at +3.5 |
| Training's simulator, `Velocity-Flat-Dukki` | 0.0 | −2.1 | +12.2 |
| Training's simulator, `Velocity-Flat-Backlash-Dukki` | −2.4 | −1.1 | +8.4 (most +9 to +9.7), after the same roll |
| Desktop MuJoCo, flat boxes under the soles | +0.3 | −4.9 | +13.2 |

The forward roll comes as the ankle torque passes through zero: the gear play changing sides (no loose screws were found, and the soles are flat). Training's gear-play model matches the robot, with the centre of mass where the model has it (the forward fall puts it within 0.5 mm, give or take 2.3 mm for each degree of IMU error) and no IMU correction: the 1° the IMU reads behind the legs, standing, is the play held to one side. The sole meshes round off at the heel (0.15 mm up 5 mm in from the face's end), and training's simulator stands on them as the robot does, as if the heel were about 5 mm shorter than a flat face; desktop MuJoCo instead rocks on the meshes' deepest vertices, which is why `lean_sim.py` uses flat boxes, and why its figures are not the robot's. So the static model is right, and the policies the robot runs (walk gearplay 10000, sit-stand 9248, stand-up 9250) all trained on the gear-play model: the backward margin is that small on the robot and in their training alike, and the backward falls and the carpet need another explanation.

### 12.2 Turning on the spot, left against right

With the pad's daemon stopped, `walk_test.py` sent exactly mirrored commands (gearplay 10000, robot settings of section 2, hard floor). Turn rates, rad/s:

| | Left | Right |
|---|---|---|
| On the spot, ±1.0 (two runs each, the order swapped) | +1.14, +1.16 | −0.57, −0.58 |
| Walking 0.15 m/s, ±0.5 | +0.41 | −0.39 |
| Simulation, on the spot, ±1.0 (32 robots) | +0.71 | −0.68 |
| Simulation, walking 0.15 m/s, ±0.5 | +0.41 | −0.22 |
| On the spot after the left leg was rebuilt (24/11/2026) | +1.13 | −0.99 |

0.10 m/s with 0.5 rad/s is inside the policy's dead zone: robot and simulation both stand still. In simulation (`sim/walk_profile.py`, new switches `IMU_ROLL_DEG` and `HIP_YAW_BIAS_DEG`) neither a hip yaw zero error of 1.5 to 3° nor the 2.3° of roll the IMU reads standing reproduced the lopsided turn: the zeros moved it by up to 0.15 rad/s the wrong way, the roll by about a fifth of the gap the right way.

`joint_sweep` (microduck `duck-control/examples/joint_sweep.rs`, robotd stopped, robot held in the air, robotd's gains) swings a joint and then its mirror twin ±15° at 0.5 and 2 Hz; `scripts/policy/sweep_summary.py` compares the sides. The left hip yaw dragged when its toe turned outwards (negative): 0.05 to 0.074 N·m against about 0.015 the other way, rising past −5°, and it reached only −9 to −10° of a −15° target; the right was even (about 0.02 to 0.04 N·m). With the left leg taken off the hip yaw, and again with only its mount and bearing refitted, the left was even (0.019 / 0.016 N·m) and reached −13.8°: the servo was clean and the drag was in the leg assembly above the bearing. Rebuilt, the left matched the right (toe outwards 0.035 against 0.043 N·m, reach 12.1 against 11.4°) and right turns on the spot came up from −0.57 to −0.99 rad/s. At kp 1 a servo has no integral action, so a steady drag of 0.05 N·m costs about 3° of travel, and turning on the spot sits at the edge of the policy's dead zone, where that shortfall halved the right turns.

The ankles, knees and hip pitches swept even left to right (friction 0.015 to 0.025 N·m each; the knees and hip pitches resist lifting the hanging leg more than lowering it, the same on both sides). The hip rolls are not yet swept.

How the sweeps compare with training's friction (25/11/2026): `sweep_summary.py` also gives a fit-free friction loss, the motor's mean power over the cycles divided by the mean speed, and `sim/sweep_warp.py` runs the same sweep in training's simulator at known multiples of the bench fit. The robot's joints lose 0.016 to 0.029 N·m in the air (hip yaw, ankle, knee and hip pitch, both sides); the simulation loses 0.013 to 0.021 at 0.5 times the bench fit, 0.023 to 0.031 at 1 times and 0.034 to 0.043 at 1.5 times. The assembled joints, unloaded, are about 0.8 to 1.1 times the bench fit, at the bottom of training's 1 to 4 times range; friction under load is not measured. The robot's joints also lag the target by one more 20 ms tick than the simulation's, the order of the recording rather than the servos. Before the rebuild, the left hip yaw's mean loss was the same as the right's (0.023 N·m): its drag was a tight spot over part of the swing, not more friction overall.

## 13. Open items

- Find the give that lets the trunk move ahead of the joints (section 5.8): with the servos holding home, push the trunk gently and log the gyro, rotor positions and output encoders together. Then set the backlash model's play from it.
- Turning in place that survives high friction: stop the ramp near 3 times, a yaw reward that still pays at large errors, or turn-in-place practice of its own. Gearplay 10000 is the last checkpoint of its run that turns.
- Grip and foot twist in training: the polyurethane soles turn hip twists into body rotation. A higher foot friction range and torsional friction at the soles (the feet use condim 3).
- More gearplay 10000 on the robot: repeated pushed stands, sideways, faster walks, slow turns (0.5 rad/s).
- Which of rotor position and output encoder is right after use (section 5.3): hard stops at power-up and after a session.
- The IMU against the legs, about 5° (section 5.2): the mount or the jig.
- robotd's odometry for Dukki's legs, and its pick-up detection model, missing from the install (`pickup_detector.onnx`).
- A softer rise for sit-stand (section 9): 9248 rises reliably but at 5 to 8 rad/s.
- Stand-up on carpet (section 11.4): training on softer ground with torsional friction at the soles, then a soft-ground row in the robot-like evaluation.
- `limp_fall` during a get-up (section 11.4): a short hold-off after the stand policy starts from lying down, and whether its 26° trigger cuts recoverable leans short (a run or two with it off, hands ready).
- Face up at home pose reads about 13° more upright on the robot than in simulation (section 11.2): something on its back or head.
- Sit-stand trained on the thigh-less model too: retrain 9248 on whole-body contact, probably without the expert's pull either.
- Combining the stand-up with walking: robotd's `stand` slot already hands over to the walk policy above 0.05 of command; its standing under pushes and the hand-overs need testing.
- Hard sideways pushes spin the robot and it then falls backwards (section 8).
- Sweep the hip rolls (section 12.2), and randomise friction by joint and direction in training, sized from the sweeps (extra drag up to about 0.05 N·m on a random side), so the next sticky joint costs less; friction fine-tunes have cost turning on the spot before (section 8), so judge it on the on-the-spot turns.
