# Policy testing on the robot

The first runs of a trained policy on the assembled Dukki, 21 and 22/11/2026: what was tested, what was measured, and what each finding means for the robot's settings and for training. The walk policy at zero command is also what the robot stands with, so most of the tests are stands.

The robot ends this round on the walk policy 9250 (run `2026-11-21_20-12-24_dukki-walk-friction-kp1-kd0p05`, microduck_rl `dukki-j288`), with robotd's leg filter at 0.7 and head filter at 0.5, the IMU median off and kd 0.07 (section 2). It stands still, walks 0.15 m/s in a straight line and recovers from most hand pushes. Why it stays restless where the simulation does not is still open (section 9).

## 1. Setup and tools

The robot runs robotd from microduck `dukki` with `/etc/robot/robotd.toml`; `scripts/policy/robotd.toml` is a copy of the one installed. Policies are exported to ONNX on the training host with microduck_rl's `scripts/export.py` and copied to `/opt/robot/policies/dukki/` on the robot.

Robot tests, `scripts/policy/`, copied to the robot's `/tmp` and run there over ssh:

- `stand_test.py N` enables the walk policy at zero velocity (after robotd's 2.5 s ramp to home), runs N seconds, then disables and relaxes, recording the whole state stream to `/tmp/stand_test.json`. A pushed stand is 60 s with hand pushes from front, back and sides for the first 40 to 45 s and hands off after; an unpushed stand is 60 s hands off.
- `walk_test.py [vx] [seconds]` sends `robot.move` at 50 Hz (robotd's deadman is 500 ms): 4 s standing, a 1 s ramp to vx, vx for the given time, a 1 s ramp down, 4 s standing. The standard test is 0.15 m/s for 6 s on the floor or a mat, about 0.7 m.
- `obs_replay.py` rebuilds the policy's 61 inputs from a recording and runs the ONNX on them. Its actions match the ones recovered from robotd's targets exactly (error 0.0000 rad) only when the inputs and the filter values are right, which makes it a check on both.
- `stand_summary.py` prints, per 5 s, the action change per tick, the gyro and the worst tilt, the restless seconds (action change above 0.03 rad a tick) and robotd's loop timing. `timeline.py` is a per-second view; `obs_compare.py` compares the robot's inputs with the simulation's.

Simulation tests, `scripts/policy/sim/`, run from a microduck_rl checkout on the training host with microduck_rl's `scripts/dukki_eval.py` setup (training randomisation kept, pushes off):

- `walk_profile.py` runs the robot's walk test on 32 simulated robots and counts falls, with switches for joint friction, kd, robotd's filters and an IMU pitch error.
- `stand_sweep.py` stands and walks 8 robots each with one change from training (delays, noise, friction, gains, calibration offsets, gear play) and reports how much the policy moves.
- `obs_dump.py`, `stand_specific.py` and `inair.py` were one-off comparisons (sections 5.1, 5.3 and 5.4).

## 2. The robot's settings

| Setting | Value | Why |
|---|---|---|
| Walk policy | 9250 | Recovered from pushes where every calmer-trained variant fell (section 4) |
| kp | 1.0 N·m/rad (`bus.bridge_kp_per_gain = 0.005` × gain 200) | What the Dukki policies train at; robotd's default gave 2.0 |
| kd | 0.07 N·m·s/rad (`bus.bridge_kd`) | Standing went from restless to still; section 5.7 |
| IMU median | off (`bus.bridge_imu_median = false`) | It made the IMU about 20 ms later than training allows; section 6 |
| Leg filter | 0.7 (`policy.legs_lowpass`) | 0.85 let a self-sustaining jitter take hold; 0.5 and below lose walks to friction (section 5.6) |
| Head filter | 0.5 (`policy.head_lowpass`) | 0.6 made no difference |
| Action scale | 1.0; `voltage_adapt` off; `idle_head` off | As trained; voltage scaling cut the actions to 78% on 6S |
| Other policy slots | `"none"` | The defaults are Pollen's alpha networks |
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
| 10750 (`dukki-walk-slowwalk2`, 371d60d9) | friction ramp to 1 to 5 times, fine speed reward, slow-walk commands, action-rate penalty halved | Three backward falls in a pushed stand, three times as twitchy (section 7) |

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

It wanders rather than growing, and does not track the restlessness. Which of the two is right after use is still open: pressing joints against their hard stops at power-up and after a session would tell. Unitree's own deployment example feeds the J288's policy the output encoder (section 8); `j288-testing.md` section 17 found the output encoder shifts by 0.6 to 1.4° with held torque, which the rotor does not.

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

With both feet planted, the trunk's pitch rate is the sum of each leg's pitch joint rates. In the restless stands the gyro's pitch rate leads that sum, taken from the rotor positions, by about 34 ms (correlation 0.6 to 0.7, five recordings). A rigid robot cannot do that. Something lets the body move before the servos register it: gear play under load, the frame flexing, or the soles. The simulation's backlash variant did not reproduce the restlessness, so its play is not that something. Measuring where the give is (push the trunk with the servos holding home and compare gyro, rotor and output encoder) is the next step (section 9).

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
| `dukki-walk-slowwalk` | friction ramped 1 to 3 → 1 to 5 over 1000 iterations, a fine speed reward (weight 1), 20% slow-walk commands | Stopped walking by 9750 |
| `dukki-walk-slowwalk2` | the ramp, fine speed reward weight 2, 50% slow-walk commands at 0.1 to 0.3 m/s, action-rate penalty −0.5 in place of −1.0 | Walks slowly (0.13 to 0.16 m/s for 0.15) and backwards, no falls of 32 at 3 to 5 times friction, 1 at 4 to 6 times; stopped turning in place and veers 0.1 to 0.3 rad/s |

Why the first two stopped: the stock speed reward scores the instantaneous speed with a std of 0.32 m/s, and a stepping body's speed swings within each stride by about as much as a slow command, so at 0.15 m/s standing still keeps 80% of the reward and so does 9250's walk. Higher friction makes stepping dearer through the action-rate penalty, the largest cost of walking, and the policy stops. The fine reward scores the speed error averaged over 1 s with std 0.1, so a slow walk earns well above standing; Pollen fixed their turning dead zone the same way, with a fine yaw term added beside the stock one (their microduck_rl 4048a3ea). Even then it stopped until the action-rate penalty was halved.

The original friction run was already drifting to standing at 1 to 3 times friction: 9250 walks 0.094 m/s at the 0.15 command, its 10000 0.046 and 10750 0.013. 9250 is a point on that slope.

slowwalk2's 10750 fell three times, backwards, in a pushed stand on the robot, with an action change of 0.2 to 0.56 a tick against 9250's 0.05 to 0.18. In simulation it is 30% twitchier than 9250 and spends 21% of its walking time with a joint above 18.6 rad/s, the fastest speed in the J288 bench identification, against 17%. Halving the action-rate penalty cost more on the robot than friction did. Keep it at −1.0.

What carried over from Pollen's microduck_rl (`develop` and branches to 10/11/2026): the fine yaw term and a turn-in-place command bucket for dead zones; their policies also walk slowly (0.2 m/s commanded, 0.09 achieved) and one would not start from standing at 0.2; their servo model's command delay (15 to 30 ms) is part of what the policy expects, and a test harness without it stood still; and they train on rough terrain with gear play, where these runs used flat ground without it.

## 8. Unitree's reinforcement-learning deployment example

Unitree published a deployment example for a small biped with 14 J288 or S288 servos and the LSM6DSV16X (support site, "Multi-Motor Reinforcement Learning Deployment Example", 16/10/2026). Its policy contract is the same 61 inputs as robotd's, and its code says it was ported from robotd (it carries "the original robotd control.rs target filter", off by default).

- For the J288 the policy's joint positions are the output encoder, the joint velocities the servo's own speed reading; the rotor position is a diagnostic option. Targets still go through the rotor position, aligned to the output encoder once, at the first command. For the S288 it is the reverse: rotor positions, and the output encoder for calibration, diagnostics and protection only.
- The S288 policies were trained with BAM's M1 model (gain profile `s288_m1_bam`, contract `rigid_rotor_pd_m1`). The J288 training code is not public.
- Calibration holds each joint by hand at the model's zero and reads the output encoder, with a direction test per servo. The IMU's gyro bias is averaged at startup; its mount is configured, not measured.
- Deployment limits target changes to 7 rad/s, ramps into the policy over 8 to 12 s, and offers a friction feed-forward (a torque in the direction of intended motion, a tanh step of up to 0.2 N·m, off until identified), which cancels friction on the robot rather than training around it.

## 9. Open items

- Find the give that lets the trunk move ahead of the joints (section 5.8): with the servos holding home, push the trunk gently and log the gyro, rotor positions and output encoders together.
- Repeat the pushed stand at kd 0.07 (one of two runs fell twice), and train the walk policy at kd 0.07 or with kd randomised around it.
- Repeat the walk test, then faster (0.25 m/s), at the current settings.
- Turning in place for any fine-tune that keeps the slow-walk switches: a fine yaw reward and the turn-in-place commands' share back.
- Which of rotor position and output encoder is right after use (section 5.3): hard stops at power-up and after a session.
- The IMU against the legs, about 5° (section 5.2): the mount or the jig.
- robotd's odometry for Dukki's legs, and its pick-up detection model, missing from the install (`pickup_detector.onnx`).
- Sit-stand and stand-up were trained at the old friction and before the joint stops; they need the same friction treatment before they are trusted on the robot.
