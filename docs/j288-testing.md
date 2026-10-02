# Microduck Unitree: J288 bench testing

![J288 stall test setup: the servo in a printed bracket on an aluminium bar clamped to the bench, with its 100 mm arm resting on a 5 kg kitchen scale](pictures/j288-stall-test-setup.jpeg)

The stall test setup. The J288 sits in a printed bracket on an aluminium bar clamped to the bench, with the 100 mm arm resting on the 5 kg kitchen scale. The same bracket and arm, without the scale, were used for the other tests.

Date: 27/09/2026, with the 6S stall repeat (29/09/2026), the BAM identification on the pendulum (section 15, 30/09/2026), and a trained policy on the pendulum and the gear play (sections 16 and 17, 02/10/2026).

This document records the bench characterisation of one J288 and its BAM model: what was run, the raw results, and what they mean for the servo bridge, the robot's mass budget and the actuator model in `microduck_rl`. The tools are in `scripts/j288/` (see its README for command lines) and the logs are in `scratch/j288_explore/`, which is not committed. Section numbers in `hardware.md` are cited as "hardware.md 5.1" and so on.

## 1. Summary

| Quantity                               | Result                                                                                                                                                       | Section  |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------- |
| Bus round trip, one servo, USB adapter | 0.21 ms median, 0.30 ms worst                                                                                                                                | 4        |
| Mode 0 (stop)                          | Windings shorted: viscous brake of 0.148 N·m·s/rad, not free                                                                                                 | 5        |
| Reported torque                        | Motor torque from current, before gearbox losses. Reads within 0.005 N·m of the command                                                                      | 5, 6     |
| Breakaway torque, unloaded             | 0.029 to 0.030 N·m                                                                                                                                           | 6        |
| Friction while moving                  | About 0.046 N·m (Coulomb term fitted over 0 to 15 rad/s)                                                                                                     | 7        |
| Armature (reflected rotor inertia)     | 7.8 × 10⁻⁴ kg·m², about 43% of the XL330 model's 1.81 × 10⁻³                                                                                                 | 7        |
| Torque response                        | Within one 1 ms loop period                                                                                                                                  | 7        |
| Reported speed                         | First-order filtered inside the servo, time constant about 15 ms. The firmware's kd term uses the filtered speed                                             | 9        |
| Motor torque limit                     | Hard clamp at 1.00 N·m (reported)                                                                                                                            | 10       |
| Output torque at stall                 | 0.48 × motor torque + 0.017 N·m, for 0.2 to 0.9 N·m of motor torque                                                                                          | 10       |
| Peak driving torque at the output      | 0.55 to 0.6 N·m (pendulum, section 15.6). The scale tests read 0.45 to 0.49 N·m, about 20% low                                                               | 10, 15   |
| Winding heating at stall               | 0.3 °C/s at 0.3 N·m, 1 °C/s at 0.5, 2 °C/s at 0.7, 3.5 °C/s at 0.9, 4.5 °C/s at the clamp (motor torque)                                                     | 10       |
| Compared with the XL330 as simulated   | Higher peak driving torque than the XL330 as trained (0.43 N·m), level with BAM's current XL330 (0.62 N·m), and full torque to about 20 rad/s against 4 to 8 | 11, 15.7 |
| BAM model                              | M6, 0.028 rad position error on a load the fit never saw. Bundled as `unitree_j288` in the bam fork                                                          | 15       |
| Trained policy on the pendulum         | Real tracking 2.2° against 2.5° in simulation; position within 0.011 rad of simulation on average, torque within 0.034 N·m                                     | 16       |
| Gear play inside the servo             | About 6 mrad (0.3°) total, from the rotor against the output encoder at zero current. The fixture added about 30 mrad more                                   | 17       |

The two findings with the most consequence:

- The J288's peak driving torque at the output is about 0.55 to 0.6 N·m, not the 1.5 N·m on Unitree's page. Unitree's figure is specified with the torque opposing the direction of motion, the back-driven case where gearbox friction helps. The first stall tests put it at 0.45 N·m; the pendulum identification showed they read about 20% low (section 15.6). It is somewhat above the XL330 the shipped policies were trained against, on a robot that gets substantially heavier (section 12), and it keeps that torque to far higher speeds (section 15.7).
- The windings heat quickly above about 0.5 N·m of motor torque. For the robot, sustained torque is likely to be the binding limit before peak torque is.

## 2. Setup

- One J288, ID 0 (factory default), in a printed bracket clamped to the bench.
- A 3D-printed arm of 6 g, about 100 mm from the output axis to the tip, on the servo's metal horn. As a uniform rod its inertia about the axis is 2.0 × 10⁻⁵ kg·m²; the horn's contribution is negligible.
- Unitree's USB to single-bus module (Artery AT32 CDC device, `/dev/cu.usbmodem3744CBC819741` on the Mac), powered from a 4S LiPo. The servo read 15.5 to 16.0 V throughout. Results are at 4S except the stall repeat on 6S (section 10.5); section 14 lists what still needs 6S.
- Test loop at 1 kHz from Python on the Mac, every command and reply logged.
- Stall tests: two kitchen scales under the arm tip, lever 100 mm. The first overloaded at about 560 g. The second is the 5 kg, 1 g resolution scale in the photo.

The torque sign convention: positive torque turns the bench arm counter-clockwise, into the scale.

Limits of the setup, visible in the photo:

- The servo sits in a printed plastic bracket with little heat sinking. The heating rates in section 10 are for that mounting; a servo bolted to metal will settle cooler over minutes.
- The arm meets the scale near the edge of the pan, and its rounded tip makes the exact lever uncertain by a few millimetres. Off-centre loading and lever error both add to the scatter between runs.
- The scale stands on a stack of paper and other objects rather than directly on a rigid surface, which may account for some of the settling seen during holds.

## 3. Tools

`scripts/j288/j288.py` is the serial driver: frame building, Unitree's CRC32, reply parsing and conversion to output-side SI units using the exact gear ratio 70070/243 (288.354). It was checked against Unitree's Python demo (`unitreerobotics/digital_servo`, `python/servo_demo.py`): identical CRCs on 2,000 random frames and identical command frames on 500 random commands. It avoids two bugs in the demo, which packs the position as unsigned (failing on any negative position) and reads the winding temperature and voltage as signed bytes.

`scripts/j288/explore.py` holds the tests: `hand`, `ramp`, `steps`, `position` and `stall`.

## 4. Communication

Probe: 200 stop frames, no motion.

| Measure                                                | Result                                                            |
| ------------------------------------------------------ | ----------------------------------------------------------------- |
| Valid replies                                          | 200 of 200 on the first CDC port; the second port does not answer |
| Round trip                                             | 0.21 ms median, 0.26 ms 95th percentile, 0.30 ms maximum          |
| State at rest                                          | ID 0, mode 0, no faults, housing 31 °C, winding 34 °C, 16.0 V     |
| Rotor-derived position against output encoder, at rest | 2.4214 against 2.4245 rad                                         |

At rest the two position readings agree to about four output-encoder counts, which suggests the multi-turn rotor position is seeded from the output encoder at power-up.

Behaviour found along the way:

- Each reply reports the state from before its own command was applied, a one-frame lag. At 1 kHz that is under 1 ms and irrelevant to the model, but it matters when reading logs.
- Timeout latch. With timeout protection enabled, a gap of about 1 s without frames latches the timeout bit and the servo then ignores enable commands. A frame with the timeout bit clear resets it. In one trial the clear showed in the reply to the tenth such frame (2.8 ms); in three later trials, with 0, 5 and 20 ms between frames, it showed in the second. The tools keep sending clearing frames for up to 0.5 s. The bridge firmware needs the same handling after any interruption.

## 5. Operating modes (hand test)

Log: `hand_2026-09-27_12h04m47.json`. The arm was turned by hand in each phase. Reported torque is fitted as a × speed + b × sign(speed) over the samples where the arm moved faster than 0.3 rad/s.

| Phase                        | How it felt     | Hand speeds reached | Reported torque fit                    |
| ---------------------------- | --------------- | ------------------- | -------------------------------------- |
| Mode 0 (stop)                | Resistance      | up to 2.1 rad/s     | −0.148 × speed, Coulomb term 0.001 N·m |
| Mode 1, kp = kd = torque = 0 | No resistance   | up to 5.2 rad/s     | ≈ 0 (0.001 × speed)                    |
| Mode 1, kd = 0.05 N·m·s/rad  | Some resistance | up to 3.2 rad/s     | −0.0484 × speed                        |

What this shows:

- Mode 0 shorts the windings. It is a viscous brake about three times stronger than kd = 0.05, not a free-wheel. Torque-off stretches of a log therefore need modelling as braking. For the robot, a limp joint will feel damped.
- Reported torque is the motor's own current-based torque. It stayed at zero while the arm was turned in zero-torque mode (so gearbox friction is not in it) and showed the braking torque in mode 0, where nothing was commanded.
- The damping gain came out 3% under the command (0.0484 against 0.05), confirming the rotor-to-output gain conversion (output gain = rotor gain × 288.35²).
- A shorted motor's damping is kt·ke/R. With Unitree's torque constant of 0.554 N·m/A and a back-EMF constant of about 0.72 V per rad/s (from the rated no-load speeds), 0.148 N·m·s/rad implies about 2.7 Ω, referred to the output. That is a starting value for identification, not a measurement.

The rotor-derived position and the output encoder disagreed while moving: a mean offset of −18 mrad, a shift of 10 to 15 mrad depending on direction, and 17 mrad of scatter unexplained by speed or direction. Timing skew was ruled out (under 1 ms). Later logs showed the output encoder jumping by tens of mrad between consecutive samples while the output moves, so most of the scatter is the output encoder, which is only trustworthy at rest. The direction-dependent part is consistent with gear play of a few tens of mrad and is still to be measured properly (section 14).

## 6. Breakaway friction (ramp)

Log: `ramp_2026-09-27_12h10m21.json`. Torque ramped at 0.01 N·m/s from rest in each direction, unloaded.

| Direction | Commanded torque at first motion (> 0.05 rad/s) | Reported at that moment | Commanded at > 0.3 rad/s |
| --------- | ----------------------------------------------- | ----------------------- | ------------------------ |
| Positive  | 0.0299 N·m                                      | 0.0237 N·m              | 0.0303 N·m               |
| Negative  | 0.0285 N·m                                      | 0.0304 N·m              | 0.0350 N·m               |

Unitree quotes 0.04 N·m to overcome static friction. Over the ramps, reported torque sat 0.004 N·m below the command, with 0.0012 N·m of noise. The arm's own gravity torque is under 0.01 N·m and is not corrected for.

## 7. Torque steps: inertia and response

Log: `steps_2026-09-27_12h10m34.json`. Torque steps from rest, each braked when the speed estimate passed 10 rad/s. Acceleration is from a quadratic fit to position after the first 5 ms.

| Step      | Mean reported torque | Acceleration  | True speed at cutoff |
| --------- | -------------------- | ------------- | -------------------- |
| +0.10 N·m | +0.0946 N·m          | +66.6 rad/s²  | +11.8 rad/s          |
| −0.10 N·m | −0.1028 N·m          | −64.1 rad/s²  | −11.8 rad/s          |
| +0.20 N·m | +0.1952 N·m          | +196.3 rad/s² | +13.3 rad/s          |
| −0.20 N·m | −0.2034 N·m          | −193.3 rad/s² | −13.4 rad/s          |
| +0.30 N·m | +0.2935 N·m          | +315.0 rad/s² | +15.0 rad/s          |
| −0.30 N·m | −0.3013 N·m          | −313.5 rad/s² | −14.9 rad/s          |

Fitting torque = J × acceleration + friction over the six steps:

| Parameter                         | Result                                 |
| --------------------------------- | -------------------------------------- |
| Total inertia J                   | 7.97 × 10⁻⁴ kg·m² (residual 0.005 N·m) |
| Arm                               | 0.20 × 10⁻⁴ kg·m²                      |
| Armature (J minus arm)            | 7.8 × 10⁻⁴ kg·m²                       |
| Implied rotor inertia (÷ 288.35²) | 0.09 g·cm²                             |
| Friction while moving             | 0.046 N·m                              |

- Friction while moving (0.046 N·m) is above breakaway (0.030 N·m), so friction grows with speed, load or both. The loaded pendulum recordings will separate these, which is what BAM's viscous and load-dependent terms are for.
- Reported torque reached the commanded value in the first reply after each step (within 1 ms) and changed sign within 1 ms when braking began. At the simulation's 5 ms step the torque loop can be treated as instantaneous.
- At this test's 10 rad/s cutoff the arm was really at 12 to 15 rad/s. The cutoff then used the servo's reported speed, which lags (section 9); the tools now use a position difference instead.

## 8. Position loop

Log: `position_2026-09-27_12h10m48.json`. kp = 0.5 N·m/rad and kd = 0.05 N·m·s/rad (output side, Unitree's quick-start values). Holds, then steps of +0.5, −0.5 and +1.0 rad and back.

| Fit of reported torque to kp·(target − position) − kd·speed + offset | kp    | kd    | Residual   |
| -------------------------------------------------------------------- | ----- | ----- | ---------- |
| Using rotor-derived position                                         | 0.489 | 0.046 | 0.0106 N·m |
| Using the output encoder                                             | 0.463 | 0.044 | 0.0118 N·m |

- The rotor-derived position fits slightly better, consistent with Unitree's description of a rotor-side PD loop, but the evidence is weak with this light arm.
- Every step settled about 0.047 rad short of target: 0.023 N·m of torque held by friction.
- Peak speeds were 4 to 9 rad/s.

## 9. Speed feedback filter

From the step and position logs. The servo's reported speed was compared with the derivative of rotor-derived position, resampled at 1 kHz.

| Model of reported speed     | RMS error  |
| --------------------------- | ---------- |
| Unfiltered derivative       | 2.49 rad/s |
| First-order low-pass, 10 ms | 0.78 rad/s |
| First-order low-pass, 12 ms | 0.50 rad/s |
| First-order low-pass, 15 ms | 0.12 rad/s |
| First-order low-pass, 20 ms | 0.42 rad/s |

In a +0.20 N·m step, at 60 ms the arm was turning at 12.1 rad/s while the servo reported 9.4 rad/s.

Which speed the firmware's damping uses, from the braking phases (commanded kd = 0.02):

| Speed used in the fit                     | Fitted kd | Residual  |
| ----------------------------------------- | --------- | --------- |
| Reported (filtered) speed                 | 0.0187    | 0.024 N·m |
| First-order low-pass of true speed, 14 ms | 0.0187    | 0.025 N·m |
| True speed                                | 0.0199    | 0.040 N·m |

The damping torque tracks the filtered speed. Consequences:

- The damping from kd lags the motion by about 15 ms. The XL330 was voltage controlled, so back-EMF gave it about 0.05 N·m·s/rad of physical, unfiltered damping. The J288's current loop cancels back-EMF, so kd plus friction is all the damping it has.
- The filter's corner is about 10.6 Hz. For a joint of about 2 × 10⁻³ kg·m², kd keeps about 95% of its damping at kp = 0.5 N·m/rad (natural frequency 2.5 Hz), 82% at kp = 2 and 64% at kp = 5. Above that, most of the damping is lost.
- The BAM model needs this filter on the kd term, or the simulation will be better damped than the robot.
- The policy's joint velocity observation should come from position differences on the bridge, not from the servo's speed field.
- If much stiffer joints are ever needed, the bridge can apply the damping itself through tau_ff, using its own low-lag speed estimate.

The evidence that kd uses the filtered speed is good but not conclusive; section 14 lists the check.

## 10. Stall torque

The arm pressed on a scale at the 100 mm lever. Output torque = scale reading × 9.81 × 0.100 m. Every run used pure torque commands (kp = kd = 0). Runs A to C and the cold clamp run were on 4S (15.5 V); run D repeated them on 6S (section 10.5).

### 10.1 Runs

| Log (27/09/2026 unless noted)              | Scale        | Outcome                                                                                                                                                                                                                  |
| ------------------------------------------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `stall_..._12h56m22`, `stall_..._12h57m09` | First        | Aborted. The script inferred the torque direction from a small probe pulse, read gear play as the arm lifting off, and drove the wrong way into the travel limit. Replaced by an explicit `--sign` and an approach phase |
| `stall_..._13h02m18`                       | First        | 0.1 to 0.3 N·m. Readings were taken after the torque released and showed about 30 g, the friction holding the arm down. Discarded; the script now holds the torque until the reading is entered                          |
| `stall_..._13h07m51` (run A)               | First        | 0.1 to 0.9 N·m. The scale overloaded at 1.1 N·m. Readings were typed in but not saved to the log (fixed since)                                                                                                           |
| `stall_..._13h16m12` (run B)               | Second, 5 kg | 0.1 to 1.3 N·m. Stopped at the 70 °C winding limit                                                                                                                                                                       |
| `stall_..._13h39m55` (run C)               | Second, 5 kg | 0.1 to 0.9 N·m. The 1.1 N·m level stopped at the winding limit without a reading                                                                                                                                         |
| `stall_..._13h49m10`                       | Second, 5 kg | 1.1 N·m alone, from cold, for a clean reading at the clamp                                                                                                                                                               |
| `stall_2026-09-29_22h28m43` (run D, 6S)    | Second, 5 kg | 0.1 to 1.1 N·m on a 6S pack (24.0 to 24.5 V), started cold. Complete, including a reading at the clamp (section 10.5)                                                                                                    |

### 10.2 Readings

| Commanded       | Reported | Run A scale (g) | Run B scale (g) | Run C scale (g)    | Mean output        | Output ÷ reported |
| --------------- | -------- | --------------- | --------------- | ------------------ | ------------------ | ----------------- |
| 0.10            | 0.096    | 60              | 68              | 76                 | 0.067 ± 0.006 N·m  | 0.67              |
| 0.20            | 0.195    | 115             | 95              | 100                | 0.101 ± 0.008 N·m  | 0.51              |
| 0.30            | 0.295    | 153             | 158             | 150                | 0.151 ± 0.003 N·m  | 0.50              |
| 0.50            | 0.495    | 340             | 257             | 260                | 0.280 ± 0.038 N·m  | 0.56              |
| 0.70            | 0.695    | 330             | 409             | 333                | 0.351 ± 0.036 N·m  | 0.50              |
| 0.90            | 0.894    | 480             | 420             | 449                | 0.441 ± 0.024 N·m  | 0.49              |
| 1.10            | 0.996    | overload        | 440             | stopped            | 0.432 N·m          | 0.43              |
| 1.30            | 0.999    |                 | about 440       |                    | 0.43 N·m           | 0.43              |
| 1.10, from cold | 0.996    |                 |                 | 480 falling to 460 | 0.471 to 0.451 N·m | 0.45 to 0.47      |

Spread is the standard deviation across runs A to C. The 1.30 reading was typed after the release and is read here as 440 g.

- Pooling runs A to C up to 0.9 N·m (18 points): output = 0.483 × reported + 0.017 N·m, residual 0.027 N·m. The ratio sits at about 0.5 from 0.2 to 0.9 N·m.
- Reported torque follows the command to within 0.005 N·m up to 0.9 N·m, then clamps at 0.996 to 1.000 N·m. The supply held at 15.5 V throughout, so this is a firmware current limit, not the battery.
- At the clamp, from cold, the output was 0.45 to 0.47 N·m. Superseded: the pendulum identification shows these scale readings were about 20% low, and the peak driving torque is about 0.55 to 0.6 N·m (section 15.6).
- Run B flattened at 0.40 to 0.43 N·m from 0.7 N·m up; run C did not repeat it, reaching 0.44 N·m at 0.9. Treated as scatter.
- During the cold hold at the clamp, rotor position (48.0 mrad), output encoder and reported torque (0.996 N·m) were constant while the winding rose from 39 to 64 °C, and the scale fell from 480 to 460 g. The drop is either real torque falling as the magnets warm (reported torque uses a fixed torque constant and would not show it) or the scale or plastic arm settling. Unresolved.

### 10.3 Why the output is half the motor torque

A 288:1 spur gearbox has several stages; at 88 to 90% each, five stages leave 55 to 60%. The same friction explains Unitree's quoted 1.5 N·m. That figure is specified with the torque opposing the motion, which means the output is being back-driven and friction adds to what the motor holds: roughly 1.0 N·m ÷ 0.6 to 0.67. Load-dependent, direction-dependent friction of this kind is what BAM's M6 model describes. The back-driven holding torque itself has not been measured yet (section 14).

### 10.4 Heating

Winding temperature rise during the stall holds, by motor torque:

| Motor torque    | Run B (4S)        | Run C (4S) | Run D (6S) |
| --------------- | ----------------- | ---------- | ---------- |
| 0.3 N·m         | 0.39 °C/s         | 0.24 °C/s  | 0.25 °C/s  |
| 0.5 N·m         | 1.05 °C/s         | 1.08 °C/s  | 1.09 °C/s  |
| 0.7 N·m         | 1.97 °C/s         | 2.33 °C/s  | 2.17 °C/s  |
| 0.9 N·m         | 3.67 °C/s         | 3.49 °C/s  | 3.41 °C/s  |
| 1.0 N·m (clamp) | 4.11 to 4.50 °C/s | 4.46 °C/s  | 4.43 °C/s  |

The cold clamp hold rose at the same 4.5 °C/s. Heating scales roughly with torque squared, as resistive loss should. At 0.9 N·m, the windings would reach Unitree's 120 °C shutdown in well under a minute. The housing rose only 1 to 3 °C per hold, so the winding sensor is the one to watch.

Supply voltage does not help. Torque is set by current, heat by current squared, and at low speed the current needed for a torque does not depend on the supply. Voltage matters only at speed, where back-EMF limits the torque available. Run D confirms it: on 6S the heating rates match the 4S runs at every level.

### 10.5 Repeat on 6S

Log: `stall_2026-09-29_22h28m43.json` (29/09/2026). Same 100 mm arm and 5 kg scale as runs B and C, on a 6S pack. The servo read 24.0 to 24.5 V throughout and started at 29 °C.

| Commanded | Reported | Scale (g) | Output    | Output ÷ reported | 4S mean output (runs A to C) |
| --------- | -------- | --------- | --------- | ----------------- | ---------------------------- |
| 0.10      | 0.095    | 46        | 0.045 N·m | 0.48              | 0.067 ± 0.006 N·m            |
| 0.20      | 0.195    | 110       | 0.108 N·m | 0.55              | 0.101 ± 0.008 N·m            |
| 0.30      | 0.294    | 194       | 0.190 N·m | 0.65              | 0.151 ± 0.003 N·m            |
| 0.50      | 0.495    | 265       | 0.260 N·m | 0.52              | 0.280 ± 0.038 N·m            |
| 0.70      | 0.694    | 356       | 0.349 N·m | 0.50              | 0.351 ± 0.036 N·m            |
| 0.90      | 0.894    | 491       | 0.482 N·m | 0.54              | 0.441 ± 0.024 N·m            |
| 1.10      | 0.996    | 498       | 0.489 N·m | 0.49              | 0.43 to 0.47 N·m (clamp)     |

- The clamp is unchanged: 0.996 N·m reported for the 1.1 N·m command, as on 4S. The firmware current limit does not depend on the supply voltage.
- Output torque is the same as on 4S within the setup's scatter. The ratio is about 0.5 (0.49 to 0.55 over the useful range); the 0.30 point is high and the 0.10 point low, as individual points varied between the 4S runs.
- The clamp level finished at 63 °C and gave a clean reading of 0.489 N·m, at the top of the 4S range (0.43 to 0.47 N·m). The scale put peak driving torque at 0.45 to 0.49 N·m; section 15.6 revises it to about 0.55 to 0.6 N·m.
- The torque-speed limit, which should differ on 6S, needs moving tests and is not covered here (section 14).

## 11. Comparison with the XL330 in the simulation

`microduck_rl` models the XL330 with BAM's M6 model (`bam/params/xl330/m6.json`) at the pinned bam commit 62bd8ce. The XL330 actuator class there sets a 1.75 A firmware current limit, which the mjlab path applies because the robot config leaves its own override commented out. With kt = 0.366 N·m/A that caps motor torque at 0.641 N·m. The voltage limit does not bind at stall anywhere in training's 6.5 to 8.2 V range.

Running BAM's own friction function for the load the modelled XL330 can lift from rest:

| XL330 motor torque        | Load it lifts from rest | Output ÷ motor |
| ------------------------- | ----------------------- | -------------- |
| 0.10 N·m                  | 0.059 N·m               | 0.59           |
| 0.30 N·m                  | 0.195 N·m               | 0.65           |
| 0.50 N·m                  | 0.330 N·m               | 0.66           |
| 0.641 N·m (current limit) | 0.425 N·m               | 0.66           |

It holds up to 0.897 N·m before being back-driven.

|                                   | XL330 as simulated | J288 as measured (4S and 6S)                            |
| --------------------------------- | ------------------ | ------------------------------------------------------- |
| Motor torque limit                | 0.64 N·m (1.75 A)  | 1.00 N·m                                                |
| Output ÷ motor, driving from rest | 0.59 to 0.66       | about 0.48                                              |
| Peak driving torque at the output | 0.425 N·m          | 0.55 to 0.6 N·m (pendulum; the scale read 0.45 to 0.49) |
| Holding torque when back-driven   | 0.90 N·m           | 1.5 N·m (Unitree), not measured                         |

The stall test pushed into the scale from rest, with friction opposing the motor, so it measures the same quantity as the "lifts from rest" column. The ROBOTIS rating of 0.52 N·m at 5 V is a datasheet figure and was never the simulation's number.

An inconsistency found on the way: `scripts/infer_policy.py` in `microduck_rl` sets `BAM_MAX_CURRENT = None` with the comment "training runs WITHOUT the firmware current limiter", and `tests/test_infer_policy_bam.py` asserts it. At the pinned commit, training does apply 1.75 A (the class default since 13/07/2026), so the CPU rehearsal runs a stronger XL330 than training does.

## 12. Torque demand of the shipped policies on a heavier robot

A headless run of the shipped policies (`pollen-robotics/microduck-policies`: `velstand.onnx` for standing, walking and turning, `alpha_sitstand.onnx` for sitting and standing) in CPU MuJoCo with the XL330 M6 actuator they were trained against, including the 1.75 A limit. Two robots:

- the model as trained, 0.737 kg;
- a J288 variant, 1.206 kg: 21 g more per servo (39 g against 18 g) on each joint's parent body, 100 g more in the head body, 75 g more battery in the trunk. The chosen pack has since been weighed at 132 g, about 32 g more than the NP-F550 rather than 75 g, so this variant is about 43 g heavier than the real robot will be (roughly 1.16 kg) and its torque figures are slightly pessimistic.

`velstand` walked, turned and stood on the heavier robot without falling. Torque is the net torque delivered to the joint (actuator torque with the friction constraint applied), excluding the first 2 s.

| Scenario      | Worst joint | Peak, trained → heavy | Time above 0.43 N·m, trained → heavy |
| ------------- | ----------- | --------------------- | ------------------------------------ |
| Standing      | knee        | 0.13 → 0.21 N·m       | 0% → 0%                              |
| Walk 0.15 m/s | knee        | 0.15 → 0.27 N·m       | 0% → 0%                              |
| Turn 1 rad/s  | knee        | 0.16 → 0.26 N·m       | 0% → 0%                              |
| Walk 0.30 m/s | knee        | 0.47 → 0.54 N·m       | 0.1% → 1.5%                          |
| Sit and stand | knee        | 0.64 → 0.85 N·m       | 0.8% → 11%                           |
| Sit and stand | hip pitch   | 0.53 → 0.58 N·m       | 0.4% → 5%                            |
| Sit and stand | neck pitch  | 0.47 → 0.58 N·m       | 0.2% → 0.9%                          |

Static torque to hold the standing pose, per knee: 0.08 N·m trained, 0.13 N·m heavy.

- Standing, turning and normal walking have headroom against the J288's peak driving torque (0.45 N·m when this was written; about 0.55 to 0.6 N·m after section 15.6, so more headroom than the table shows).
- Top-speed walking brushes the limit in short spikes.
- Sitting and standing exceed 0.43 N·m on the heavy robot for 5 to 11% of the motion, and peak at 0.85 N·m at the knee, beyond even the revised 0.55 to 0.6 N·m. The XL330 model already reaches its own current limit there. Behaviours like this need retraining against J288 limits, or a lighter robot.
- The heavier head roughly doubles the neck torques.
- Heat may bind before peak torque. Holding a knee at 0.13 N·m of output needs somewhere between about 0.1 N·m of motor torque (gearbox friction helping to hold) and 0.4 N·m (motor driving through it), and where the temperature settles over minutes is unknown.

Caveats: the policies are XL330-trained, with XL330 stiffness and damping and no action delay, and the servo mass placement is an approximation. These are demands of the existing policies, not limits of a J288-trained one. The harness is not in the repository yet.

## 13. Starting parameters for the J288 actuator model

These were the starting values for identification. The fitted values are in section 15.4.

| Parameter               | Starting value                                                      | Source                         |
| ----------------------- | ------------------------------------------------------------------- | ------------------------------ |
| Gear ratio              | 70070/243 = 288.354                                                 | Unitree demo code              |
| Control law             | tau = tau_ff + kp·(p_des − q) + kd·(w_des − LP₁₅(w)), rotor-side PD | Unitree docs, sections 8 and 9 |
| Speed filter on kd      | First order, about 15 ms                                            | Section 9                      |
| Torque loop             | Instantaneous at a 5 ms step                                        | Section 7                      |
| Motor torque limit      | 1.00 N·m                                                            | Section 10                     |
| Armature                | 7.8 × 10⁻⁴ kg·m²                                                    | Section 7                      |
| Breakaway friction      | 0.03 N·m                                                            | Section 6                      |
| Friction while moving   | about 0.046 N·m                                                     | Section 7                      |
| Load-dependent friction | Output about 0.48 × motor torque when driving                       | Section 10                     |
| Mode 0                  | Viscous brake, 0.148 N·m·s/rad                                      | Section 5                      |
| Torque constant         | 0.554 N·m/A                                                         | Unitree spec page              |
| Back-EMF constant       | about 0.72 V per rad/s                                              | Unitree no-load speeds         |
| Resistance              | about 2.7 Ω (output-referred)                                       | Section 5, estimate            |

The mjlab `BamActuator` at the pinned commit accepts only voltage-controlled actuators, so the J288 needed its own actuator class and mjlab path. Both are now in the bam fork (section 15.8).

## 14. Open questions and next tests

Done since the first version: the 6S stall repeat (section 10.5), the pendulum rig and the BAM identification (section 15), and the check at the training timestep (section 15.9).

- Thermal soak: hold about 0.2 N·m at the output (the heavy robot's standing knee) for 5 to 10 minutes, once reached from below (driving) and once from above (friction helping), logging winding temperature. Answers whether standing is sustainable and gives the data for a winding-temperature model alongside BAM.
- Back-driven holding torque: M6 predicts 2.1 N·m, M3 and M4 1.5 to 1.6 N·m, Unitree quotes 1.5 N·m. No recording back-drove the servo that hard; a pendulum setup well beyond capacity (the 150 mm arm with 588 g) or a stall run that ramps down would settle it.
- Voltage-limited torque: no recording reached it (peak 18.6 rad/s), so `R` and `ke` are not separated and the torque-speed envelope above about 20 rad/s rests on the assumed back-EMF constant. Fast moves on the light setups at high kp would reach it.
- Speed filter: 15.3 ms from the pendulum logs against 17.0 ms from the 4S bench logs with the bare arm. Unexplained.
- Gearbox wind-up: the rotor-derived angle and the output encoder drift apart by up to 30 to 40 mrad within a recording under load (section 15.2). The model treats the gearbox as rigid. Section 17 suggests most of this was not wind-up: the gearbox play is about 6 mrad and the encoder appears to be disturbed by winding current.
- Output encoder under current: does the winding current really shift the output encoder's reading (section 17.3)? A check with the output clamped, the encoder read at several held torques and at zero current in between, would settle it.
- Play outside the servo: in section 17 the output shaft moved about 30 mrad inside the clamp, through the horn, its spline, the horn screw or the fixture. Rock the horn on the shaft by hand with the servo braked, then repeat the test with the clamp directly on the horn.
- Internal PD loop rate: not documented and not yet measured.

## 15. BAM identification on the pendulum

Dates: 29/09/2026 to 30/09/2026. The servo, bracket and bench were as in section 2, on the 6S pack (24.0 to 24.5 V throughout). The arms and masses are listed under "Test arm setups" above. The tools are in the bam fork (`../bam`, branch `unitree-j288`): `bam.unitree.calibrate`, `bam.unitree.record` and `bam.unitree.all_record` for recording, `bam.unitree.fit_control` for the control-law fit. The logs and fit outputs are in the fork's working directory (`data_raw/`, `data_raw_validation/`, `data_proc/`, `fits/`) and are not committed.

### 15.1 Setups

| Arm    | Total | End mass | Peak gravity torque | Load inertia (× 10⁻³ kg·m²) | Mass inertia (`--mass-inertia`) | Role                                                 | Recordings used | Set aside                                 | Winding peak |
| ------ | ----- | -------- | ------------------- | --------------------------- | ------------------------------- | ---------------------------------------------------- | --------------- | ----------------------------------------- | ------------ |
| 75 mm  | 70 g  | 55 g     | 0.046 N·m           | 0.36                        | 2.08e-05                        | Training: light, armature-dominated                  | 22              | 1 (loop gap)                              | 42 to 43 °C  |
| 75 mm  | 246 g | 231 g    | 0.175 N·m           | 1.41                        | 8.73e-05                        | Training                                             | 22              | none                                      | 41 to 44 °C  |
| 75 mm  | 415 g | 400 g    | 0.300 N·m           | 2.43                        | 1.51e-04                        | Validation, never fitted (reduced session)           | 7               | 1 (loop gap)                              | 41 to 45 °C  |
| 75 mm  | 586 g | 571 g    | 0.425 N·m           | 3.46                        | 2.16e-04                        | Training: at the driving limit                       | 22              | 2 (chirp past the limit; loop gap)        | 41 to 52 °C  |
| 150 mm | 108 g | 91 g     | 0.146 N·m           | 2.21                        | 3.44e-05                        | Training                                             | 22              | 3 (loop gaps), plus 19 repeats kept aside | 40 to 45 °C  |
| 150 mm | 333 g | 316 g    | 0.477 N·m           | 7.36                        | 1.19e-04                        | Training: at the driving limit                       | 22              | 4 (loop gaps)                             | 46 to 52 °C  |
| 150 mm | 417 g | 400 g    | 0.601 N·m           | 9.28                        | 1.51e-04                        | Training: beyond the driving limit (reduced session) | 14              | none                                      | 46 to 55 °C  |

- Peak gravity torque is with the arm horizontal. Load inertia includes the masses' own inertia: discs 55 mm across, swinging flat, m·r²/2 (6.7% of the load on the 75 mm arm, 1.7% on the 150 mm arm).
- A full session is 22 recordings: BAM's `sin_time_square`, `sin_sin`, `lift_and_drop` and `up_and_down` at four gain pairs (kp 0.97 and 2.01 N·m/rad × kd 0.02 and 0.05 N·m·s/rad), plus a torque chirp and torque steps (kp = kd = 0), two speed-mode runs and two damping-mode runs. The validation session used kp 0.97/kd 0.02 and kp 4.03/kd 0.05, twice the stiffest fitted gain.
- Every recording set aside was re-recorded. All but one were for loop pauses over 10 ms (the Mac pausing the recording process), which the recorder now detects and sets aside by itself; running under `caffeinate -i` helps. The 586 g torque chirp went past the ±2 rad soft limit at the pendulum's natural frequency; chirps now default to a quarter of the gravity torque.
- The 108 g session was run twice. The 19 earlier copies are kept in `data_raw/l150_m108/repeat/` and out of the fit. Repeated recordings agree to within 0.007 to 0.05 rad RMS, a floor on how closely any model can match this rig.

### 15.2 Calibrating the hanging angle

BAM measures pendulum angles from the arm hanging straight down. Two methods were tried on the 75 mm arm:

| Method                                                                                   | Result                                                                                                                                                                                        |
| ---------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Torque: hold at angles either side of hanging, from both directions, and fit gravity     | Three runs disagreed by up to 0.07 rad. The gearbox friction is wide (0.03 to 0.06 N·m) and strongly load-dependent, and leaves 0.01 to 0.02 N·m that no fit separated from the hanging angle |
| Level: arm set horizontal by hand on each side with a level, output encoder read at each | The two sides agreed to 0.008 rad (75 mm arm) and 0.007 rad (150 mm arm). Used for all recordings                                                                                             |

The torque method has since been removed from `bam.unitree.calibrate`, which now uses the level method only:

```
python -m bam.unitree.calibrate --port /dev/cu.usbmodem3744CBC819741 --output zero_150mm.json
```

The zero is stored against the absolute output encoder and recovered at the start of each recording. Across recordings it scatters by about 10 mrad (standard deviation). Reading it with the arm lifted 0.3 rad to load the gears did not reduce that: within a single recording the rotor-derived angle and the output encoder drift apart by up to 30 to 40 mrad once the arm has worked against its load, so the gearbox winds up and holds part of it through friction. At 10 mrad, the effect on gravity torque is under 0.006 N·m on the heaviest setup.

Section 17 revises this. The play inside the servo measured about 6 mrad, and the output encoder appears to read wrong while the winding carries current, so the 30 to 40 mrad drift under load was probably mostly an encoder effect, with some play outside the servo, rather than wind-up. The zero itself is read at rest, without current, and is not affected.

### 15.3 Fitting

Two stages. Stage 1 fits the control law directly to the logged reported torque (no simulation); stage 2 is BAM's own fit, with the stage 1 values held fixed, on 124 recordings resampled at 2 ms.

| Stage 1 parameter     | Fitted                                     | Notes                                                                                                            |
| --------------------- | ------------------------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| Speed filter          | 15.3 ms                                    | 17.0 ms from the 4S bench logs (bare arm); unexplained                                                           |
| Torque limit          | 1.002 N·m                                  | 0.996 N·m at the stall tests' clamp                                                                              |
| R                     | 2.81 Ω                                     | `ke` held at 0.72 V per rad/s: nothing reached the voltage limit (peak 18.6 rad/s), so the two are not separated |
| Replayed torque error | 0.0048 N·m mean, 0.009 N·m 95th percentile | About the servo's own noise                                                                                      |

| Model | Adds                         | Training error | Validation error (415 g, not fitted) | Validation torque error | Lift from rest | Hold when back-driven |
| ----- | ---------------------------- | -------------- | ------------------------------------ | ----------------------- | -------------- | --------------------- |
| M1    | Coulomb and viscous friction | 0.058 rad      | 0.049 rad                            | 0.040 N·m               | 0.95 N·m       | 1.06 N·m              |
| M2    | + Stribeck                   | 0.058          | 0.047                                | 0.041                   | 0.95           | 1.06                  |
| M3    | + load-dependent friction    | 0.045          | 0.036                                | 0.024                   | 0.65           | 1.52                  |
| M4    | + Stribeck on the load terms | 0.041          | 0.037                                | 0.024                   | 0.61           | 1.61                  |
| M5    | + direction-dependent        | 0.030          | 0.029                                | 0.024                   | 0.62           | 2.15                  |
| M6    | + quadratic load term        | 0.030          | 0.028                                | 0.023                   | 0.62           | 2.12                  |

M6 was fitted for 20,000 trials, the others for 5,000. M6 is the chosen model; M5 is essentially as good with fewer parameters. The friction models that ignore load (M1, M2) are clearly worse: the J288's friction depends on load and on direction.

### 15.4 M6 parameters

| Parameter                             | Value            | Bench measurement                                                                    |
| ------------------------------------- | ---------------- | ------------------------------------------------------------------------------------ |
| Armature                              | 6.3 × 10⁻⁴ kg·m² | 7.8 × 10⁻⁴ (section 7)                                                               |
| Coulomb friction                      | 0.012 N·m        |                                                                                      |
| Stribeck friction (added at rest)     | 0.012 N·m        | Static total 0.024 N·m against 0.03 N·m breakaway (section 6)                        |
| Viscous friction                      | 0.0053 N·m·s/rad |                                                                                      |
| Load friction, motor side / load side | 0.065 / 0.265    | The load-side term carries most of the gearbox loss                                  |
| Command delay                         | 0.02 ms          | Effectively zero on the bench adapter                                                |
| Zero offset                           | +19.7 mrad       | One value shared by two separately calibrated arms, plus any systematic gear wind-up |

The six fitted parameter sets are bundled in the bam fork as `bam/params/unitree_j288/m1.json` to `m6.json`, so they load by name: `load_model(motor_name="unitree_j288", model="m6")`, or `J288BamActuatorCfg(motor_name="unitree_j288", model="m6", ...)` in mjlab.

### 15.5 Error by setup (M6)

| Setup         | Position error | Validation recording (75 mm, 415 g) | Position error | Torque error |
| ------------- | -------------- | ----------------------------------- | -------------- | ------------ |
| 75 mm, 70 g   | 0.012 rad      | `sin_time_square`, kp 0.97, kd 0.02 | 0.019 rad      | 0.025 N·m    |
| 75 mm, 246 g  | 0.021          | `sin_time_square`, kp 4.03, kd 0.05 | 0.008          | 0.040        |
| 75 mm, 586 g  | 0.032          | `up_and_down`, kp 0.97, kd 0.02     | 0.039          | 0.036        |
| 150 mm, 108 g | 0.022          | `up_and_down`, kp 4.03, kd 0.05     | 0.008          | 0.037        |
| 150 mm, 333 g | 0.049          | `speed_sine`, kd 0.05               | 0.033          | 0.006        |
| 150 mm, 417 g | 0.053          | `lift_and_damp`, kp 2.01, kd 0.05   | 0.026          | 0.015        |
|               |                | `torque_steps`                      | 0.060          | 0.004        |

- The held-out load is predicted as well as the loads either side of it, so the model interpolates between loads rather than fitting each one.
- The runs at kp 4.03, twice any fitted gain, have the lowest errors of all. The model extends to stiffer gains than it was fitted at.
- Pure torque control (`torque_steps`) has no position feedback, so small errors accumulate; its torque error is at the noise level.
- The 150 mm setups, with the larger inertias and the loads at and beyond the driving limit, are the hardest (0.05 rad).

### 15.6 Peak driving torque, corrected

The 417 g setup on the 150 mm arm asks the servo to lift 0.60 N·m of gravity at horizontal:

| Run                      | Real servo reached | M6 reached | Gravity torque held (real) | Reported motor torque |
| ------------------------ | ------------------ | ---------- | -------------------------- | --------------------- |
| `up_and_down`, kp 2.01   | 65.3°              | 65.0°      | 0.546 N·m                  | 0.93 N·m              |
| `lift_and_drop`, kp 2.01 | 66.7°              | 65.8°      | 0.552 N·m                  | 0.83 N·m              |
| `up_and_down`, kp 0.97   | 51.9°              | 49.8°      | 0.473 N·m                  | 0.71 N·m              |
| `lift_and_drop`, kp 0.97 | 51.3°              | 50.4°      | 0.469 N·m                  | 0.66 N·m              |

The servo held 0.55 N·m of gravity, set by a known mass and arm, while still below its 1.0 N·m clamp, and the model agrees to within 1 to 2°. At the clamp the model gives 0.615 N·m. The scale tests of section 10 read about 20% low, most likely from the lever length and contact uncertainties noted in section 2. Peak driving torque at the output: about 0.55 to 0.6 N·m.

The back-driven holding torque is not verified: M6 gives 2.1 N·m, M3 and M4 1.5 to 1.6 N·m, Unitree quotes 1.5 N·m (section 14).

### 15.7 Compared with the XL330 model

The same BAM simulator and friction function, for the J288 (M6) and for BAM's XL330 M6 model in two versions: as `microduck_rl` trains with it (pinned bam commit 62bd8ce, 1.75 A current limit) and as upstream BAM has it now (refitted, no current limit in voltage mode). Both XL330s at 7.4 V.

Output torque available while driving a load (N·m):

| Speed (rad/s)          | 0    | 4    | 8    | 10   | 12   | 15   | 20   | 25   | 30   |
| ---------------------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| XL330 as trained       | 0.42 | 0.44 | 0.38 | 0.30 | 0.22 | 0.10 | 0    | 0    | 0    |
| XL330 upstream         | 0.62 | 0.55 | 0.39 | 0.32 | 0.24 | 0.12 | 0    | 0    | 0    |
| J288, 21 V (6S low)    | 0.61 | 0.71 | 0.70 | 0.69 | 0.68 | 0.67 | 0.65 | 0.32 | 0    |
| J288, 25.2 V (6S full) | 0.61 | 0.71 | 0.70 | 0.69 | 0.68 | 0.67 | 0.65 | 0.63 | 0.39 |

|                  | Lift from rest | Hold when back-driven | Free-running speed                   |
| ---------------- | -------------- | --------------------- | ------------------------------------ |
| XL330 as trained | 0.43 N·m       | 0.90 N·m              | about 17 rad/s                       |
| XL330 upstream   | 0.62           | 1.70                  | about 18 rad/s                       |
| J288             | 0.62           | 2.12 (unverified)     | 28 rad/s (21 V) to 34 rad/s (25.2 V) |

A 1 rad move of a leg-sized inertia (2 × 10⁻³ kg·m², no gravity):

|                                          | To 90% | Settled within 5% | Peak speed | Overshoot |
| ---------------------------------------- | ------ | ----------------- | ---------- | --------- |
| XL330 as trained, kp 200 (robot setting) | 216 ms | 372 ms            | 5.5 rad/s  | 5%        |
| XL330 as trained, kp 400                 | 144 ms | 333 ms            | 8.8 rad/s  | 16%       |
| J288, kp 2.01, kd 0.05                   | 80 ms  | 234 ms            | 16.9 rad/s | 21%       |
| J288, kp 4.03, kd 0.05                   | 74 ms  | 292 ms            | 20.7 rad/s | 38%       |

- Held still, the J288 is as strong as the upstream XL330 model and about 45% stronger than the XL330 the policies were trained against.
- In motion it is much stronger: full torque to about 20 rad/s, where the XL330 has lost most of its torque by 10 to 15 rad/s. At 12 rad/s it has about three times the XL330's torque.
- It moves a leg-sized load two to three times faster. Its overshoot at kd 0.05 comes from light, lagged damping and is a tuning choice: more kd, or the trajectory's speed as `w_des`, trades speed for less overshoot.
- The J288's envelope above about 20 rad/s rests on the assumed back-EMF constant (section 15.3).

### 15.8 In the bam fork

| File                                                     | Contents                                                                                                                                       |
| -------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `bam/unitree/actuator.py`                                | `J288Actuator` (registered as `unitree_j288`) and the `J288Pendulum` testbench                                                                 |
| `bam/unitree/digital_servo.py`                           | J288/S288 serial driver and protocol constants                                                                                                 |
| `bam/unitree/calibrate.py`, `record.py`, `all_record.py` | Hanging-angle calibration, one recording, a session for one setup                                                                              |
| `bam/unitree/fit_control.py`                             | Stage 1 control-law fit, and evaluation of position and torque against logs                                                                    |
| `bam/unitree/mjlab.py`                                   | `J288BamActuator` for mjlab: full hybrid command, per-joint gains, mode 0, `reported_torque`; `J288Simulator` for log rollouts at any timestep |
| `bam/params/unitree_j288/m1.json` to `m6.json`           | The fitted models                                                                                                                              |
| `tests/unitree/`                                         | 39 tests, including a simulated servo for the recorder and calibration                                                                         |

### 15.9 At the training timestep

`microduck_rl` trains at a 5 ms physics step with MuJoCo's `implicitfast` integrator (decimation 4, a 50 Hz policy); the model was fitted at 2 ms. Checked on mjlab with `bam.unitree.mjlab.J288Simulator` and M6, 01/10/2026.

| Validation recordings (75 mm, 415 g), mean over 7 | Position error | Torque error |
| --- | --- | --- |
| 2 ms, Euler (as fitted) | 0.0278 rad | 0.0230 N·m |
| 2 ms, implicitfast | 0.0278 rad | 0.0230 N·m |
| 5 ms, implicitfast (as trained) | 0.0277 rad | 0.0228 N·m |

No recording changed by more than 0.002 rad between 2 and 5 ms, including the kp 4.03 runs.

Beyond the recorded gains, simulation alone at 5 ms against 1 ms, kp 1 to 8 with kd 0.02 to 0.08, `sin_time_square` and `up_and_down`:

| Setup | Worst mean position difference | Worst single position difference | Worst torque difference |
| --- | --- | --- | --- |
| 75 mm, 70 g (armature-dominated, the fastest response) | 0.0006 rad | 0.003 rad | 0.005 N·m |
| 150 mm, 417 g | 0.0024 rad | 0.016 rad | 0.009 N·m |

The step's effect is at least ten times smaller than the model's error against the real servo, over the whole kp 0 to 8 design range. The largest differences are at kp 8 with light damping (kd 0.02). Every robot joint carries at least the armature, so none responds faster than the light setup. This covers one joint; contacts and coupled joints on the full robot are mjlab's usual 5 ms behaviour and are not J288-specific.

## 16. A trained policy on the pendulum

Date: 02/10/2026. The first end-to-end test of the RL chain: a policy trained in `microduck_rl` on the J288 M6 model (branch `dukki-j288`, task `Mjlab-Testbench-J288`), exported to ONNX and run on the real servo at 50 Hz from the Mac. The policy tracks target angles on the 150 mm pendulum: observation joint angle from hanging, joint velocity as the position difference over one 20 ms tick, last action and target; action the position target. kp 2.0 N·m/rad, kd 0.05 N·m·s/rad, 6S at 23.5 to 24 V. Between ticks the held command is resent every 5 ms, which logs the replies at 200 Hz and keeps the servo's frame timeout from latching.

The script is `scripts/j288_testbench_sim2real.py` in `microduck_rl`. It runs the same 30 s schedule of eight target angles, held 4 s each, in the training env and on the servo. Logs and plots are in `scratch/j288_sim2real/`.

### 16.1 Training

With only the regularisation of the XL330 testbench task, the policy moved bang-bang: the 1.0 N·m clamp on every target change and about 27 rad/s, past the 18.6 rad/s the identification ever reached. The bench task now penalises joint speed above 15 rad/s and torque more strongly. With a speed weight of 0.05 the peak was still 23 rad/s; at 0.5 it is 17.4 rad/s, inside the identified range, with the clamp reached only briefly at the start of large steps (0.4% of samples). 600 iterations on the Mac's CPU, 512 environments, about 17 minutes. The policy was trained with the 108 g end mass.

### 16.2 Runs

| Run | Rig | Tracking error, real (sim) | Position, sim against real | Torque, sim against real | Winding |
| --- | --- | --- | --- | --- | --- |
| 1 | 108 g, arm screws loose | 3.4° (1.8°) | 0.043 rad | 0.194 N·m | 40 to 50 °C |
| 2 | 108 g, screws tightened | 1.8° (1.8°) | 0.017 rad | 0.063 N·m | 41 to 44 °C |
| 3 | 163 g, end mass bolted on | 2.2° (2.5°) | 0.011 rad | 0.034 N·m | 41 to 44 °C |

Run 3's simulation uses the 163 g rig (end mass 146 g including the bolt, `l150_m163`); the policy is the same.

- Run 1 limit-cycled wherever gravity torque was small (|q| below about 0.7 rad): ±0.1 rad and about ±0.75 N·m at 16.7 Hz, three policy ticks, with the policy's action oscillating with it. The simulation did not, even with 5 to 20 ms of command delay added, and the host loop's timing was steady (5.0 ms mean, 7.3 ms worst step). Large-angle holds, where gravity loads the arm against one side, were steady.
- Run 2 removed most of it. The one hold left oscillating (0.13 rad peak to peak) was the lightest-loaded one, at 0.29 rad approached from above.
- Run 3 holds still everywhere, including that one. The slop was in the arm and end mass, not the servo or the model.
- In run 3 the remaining differences are a little more ringing on the real servo for 0.2 to 0.3 s after each step, and holding torques that differ by up to 0.08 N·m on two holds (0.22 against 0.30 N·m, and 0.06 against 0.02 N·m approached from above). Both are within the model's validation error (section 15.5).

The chain is validated on this rig: actuator model, gains, the position-difference velocity observation, ONNX export and a 50 Hz host loop. Run 1 is also a warning for the robot: a few hundredths of a radian of slop in a lightly loaded joint was enough to make a policy trained without it limit-cycle.

## 17. Gear play

Date: 02/10/2026. The output clamped so it cannot turn either way, a slow torque cycle through zero in torque mode (kp = kd = 0), ±0.15 N·m of motor torque, three cycles per run. The test is `explore.py backlash` (see `scripts/j288/README.md`); logs and plots are `scratch/j288_explore/backlash_2026-10-02_*`. Three runs, re-clamped between the first and the second.

### 17.1 Rotor-derived position

Against reported motor torque, the rotor-derived position traces a repeatable loop. The play is taken up in two steps, at about ±0.025 N·m (M6's motor-side static friction is 0.024 N·m) and about ±0.07 N·m, and beyond about ±0.08 N·m the position is flat: past the play, the gearbox is stiff at these torques.

| Run | Rotor travel between the plateaus |
| --- | --- |
| 1 | 43.6 mrad (2.5°) |
| 2 (re-clamped) | 39.1 mrad (2.2°) |
| 3 | 38.7 mrad (2.2°) |

This is everything between the motor and the clamp: gearbox, output spline, horn, horn screw and fixture.

### 17.2 Split, at zero current

Run 3 switched to mode 0 for 0.3 s at each end of each cycle, after holding the torque. With the output clamped, friction keeps the rotor on its side of the play (it moved under 0.12 mrad in every rest), and with no current the output encoder settles at once, with noise at its 0.77 mrad resolution.

| Cycle | Rotor | Output shaft (encoder) | Inside the servo (rotor minus encoder) |
| --- | --- | --- | --- |
| 0 | 36.7 mrad | 36.0 mrad | 0.7 mrad |
| 1 | 37.3 | 31.4 | 5.8 |
| 2 | 35.3 | 29.1 | 6.1 |

- Play inside the servo: about 6 mrad (0.3°), roughly ±3 mrad, with a few mrad of scatter (cycle 0 reads 0.7). This agrees with the roughly 3 mrad the rotor and encoder sit apart at rest (README, probe).
- The output shaft itself moved about 30 mrad inside the clamp, and its positive end crept 8 mrad between cycles 1 and 2: something between the shaft and the fixture gives, through the spline, horn, horn screw or the clamp. Not yet identified (section 14).
- Against the robot models: their backlash hinges use ±1° (±17 mrad), an XL330 figure. The servo alone is well under that, but a joint's play includes its horn and printed parts, so the hinges should be set from an assembled joint.

### 17.3 Output encoder under current

Not fully confirmed. Under held torque, the output encoder drifts by 10 to 25 mrad with the torque while the rotor-derived position is flat and the shaft is clamped, and it jumps around near zero torque. Read under current, it gave 30 mrad of output-shaft motion in run 1 and 50 mrad in run 2, more than the rotor's own 39 mrad, which is impossible with a rigid shaft. Read at zero current (run 3), it is steady and consistent. The likely cause is the winding's field disturbing the magnetic encoder, which would make "reliable only at rest" mean "reliable without current". Other explanations are not ruled out, such as the shaft really moving in a loose fixture while the rotor's friction holds it. Until the check in section 14 is done, read the output encoder only at zero current.
