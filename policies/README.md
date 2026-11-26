# Dukki policies

Every policy exported for the robot, as ONNX, copied from the robot's `/opt/robot/policies/dukki/` (and two that only ever went as far as the training host) on 26/11/2026. Each was exported from a microduck_rl (`dukki-j288`) checkpoint with `scripts/export.py`, which bakes the observation normaliser in; the run is also in each file's `run_path` metadata. The training checkpoints themselves stay on the training host, under `microduck_rl/logs/rsl_rl/` (not in git).

To install one, copy it to `/opt/robot/policies/dukki/` on the robot and point a slot at it in `/var/lib/robot/robotd.toml` (`scripts/policy/robotd.toml` is the copy in this repository). All take the same 61 inputs and give 14 actions. The policy log is `docs/policy-testing.md`.

## On the robot now

| File | Slot | Run (`logs/rsl_rl/...`) | Notes |
|---|---|---|---|
| `dukki_walk_mpush2_12750.onnx` | `walk`, also standing still | `velocity/2026-11-25_20-36-09_dukki-walk-mpush2-kp1-kdr` | msoft2 11750 fine-tuned with bigger pushes; legs filter 0.84 (policy-testing.md sections 2 and 13) |
| `dukki_standup_gentle_10250.onnx` | `stand`, getting up only | `microduck_stand/2026-11-25_16-21-13_dukki-standup-gentle-kp1-kdr` | stand-up 9250 fine-tuned on soft ground with a gentler get-up (section 13) |
| `dukki_sitstand_gearplay_9248.onnx` | `sitstand` | `microduck_sitstand/2026-11-22_18-31-58_dukki-sitstand-gearplay-kp1-kdr` | section 9 |
| `dukki_jump_gearplay_11000.onnx` | skill `jump`, on X | `dukki_jump/2026-11-25_17-18-43_dukki-jump-gearplay-kp1-kdr` | the jump on the gear-play recipe |

## Earlier, kept for comparison

| File | Run (`logs/rsl_rl/...`) | Notes |
|---|---|---|
| `dukki_walk_8500.onnx` | `velocity/2026-11-09_09-32-29_dukki-walk-headroll15-kp1-kd0p05` | First walk on the robot; ran away unfiltered (section 4) |
| `dukki_walk_friction_9250.onnx` | `velocity/2026-11-21_20-12-24_dukki-walk-friction-kp1-kd0p05` | Joint friction 1 to 3 times; the robot's walk 21 to 22/11 (section 4) |
| `dukki_walk_robust_9500.onnx` | `velocity/2026-11-21_21-58-34_dukki-walk-robust-kp1-kd0p05` | Encoder offsets and kp spread; fell backwards under pushes (section 4) |
| `dukki_walk_lownoise_10249.onnx` | `velocity/2026-11-21_23-21-31_dukki-walk-lownoise-kp1-kd0p05` | Low action noise; fell backwards under pushes (section 4) |
| `dukki_walk_lownoise_nospread_10000.onnx` | `velocity/2026-11-22_00-06-02_dukki-walk-lownoise-nospread-kp1-kd0p05` | Calmest quiet stand; fell backwards under pushes (section 4) |
| `dukki_walk_slowwalk2_10750.onnx` | `velocity/2026-11-22_11-50-36_dukki-walk-slowwalk2-kp1-kd0p05` | Friction 1 to 5 times, slow walking; twitchy (section 7) |
| `dukki_walk_gearplay_10000.onnx` | `velocity/2026-11-22_16-16-34_dukki-walk-gearplay-kp1-kdr` | Gear play; the robot's walk 22 to 25/11 (section 8) |
| `dukki_walk_msoft2_11750.onnx` | `velocity/2026-11-25_12-34-01_dukki-walk-msoft2-kp1-kdr` | Soft ground, walking paid more than standing; the start of mpush2 |
| `dukki_walk_mterrain2_12500.onnx` | `velocity/2026-11-25_13-38-09_dukki-walk-mterrain2-kp1-kdr` | msoft2 on Dukki's terrain |
| `dukki_walk_mterrain4_12750.onnx` | `velocity/2026-11-25_18-18-00_dukki-walk-mterrain4-kp1-kdr` | msoft2 on Dukki's terrain, turning on the spot on the flat only; never on the robot |
| `dukki_walk_mpush_13249.onnx` | `velocity/2026-11-25_19-29-02_dukki-walk-mpush-kp1-kdr` | msoft2 with bigger pushes, the first try at mpush2; never on the robot |
| `dukki_walk_standgain3_13500.onnx` | `velocity/2026-11-26_17-06-24_dukki-walk-standgain3-kp1-kdr` | mpush2 with a penalty on its standing gain; two falls on carpet (section 13.4) |
| `dukki_standup_gearplay_7499.onnx` | `microduck_stand/2026-11-23_10-09-23_dukki-standup-gearplay-kp1-kdr` | Stand-up with gear play (section 11.1) |
| `dukki_standup_nobc_9250.onnx` | `microduck_stand/2026-11-23_16-32-59_dukki-standup-nobc-kp1-kdr` | Whole-body contact, no expert; the stand slot 23 to 25/11 (section 11.4) |
| `dukki_standup_soft_10250.onnx` | `microduck_stand/2026-11-25_15-25-38_dukki-standup-soft-kp1-kdr` | 9250 fine-tuned on soft ground, before gentle |
| `dukki_sitstand_stops_7749.onnx` | `microduck_sitstand/2026-11-21_19-12-02_dukki-sitstand-stops-kp1-kd0p05` | Sat well, fell backwards rising (section 9) |
| `dukki_sitstand_softrise_10500.onnx` | `microduck_sitstand/2026-11-22_21-41-36_dukki-sitstand-softrise-kp1-kdr` | A try at a softer rise from the seat |
