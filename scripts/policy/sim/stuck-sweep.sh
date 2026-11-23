#!/usr/bin/env bash
# stuck-sweep.sh <checkpoint>: the robot's stuck face-up pose (runs/stuck_faceup_{1,2}.json) in simulation,
# robot-like (kd 0.07, legs 0.7, no spread), backlash model; and the plain face-up start's failures. SWEEP_OUT names
# the result file. Runs on the training host from ~/not_work/dukki/runs/, where this folder's standup_from_robot.py
# and stuck_faceup_*.json (the robot's stuck pose, 23/11/2026) are copied.
. ~/not_work/dukki/env.sh
cd ~/not_work/dukki/microduck_rl
CK=$1; E=~/not_work/dukki/runs/eval/standup-stuck; mkdir -p $E; OUT=$E/${SWEEP_OUT:-sweep.txt}; : > $OUT
export DUKKI_KP=1 DUKKI_KD=0.05 DUKKI_KD_RANGE=0.07,0.07 DUKKI_CALIBRATION_SPREAD=0 DUKKI_LEGS_LOWPASS=0.7 MUJOCO_GL=egl
R=../runs/standup_from_robot.py
run() { echo "== $*" >> $OUT; uv run python $R --checkpoint $CK "$@" 2>&1 | grep -E "^   |Error|error" >> $OUT; }
DUKKI_FRICTION=1,1 run --pose ../runs/stuck_faceup_1.json --pin --render $E/pinned1
DUKKI_FRICTION=1,1 run --pose ../runs/stuck_faceup_1.json --render $E/free1
DUKKI_FRICTION=3,3 run --pose ../runs/stuck_faceup_1.json --pin
DUKKI_FRICTION=3,3 run --pose ../runs/stuck_faceup_1.json
DUKKI_FRICTION=1,1 run --pose ../runs/stuck_faceup_2.json --pin
DUKKI_FRICTION=1,1 run --pose ../runs/stuck_faceup_1.json --failures --render $E/faceup_start
DUKKI_FRICTION=1,4 run --pose ../runs/stuck_faceup_1.json --failures
touch $E/done
