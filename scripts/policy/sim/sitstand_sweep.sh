#!/bin/bash
# Sit-stand in simulation against gear play and joint friction (docs/policy-testing.md section 9). Copy to the
# training host and run from there; results in runs/eval/sitstand-sweep/sweep.txt. Which part of the robot-like
# simulation breaks the rise from seated: gear play, joint friction, or both together?
# Sit-stand 7749 (on the robot) and the gear-play fine-tune 9248, kd 0.07, pushes off, head still.
. ~/not_work/dukki/env.sh
cd ~/not_work/dukki/microduck_rl
export DUKKI_KP=1 DUKKI_KD=0.05 DUKKI_CALIBRATION_SPREAD=0 MUJOCO_GL=egl
R=logs/rsl_rl/microduck_sitstand
E=~/not_work/dukki/runs/eval/sitstand-sweep; mkdir -p $E; : > $E/sweep.txt
for ck in $R/2026-11-21_19-12-02_dukki-sitstand-stops-kp1-kd0p05/model_7749.pt $(ls -d $R/*_dukki-sitstand-gearplay-kp1-kdr | tail -1)/model_9248.pt; do
  for cfg in "plain 1,1" "plain 3,5" "play 1,1" "play 2,2" "play 3,3"; do
    set -- $cfg
    task=Mjlab-SitStand-Flat-Dukki; [ $1 = play ] && task=Mjlab-SitStand-Flat-Backlash-Dukki
    echo "== $(basename $ck) $1 friction $2 kd 0.07" >> $E/sweep.txt
    DUKKI_FRICTION=$2 DUKKI_KD_RANGE=0.07,0.07 uv run scripts/dukki_sitstand_eval.py --task $task --checkpoint $ck \
      --envs 256 --no-pushes --head-zero 2>&1 | grep reached | grep -E "stand from seated|sit from standing" | cut -c1-110 >> $E/sweep.txt
  done
done
echo ALLDONE >> $E/sweep.txt
